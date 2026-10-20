#include "modules/common/util/perf_trace_writer.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <new>
#include <sstream>
#include <string_view>

#include <sys/prctl.h>
#include <sys/resource.h>

namespace apollo {
namespace common {
namespace util {

enum class TraceRecordKind : uint8_t {
  LEGACY,
  EVENT,
  EDGE,
  BOUNDARY,
  FUSION_INPUT,
  TRACE_ANCHOR,
  CONTROL_TIMER_DIAGNOSTIC,
  PREDICTION_WORKER_EVENT,
};

// The producer-side record is deliberately fixed-size and trivially copyable:
// no allocation, formatting, file access, or borrowed string lifetime is left
// on Apollo application threads.
struct QueuedTraceRecord {
  TraceRecordKind kind = TraceRecordKind::EVENT;

  uint64_t run_generation = 0;
  uint64_t event_id = 0;
  uint64_t trace_id = 0;
  uint64_t parent_trace_id = 0;
  uint64_t primary_parent_trace_id = 0;
  uint64_t mono_ns = 0;
  uint64_t measurement_time_ns = 0;
  uint64_t source_ts_ns = 0;
  uint64_t lidar_ts_ns = 0;
  uint64_t camera_ts_ns = 0;
  uint64_t radar_ts_ns = 0;
  uint64_t data_ts_ns = 0;
  uint64_t fusion_trace_id = 0;
  uint64_t frame_time_ns = 0;
  uint64_t sensor_ts_ns = 0;
  uint64_t preproc_enter_ns = 0;

  uint64_t frame_instance_id = 0;
  uint64_t task_id = 0;
  int64_t group_id = -1;
  int64_t obstacle_id = -1;
  uint64_t model_call_id = 0;
  uint64_t inference_id = 0;
  int64_t lane_sequence_index = -1;
  int64_t pool_level = -1;
  int64_t input_rows = -1;
  int64_t input_cols = -1;
  char status[32] = {};
  char model_type[64] = {};
  char device_type[32] = {};

  double header_ts_sec = 0.0;
  double sensor_time_sec = 0.0;
  double ingress_ms = 0.0;

  uint32_t proc_id = 0;
  uint32_t seq = 0;
  uint32_t upstream_seq = 0;
  uint32_t pid = 0;
  uint32_t tid = 0;
  uint32_t input_seq = 0;
  uint32_t output_seq = 0;
  uint32_t object_count = 0;
  uint32_t parent_trace_count = 0;

  bool trace_valid = false;
  bool is_main_sensor = false;

  char module[96] = {};
  char phase[64] = {};
  char edge[8] = {};
  char channel[256] = {};
  char source_kind[16] = {};
  char sensor[96] = {};
  char sensor_kind[32] = {};
  char sensor_id[128] = {};
};

// The normal path is a bounded MPMC ring based on per-slot sequence numbers.
// If the background writer is temporarily slower than producers, records spill
// into a chunked std::deque instead of being dropped or waiting for disk I/O.
// The overflow path may allocate memory, but it is only reached after the
// preallocated primary ring fills.
class AsyncTraceQueue {
 public:
  explicit AsyncTraceQueue(uint64_t capacity)
      : capacity_(capacity), mask_(capacity - 1), cells_(new Cell[capacity]) {
    for (uint64_t i = 0; i < capacity_; ++i) {
      cells_[i].sequence.store(i, std::memory_order_relaxed);
    }
  }

  bool EnqueueLossless(const QueuedTraceRecord& record) {
    if (TryEnqueuePrimary(record)) {
      UpdateMaxPending();
      return true;
    }

    try {
      {
        std::lock_guard<std::mutex> lock(overflow_mutex_);
        overflow_queue_.push_back(record);
        overflow_pending_.fetch_add(1, std::memory_order_release);
      }
      overflow_records_.fetch_add(1, std::memory_order_relaxed);
      UpdateMaxPending();
      return true;
    } catch (const std::bad_alloc&) {
      allocation_failures_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
  }

  bool TryDequeue(QueuedTraceRecord* record) {
    // Once overflow exists, drain it first so a sustained producer load cannot
    // starve records that spilled while the primary ring was full.
    if (overflow_pending_.load(std::memory_order_acquire) != 0 &&
        TryDequeueOverflow(record)) {
      return true;
    }
    if (TryDequeuePrimary(record)) {
      return true;
    }
    return TryDequeueOverflow(record);
  }

  uint64_t ApproxSize() const {
    const uint64_t tail = enqueue_pos_.load(std::memory_order_relaxed);
    const uint64_t head = dequeue_pos_.load(std::memory_order_relaxed);
    const uint64_t primary = tail >= head ? tail - head : 0;
    return primary + overflow_pending_.load(std::memory_order_relaxed);
  }

  uint64_t Capacity() const { return capacity_; }

  uint64_t OverflowRecords() const {
    return overflow_records_.load(std::memory_order_relaxed);
  }

  uint64_t AllocationFailures() const {
    return allocation_failures_.load(std::memory_order_relaxed);
  }

  uint64_t MaxPendingRecords() const {
    return max_pending_records_.load(std::memory_order_relaxed);
  }

  void ResetRunCounters() {
    overflow_records_.store(0, std::memory_order_relaxed);
    allocation_failures_.store(0, std::memory_order_relaxed);
    max_pending_records_.store(0, std::memory_order_relaxed);
  }

 private:
  bool TryEnqueuePrimary(const QueuedTraceRecord& record) {
    uint64_t pos = enqueue_pos_.load(std::memory_order_relaxed);
    for (;;) {
      Cell& cell = cells_[pos & mask_];
      const uint64_t seq = cell.sequence.load(std::memory_order_acquire);
      const int64_t diff = static_cast<int64_t>(seq) -
                           static_cast<int64_t>(pos);
      if (diff == 0) {
        if (enqueue_pos_.compare_exchange_weak(
                pos, pos + 1, std::memory_order_relaxed,
                std::memory_order_relaxed)) {
          cell.record = record;
          cell.sequence.store(pos + 1, std::memory_order_release);
          return true;
        }
      } else if (diff < 0) {
        return false;
      } else {
        pos = enqueue_pos_.load(std::memory_order_relaxed);
      }
    }
  }

  bool TryDequeuePrimary(QueuedTraceRecord* record) {
    uint64_t pos = dequeue_pos_.load(std::memory_order_relaxed);
    for (;;) {
      Cell& cell = cells_[pos & mask_];
      const uint64_t seq = cell.sequence.load(std::memory_order_acquire);
      const int64_t diff = static_cast<int64_t>(seq) -
                           static_cast<int64_t>(pos + 1);
      if (diff == 0) {
        if (dequeue_pos_.compare_exchange_weak(
                pos, pos + 1, std::memory_order_relaxed,
                std::memory_order_relaxed)) {
          *record = cell.record;
          cell.sequence.store(pos + capacity_, std::memory_order_release);
          return true;
        }
      } else if (diff < 0) {
        return false;
      } else {
        pos = dequeue_pos_.load(std::memory_order_relaxed);
      }
    }
  }

  bool TryDequeueOverflow(QueuedTraceRecord* record) {
    if (overflow_pending_.load(std::memory_order_acquire) == 0) {
      return false;
    }
    std::lock_guard<std::mutex> lock(overflow_mutex_);
    if (overflow_queue_.empty()) {
      return false;
    }
    *record = overflow_queue_.front();
    overflow_queue_.pop_front();
    overflow_pending_.fetch_sub(1, std::memory_order_release);
    return true;
  }

  void UpdateMaxPending() {
    const uint64_t pending = ApproxSize();
    uint64_t observed = max_pending_records_.load(std::memory_order_relaxed);
    while (pending > observed &&
           !max_pending_records_.compare_exchange_weak(
               observed, pending, std::memory_order_relaxed,
               std::memory_order_relaxed)) {
    }
  }

  struct Cell {
    std::atomic<uint64_t> sequence{0};
    QueuedTraceRecord record;
  };

  const uint64_t capacity_;
  const uint64_t mask_;
  std::unique_ptr<Cell[]> cells_;
  alignas(64) std::atomic<uint64_t> enqueue_pos_{0};
  alignas(64) std::atomic<uint64_t> dequeue_pos_{0};
  std::mutex overflow_mutex_;
  std::deque<QueuedTraceRecord> overflow_queue_;
  std::atomic<uint64_t> overflow_pending_{0};
  std::atomic<uint64_t> overflow_records_{0};
  std::atomic<uint64_t> allocation_failures_{0};
  std::atomic<uint64_t> max_pending_records_{0};
};

std::string FormatSensorTimeSecCsv(double sensor_time_sec) {
  std::ostringstream oss;
  oss << std::setprecision(17) << sensor_time_sec;
  return oss.str();
}

namespace {

constexpr uint64_t kAsyncQueueCapacity = 4096;
constexpr uint64_t kDrainBatchSize = 4096;
constexpr uint64_t kEnqueueSampleStride = 1024;
constexpr int kActiveIdleSleepUs = 1000;
constexpr int kInactiveSleepMs = 10;
constexpr int kWriterNiceValue = 10;

uint64_t ThreadCpuNowNs() {
  struct timespec ts;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) {
    return 0;
  }
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

void UpdateAtomicMax(std::atomic<uint64_t>* target, uint64_t value) {
  uint64_t observed = target->load(std::memory_order_relaxed);
  while (value > observed &&
         !target->compare_exchange_weak(observed, value,
                                        std::memory_order_relaxed,
                                        std::memory_order_relaxed)) {
  }
}

bool CopyFixedText(const char* src, char* dst, size_t capacity) {
  if (capacity == 0) {
    return src != nullptr && *src != '\0';
  }
  if (src == nullptr) {
    dst[0] = '\0';
    return false;
  }

  size_t n = 0;
  while (n + 1 < capacity && src[n] != '\0') {
    dst[n] = src[n];
    ++n;
  }
  dst[n] = '\0';
  return src[n] != '\0';
}

bool CopyFixedText(const std::string& src, char* dst, size_t capacity) {
  return CopyFixedText(src.c_str(), dst, capacity);
}

std::string CsvEscape(const char* s) {
  if (s == nullptr || *s == '\0') {
    return "";
  }
  std::string_view sv(s);
  if (sv.find_first_of(",\"\r\n") == std::string_view::npos) {
    return std::string(sv);
  }
  std::string out;
  out.reserve(sv.size() + 2);
  out.push_back('"');
  for (char c : sv) {
    if (c == '"') {
      out += "\"\"";
    } else {
      out.push_back(c);
    }
  }
  out.push_back('"');
  return out;
}

std::string CsvEscape(const std::string& s) { return CsvEscape(s.c_str()); }

}  // namespace

PerfTraceWriter& PerfTraceWriter::Instance() {
  static PerfTraceWriter inst;
  return inst;
}

PerfTraceWriter::PerfTraceWriter()
    : async_queue_(new AsyncTraceQueue(kAsyncQueueCapacity)) {
  io_thread_ = std::thread(&PerfTraceWriter::PollLoop, this);
}

PerfTraceWriter::~PerfTraceWriter() {
  running_.store(false, std::memory_order_relaxed);
  if (io_thread_.joinable()) io_thread_.join();
}

void PerfTraceWriter::PollLoop() {
  (void)prctl(PR_SET_NAME, "perf_trace_io", 0, 0, 0);
  // Keep trace serialization below application work in normal CFS scheduling.
  (void)setpriority(PRIO_PROCESS, static_cast<id_t>(GetTid()),
                    kWriterNiceValue);

  auto next_trigger_poll = std::chrono::steady_clock::now();
  auto next_flush = next_trigger_poll + std::chrono::seconds(1);

  while (running_.load(std::memory_order_relaxed)) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_trigger_poll) {
      next_trigger_poll = now + std::chrono::milliseconds(POLL_INTERVAL_MS);

      std::string run_id;
      if (access(TRIGGER_FILE, F_OK) == 0) {
        std::ifstream f(TRIGGER_FILE);
        if (f.is_open()) {
          std::getline(f, run_id);
        }
      }

      if (run_id.empty()) {
        if (!current_run_id_.empty()) {
          StopRun();
        }
      } else if (run_id != current_run_id_) {
        StopRun();
        StartRun(run_id);
      }
    }

    const bool drained = DrainQueuedRecords(kDrainBatchSize);
    if (now >= next_flush) {
      if (!FlushAllFiles()) {
        RecordIoError();
      }
      next_flush = now + std::chrono::seconds(1);
    }

    if (!drained) {
      if (active_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(
            std::chrono::microseconds(kActiveIdleSleepUs));
      } else {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kInactiveSleepMs));
      }
    }
  }

  StopRun();
}

void PerfTraceWriter::StartRun(const std::string& run_id) {
  const std::string run_dir =
      std::string(kScbDataRoot) + "/" + run_id + "/trace/";
  std::error_code ec;
  std::filesystem::create_directories(run_dir, ec);
  if (ec) {
    std::cerr << "[perf_trace] 创建目录失败: " << run_dir << " : "
              << ec.message() << std::endl;
    return;
  }

  current_run_id_ = run_id;
  current_run_dir_ = run_dir;
  written_records_ = 0;
  io_errors_ = 0;
  run_start_mono_ns_ = MonoNowNs();
  writer_cpu_start_ns_ = ThreadCpuNowNs();
  truncated_records_.store(0, std::memory_order_relaxed);
  enqueue_sample_count_.store(0, std::memory_order_relaxed);
  enqueue_sample_total_ns_.store(0, std::memory_order_relaxed);
  enqueue_sample_max_ns_.store(0, std::memory_order_relaxed);
  async_queue_->ResetRunCounters();
  run_generation_.fetch_add(1, std::memory_order_release);
  active_.store(true, std::memory_order_release);
}

void PerfTraceWriter::StopRun() {
  active_.store(false, std::memory_order_release);
  while (producers_in_flight_.load(std::memory_order_acquire) != 0) {
    std::this_thread::yield();
  }
  while (DrainQueuedRecords(kDrainBatchSize)) {
  }

  if (!current_run_id_.empty()) {
    if (!FlushAllFiles()) {
      RecordIoError();
    }
    WriteHealthSummary();
  }
  CloseAllFiles();
  current_run_id_.clear();
  current_run_dir_.clear();
}

bool PerfTraceWriter::EnqueueRecord(const QueuedTraceRecord& record) {
  if (!active_.load(std::memory_order_acquire) ||
      record.run_generation !=
          run_generation_.load(std::memory_order_acquire)) {
    return false;
  }

  producers_in_flight_.fetch_add(1, std::memory_order_acq_rel);
  if (!active_.load(std::memory_order_acquire) ||
      record.run_generation !=
          run_generation_.load(std::memory_order_acquire)) {
    producers_in_flight_.fetch_sub(1, std::memory_order_release);
    return false;
  }

  static thread_local uint64_t enqueue_sample_sequence = 0;
  const bool sample =
      (++enqueue_sample_sequence % kEnqueueSampleStride) == 0;
  const uint64_t enqueue_start_ns = sample ? MonoNowNs() : 0;
  const bool enqueued = async_queue_->EnqueueLossless(record);
  if (sample) {
    const uint64_t elapsed_ns = MonoNowNs() - enqueue_start_ns;
    enqueue_sample_count_.fetch_add(1, std::memory_order_relaxed);
    enqueue_sample_total_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed);
    UpdateAtomicMax(&enqueue_sample_max_ns_, elapsed_ns);
  }
  if (!enqueued) {
    // Memory allocation failure is a capture failure, not permission to lose
    // individual records silently.  Stop accepting the run; trace_health will
    // mark it invalid for baseline analysis.
    active_.store(false, std::memory_order_release);
  }
  producers_in_flight_.fetch_sub(1, std::memory_order_release);
  return enqueued;
}

bool PerfTraceWriter::CaptureRunGeneration(uint64_t* generation) const {
  const uint64_t before = run_generation_.load(std::memory_order_acquire);
  if (!active_.load(std::memory_order_acquire)) {
    return false;
  }
  const uint64_t after = run_generation_.load(std::memory_order_acquire);
  if (before != after) {
    return false;
  }
  *generation = after;
  return true;
}

bool PerfTraceWriter::DrainQueuedRecords(uint64_t max_records) {
  bool drained = false;
  QueuedTraceRecord record;
  for (uint64_t i = 0; i < max_records; ++i) {
    if (!async_queue_->TryDequeue(&record)) {
      break;
    }
    if (WriteQueuedRecord(record)) {
      ++written_records_;
    } else {
      RecordIoError();
    }
    drained = true;
  }
  return drained;
}

void PerfTraceWriter::CloseAllFiles() {
  files_.clear();
}

bool PerfTraceWriter::FlushAllFiles() {
  bool ok = true;
  for (auto& item : files_) {
    if (item.second && item.second->is_open()) {
      item.second->flush();
      ok = item.second->good() && ok;
    }
  }
  return ok;
}

void PerfTraceWriter::RecordIoError() {
  ++io_errors_;
  // A stream failure makes completeness unknowable. Stop accepting new
  // records and let the health file invalidate the run instead of silently
  // continuing with partial data.
  active_.store(false, std::memory_order_release);
}

void PerfTraceWriter::WriteHealthSummary() {
  const std::string health_dir = current_run_dir_ + "trace_health/";
  std::error_code ec;
  std::filesystem::create_directories(health_dir, ec);
  if (ec) {
    std::cerr << "[perf_trace] 创建健康统计目录失败: " << health_dir << " : "
              << ec.message() << std::endl;
    return;
  }

  const std::string path =
      health_dir + "perf_trace_writer." + std::to_string(getpid()) + ".csv";
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    std::cerr << "[perf_trace] 打开健康统计失败: " << path << std::endl;
    return;
  }
  out << "run_id,schema_version,pid,primary_queue_capacity,written_records,"
         "overflow_records,allocation_failures,truncated_records,"
         "max_pending_records,io_errors,run_wall_ns,writer_thread_cpu_ns,"
         "enqueue_sample_stride,enqueue_sample_count,"
         "enqueue_sample_total_ns,enqueue_sample_max_ns\n";
  const uint64_t now_mono_ns = MonoNowNs();
  const uint64_t now_cpu_ns = ThreadCpuNowNs();
  const uint64_t run_wall_ns =
      now_mono_ns >= run_start_mono_ns_ ? now_mono_ns - run_start_mono_ns_ : 0;
  const uint64_t writer_cpu_ns = now_cpu_ns >= writer_cpu_start_ns_
                                     ? now_cpu_ns - writer_cpu_start_ns_
                                     : 0;
  out << current_run_id_ << ",async_writer_v3," << getpid() << ','
      << async_queue_->Capacity() << ',' << written_records_ << ','
      << async_queue_->OverflowRecords() << ','
      << async_queue_->AllocationFailures() << ','
      << truncated_records_.load(std::memory_order_relaxed) << ','
      << async_queue_->MaxPendingRecords() << ',' << io_errors_ << ','
      << run_wall_ns << ',' << writer_cpu_ns << ','
      << kEnqueueSampleStride << ','
      << enqueue_sample_count_.load(std::memory_order_relaxed) << ','
      << enqueue_sample_total_ns_.load(std::memory_order_relaxed) << ','
      << enqueue_sample_max_ns_.load(std::memory_order_relaxed) << '\n';
}

std::string PerfTraceWriter::MakeTableRelativePathLocked(
    TraceTable table, const std::string& module) const {
  const std::string pid = std::to_string(static_cast<uint32_t>(getpid()));
  switch (table) {
    case TraceTable::EVENTS:
      return std::string("events/") + module + "." + pid + ".csv";
    case TraceTable::MESSAGE_CONTEXT:
      return std::string("message_context/") + module + "." + pid + ".csv";
    case TraceTable::FUSION_INPUTS:
      (void)module;
      return std::string("fusion_inputs/perception.multi_sensor_fusion.") + pid +
             ".csv";
    case TraceTable::TRACE_ANCHOR:
      (void)module;
      return std::string("trace_anchor/perception.") + pid + ".csv";
    case TraceTable::PREDICTION_WORKER_EVENTS:
      return std::string("prediction_worker_events/prediction.") + pid + ".csv";
    case TraceTable::CONTROL_TIMER_DIAGNOSTICS:
      (void)module;
      return std::string("control_timer_diagnostics/control.") + pid + ".csv";
    case TraceTable::LEGACY_WIDE:
    default:
      return module + ".csv";
  }
}

std::ofstream* PerfTraceWriter::GetLegacyStreamLocked(const std::string& module) {
  const std::string key = std::string("L:") + module;
  auto it = files_.find(key);
  if (it != files_.end()) return it->second.get();

  const std::string path = current_run_dir_ + module + ".csv";
  auto ofs = std::make_unique<std::ofstream>(
      path, std::ios::out | std::ios::app);

  if (!ofs->is_open()) {
    std::cerr << "[perf_trace] 打开文件失败: " << path << std::endl;
    return nullptr;
  }

  std::error_code file_size_ec;
  const uintmax_t file_size = std::filesystem::file_size(path, file_size_ec);
  if (file_size_ec) {
    std::cerr << "[perf_trace] 查询文件大小失败: " << path << " : "
              << file_size_ec.message() << std::endl;
    return nullptr;
  }
  if (file_size == 0) {
    *ofs << "run_id,module,phase,mono_ns,header_ts_sec,measurement_time_ns,"
         << "source_ts_ns,source_kind,lidar_ts_ns,camera_ts_ns,radar_ts_ns,"
         << "seq,upstream_seq,pid,tid,channel,"
         << "trace_id,parent_trace_id\n";
    ofs->flush();
    if (!ofs->good()) {
      std::cerr << "[perf_trace] 写入表头失败: " << path << std::endl;
      return nullptr;
    }
  }

  auto* ptr = ofs.get();
  files_.emplace(key, std::move(ofs));
  return ptr;
}

std::ofstream* PerfTraceWriter::GetTableStreamLocked(TraceTable table,
                                                       const std::string& module) {
  const std::string rel = MakeTableRelativePathLocked(table, module);
  const std::string key =
      std::string("T:") + std::to_string(static_cast<int>(table)) + ":" + rel;
  auto it = files_.find(key);
  if (it != files_.end()) return it->second.get();

  const std::string full = current_run_dir_ + rel;
  std::error_code ec;
  std::filesystem::create_directories(
      std::filesystem::path(full).parent_path(), ec);
  if (ec) {
    std::cerr << "[perf_trace] 创建目录失败: " << full << " : "
              << ec.message() << std::endl;
    return nullptr;
  }

  auto ofs = std::make_unique<std::ofstream>(
      full, std::ios::out | std::ios::app);

  if (!ofs->is_open()) {
    std::cerr << "[perf_trace] 打开文件失败: " << full << std::endl;
    return nullptr;
  }

  std::error_code file_size_ec;
  const uintmax_t file_size = std::filesystem::file_size(full, file_size_ec);
  if (file_size_ec) {
    std::cerr << "[perf_trace] 查询文件大小失败: " << full << " : "
              << file_size_ec.message() << std::endl;
    return nullptr;
  }
  if (file_size == 0) {
    switch (table) {
      case TraceTable::EVENTS:
        *ofs << "run_id,schema_version,event_id,trace_id,proc_id,module,phase,"
                "mono_ns,pid,tid\n";
        break;
      case TraceTable::MESSAGE_CONTEXT:
        *ofs << "run_id,schema_version,event_id,trace_id,proc_id,module,edge,"
                "mono_ns,trace_valid,channel,input_seq,output_seq,data_ts_ns,"
                "object_count,primary_parent_trace_id,parent_trace_count,"
                "pid,tid\n";
        break;
      case TraceTable::FUSION_INPUTS:
        *ofs << "run_id,schema_version,fusion_trace_id,parent_trace_id,sensor,"
                "sensor_kind,frame_time_ns,sensor_ts_ns,object_count,"
                "is_main_sensor\n";
        break;
      case TraceTable::TRACE_ANCHOR:
        *ofs << "run_id,schema_version,trace_id,sensor_kind,sensor_id,"
                "data_ts_ns,sensor_time_sec,preproc_enter_ns,ingress_ms\n";
        break;
      case TraceTable::PREDICTION_WORKER_EVENTS:
        *ofs << "run_id,schema_version,event_id,mono_ns,pid,tid,trace_id,"
                "frame_instance_id,task_id,group_id,obstacle_id,model_call_id,"
                "inference_id,lane_sequence_index,pool_level,phase,status,"
                "model_type,device_type,input_rows,input_cols\n";
        break;
      case TraceTable::CONTROL_TIMER_DIAGNOSTICS:
        *ofs << "run_id,schema_version,event_id,trace_id,proc_id,phase,"
                "start_mono_ns,end_mono_ns,wall_ns,thread_cpu_ns,"
                "mutex_lock_wall_ns,cv_notify_ns,voluntary_context_switches,"
                "involuntary_context_switches,cpu_start,cpu_end,"
                "overrun_ns,rate_rebased,pid,tid\n";
        break;
      default:
        break;
    }
    ofs->flush();
    if (!ofs->good()) {
      std::cerr << "[perf_trace] 写入表头失败: " << full << std::endl;
      return nullptr;
    }
  }

  auto* ptr = ofs.get();
  files_.emplace(key, std::move(ofs));
  return ptr;
}

uint64_t PerfTraceWriter::CaptureSessionToken() const {
  uint64_t generation = 0;
  return CaptureRunGeneration(&generation) ? generation : 0;
}

bool PerfTraceWriter::WritePredictionWorkerEvent(
    const PredictionWorkerEvent& row, uint64_t expected_token) {
  if (expected_token == 0 || CaptureSessionToken() != expected_token) {
    return false;
  }
  QueuedTraceRecord record;
  record.run_generation = expected_token;
  record.kind = TraceRecordKind::PREDICTION_WORKER_EVENT;
  record.event_id = NextEventId();
  record.mono_ns = MonoNowNs();
  record.pid = static_cast<uint32_t>(getpid());
  record.tid = GetTid();
  record.trace_id = row.trace_id;
  record.frame_instance_id = row.frame_instance_id;
  record.task_id = row.task_id;
  record.group_id = row.group_id;
  record.obstacle_id = row.obstacle_id;
  record.model_call_id = row.model_call_id;
  record.inference_id = row.inference_id;
  record.lane_sequence_index = row.lane_sequence_index;
  record.pool_level = row.pool_level;
  record.input_rows = row.input_rows;
  record.input_cols = row.input_cols;
  bool truncated = false;
  truncated |= CopyFixedText(row.phase, record.phase, sizeof(record.phase));
  truncated |= CopyFixedText(row.status, record.status, sizeof(record.status));
  truncated |= CopyFixedText(row.model_type, record.model_type,
                             sizeof(record.model_type));
  truncated |= CopyFixedText(row.device_type, record.device_type,
                             sizeof(record.device_type));
  const bool enqueued = EnqueueRecord(record);
  if (truncated && enqueued) {
    truncated_records_.fetch_add(1, std::memory_order_relaxed);
  }
  return enqueued;
}

void PerfTraceWriter::Write(const PerfTraceRow& row) {
  uint64_t generation = 0;
  if (!CaptureRunGeneration(&generation)) return;
  QueuedTraceRecord record;
  record.run_generation = generation;
  record.kind = TraceRecordKind::LEGACY;
  bool truncated = false;
  truncated |= CopyFixedText(row.module, record.module, sizeof(record.module));
  truncated |= CopyFixedText(row.phase, record.phase, sizeof(record.phase));
  truncated |= CopyFixedText(row.source_kind, record.source_kind,
                             sizeof(record.source_kind));
  truncated |= CopyFixedText(row.channel, record.channel,
                             sizeof(record.channel));
  record.mono_ns = row.mono_ns;
  record.header_ts_sec = row.header_ts_sec;
  record.measurement_time_ns = row.measurement_time_ns;
  record.source_ts_ns = row.source_ts_ns;
  record.lidar_ts_ns = row.lidar_ts_ns;
  record.camera_ts_ns = row.camera_ts_ns;
  record.radar_ts_ns = row.radar_ts_ns;
  record.seq = row.seq;
  record.upstream_seq = row.upstream_seq;
  record.pid = row.pid;
  record.tid = row.tid;
  record.trace_id = row.trace_id;
  record.parent_trace_id = row.parent_trace_id;
  const bool enqueued = EnqueueRecord(record);
  if (truncated && enqueued) {
    truncated_records_.fetch_add(1, std::memory_order_relaxed);
  }
}

bool PerfTraceWriter::WriteEventRowLocked(const EventRow& row) {
  const std::string mod = row.module ? row.module : "";
  auto* ofs = GetTableStreamLocked(TraceTable::EVENTS, mod);
  if (!ofs) return false;

  *ofs << current_run_id_ << ','
       << kE2eTraceSchemaVersion << ','
       << row.event_id << ','
       << row.trace_id << ','
       << row.proc_id << ','
       << CsvEscape(row.module) << ','
       << CsvEscape(row.phase) << ','
       << row.mono_ns << ','
       << row.pid << ','
       << row.tid << '\n';
  return ofs->good();
}

bool PerfTraceWriter::WriteEdgeRowLocked(const EdgeRow& row) {
  const std::string mod = row.module ? row.module : "";
  auto* ofs = GetTableStreamLocked(TraceTable::MESSAGE_CONTEXT, mod);
  if (!ofs) return false;

  *ofs << current_run_id_ << ','
       << kE2eTraceSchemaVersion << ','
       << row.event_id << ','
       << row.trace_id << ','
       << row.proc_id << ','
       << CsvEscape(row.module) << ','
       << row.edge << ','
       << row.mono_ns << ','
       << (row.trace_valid ? 1 : 0) << ','
       << CsvEscape(row.channel) << ','
       << row.input_seq << ','
       << row.output_seq << ','
       << row.data_ts_ns << ','
       << row.object_count << ','
       << row.primary_parent_trace_id << ','
       << row.parent_trace_count << ','
       << row.pid << ','
       << row.tid << '\n';
  return ofs->good();
}

uint64_t PerfTraceWriter::WriteEvent(const char* module, const char* phase,
                                     uint64_t trace_id, uint32_t proc_id) {
  return WriteEventAt(module, phase, trace_id, proc_id, MonoNowNs());
}

uint64_t PerfTraceWriter::WriteEventAt(const char* module, const char* phase,
                                       uint64_t trace_id, uint32_t proc_id,
                                       uint64_t mono_ns) {
  uint64_t generation = 0;
  if (!CaptureRunGeneration(&generation)) return 0;
  QueuedTraceRecord record;
  record.run_generation = generation;
  record.kind = TraceRecordKind::EVENT;
  record.event_id = NextEventId();
  record.trace_id = trace_id;
  record.proc_id = proc_id;
  record.mono_ns = mono_ns;
  record.pid = static_cast<uint32_t>(getpid());
  record.tid = GetTid();
  bool truncated = false;
  truncated |= CopyFixedText(module, record.module, sizeof(record.module));
  truncated |= CopyFixedText(phase, record.phase, sizeof(record.phase));
  const bool enqueued = EnqueueRecord(record);
  if (truncated && enqueued) {
    truncated_records_.fetch_add(1, std::memory_order_relaxed);
  }
  return enqueued ? record.event_id : 0;
}

void PerfTraceWriter::WriteEdge(uint64_t event_id, const char* module,
                                const char* edge, uint64_t trace_id,
                                uint32_t proc_id, bool trace_valid,
                                const char* channel, uint32_t input_seq,
                                uint32_t output_seq, uint64_t data_ts_ns,
                                uint32_t object_count) {
  uint64_t generation = 0;
  if (!CaptureRunGeneration(&generation)) return;
  QueuedTraceRecord record;
  record.run_generation = generation;
  record.kind = TraceRecordKind::EDGE;
  record.event_id = event_id;
  record.trace_id = trace_id;
  record.proc_id = proc_id;
  record.mono_ns = MonoNowNs();
  record.trace_valid = trace_valid;
  record.input_seq = input_seq;
  record.output_seq = output_seq;
  record.data_ts_ns = data_ts_ns;
  record.object_count = object_count;
  record.pid = static_cast<uint32_t>(getpid());
  record.tid = GetTid();
  bool truncated = false;
  truncated |= CopyFixedText(module, record.module, sizeof(record.module));
  truncated |= CopyFixedText(edge, record.edge, sizeof(record.edge));
  truncated |= CopyFixedText(channel, record.channel, sizeof(record.channel));
  const bool enqueued = EnqueueRecord(record);
  if (truncated && enqueued) {
    truncated_records_.fetch_add(1, std::memory_order_relaxed);
  }
}

void PerfTraceWriter::EmitInputBoundary(
    const char* module, const char* phase, uint64_t trace_id, uint32_t proc_id,
    bool trace_valid, const char* channel, uint32_t input_seq,
    uint64_t data_ts_ns, uint32_t object_count,
    uint64_t primary_parent_trace_id, uint32_t parent_trace_count) {
  uint64_t generation = 0;
  if (!CaptureRunGeneration(&generation)) return;
  QueuedTraceRecord record;
  record.run_generation = generation;
  record.kind = TraceRecordKind::BOUNDARY;
  record.event_id = NextEventId();
  record.trace_id = trace_id;
  record.proc_id = proc_id;
  record.mono_ns = MonoNowNs();
  record.trace_valid = trace_valid;
  record.input_seq = input_seq;
  record.data_ts_ns = data_ts_ns;
  record.object_count = object_count;
  record.primary_parent_trace_id = primary_parent_trace_id;
  record.parent_trace_count = parent_trace_count;
  record.pid = static_cast<uint32_t>(getpid());
  record.tid = GetTid();
  bool truncated = false;
  truncated |= CopyFixedText(module, record.module, sizeof(record.module));
  truncated |= CopyFixedText(phase, record.phase, sizeof(record.phase));
  truncated |= CopyFixedText("in", record.edge, sizeof(record.edge));
  truncated |= CopyFixedText(channel, record.channel, sizeof(record.channel));
  const bool enqueued = EnqueueRecord(record);
  if (truncated && enqueued) {
    truncated_records_.fetch_add(1, std::memory_order_relaxed);
  }
}

void PerfTraceWriter::EmitOutputBoundary(
    const char* module, const char* phase, uint64_t trace_id, uint32_t proc_id,
    bool trace_valid, const char* channel, uint32_t output_seq,
    uint64_t data_ts_ns, uint32_t object_count,
    uint64_t primary_parent_trace_id, uint32_t parent_trace_count) {
  EmitOutputBoundaryAt(module, phase, trace_id, proc_id, trace_valid, channel,
                       output_seq, data_ts_ns, object_count, MonoNowNs(),
                       primary_parent_trace_id, parent_trace_count);
}

void PerfTraceWriter::EmitOutputBoundaryAt(
    const char* module, const char* phase, uint64_t trace_id, uint32_t proc_id,
    bool trace_valid, const char* channel, uint32_t output_seq,
    uint64_t data_ts_ns, uint32_t object_count, uint64_t mono_ns,
    uint64_t primary_parent_trace_id, uint32_t parent_trace_count) {
  uint64_t generation = 0;
  if (!CaptureRunGeneration(&generation)) return;
  QueuedTraceRecord record;
  record.run_generation = generation;
  record.kind = TraceRecordKind::BOUNDARY;
  record.event_id = NextEventId();
  record.trace_id = trace_id;
  record.proc_id = proc_id;
  record.mono_ns = mono_ns;
  record.trace_valid = trace_valid;
  record.output_seq = output_seq;
  record.data_ts_ns = data_ts_ns;
  record.object_count = object_count;
  record.primary_parent_trace_id = primary_parent_trace_id;
  record.parent_trace_count = parent_trace_count;
  record.pid = static_cast<uint32_t>(getpid());
  record.tid = GetTid();
  bool truncated = false;
  truncated |= CopyFixedText(module, record.module, sizeof(record.module));
  truncated |= CopyFixedText(phase, record.phase, sizeof(record.phase));
  truncated |= CopyFixedText("out", record.edge, sizeof(record.edge));
  truncated |= CopyFixedText(channel, record.channel, sizeof(record.channel));
  const bool enqueued = EnqueueRecord(record);
  if (truncated && enqueued) {
    truncated_records_.fetch_add(1, std::memory_order_relaxed);
  }
}

bool PerfTraceWriter::WriteFusionRowLocked(const FusionInputRow& row) {
  const std::string module_tag = "perception.multi_sensor_fusion";
  auto* ofs = GetTableStreamLocked(TraceTable::FUSION_INPUTS, module_tag);
  if (!ofs) return false;

  *ofs << current_run_id_ << ','
       << kE2eTraceSchemaVersion << ','
       << row.fusion_trace_id << ','
       << row.parent_trace_id << ','
       << CsvEscape(row.sensor) << ','
       << CsvEscape(row.sensor_kind) << ','
       << row.frame_time_ns << ','
       << row.sensor_ts_ns << ','
       << row.object_count << ','
       << (row.is_main_sensor ? 1 : 0) << '\n';
  return ofs->good();
}

bool PerfTraceWriter::WriteTraceAnchorRowLocked(const TraceAnchorRow& row) {
  const std::string module_tag = "trace_anchor";
  auto* ofs = GetTableStreamLocked(TraceTable::TRACE_ANCHOR, module_tag);
  if (!ofs) return false;

  *ofs << current_run_id_ << ','
       << kE2eTraceSchemaVersion << ','
       << row.trace_id << ','
       << CsvEscape(row.sensor_kind) << ','
       << CsvEscape(row.sensor_id) << ','
       << row.data_ts_ns << ','
       << FormatSensorTimeSecCsv(row.sensor_time_sec) << ','
       << row.preproc_enter_ns << ','
       << row.ingress_ms << '\n';
  return ofs->good();
}

bool PerfTraceWriter::WriteQueuedRecord(const QueuedTraceRecord& record) {
  if (current_run_id_.empty()) {
    return false;
  }

  switch (record.kind) {
    case TraceRecordKind::LEGACY: {
      auto* ofs = GetLegacyStreamLocked(record.module);
      if (!ofs) return false;
      *ofs << current_run_id_ << ',' << record.module << ',' << record.phase
           << ',' << record.mono_ns << ',' << record.header_ts_sec << ','
           << record.measurement_time_ns << ',' << record.source_ts_ns << ','
           << record.source_kind << ',' << record.lidar_ts_ns << ','
           << record.camera_ts_ns << ',' << record.radar_ts_ns << ','
           << record.seq << ',' << record.upstream_seq << ',' << record.pid
           << ',' << record.tid << ',' << record.channel << ','
           << record.trace_id << ',' << record.parent_trace_id << '\n';
      return ofs->good();
    }
    case TraceRecordKind::EVENT: {
      EventRow row;
      row.event_id = record.event_id;
      row.trace_id = record.trace_id;
      row.proc_id = record.proc_id;
      row.module = record.module;
      row.phase = record.phase;
      row.mono_ns = record.mono_ns;
      row.pid = record.pid;
      row.tid = record.tid;
      return WriteEventRowLocked(row);
    }
    case TraceRecordKind::EDGE: {
      EdgeRow row;
      row.event_id = record.event_id;
      row.trace_id = record.trace_id;
      row.proc_id = record.proc_id;
      row.module = record.module;
      row.edge = record.edge;
      row.mono_ns = record.mono_ns;
      row.trace_valid = record.trace_valid;
      row.channel = record.channel;
      row.input_seq = record.input_seq;
      row.output_seq = record.output_seq;
      row.data_ts_ns = record.data_ts_ns;
      row.object_count = record.object_count;
      row.primary_parent_trace_id = record.primary_parent_trace_id;
      row.parent_trace_count = record.parent_trace_count;
      row.pid = record.pid;
      row.tid = record.tid;
      return WriteEdgeRowLocked(row);
    }
    case TraceRecordKind::BOUNDARY: {
      EventRow event;
      event.event_id = record.event_id;
      event.trace_id = record.trace_id;
      event.proc_id = record.proc_id;
      event.module = record.module;
      event.phase = record.phase;
      event.mono_ns = record.mono_ns;
      event.pid = record.pid;
      event.tid = record.tid;

      EdgeRow edge;
      edge.event_id = record.event_id;
      edge.trace_id = record.trace_id;
      edge.proc_id = record.proc_id;
      edge.module = record.module;
      edge.edge = record.edge;
      edge.mono_ns = record.mono_ns;
      edge.trace_valid = record.trace_valid;
      edge.channel = record.channel;
      edge.input_seq = record.input_seq;
      edge.output_seq = record.output_seq;
      edge.data_ts_ns = record.data_ts_ns;
      edge.object_count = record.object_count;
      edge.primary_parent_trace_id = record.primary_parent_trace_id;
      edge.parent_trace_count = record.parent_trace_count;
      edge.pid = record.pid;
      edge.tid = record.tid;

      const bool event_ok = WriteEventRowLocked(event);
      const bool edge_ok = WriteEdgeRowLocked(edge);
      return event_ok && edge_ok;
    }
    case TraceRecordKind::FUSION_INPUT: {
      FusionInputRow row;
      row.fusion_trace_id = record.fusion_trace_id;
      row.parent_trace_id = record.parent_trace_id;
      row.sensor = record.sensor;
      row.sensor_kind = record.sensor_kind;
      row.frame_time_ns = record.frame_time_ns;
      row.sensor_ts_ns = record.sensor_ts_ns;
      row.object_count = record.object_count;
      row.is_main_sensor = record.is_main_sensor;
      return WriteFusionRowLocked(row);
    }
    case TraceRecordKind::TRACE_ANCHOR: {
      TraceAnchorRow row;
      row.trace_id = record.trace_id;
      row.sensor_kind = record.sensor_kind;
      row.sensor_id = record.sensor_id;
      row.data_ts_ns = record.data_ts_ns;
      row.sensor_time_sec = record.sensor_time_sec;
      row.preproc_enter_ns = record.preproc_enter_ns;
      row.ingress_ms = record.ingress_ms;
      return WriteTraceAnchorRowLocked(row);
    }
    case TraceRecordKind::PREDICTION_WORKER_EVENT: {
      auto* ofs = GetTableStreamLocked(
          TraceTable::PREDICTION_WORKER_EVENTS, "prediction");
      if (!ofs) return false;
      *ofs << current_run_id_ << ',' << kE2eTraceSchemaVersion << ','
           << record.event_id << ',' << record.mono_ns << ','
           << record.pid << ',' << record.tid << ',' << record.trace_id << ','
           << record.frame_instance_id << ',' << record.task_id << ','
           << record.group_id << ',' << record.obstacle_id << ','
           << record.model_call_id << ',' << record.inference_id << ','
           << record.lane_sequence_index << ',' << record.pool_level << ','
           << CsvEscape(record.phase) << ',' << CsvEscape(record.status) << ','
           << CsvEscape(record.model_type) << ',' << CsvEscape(record.device_type)
           << ',' << record.input_rows << ',' << record.input_cols << '\n';
      return ofs->good();
    }
    case TraceRecordKind::CONTROL_TIMER_DIAGNOSTIC: {
      auto* ofs = GetTableStreamLocked(
          TraceTable::CONTROL_TIMER_DIAGNOSTICS, "control");
      if (!ofs) return false;
      const uint64_t wall_ns =
          record.mono_ns >= record.measurement_time_ns
              ? record.mono_ns - record.measurement_time_ns
              : 0;
      *ofs << current_run_id_ << ',' << kE2eTraceSchemaVersion << ','
           << record.event_id << ',' << record.trace_id << ','
           << record.proc_id << ',' << CsvEscape(record.phase) << ','
           << record.measurement_time_ns << ',' << record.mono_ns << ','
           << wall_ns << ',' << record.source_ts_ns << ','
           << record.lidar_ts_ns << ',' << record.camera_ts_ns << ','
           << record.seq << ',' << record.upstream_seq << ','
           << static_cast<int32_t>(record.input_seq) << ','
           << static_cast<int32_t>(record.output_seq) << ','
           << static_cast<int64_t>(record.data_ts_ns) << ','
           << (record.trace_valid ? 1 : 0) << ',' << record.pid << ','
           << record.tid << '\n';
      return ofs->good();
    }
  }
  return false;
}

void PerfTraceWriter::WriteFusionInput(const FusionInputRow& row) {
  uint64_t generation = 0;
  if (!CaptureRunGeneration(&generation)) return;
  QueuedTraceRecord record;
  record.run_generation = generation;
  record.kind = TraceRecordKind::FUSION_INPUT;
  record.fusion_trace_id = row.fusion_trace_id;
  record.parent_trace_id = row.parent_trace_id;
  record.frame_time_ns = row.frame_time_ns;
  record.sensor_ts_ns = row.sensor_ts_ns;
  record.object_count = row.object_count;
  record.is_main_sensor = row.is_main_sensor;
  bool truncated = false;
  truncated |= CopyFixedText(row.sensor, record.sensor, sizeof(record.sensor));
  truncated |= CopyFixedText(row.sensor_kind, record.sensor_kind,
                             sizeof(record.sensor_kind));
  const bool enqueued = EnqueueRecord(record);
  if (truncated && enqueued) {
    truncated_records_.fetch_add(1, std::memory_order_relaxed);
  }
}

void PerfTraceWriter::WriteTraceAnchor(const TraceAnchorRow& row) {
  uint64_t generation = 0;
  if (!CaptureRunGeneration(&generation)) return;
  QueuedTraceRecord record;
  record.run_generation = generation;
  record.kind = TraceRecordKind::TRACE_ANCHOR;
  record.trace_id = row.trace_id;
  record.data_ts_ns = row.data_ts_ns;
  record.sensor_time_sec = row.sensor_time_sec;
  record.preproc_enter_ns = row.preproc_enter_ns;
  record.ingress_ms = row.ingress_ms;
  bool truncated = false;
  truncated |= CopyFixedText(row.sensor_kind, record.sensor_kind,
                             sizeof(record.sensor_kind));
  truncated |=
      CopyFixedText(row.sensor_id, record.sensor_id, sizeof(record.sensor_id));
  const bool enqueued = EnqueueRecord(record);
  if (truncated && enqueued) {
    truncated_records_.fetch_add(1, std::memory_order_relaxed);
  }
}

void PerfTraceWriter::WriteControlTimerDiagnostic(
    const ControlTimerDiagnosticRow& row) {
  uint64_t generation = 0;
  if (!CaptureRunGeneration(&generation)) return;
  QueuedTraceRecord record;
  record.run_generation = generation;
  record.kind = TraceRecordKind::CONTROL_TIMER_DIAGNOSTIC;
  record.event_id = NextEventId();
  record.trace_id = row.trace_id;
  record.proc_id = row.proc_id;
  record.measurement_time_ns = row.start_mono_ns;
  record.mono_ns = row.end_mono_ns;
  record.source_ts_ns = row.thread_cpu_ns;
  record.lidar_ts_ns = row.mutex_lock_wall_ns;
  record.camera_ts_ns = row.cv_notify_ns;
  record.seq = static_cast<uint32_t>(std::min<uint64_t>(
      row.voluntary_context_switches, UINT32_MAX));
  record.upstream_seq = static_cast<uint32_t>(std::min<uint64_t>(
      row.involuntary_context_switches, UINT32_MAX));
  record.input_seq = static_cast<uint32_t>(row.cpu_start);
  record.output_seq = static_cast<uint32_t>(row.cpu_end);
  record.data_ts_ns = static_cast<uint64_t>(row.overrun_ns);
  record.trace_valid = row.rate_rebased;
  record.pid = static_cast<uint32_t>(getpid());
  record.tid = GetTid();
  const bool truncated =
      CopyFixedText(row.phase, record.phase, sizeof(record.phase));
  const bool enqueued = EnqueueRecord(record);
  if (truncated && enqueued) {
    truncated_records_.fetch_add(1, std::memory_order_relaxed);
  }
}

void PerfTraceWriter::RecordFusionInputs(
    uint64_t fusion_trace_id, const std::vector<FusionInputRow>& inputs) {
  uint64_t generation = 0;
  if (!CaptureRunGeneration(&generation)) return;
  for (const FusionInputRow& in : inputs) {
    QueuedTraceRecord record;
    record.run_generation = generation;
    record.kind = TraceRecordKind::FUSION_INPUT;
    record.fusion_trace_id = fusion_trace_id;
    record.parent_trace_id = in.parent_trace_id;
    record.frame_time_ns = in.frame_time_ns;
    record.sensor_ts_ns = in.sensor_ts_ns;
    record.object_count = in.object_count;
    record.is_main_sensor = in.is_main_sensor;
    bool truncated = false;
    truncated |=
        CopyFixedText(in.sensor, record.sensor, sizeof(record.sensor));
    truncated |= CopyFixedText(in.sensor_kind, record.sensor_kind,
                               sizeof(record.sensor_kind));
    const bool enqueued = EnqueueRecord(record);
    if (truncated && enqueued) {
      truncated_records_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

}  // namespace util
}  // namespace common
}  // namespace apollo
