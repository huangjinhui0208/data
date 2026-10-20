#pragma once

#include "modules/common/util/trace_header_util.h"

#include <stdint.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <time.h>

namespace apollo {
namespace common {
namespace util {

struct PerfTraceRow {
  std::string module;
  std::string phase;
  uint64_t mono_ns = 0;
  double header_ts_sec = 0.0;
  uint64_t measurement_time_ns = 0;
  uint64_t source_ts_ns = 0;
  std::string source_kind;
  uint64_t lidar_ts_ns = 0;
  uint64_t camera_ts_ns = 0;
  uint64_t radar_ts_ns = 0;
  uint32_t seq = 0;
  uint32_t upstream_seq = 0;
  uint32_t pid = 0;
  uint32_t tid = 0;
  std::string channel;
  uint64_t trace_id = 0;
  uint64_t parent_trace_id = 0;
};

inline uint64_t MonoNowNs() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

inline uint32_t GetTid() {
  // Linux TIDs do not change during a thread's lifetime.  Avoid paying for a
  // syscall at every trace point on the application hot path.
  static thread_local const uint32_t tid =
      static_cast<uint32_t>(syscall(SYS_gettid));
  return tid;
}

inline uint64_t PickSourceTsNs(uint64_t lidar_ts_ns, uint64_t camera_ts_ns,
                               uint64_t radar_ts_ns, std::string* kind) {
  if (lidar_ts_ns != 0) {
    if (kind) *kind = "lidar";
    return lidar_ts_ns;
  }
  if (camera_ts_ns != 0) {
    if (kind) *kind = "camera";
    return camera_ts_ns;
  }
  if (radar_ts_ns != 0) {
    if (kind) *kind = "radar";
    return radar_ts_ns;
  }
  if (kind) *kind = "none";
  return 0;
}

inline constexpr const char* kE2eTraceSchemaVersion = "e2e_trace_v3";

inline uint64_t NextEventId() {
  static std::atomic<uint64_t> s_event_seq{0};
  return s_event_seq.fetch_add(1, std::memory_order_relaxed);
}

struct EventRow {
  uint64_t event_id = 0;
  uint64_t trace_id = 0;
  uint32_t proc_id = 0;
  const char* module = nullptr;
  const char* phase = nullptr;
  uint64_t mono_ns = 0;
  uint32_t pid = 0;
  uint32_t tid = 0;
};

// Prediction-only task lineage. Strings are copied before this call returns.
struct PredictionWorkerEvent {
  uint64_t trace_id = 0;
  uint64_t frame_instance_id = 0;
  uint64_t task_id = 0;
  int64_t group_id = -1;
  int64_t obstacle_id = -1;
  uint64_t model_call_id = 0;
  uint64_t inference_id = 0;
  int64_t lane_sequence_index = -1;
  int64_t pool_level = -1;
  const char* phase = nullptr;
  const char* status = nullptr;
  const char* model_type = nullptr;
  const char* device_type = nullptr;
  int64_t input_rows = -1;
  int64_t input_cols = -1;
};

struct EdgeRow {
  uint64_t event_id = 0;
  uint64_t trace_id = 0;
  uint32_t proc_id = 0;
  const char* module = nullptr;
  const char* edge = nullptr;
  uint64_t mono_ns = 0;
  bool trace_valid = false;
  const char* channel = nullptr;
  uint32_t input_seq = 0;
  uint32_t output_seq = 0;
  uint64_t data_ts_ns = 0;
  uint32_t object_count = 0;
  uint64_t primary_parent_trace_id = 0;
  uint32_t parent_trace_count = 0;
  uint32_t pid = 0;
  uint32_t tid = 0;
};

struct FusionInputRow {
  uint64_t fusion_trace_id = 0;
  uint64_t parent_trace_id = 0;
  std::string sensor;
  std::string sensor_kind;
  uint64_t frame_time_ns = 0;
  uint64_t sensor_ts_ns = 0;
  uint32_t object_count = 0;
  bool is_main_sensor = false;
};

struct TraceAnchorRow {
  uint64_t trace_id = 0;
  std::string sensor_kind;
  std::string sensor_id;
  uint64_t data_ts_ns = 0;
  double sensor_time_sec = 0.0;
  uint64_t preproc_enter_ns = 0;
  double ingress_ms = 0.0;
};

// Low-rate diagnostic emitted only for abnormal Control timer scheduling.
// Keeping this separate from EventRow preserves the event pairing schema and
// gives numeric measurements explicit meanings instead of packing them into
// trace_id/proc_id.
struct ControlTimerDiagnosticRow {
  const char* phase = nullptr;
  uint64_t trace_id = 0;
  uint32_t proc_id = 0;
  uint64_t start_mono_ns = 0;
  uint64_t end_mono_ns = 0;
  uint64_t thread_cpu_ns = 0;
  uint64_t mutex_lock_wall_ns = 0;
  uint64_t cv_notify_ns = 0;
  uint64_t voluntary_context_switches = 0;
  uint64_t involuntary_context_switches = 0;
  int32_t cpu_start = -1;
  int32_t cpu_end = -1;
  int64_t overrun_ns = 0;
  bool rate_rebased = false;
};

// Default ostream precision is 6 and truncates Unix epoch seconds. Use 17
// significant digits so CSV round-trip keeps sub-second (ns-class) info.
std::string FormatSensorTimeSecCsv(double sensor_time_sec);

static constexpr const char* kScbDataRoot = "/apollo/data/scb_data";
static constexpr const char* TRIGGER_FILE =
    "/apollo/data/scb_data/.collect_active";
static constexpr int POLL_INTERVAL_MS = 100;

enum class TraceTable {
  LEGACY_WIDE,
  EVENTS,
  MESSAGE_CONTEXT,
  FUSION_INPUTS,
  TRACE_ANCHOR,
  CONTROL_TIMER_DIAGNOSTICS,
  PREDICTION_WORKER_EVENTS,
};

struct QueuedTraceRecord;
class AsyncTraceQueue;

class PerfTraceWriter {
 public:
  static PerfTraceWriter& Instance();
  ~PerfTraceWriter();

  bool IsActive() const {
    return active_.load(std::memory_order_relaxed);
  }

  // Zero means no active capture. Retain this token with asynchronous tasks.
  uint64_t CaptureSessionToken() const;
  // Rejects inactive or stale sessions, including a session change during enqueue.
  bool WritePredictionWorkerEvent(const PredictionWorkerEvent& row,
                                  uint64_t expected_token);

  void Write(const PerfTraceRow& row);

  uint64_t WriteEvent(const char* module, const char* phase, uint64_t trace_id,
                      uint32_t proc_id);

  // Same as WriteEvent but records a pre-captured mono timestamp (e.g. publish
  // enter).  Both methods only enqueue fixed-size data; CSV I/O is performed by
  // the background writer thread.
  uint64_t WriteEventAt(const char* module, const char* phase, uint64_t trace_id,
                        uint32_t proc_id, uint64_t mono_ns);

  void WriteEdge(uint64_t event_id, const char* module, const char* edge,
                 uint64_t trace_id, uint32_t proc_id, bool trace_valid,
                 const char* channel, uint32_t input_seq, uint32_t output_seq,
                 uint64_t data_ts_ns, uint32_t object_count);

  void WriteFusionInput(const FusionInputRow& row);

  void WriteTraceAnchor(const TraceAnchorRow& row);

  void WriteControlTimerDiagnostic(const ControlTimerDiagnosticRow& row);

  void RecordFusionInputs(uint64_t fusion_trace_id,
                          const std::vector<FusionInputRow>& inputs);

  // Input/output boundaries (shared mono_ns + shared event_id for both tables).
  void EmitInputBoundary(const char* module, const char* phase,
                         uint64_t trace_id, uint32_t proc_id, bool trace_valid,
                         const char* channel, uint32_t input_seq,
                         uint64_t data_ts_ns, uint32_t object_count,
                         uint64_t primary_parent_trace_id = 0,
                         uint32_t parent_trace_count = 0);

  void EmitOutputBoundary(const char* module, const char* phase,
                          uint64_t trace_id, uint32_t proc_id, bool trace_valid,
                          const char* channel, uint32_t output_seq,
                          uint64_t data_ts_ns, uint32_t object_count,
                          uint64_t primary_parent_trace_id = 0,
                          uint32_t parent_trace_count = 0);

  // Same as EmitOutputBoundary but uses a pre-captured mono_ns for both the
  // events row and the message_context (out) edge.
  void EmitOutputBoundaryAt(const char* module, const char* phase,
                            uint64_t trace_id, uint32_t proc_id, bool trace_valid,
                            const char* channel, uint32_t output_seq,
                            uint64_t data_ts_ns, uint32_t object_count,
                            uint64_t mono_ns,
                            uint64_t primary_parent_trace_id = 0,
                            uint32_t parent_trace_count = 0);

 private:
  PerfTraceWriter();
  void PollLoop();

  bool CaptureRunGeneration(uint64_t* generation) const;
  bool EnqueueRecord(const QueuedTraceRecord& record);
  bool DrainQueuedRecords(uint64_t max_records);
  bool WriteQueuedRecord(const QueuedTraceRecord& record);
  void StartRun(const std::string& run_id);
  void StopRun();
  bool FlushAllFiles();
  void RecordIoError();
  void WriteHealthSummary();

  std::ofstream* GetLegacyStreamLocked(const std::string& module);
  std::ofstream* GetTableStreamLocked(TraceTable table,
                                      const std::string& module);
  std::string MakeTableRelativePathLocked(TraceTable table,
                                             const std::string& module) const;

  void CloseAllFiles();

  bool WriteEventRowLocked(const EventRow& row);
  bool WriteEdgeRowLocked(const EdgeRow& row);
  bool WriteFusionRowLocked(const FusionInputRow& row);
  bool WriteTraceAnchorRowLocked(const TraceAnchorRow& row);

  std::atomic<bool> active_{false};

  std::string current_run_id_;
  std::string current_run_dir_;
  std::unordered_map<std::string, std::unique_ptr<std::ofstream>> files_;

  std::unique_ptr<AsyncTraceQueue> async_queue_;
  std::thread io_thread_;
  std::atomic<uint64_t> run_generation_{0};
  std::atomic<uint64_t> producers_in_flight_{0};
  std::atomic<uint64_t> truncated_records_{0};
  std::atomic<uint64_t> enqueue_sample_count_{0};
  std::atomic<uint64_t> enqueue_sample_total_ns_{0};
  std::atomic<uint64_t> enqueue_sample_max_ns_{0};
  uint64_t written_records_ = 0;
  uint64_t io_errors_ = 0;
  uint64_t run_start_mono_ns_ = 0;
  uint64_t writer_cpu_start_ns_ = 0;
  std::atomic<bool> running_{true};
};

// Emits a module phase when the current C++ scope exits.  Construct this
// immediately after proc_enter so normal returns and early returns share the
// same callback boundary.  The destructor captures its timestamp before the
// asynchronous enqueue, so proc_exit enqueue cost is outside callback_total.
class ScopedPerfTraceEvent final {
 public:
  ScopedPerfTraceEvent(const char* module, const char* phase,
                       uint64_t trace_id, uint32_t proc_id)
      : module_(module),
        phase_(phase),
        trace_id_(trace_id),
        proc_id_(proc_id) {}

  ~ScopedPerfTraceEvent() {
    auto& writer = PerfTraceWriter::Instance();
    if (writer.IsActive()) {
      writer.WriteEvent(module_, phase_, trace_id_, proc_id_);
    }
  }

  ScopedPerfTraceEvent(const ScopedPerfTraceEvent&) = delete;
  ScopedPerfTraceEvent& operator=(const ScopedPerfTraceEvent&) = delete;

 private:
  const char* module_;
  const char* phase_;
  uint64_t trace_id_;
  uint32_t proc_id_;
};

inline void ApolloTraceV2Record(const char* module, const char* phase,
                                double header_ts_sec, uint64_t measurement_time_ns,
                                uint64_t lidar_ts_ns, uint64_t camera_ts_ns,
                                uint64_t radar_ts_ns, uint32_t seq,
                                uint32_t upstream_seq, const char* channel,
                                uint64_t trace_id) {
  auto& w = PerfTraceWriter::Instance();
  if (!w.IsActive()) {
    return;
  }
  PerfTraceRow row;
  row.module = module;
  row.phase = phase;
  row.mono_ns = MonoNowNs();
  row.header_ts_sec = header_ts_sec;
  row.measurement_time_ns = measurement_time_ns;
  row.lidar_ts_ns = lidar_ts_ns;
  row.camera_ts_ns = camera_ts_ns;
  row.radar_ts_ns = radar_ts_ns;
  row.source_ts_ns =
      PickSourceTsNs(lidar_ts_ns, camera_ts_ns, radar_ts_ns, &row.source_kind);
  row.seq = seq;
  row.upstream_seq = upstream_seq;
  row.pid = static_cast<uint32_t>(getpid());
  row.tid = GetTid();
  row.channel = channel;
  row.trace_id = trace_id;
  w.Write(row);
}

inline void RecordFusionInputs(
    uint64_t fusion_trace_id,
    const std::vector<FusionInputRow>& inputs) {
  PerfTraceWriter::Instance().RecordFusionInputs(fusion_trace_id, inputs);
}

inline void RecordTraceAnchor(const TraceAnchorRow& row) {
  PerfTraceWriter::Instance().WriteTraceAnchor(row);
}

}  // namespace util
}  // namespace common
}  // namespace apollo

#define APOLLO_TRACE_V2(MODULE, PHASE, HEADER_TS_SEC, MEASUREMENT_TIME_NS,       \
                        LIDAR_TS, CAMERA_TS, RADAR_TS, SEQ, UP_SEQ, CHANNEL,       \
                        TRACE_ID)                                                   \
  ::apollo::common::util::ApolloTraceV2Record(                                      \
      (MODULE), (PHASE), static_cast<double>(HEADER_TS_SEC),                         \
      static_cast<uint64_t>(MEASUREMENT_TIME_NS),                                    \
      static_cast<uint64_t>(LIDAR_TS), static_cast<uint64_t>(CAMERA_TS),               \
      static_cast<uint64_t>(RADAR_TS), static_cast<uint32_t>(SEQ),                    \
      static_cast<uint32_t>(UP_SEQ), (CHANNEL),                                       \
      static_cast<uint64_t>(TRACE_ID))

#define APOLLO_TRACE(MODULE, PHASE, HEADER_TS_SEC, LIDAR_TS, CAMERA_TS, RADAR_TS, \
                     SEQ, UP_SEQ, CHANNEL)                                        \
  APOLLO_TRACE_V2(MODULE, PHASE, HEADER_TS_SEC, 0ULL, LIDAR_TS, CAMERA_TS, RADAR_TS, \
                    SEQ, UP_SEQ, CHANNEL, 0)

#define APOLLO_TRACE_EVENT(MODULE, PHASE, TRACE_ID, PROC_ID)                           \
  ((::apollo::common::util::PerfTraceWriter::Instance().IsActive())                    \
       ? ::apollo::common::util::PerfTraceWriter::Instance().WriteEvent(                \
               (MODULE), (PHASE), static_cast<uint64_t>(TRACE_ID),                      \
               static_cast<uint32_t>(PROC_ID))                                         \
       : 0ULL)

#define APOLLO_TRACE_EVENT_AT(MONO_NS, MODULE, PHASE, TRACE_ID, PROC_ID)               \
  ((::apollo::common::util::PerfTraceWriter::Instance().IsActive())                    \
       ? ::apollo::common::util::PerfTraceWriter::Instance().WriteEventAt(              \
               (MODULE), (PHASE), static_cast<uint64_t>(TRACE_ID),                      \
               static_cast<uint32_t>(PROC_ID), static_cast<uint64_t>(MONO_NS))          \
       : 0ULL)

#define APOLLO_TRACE_INPUT(MODULE, PHASE, TRACE_ID, PROC_ID, TRACE_VALID, CHANNEL, \
                           INPUT_SEQ, DATA_TS_NS, OBJECT_COUNT)                    \
  do {                                                                               \
    auto& _apollo_ptw_inst =                                                         \
        ::apollo::common::util::PerfTraceWriter::Instance();                           \
    if (_apollo_ptw_inst.IsActive()) {                                              \
      _apollo_ptw_inst.EmitInputBoundary(                                            \
          (MODULE), (PHASE), static_cast<uint64_t>(TRACE_ID),                        \
          static_cast<uint32_t>(PROC_ID), !!(TRACE_VALID), (CHANNEL),                \
          static_cast<uint32_t>(INPUT_SEQ), static_cast<uint64_t>(DATA_TS_NS),       \
          static_cast<uint32_t>(OBJECT_COUNT));                                      \
    }                                                                                \
  } while (0)

#define APOLLO_TRACE_OUTPUT(MODULE, PHASE, TRACE_ID, PROC_ID, TRACE_VALID, CHANNEL, \
                            OUTPUT_SEQ, DATA_TS_NS, OBJECT_COUNT)                   \
  do {                                                                               \
    auto& _apollo_ptw_inst_o =                                                       \
        ::apollo::common::util::PerfTraceWriter::Instance();                           \
    if (_apollo_ptw_inst_o.IsActive()) {                                            \
      _apollo_ptw_inst_o.EmitOutputBoundary(                                         \
          (MODULE), (PHASE), static_cast<uint64_t>(TRACE_ID),                        \
          static_cast<uint32_t>(PROC_ID), !!(TRACE_VALID), (CHANNEL),               \
          static_cast<uint32_t>(OUTPUT_SEQ), static_cast<uint64_t>(DATA_TS_NS),       \
          static_cast<uint32_t>(OBJECT_COUNT));                                       \
    }                                                                                \
  } while (0)

// Record output boundary with a pre-captured mono timestamp (call after Write).
#define APOLLO_TRACE_OUTPUT_AT(MONO_NS, MODULE, PHASE, TRACE_ID, PROC_ID,              \
                               TRACE_VALID, CHANNEL, OUTPUT_SEQ, DATA_TS_NS,           \
                               OBJECT_COUNT)                                          \
  do {                                                                               \
    auto& _apollo_ptw_inst_oat =                                                     \
        ::apollo::common::util::PerfTraceWriter::Instance();                           \
    if (_apollo_ptw_inst_oat.IsActive()) {                                           \
      _apollo_ptw_inst_oat.EmitOutputBoundaryAt(                                     \
          (MODULE), (PHASE), static_cast<uint64_t>(TRACE_ID),                        \
          static_cast<uint32_t>(PROC_ID), !!(TRACE_VALID), (CHANNEL),                \
          static_cast<uint32_t>(OUTPUT_SEQ), static_cast<uint64_t>(DATA_TS_NS),       \
          static_cast<uint32_t>(OBJECT_COUNT), static_cast<uint64_t>(MONO_NS));       \
    }                                                                                \
  } while (0)

// Record message boundary with parent span metadata from a protobuf Header.
#define APOLLO_TRACE_INPUT_HDR(MODULE, PHASE, HEADER, PROC_ID, TRACE_VALID, CHANNEL, \
                               INPUT_SEQ, DATA_TS_NS, OBJECT_COUNT)                  \
  do {                                                                               \
    const auto& _apollo_in_hdr = (HEADER);                                           \
    const uint64_t _apollo_in_tid =                                                  \
        _apollo_in_hdr.has_trace_id()                                                \
            ? static_cast<uint64_t>(_apollo_in_hdr.trace_id())                      \
            : 0ULL;                                                                  \
    const uint64_t _apollo_in_pp =                                                   \
        ::apollo::common::util::PrimaryParentTraceId(_apollo_in_hdr);              \
    const uint32_t _apollo_in_pc =                                                   \
        ::apollo::common::util::ParentTraceCount(_apollo_in_hdr);                  \
    auto& _apollo_ptw_in = ::apollo::common::util::PerfTraceWriter::Instance();      \
    if (_apollo_ptw_in.IsActive()) {                                                 \
      _apollo_ptw_in.EmitInputBoundary(                                              \
          (MODULE), (PHASE), _apollo_in_tid, static_cast<uint32_t>(PROC_ID),         \
          !!(TRACE_VALID), (CHANNEL), static_cast<uint32_t>(INPUT_SEQ),              \
          static_cast<uint64_t>(DATA_TS_NS), static_cast<uint32_t>(OBJECT_COUNT),    \
          _apollo_in_pp, _apollo_in_pc);                                             \
    }                                                                                \
  } while (0)

#define APOLLO_TRACE_OUTPUT_HDR(MODULE, PHASE, HEADER, PROC_ID, TRACE_VALID,        \
                                CHANNEL, OUTPUT_SEQ, DATA_TS_NS, OBJECT_COUNT)     \
  do {                                                                               \
    const auto& _apollo_out_hdr = (HEADER);                                          \
    const uint64_t _apollo_out_tid =                                                 \
        _apollo_out_hdr.has_trace_id()                                               \
            ? static_cast<uint64_t>(_apollo_out_hdr.trace_id())                     \
            : 0ULL;                                                                  \
    const uint64_t _apollo_out_pp =                                                  \
        ::apollo::common::util::PrimaryParentTraceId(_apollo_out_hdr);             \
    const uint32_t _apollo_out_pc =                                                  \
        ::apollo::common::util::ParentTraceCount(_apollo_out_hdr);                 \
    auto& _apollo_ptw_out = ::apollo::common::util::PerfTraceWriter::Instance();     \
    if (_apollo_ptw_out.IsActive()) {                                                \
      _apollo_ptw_out.EmitOutputBoundary(                                            \
          (MODULE), (PHASE), _apollo_out_tid, static_cast<uint32_t>(PROC_ID),        \
          !!(TRACE_VALID), (CHANNEL), static_cast<uint32_t>(OUTPUT_SEQ),             \
          static_cast<uint64_t>(DATA_TS_NS), static_cast<uint32_t>(OBJECT_COUNT),    \
          _apollo_out_pp, _apollo_out_pc);                                           \
    }                                                                                \
  } while (0)

#define APOLLO_TRACE_OUTPUT_HDR_AT(MONO_NS, MODULE, PHASE, HEADER, PROC_ID,         \
                                   TRACE_VALID, CHANNEL, OUTPUT_SEQ, DATA_TS_NS,    \
                                   OBJECT_COUNT)                                   \
  do {                                                                               \
    const auto& _apollo_out_hdr_at = (HEADER);                                       \
    const uint64_t _apollo_out_tid_at =                                              \
        _apollo_out_hdr_at.has_trace_id()                                            \
            ? static_cast<uint64_t>(_apollo_out_hdr_at.trace_id())                  \
            : 0ULL;                                                                  \
    const uint64_t _apollo_out_pp_at =                                               \
        ::apollo::common::util::PrimaryParentTraceId(_apollo_out_hdr_at);          \
    const uint32_t _apollo_out_pc_at =                                               \
        ::apollo::common::util::ParentTraceCount(_apollo_out_hdr_at);              \
    auto& _apollo_ptw_out_at =                                                       \
        ::apollo::common::util::PerfTraceWriter::Instance();                         \
    if (_apollo_ptw_out_at.IsActive()) {                                             \
      _apollo_ptw_out_at.EmitOutputBoundaryAt(                                       \
          (MODULE), (PHASE), _apollo_out_tid_at, static_cast<uint32_t>(PROC_ID),     \
          !!(TRACE_VALID), (CHANNEL), static_cast<uint32_t>(OUTPUT_SEQ),             \
          static_cast<uint64_t>(DATA_TS_NS), static_cast<uint32_t>(OBJECT_COUNT),    \
          static_cast<uint64_t>(MONO_NS), _apollo_out_pp_at, _apollo_out_pc_at);    \
    }                                                                                \
  } while (0)
