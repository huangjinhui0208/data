/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <atomic>
#include <cstdint>
#include <exception>

#include "modules/common/util/perf_trace_writer.h"

namespace apollo {
namespace prediction {

struct PredictionTraceContext {
  uint64_t trace_id = 0;
  uint64_t model_call_id = 0;
  uint64_t session_token = 0;
  uint64_t frame_instance_id = 0;
  uint64_t task_id = 0;
  uint64_t inference_id = 0;
  int64_t group_id = -1;
  int64_t obstacle_id = -1;
  int64_t lane_sequence_index = -1;
  int64_t pool_level = -1;
  int64_t input_rows = -1;
  int64_t input_cols = -1;
  const char* model_type = nullptr;
  const char* device_type = nullptr;
};

inline PredictionTraceContext& MutablePredictionTraceContext() {
  static thread_local PredictionTraceContext context;
  return context;
}

inline uint64_t NextPredictionModelCallId() {
  static std::atomic<uint64_t> next_id{1};
  return next_id.fetch_add(1, std::memory_order_relaxed);
}

inline uint64_t NextPredictionWorkerId() {
  static std::atomic<uint64_t> next_id{1};
  return next_id.fetch_add(1, std::memory_order_relaxed);
}

inline void WritePredictionWorkerEvent(const PredictionTraceContext& context,
                                       const char* phase,
                                       const char* status = "ok") {
  if (context.session_token == 0) { return; }
  apollo::common::util::PredictionWorkerEvent row;
  row.trace_id = context.trace_id;
  row.frame_instance_id = context.frame_instance_id;
  row.task_id = context.task_id;
  row.group_id = context.group_id;
  row.obstacle_id = context.obstacle_id;
  row.model_call_id = context.model_call_id;
  row.inference_id = context.inference_id;
  row.lane_sequence_index = context.lane_sequence_index;
  row.pool_level = context.pool_level;
  row.input_rows = context.input_rows;
  row.input_cols = context.input_cols;
  row.model_type = context.model_type;
  row.device_type = context.device_type;
  row.phase = phase;
  row.status = status;
  apollo::common::util::PerfTraceWriter::Instance().WritePredictionWorkerEvent(
      row, context.session_token);
}

class ScopedPredictionWorkerPhase {
 public:
  ScopedPredictionWorkerPhase(const char* begin, const char* end)
      : context_(MutablePredictionTraceContext()), end_(end),
        uncaught_(std::uncaught_exception()) {
    WritePredictionWorkerEvent(context_, begin);
  }
  ~ScopedPredictionWorkerPhase() {
    WritePredictionWorkerEvent(context_, end_,
        std::uncaught_exception() && !uncaught_ ? "exception" : status_);
  }
  void SetStatus(const char* status) { status_ = status; }
  ScopedPredictionWorkerPhase(const ScopedPredictionWorkerPhase&) = delete;
  ScopedPredictionWorkerPhase& operator=(const ScopedPredictionWorkerPhase&) = delete;
 private:
  PredictionTraceContext context_;
  const char* end_;
  bool uncaught_;
  const char* status_ = "ok";
};

class ScopedPredictionTraceContext {
 public:
  explicit ScopedPredictionTraceContext(const PredictionTraceContext& context)
      : previous_(MutablePredictionTraceContext()) {
    MutablePredictionTraceContext() = context;
  }
  ScopedPredictionTraceContext(uint64_t trace_id, uint64_t model_call_id)
      : previous_(MutablePredictionTraceContext()) {
    auto& context = MutablePredictionTraceContext();
    context.trace_id = trace_id;
    context.model_call_id = model_call_id;
  }

  ~ScopedPredictionTraceContext() {
    MutablePredictionTraceContext() = previous_;
  }

  ScopedPredictionTraceContext(const ScopedPredictionTraceContext&) = delete;
  ScopedPredictionTraceContext& operator=(
      const ScopedPredictionTraceContext&) = delete;

 private:
  PredictionTraceContext previous_;
};

class ScopedPredictionTracePhase {
 public:
  ScopedPredictionTracePhase(const char* enter_phase, const char* exit_phase)
      : exit_phase_(exit_phase), context_(MutablePredictionTraceContext()) {
    WritePredictionWorkerEvent(context_, enter_phase);
    if (context_.trace_id != 0 && context_.model_call_id != 0) {
      APOLLO_TRACE_EVENT("prediction", enter_phase, context_.trace_id,
                         context_.model_call_id);
    }
  }

  ~ScopedPredictionTracePhase() {
    WritePredictionWorkerEvent(context_, exit_phase_,
        std::uncaught_exception() ? "exception" : "ok");
    if (context_.trace_id != 0 && context_.model_call_id != 0) {
      APOLLO_TRACE_EVENT("prediction", exit_phase_, context_.trace_id,
                         context_.model_call_id);
    }
  }

  ScopedPredictionTracePhase(const ScopedPredictionTracePhase&) = delete;
  ScopedPredictionTracePhase& operator=(
      const ScopedPredictionTracePhase&) = delete;

 private:
  const char* exit_phase_;
  PredictionTraceContext context_;
};

// Use a fresh call id for a repeated inference nested inside one evaluator
// call (for example, one forward pass per lane sequence). The original frame
// trace id is retained while the outer evaluator context is restored on exit.
class ScopedPredictionTraceSubphase {
 public:
  ScopedPredictionTraceSubphase(const char* enter_phase,
                                const char* exit_phase)
      : context_(MutablePredictionTraceContext().trace_id,
                 NextPredictionModelCallId()),
        phase_(enter_phase, exit_phase) {}

  ScopedPredictionTraceSubphase(const ScopedPredictionTraceSubphase&) = delete;
  ScopedPredictionTraceSubphase& operator=(
      const ScopedPredictionTraceSubphase&) = delete;

 private:
  ScopedPredictionTraceContext context_;
  ScopedPredictionTracePhase phase_;
};

}  // namespace prediction
}  // namespace apollo
