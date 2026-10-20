/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 */

#pragma once

#include <future>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include "cyber/base/bounded_queue.h"
#include "cyber/common/log.h"
#include "modules/prediction/common/prediction_trace_context.h"
#include "modules/prediction/common/prediction_thread_trace.h"

namespace apollo {
namespace prediction {

class BaseThreadPool {
 public:
  BaseThreadPool(int thread_num, int next_thread_pool_level);

  void Stop();

  ~BaseThreadPool();

  template <typename InputIter, typename F>
  void ForEach(InputIter begin, InputIter end, F f) {
    ForEach(begin, end, f, [](typename std::iterator_traits<InputIter>::reference) {
      auto context = MutablePredictionTraceContext();
      // Generic nested ForEach creates child tasks, not aliases of the parent.
      context.task_id = 0;
      return context;
    });
  }

  template <typename InputIter, typename F, typename Metadata>
  void ForEach(InputIter begin, InputIter end, F f, Metadata metadata) {
    if (MutablePredictionTraceContext().session_token == 0) {
      std::vector<std::future<void>> futures;
      for (auto iter = begin; iter != end; ++iter) {
        futures.emplace_back(this->Post([&, iter] { f(*iter); }));
      }
      for (auto& future : futures) {
        if (future.valid()) {
          PredictionThreadTraceInvoke(PredictionThreadTracePoint::kFuture,
              [&] { future.get(); });
        }
        else { AERROR << "Future is invalid."; }
      }
      return;
    }
    std::vector<std::future<void>> futures;
    std::vector<PredictionTraceContext> contexts;
    for (auto iter = begin; iter != end; ++iter) {
      auto& elem = *iter;
      auto context = metadata(elem);
      if (context.session_token != 0 && context.task_id == 0) {
        context.task_id = NextPredictionWorkerId();
      }
      context.pool_level = pool_level_;
      contexts.push_back(context);
      futures.emplace_back(this->Post([&, iter] { f(*iter); }, context));
    }
    for (size_t i = 0; i < futures.size(); ++i) {
      auto& future = futures[i];
      ScopedPredictionTraceContext context(contexts[i]);
      ScopedPredictionWorkerPhase get_phase("future_get_begin", "future_get_end");
      if (future.valid()) {
        PredictionThreadTraceInvoke(PredictionThreadTracePoint::kFuture,
            [&] { future.get(); });
      } else {
        get_phase.SetStatus("invalid");
        AERROR << "Future is invalid.";
      }
    }
  }

  template <typename FuncType>
  std::future<typename std::result_of<FuncType()>::type> Post(FuncType&& func) {
    auto context = MutablePredictionTraceContext();
    if (context.session_token != 0) { context.task_id = NextPredictionWorkerId(); }
    context.pool_level = pool_level_;
    return Post(std::forward<FuncType>(func), context);
  }

  template <typename FuncType>
  std::future<typename std::result_of<FuncType()>::type> Post(
      FuncType&& func, PredictionTraceContext context) {
    typedef typename std::result_of<FuncType()>::type ReturnType;
    typedef typename std::packaged_task<ReturnType()> TaskType;
    std::shared_ptr<TaskType> task =
        std::make_shared<TaskType>(std::move(func));
    std::future<ReturnType> returned_future = task->get_future();
    WritePredictionWorkerEvent(context, "task_submit_begin", "pending");
    if (stopped_) {
      WritePredictionWorkerEvent(context, "task_submit_end", "rejected_stopped");
      return std::future<ReturnType>();
    }
    const bool accepted = task_queue_.Enqueue([task, context]() {
      ScopedPredictionTraceContext restore(context);
      WritePredictionWorkerEvent(context, "task_begin");
      (*task)();
      // packaged_task has published the future before this observation.
      WritePredictionWorkerEvent(context, "task_end", "future_ready_upper_bound");
    });
    WritePredictionWorkerEvent(context, "task_submit_end",
                               accepted ? "accepted" : "rejected_queue");
    return returned_future;
  }

  static std::vector<int> THREAD_POOL_CAPACITY;

 private:
  std::vector<std::thread> workers_;
  apollo::cyber::base::BoundedQueue<std::function<void()>> task_queue_;
  std::atomic_bool stopped_;
  int pool_level_;
};

template <int LEVEL>
class LevelThreadPool : public BaseThreadPool {
 public:
  static LevelThreadPool* Instance() {
    static LevelThreadPool<LEVEL> pool;
    return &pool;
  }

 private:
  LevelThreadPool() : BaseThreadPool(THREAD_POOL_CAPACITY[LEVEL], LEVEL + 1) {
    ADEBUG << "Level = " << LEVEL
           << "; thread pool capacity = " << THREAD_POOL_CAPACITY[LEVEL];
  }
};

class PredictionThreadPool {
 public:
  static BaseThreadPool* Instance();

  template <typename InputIter, typename F, typename Metadata>
  static void ForEach(InputIter begin, InputIter end, F f, Metadata metadata) {
    Instance()->ForEach(begin, end, f, metadata);
  }

  static thread_local int s_thread_pool_level;

  template <typename InputIter, typename F>
  static void ForEach(InputIter begin, InputIter end, F f) {
    Instance()->ForEach(begin, end, f);
  }
};

}  // namespace prediction
}  // namespace apollo
