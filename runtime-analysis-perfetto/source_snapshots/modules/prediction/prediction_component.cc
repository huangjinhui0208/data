/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/prediction_component.h"
#include "modules/prediction/common/prediction_thread_trace.h"

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/perf_trace_writer.h"

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/offline_features.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/prediction/scenario/scenario_manager.h"
#include "modules/common/util/data_extraction.h"
#include <sys/syscall.h>
#include <unistd.h>

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::cyber::Clock;
using apollo::perception::PerceptionObstacles;
using apollo::perception::PerceptionWaste;
using apollo::planning::ADCTrajectory;

PredictionComponent::~PredictionComponent() {}

std::string PredictionComponent::Name() const {
  return FLAGS_prediction_module_name;
}

void PredictionComponent::OfflineProcessFeatureProtoFile(
    const std::string& features_proto_file_name) {
  auto obstacles_container_ptr =
      container_manager_->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  obstacles_container_ptr->Clear();
  Features features;
  apollo::cyber::common::GetProtoFromBinaryFile(features_proto_file_name,
                                                &features);
  for (const Feature& feature : features.feature()) {
    obstacles_container_ptr->InsertFeatureProto(feature);
    Obstacle* obstacle_ptr = obstacles_container_ptr->GetObstacle(feature.id());
    if (!obstacle_ptr) {
      continue;
    }
    evaluator_manager_->EvaluateObstacle(obstacle_ptr, obstacles_container_ptr);
  }
}

bool PredictionComponent::Init() {
  InitializePredictionThreadTrace();
  component_start_time_ = Clock::NowInSeconds();

  AINFO << "[PERF_DIAGNOSTIC_LOGS] prediction="
        << (FLAGS_enable_prediction_perf_diagnostic_logs ? "ENABLED"
                                                         : "DISABLED")
        << " flag=enable_prediction_perf_diagnostic_logs";

  container_manager_ = std::make_shared<ContainerManager>();
  evaluator_manager_.reset(new EvaluatorManager());
  predictor_manager_.reset(new PredictorManager());
  scenario_manager_.reset(new ScenarioManager());

  PredictionConf prediction_conf;
  if (!ComponentBase::GetProtoConfig(&prediction_conf)) {
    AERROR << "Unable to load prediction conf file: "
           << ComponentBase::ConfigFilePath();
    return false;
  }
  ADEBUG << "Prediction config file is loaded into: "
         << prediction_conf.ShortDebugString();

  if (!MessageProcess::Init(container_manager_.get(), evaluator_manager_.get(),
                            predictor_manager_.get(), prediction_conf)) {
    return false;
  }

  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      prediction_conf.topic_conf().planning_trajectory_topic(), nullptr);

  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          prediction_conf.topic_conf().localization_topic(), nullptr);

  storytelling_reader_ = node_->CreateReader<storytelling::Stories>(
      prediction_conf.topic_conf().storytelling_topic(), nullptr);

  prediction_writer_ = node_->CreateWriter<PredictionObstacles>(
      prediction_conf.topic_conf().prediction_topic());

  container_writer_ = node_->CreateWriter<SubmoduleOutput>(
      prediction_conf.topic_conf().container_topic_name());

  adc_container_writer_ = node_->CreateWriter<ADCTrajectoryContainer>(
      prediction_conf.topic_conf().adccontainer_topic_name());

  perception_obstacles_writer_ = node_->CreateWriter<PerceptionObstacles>(
      prediction_conf.topic_conf().perception_obstacles_topic_name());

  return true;
}

bool PredictionComponent::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  AINFO << "[PRED_TID] input_perception_seq="
        << perception_obstacles->header().sequence_num()
        << " tid=" << syscall(SYS_gettid);
  if (FLAGS_use_lego) {
    return ContainerSubmoduleProcess(perception_obstacles);
  }
  return PredictionEndToEndProc(perception_obstacles);
}

bool PredictionComponent::ContainerSubmoduleProcess(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  constexpr static size_t kHistorySize = 10;
  const auto frame_start_time = Clock::Now();
  localization_reader_->Observe();
  auto ptr_localization_msg = localization_reader_->GetLatestObserved();
  if (ptr_localization_msg == nullptr) {
    AERROR << "Prediction: cannot receive any localization message.";
    return false;
  }
  MessageProcess::OnLocalization(container_manager_.get(),
                                 *ptr_localization_msg);

  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    MessageProcess::OnPlanning(container_manager_.get(), *ptr_trajectory_msg);
  }

  storytelling_reader_->Observe();
  auto ptr_storytelling_msg = storytelling_reader_->GetLatestObserved();
  if (ptr_storytelling_msg != nullptr) {
    MessageProcess::OnStoryTelling(container_manager_.get(),
                                   *ptr_storytelling_msg);
  }

  MessageProcess::ContainerProcess(container_manager_, *perception_obstacles,
                                   scenario_manager_.get());

  auto obstacles_container_ptr =
      container_manager_->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(obstacles_container_ptr);

  auto adc_trajectory_container_ptr =
      container_manager_->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK_NOTNULL(adc_trajectory_container_ptr);

  SubmoduleOutput submodule_output =
      obstacles_container_ptr->GetSubmoduleOutput(kHistorySize,
                                                  frame_start_time);
  submodule_output.set_curr_scenario(scenario_manager_->scenario());
  container_writer_->Write(submodule_output);
  adc_container_writer_->Write(*adc_trajectory_container_ptr);
  perception_obstacles_writer_->Write(*perception_obstacles);
  return true;
}

bool PredictionComponent::PredictionEndToEndProc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  PreparePredictionThreadTrace("prediction_callback");

  const auto& perc_hdr = perception_obstacles->header();
  const bool trace_edge_valid = perc_hdr.trace_id() != 0;
  APOLLO_TRACE_INPUT_HDR(
      "prediction", "proc_enter", perc_hdr, 0, trace_edge_valid,
      "/apollo/perception/obstacles", perc_hdr.sequence_num(),
      perc_hdr.lidar_timestamp(),
      static_cast<uint32_t>(perception_obstacles->perception_obstacle_size()));
  ::apollo::common::util::ScopedPerfTraceEvent proc_exit_trace(
      "prediction", "proc_exit",
      perc_hdr.has_trace_id() ? static_cast<uint64_t>(perc_hdr.trace_id())
                              : 0ULL,
      0);

  if (FLAGS_prediction_test_mode &&
      (Clock::NowInSeconds() - component_start_time_ >
       FLAGS_prediction_test_duration)) {
    ADEBUG << "Prediction finished running in test mode";
  }

  if (FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Relative map is empty.";
    return false;
  }

  frame_start_time_ = Clock::NowInSeconds();
  auto end_time1 = std::chrono::system_clock::now();

  APOLLO_TRACE_EVENT("prediction", "container_enter",
                     perception_obstacles->header().trace_id(), 0);

  localization_reader_->Observe();
  auto ptr_localization_msg = localization_reader_->GetLatestObserved();
  if (ptr_localization_msg == nullptr) {
    AERROR << "Prediction: cannot receive any localization message.";
    return false;
  }
  MessageProcess::OnLocalization(container_manager_.get(),
                                 *ptr_localization_msg);
  auto end_time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time2 - end_time1;
  ADEBUG << "Time for updating PoseContainer: " << diff.count() * 1000
         << " msec.";

  storytelling_reader_->Observe();
  auto ptr_storytelling_msg = storytelling_reader_->GetLatestObserved();
  if (ptr_storytelling_msg != nullptr) {
    MessageProcess::OnStoryTelling(container_manager_.get(),
                                   *ptr_storytelling_msg);
  }

  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    MessageProcess::OnPlanning(container_manager_.get(), *ptr_trajectory_msg);
  }
  APOLLO_TRACE_EVENT("prediction", "container_exit",
                     perception_obstacles->header().trace_id(), 0);
  auto end_time3 = std::chrono::system_clock::now();
  diff = end_time3 - end_time2;
  ADEBUG << "Time for updating ADCTrajectoryContainer: " << diff.count() * 1000
         << " msec.";

  auto perception_msg = *perception_obstacles;
  PredictionObstacles prediction_obstacles;
  APOLLO_TRACE_EVENT("prediction", "perception_enter",
                     perception_msg.header().trace_id(), 0);
  MessageProcess::OnPerception(
      perception_msg, container_manager_, evaluator_manager_.get(),
      predictor_manager_.get(), scenario_manager_.get(), &prediction_obstacles);
  APOLLO_TRACE_EVENT("prediction", "perception_exit",
                     perception_msg.header().trace_id(), 0);
  auto end_time4 = std::chrono::system_clock::now();
  diff = end_time4 - end_time3;
  ADEBUG << "Time for updating PerceptionContainer: " << diff.count() * 1000
         << " msec.";

  prediction_obstacles.set_start_timestamp(frame_start_time_);
  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
  prediction_obstacles.mutable_header()->set_lidar_timestamp(
      perception_msg.header().lidar_timestamp());
  prediction_obstacles.mutable_header()->set_camera_timestamp(
      perception_msg.header().camera_timestamp());
  prediction_obstacles.mutable_header()->set_radar_timestamp(
      perception_msg.header().radar_timestamp());
  for (const PerceptionWaste& perception_waste :
      perception_msg.perception_waste()) {
    prediction_obstacles.add_perception_waste()->CopyFrom(
      perception_waste);
  }

  prediction_obstacles.set_perception_error_code(perception_msg.error_code());

  // Minimal prediction input/output logging for analysis
  if (FLAGS_enable_prediction_perf_diagnostic_logs) {
    for (const auto &p : perception_msg.perception_obstacle()) {
      AINFO << "[PREDICTION_INPUT_OBS]"
            << " time=" << frame_start_time_
            << " trace_id=" << perception_msg.header().trace_id()
            << " input_seq=" << perception_msg.header().sequence_num()
            << " perception_header_time="
            << perception_msg.header().timestamp_sec()
            << " id=" << p.id()
            << " type=" << p.type()
            << " pos_x=" << p.position().x()
            << " pos_y=" << p.position().y()
            << " theta=" << p.theta()
            << " vel_x=" << p.velocity().x()
            << " vel_y=" << p.velocity().y();
    }

    for (const auto &pred_obs : prediction_obstacles.prediction_obstacle()) {
      const auto &per = pred_obs.perception_obstacle();
      const auto &traj0 = (pred_obs.trajectory_size() > 0)
                              ? pred_obs.trajectory(0)
                              : prediction::Trajectory();
      const int traj0_point_count = traj0.trajectory_point_size();
      double traj0_first_t = 0.0, traj0_last_t = 0.0;
      double traj0_first_x = 0.0, traj0_first_y = 0.0;
      double traj0_last_x = 0.0, traj0_last_y = 0.0;
      if (traj0_point_count > 0) {
        traj0_first_t = traj0.trajectory_point(0).relative_time();
        traj0_last_t =
            traj0.trajectory_point(traj0_point_count - 1).relative_time();
        traj0_first_x = traj0.trajectory_point(0).path_point().x();
        traj0_first_y = traj0.trajectory_point(0).path_point().y();
        traj0_last_x =
            traj0.trajectory_point(traj0_point_count - 1).path_point().x();
        traj0_last_y =
            traj0.trajectory_point(traj0_point_count - 1).path_point().y();
      }
      AINFO << "[PREDICTION_OUTPUT_OBS]"
            << " time=" << frame_start_time_
            << " trace_id=" << perception_msg.header().trace_id()
            << " input_seq=" << perception_msg.header().sequence_num()
            << " prediction_header_time="
            << prediction_obstacles.header().timestamp_sec()
            << " start_timestamp=" << prediction_obstacles.start_timestamp()
            << " end_timestamp=" << prediction_obstacles.end_timestamp()
            << " id=" << per.id()
            << " type=" << per.type()
            << " traj_count=" << pred_obs.trajectory_size()
            << " pred_is_static=" << static_cast<int>(pred_obs.is_static())
            << " pred_has_is_static="
            << static_cast<int>(pred_obs.has_is_static())
            << " traj0_probability="
            << (traj0_point_count > 0 ? traj0.probability() : 0.0)
            << " traj0_point_count=" << traj0_point_count
            << " traj0_first_t=" << traj0_first_t
            << " traj0_last_t=" << traj0_last_t
            << " traj0_first_x=" << traj0_first_x
            << " traj0_first_y=" << traj0_first_y
            << " traj0_last_x=" << traj0_last_x
            << " traj0_last_y=" << traj0_last_y;
    }
  }

  if (FLAGS_prediction_test_mode) {
    for (auto const& prediction_obstacle :
         prediction_obstacles.prediction_obstacle()) {
      for (auto const& trajectory : prediction_obstacle.trajectory()) {
        for (auto const& trajectory_point : trajectory.trajectory_point()) {
          if (!ValidationChecker::ValidTrajectoryPoint(trajectory_point)) {
            AERROR << "Invalid trajectory point ["
                   << trajectory_point.ShortDebugString() << "]";
            break;
          }
        }
      }
    }
  }

  auto end_time5 = std::chrono::system_clock::now();
  diff = end_time5 - end_time1;
  ADEBUG << "End to end time elapsed: " << diff.count() * 1000 << " msec.";
  if (FLAGS_enable_prediction_perf_diagnostic_logs) {
    AINFO << "Prediction end to end time: " << diff.count() * 1000 << " ms.";
  }

  // Publish output
  common::util::FillHeader(node_->Name(), &prediction_obstacles);
  // Keep trace chain after FillHeader to avoid overwrite.
  if (perception_msg.header().has_trace_id()) {
    prediction_obstacles.mutable_header()->set_trace_id(
        perception_msg.header().trace_id());
  }
  for (const auto parent_trace_id : perception_msg.header().parent_trace_ids()) {
    prediction_obstacles.mutable_header()->add_parent_trace_ids(parent_trace_id);
  }

  {
    const uint64_t out_tid = prediction_obstacles.header().trace_id();
    const uint64_t publish_enter_ns = ::apollo::common::util::MonoNowNs();
    prediction_writer_->Write(prediction_obstacles);
    const uint64_t publish_done_ns = ::apollo::common::util::MonoNowNs();
    APOLLO_TRACE_EVENT_AT(publish_enter_ns, "prediction", "writer_enter",
                          out_tid, 0);
    APOLLO_TRACE_EVENT_AT(publish_done_ns, "prediction", "writer_done", out_tid,
                          0);
    APOLLO_TRACE_OUTPUT_HDR_AT(
        publish_enter_ns, "prediction", "output_pub",
        prediction_obstacles.header(), 0,
        prediction_obstacles.header().trace_id() != 0, "/apollo/prediction",
        prediction_obstacles.header().sequence_num(),
        prediction_obstacles.header().lidar_timestamp(),
        static_cast<uint32_t>(prediction_obstacles.prediction_obstacle_size()));
  }

  if (FLAGS_prediction_eval_mode) {
    for (auto const& prediction_obstacle :
          prediction_obstacles.prediction_obstacle()) {
      AINFO << "prediction_eval_log (info): "
            << std::to_string(prediction_obstacle.timestamp()) << ", "
            << prediction_obstacle.perception_obstacle().id() << ", "
            << prediction_obstacle.priority().priority() << ", "
            << prediction_obstacle.perception_obstacle().type() << ", "
            << std::to_string(
              prediction_obstacle.perception_obstacle().position().x())
            << ", " << std::to_string(
              prediction_obstacle.perception_obstacle().position().y());

      if (prediction_obstacle.trajectory_size() != 0) {
        for (auto const& trajectory : prediction_obstacle.trajectory()) {
          for (auto const& trajectory_point : trajectory.trajectory_point()) {
            AINFO << "prediction_eval_log (traj): "
                  << std::to_string(prediction_obstacle.timestamp()) << ", "
                  << prediction_obstacle.perception_obstacle().id() << ", "
                  << std::to_string(trajectory_point.path_point().x()) << ", "
                  << std::to_string(trajectory_point.path_point().y());
          }
          break;
        }
      }
    }
  }

  return true;
}

}  // namespace prediction
}  // namespace apollo
