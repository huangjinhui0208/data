/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/multi_sensor_fusion/multi_sensor_fusion_component.h"

#include <algorithm>
#include <atomic>
#include <string>
#include <vector>

#include "cyber/profiler/profiler.h"
#include "cyber/time/clock.h"
#include "modules/common/util/perf_trace_writer.h"
#include "modules/common/util/trace_header_util.h"
#include "modules/common/util/trace_id.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/onboard/common_flags/common_flags.h"
#include "modules/perception/common/onboard/msg_serializer/msg_serializer.h"

namespace apollo {
namespace perception {
namespace fusion {

namespace {

void VerifyFusionParentSets(
    uint64_t fusion_trace_id,
    const apollo::common::Header& header,
    const std::vector<::apollo::common::util::FusionInputRow>& rows) {
  std::vector<uint64_t> header_parents;
  header_parents.reserve(header.parent_trace_ids_size());
  for (int i = 0; i < header.parent_trace_ids_size(); ++i) {
    header_parents.push_back(header.parent_trace_ids(i));
  }
  std::vector<uint64_t> audit_parents;
  audit_parents.reserve(rows.size());
  for (const auto& row : rows) {
    audit_parents.push_back(row.parent_trace_id);
  }
  auto sorted_unique = [](std::vector<uint64_t> v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
  };
  const auto hs = sorted_unique(header_parents);
  const auto as = sorted_unique(audit_parents);
  if (hs != as) {
    AWARN << "[trace] fusion parent mismatch fusion_trace_id=" << fusion_trace_id
          << " header_parents=" << hs.size()
          << " fusion_inputs_parents=" << as.size();
  }
}

std::string InferSensorKind(const std::string& sid) {
  if (sid.find("Radar") != std::string::npos ||
      sid.find("radar") != std::string::npos) {
    return "radar";
  }
  if (sid.find("camera") != std::string::npos ||
      sid.find("Camera") != std::string::npos ||
      sid.find("traffic") != std::string::npos ||
      sid.find("Traffic") != std::string::npos) {
    return "camera";
  }
  if (sid.find("front_") != std::string::npos ||
      sid.find("_6mm") != std::string::npos ||
      sid.find("_12mm") != std::string::npos ||
      sid.find("6mm") != std::string::npos ||
      sid.find("12mm") != std::string::npos) {
    return "camera";
  }
  return "lidar";
}

}  // namespace

uint32_t MultiSensorFusionComponent::s_seq_num_ = 0;
std::mutex MultiSensorFusionComponent::s_mutex_;
std::atomic<uint32_t> MultiSensorFusionComponent::s_fusion_local_seq_{0};

bool MultiSensorFusionComponent::Init() {
  FusionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  AINFO << "Fusion Component Configs: " << comp_config.DebugString();

  // to load component configs
  object_in_roi_check_ = comp_config.object_in_roi_check();
  radius_for_roi_object_check_ = comp_config.radius_for_roi_object_check();

  // init algorithm plugin
  ACHECK(InitAlgorithmPlugin(comp_config))
      << "Failed to init algorithm plugin.";

  fusion_output_obstacles_channel_ =
      comp_config.output_obstacles_channel_name();

  writer_ = node_->CreateWriter<PerceptionObstacles>(
      fusion_output_obstacles_channel_);
  inner_writer_ = node_->CreateWriter<SensorFrameMessage>(
      comp_config.output_viz_fused_content_channel_name());

  algorithm::SensorManager::Instance()->Init();
  return true;
}

bool MultiSensorFusionComponent::Proc(
    const std::shared_ptr<SensorFrameMessage>& message) {
  PERF_FUNCTION()
  if (!message) {
    return false;
  }
  const uint64_t trace_id = message->trace_id_;
  const bool trace_valid = (trace_id != 0);
  const uint64_t lidar_timestamp_ns = message->lidar_timestamp_;
  static std::atomic<uint32_t> s_fusion_proc_seq{0};
  const uint32_t proc_id =
      s_fusion_proc_seq.fetch_add(1, std::memory_order_relaxed) + 1u;

  const std::string fusion_input_channel =
      std::string("/perception/fusion/input/") + message->sensor_id_;
  APOLLO_TRACE_INPUT(
      "perception.multi_sensor_fusion", "proc_enter", trace_id, proc_id,
      trace_valid, fusion_input_channel.c_str(), 0, lidar_timestamp_ns, 0);
  ::apollo::common::util::ScopedPerfTraceEvent proc_exit_trace(
      "perception.multi_sensor_fusion", "proc_exit", trace_id, proc_id);

  if (message->process_stage_ == onboard::ProcessStage::SENSOR_FUSION) {
    return true;
  }
  UpdateActiveTraces(message->trace_id_, message->sensor_id_);

  std::shared_ptr<PerceptionObstacles> out_message(new (std::nothrow)
                                                       PerceptionObstacles);
  std::shared_ptr<SensorFrameMessage> viz_message(new (std::nothrow)
                                                      SensorFrameMessage);
  std::vector<::apollo::common::util::FusionInputRow> fusion_audit_rows;
  bool status =
      InternalProc(message, out_message, viz_message, &fusion_audit_rows);

  if (status) {
    bool is_main_sensor =
        algorithm::SensorManager::Instance()->IsMainSensor(message->sensor_id_);
    if (is_main_sensor) {
      const uint64_t publish_enter_ns = ::apollo::common::util::MonoNowNs();
      writer_->Write(out_message);

      const auto& hdr = out_message->header();
      const uint64_t out_tid =
          hdr.has_trace_id() ? static_cast<uint64_t>(hdr.trace_id()) : 0ULL;

      ::apollo::common::util::RecordFusionInputs(out_tid, fusion_audit_rows);
      // AINFO << "[FUSION_OBS_FRAME]"
      //         << " seq=" << out_message->header().sequence_num()
      //         << " header_time=" << std::setprecision(16)
      //         << out_message->header().timestamp_sec()
      //         << " lidar_timestamp=" << out_message->header().lidar_timestamp()
      //         << " obstacle_count=" << out_message->perception_obstacle_size();
      AINFO << "[FUSION_OBS_FRAME]"
            << " seq=" << out_message->header().sequence_num()
            << " trace_id=" << out_tid
            << " header_time=" << std::setprecision(16)
            << out_message->header().timestamp_sec()
            << " lidar_timestamp=" << out_message->header().lidar_timestamp()
            << " obstacle_count=" << out_message->perception_obstacle_size();

        // for (const auto& obs : out_message->perception_obstacle()) {
        //   AINFO << "[FUSION_OBS]"
        //         << " seq=" << out_message->header().sequence_num()
        //         << " header_time=" << std::setprecision(16)
        //         << out_message->header().timestamp_sec()
        //         << " obs_time=" << obs.timestamp()
        //         << " id=" << obs.id()
        //         << " type=" << obs.type()
        //         << " sub_type=" << obs.sub_type()
        //         << " length=" << obs.length()
        //         << " width=" << obs.width()
        //         << " height=" << obs.height()
        //         << " tracking_time=" << obs.tracking_time()
        //         << " confidence=" << obs.confidence();
        // }
        for (const auto& obs : out_message->perception_obstacle()) {
          const bool has_pos = obs.has_position();
          const bool has_vel = obs.has_velocity();
          const bool has_anchor = obs.has_anchor_point();

          const double pos_x = has_pos ? obs.position().x()
                                      : std::numeric_limits<double>::quiet_NaN();
          const double pos_y = has_pos ? obs.position().y()
                                      : std::numeric_limits<double>::quiet_NaN();
          const double pos_z = has_pos ? obs.position().z()
                                      : std::numeric_limits<double>::quiet_NaN();

          const double vel_x = has_vel ? obs.velocity().x()
                                      : std::numeric_limits<double>::quiet_NaN();
          const double vel_y = has_vel ? obs.velocity().y()
                                      : std::numeric_limits<double>::quiet_NaN();
          const double vel_z = has_vel ? obs.velocity().z()
                                      : std::numeric_limits<double>::quiet_NaN();
          const double speed = has_vel ? std::hypot(vel_x, vel_y)
                                      : std::numeric_limits<double>::quiet_NaN();

          double poly_min_x = std::numeric_limits<double>::quiet_NaN();
          double poly_max_x = std::numeric_limits<double>::quiet_NaN();
          double poly_min_y = std::numeric_limits<double>::quiet_NaN();
          double poly_max_y = std::numeric_limits<double>::quiet_NaN();

          if (obs.polygon_point_size() > 0) {
            poly_min_x = std::numeric_limits<double>::infinity();
            poly_max_x = -std::numeric_limits<double>::infinity();
            poly_min_y = std::numeric_limits<double>::infinity();
            poly_max_y = -std::numeric_limits<double>::infinity();

            for (const auto& p : obs.polygon_point()) {
              poly_min_x = std::min(poly_min_x, p.x());
              poly_max_x = std::max(poly_max_x, p.x());
              poly_min_y = std::min(poly_min_y, p.y());
              poly_max_y = std::max(poly_max_y, p.y());
            }
          }

          const double anchor_x = has_anchor ? obs.anchor_point().x()
                                            : std::numeric_limits<double>::quiet_NaN();
          const double anchor_y = has_anchor ? obs.anchor_point().y()
                                            : std::numeric_limits<double>::quiet_NaN();
          const double anchor_z = has_anchor ? obs.anchor_point().z()
                                            : std::numeric_limits<double>::quiet_NaN();

          AINFO << "[FUSION_OBS]"
                << " seq=" << out_message->header().sequence_num()
                << " trace_id=" << out_tid
                << " header_time=" << std::setprecision(16)
                << out_message->header().timestamp_sec()
                << " obs_time=" << obs.timestamp()

                << " id=" << obs.id()
                << " type=" << obs.type()
                << " sub_type=" << obs.sub_type()

                << " pos_x=" << pos_x
                << " pos_y=" << pos_y
                << " pos_z=" << pos_z
                << " theta=" << obs.theta()

                << " vel_x=" << vel_x
                << " vel_y=" << vel_y
                << " vel_z=" << vel_z
                << " speed=" << speed

                << " has_anchor=" << has_anchor
                << " anchor_x=" << anchor_x
                << " anchor_y=" << anchor_y
                << " anchor_z=" << anchor_z

                << " length=" << obs.length()
                << " width=" << obs.width()
                << " height=" << obs.height()

                << " polygon_size=" << obs.polygon_point_size()
                << " poly_min_x=" << poly_min_x
                << " poly_max_x=" << poly_max_x
                << " poly_min_y=" << poly_min_y
                << " poly_max_y=" << poly_max_y

                << " tracking_time=" << obs.tracking_time()
                << " confidence=" << obs.confidence()
                << " measurement_count=" << obs.measurements_size();
      }
      AINFO << "Send fusion processing output message.";
      if (onboard::FLAGS_obs_enable_visualization) {
        inner_writer_->Write(viz_message);
      }

      APOLLO_TRACE_OUTPUT_HDR_AT(
          publish_enter_ns, "perception.multi_sensor_fusion", "output_pub", hdr,
          proc_id, out_tid != 0ULL, fusion_output_obstacles_channel_.c_str(),
          hdr.has_sequence_num() ? hdr.sequence_num() : 0u,
          hdr.has_lidar_timestamp() ? hdr.lidar_timestamp() : 0ULL,
          static_cast<uint32_t>(out_message->perception_obstacle_size()));
    } else {
      AINFO << "Fusion receive from " << message->sensor_id_
            << ". Skip because it is not the main sensor.";
    }
  }
  return status;
}

bool MultiSensorFusionComponent::InitAlgorithmPlugin(
    const FusionComponentConfig& config) {
  auto fusion_param = config.fusion_param();
  FusionInitOptions init_options;
  init_options.config_path = fusion_param.config_path();
  init_options.config_file = fusion_param.config_file();
  BaseFusionSystem* fusion =
      BaseFusionSystemRegisterer::GetInstanceByName(fusion_param.name());
  CHECK_NOTNULL(fusion);
  fusion_.reset(fusion);
  ACHECK(fusion_->Init(init_options)) << "Failed to init ProbabilisticFusion";

  if (onboard::FLAGS_obs_enable_hdmap_input && object_in_roi_check_) {
    hdmap_input_ = map::HDMapInput::Instance();
    ACHECK(hdmap_input_->Init()) << "Failed to init hdmap input.";
  }
  AINFO << "Init algorithm successfully, onboard fusion: "
        << fusion_param.name();
  return true;
}

bool MultiSensorFusionComponent::InternalProc(
    const std::shared_ptr<SensorFrameMessage const>& in_message,
    std::shared_ptr<PerceptionObstacles> out_message,
    std::shared_ptr<SensorFrameMessage> viz_message,
    std::vector<::apollo::common::util::FusionInputRow>* fusion_audit_rows) {
  if (fusion_audit_rows != nullptr) {
    fusion_audit_rows->clear();
  }

  const uint64_t trace_id = (in_message ? in_message->trace_id_ : 0);

  {
    std::unique_lock<std::mutex> lock(s_mutex_);
    s_seq_num_++;
  }

  PERF_BLOCK("fusion_process")
  const double timestamp = in_message->timestamp_;
  const uint64_t lidar_timestamp = in_message->lidar_timestamp_;
  std::vector<base::ObjectPtr> valid_objects;
  if (in_message->error_code_ != apollo::common::ErrorCode::OK) {
    if (!onboard::MsgSerializer::SerializeMsg(
            timestamp, lidar_timestamp, in_message->seq_num_, valid_objects,
            in_message->error_code_, out_message.get())) {
      AERROR << "Failed to gen PerceptionObstacles object.";
      return false;
    }
    if (in_message->trace_id_ != 0) {
      out_message->mutable_header()->set_trace_id(in_message->trace_id_);
    }
    if (onboard::FLAGS_obs_enable_visualization) {
      viz_message->process_stage_ = onboard::ProcessStage::SENSOR_FUSION;
      viz_message->error_code_ = in_message->error_code_;
    }
    AERROR << "Fusion receive message with error code, skip it.";
    return true;
  }
  base::FramePtr frame = in_message->frame_;
  frame->timestamp = in_message->timestamp_;

  std::vector<base::ObjectPtr> fused_objects;
  if (!fusion_->Fuse(frame, &fused_objects)) {
    AERROR << "Failed to call fusion plugin.";
    return false;
  }
  PERF_BLOCK_END

  bool is_main_sensor = algorithm::SensorManager::Instance()->IsMainSensor(
      in_message->sensor_id_);
  if (!is_main_sensor) {
    return true;
  }

  PERF_BLOCK("fusion_roi_check")
  Eigen::Matrix4d sensor2world_pose =
      in_message->frame_->sensor2world_pose.matrix();
  if (object_in_roi_check_ && onboard::FLAGS_obs_enable_hdmap_input) {
    base::HdmapStructPtr hdmap(new base::HdmapStruct());
    if (hdmap_input_) {
      base::PointD position;
      position.x = sensor2world_pose(0, 3);
      position.y = sensor2world_pose(1, 3);
      position.z = sensor2world_pose(2, 3);
      hdmap_input_->GetRoiHDMapStruct(position, radius_for_roi_object_check_,
                                      hdmap);
      valid_objects.assign(fused_objects.begin(), fused_objects.end());
    } else {
      valid_objects.assign(fused_objects.begin(), fused_objects.end());
    }
  } else {
    valid_objects.assign(fused_objects.begin(), fused_objects.end());
  }
  PERF_BLOCK_END

  PERF_BLOCK("fusion_serialize_message")
  if (onboard::FLAGS_obs_enable_visualization) {
    viz_message->timestamp_ = in_message->timestamp_;
    viz_message->seq_num_ = in_message->seq_num_;
    viz_message->frame_ = base::FramePool::Instance().Get();
    viz_message->frame_->sensor2world_pose =
        in_message->frame_->sensor2world_pose;
    viz_message->sensor_id_ = in_message->sensor_id_;
    viz_message->hdmap_ = in_message->hdmap_;
    viz_message->process_stage_ = onboard::ProcessStage::SENSOR_FUSION;
    viz_message->error_code_ = in_message->error_code_;
    viz_message->frame_->objects = fused_objects;
  }
  apollo::common::ErrorCode error_code = apollo::common::ErrorCode::OK;
  if (!onboard::MsgSerializer::SerializeMsg(timestamp, lidar_timestamp,
                                            in_message->seq_num_, valid_objects,
                                            error_code, out_message.get())) {
    AERROR << "Failed to gen PerceptionObstacles object.";
    return false;
  }

  const uint64_t trace_F = ::apollo::common::util::MakeTraceId(
      ::apollo::common::util::kTraceKindFusion, 0,
      s_fusion_local_seq_.fetch_add(1, std::memory_order_relaxed));
  out_message->mutable_header()->set_trace_id(trace_F);

  const auto active_entries = SnapshotActiveTracesEntries();
  std::vector<ActiveTrace> main_entries;
  std::vector<ActiveTrace> other_entries;
  main_entries.reserve(active_entries.size());
  other_entries.reserve(active_entries.size());
  for (const auto& e : active_entries) {
    if (algorithm::SensorManager::Instance()->IsMainSensor(e.sensor_id)) {
      main_entries.push_back(e);
    } else {
      other_entries.push_back(e);
    }
  }
  const auto add_parents = [&](const std::vector<ActiveTrace>& entries) {
    for (const auto& e : entries) {
      out_message->mutable_header()->add_parent_trace_ids(e.trace_id);
    }
  };
  add_parents(main_entries);
  add_parents(other_entries);

  if (fusion_audit_rows != nullptr && !active_entries.empty()) {
    const uint32_t obj_cnt = static_cast<uint32_t>(valid_objects.size());
    const uint64_t frame_ts =
        static_cast<uint64_t>(in_message->lidar_timestamp_);
    fusion_audit_rows->reserve(active_entries.size());
    for (const auto& e : active_entries) {
      ::apollo::common::util::FusionInputRow row;
      row.parent_trace_id = e.trace_id;
      row.sensor = e.sensor_id;
      row.sensor_kind = InferSensorKind(row.sensor);
      row.frame_time_ns = frame_ts;
      row.sensor_ts_ns = frame_ts;
      row.object_count = obj_cnt;
      row.is_main_sensor =
          algorithm::SensorManager::Instance()->IsMainSensor(row.sensor);
      fusion_audit_rows->push_back(row);
    }
    VerifyFusionParentSets(trace_F, out_message->header(), *fusion_audit_rows);
  }

  PERF_BLOCK_END

  const double cur_time = ::apollo::cyber::Clock::NowInSeconds();
  const double latency = (cur_time - timestamp) * 1e3;
  AINFO << std::setprecision(16) << "FRAME_STATISTICS:Obstacle:End:msg_time["
        << timestamp << "]:cur_time[" << cur_time << "]:cur_latency[" << latency
        << "]:obj_cnt[" << valid_objects.size() << "]";
  AINFO << "publish_number: " << valid_objects.size() << " obj";
  return true;
}

void MultiSensorFusionComponent::UpdateActiveTraces(
    uint64_t trace_id, const std::string& sensor_id) {
  const uint64_t now = ::apollo::common::util::MonoNowNs();
  std::lock_guard<std::mutex> lk(active_traces_mutex_);
  for (auto it = active_traces_.begin(); it != active_traces_.end();) {
    if (it->sensor_id == sensor_id) {
      it = active_traces_.erase(it);
    } else {
      ++it;
    }
  }
  if (trace_id != 0) {
    active_traces_.push_back({trace_id, now, sensor_id});
  }
  while (!active_traces_.empty() &&
         now - active_traces_.front().inject_mono_ns > active_window_ns_) {
    active_traces_.pop_front();
  }
}

std::vector<MultiSensorFusionComponent::ActiveTrace>
MultiSensorFusionComponent::SnapshotActiveTracesEntries() {
  std::lock_guard<std::mutex> lk(active_traces_mutex_);
  return std::vector<ActiveTrace>(active_traces_.begin(), active_traces_.end());
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
