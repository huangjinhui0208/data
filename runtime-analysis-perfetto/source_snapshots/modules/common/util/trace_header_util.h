#pragma once

#include "modules/common_msgs/basic_msgs/header.pb.h"

namespace apollo {
namespace common {
namespace util {

// Copy trace chain fields from src to dst after FillHeader.
// Does not modify module_name, timestamp_sec, or sequence_num.
inline void CopyTraceHeaderFields(const Header& src, Header* dst) {
  if (dst == nullptr) {
    return;
  }
  if (src.has_trace_id()) {
    dst->set_trace_id(src.trace_id());
  }
  dst->clear_parent_trace_ids();
  for (int i = 0; i < src.parent_trace_ids_size(); ++i) {
    dst->add_parent_trace_ids(src.parent_trace_ids(i));
  }
  if (src.has_lidar_timestamp()) {
    dst->set_lidar_timestamp(src.lidar_timestamp());
  }
  if (src.has_camera_timestamp()) {
    dst->set_camera_timestamp(src.camera_timestamp());
  }
  if (src.has_radar_timestamp()) {
    dst->set_radar_timestamp(src.radar_timestamp());
  }
}

// First parent in header (fusion publishes main sensor first).
inline uint64_t PrimaryParentTraceId(const Header& h) {
  if (h.parent_trace_ids_size() == 0) {
    return 0;
  }
  return h.parent_trace_ids(0);
}

inline uint32_t ParentTraceCount(const Header& h) {
  return static_cast<uint32_t>(h.parent_trace_ids_size());
}

}  // namespace util
}  // namespace common
}  // namespace apollo
