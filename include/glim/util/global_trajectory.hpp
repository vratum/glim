#pragma once

#include <glim/mapping/sub_map.hpp>

namespace glim {

// Use the optimized submap anchor for the entire history, including old frames.
// This is shared by the live path and the saved trajectory.
template <typename Visitor>
void visit_global_trajectory(const SubMap& submap, Visitor&& visitor) {
  if (submap.frames.empty()) {
    return;
  }
  const Eigen::Isometry3d T_world_odom =
    submap.T_world_origin * submap.T_origin_endpoint_L * submap.frames.front()->T_world_imu.inverse();
  for (const auto& frame : submap.frames) {
    const Eigen::Isometry3d T_world_imu = T_world_odom * frame->T_world_imu;
    const Eigen::Isometry3d T_world_lidar = T_world_imu * frame->T_lidar_imu.inverse();
    visitor(frame->stamp, T_world_imu, T_world_lidar);
  }
}

}  // namespace glim
