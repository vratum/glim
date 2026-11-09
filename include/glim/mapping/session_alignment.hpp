#pragma once
#include <Eigen/Geometry>
#include <vector>

namespace glim {
// A physical restart is connected by measured geometry, never by an IMU gap.
struct SessionConstraint {
  int reference_id;
  int pending_id;
  Eigen::Isometry3d T_reference_source = Eigen::Isometry3d::Identity();
};
struct SessionAlignment {
  bool accepted = false;
  Eigen::Isometry3d T_map_session = Eigen::Isometry3d::Identity();
  std::vector<SessionConstraint> constraints;
};
}  // namespace glim
