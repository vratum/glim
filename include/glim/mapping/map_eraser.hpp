#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include <glim/mapping/sub_map.hpp>

namespace glim {

struct VerticalEraseResult {
  std::size_t erased_points = 0;
  std::vector<int> changed_submap_ids;
};

/// Remove every map point inside a vertical cylinder in world coordinates.
///
/// The cylinder is aligned with the map Z axis, so its footprint stays circular
/// when the map is viewed from above. Point attributes are retained for every
/// point outside the footprint.
VerticalEraseResult erase_vertical_cylinder(
  const std::vector<SubMap::Ptr>& submaps,
  const Eigen::Vector2d& center_xy,
  double radius_m);

}  // namespace glim
