#include <glim/mapping/map_eraser.hpp>

#include <cmath>
#include <stdexcept>

#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace glim {

VerticalEraseResult erase_vertical_cylinder(
  const std::vector<SubMap::Ptr>& submaps,
  const Eigen::Vector2d& center_xy,
  const double radius_m) {
  if (!center_xy.allFinite() || !std::isfinite(radius_m) || radius_m <= 0.0) {
    throw std::invalid_argument("vertical eraser requires a finite center and a positive radius");
  }

  const double radius_squared = radius_m * radius_m;
  VerticalEraseResult result;

  for (const auto& submap : submaps) {
    if (!submap || !submap->frame || !submap->frame->has_points()) {
      continue;
    }

    std::vector<int> retained_indices;
    retained_indices.reserve(submap->frame->size());
    for (std::size_t point_index = 0; point_index < submap->frame->size(); ++point_index) {
      const Eigen::Vector4d world_point = submap->T_world_origin * submap->frame->points[point_index];
      const Eigen::Vector2d offset = world_point.head<2>() - center_xy;
      if (!world_point.head<3>().allFinite() || offset.squaredNorm() > radius_squared) {
        retained_indices.push_back(static_cast<int>(point_index));
      }
    }

    const std::size_t erased = submap->frame->size() - retained_indices.size();
    if (!erased) {
      continue;
    }

    submap->frame = gtsam_points::sample(submap->frame, retained_indices);
    result.erased_points += erased;
    result.changed_submap_ids.push_back(submap->id);
  }

  return result;
}

}  // namespace glim
