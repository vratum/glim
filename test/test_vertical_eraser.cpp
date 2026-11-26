#include <glim/mapping/map_eraser.hpp>

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace {

glim::SubMap::Ptr make_submap(
  const int id,
  const Eigen::Vector3d& origin,
  const std::vector<Eigen::Vector4d>& points) {
  auto submap = std::make_shared<glim::SubMap>();
  submap->id = id;
  submap->T_world_origin = Eigen::Isometry3d::Identity();
  submap->T_world_origin.translation() = origin;
  auto frame = std::make_shared<gtsam_points::PointCloudCPU>(points);
  std::vector<double> intensities(points.size());
  for (std::size_t i = 0; i < intensities.size(); ++i) {
    intensities[i] = static_cast<double>(i + 1);
  }
  frame->add_intensities(intensities);
  submap->frame = frame;
  return submap;
}

}  // namespace

int main() {
  const auto nearby = make_submap(
    4,
    Eigen::Vector3d::Zero(),
    {{0.0, 0.0, -20.0, 1.0}, {0.6, 0.6, 120.0, 1.0}, {1.1, 0.0, 0.0, 1.0}});
  const auto translated = make_submap(
    9,
    Eigen::Vector3d(10.0, 20.0, 0.0),
    {{0.0, 0.0, 3.0, 1.0}, {2.0, 0.0, 4.0, 1.0}});
  const std::vector<glim::SubMap::Ptr> submaps{nearby, translated};

  const auto first = glim::erase_vertical_cylinder(submaps, Eigen::Vector2d(0.0, 0.0), 1.0);
  assert(first.erased_points == 2);
  assert((first.changed_submap_ids == std::vector<int>{4}));
  assert(nearby->frame->size() == 1);
  assert(std::abs(nearby->frame->points[0].x() - 1.1) < 1e-12);
  assert(nearby->frame->has_intensities());
  assert(std::abs(nearby->frame->intensities[0] - 3.0) < 1e-12);

  const auto second = glim::erase_vertical_cylinder(submaps, Eigen::Vector2d(10.0, 20.0), 0.5);
  assert(second.erased_points == 1);
  assert((second.changed_submap_ids == std::vector<int>{9}));
  assert(translated->frame->size() == 1);
  assert(std::abs(translated->frame->points[0].x() - 2.0) < 1e-12);

  bool rejected = false;
  try {
    glim::erase_vertical_cylinder(submaps, Eigen::Vector2d::Zero(), 0.0);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);
  std::cout << "vertical eraser: world footprint and point attributes passed\n";
}
