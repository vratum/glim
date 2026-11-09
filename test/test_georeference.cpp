#include <glim/util/georeference.hpp>
#include <glim/mapping/sub_map.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>

using namespace glim;

int main() {
  const GeographicCoordinate origin{41.02, 28.98};
  Wgs84Projection projection(origin);
  assert(projection.forward(origin).norm() < 1e-7);
  const auto east = Wgs84Projection({0,0}).forward({0,1});
  assert(std::abs(east.x() - 111319.490793) < 0.001 && std::abs(east.y()) < 0.001);
  for (const Eigen::Vector2d point : {Eigen::Vector2d(100, 200), Eigen::Vector2d(-800, 300)}) {
    assert((projection.forward(projection.reverse(point)) - point).norm() < 1e-6);
  }
  bool rejected = false;
  try { projection.forward({91, 0}); } catch (const std::exception&) { rejected = true; }
  assert(rejected);

  GeoreferenceGeometry geometry;
  geometry[7].cloud_id = "test-cloud";
  geometry[7].pose.translation() = Eigen::Vector3d(15, -3, 8);
  Georeference model;
  const Eigen::Rotation2Dd rotation(0.7);
  const Eigen::Vector2d translation(300, -150);
  const std::vector<Eigen::Vector3d> locations{{0,0,0}, {120,0,2}, {0,150,-1}, {120,150,4}, {60,80,0}, {-90,40,0}};
  for (std::size_t i = 0; i < locations.size(); ++i) {
    GeographicControlPoint point;
    point.id = "point-" + std::to_string(i);
    point.label = point.id;
    point.submap_id = 7;
    point.cloud_id = "test-cloud";
    point.point_in_submap = geometry[7].pose.inverse() * locations[i];
    point.wgs84 = projection.reverse(rotation * locations[i].head<2>() + translation);
    model.points.push_back(point);
  }
  model.fit(geometry);
  assert(model.current(geometry));
  assert(model.solution->rmse_m < 1e-5);
  assert(!model.solution->weak_geometry);
  const Eigen::Vector3d probe(35, 70, 12);
  const Eigen::Vector2d expected = rotation * probe.head<2>() + translation;
  assert((projection.forward(model.solution->to_wgs84(probe)) - expected).norm() < 1e-5);

  // Human input is noisy: a 10 m displacement must not invalidate the fit.
  auto noisy = model;
  for (std::size_t i = 0; i < noisy.points.size(); ++i) {
    auto xy = projection.forward(noisy.points[i].wgs84);
    xy += Eigen::Vector2d(i % 2 ? 8 : -8, i % 3 ? 3 : -6);
    noisy.points[i].wgs84 = projection.reverse(xy);
  }
  noisy.fit(geometry);
  assert(noisy.current(geometry));
  assert(noisy.solution->rmse_m > 5 && noisy.solution->rmse_m < 15);
  assert((projection.forward(noisy.solution->to_wgs84(probe)) - expected).norm() < 10);
  for (const auto& residual : noisy.solution->residuals) assert(!residual.high_residual);

  // A gross mistake is downweighted, not silently deleted.
  auto bad = model;
  bad.points.back().wgs84 = projection.reverse(projection.forward(bad.points.back().wgs84) + Eigen::Vector2d(250, -200));
  bad.fit(geometry);
  assert(bad.solution->residuals.back().high_residual);
  assert(bad.points.size() == model.points.size());
  assert((projection.forward(bad.solution->to_wgs84(probe)) - expected).norm() < 15);
  bad.points.back().accuracy_m = 10000;
  bad.fit(geometry);
  assert((projection.forward(bad.solution->to_wgs84(probe)) - expected).norm() < 0.1);

  // A tilted map still uses its gravity-defined horizontal plane.
  auto tilted = model;
  const Eigen::Matrix3d tilt = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix();
  tilted.up_in_map = tilt * Eigen::Vector3d::UnitZ();
  for (auto& p : tilted.points) p.point_in_submap = geometry[7].pose.inverse() * (tilt * (geometry[7].pose * p.point_in_submap));
  tilted.fit(geometry);
  assert((projection.forward(tilted.solution->to_wgs84(tilt * probe)) - expected).norm() < 1e-5);

  // Incorrect geometry and clouds must not reuse a previous calibration.
  auto moved = geometry;
  moved[7].pose.translation().x() += 1;
  assert(!model.current(moved));
  model.fit(moved);
  assert(model.current(moved));
  moved[7].cloud_id = "another-map";
  assert(!model.current(moved));
  rejected = false;
  try { model.fit(moved); } catch (const std::exception&) { rejected = true; }
  assert(rejected);

  model.fit(geometry);
  const auto path = std::filesystem::temp_directory_path() / ("glim-georef-test-" + std::to_string(getpid()) + ".json");
  model.save(path);
  auto loaded = Georeference::load(path);
  assert(loaded.current(geometry));
  assert(loaded.points.size() == model.points.size());
  assert((projection.forward(loaded.solution->to_wgs84(probe)) - expected).norm() < 1e-5);
  loaded.points[0].wgs84.latitude += 0.001;
  assert(!loaded.current(geometry));
  loaded.save(path);
  assert(!Georeference::load(path).solution);
  loaded.points.pop_back();
  loaded.invalidate();
  loaded.save(path);
  assert(!Georeference::load(path).solution);
  std::filesystem::remove(path);

  // Validate identities against the actual compact dump, including float rounding.
  const auto dump = std::filesystem::temp_directory_path() / ("glim-georef-dump-" + std::to_string(getpid()));
  std::filesystem::create_directories(dump / "000007");
  SubMap submap;
  submap.id = 7;
  submap.T_world_origin = geometry.at(7).pose;
  submap.T_world_origin.translation().x() = 15.123456789;
  submap.T_origin_endpoint_L.setIdentity();
  submap.T_origin_endpoint_R.setIdentity();
  EstimationFrame::Ptr frame(new EstimationFrame);
  frame->id = 0; frame->stamp = 0; frame->frame_id = FrameID::IMU;
  frame->T_lidar_imu.setIdentity(); frame->T_world_lidar.setIdentity();
  frame->v_world_imu.setZero(); frame->imu_bias.setZero();
  submap.frames.push_back(frame); submap.odom_frames.push_back(frame);
  std::vector<Eigen::Vector4d> cloud_points;
  for (int i = 0; i < 30; ++i) cloud_points.emplace_back(i * 0.123456789, i % 3, 0, 1);
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(cloud_points);
  Eigen::Matrix4d point_cov = Eigen::Matrix4d::Identity() * 0.01;
  point_cov(3,3) = 0;
  cloud->add_covs(std::vector<Eigen::Matrix4d>(cloud_points.size(), point_cov));
  submap.frame = cloud;
  geometry[7] = {submap.T_world_origin, Georeference::cloud_id(*submap.frame)};
  for (auto& point : model.points) point.cloud_id = geometry.at(7).cloud_id;
  model.fit(geometry);
  submap.save((dump / "000007").string());
  assert(model.current(Georeference::load_geometry(dump)));
  std::filesystem::remove_all(dump);

  // Two well separated points work, but have no redundancy.
  model.points.resize(2);
  model.fit(geometry);
  assert(model.solution->weak_geometry);
  model.points[1].point_in_submap = model.points[0].point_in_submap;
  rejected = false;
  try { model.fit(geometry); } catch (const std::exception&) { rejected = true; }
  assert(rejected);
  assert(!model.solution);
  std::cout << "georeference: projection, noisy/incorrect controls, map changes and persistence passed\n";
}
