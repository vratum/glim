#include <glim/util/georeference.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <proj.h>
#include <nlohmann/json.hpp>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <glim/mapping/sub_map.hpp>

namespace glim {
namespace {
using Json = nlohmann::json;

void valid_coordinate(const GeographicCoordinate& point) {
  if (!std::isfinite(point.latitude) || !std::isfinite(point.longitude) ||
      std::abs(point.latitude) > 90 || std::abs(point.longitude) > 180) {
    throw std::runtime_error("WGS84 latitude must be [-90,90], longitude [-180,180] degrees");
  }
}

template <typename Derived> Json array(const Eigen::MatrixBase<Derived>& matrix) {
  Json out = Json::array();
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) out.push_back(matrix(row, col));
  }
  return out;
}

template <int Rows, int Cols> Eigen::Matrix<double, Rows, Cols> matrix(const Json& value) {
  if (!value.is_array() || value.size() != Rows * Cols) throw std::runtime_error("Invalid georeference matrix");
  Eigen::Matrix<double, Rows, Cols> out;
  for (int row = 0; row < Rows; ++row) {
    for (int col = 0; col < Cols; ++col) out(row, col) = value.at(row * Cols + col).get<double>();
  }
  if (!out.allFinite()) throw std::runtime_error("Non-finite georeference matrix");
  return out;
}

std::string hash_bytes(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : value) { hash ^= byte; hash *= 1099511628211ULL; }
  std::ostringstream out;
  out << std::hex << hash;
  return out.str();
}

// The residual and its derivative are evaluated by GTSAM's Pose2 implementation.
class ControlPointFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
public:
  ControlPointFactor(const Eigen::Vector2d& local, const Eigen::Vector2d& measured, double sigma)
  : gtsam::NoiseModelFactor1<gtsam::Pose2>(
      gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5),
                                      gtsam::noiseModel::Isotropic::Sigma(2, sigma)), 0),
    local(local), measured(measured) {}
  gtsam::Vector evaluateError(const gtsam::Pose2& pose, gtsam::OptionalMatrixType H = nullptr) const override {
    gtsam::Matrix23 jacobian;
    const auto prediction = pose.transformFrom(local, jacobian);
    if (H) *H = jacobian;
    return prediction - measured;
  }
private:
  Eigen::Vector2d local;
  Eigen::Vector2d measured;
};
}  // namespace

struct Wgs84Projection::Impl {
  PJ_CONTEXT* context = proj_context_create();
  PJ* operation = nullptr;
  ~Impl() { proj_destroy(operation); proj_context_destroy(context); }
};

Wgs84Projection::Wgs84Projection(const GeographicCoordinate& origin) : impl(new Impl) {
  valid_coordinate(origin);
  std::ostringstream target;
  target.imbue(std::locale::classic());
  target << std::setprecision(17) << "+proj=aeqd +ellps=WGS84 +units=m +type=crs +lat_0="
         << origin.latitude << " +lon_0=" << origin.longitude;
  PJ* raw = proj_create_crs_to_crs(impl->context, "EPSG:4326", target.str().c_str(), nullptr);
  if (raw) {
    impl->operation = proj_normalize_for_visualization(impl->context, raw);
    proj_destroy(raw);
  }
  if (!impl->operation) throw std::runtime_error("PROJ could not create the WGS84 projection");
}
Wgs84Projection::~Wgs84Projection() = default;

Eigen::Vector2d Wgs84Projection::forward(const GeographicCoordinate& coordinate) const {
  valid_coordinate(coordinate);
  proj_errno_reset(impl->operation);
  const auto result = proj_trans(impl->operation, PJ_FWD, proj_coord(coordinate.longitude, coordinate.latitude, 0, 0));
  if (proj_errno(impl->operation) || !std::isfinite(result.xy.x) || !std::isfinite(result.xy.y)) {
    throw std::runtime_error("PROJ WGS84 projection failed");
  }
  return {result.xy.x, result.xy.y};
}

GeographicCoordinate Wgs84Projection::reverse(const Eigen::Vector2d& metres) const {
  if (!metres.allFinite()) throw std::runtime_error("Non-finite projected coordinate");
  proj_errno_reset(impl->operation);
  const auto result = proj_trans(impl->operation, PJ_INV, proj_coord(metres.x(), metres.y(), 0, 0));
  if (proj_errno(impl->operation)) throw std::runtime_error("PROJ inverse WGS84 projection failed");
  GeographicCoordinate coordinate{result.xy.y, result.xy.x};
  valid_coordinate(coordinate);
  return coordinate;
}

GeographicCoordinate GeoreferenceSolution::to_wgs84(const Eigen::Vector3d& point) const {
  if (!point.allFinite()) throw std::runtime_error("Non-finite map coordinate");
  const Eigen::Vector3d horizontal = horizontal_from_map * point;
  return Wgs84Projection(origin).reverse((map_to_grid * Eigen::Vector3d(horizontal.x(), horizontal.y(), 1)).head<2>());
}

void Georeference::invalidate() { solution.reset(); }

std::string Georeference::controls_fingerprint() const {
  Json value = {array(up_in_map), Json::array()};
  for (const auto& p : points) value[1].push_back({p.id, p.submap_id, p.cloud_id, array(p.point_in_submap), p.wgs84.latitude, p.wgs84.longitude, p.accuracy_m, p.enabled});
  return hash_bytes(value.dump());
}

void Georeference::validate_anchors(const GeoreferenceGeometry& geometry) const {
  std::set<std::string> ids;
  for (const auto& point : points) {
    valid_coordinate(point.wgs84);
    if (point.id.empty() || !ids.insert(point.id).second || !point.point_in_submap.allFinite() ||
        !std::isfinite(point.accuracy_m) || point.accuracy_m <= 0) {
      throw std::runtime_error("Invalid control point or horizontal accuracy");
    }
    const auto found = geometry.find(point.submap_id);
    if (point.enabled && (found == geometry.end() || point.cloud_id.empty() || found->second.cloud_id != point.cloud_id)) {
      throw std::runtime_error("Control point '" + point.label + "' belongs to a missing or different submap");
    }
  }
}

std::string Georeference::fingerprint(const GeoreferenceGeometry& geometry) {
  std::ostringstream text;
  text.imbue(std::locale::classic());
  // Match SubMap::save's six significant digits, including after reloading.
  text << std::setprecision(6);
  for (const auto& [id, submap] : geometry) {
    text << id << ':' << submap.cloud_id << ':';
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        const double value = submap.pose.matrix()(row, col);
        if (!std::isfinite(value)) throw std::runtime_error("Non-finite submap pose");
        text << (value == 0 ? 0.0 : value) << ',';
      }
    }
  }
  return hash_bytes(text.str());
}

std::string Georeference::cloud_id(const gtsam_points::PointCloud& cloud) {
  std::uint64_t hash = 1469598103934665603ULL;
  // Compact dumps preserve float XYZ; hash in that representation across reloads.
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      const float value = static_cast<float>(cloud.points[i][axis]);
      const auto bytes = reinterpret_cast<const unsigned char*>(&value);
      for (std::size_t byte = 0; byte < sizeof(value); ++byte) { hash ^= bytes[byte]; hash *= 1099511628211ULL; }
    }
  }
  return std::to_string(cloud.size()) + "-" + std::to_string(hash);
}

GeoreferenceGeometry Georeference::load_geometry(const std::filesystem::path& dump) {
  GeoreferenceGeometry geometry;
  for (const auto& entry : std::filesystem::directory_iterator(dump)) {
    const auto name = entry.path().filename().string();
    if (name.size() != 6 || !std::all_of(name.begin(), name.end(), [](char c) { return c >= '0' && c <= '9'; }) || !entry.is_directory()) continue;
    auto submap = SubMap::load(entry.path().string());
    if (!submap || !submap->frame) throw std::runtime_error("Cannot load georeferenced submap " + name);
    geometry.emplace(submap->id, GeoreferenceSubmap{submap->T_world_origin, cloud_id(*submap->frame)});
  }
  return geometry;
}

bool Georeference::current(const GeoreferenceGeometry& geometry) const {
  if (!solution || solution->geometry_fingerprint != fingerprint(geometry) || solution->controls_fingerprint != controls_fingerprint()) return false;
  try { validate_anchors(geometry); } catch (const std::exception&) { return false; }
  return true;
}

void Georeference::fit(const GeoreferenceGeometry& geometry) {
  invalidate();
  validate_anchors(geometry);
  std::vector<const GeographicControlPoint*> active;
  for (const auto& point : points) if (point.enabled) active.push_back(&point);
  if (active.size() < 2) throw std::runtime_error("Add at least two distinct control points; 4-6 spread over the map are recommended");
  if (!up_in_map.allFinite() || up_in_map.norm() < 0.99) throw std::runtime_error("Invalid map vertical axis");
  GeoreferenceSolution result;
  const Eigen::Vector3d up = up_in_map.normalized();
  Eigen::Vector3d horizontal = Eigen::Vector3d::UnitX() - up.x() * up;
  if (horizontal.norm() < 0.1) horizontal = Eigen::Vector3d::UnitY() - up.y() * up;
  horizontal.normalize();
  result.horizontal_from_map.row(0) = horizontal;
  result.horizontal_from_map.row(1) = up.cross(horizontal);
  result.horizontal_from_map.row(2) = up;
  result.origin = active.front()->wgs84;
  Wgs84Projection projection(result.origin);
  Eigen::Matrix<double, 2, Eigen::Dynamic> local(2, active.size()), measured(2, active.size());
  double sigma_sum = 0;
  for (std::size_t i = 0; i < active.size(); ++i) {
    const auto& point = *active[i];
    local.col(i) = (result.horizontal_from_map * (geometry.at(point.submap_id).pose * point.point_in_submap)).head<2>();
    measured.col(i) = projection.forward(point.wgs84);
    sigma_sum += point.accuracy_m;
    for (std::size_t j = 0; j < i; ++j) result.baseline_m = std::max(result.baseline_m, (local.col(i) - local.col(j)).norm());
  }
  if (result.baseline_m < 1e-3 || (measured.colwise() - measured.rowwise().mean()).norm() < 1e-3) {
    throw std::runtime_error("Control points must have distinct map and geographic positions");
  }
  const Eigen::Matrix3d initial = Eigen::umeyama(local, measured, false);
  gtsam::NonlinearFactorGraph graph;
  for (std::size_t i = 0; i < active.size(); ++i) graph.emplace_shared<ControlPointFactor>(local.col(i), measured.col(i), active[i]->accuracy_m);
  gtsam::Values values;
  values.insert(0, gtsam::Pose2(initial(0,2), initial(1,2), std::atan2(initial(1,0), initial(0,0))));
  gtsam::LevenbergMarquardtParams params;
  params.maxIterations = 100;
  const auto optimized = gtsam::LevenbergMarquardtOptimizer(graph, values, params).optimize();
  const auto pose = optimized.at<gtsam::Pose2>(0);
  result.map_to_grid = pose.matrix();
  result.covariance = gtsam::Marginals(graph, optimized).marginalCovariance(0);
  if (!result.map_to_grid.allFinite() || !result.covariance.allFinite()) throw std::runtime_error("Georeference fit is singular");
  double squared_error = 0;
  for (std::size_t i = 0; i < active.size(); ++i) {
    const double error = (pose.transformFrom(Eigen::Vector2d(local.col(i))) - measured.col(i)).norm();
    result.residuals.push_back({active[i]->id, error, error > 3 * active[i]->accuracy_m});
    squared_error += error * error;
    result.max_error_m = std::max(result.max_error_m, error);
  }
  result.rmse_m = std::sqrt(squared_error / active.size());
  result.weak_geometry = active.size() < 3 || result.baseline_m < 5 * sigma_sum / active.size() || std::sqrt(result.covariance(2,2)) > 0.17;
  result.geometry_fingerprint = fingerprint(geometry);
  result.controls_fingerprint = controls_fingerprint();
  solution = result;
}

void Georeference::save(const std::filesystem::path& path, const std::string& source_fingerprint) const {
  Json document{{"format_version", 1}, {"crs", "EPSG:4326"}, {"projection", "AEQD"},
                {"up_in_map", array(up_in_map)}, {"points", Json::array()}};
  for (const auto& p : points) {
    valid_coordinate(p.wgs84);
    document["points"].push_back({{"id",p.id}, {"label",p.label}, {"submap_id",p.submap_id}, {"cloud_id",p.cloud_id},
      {"point_in_submap",array(p.point_in_submap)}, {"latitude",p.wgs84.latitude}, {"longitude",p.wgs84.longitude},
      {"accuracy_m",p.accuracy_m}, {"enabled",p.enabled}});
  }
  if (!source_fingerprint.empty()) document["source_fingerprint"] = source_fingerprint;
  if (solution) {
    const auto& s = *solution;
    document["solution"] = {{"geometry_fingerprint",s.geometry_fingerprint}, {"controls_fingerprint",s.controls_fingerprint}, {"origin_latitude",s.origin.latitude},
      {"origin_longitude",s.origin.longitude}, {"horizontal_from_map",array(s.horizontal_from_map)},
      {"map_to_grid",array(s.map_to_grid)}, {"covariance",array(s.covariance)}, {"rmse_m",s.rmse_m},
      {"max_error_m",s.max_error_m}, {"baseline_m",s.baseline_m}, {"weak_geometry",s.weak_geometry}, {"residuals",Json::array()}};
    for (const auto& r : s.residuals) document["solution"]["residuals"].push_back({{"point_id",r.point_id}, {"metres",r.metres}, {"high_residual",r.high_residual}});
  }
  const auto temporary = path.string() + ".tmp." + std::to_string(getpid());
  try {
    std::ofstream output(temporary);
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output << document.dump(2) << '\n';
    output.close();
    std::filesystem::rename(temporary, path);
  } catch (...) { std::filesystem::remove(temporary); throw; }
}

Georeference Georeference::load(const std::filesystem::path& path) {
  std::ifstream input(path);
  Json document;
  input >> document;
  if (document.at("format_version") != 1 || document.at("crs") != "EPSG:4326" || document.at("projection") != "AEQD") {
    throw std::runtime_error("Unsupported georeference format or CRS");
  }
  Georeference out;
  out.up_in_map = matrix<3,1>(document.at("up_in_map"));
  for (const auto& p : document.at("points")) {
    out.points.push_back({p.at("id"), p.at("label"), p.at("submap_id"), p.at("cloud_id"), matrix<3,1>(p.at("point_in_submap")),
      {p.at("latitude"), p.at("longitude")}, p.at("accuracy_m"), p.at("enabled")});
  }
  // Validate scalar inputs even before the corresponding geometry is available.
  GeoreferenceGeometry identities;
  for (const auto& point : out.points) identities[point.submap_id].cloud_id = point.cloud_id;
  out.validate_anchors(identities);
  if (document.contains("solution")) {
    const auto& s = document.at("solution");
    GeoreferenceSolution result;
    result.geometry_fingerprint = s.at("geometry_fingerprint");
    result.controls_fingerprint = s.at("controls_fingerprint");
    result.origin = {s.at("origin_latitude"), s.at("origin_longitude")};
    valid_coordinate(result.origin);
    result.horizontal_from_map = matrix<3,3>(s.at("horizontal_from_map"));
    result.map_to_grid = matrix<3,3>(s.at("map_to_grid"));
    result.covariance = matrix<3,3>(s.at("covariance"));
    if (!result.horizontal_from_map.isUnitary(1e-6) || result.horizontal_from_map.determinant() < 0.999 ||
        !result.map_to_grid.topLeftCorner<2,2>().isUnitary(1e-6) || result.map_to_grid.topLeftCorner<2,2>().determinant() < 0.999 ||
        (result.map_to_grid.row(2) - Eigen::RowVector3d(0,0,1)).norm() > 1e-9) throw std::runtime_error("Invalid georeference rotation");
    result.rmse_m = s.at("rmse_m");
    result.max_error_m = s.at("max_error_m");
    result.baseline_m = s.at("baseline_m");
    result.weak_geometry = s.at("weak_geometry");
    for (const auto& r : s.at("residuals")) result.residuals.push_back({r.at("point_id"),r.at("metres"),r.at("high_residual")});
    out.solution = result;
    if (result.controls_fingerprint != out.controls_fingerprint()) out.invalidate();
  }
  return out;
}
}  // namespace glim
