#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Geometry>

namespace gtsam_points { struct PointCloud; }

namespace glim {

struct GeographicCoordinate {
  double latitude = 0.0;
  double longitude = 0.0;
};

// WGS84 degrees <-> local AEQD metres. PROJ owns all geodetic calculations.
class Wgs84Projection {
public:
  explicit Wgs84Projection(const GeographicCoordinate& origin);
  ~Wgs84Projection();
  Wgs84Projection(const Wgs84Projection&) = delete;
  Wgs84Projection& operator=(const Wgs84Projection&) = delete;
  Eigen::Vector2d forward(const GeographicCoordinate& coordinate) const;
  GeographicCoordinate reverse(const Eigen::Vector2d& metres) const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};

struct GeographicControlPoint {
  std::string id;
  std::string label;
  int submap_id = -1;
  std::string cloud_id;
  Eigen::Vector3d point_in_submap = Eigen::Vector3d::Zero();
  GeographicCoordinate wgs84;
  // One standard deviation per horizontal axis; manual input defaults to 10 m.
  double accuracy_m = 10.0;
  bool enabled = true;
};

struct GeoreferenceSubmap {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  std::string cloud_id;
};
using GeoreferenceGeometry = std::map<int, GeoreferenceSubmap>;

struct GeoreferenceResidual {
  std::string point_id;
  double metres = 0.0;
  bool high_residual = false;
};

struct GeoreferenceDiagnostic {
  std::string point_id;
  double deviation_m = 0.0;
  bool independent_check = false;
  bool suspect = false;
  // Estimates from the other controls, with the original map height preserved.
  std::optional<Eigen::Vector3d> suggested_position;
  std::optional<GeographicCoordinate> suggested_wgs84;
};

struct GeoreferenceSolution {
  std::string geometry_fingerprint;
  std::string controls_fingerprint;
  GeographicCoordinate origin;
  Eigen::Matrix3d horizontal_from_map = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d map_to_grid = Eigen::Matrix3d::Identity();
  // GTSAM Pose2 local tangent covariance: tx, ty, heading (metres/radians).
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  double rmse_m = 0.0;
  double max_error_m = 0.0;
  double baseline_m = 0.0;
  bool weak_geometry = true;
  std::vector<GeoreferenceResidual> residuals;

  GeographicCoordinate to_wgs84(const Eigen::Vector3d& point_in_map) const;
};

class Georeference {
public:
  std::vector<GeographicControlPoint> points;
  Eigen::Vector3d up_in_map = Eigen::Vector3d::UnitZ();
  std::optional<GeoreferenceSolution> solution;

  void invalidate();
  void fit(const GeoreferenceGeometry& geometry);
  bool current(const GeoreferenceGeometry& geometry) const;
  std::vector<GeoreferenceDiagnostic> diagnostics(const GeoreferenceGeometry& geometry) const;
  void validate_anchors(const GeoreferenceGeometry& geometry) const;
  void save(const std::filesystem::path& path, const std::string& source_fingerprint = "") const;
  static Georeference load(const std::filesystem::path& path);
  static std::string fingerprint(const GeoreferenceGeometry& geometry);
  static std::string cloud_id(const gtsam_points::PointCloud& cloud);
  static GeoreferenceGeometry load_geometry(const std::filesystem::path& dump);
private:
  std::string controls_fingerprint() const;
};

}  // namespace glim
