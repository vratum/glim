#pragma once

#include <array>
#include <optional>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/georeference.hpp>

namespace glim {

// All methods run on the viewer thread. Mapping callbacks only mark geometry dirty.
class GeoreferencePanel {
public:
  void open_map(const std::string& path, bool additional);
  void save_map(const std::string& path);
  void clear();
  void update_geometry(const std::vector<SubMap::ConstPtr>& submaps, const std::vector<Eigen::Isometry3d>& poses);
  void draw();
private:
  enum class PickMode { NONE, ADD, MOVE, INSPECT };
  void pick();
  void draw_editor();
  void edit(const GeographicControlPoint& point, int index);
  void changed();
  void persist();
  void draw_markers();
  void refresh_diagnostics();
  const GeoreferenceDiagnostic* diagnostic(const std::string& id) const;

  Georeference model;
  GeoreferenceGeometry geometry;
  std::vector<GeoreferenceDiagnostic> control_diagnostics;
  std::vector<SubMap::ConstPtr> submaps;
  std::map<int, const gtsam_points::PointCloud*> cached_clouds;
  std::filesystem::path map_path;
  std::string message;
  std::string load_error;
  bool current = false;
  bool markers_dirty = true;
  bool editor_requested = false;
  int selected = -1;
  int editing_index = -1;
  GeographicControlPoint editing;
  std::array<char,128> editing_label{};
  std::optional<GeographicControlPoint> deleted;
  std::optional<GeographicCoordinate> inspected;
  PickMode pick_mode = PickMode::NONE;
};
}  // namespace glim
