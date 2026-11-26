#pragma once

#include <optional>
#include <unordered_set>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim/viewer/interactive_viewer.hpp>

namespace guik {
class ProgressModal;
class ProgressInterface;
}  // namespace guik

namespace glim {

class GeoreferencePanel;

class OfflineViewer : public InteractiveViewer {
public:
  OfflineViewer(const std::string& init_map_path = "", const std::string& export_map_path = "");
  virtual ~OfflineViewer() override;

private:
  virtual void setup_ui() override;
  void on_submaps_changed() override { georeference_geometry_dirty = true; }

  void main_menu();
  void draw_vertical_eraser();
  void request_vertical_erase(const Eigen::Vector3d& center);
  void finish_vertical_erase(const VerticalEraseResult& result);
  void draw_vertical_eraser_footprint();
  double vertical_eraser_plane_z() const;
  std::optional<Eigen::Vector3d> vertical_eraser_center_from_pixel(const Eigen::Vector2i& pixel, double plane_z) const;

  std::shared_ptr<GlobalMapping> load_map(guik::ProgressInterface& progress, const std::string& path, std::shared_ptr<GlobalMapping> global_mapping);
  bool save_map(guik::ProgressInterface& progress, const std::string& path);
  bool export_map(guik::ProgressInterface& progress, const std::string& path);

private:
  std::string init_map_path;
  std::string export_map_path;
  std::unique_ptr<guik::ProgressModal> progress_modal;
  std::unique_ptr<GeoreferencePanel> georeference_panel;
  bool georeference_geometry_dirty = true;
  std::string pending_open_path;
  bool pending_open_additional = false;
  std::string pending_save_path;
  bool save_georeference_pending = false;
  bool vertical_eraser_active = false;
  bool vertical_eraser_has_footprint = false;
  float vertical_eraser_radius_m = 2.0f;
  Eigen::Vector3d vertical_eraser_center = Eigen::Vector3d::Zero();
  std::string vertical_eraser_message;

  std::unordered_set<std::string> imported_shared_libs;
  std::unique_ptr<AsyncGlobalMapping> async_global_mapping;
};

}  // namespace glim
