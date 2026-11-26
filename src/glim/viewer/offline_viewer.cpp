#include <glim/viewer/offline_viewer.hpp>
#include <glim/viewer/interactive/georeference_panel.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <boost/filesystem.hpp>
#include <gtsam_points/config.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#include <glim/util/config.hpp>

#include <spdlog/spdlog.h>
#include <portable-file-dialogs.h>
#include <glk/io/ply_io.hpp>
#include <glk/thin_lines.hpp>
#include <guik/recent_files.hpp>
#include <guik/progress_modal.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

OfflineViewer::OfflineViewer(const std::string& init_map_path, const std::string& export_map_path)
: InteractiveViewer(false), init_map_path(init_map_path), export_map_path(export_map_path) { start(); }

OfflineViewer::~OfflineViewer() {
  // Stop UI callbacks before destroying the panel they capture.
  kill_switch = true;
  if (thread.joinable()) thread.join();
  // A load/save worker also captures this object. Finish it while all fields
  // and the mapping backend are still alive.
  progress_modal.reset();
  async_global_mapping.reset();
}

void OfflineViewer::setup_ui() {
  auto viewer = guik::LightViewer::instance();
  viewer->register_ui_callback("main_menu", [this] { main_menu(); });
  georeference_panel.reset(new GeoreferencePanel);
  viewer->register_ui_callback("georeference", [this] {
    if (georeference_geometry_dirty) {
      georeference_panel->update_geometry(submaps, submap_poses);
      georeference_geometry_dirty = false;
    }
    if (save_georeference_pending) {
      georeference_panel->save_map(pending_save_path);
      save_georeference_pending = false;
    }
    georeference_panel->draw();
  });
  viewer->register_ui_callback("vertical_eraser", [this] { draw_vertical_eraser(); });

  progress_modal.reset(new guik::ProgressModal("offline_viewer_progress"));

#ifdef GTSAM_POINTS_USE_CUDA
  gtsam_points::LinearizationHook::register_hook([] { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif
}

void OfflineViewer::main_menu() {
  bool start_open_map = false || !init_map_path.empty();
  bool start_close_map = false;
  bool start_save_map = false;
  bool start_export_map = false;

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (!async_global_mapping) {  // if a previously loaded map does not yet exist
        if (ImGui::MenuItem("Open New Map")) {
          start_open_map = true;
        }
      } else {
        if (ImGui::MenuItem("Open Additional Map")) {
          start_open_map = true;
        }
      }

      if (ImGui::MenuItem("Close Map")) {
        if (pfd::message("Warning", "Close the map?").result() == pfd::button::ok) {
          start_close_map = true;
        }
      }

      if (ImGui::BeginMenu("Save")) {
        if (ImGui::MenuItem("Save Map")) {
          start_save_map = true;
        }

        if (ImGui::MenuItem("Export Points")) {
          start_export_map = true;
        }

        ImGui::EndMenu();
      }

      if (ImGui::MenuItem("Quit")) {
        if (pfd::message("Warning", "Quit?").result() == pfd::button::ok) {
          request_to_terminate = true;
        }
      }

      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }

  // open map
  if (start_open_map) {
    logger->debug("open map");
    std::string map_path;

    guik::RecentFiles recent_files("offline_viewer_open");
    if (init_map_path.empty()) {
      map_path = pfd::select_folder("Select a dump directory", recent_files.most_recent()).result();
    } else {
      map_path = init_map_path;
      init_map_path.clear();
    }

    if (!map_path.empty()) {
      logger->debug("open map from {}", map_path);
      recent_files.push(map_path);

      if (boost::filesystem::exists(map_path + "/config")) {
        logger->info("Use config from {}", map_path + "/config");
        GlobalConfig::instance(map_path + "/config", true);
      } else {
        logger->warn("No config found in {}", map_path);
      }

      const Config config_ros(GlobalConfig::get_config_path("config_ros"));
      const std::vector<std::string> ext_module_names = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules", {});
      for (const auto& name : ext_module_names) {
        if (name.find("viewer") != std::string::npos || name.find("monitor") != std::string::npos) {
          continue;
        }
        if (imported_shared_libs.count(name)) {
          logger->debug("Extension module {} already loaded", name);
          continue;
        }

        logger->info("Export classes from {}", name);
        ExtensionModule::export_classes(name);
        imported_shared_libs.insert(name);
      }

      // if a map is already loaded, use existing map to load new map into
      std::shared_ptr<GlobalMapping> global_mapping;
      if (async_global_mapping) {
        logger->info("global map already exists, loading new map into existing global map");
        global_mapping = std::dynamic_pointer_cast<GlobalMapping>(async_global_mapping->get_global_mapping());
      }

      pending_open_path = map_path;
      pending_open_additional = static_cast<bool>(global_mapping);
      progress_modal->open<std::shared_ptr<GlobalMapping>>("open", [this, map_path, global_mapping](guik::ProgressInterface& progress) {
        return load_map(progress, map_path, global_mapping);
      });
    }
  }
  auto open_result = progress_modal->run<std::shared_ptr<GlobalMapping>>("open");
  if (open_result) {
    if (!(*open_result)) {
      pfd::message("Error", "Failed to load map").result();
    } else {
      if (!pending_open_additional) async_global_mapping.reset(new glim::AsyncGlobalMapping(*open_result, 1e6));
      georeference_panel->open_map(pending_open_path, pending_open_additional);
      georeference_geometry_dirty = true;
      // Let the load popup close before opening another progress task.
      return;
    }
  }

  // save map
  if (start_save_map) {
    if (!async_global_mapping) {
      logger->warn("No map data to save");
    } else {
      guik::RecentFiles recent_files("offline_viewer_save");
      const std::string path = pfd::select_folder("Select a directory to save the map", recent_files.most_recent()).result();
      if (!path.empty()) {
        recent_files.push(path);
        pending_save_path = path;
        progress_modal->open<bool>("save", [this, path](guik::ProgressInterface& progress) { return save_map(progress, path); });
      }
    }
  }
  auto save_result = progress_modal->run<bool>("save");
  if (save_result && *save_result) save_georeference_pending = true;

  // export map
  if (start_export_map) {
    guik::RecentFiles recent_files("offline_viewer_export");
    const std::string path = pfd::save_file("Select the file destination", recent_files.most_recent(), {"PLY", "*.ply"}).result();
    if (!path.empty()) {
      recent_files.push(path);
      progress_modal->open<bool>("export", [this, path](guik::ProgressInterface& progress) { return export_map(progress, path); });
    }
  }
  auto export_result = progress_modal->run<bool>("export");

  const auto erase_result = progress_modal->run<VerticalEraseResult>("vertical_erase");
  if (erase_result) {
    finish_vertical_erase(*erase_result);
  }

  if (async_global_mapping && !export_map_path.empty()) {
    logger->info("Exporting map to {}", export_map_path);
    const std::string path = export_map_path;
    progress_modal->open<std::string>("auto_export", [this, path = path](guik::ProgressInterface& progress) {
      export_map(progress, path);
      return path;
    });
    export_map_path.clear();
  }
  auto auto_export_result = progress_modal->run<std::string>("auto_export");
  if (auto_export_result) {
    logger->info("Exported map to {}", *auto_export_result);
    request_to_terminate = true;
  }

  // close map
  if (start_close_map) {
    if (async_global_mapping) {
      logger->info("Closing map");
      async_global_mapping->join();
      async_global_mapping.reset();
      georeference_panel->clear();
      clear();
      vertical_eraser_active = false;
      vertical_eraser_has_footprint = false;
    } else {
      logger->warn("No map to close");
    }
  }
}

void OfflineViewer::draw_vertical_eraser() {
  auto viewer = guik::LightViewer::instance();
  const bool map_loaded = static_cast<bool>(async_global_mapping);

  if (ImGui::Begin("Map eraser - vertical", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("Remove map points in a vertical column.");
    ImGui::TextUnformatted("Use top view and click anywhere to erase an area at every height.");
    ImGui::BeginDisabled(!map_loaded || progress_modal->is_running());
    ImGui::SetNextItemWidth(180.0f);
    ImGui::DragFloat("Radius (m)", &vertical_eraser_radius_m, 0.1f, 0.05f, 1000.0f, "%.2f");
    if (!vertical_eraser_active) {
      if (ImGui::Button("Erase on map", ImVec2(150.0f, 30.0f))) {
        vertical_eraser_active = true;
        vertical_eraser_message = "Left-click anywhere in the map view to place the eraser.";
      }
    } else {
      if (ImGui::Button("Stop erasing", ImVec2(150.0f, 30.0f))) {
        vertical_eraser_active = false;
      }
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.25f, 1.0f), "Click anywhere in the map view.");
    }
    ImGui::EndDisabled();
    if (!map_loaded) {
      ImGui::TextDisabled("Open a map to use the eraser.");
    }
    ImGui::TextDisabled("Save the map after editing.");
    if (!vertical_eraser_message.empty()) {
      ImGui::TextWrapped("%s", vertical_eraser_message.c_str());
    }
  }
  ImGui::End();

  if (vertical_eraser_active && map_loaded && !progress_modal->is_running()) {
    const auto& io = ImGui::GetIO();
    if (!io.WantCaptureMouse && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      const Eigen::Vector2i pixel(static_cast<int>(io.MousePos.x), static_cast<int>(io.MousePos.y));
      const auto info = viewer->pick_info(pixel, 0);
      const float depth = viewer->pick_depth(pixel, 0);
      if (info[0] == static_cast<int>(PickType::POINTS) && std::isfinite(depth) && depth > 0.0f && depth < 1.0f) {
        vertical_eraser_center = viewer->unproject(pixel, depth).cast<double>();
      } else {
        const auto center = vertical_eraser_center_from_pixel(pixel, vertical_eraser_plane_z());
        if (!center) {
          vertical_eraser_message = "Cannot place the eraser from this camera angle. Switch to top view.";
          draw_vertical_eraser_footprint();
          return;
        }
        vertical_eraser_center = *center;
      }
      vertical_eraser_has_footprint = true;
      request_vertical_erase(vertical_eraser_center);
    }
  }

  draw_vertical_eraser_footprint();
}

double OfflineViewer::vertical_eraser_plane_z() const {
  if (vertical_eraser_has_footprint) {
    return vertical_eraser_center.z();
  }

  std::vector<double> heights;
  heights.reserve(submap_poses.size());
  for (const auto& pose : submap_poses) {
    if (std::isfinite(pose.translation().z())) {
      heights.push_back(pose.translation().z());
    }
  }
  if (heights.empty()) {
    return 0.0;
  }
  const auto middle = heights.begin() + heights.size() / 2;
  std::nth_element(heights.begin(), middle, heights.end());
  return *middle;
}

std::optional<Eigen::Vector3d> OfflineViewer::vertical_eraser_center_from_pixel(
  const Eigen::Vector2i& pixel,
  const double plane_z) const {
  auto viewer = guik::LightViewer::instance();
  const Eigen::Vector3d near_point = viewer->unproject(pixel, 0.001f).cast<double>();
  const Eigen::Vector3d far_point = viewer->unproject(pixel, 0.999f).cast<double>();
  const Eigen::Vector3d direction = far_point - near_point;
  if (!near_point.allFinite() || !direction.allFinite() || std::abs(direction.z()) < 1e-9) {
    return std::nullopt;
  }

  const double distance = (plane_z - near_point.z()) / direction.z();
  const Eigen::Vector3d center = near_point + distance * direction;
  if (!std::isfinite(distance) || !center.allFinite()) {
    return std::nullopt;
  }
  return center;
}

void OfflineViewer::request_vertical_erase(const Eigen::Vector3d& center) {
  const Eigen::Vector2d center_xy = center.head<2>();
  const double radius_m = vertical_eraser_radius_m;
  vertical_eraser_message = "Erasing map points...";
  progress_modal->open<VerticalEraseResult>(
    "vertical_erase",
    [this, center_xy, radius_m](guik::ProgressInterface& progress) {
      progress.set_title("Erase map points");
      progress.set_text("Removing the vertical column");
      progress.set_maximum(1);
      const auto result = async_global_mapping->erase_vertical_cylinder(center_xy, radius_m);
      progress.increment();
      return result;
    });
}

void OfflineViewer::finish_vertical_erase(const VerticalEraseResult& result) {
  if (!result.erased_points) {
    vertical_eraser_message = "No map points were inside this vertical column.";
    return;
  }

  auto viewer = guik::LightViewer::instance();
  for (const int submap_id : result.changed_submap_ids) {
    viewer->remove_drawable("submap_" + std::to_string(submap_id));
  }
  viewer_dirty = true;
  georeference_geometry_dirty = true;
  vertical_eraser_message =
    "Erased " + std::to_string(result.erased_points) + " map points. Use File > Save > Save Map to keep this edit.";
  logger->info(
    "vertical eraser removed {} points from {} submaps",
    result.erased_points,
    result.changed_submap_ids.size());
}

void OfflineViewer::draw_vertical_eraser_footprint() {
  auto viewer = guik::LightViewer::instance();
  if (!vertical_eraser_has_footprint) {
    viewer->remove_drawable("vertical_eraser_footprint");
    return;
  }

  constexpr int segments = 64;
  constexpr double pi = 3.14159265358979323846;
  std::vector<Eigen::Vector3f> circle;
  circle.reserve(segments);
  for (int i = 0; i < segments; ++i) {
    const double angle = 2.0 * pi * static_cast<double>(i) / segments;
    circle.emplace_back(
      static_cast<float>(vertical_eraser_center.x() + vertical_eraser_radius_m * std::cos(angle)),
      static_cast<float>(vertical_eraser_center.y() + vertical_eraser_radius_m * std::sin(angle)),
      static_cast<float>(vertical_eraser_center.z()));
  }
  auto footprint = std::make_shared<glk::ThinLines>(circle, true);
  footprint->set_line_width(2.0f);
  viewer->update_drawable("vertical_eraser_footprint", footprint, guik::FlatColor(1.0f, 0.25f, 0.1f, 1.0f));
}

std::shared_ptr<glim::GlobalMapping> OfflineViewer::load_map(guik::ProgressInterface& progress, const std::string& path, std::shared_ptr<GlobalMapping> global_mapping) {
  progress.set_title("Load map");
  progress.set_text("Now loading");
  progress.set_maximum(1);

  if (global_mapping == nullptr) {  // if no map is loaded yet initialize new GlobalMapping
    glim::GlobalMappingParams params;
    params.isam2_relinearize_skip = 1;
    params.isam2_relinearize_thresh = 0.0;

    if (!export_map_path.empty()) {
      params.enable_optimization = false;
    } else {
      const auto result = pfd::message("Confirm", "Do optimization?", pfd::choice::yes_no).result();
      params.enable_optimization = (result == pfd::button::ok) || (result == pfd::button::yes);
    }

    logger->info("enable_optimization={}", params.enable_optimization);
    global_mapping.reset(new glim::GlobalMapping(params));
  }

  const bool loaded = pending_open_additional ? async_global_mapping->load(path) : global_mapping->load(path);
  if (!loaded) {
    logger->error("failed to load {}", path);
    return nullptr;
  }

  return global_mapping;
}

bool OfflineViewer::save_map(guik::ProgressInterface& progress, const std::string& path) {
  progress.set_title("Save map");
  progress.set_text("Now saving");
  async_global_mapping->save(path);
  return true;
}

bool OfflineViewer::export_map(guik::ProgressInterface& progress, const std::string& path) {
  progress.set_title("Export points");
  progress.set_text("Concatenating submaps");
  progress.set_maximum(3);
  progress.increment();
  auto points = async_global_mapping->export_points();

  if (!points || !points->has_points()) {
    logger->warn("No points available for export");
    return false;
  }

  progress.set_text("Writing to file");
  progress.increment();

  glk::PLYData ply;
  ply.vertices.reserve(points->size());
  const bool has_intensities = points->has_intensities();
  if (has_intensities) {
    ply.intensities.reserve(points->size());
  }

  for (size_t i = 0; i < points->size(); ++i) {
    ply.vertices.push_back(points->points[i].head<3>().cast<float>());
    if (has_intensities) {
      ply.intensities.push_back(points->intensities[i]);
    }
  }

  glk::save_ply_binary(path, ply);
  return true;
}

}  // namespace glim
