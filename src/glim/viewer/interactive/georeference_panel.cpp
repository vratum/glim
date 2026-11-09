#include <glim/viewer/interactive/georeference_panel.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <regex>
#include <imgui.h>
#include <guik/viewer/light_viewer.hpp>

namespace glim {
namespace {
const ImVec4 warning(1.0f, 0.72f, 0.25f, 1.0f);
const ImVec4 good(0.3f, 0.9f, 0.55f, 1.0f);
}

void GeoreferencePanel::clear() {
  guik::LightViewer::instance()->remove_drawable(std::regex("georef_.*"));
  model = Georeference();
  geometry.clear();
  submaps.clear();
  cached_clouds.clear();
  map_path.clear();
  message.clear();
  load_error.clear();
  deleted.reset();
  inspected.reset();
  selected = -1;
  current = false;
  markers_dirty = true;
  editor_requested = false;
  editing_index = -1;
  pick_mode = PickMode::NONE;
}

void GeoreferencePanel::open_map(const std::string& path, bool additional) {
  if (additional) {
    current = false;
    model.invalidate();
    message = "Maps merged. Existing controls stay on their original submaps; recalculate and Save Map.";
    return;
  }
  clear();
  map_path = path;
  if (std::filesystem::exists(map_path / "georeference.json")) {
    try { model = Georeference::load(map_path / "georeference.json"); }
    catch (const std::exception& error) { load_error = error.what(); }
  }
  markers_dirty = true;
}

void GeoreferencePanel::save_map(const std::string& path) {
  if (!load_error.empty()) return;
  map_path = path;
  // Save Map may run one last graph optimization before writing the dump.
  // Its pose callbacks have been applied before this method is dispatched.
  if (model.solution && !model.current(geometry)) {
    try { model.fit(geometry); current = true; }
    catch (const std::exception& error) { current = false; message = error.what(); }
  }
  persist();
}

void GeoreferencePanel::update_geometry(const std::vector<SubMap::ConstPtr>& maps, const std::vector<Eigen::Isometry3d>& poses) {
  submaps = maps;
  GeoreferenceGeometry next;
  for (std::size_t i = 0; i < maps.size() && i < poses.size(); ++i) {
    const auto& map = maps[i];
    if (!map->frame) continue;
    std::string cloud;
    if (cached_clouds[map->id] == map->frame.get() && geometry.count(map->id)) cloud = geometry.at(map->id).cloud_id;
    else cloud = Georeference::cloud_id(*map->frame);
    cached_clouds[map->id] = map->frame.get();
    next[map->id] = {poses[i], cloud};
  }
  geometry = std::move(next);
  current = model.current(geometry);
  if (model.solution && !current) message = "Map geometry changed. Recalculate the geographic alignment.";
  inspected.reset();
  markers_dirty = true;
}

void GeoreferencePanel::persist() {
  if (map_path.empty() || !load_error.empty()) return;
  try {
    model.save(map_path / "georeference.json");
    message = "Coordinates saved. Map optimization/merging also requires File > Save > Save Map.";
  } catch (const std::exception& error) { message = std::string("Could not save coordinates: ") + error.what(); }
}

void GeoreferencePanel::changed() {
  model.invalidate();
  current = false;
  pick_mode = PickMode::NONE;
  inspected.reset();
  markers_dirty = true;
  persist();
}

void GeoreferencePanel::edit(const GeographicControlPoint& point, int index) {
  editing = point;
  editing_index = index;
  std::snprintf(editing_label.data(), editing_label.size(), "%s", point.label.c_str());
  editor_requested = true;
}

void GeoreferencePanel::pick() {
  auto& io = ImGui::GetIO();
  if (pick_mode == PickMode::NONE || io.WantCaptureMouse || !ImGui::IsMouseClicked(0)) return;
  const Eigen::Vector2i pixel(static_cast<int>(io.MousePos.x), static_cast<int>(io.MousePos.y));
  auto viewer = guik::LightViewer::instance();
  const auto info = viewer->pick_info(pixel, 0);
  const float depth = viewer->pick_depth(pixel, 0);
  if (info[0] != 1 || !std::isfinite(depth) || depth <= 0 || depth >= 1 || !geometry.count(info[3])) {
    message = "Click a visible map point. Empty space and coordinate axes cannot be selected.";
    return;
  }
  const auto found = std::find_if(submaps.begin(), submaps.end(), [&](const auto& submap) { return submap->id == info[3]; });
  if (found == submaps.end() || !(*found)->frame || !(*found)->frame->size()) return;
  const auto& submap = geometry.at(info[3]);
  const Eigen::Vector3d approximate = submap.pose.inverse() * viewer->unproject(pixel, depth).cast<double>();
  // Resolve the framebuffer sample to an actual cloud point in double precision.
  Eigen::Vector3d local = approximate;
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < (*found)->frame->size(); ++i) {
    const Eigen::Vector3d candidate = (*found)->frame->points[i].head<3>();
    const double distance = (candidate - approximate).squaredNorm();
    if (candidate.allFinite() && distance < nearest) { local = candidate; nearest = distance; }
  }
  if (!std::isfinite(nearest)) return;
  if (pick_mode == PickMode::INSPECT) {
    try { if (current) inspected = model.solution->to_wgs84(submap.pose * local); }
    catch (const std::exception& error) { message = error.what(); }
  } else if (pick_mode == PickMode::MOVE && selected >= 0 && selected < static_cast<int>(model.points.size())) {
    auto& point = model.points[selected];
    point.submap_id = info[3];
    point.cloud_id = submap.cloud_id;
    point.point_in_submap = local;
    changed();
  } else {
    GeographicControlPoint point;
    point.id = "gcp-" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    point.label = "Point " + std::to_string(model.points.size() + 1);
    point.submap_id = info[3];
    point.cloud_id = submap.cloud_id;
    point.point_in_submap = local;
    if (!model.points.empty()) point.wgs84 = model.points.back().wgs84;
    edit(point, -1);
  }
  pick_mode = PickMode::NONE;
}

void GeoreferencePanel::draw_editor() {
  if (editor_requested) { ImGui::OpenPopup("WGS84 control point"); editor_requested = false; }
  if (!ImGui::BeginPopupModal("WGS84 control point", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  ImGui::TextUnformatted("Coordinates in decimal degrees (WGS84)");
  ImGui::InputText("Name", editing_label.data(), editing_label.size());
  ImGui::InputDouble("Latitude", &editing.wgs84.latitude, 0, 0, "%.8f");
  ImGui::InputDouble("Longitude", &editing.wgs84.longitude, 0, 0, "%.8f");
  ImGui::InputDouble("Horizontal accuracy (m)", &editing.accuracy_m, 1, 5, "%.1f");
  ImGui::TextDisabled("Use 10 m for approximate manual coordinates.");
  ImGui::TextDisabled("Accuracy is one standard deviation per horizontal axis.");
  const bool valid = std::isfinite(editing.wgs84.latitude) && std::abs(editing.wgs84.latitude) <= 90 &&
    std::isfinite(editing.wgs84.longitude) && std::abs(editing.wgs84.longitude) <= 180 &&
    std::isfinite(editing.accuracy_m) && editing.accuracy_m > 0;
  if (!valid) ImGui::TextColored(warning, "Check latitude, longitude and positive accuracy.");
  ImGui::BeginDisabled(!valid);
  if (ImGui::Button(editing_index < 0 ? "Add coordinate" : "Apply changes", ImVec2(160,0))) {
    editing.label = editing_label.data();
    if (editing_index < 0) { model.points.push_back(editing); selected = static_cast<int>(model.points.size()) - 1; }
    else model.points.at(editing_index) = editing;
    changed();
    ImGui::CloseCurrentPopup();
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel", ImVec2(100,0))) ImGui::CloseCurrentPopup();
  ImGui::EndPopup();
}

void GeoreferencePanel::draw_markers() {
  auto viewer = guik::LightViewer::instance();
  if (markers_dirty) {
    viewer->remove_drawable(std::regex("georef_.*"));
    for (std::size_t i = 0; i < model.points.size(); ++i) {
      const auto& point = model.points[i];
      if (!geometry.count(point.submap_id) || geometry.at(point.submap_id).cloud_id != point.cloud_id) continue;
      const Eigen::Vector3d world = geometry.at(point.submap_id).pose * point.point_in_submap;
      const Eigen::Affine3f transform = Eigen::Translation3f(world.cast<float>()) * Eigen::UniformScaling<float>(selected == static_cast<int>(i) ? 0.35f : 0.22f);
      const Eigen::Vector4f color = point.enabled ? Eigen::Vector4f(1,0.7f,0.1f,1) : Eigen::Vector4f(0.5f,0.5f,0.5f,1);
      viewer->update_sphere("georef_" + point.id, guik::FlatColor(color, transform).add("info_values", Eigen::Vector4i(0,0,0,0)));
    }
    markers_dirty = false;
  }
  const Eigen::Matrix4f camera = viewer->projection_matrix() * viewer->view_matrix();
  const auto size = viewer->canvas_size();
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    const auto& point = model.points[i];
    if (!geometry.count(point.submap_id)) continue;
    const Eigen::Vector3d world = geometry.at(point.submap_id).pose * point.point_in_submap;
    const Eigen::Vector4f clip = camera * Eigen::Vector4f(world.x(),world.y(),world.z(),1);
    if (clip.w() <= 0) continue;
    const Eigen::Vector3f ndc = clip.head<3>() / clip.w();
    if (ndc.cwiseAbs().maxCoeff() > 1) continue;
    const ImVec2 screen((ndc.x()+1)*0.5f*size.x()+8, (1-ndc.y())*0.5f*size.y());
    ImGui::GetBackgroundDrawList()->AddText(screen, IM_COL32(255,200,60,255), (std::to_string(i+1) + ": " + point.label).c_str());
  }
}

void GeoreferencePanel::draw() {
  if (map_path.empty()) return;
  pick();
  draw_markers();
  ImGui::SetNextWindowSize(ImVec2(620,510), ImGuiCond_FirstUseEver);
  if (ImGui::Begin("Georeferencing - WGS84")) {
    ImGui::TextWrapped("Add controls on recognisable map features. Approximate manual coordinates are expected (default accuracy: 10 m).");
    if (!load_error.empty()) ImGui::TextColored(warning, "Could not load georeference.json: %s", load_error.c_str());
    ImGui::BeginDisabled(!load_error.empty() || geometry.empty());
    if (ImGui::Button("Add coordinate", ImVec2(150,32))) pick_mode = PickMode::ADD;
    ImGui::SameLine();
    ImGui::BeginDisabled(selected < 0);
    if (ImGui::Button("Delete coordinate", ImVec2(150,32))) {
      deleted = model.points.at(selected);
      model.points.erase(model.points.begin() + selected);
      selected = -1;
      changed();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!deleted);
    if (ImGui::Button("Undo delete")) { model.points.push_back(*deleted); deleted.reset(); selected = model.points.size()-1; changed(); }
    ImGui::EndDisabled();
    if (pick_mode != PickMode::NONE) {
      ImGui::TextColored(warning, "Left-click a map point (%s).", pick_mode == PickMode::ADD ? "add coordinate" : pick_mode == PickMode::MOVE ? "move selected control" : "inspect WGS84");
      ImGui::SameLine();
      if (ImGui::SmallButton("Cancel picking")) pick_mode = PickMode::NONE;
    }
    if (ImGui::BeginTable("controls", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable, ImVec2(0,180))) {
      ImGui::TableSetupColumn("Control"); ImGui::TableSetupColumn("Latitude"); ImGui::TableSetupColumn("Longitude"); ImGui::TableSetupColumn("Error / accuracy (m)");
      ImGui::TableHeadersRow();
      for (std::size_t i = 0; i < model.points.size(); ++i) {
        auto& p = model.points[i];
        ImGui::PushID(p.id.c_str());
        ImGui::TableNextRow(); ImGui::TableNextColumn();
        if (ImGui::Selectable((std::to_string(i+1) + ": " + p.label + (p.enabled ? "" : " [off]")).c_str(), selected == static_cast<int>(i), ImGuiSelectableFlags_SpanAllColumns)) { selected = i; markers_dirty = true; }
        ImGui::TableNextColumn(); ImGui::Text("%.7f", p.wgs84.latitude);
        ImGui::TableNextColumn(); ImGui::Text("%.7f", p.wgs84.longitude);
        ImGui::TableNextColumn();
        auto residual = model.solution ? std::find_if(model.solution->residuals.begin(),model.solution->residuals.end(),[&](const auto& r) { return r.point_id == p.id; }) : std::vector<GeoreferenceResidual>::const_iterator();
        if (current && residual != model.solution->residuals.end()) ImGui::TextColored(residual->high_residual ? warning : ImGui::GetStyleColorVec4(ImGuiCol_Text), "%.1f / %.1f%s", residual->metres, p.accuracy_m, residual->high_residual ? " !" : "");
        else ImGui::Text("-- / %.1f", p.accuracy_m);
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
    ImGui::BeginDisabled(selected < 0);
    if (ImGui::Button("Edit selected")) edit(model.points.at(selected), selected);
    ImGui::SameLine();
    if (ImGui::Button("Move on map")) pick_mode = PickMode::MOVE;
    if (selected >= 0) {
      ImGui::SameLine();
      if (ImGui::Checkbox("Use in alignment", &model.points.at(selected).enabled)) changed();
    }
    ImGui::EndDisabled();
    ImGui::Separator();
    if (ImGui::Button("Calculate alignment", ImVec2(170,30))) {
      try { model.fit(geometry); current = true; persist(); }
      catch (const std::exception& error) { current = false; message = error.what(); }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save coordinates", ImVec2(150,30))) persist();
    ImGui::SameLine();
    ImGui::BeginDisabled(!current);
    if (ImGui::Button("Inspect WGS84", ImVec2(130,30))) pick_mode = PickMode::INSPECT;
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    if (current) {
      const auto& s = *model.solution;
      ImGui::TextColored(good, "Alignment available: RMSE %.1f m, max %.1f m", s.rmse_m, s.max_error_m);
      ImGui::Text("Control spread %.1f m; heading uncertainty ~%.1f deg", s.baseline_m, std::sqrt(s.covariance(2,2))*180/M_PI);
      if (s.weak_geometry) ImGui::TextColored(warning, "Weak geometry: add more controls farther apart (4-6 recommended).");
      ImGui::TextWrapped("Errors are agreement with your controls, not proven GNSS accuracy. A common coordinate offset cannot be detected.");
      ImGui::TextWrapped("Large residuals (!) receive less influence. Disable a control to exclude it completely.");
    } else ImGui::TextColored(warning, "Alignment unavailable / needs calculation.");
    if (inspected) ImGui::Text("Selected point: latitude %.8f, longitude %.8f", inspected->latitude, inspected->longitude);
    if (!message.empty()) ImGui::TextWrapped("%s", message.c_str());
  }
  ImGui::End();
  draw_editor();
}
}  // namespace glim
