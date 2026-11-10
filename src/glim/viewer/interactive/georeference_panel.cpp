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
const ImVec4 bad(1.0f, 0.25f, 0.25f, 1.0f);
}

void GeoreferencePanel::clear() {
  guik::LightViewer::instance()->remove_drawable(std::regex("georef_.*"));
  model = Georeference();
  geometry.clear();
  control_diagnostics.clear();
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
    control_diagnostics.clear();
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
  refresh_diagnostics();
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
  refresh_diagnostics();
  if (model.solution && !current) message = "Map geometry changed. Recalculate the geographic alignment.";
  inspected.reset();
  markers_dirty = true;
}

void GeoreferencePanel::persist() {
  if (map_path.empty() || !load_error.empty()) return;
  try {
    model.save(map_path / "georeference.json");
    message = "Coordinates saved.";
  } catch (const std::exception& error) { message = std::string("Could not save coordinates: ") + error.what(); }
}

void GeoreferencePanel::changed() {
  model.invalidate();
  current = false;
  control_diagnostics.clear();
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

const GeoreferenceDiagnostic* GeoreferencePanel::diagnostic(const std::string& id) const {
  const auto found = std::find_if(control_diagnostics.begin(), control_diagnostics.end(), [&](const auto& d) { return d.point_id == id; });
  return found == control_diagnostics.end() ? nullptr : &*found;
}

void GeoreferencePanel::refresh_diagnostics() {
  control_diagnostics = current ? model.diagnostics(geometry) : std::vector<GeoreferenceDiagnostic>();
  markers_dirty = true;
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
  const bool valid = std::isfinite(editing.wgs84.latitude) && std::abs(editing.wgs84.latitude) <= 90 &&
    std::isfinite(editing.wgs84.longitude) && std::abs(editing.wgs84.longitude) <= 180;
  if (!valid) ImGui::TextColored(warning, "Check latitude [-90,90] and longitude [-180,180].");
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
      const auto check = diagnostic(point.id);
      const bool suspect = check && check->suspect;
      const Eigen::Vector3d world = geometry.at(point.submap_id).pose * point.point_in_submap;
      const Eigen::Affine3f transform = Eigen::Translation3f(world.cast<float>()) * Eigen::UniformScaling<float>(suspect ? 0.45f : selected == static_cast<int>(i) ? 0.35f : 0.22f);
      const Eigen::Vector4f color = !point.enabled ? Eigen::Vector4f(0.5f,0.5f,0.5f,1) : suspect ? Eigen::Vector4f(1,0.15f,0.15f,1) : Eigen::Vector4f(1,0.7f,0.1f,1);
      viewer->update_sphere("georef_" + point.id, guik::FlatColor(color, transform).add("info_values", Eigen::Vector4i(0,0,0,0)));
      if (suspect && check->suggested_position) {
        const Eigen::Vector3f expected = check->suggested_position->cast<float>();
        const Eigen::Affine3f ghost = Eigen::Translation3f(expected) * Eigen::UniformScaling<float>(0.65f);
        viewer->update_sphere("georef_ghost_" + point.id,
          guik::FlatColor(0.15f,1.0f,0.4f,0.3f,ghost).add("info_values",Eigen::Vector4i(0,0,0,0)).make_transparent());
      }
    }
    markers_dirty = false;
  }
  const Eigen::Matrix4f camera = viewer->projection_matrix() * viewer->view_matrix();
  const auto size = viewer->canvas_size();
  const auto project = [&](const Eigen::Vector3d& world) -> std::optional<ImVec2> {
    const Eigen::Vector4f clip = camera * Eigen::Vector4f(world.x(),world.y(),world.z(),1);
    if (clip.w() <= 0) return std::nullopt;
    const Eigen::Vector3f ndc = clip.head<3>() / clip.w();
    if (ndc.cwiseAbs().maxCoeff() > 1) return std::nullopt;
    return ImVec2((ndc.x()+1)*0.5f*size.x(), (1-ndc.y())*0.5f*size.y());
  };
  const auto label = [&](const Eigen::Vector3d& world, const std::string& text, ImU32 color) {
    if (const auto screen = project(world)) ImGui::GetBackgroundDrawList()->AddText(ImVec2(screen->x+8,screen->y),color,text.c_str());
  };
  for (std::size_t i = 0; i < model.points.size(); ++i) {
    const auto& point = model.points[i];
    if (!geometry.count(point.submap_id) || geometry.at(point.submap_id).cloud_id != point.cloud_id) continue;
    const auto check = diagnostic(point.id);
    const bool suspect = check && check->suspect;
    const Eigen::Vector3d world = geometry.at(point.submap_id).pose * point.point_in_submap;
    label(world,std::to_string(i+1) + ": " + point.label,suspect ? IM_COL32(255,65,65,255) : IM_COL32(255,200,60,255));
    if (suspect && check->suggested_position) {
      label(*check->suggested_position,std::to_string(i+1) + ": suggested location",IM_COL32(60,255,115,230));
      const auto from = project(world), to = project(*check->suggested_position);
      if (from && to) {
        const Eigen::Vector2f delta(to->x-from->x,to->y-from->y);
        if (delta.norm() > 20) {
          const Eigen::Vector2f direction = delta.normalized(), side(-direction.y(),direction.x());
          const Eigen::Vector2f start = Eigen::Vector2f(from->x,from->y)+6*direction;
          const Eigen::Vector2f end = Eigen::Vector2f(to->x,to->y)-5*direction;
          const Eigen::Vector2f base = end-12*direction;
          auto draw = ImGui::GetBackgroundDrawList();
          const ImU32 color = IM_COL32(60,255,115,190);
          draw->AddLine(ImVec2(start.x(),start.y()),ImVec2(base.x(),base.y()),color,2);
          draw->AddTriangleFilled(ImVec2(end.x(),end.y()),ImVec2(base.x()+5*side.x(),base.y()+5*side.y()),ImVec2(base.x()-5*side.x(),base.y()-5*side.y()),color);
        }
      }
    }
  }
}

void GeoreferencePanel::draw_info() {
  if (ImGui::SmallButton("(i)##georeference_info")) ImGui::OpenPopup("Georeferencing details");
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("Georeferencing details");
  ImGui::SetNextWindowSize(ImVec2(500,0), ImGuiCond_Appearing);
  if (!ImGui::BeginPopup("Georeferencing details")) return;
  ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x);
  ImGui::TextUnformatted("Georeferencing details");
  ImGui::Separator();
  ImGui::TextWrapped("Add controls on recognisable map features. Calculate alignment to check how well they agree.");
  if (current) {
    const auto& s = *model.solution;
    ImGui::TextColored(good, "Alignment available: RMSE %.1f m, max %.1f m", s.rmse_m, s.max_error_m);
    ImGui::Text("Control spread %.1f m", s.baseline_m);
    if (s.weak_geometry) ImGui::TextColored(warning, "Weak geometry: add more controls farther apart (4-6 recommended).");
    if (std::count_if(model.points.begin(),model.points.end(),[](const auto& p) { return p.enabled; }) < 4) {
      ImGui::TextColored(warning,"Use at least 4 well-spread controls for reliable individual checks.");
    } else if (std::any_of(control_diagnostics.begin(),control_diagnostics.end(),[](const auto& d) { return !d.independent_check; })) {
      ImGui::TextColored(warning,"Some controls lack reliable support from the others.");
    }
  } else ImGui::TextColored(warning, "Alignment unavailable / needs calculation.");
  ImGui::Separator();
  ImGui::TextWrapped("Errors are agreement with your controls, not proven GNSS accuracy. A common coordinate offset cannot be detected.");
  ImGui::TextColored(bad,"Red: inconsistent control");
  ImGui::TextColored(good,"Green ghost: suggested location");
  ImGui::TextWrapped("Suggestions use the other controls. Check both the map position and WGS84; controls are never moved automatically.");
  ImGui::TextWrapped("Inspect WGS84 shows the calculated latitude and longitude of a map point.");
  ImGui::TextWrapped("Coordinates are saved automatically. After map optimization or merging, recalculate and use File > Save > Save Map.");
  ImGui::PopTextWrapPos();
  ImGui::EndPopup();
}

void GeoreferencePanel::draw() {
  if (map_path.empty()) return;
  pick();
  draw_markers();
  ImGui::SetNextWindowSize(ImVec2(620,340), ImGuiCond_FirstUseEver);
  if (ImGui::Begin("Georeferencing - WGS84")) {
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
      ImGui::TableSetupColumn("Control"); ImGui::TableSetupColumn("Latitude"); ImGui::TableSetupColumn("Longitude"); ImGui::TableSetupColumn("Deviation (m)");
      ImGui::TableHeadersRow();
      for (std::size_t i = 0; i < model.points.size(); ++i) {
        auto& p = model.points[i];
        ImGui::PushID(p.id.c_str());
        ImGui::TableNextRow(); ImGui::TableNextColumn();
        if (ImGui::Selectable((std::to_string(i+1) + ": " + p.label + (p.enabled ? "" : " [off]")).c_str(), selected == static_cast<int>(i), ImGuiSelectableFlags_SpanAllColumns)) { selected = i; markers_dirty = true; }
        ImGui::TableNextColumn(); ImGui::Text("%.7f", p.wgs84.latitude);
        ImGui::TableNextColumn(); ImGui::Text("%.7f", p.wgs84.longitude);
        ImGui::TableNextColumn();
        const auto check = diagnostic(p.id);
        if (current && check) {
          ImGui::TextColored(check->suspect ? bad : ImGui::GetStyleColorVec4(ImGuiCol_Text), "%.1f%s", check->deviation_m, check->suspect ? " !" : "");
          if (ImGui::IsItemHovered()) ImGui::SetTooltip(check->independent_check ? "Checked against the other controls, excluding this point." : "Alignment residual; not enough independent support to suggest a correction.");
        } else ImGui::TextUnformatted("--");
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
      try { model.fit(geometry); current = true; refresh_diagnostics(); persist(); }
      catch (const std::exception& error) { current = false; refresh_diagnostics(); message = error.what(); }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save coordinates", ImVec2(150,30))) persist();
    ImGui::SameLine();
    ImGui::BeginDisabled(!current);
    if (ImGui::Button("Inspect WGS84", ImVec2(130,30))) pick_mode = PickMode::INSPECT;
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::SameLine();
    draw_info();
    if (current && selected >= 0) {
      const auto check = diagnostic(model.points.at(selected).id);
      if (check && check->suspect && check->suggested_wgs84) {
        ImGui::Text("Selected control disagrees with others by %.1f m.",check->deviation_m);
        double latitude = check->suggested_wgs84->latitude, longitude = check->suggested_wgs84->longitude;
        ImGui::TextDisabled("Suggested WGS84 at the current map point:");
        ImGui::SetNextItemWidth(210);
        ImGui::InputDouble("Suggested latitude",&latitude,0,0,"%.8f",ImGuiInputTextFlags_ReadOnly);
        ImGui::SetNextItemWidth(210);
        ImGui::InputDouble("Suggested longitude",&longitude,0,0,"%.8f",ImGuiInputTextFlags_ReadOnly);
      }
    }
    if (inspected) ImGui::Text("Selected point: latitude %.8f, longitude %.8f", inspected->latitude, inspected->longitude);
    if (!message.empty()) ImGui::TextWrapped("%s", message.c_str());
  }
  ImGui::End();
  draw_editor();
}
}  // namespace glim
