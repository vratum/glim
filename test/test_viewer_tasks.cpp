#include <glim/viewer/interactive_viewer.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/util/config.hpp>
#include <glim/util/trajectory_manager.hpp>
#include <imgui.h>
#include <cassert>
#include <iostream>

using namespace glim;

class Viewer final : public InteractiveViewer {
public:
  Viewer() : InteractiveViewer(false) {}
  using InteractiveViewer::invoke;
  using InteractiveViewer::process_pending_tasks;
  using InteractiveViewer::globalmap_on_insert_submap;
  using InteractiveViewer::globalmap_on_update_submaps;
  using InteractiveViewer::drawable_selection;
  std::size_t size() const { return submaps.size(); }
  int changes = 0;
private:
  void on_submaps_changed() override { ++changes; }
};

int main(int argc, char** argv) {
  assert(argc == 2);
  GlobalConfig::instance(argv[1]);
  TrajectoryManager trajectory;
  Eigen::Isometry3d anchor = Eigen::Isometry3d::Identity();
  anchor.translation().x() = 100;
  // No upper odometry bracket: offline loading must not read past the end.
  trajectory.update_anchor(12345, anchor);
  assert(trajectory.get_T_world_odom().matrix().isApprox(Eigen::Matrix4d::Identity()));
  trajectory.add_odom(1, Eigen::Isometry3d::Identity());
  trajectory.update_anchor(2, anchor);
  assert(trajectory.get_T_world_odom().matrix().isApprox(Eigen::Matrix4d::Identity()));
  trajectory.update_anchor(1, anchor);
  assert(trajectory.get_T_world_odom().matrix().isApprox(anchor.matrix()));
  for (int cycle = 0; cycle < 5; ++cycle) {
    {
      Viewer viewer;
      int completed = 0;
      viewer.invoke([&] {
        ++completed;
        // Executing a task with the queue mutex held used to deadlock here.
        viewer.invoke([&] { ++completed; });
      });
      viewer.process_pending_tasks();
      assert(completed == 1);
      viewer.process_pending_tasks();
      assert(completed == 2);
      std::vector<SubMap::Ptr> maps;
      for (int i = 0; i < 1000; ++i) {
        auto map = std::make_shared<SubMap>();
        map->id = i;
        map->session_id = 0;
        map->T_world_origin.setIdentity();
        auto frame = std::make_shared<EstimationFrame>();
        frame->stamp = i + 100;
        map->frames.push_back(frame);
        viewer.globalmap_on_insert_submap(map);
        maps.push_back(map);
      }
      viewer.process_pending_tasks();
      assert(viewer.size() == 1000 && viewer.changes == 1);
      // Repeated converged optimizer updates must not rebuild the same map.
      for (int i = 0; i < 100; ++i) viewer.globalmap_on_update_submaps(maps);
      viewer.process_pending_tasks();
      assert(viewer.changes == 1);
      maps.back()->T_world_origin.translation().x() = 5;
      viewer.globalmap_on_update_submaps(maps);
      viewer.process_pending_tasks();
      assert(viewer.changes == 2);
      // Collapsing Selection must still balance ImGui::Begin/End.
      ImGui::CreateContext();
      auto& io = ImGui::GetIO();
      io.IniFilename = nullptr;
      io.DisplaySize = ImVec2(800,600);
      io.DeltaTime = 1.0f/60;
      unsigned char* pixels;
      int width, height;
      io.Fonts->GetTexDataAsRGBA32(&pixels,&width,&height);
      ImGui::NewFrame();
      ImGui::SetNextWindowCollapsed(true,ImGuiCond_Always);
      viewer.drawable_selection();
      ImGui::EndFrame();
      ImGui::DestroyContext();
    }
    assert(!OdometryEstimationCallbacks::on_new_frame);
    assert(!GlobalMappingCallbacks::on_insert_submap);
    assert(!GlobalMappingCallbacks::on_update_submaps);
    assert(!GlobalMappingCallbacks::on_smoother_update);
    assert(!GlobalMappingCallbacks::on_smoother_update_result);
  }
  std::cout << "viewer: batched updates, queue reentrancy, trajectory bounds and teardown passed\n";
}
