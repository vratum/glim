#include <glim/mapping/async_global_mapping.hpp>

#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <stdexcept>

#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>
#include <glim/mapping/callbacks.hpp>

namespace glim {

AsyncGlobalMapping::AsyncGlobalMapping(const std::shared_ptr<glim::GlobalMappingBase>& global_mapping, const int optimization_interval)
: global_mapping(global_mapping),
  optimization_interval(optimization_interval),
  logger(create_module_logger("global")) {
  Config config(GlobalConfig::get_config_path("config_global_mapping"));
  const int online_iterations = config.param<int>("global_mapping", "online_optimization_iterations", 8);
  const int online_interval_ms = config.param<int>("global_mapping", "online_optimization_interval_ms", 20);
  if (online_iterations < 1 || online_iterations > 100 || online_interval_ms < 0 || online_interval_ms > 1000) {
    throw std::invalid_argument("online optimization requires iterations in [1,100] and interval_ms in [0,1000]");
  }
  request_to_optimize = false;
  request_to_recover = false;
  request_to_find_overlapping_submaps.store(-1.0);

  GlobalMappingCallbacks::request_to_optimize.add([this] { request_to_optimize = true; });
  GlobalMappingCallbacks::request_to_recover.add([this] { request_to_recover = true; });
  GlobalMappingCallbacks::request_to_find_overlapping_submaps.add([this](double min_overlap) { request_to_find_overlapping_submaps.store(min_overlap); });

  kill_switch = false;
  end_of_sequence = false;
  thread = std::thread([this, online_iterations, online_interval_ms] { run(online_iterations, online_interval_ms); });
}

AsyncGlobalMapping::~AsyncGlobalMapping() {
  kill_switch = true;
  join();
}

#ifdef GLIM_USE_OPENCV
void AsyncGlobalMapping::insert_image(const double stamp, const cv::Mat& image) {
  input_image_queue.push_back(std::make_pair(stamp, image));
}
#endif

void AsyncGlobalMapping::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Eigen::Matrix<double, 7, 1> imu_data;
  imu_data << stamp, linear_acc, angular_vel;
  input_imu_queue.push_back(imu_data);
}

void AsyncGlobalMapping::insert_submap(const SubMap::Ptr& submap) {
  input_submap_queue.push_back(submap);
}

void AsyncGlobalMapping::join() {
  end_of_sequence = true;
  if (thread.joinable()) {
    thread.join();
    GlobalMappingCallbacks::on_before_final_optimization();
    // A late loop can require several nonlinear updates after the final input.
    // Drain those updates before saving; join() may also be called again by the
    // destructor, so only finalize when the worker was actually joined here.
    Config config(GlobalConfig::get_config_path("config_global_mapping"));
    const int final_updates = config.param<int>("global_mapping", "final_optimization_iterations", 0);
    if (final_updates < 0 || final_updates > 100) {
      throw std::invalid_argument("final_optimization_iterations must be in [0,100]");
    }
    for (int i = 0; i < final_updates; ++i) {
      logger->info("final global optimization update {}/{}", i+1, final_updates);
      global_mapping->optimize();
    }
  }
}

int AsyncGlobalMapping::workload() const {
  return input_submap_queue.size();
}

void AsyncGlobalMapping::save(const std::string& path) {
  logger->info("saving to {}...", path);
  std::lock_guard<std::mutex> lock(global_mapping_mutex);
  global_mapping->save(path);
  logger->info("saved");
}

bool AsyncGlobalMapping::replay_submaps() {
  std::lock_guard<std::mutex> lock(global_mapping_mutex);
  return global_mapping->replay_submaps();
}

gtsam_points::PointCloud::Ptr AsyncGlobalMapping::export_points() {
  std::lock_guard<std::mutex> lock(global_mapping_mutex);
  logger->info("exporting points");
  logger->info("global_mapping: {}", global_mapping.get() == nullptr);
  auto points = global_mapping->export_points();
  return points;
}

void AsyncGlobalMapping::run(int online_iterations, int online_interval_ms) {
  using Clock = std::chrono::steady_clock;
  auto last_optimization_time = Clock::now();
  auto next_online_update = last_optimization_time;
  int remaining_online_updates = 0;

  while (!kill_switch) {
#ifdef GLIM_USE_OPENCV
    auto images = input_image_queue.get_all_and_clear();
#endif
    auto imu_frames = input_imu_queue.get_all_and_clear();
    auto submaps = input_submap_queue.get_all_and_clear();

    const bool idle =
#ifdef GLIM_USE_OPENCV
      images.empty() &&
#endif
      imu_frames.empty() && submaps.empty();
    if (idle && end_of_sequence) break;

    if (!idle) {
      std::lock_guard<std::mutex> lock(global_mapping_mutex);
      for (const auto& imu : imu_frames) {
        global_mapping->insert_imu(imu[0], imu.block<3, 1>(1, 0), imu.block<3, 1>(4, 0));
      }
#ifdef GLIM_USE_OPENCV
      for (const auto& image : images) global_mapping->insert_image(image.first, image.second);
#endif
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
        // Inserting a submap updates the graph. Merely receiving IMU does not.
        last_optimization_time = Clock::now();
      }
    }

    // Requests and periodic refinement must not depend on an empty input queue.
    // exchange() also preserves requests raised while optimize() is executing.
    const double min_overlap = request_to_find_overlapping_submaps.exchange(-1.0);
    const bool recover = request_to_recover.exchange(false);
    const bool requested = request_to_optimize.exchange(false);
    if (requested) {
      remaining_online_updates = std::max(remaining_online_updates, online_iterations);
      next_online_update = Clock::now();
    }
    const auto now = Clock::now();
    const bool online_due = remaining_online_updates && now >= next_online_update;
    const bool periodic_due = now - last_optimization_time >= std::chrono::seconds(optimization_interval);
    if (min_overlap > 0.0 || recover || online_due || periodic_due) {
      std::lock_guard<std::mutex> lock(global_mapping_mutex);
      if (min_overlap > 0.0) global_mapping->find_overlapping_submaps(min_overlap);
      if (recover) global_mapping->recover_graph();
      if (online_due || periodic_due) {
        global_mapping->optimize();
        if (online_due) {
          --remaining_online_updates;
          logger->info("online global optimization update; remaining={}", remaining_online_updates);
        }
      }
      last_optimization_time = Clock::now();
      next_online_update = last_optimization_time + std::chrono::milliseconds(online_interval_ms);
    }
    // One refinement per pass, with input consumption between refinements.
    if (idle) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace glim
