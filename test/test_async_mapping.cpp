#include <glim/mapping/async_global_mapping.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/config.hpp>
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

using namespace glim;

class Mapping final : public GlobalMappingBase {
public:
  void optimize() override {
    enter();
    ++optimizations;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    --active;
  }
  bool load(const std::string&) override {
    enter();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    --active;
    return true;
  }
  std::atomic_int optimizations{0};
private:
  void enter() { assert(active.fetch_add(1) == 0); }
  std::atomic_int active{0};
};

int main(int argc, char** argv) {
  assert(argc == 2);
  GlobalConfig::instance(argv[1]);
  for (int cycle = 0; cycle < 20; ++cycle) {
    auto mapping = std::make_shared<Mapping>();
    {
      AsyncGlobalMapping worker(mapping, 1000000);
      assert(GlobalMappingCallbacks::request_to_optimize);
      GlobalMappingCallbacks::request_to_optimize();
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (!mapping->optimizations && std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
      assert(mapping->optimizations > 0);
      // Loading an additional map must never overlap an optimizer update.
      assert(worker.load("additional-map"));
    }
    // Closing/reopening must not retain callbacks capturing the deleted worker.
    assert(!GlobalMappingCallbacks::request_to_optimize);
    assert(!GlobalMappingCallbacks::request_to_recover);
    assert(!GlobalMappingCallbacks::request_to_find_overlapping_submaps);
    GlobalMappingCallbacks::request_to_optimize();
    GlobalMappingCallbacks::request_to_recover();
    GlobalMappingCallbacks::request_to_find_overlapping_submaps(0.2);
  }
  std::cout << "async mapping: repeated close/reopen and serialized loading passed\n";
}
