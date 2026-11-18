#pragma once
#include <algorithm>
#include <vector>

namespace glim {
struct ImplicitNeighbor {
  int id;
  double overlap;
  double stamp;
};

// Keep the odometry chain and reserve observations from an earlier visit;
// otherwise a dense recent cluster can consume the entire geometry budget.
inline std::vector<int> select_implicit_neighbors(std::vector<ImplicitNeighbor> candidates,
                                                int previous, double stamp, int limit) {
  if (limit == 0 || candidates.size() <= static_cast<size_t>(limit)) {
    std::vector<int> ids;
    for (const auto& candidate : candidates) ids.push_back(candidate.id);
    return ids;
  }
  std::stable_sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
    return a.overlap != b.overlap ? a.overlap > b.overlap : a.id > b.id;
  });
  std::vector<int> selected;
  const auto add = [&](int id) {
    if (selected.size() < static_cast<size_t>(limit) &&
        std::find(selected.begin(), selected.end(), id) == selected.end()) selected.push_back(id);
  };
  for (const auto& candidate : candidates) if (candidate.id == previous) add(previous);
  int older = 0;
  for (const auto& candidate : candidates) {
    if (stamp - candidate.stamp >= 30.0 && older < 2) { add(candidate.id); ++older; }
  }
  for (const auto& candidate : candidates) add(candidate.id);
  std::sort(selected.begin(), selected.end());
  return selected;
}
}  // namespace glim
