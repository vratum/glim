#include <glim/mapping/implicit_neighbors.hpp>
#include <cassert>

int main() {
  std::vector<glim::ImplicitNeighbor> candidates;
  for (int i = 0; i < 100; ++i) candidates.push_back({i, .99, 95.});
  candidates.push_back({100, .3, 20.});
  candidates.push_back({101, .4, 30.});
  candidates.push_back({102, .2, 99.});
  auto bounded = glim::select_implicit_neighbors(candidates, 102, 100., 6);
  assert(bounded.size() == 6);
  for (int id : {100,101,102}) assert(std::find(bounded.begin(),bounded.end(),id)!=bounded.end());
  assert(glim::select_implicit_neighbors(candidates,102,100.,0).size()==candidates.size());
  std::vector<glim::ImplicitNeighbor> sparse{{1,.8,1.},{2,.7,2.}};
  assert(glim::select_implicit_neighbors(sparse,2,100.,6)==std::vector<int>({1,2}));
  candidates.erase(candidates.end()-3,candidates.end());
  assert(glim::select_implicit_neighbors(candidates,99,100.,6).size()==6);
}
