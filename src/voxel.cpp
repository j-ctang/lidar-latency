#include <cmath>
#include <map>
#include <tuple>

#include "lidar/pipeline.hpp"

namespace lidar {

PointCloud voxel_downsample(const PointCloud& in, float leaf) {
  struct Acc { double x = 0, y = 0, z = 0, i = 0; int n = 0; };
  std::map<std::tuple<int, int, int>, Acc> grid;
  for (const Point& p : in) {
    auto key = std::make_tuple(static_cast<int>(std::floor(p.x / leaf)),
                               static_cast<int>(std::floor(p.y / leaf)),
                               static_cast<int>(std::floor(p.z / leaf)));
    Acc& a = grid[key];
    a.x += p.x; a.y += p.y; a.z += p.z; a.i += p.intensity; ++a.n;
  }
  PointCloud out;
  out.reserve(grid.size());
  for (const auto& kv : grid) {
    const Acc& a = kv.second;
    out.push_back({static_cast<float>(a.x / a.n), static_cast<float>(a.y / a.n),
                   static_cast<float>(a.z / a.n), static_cast<float>(a.i / a.n)});
  }
  return out;
}

}  // namespace lidar
