#pragma once
#include <vector>

namespace lidar {

struct Point {
  float x = 0, y = 0, z = 0, intensity = 0;
};

using PointCloud = std::vector<Point>;
using Cluster = std::vector<int>;  // indices into a cloud

}  // namespace lidar
