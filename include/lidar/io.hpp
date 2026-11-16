#pragma once
#include <string>
#include "lidar/types.hpp"

namespace lidar {

// Loads a nuScenes LIDAR_TOP .pcd.bin: float32 x,y,z,intensity,ring per point.
// Throws std::runtime_error if the file cannot be read or its size is not a
// multiple of 5 floats. Ring index is dropped.
PointCloud load_nuscenes_bin(const std::string& path);

}  // namespace lidar
