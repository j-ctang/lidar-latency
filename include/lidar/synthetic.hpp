#pragma once
#include <cstdint>
#include "lidar/types.hpp"

namespace lidar {

struct SceneParams {
  int num_points = 30000;   // roughly a nuScenes LIDAR_TOP sweep
  int num_boxes = 12;
  float range = 40.0f;      // ground extends +/- range metres
  float ground_z = -1.8f;   // sensor sits ~1.8 m above road
  float noise = 0.02f;      // gaussian sigma, metres
  float ground_fraction = 0.6f;
};

// Ground plane + axis-aligned boxes (cars/poles) + gaussian noise.
// Deterministic for a given seed.
PointCloud generate_scene(std::uint32_t seed, const SceneParams& p = {});

}  // namespace lidar
