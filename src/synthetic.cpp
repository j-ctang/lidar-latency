#include "lidar/synthetic.hpp"

#include <random>

namespace lidar {

PointCloud generate_scene(std::uint32_t seed, const SceneParams& p) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> u(-p.range, p.range);
  std::uniform_real_distribution<float> u01(0.f, 1.f);
  std::normal_distribution<float> n(0.f, p.noise);

  PointCloud cloud;
  cloud.reserve(p.num_points);
  const int n_ground = static_cast<int>(p.num_points * p.ground_fraction);
  for (int i = 0; i < n_ground; ++i)
    cloud.push_back({u(rng), u(rng), p.ground_z + n(rng), u01(rng)});

  struct Box { float cx, cy, sx, sy, h; };
  std::vector<Box> boxes;
  std::uniform_real_distribution<float> pos(-p.range * 0.8f, p.range * 0.8f);
  std::uniform_real_distribution<float> sz(0.5f, 4.5f), ht(1.0f, 2.5f);
  for (int b = 0; b < p.num_boxes; ++b)
    boxes.push_back({pos(rng), pos(rng), sz(rng), sz(rng), ht(rng)});

  const int n_obj = p.num_points - n_ground;
  for (int i = 0; i < n_obj; ++i) {
    const Box& b = boxes[i % boxes.size()];
    // Sample surface-ish: uniform inside the box volume is fine for clustering.
    cloud.push_back({b.cx + (u01(rng) - 0.5f) * b.sx + n(rng),
                     b.cy + (u01(rng) - 0.5f) * b.sy + n(rng),
                     p.ground_z + 0.3f + u01(rng) * b.h + n(rng), u01(rng)});
  }
  return cloud;
}

}  // namespace lidar
