#include <cmath>
#include <random>

#include "lidar/pipeline.hpp"

namespace lidar {

PointCloud remove_ground(const PointCloud& in, const GroundParams& prm) {
  if (in.size() < 3) return in;
  std::mt19937 rng(prm.seed);
  std::uniform_int_distribution<std::size_t> pick(0, in.size() - 1);

  float best[4] = {0, 0, 1, 0};
  std::size_t best_inliers = 0;
  for (int it = 0; it < prm.iterations; ++it) {
    const Point &a = in[pick(rng)], &b = in[pick(rng)], &c = in[pick(rng)];
    const float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    const float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    float nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-6f) continue;  // degenerate sample
    nx /= len; ny /= len; nz /= len;
    if (std::fabs(nz) < 0.9f) continue;  // ground must be near-horizontal
    const float d = -(nx * a.x + ny * a.y + nz * a.z);
    std::size_t inliers = 0;
    for (const Point& p : in)
      if (std::fabs(nx * p.x + ny * p.y + nz * p.z + d) < prm.distance) ++inliers;
    if (inliers > best_inliers) {
      best_inliers = inliers;
      best[0] = nx; best[1] = ny; best[2] = nz; best[3] = d;
    }
  }
  if (best_inliers == 0) return in;

  PointCloud out;
  out.reserve(in.size() - best_inliers);
  for (const Point& p : in)
    if (std::fabs(best[0] * p.x + best[1] * p.y + best[2] * p.z + best[3]) >= prm.distance)
      out.push_back(p);
  return out;
}

}  // namespace lidar
