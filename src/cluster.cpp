#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>

#include "lidar/pipeline.hpp"

namespace lidar {

namespace {

std::vector<Cluster> cluster_naive(const PointCloud& in, const ClusterParams& prm) {
  const float tol2 = prm.tolerance * prm.tolerance;
  const int n = static_cast<int>(in.size());
  std::vector<bool> visited(n, false);
  std::vector<Cluster> clusters;
  for (int seed = 0; seed < n; ++seed) {
    if (visited[seed]) continue;
    Cluster c;
    std::queue<int> q;
    q.push(seed);
    visited[seed] = true;
    while (!q.empty()) {
      const int i = q.front();
      q.pop();
      c.push_back(i);
      // Brute-force neighbour scan: the deliberate O(n^2) hot spot.
      for (int j = 0; j < n; ++j) {
        if (visited[j]) continue;
        const float dx = in[i].x - in[j].x, dy = in[i].y - in[j].y,
                    dz = in[i].z - in[j].z;
        if (dx * dx + dy * dy + dz * dz <= tol2) {
          visited[j] = true;
          q.push(j);
        }
      }
    }
    const int sz = static_cast<int>(c.size());
    if (sz >= prm.min_size && sz <= prm.max_size) clusters.push_back(std::move(c));
  }
  return clusters;
}

// 21 bits per axis, offset so negative cells pack into an unsigned key.
std::uint64_t pack(std::int64_t cx, std::int64_t cy, std::int64_t cz) {
  constexpr std::int64_t off = 1 << 20, mask = (1 << 21) - 1;
  return (static_cast<std::uint64_t>((cx + off) & mask) << 42) |
         (static_cast<std::uint64_t>((cy + off) & mask) << 21) |
         static_cast<std::uint64_t>((cz + off) & mask);
}

std::vector<Cluster> cluster_grid(const PointCloud& in, const ClusterParams& prm) {
  const float tol2 = prm.tolerance * prm.tolerance;
  const int n = static_cast<int>(in.size());
  // Cell slightly larger than tolerance so float rounding in x / cell can
  // never put two points within tolerance more than one cell apart.
  const float inv_cell = 1.0f / (prm.tolerance * 1.0001f);

  // Sort point indices by cell key; each cell is then a contiguous run.
  std::vector<std::int64_t> cx(n), cy(n), cz(n);
  std::vector<std::pair<std::uint64_t, int>> keyed(n);
  for (int i = 0; i < n; ++i) {
    cx[i] = static_cast<std::int64_t>(std::floor(in[i].x * inv_cell));
    cy[i] = static_cast<std::int64_t>(std::floor(in[i].y * inv_cell));
    cz[i] = static_cast<std::int64_t>(std::floor(in[i].z * inv_cell));
    keyed[i] = {pack(cx[i], cy[i], cz[i]), i};
  }
  std::sort(keyed.begin(), keyed.end());
  std::vector<int> order(n);
  std::unordered_map<std::uint64_t, std::pair<int, int>> cells;  // key -> [b, e)
  cells.reserve(static_cast<std::size_t>(n));
  for (int k = 0; k < n;) {
    int e = k;
    while (e < n && keyed[e].first == keyed[k].first) { order[e] = keyed[e].second; ++e; }
    cells.emplace(keyed[k].first, std::make_pair(k, e));
    k = e;
  }

  std::vector<bool> visited(n, false);
  std::vector<Cluster> clusters;
  std::vector<int> found;
  for (int seed = 0; seed < n; ++seed) {
    if (visited[seed]) continue;
    Cluster c;
    std::queue<int> q;
    q.push(seed);
    visited[seed] = true;
    while (!q.empty()) {
      const int i = q.front();
      q.pop();
      c.push_back(i);
      found.clear();
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
          for (int dz = -1; dz <= 1; ++dz) {
            auto it = cells.find(pack(cx[i] + dx, cy[i] + dy, cz[i] + dz));
            if (it == cells.end()) continue;
            for (int k = it->second.first; k < it->second.second; ++k) {
              const int j = order[k];
              if (visited[j]) continue;
              const float ex = in[i].x - in[j].x, ey = in[i].y - in[j].y,
                          ez = in[i].z - in[j].z;
              if (ex * ex + ey * ey + ez * ez <= tol2) found.push_back(j);
            }
          }
      // Push in ascending index order, as the naive scan does, so the
      // output matches it exactly (not only as a partition).
      std::sort(found.begin(), found.end());
      for (int j : found) {
        visited[j] = true;
        q.push(j);
      }
    }
    const int sz = static_cast<int>(c.size());
    if (sz >= prm.min_size && sz <= prm.max_size) clusters.push_back(std::move(c));
  }
  return clusters;
}

}  // namespace

std::vector<Cluster> euclidean_cluster(const PointCloud& in, const ClusterParams& prm) {
  return prm.search == NeighborSearch::Naive ? cluster_naive(in, prm)
                                             : cluster_grid(in, prm);
}

}  // namespace lidar
