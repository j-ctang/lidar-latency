#include <queue>

#include "lidar/pipeline.hpp"

namespace lidar {

std::vector<Cluster> euclidean_cluster(const PointCloud& in, const ClusterParams& prm) {
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

}  // namespace lidar
