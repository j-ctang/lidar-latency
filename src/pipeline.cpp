#include "lidar/pipeline.hpp"

#include <chrono>

namespace lidar {

namespace {
using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
}  // namespace

FrameResult run_pipeline(const PointCloud& in, const PipelineParams& p) {
  FrameResult r;
  r.in_points = in.size();
  const auto t_start = Clock::now();

  auto t0 = Clock::now();
  PointCloud vox = voxel_downsample(in, p.leaf);
  r.t.voxel_ms = ms_since(t0);

  t0 = Clock::now();
  PointCloud obj = remove_ground(vox, p.ground);
  r.t.ground_ms = ms_since(t0);

  t0 = Clock::now();
  auto clusters = euclidean_cluster(obj, p.cluster);
  r.t.cluster_ms = ms_since(t0);

  r.t.total_ms = ms_since(t_start);
  r.voxel_points = vox.size();
  r.nonground_points = obj.size();
  r.clusters = clusters.size();
  return r;
}

}  // namespace lidar
