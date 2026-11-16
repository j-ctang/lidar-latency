#pragma once
#include <cstdint>
#include <vector>
#include "lidar/types.hpp"

namespace lidar {

// Naive baseline: std::map keyed by voxel index, centroid per voxel.
PointCloud voxel_downsample(const PointCloud& in, float leaf);

struct GroundParams {
  int iterations = 100;
  float distance = 0.15f;   // inlier threshold, metres
  std::uint32_t seed = 42;
};
// RANSAC plane fit; returns the points that are NOT on the plane.
PointCloud remove_ground(const PointCloud& in, const GroundParams& p = {});

struct ClusterParams {
  float tolerance = 0.5f;
  int min_size = 10;
  int max_size = 100000;
};
// Naive Euclidean clustering: BFS with O(n) brute-force neighbour scan, so
// O(n^2) per frame. Clusters outside [min_size, max_size] are dropped.
std::vector<Cluster> euclidean_cluster(const PointCloud& in,
                                       const ClusterParams& p = {});

struct PipelineParams {
  float leaf = 0.2f;
  GroundParams ground;
  ClusterParams cluster;
};

struct StageTimes {
  double voxel_ms = 0, ground_ms = 0, cluster_ms = 0, total_ms = 0;
};

struct FrameResult {
  StageTimes t;
  std::size_t in_points = 0, voxel_points = 0, nonground_points = 0;
  std::size_t clusters = 0;
};

FrameResult run_pipeline(const PointCloud& in, const PipelineParams& p = {});

}  // namespace lidar
