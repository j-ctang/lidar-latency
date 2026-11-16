#include <gtest/gtest.h>

#include "lidar/pipeline.hpp"
#include "lidar/synthetic.hpp"

using namespace lidar;

TEST(Synthetic, DeterministicAndSized) {
  SceneParams p;
  p.num_points = 5000;
  PointCloud a = generate_scene(7, p), b = generate_scene(7, p);
  ASSERT_EQ(a.size(), 5000u);
  EXPECT_FLOAT_EQ(a[123].x, b[123].x);
}

TEST(Pipeline, FindsObjectsInSmallSyntheticScene) {
  SceneParams sp;
  sp.num_points = 4000;
  sp.num_boxes = 3;
  sp.range = 15.0f;
  FrameResult r = run_pipeline(generate_scene(3, sp));
  EXPECT_EQ(r.in_points, 4000u);
  EXPECT_LT(r.voxel_points, r.in_points);
  EXPECT_LT(r.nonground_points, r.voxel_points);
  EXPECT_GE(r.clusters, 1u);
  EXPECT_LE(r.clusters, 6u);  // boxes may overlap or split, but not explode
  EXPECT_GE(r.t.total_ms, r.t.cluster_ms);
}
