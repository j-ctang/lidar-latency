#include <gtest/gtest.h>

#include "lidar/pipeline.hpp"

using namespace lidar;

TEST(Voxel, MergesPointsInSameCellToCentroid) {
  PointCloud in = {{0.01f, 0.01f, 0.01f, 1}, {0.09f, 0.09f, 0.09f, 3}};
  PointCloud out = voxel_downsample(in, 0.1f);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].x, 0.05f, 1e-6);
  EXPECT_NEAR(out[0].intensity, 2.0f, 1e-6);
}

TEST(Voxel, KeepsSeparateCellsAndNegativeCoords) {
  // -0.05 and +0.05 straddle zero: floor() must put them in different cells.
  PointCloud in = {{-0.05f, 0, 0, 0}, {0.05f, 0, 0, 0}, {1.0f, 1.0f, 1.0f, 0}};
  EXPECT_EQ(voxel_downsample(in, 0.1f).size(), 3u);
}

TEST(Voxel, EmptyInput) { EXPECT_TRUE(voxel_downsample({}, 0.1f).empty()); }
