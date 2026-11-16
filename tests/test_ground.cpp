#include <gtest/gtest.h>

#include <random>

#include "lidar/pipeline.hpp"

using namespace lidar;

TEST(Ground, RemovesFlatPlaneKeepsObstacle) {
  PointCloud in;
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> u(-10, 10);
  for (int i = 0; i < 500; ++i) in.push_back({u(rng), u(rng), -1.8f, 0});
  for (int i = 0; i < 50; ++i) in.push_back({2.0f, 2.0f, -1.0f + 0.02f * i, 0});
  PointCloud out = remove_ground(in);
  EXPECT_EQ(out.size(), 50u);
  for (const Point& p : out) EXPECT_GT(p.z, -1.5f);
}

TEST(Ground, HandlesTinyInput) {
  PointCloud in = {{0, 0, 0, 0}, {1, 0, 0, 0}};
  EXPECT_EQ(remove_ground(in).size(), 2u);
}

TEST(Ground, RejectsVerticalWallAsGround) {
  // Only a vertical wall: no near-horizontal plane, so nothing is removed.
  PointCloud in;
  for (int i = 0; i < 20; ++i)
    for (int j = 0; j < 20; ++j) in.push_back({5.0f, i * 0.1f, j * 0.1f, 0});
  EXPECT_EQ(remove_ground(in).size(), in.size());
}
