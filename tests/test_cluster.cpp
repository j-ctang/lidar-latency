#include <gtest/gtest.h>

#include <algorithm>

#include "lidar/pipeline.hpp"

using namespace lidar;

static PointCloud blob(float cx, float cy, int n) {
  PointCloud c;
  for (int i = 0; i < n; ++i) c.push_back({cx + 0.05f * (i % 5), cy + 0.05f * (i / 5), 0, 0});
  return c;
}

TEST(Cluster, SeparatesTwoBlobs) {
  PointCloud in = blob(0, 0, 20);
  PointCloud b = blob(10, 10, 30);
  in.insert(in.end(), b.begin(), b.end());
  auto cl = euclidean_cluster(in, {0.5f, 5, 1000});
  ASSERT_EQ(cl.size(), 2u);
  std::vector<std::size_t> sizes = {cl[0].size(), cl[1].size()};
  std::sort(sizes.begin(), sizes.end());
  EXPECT_EQ(sizes[0], 20u);
  EXPECT_EQ(sizes[1], 30u);
}

TEST(Cluster, ChainsTransitiveNeighbours) {
  PointCloud in;
  for (int i = 0; i < 10; ++i) in.push_back({0.4f * i, 0, 0, 0});  // 0.4 m steps
  auto cl = euclidean_cluster(in, {0.5f, 1, 1000});
  ASSERT_EQ(cl.size(), 1u);
  EXPECT_EQ(cl[0].size(), 10u);
}

TEST(Cluster, DropsSmallAndLargeClusters) {
  PointCloud in = blob(0, 0, 3);
  PointCloud big = blob(10, 0, 50);
  PointCloud mid = blob(20, 0, 20);
  in.insert(in.end(), big.begin(), big.end());
  in.insert(in.end(), mid.begin(), mid.end());
  auto cl = euclidean_cluster(in, {0.5f, 5, 40});
  ASSERT_EQ(cl.size(), 1u);
  EXPECT_EQ(cl[0].size(), 20u);
}
