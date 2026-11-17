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

// ---- Grid search must match the naive search exactly. ----

#include <filesystem>
#include <random>

#include "lidar/io.hpp"
#include "lidar/synthetic.hpp"

namespace {

std::vector<Cluster> run(const PointCloud& in, NeighborSearch s) {
  ClusterParams p;
  p.search = s;
  return euclidean_cluster(in, p);
}

// Same cluster count, same point-set partition, and same exact output.
void expect_same(const PointCloud& in) {
  auto naive = run(in, NeighborSearch::Naive);
  auto grid = run(in, NeighborSearch::Grid);
  ASSERT_EQ(naive.size(), grid.size());
  auto canon = [](std::vector<Cluster> cl) {
    for (auto& c : cl) std::sort(c.begin(), c.end());
    std::sort(cl.begin(), cl.end());
    return cl;
  };
  EXPECT_EQ(canon(naive), canon(grid)) << "partition differs";
  EXPECT_EQ(naive, grid) << "order differs";
}

}  // namespace

TEST(ClusterGrid, MatchesNaiveOnSyntheticScenes) {
  for (std::uint32_t seed : {0u, 1u, 7u}) {
    PointCloud obj = remove_ground(voxel_downsample(generate_scene(seed), 0.2f));
    SCOPED_TRACE(seed);
    expect_same(obj);
  }
}

TEST(ClusterGrid, MatchesNaiveOnCellBoundaries) {
  // Points on exact multiples of the tolerance, negative coords, and pairs at
  // exactly the tolerance distance.
  PointCloud in;
  for (int i = -20; i <= 20; ++i)
    for (int j = -2; j <= 2; ++j) in.push_back({0.5f * i, 0.5f * j, 0.5f * (i % 3), 0});
  std::mt19937 rng(3);
  std::uniform_real_distribution<float> u(-5.f, 5.f);
  for (int i = 0; i < 2000; ++i) in.push_back({u(rng), u(rng), u(rng) * 0.2f, 0});
  expect_same(in);
}

TEST(ClusterGrid, MatchesNaiveOnRealFrames) {
  // Real nuScenes sweeps are not in the repo (licence); skip when absent.
  const std::filesystem::path dir = LIDAR_SOURCE_DIR "/data/sweeps/LIDAR_TOP";
  if (!std::filesystem::is_directory(dir)) GTEST_SKIP() << "no nuScenes data";
  std::vector<std::filesystem::path> files;
  for (const auto& e : std::filesystem::directory_iterator(dir))
    if (e.path().string().size() > 8 &&
        e.path().string().compare(e.path().string().size() - 8, 8, ".pcd.bin") == 0)
      files.push_back(e.path());
  std::sort(files.begin(), files.end());
  if (files.size() < 300) GTEST_SKIP() << "too few sweeps";
  for (std::size_t f : {0u, 100u, 299u}) {
    SCOPED_TRACE(files[f].string());
    expect_same(remove_ground(voxel_downsample(load_nuscenes_bin(files[f].string()), 0.2f)));
  }
}
