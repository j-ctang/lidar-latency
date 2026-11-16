// Per-stage micro-benchmarks on one synthetic ~30k point frame.
#include <benchmark/benchmark.h>

#include "lidar/pipeline.hpp"
#include "lidar/synthetic.hpp"

using namespace lidar;

static const PointCloud& frame() { static PointCloud c = generate_scene(0); return c; }
static const PointCloud& voxeled() { static PointCloud c = voxel_downsample(frame(), 0.2f); return c; }
static const PointCloud& nonground() { static PointCloud c = remove_ground(voxeled()); return c; }

static void BM_Voxel(benchmark::State& s) {
  for (auto _ : s) benchmark::DoNotOptimize(voxel_downsample(frame(), 0.2f));
}
static void BM_Ground(benchmark::State& s) {
  for (auto _ : s) benchmark::DoNotOptimize(remove_ground(voxeled()));
}
static void BM_Cluster(benchmark::State& s) {
  for (auto _ : s) benchmark::DoNotOptimize(euclidean_cluster(nonground()));
}
static void BM_Pipeline(benchmark::State& s) {
  for (auto _ : s) benchmark::DoNotOptimize(run_pipeline(frame()));
}
BENCHMARK(BM_Voxel)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Ground)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Cluster)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Pipeline)->Unit(benchmark::kMillisecond);
BENCHMARK_MAIN();
