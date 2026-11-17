// Replays lidar frames through the pipeline at 20 Hz (or as fast as possible)
// and writes per-frame, per-stage latency to CSV.
//
//   lidar_replay --synthetic N [--out results/x.csv] [--realtime]
//   lidar_replay --data DIR    [--out results/x.csv] [--realtime] [--frames N]
//   [--naive-cluster]  use the O(n^2) baseline neighbour search
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "lidar/io.hpp"
#include "lidar/pipeline.hpp"
#include "lidar/synthetic.hpp"

namespace fs = std::filesystem;
using namespace lidar;

namespace {

double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  // Nearest-rank percentile.
  std::size_t k = static_cast<std::size_t>(q / 100.0 * v.size() + 0.999999);
  k = std::clamp<std::size_t>(k, 1, v.size());
  return v[k - 1];
}

int usage() {
  std::cerr << "usage: lidar_replay (--synthetic N | --data DIR) [--frames N]"
               " [--out FILE] [--realtime] [--naive-cluster]\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  int synthetic = 0, max_frames = -1;
  std::string data_dir, out = "results/replay.csv";
  bool realtime = false;
  PipelineParams params;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) { usage(); std::exit(2); }
      return argv[++i];
    };
    if (a == "--synthetic") synthetic = std::stoi(next());
    else if (a == "--data") data_dir = next();
    else if (a == "--frames") max_frames = std::stoi(next());
    else if (a == "--out") out = next();
    else if (a == "--realtime") realtime = true;
    else if (a == "--naive-cluster") params.cluster.search = NeighborSearch::Naive;
    else return usage();
  }
  if ((synthetic > 0) == !data_dir.empty()) return usage();

  std::vector<fs::path> files;
  if (!data_dir.empty()) {
    for (const auto& e : fs::recursive_directory_iterator(data_dir))
      if (e.is_regular_file() && e.path().string().size() > 8 &&
          e.path().string().substr(e.path().string().size() - 8) == ".pcd.bin")
        files.push_back(e.path());
    std::sort(files.begin(), files.end());  // nuScenes names sort by timestamp
    if (files.empty()) { std::cerr << "no .pcd.bin in " << data_dir << "\n"; return 1; }
  }
  const int n = synthetic > 0 ? synthetic : static_cast<int>(files.size());
  const int frames = max_frames > 0 ? std::min(n, max_frames) : n;

  std::ofstream csv(out);
  if (!csv) { std::cerr << "cannot write " << out << "\n"; return 1; }
  csv << "frame,in_points,voxel_points,nonground_points,clusters,"
         "voxel_ms,ground_ms,cluster_ms,total_ms\n";

  std::vector<double> totals;
  const auto period = std::chrono::milliseconds(50);  // 20 Hz
  auto next_tick = std::chrono::steady_clock::now();
  for (int f = 0; f < frames; ++f) {
    // Load outside the timed region: we measure perception, not disk I/O.
    PointCloud cloud = synthetic > 0 ? generate_scene(static_cast<std::uint32_t>(f))
                                     : load_nuscenes_bin(files[f].string());
    if (realtime) std::this_thread::sleep_until(next_tick);
    next_tick += period;
    FrameResult r = run_pipeline(cloud, params);
    totals.push_back(r.t.total_ms);
    char line[256];
    std::snprintf(line, sizeof line, "%d,%zu,%zu,%zu,%zu,%.3f,%.3f,%.3f,%.3f\n", f,
                  r.in_points, r.voxel_points, r.nonground_points, r.clusters,
                  r.t.voxel_ms, r.t.ground_ms, r.t.cluster_ms, r.t.total_ms);
    csv << line;
  }

  // Frame 0 is cold (first-touch page faults, cold caches). Report both.
  auto report = [](const char* label, const std::vector<double>& t) {
    const auto over = std::count_if(t.begin(), t.end(), [](double x) { return x > 50.0; });
    std::printf("%s frames=%zu p50=%.2f ms p99=%.2f ms max=%.2f ms frames>50ms=%ld\n",
                label, t.size(), percentile(t, 50), percentile(t, 99),
                t.empty() ? 0.0 : *std::max_element(t.begin(), t.end()),
                static_cast<long>(over));
  };
  report("all     ", totals);
  if (totals.size() > 1)
    report("no-frame0", std::vector<double>(totals.begin() + 1, totals.end()));
  std::printf("wrote %s\n", out.c_str());
  return 0;
}
