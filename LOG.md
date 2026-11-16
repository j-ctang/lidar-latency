# Build log

Dated record of steps, measurements, and issues. Raw benchmark output lives in `results/`.

## Machine

- Apple M5 Pro (`machdep.cpu.brand_string`), model Mac17,9 (`hw.model`), 15 cores, macOS 26.7
- Apple clang 21.0.0 (clang-2100.4.1.1), CMake 4.4.3
- Release build, `-O2 -DNDEBUG`
- CI: ubuntu-latest, GCC 15.2.0

## Metrics

| Date | Change | p50 ms | p99 ms | Frames > 50 ms | Notes |
|---|---|---|---|---|---|
| 2026-11-16 | Naive baseline, synthetic | 29.52 | 41.90 | 1 / 200 | 30k pts/frame; mean voxel 2.75, ground 0.65, cluster 26.28 ms; `results/baseline_synthetic.csv` |

## Steps

### 2026-11-16 — Project skeleton

- CMake project: `lidar` library, `lidar_replay` CLI, `lidar_tests` (GoogleTest 1.15.2), `lidar_bench` (Google Benchmark 1.9.1). Both fetched with FetchContent.
- Release flags pinned to `-O2` (CMake's default Release is `-O3`).

### 2026-11-16 — Loader and synthetic scenes

- `load_nuscenes_bin`: reads float32 `x,y,z,intensity,ring` (20 bytes per point), drops ring, throws on a size that is not a multiple of 20 bytes.
- `generate_scene(seed)`: 30,000 points; 60% on a ground plane at z = -1.8 m over ±40 m, 40% inside 12 random boxes (0.5–4.5 m wide, 1–2.5 m tall); 2 cm gaussian noise. Deterministic per seed.
- `scripts/get_data.md`: manual nuScenes v1.0-mini download steps (account required). `data/` is gitignored.

### 2026-11-16 — Naive baseline pipeline

- Voxel downsample: `std::map<tuple<int,int,int>>`, centroid per 0.2 m voxel.
- Ground removal: RANSAC plane, 100 iterations, 0.15 m threshold, rejects planes with |n_z| < 0.9.
- Euclidean clustering: BFS, brute-force O(n) neighbour scan per point, so O(n²); 0.5 m tolerance, min 10 points.
- `run_pipeline` times each stage with `std::chrono::steady_clock`.
- Typical frame: 30,000 in, ~25,400 after voxel, ~7,900 non-ground, ~11.5 clusters (12 boxes; some overlap).

### 2026-11-16 — Tests and CI

- Tests: loader (valid, truncated, missing file), voxel (centroid, negative coords, empty), ground (plane removal, tiny input, vertical wall), clustering (two blobs, transitive chain, size limits), synthetic determinism, end-to-end small scene.
- GitHub Actions on ubuntu-latest: configure, build, ctest, 5-frame replay smoke test. First run green.

### 2026-11-16 — Baseline measurement

- `./build/lidar_replay --synthetic 200 --out results/baseline_synthetic.csv`
- Output: `frames=200 p50=29.52 ms p99=41.90 ms max=51.20 ms frames>50ms=1`
- Google Benchmark (one frame, steady state): voxel 2.47 ms, ground 0.643 ms, cluster 26.0 ms, pipeline 29.3 ms.
- Clustering is ~89% of frame time. It is the stage to optimize next.
- Point loading and scene generation are outside the timed region. Frames run back-to-back (no `--realtime` pacing) for the baseline.

## Issues

- **The only frame over 50 ms is frame 0 (51.20 ms: voxel 6.97, cluster 42.77).** Cause: cold start; first-touch page faults and cold caches on the first frame. Not fixed: it is real first-frame latency, so it stays in the numbers. Later runs should report it separately or add warm-up frames, and say which.
- **`timeout` not found on macOS when wrapping `gh run watch`.** Cause: `timeout` is GNU coreutils and not in base macOS. Fix: ran `gh run watch` without the wrapper.
- **Synthetic numbers are not nuScenes numbers.** The synthetic scene keeps ~85% of points after a 0.2 m voxel, while real sweeps are sparser far out and denser near the car. Baseline on real sweeps still to be measured once v1.0-mini is downloaded by hand.
