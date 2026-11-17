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
| 2026-11-16 | Naive baseline, nuScenes mini | 53.25 | 79.41 | 240 / 400 | 34.7k pts/frame, 11.7k after voxel, 8.2k non-ground, 109 clusters; mean voxel 1.94, ground 0.40, cluster 49.62 ms; `results/baseline_nuscenes.csv` |
| 2026-11-17 | Naive rerun, nuScenes mini (same session as grid) | 54.66 | 81.49 | 260 / 400 | low power mode on; excl. frame 0: 54.70 / 81.51, 260 / 399; mean cluster 50.84 ms; `results/opt1_naive_rerun_nuscenes.csv` |
| 2026-11-17 | Grid hash clustering, nuScenes mini | 6.94 | 8.34 | 0 / 400 | excl. frame 0: 6.94 / 8.40, 0 / 399; mean voxel 2.05, ground 0.42, cluster 4.32 ms; 7.9x p50, 9.8x p99, 11.8x cluster vs same-session naive; `results/opt1_grid_nuscenes.csv` |
| 2026-11-17 | Naive rerun, synthetic (same session) | 48.08 | 65.82 | 76 / 200 | low power mode on, see Issues; `results/opt1_naive_rerun_synthetic.csv` |
| 2026-11-17 | Grid hash clustering, synthetic | 10.07 | 11.33 | 0 / 200 | excl. frame 0: 10.07 / 11.35; mean cluster 4.33 ms; 4.8x p50, 5.8x p99 vs same-session naive; `results/opt1_grid_synthetic.csv` |
| 2026-11-17 | FINAL naive, nuScenes mini, Low Power Mode off | 39.79 | 71.08 | 93 / 400 | excl. frame 0: 39.96 / 71.40; `results/final_naive_nuscenes.csv` |
| 2026-11-17 | FINAL grid, nuScenes mini, Low Power Mode off | 5.03 | 6.82 | 0 / 400 | excl. frame 0: 5.03 / 6.90; 7.9x p50, 10.4x p99 vs same-session naive; `results/final_grid_nuscenes.csv` |
| 2026-11-17 | FINAL naive, synthetic, Low Power Mode off | 32.46 | 46.53 | 0 / 200 | `results/final_naive_synthetic.csv` |
| 2026-11-17 | FINAL grid, synthetic, Low Power Mode off | 6.84 | 8.32 | 0 / 200 | 4.7x p50, 5.6x p99; `results/final_grid_synthetic.csv` |

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

### 2026-11-16: Baseline on real nuScenes sweeps

- Downloaded `v1.0-mini.tgz` (4.17 GB) from the official nuScenes link used by the devkit tutorial (`https://www.nuscenes.org/data/v1.0-mini.tgz`), after accepting the nuScenes terms of use. Extracted into `data/` (gitignored). 3,531 LIDAR_TOP sweeps.
- Replayed the first 400 sweeps back to back: p50 53.25 ms, p99 79.41 ms, max 98.46 ms, 240 of 400 frames over the 50 ms budget.
- Clustering is 95% of mean frame time (49.62 of 51.97 ms). Real sweeps keep more non-ground points and produce about 9x more clusters than the synthetic scene.

### 2026-11-17: Profile before optimizing

- Instruments Time Profiler (`xctrace record --template 'Time Profiler'`) over the 400-sweep replay, summarized with `scripts/xctrace_summary.py`.
- Baseline: `euclidean_cluster` 94.4% inclusive (94.0% self), voxel 3.8%, ground 0.8%, file loading 1.0%. `results/profile_baseline.txt`.
- Confirms the timer breakdown: the O(n²) neighbour scan is the hot spot.

### 2026-11-17: Grid hash neighbour search

- Uniform grid, cell = tolerance (x1.0001 so float rounding cannot put two in-range points two cells apart). Points sorted by packed 63-bit cell key; `unordered_map` maps key to its index range. Each BFS pop scans the 27 neighbour cells.
- Same semantics: neighbours found per pop are sorted by index before pushing, so BFS order matches the naive scan and output is identical, not just the same partition.
- Naive kept as `ClusterParams::search = NeighborSearch::Naive`, CLI `--naive-cluster`.
- Tests: grid == naive (cluster count, canonical partition, exact output) on 3 synthetic scenes, a cell-boundary and negative-coordinate cloud, and nuScenes sweeps 0, 100, 299 (skipped when `data/` is absent, as in CI). Breaking the search (scanning only dz = 0) fails all three.
- Same 400 sweeps, same `-O2` build: p50 54.66 -> 6.94 ms (7.9x), p99 81.49 -> 8.34 ms (9.8x), frames > 50 ms 260 -> 0. Mean clustering 50.84 -> 4.32 ms (11.8x). Per-frame cluster and point counts equal the committed baseline CSV on all 400 frames.
- Replay now prints stats for all frames and excluding frame 0. With the grid, frame 0 is not an outlier (8.40 vs 8.34 ms p99), so both are within 0.1 ms.
- Re-profiled: clustering 60% inclusive, voxel 28.5%, ground 6%. `results/profile_opt1_grid.txt`. Clustering is still the largest stage, so I did not start the voxel change (it was conditional on clustering no longer dominating). Next candidates: voxel `std::map` (sort-based keeps the current output order, which RANSAC depends on) and the per-pop `unordered_map` lookups in clustering.

### 2026-11-17: Final numbers with Low Power Mode off

- `pmset -g` shows `powermode 0`. Same build, same 400 sweeps, naive and grid run back to back.
- nuScenes: p50 39.79 -> 5.03 ms (7.9x), p99 71.08 -> 6.82 ms (10.4x), frames > 50 ms 93 -> 0.
- These are the headline numbers. Earlier rows were measured in Low Power Mode.

## Issues

- **The only frame over 50 ms is frame 0 (51.20 ms: voxel 6.97, cluster 42.77).** Cause: cold start; first-touch page faults and cold caches on the first frame. Not fixed: it is real first-frame latency, so it stays in the numbers. Later runs should report it separately or add warm-up frames, and say which.
- **`timeout` not found on macOS when wrapping `gh run watch`.** Cause: `timeout` is GNU coreutils and not in base macOS. Fix: ran `gh run watch` without the wrapper.
- **Synthetic numbers are not nuScenes numbers.** The synthetic scene keeps ~85% of points after a 0.2 m voxel, while real sweeps are sparser far out and denser near the car. Baseline on real sweeps still to be measured once v1.0-mini is downloaded by hand.
- Synthetic scene underestimated cost: real sweeps run 1.8x slower at p50 (53.25 vs 29.52 ms) and miss the 50 ms budget on 60% of frames. Real data is now the reference for all later comparisons.
- **Synthetic numbers on 2026-11-17 are ~1.6x slower than the 2026-11-16 synthetic baseline with unchanged code.** Cause: macOS Low Power Mode is on (`pmset -g`: `powermode 1`). The baseline commit rebuilt and rerun now gives p50 49.82 ms vs 29.52 ms recorded. Fix: every before/after comparison above uses a naive rerun in the same session, same build flags, not the older rows. The 2026-11-16 nuScenes baseline (53.25 ms) is within 3% of today's naive rerun (54.66 ms), so it was likely also in Low Power Mode. Not fixed: power mode is a machine setting; rerun all rows with it off for absolute numbers.
- **First xctrace export parse found 0 samples.** Cause: the time-profile table uses `<tagged-backtrace>`, not `<backtrace>`. Fix: parse `tagged-backtrace` in `scripts/xctrace_summary.py`.

