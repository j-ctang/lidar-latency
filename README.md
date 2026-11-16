# lidar-latency

A C++17 lidar perception pipeline that replays nuScenes lidar sweeps at 20 Hz, measures the latency of each stage, and then optimizes the slowest stage.

Pipeline: voxel downsample, ground removal, Euclidean clustering.

Goal: cut p99 frame latency at least 2x and count frames over a 50 ms budget, before and after optimization.

Status: in progress. See [LOG.md](LOG.md) for each step, measurements, and issues.
