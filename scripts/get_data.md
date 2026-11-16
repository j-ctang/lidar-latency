# Getting nuScenes mini sweeps

nuScenes requires a free account and accepting its terms of use, so the data is
downloaded by hand and never committed. `data/` is gitignored.

1. Create an account and sign in at <https://www.nuscenes.org/nuscenes#download>.
2. Under **Full dataset (v1.0)**, download **Mini** (`v1.0-mini.tgz`, about 4 GB).
3. Extract it outside the repo or into `data/`:

   ```sh
   mkdir -p data
   tar -xzf ~/Downloads/v1.0-mini.tgz -C data
   ```

4. The lidar sweeps are then in `data/sweeps/LIDAR_TOP/*.pcd.bin`
   (keyframes are in `data/samples/LIDAR_TOP/`). Each file is float32
   `x, y, z, intensity, ring` per point, about 34k points.
5. Replay them:

   ```sh
   ./build/lidar_replay --data data/sweeps/LIDAR_TOP --frames 400 \
       --out results/baseline_nuscenes.csv
   ```

   File names contain the timestamp, so sorting by name gives replay order.
   Add `--realtime` to pace frames at 20 Hz.
