import os
import numpy as np

# Config
DATA_FOLDER = "./2011_09_26_drive_0011_extract/velodyne_points/data"
X_MIN, X_MAX = -50, 50
Y_MIN, Y_MAX = -50, 50
Z_MIN, Z_MAX = -5, 5
VOXEL_SIZE = 0.5

# Grid size
nx = int((X_MAX - X_MIN) / VOXEL_SIZE)
ny = int((Y_MAX - Y_MIN) / VOXEL_SIZE)
nz = int((Z_MAX - Z_MIN) / VOXEL_SIZE)
occupancy_grid = np.zeros((nx, ny, nz), dtype=np.uint8)

# Process files
for fname in sorted(os.listdir(DATA_FOLDER)):
    if fname.endswith(".txt"):
        fpath = os.path.join(DATA_FOLDER, fname)
        try:
            pts = np.loadtxt(fpath)
            xyz = pts[:, :3]
        except Exception as e:
            print(f"[ERROR] Failed to load {fname}: {e}")
            continue

        idx = np.floor((xyz - [X_MIN, Y_MIN, Z_MIN]) / VOXEL_SIZE).astype(int)
        valid = (
            (idx[:, 0] >= 0) & (idx[:, 0] < nx) &
            (idx[:, 1] >= 0) & (idx[:, 1] < ny) &
            (idx[:, 2] >= 0) & (idx[:, 2] < nz)
        )
        idx = idx[valid]
        for x, y, z in idx:
            occupancy_grid[x, y, z] = 1

# Save
np.save("lidar/occupancy_grid.npy", occupancy_grid)
print(f"[DONE] Occupancy grid saved as lidar/occupancy_grid.npy with shape {occupancy_grid.shape}")
