import numpy as np
from scipy.ndimage import distance_transform_edt

class APFAvoidance:
    def __init__(self, occupancy_grid, voxel_size, repulsive_gain=5.0, influence_radius=3.0):
        self.grid = occupancy_grid
        self.voxel_size = voxel_size
        self.repulsive_gain = repulsive_gain
        self.influence_radius = influence_radius

        # Precompute distance transform (distance to nearest obstacle for each voxel)
        self.distance_map = distance_transform_edt(1 - occupancy_grid) * voxel_size

    def get_repulsive_force(self, pos, x_min, y_min, z_min):
        # Convert world pos to grid index
        grid_pos = np.floor((pos - np.array([x_min, y_min, z_min])) / self.voxel_size).astype(int)

        # Check bounds
        if not ((0 <= grid_pos[0] < self.grid.shape[0]) and
                (0 <= grid_pos[1] < self.grid.shape[1]) and
                (0 <= grid_pos[2] < self.grid.shape[2])):
            return np.zeros(3)

        dist = self.distance_map[tuple(grid_pos)]
        if dist > self.influence_radius or dist <= 1e-6:
            return np.zeros(3)

        # Compute gradient (repulsive force direction)
        gx, gy, gz = np.gradient(-self.distance_map)
        grad = np.array([gx[tuple(grid_pos)], gy[tuple(grid_pos)], gz[tuple(grid_pos)]])

        # Normalize + scale repulsion
        force = self.repulsive_gain * grad / (dist**2 + 1e-6)
        return force
