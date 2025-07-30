import numpy as np

def world_to_grid(pos, x_min, y_min, z_min, voxel_size):
    return tuple(np.floor((pos - np.array([x_min, y_min, z_min])) / voxel_size).astype(int))

def grid_to_world(index, x_min, y_min, z_min, voxel_size):
    return np.array(index) * voxel_size + np.array([x_min, y_min, z_min]) + voxel_size / 2
