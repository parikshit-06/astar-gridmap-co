import numpy as np
from world import Open3DWorld
from a_star import AStar3D
from grid_utils import world_to_grid, grid_to_world
from controller.path_follower import PathFollower
from controller.avoidance_apf import APFAvoidance

class DroneEnv:
    def __init__(self, mesh_paths):
        self.world = Open3DWorld(mesh_paths)
        self.bounds = np.array([[-2, 2], [-2, 2], [-2, 2]])
        self.goal = np.array([1.5, 1.5, 1.5])

        # Load voxel grid
        self.occupancy_grid = np.load("lidar/occupancy_grid.npy")
        self.astar = AStar3D(self.occupancy_grid)

        # Voxel parameters (match with grid construction script)
        self.x_min, self.y_min, self.z_min = -50, -50, -5
        self.voxel_size = 0.5

        self.follower = PathFollower(kp=0.1)
        self.avoider = APFAvoidance(self.occupancy_grid, self.voxel_size)

    def render(self):
        self.world.render(drone_position=self.drone_pos, goal_position=self.goal)

    def step(self, _action=None):
        if self.path_index >= len(self.path):
            return self.drone_pos, 0, True

        target = self.path[self.path_index]
        attractive_force, reached = self.follower.get_action(self.drone_pos, target)
        repulsive_force = self.avoider.get_repulsive_force(self.drone_pos, self.x_min, self.y_min, self.z_min)

        # Combine forces
        total_force = attractive_force + repulsive_force
        force_magnitude = np.linalg.norm(total_force)

        if force_magnitude > 1e-6:
            action = 0.1 * total_force / force_magnitude
        else:
            action = np.zeros(3)

        self.drone_pos += action
        self.drone_pos = np.clip(self.drone_pos, self.bounds[:, 0], self.bounds[:, 1])

        if reached:
            self.path_index += 1

        reward = -np.linalg.norm(self.goal - self.drone_pos)
        done = reward > -0.2 or self.steps > 500

        # Trigger replan if repulsive force is strong
        if np.linalg.norm(repulsive_force) > 1.0:
            self.replan_from_current()

        self.steps += 1
        return self.drone_pos, reward, done

    def reset(self):
        self.drone_pos = np.random.uniform(-1, 1, size=3)
        self.steps = 0

        # Plan path
        start_idx = world_to_grid(self.drone_pos, self.x_min, self.y_min, self.z_min, self.voxel_size)
        goal_idx = world_to_grid(self.goal, self.x_min, self.y_min, self.z_min, self.voxel_size)
        voxel_path = self.astar.plan(start_idx, goal_idx)

        if voxel_path is None:
            raise Exception("No path found. Try resetting.")

        # Convert path to world coordinates
        self.path = [grid_to_world(p, self.x_min, self.y_min, self.z_min, self.voxel_size) for p in voxel_path]
        self.path_index = 0

        return self.drone_pos
    
    def replan_from_current(self):
        """Replans the path from current position to goal."""
        start_idx = world_to_grid(self.drone_pos, self.x_min, self.y_min, self.z_min, self.voxel_size)
        goal_idx = world_to_grid(self.goal, self.x_min, self.y_min, self.z_min, self.voxel_size)
        voxel_path = self.astar.plan(start_idx, goal_idx)

        if voxel_path is None:
            print("[WARN] Replan failed — No path found.")
            return False

        self.path = [grid_to_world(p, self.x_min, self.y_min, self.z_min, self.voxel_size) for p in voxel_path]
        self.path_index = 0
        print("[INFO] Path replanned from current position.")
        return True


    def render(self):
        self.world.render(drone_position=self.drone_pos, goal_position=self.goal)

    def step(self, _action=None):
        if self.path_index >= len(self.path):
            return self.drone_pos, 0, True

        target = self.path[self.path_index]
        attractive_force, reached = self.follower.get_action(self.drone_pos, target)
        repulsive_force = self.avoider.get_repulsive_force(self.drone_pos, self.x_min, self.y_min, self.z_min)

        # Combine forces
        total_force = attractive_force + repulsive_force
        force_magnitude = np.linalg.norm(total_force)

        if force_magnitude > 1e-6:
            action = 0.1 * total_force / force_magnitude
        else:
            action = np.zeros(3)

        self.drone_pos += action
        self.drone_pos = np.clip(self.drone_pos, self.bounds[:, 0], self.bounds[:, 1])

        if reached:
            self.path_index += 1

        reward = -np.linalg.norm(self.goal - self.drone_pos)
        done = reward > -0.2 or self.steps > 500

        # Trigger replan if repulsive force is strong
        if np.linalg.norm(repulsive_force) > 1.0:
            self.replan_from_current()

        self.steps += 1
        return self.drone_pos, reward, done
