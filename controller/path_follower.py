import numpy as np

class PathFollower:
    def __init__(self, path, step_size=0.1, tolerance=0.2):
        """
        path: np.ndarray of shape (N, 3) — the 3D waypoints
        step_size: how much to move per step
        tolerance: how close to a waypoint before switching to the next
        """
        self.path = path
        self.index = 0
        self.step_size = step_size
        self.tolerance = tolerance

    def reset(self):
        self.index = 0

    def get_action(self, current_pos):
        if self.index >= len(self.path):
            return np.zeros(3)  # already at goal

        target = self.path[self.index]
        direction = target - current_pos
        distance = np.linalg.norm(direction)

        if distance < self.tolerance:
            self.index += 1
            if self.index >= len(self.path):
                return np.zeros(3)  # goal reached
            target = self.path[self.index]
            direction = target - current_pos

        unit_direction = direction / (np.linalg.norm(direction) + 1e-8)
        return self.step_size * unit_direction
