import numpy as np
import heapq

class AStar3D:
    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.shape = occupancy_grid.shape

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def get_neighbors(self, node):
        directions = [(dx, dy, dz)
                      for dx in [-1, 0, 1]
                      for dy in [-1, 0, 1]
                      for dz in [-1, 0, 1]
                      if not (dx == dy == dz == 0)]
        neighbors = []
        for dx, dy, dz in directions:
            nx, ny, nz = node[0] + dx, node[1] + dy, node[2] + dz
            if 0 <= nx < self.shape[0] and 0 <= ny < self.shape[1] and 0 <= nz < self.shape[2]:
                if self.grid[nx, ny, nz] == 0:  # not occupied
                    neighbors.append((nx, ny, nz))
        return neighbors

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def plan(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            visited.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in visited:
                    continue
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found
