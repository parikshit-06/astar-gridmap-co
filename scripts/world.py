import open3d as o3d
import numpy as np
import os

class Open3DWorld:
    def __init__(self, mesh_paths):
        self.meshes = []
        for path in mesh_paths:
            print(f"Trying to load mesh from: {path}")
            mesh = o3d.io.read_triangle_mesh(path)
            print(f"Loaded mesh with {len(np.asarray(mesh.vertices))} vertices")
            if len(np.asarray(mesh.vertices)) == 0:
                raise ValueError(f"Failed to load mesh properly: {path}")
            mesh.compute_vertex_normals()
            self.meshes.append(mesh)


    def render(self, drone_position=None, goal_position=None):
        geometries = self.meshes.copy()

        if drone_position is not None:
            drone_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
            drone_sphere.translate(drone_position)
            drone_sphere.paint_uniform_color([1, 0, 0])
            geometries.append(drone_sphere)

        if goal_position is not None:
            goal_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
            goal_sphere.translate(goal_position)
            goal_sphere.paint_uniform_color([0, 1, 0])
            geometries.append(goal_sphere)

        vis = o3d.visualization.Visualizer()
        o3d.visualization.draw_geometries(
            geometries,
            window_name="Drone World",
            width=800,
            height=600
        )
        vis.create_window()
        for g in geometries:
            vis.add_geometry(g)
        vis.run()
        vis.destroy_window()

    def get_obstacle_positions(self):
        # Optional: return obstacle bounding boxes
        return [mesh.get_axis_aligned_bounding_box() for mesh in self.meshes]
