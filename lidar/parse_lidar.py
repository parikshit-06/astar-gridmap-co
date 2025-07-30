import numpy as np
import open3d as o3d
import os

def load_lidar_txt(file_path):
    """Load a single LiDAR frame from .txt"""
    try:
        points = np.loadtxt(file_path)
        return points[:, :3]  # Drop reflectance if needed
    except Exception as e:
        print(f"[ERROR] Failed to load {file_path}: {e}")
        return None

def load_all_lidar_txt(folder_path):
    """Load all LiDAR frames from .txt files in folder"""
    lidar_frames = []
    file_list = sorted(os.listdir(folder_path))
    
    for fname in file_list:
        if not fname.endswith(".txt"):
            continue
        full_path = os.path.join(folder_path, fname)
        pts = load_lidar_txt(full_path)
        if pts is not None:
            lidar_frames.append(pts)

    print(f"[INFO] Loaded {len(lidar_frames)} frames from {folder_path}")
    return lidar_frames

def visualize_point_cloud(xyz):
    """Visualize 3D point cloud using Open3D"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    # Update path to your dataset directory
    folder_path = "./2011_09_26_drive_0011_extract/velodyne_points/data"
    
    frames = load_all_lidar_txt(folder_path)

    # Visualize
    if frames:
        visualize_point_cloud(frames[10])
