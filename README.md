# 3d-path-planning

A modular Python framework for 3D obstacle-aware drone navigation using voxelized LiDAR data, A* path planning, and artificial-potential-field (APF) avoidance — with real-time visualization via Open3D.

---

## 🚀 Features

- **LiDAR Ingestion & Voxelization**  
  Parse raw LiDAR `.txt` frames (KITTI format) into a 3D occupancy grid (`.npy`).

- **3D A* Path Planner**  
  Grid-based A* search in voxel space for collision-free waypoint sequences.

- **Path Follower**  
  Simple proportional controller to track waypoints in continuous 3D.

- **APF Obstacle Avoidance**  
  Repulsive potential fields layered on top of the planner to dodge unforeseen obstacles.

- **Replanning Fallback**  
  Automatic A* replanning when repulsive forces exceed a threshold.

- **Real-time Visualization**  
  Open3D renders the environment, current path, drone and goal positions.

---

## 📂 Repository Structure
```
aster-gridmap-co/
├── lidar/
│   ├── parse_lidar.py          # loads .txt frames and creates occupancy grid
|   ├── voxelizer_visualize.py  # voxelized env
│   └── occupancy_grid.npy      # precomputed voxel map
├── controller/
│   ├── path_follower.py        # proportional path-tracking controller
│   └── avoidance_apf.py        # APF-based repulsion logic
├── meshes/
│   ├── cube.obj  
│   ├── ditto.vtk    
│   ├── duck.dae
│   ├── plane100.obj
|   ├── quadrotor_base.obj
|   └── sphere_smooth.obj
├── sim/
│   ├── world.py                # Open3DWorld renderer
│   ├── drone_env.py            # DroneEnv: integrates planner, controllers, sim loop
│   ├── main.py                 # entry point: runs multiple episodes & logs metrics
│   ├── a_star.py               # 3D A* implementation
│   └── grid_utils.py           # world↔grid coordinate transforms
├── media/
│   └── demo.mp4                # optional recorded demo
├── notebooks/
│   └── run_planner.ipynb       # Jupyter experiments (A* visualization)
└── README.md                   # this file
```
---

## ⚙️ Prerequisites

- **Python 3.10** (3.11 has compatibility issues with Open3D/Scipy)  
- [Miniconda](https://docs.conda.io/) or [Anaconda](https://www.anaconda.com/)

---

## 🛠 Installation

```bash
# create & activate environment
conda create -n drone3d python=3.10
conda activate drone3d

# install core dependencies
conda install -c conda-forge numpy scipy open3d matplotlib

# (optional) additional tools
pip install tqdm
````

---

## 🎮 Running the Simulation

1. **Ensure** `lidar/occupancy_grid.npy` has been generated.
   If not, run:

   ```bash
   python lidar/parse_lidar.py
   ```

2. **Launch** the main simulation:

   ```bash
   python sim/main.py
   ```

---

## 📊 Logging & Metrics

By default, `main.py` runs multiple episodes and writes `logs/simulation_metrics.csv` with:

| run | steps | path\_length | final\_distance |
| --- | ----- | ------------ | --------------- |

Use this data to evaluate performance.

---

## 🔮 Future Work

* **RRT**\* and **hybrid planners** for faster global planning
* **Reinforcement-learning** policy for local navigation
* **MATLAB/Simulink** low-level control integration
* **ROS2/Gazebo** end-to-end deployment
* **Dynamic obstacles** and **real-time SLAM**

---

## License
This project is released under the MIT License. See LICENSE for details.

---

Note: This project is for research and educational purposes. Always follow local regulations and safety guidelines when operating drones
