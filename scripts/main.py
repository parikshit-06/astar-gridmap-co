import os
from drone_env import DroneEnv

if __name__ == "__main__":
    mesh_paths = ["meshes/quadrotor_base.obj"]
    env = DroneEnv(mesh_paths)
    obs = env.reset()
    done = False

    while not done:
        obs, reward, done = env.step()
        env.render()
