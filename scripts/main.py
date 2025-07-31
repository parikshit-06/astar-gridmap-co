from drone_env import DroneEnv
from logger import MetricsLogger
import numpy as np

if __name__ == "__main__":
    mesh_paths = ["meshes/quadrotor_base.obj"]
    env = DroneEnv(mesh_paths)
    logger = MetricsLogger()

    num_runs = 10

    for run in range(num_runs):
        obs = env.reset()
        done = False
        total_steps = 0
        trajectory = [obs.copy()]

        while not done:
            obs, reward, done = env.step()
            trajectory.append(obs.copy())
            total_steps += 1

        # Metrics
        path_length = np.sum(np.linalg.norm(np.diff(trajectory, axis=0), axis=1))
        final_distance = np.linalg.norm(env.goal - obs)
        logger.log(run_id=run, steps=total_steps, path_length=path_length, final_distance=final_distance)
        print(f"Run {run} | Steps: {total_steps} | Path: {path_length:.2f} | Final dist: {final_distance:.2f}")
