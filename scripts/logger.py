import csv
import os

class MetricsLogger:
    def __init__(self, filename="logs/simulation_metrics.csv"):
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        self.filename = filename
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["run", "steps", "path_length", "final_distance"])

    def log(self, run_id, steps, path_length, final_distance):
        with open(self.filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([run_id, steps, path_length, final_distance])
