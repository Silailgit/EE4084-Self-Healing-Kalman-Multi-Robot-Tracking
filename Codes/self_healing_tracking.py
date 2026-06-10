import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


DT = 0.064
DROP_PROBABILITY = 0.15  # ölçümlerin %15'i kaybolacak
np.random.seed(42)


class KalmanFilter2D:
    def __init__(self, dt=DT):
        self.dt = dt
        self.x = np.zeros((4, 1))

        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        self.P = np.eye(4) * 1.0
        self.Q = np.eye(4) * 0.01
        self.R = np.eye(2) * 0.05

    def initialize(self, meas_x, meas_z):
        self.x = np.array([[meas_x], [meas_z], [0], [0]])

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, meas_x, meas_z):
        z = np.array([[meas_x], [meas_z]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def get_position(self):
        return self.x[0, 0].item(), self.x[1, 0].item()


df = pd.read_csv("tracking_log.csv")
df["target"] = df["target"].astype(str).str.strip()

results = []

for target_name in df["target"].unique():
    target_df = df[df["target"] == target_name].reset_index(drop=True)

    kf = KalmanFilter2D()
    kf.initialize(target_df.loc[0, "meas_x"], target_df.loc[0, "meas_z"])

    for i, row in target_df.iterrows():
        kf.predict()

        measurement_available = np.random.rand() > DROP_PROBABILITY

        if measurement_available:
            kf.update(row["meas_x"], row["meas_z"])

        est_x, est_z = kf.get_position()

        results.append({
            "time": row["time"],
            "target": target_name,
            "true_x": row["true_x"],
            "true_z": row["true_z"],
            "meas_x": row["meas_x"] if measurement_available else np.nan,
            "meas_z": row["meas_z"] if measurement_available else np.nan,
            "measurement_available": measurement_available,
            "est_x": est_x,
            "est_z": est_z
        })

results_df = pd.DataFrame(results)
results_df.to_csv("self_healing_results.csv", index=False)

print("Self-healing tracking completed.")
print("Saved: self_healing_results.csv")

for target_name in results_df["target"].unique():
    target_df = results_df[results_df["target"] == target_name]

    true_pos = target_df[["true_x", "true_z"]].values
    est_pos = target_df[["est_x", "est_z"]].values

    available_df = target_df[target_df["measurement_available"] == True]
    true_meas_pos = available_df[["true_x", "true_z"]].values
    meas_pos = available_df[["meas_x", "meas_z"]].values

    meas_rmse = np.sqrt(np.mean((true_meas_pos - meas_pos) ** 2))
    self_healing_rmse = np.sqrt(np.mean((true_pos - est_pos) ** 2))

    loss_rate = 1 - target_df["measurement_available"].mean()

    print(f"\n{target_name}")
    print(f"Measurement loss rate: {loss_rate * 100:.2f}%")
    print(f"Available Measurement RMSE: {meas_rmse:.4f}")
    print(f"Self-Healing Kalman RMSE: {self_healing_rmse:.4f}")

    plt.figure()
    plt.plot(target_df["true_x"], target_df["true_z"], label="True Position")
    plt.scatter(target_df["meas_x"], target_df["meas_z"], s=8, label="Available Measurements")
    plt.plot(target_df["est_x"], target_df["est_z"], label="Self-Healing Kalman Estimate")

    plt.title(f"Self-Healing Tracking - {target_name}")
    plt.xlabel("X Position")
    plt.ylabel("Z Position")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{target_name}_self_healing_plot.png", dpi=300)
    plt.close()