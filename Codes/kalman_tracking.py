import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


DT = 0.064  # Webots TIME_STEP = 64 ms


class KalmanFilter2D:
    def __init__(self, dt=DT):
        self.dt = dt

        # State: [x, z, vx, vz]
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
        return float(self.x[0]), float(self.x[1])


df = pd.read_csv("tracking_log.csv")
print(df.head(10))
print(df["target"].unique())
print(df["target"].value_counts())

results = []

for target_name in df["target"].unique():
    target_df = df[df["target"] == target_name].reset_index(drop=True)

    kf = KalmanFilter2D()
    kf.initialize(target_df.loc[0, "meas_x"], target_df.loc[0, "meas_z"])

    for _, row in target_df.iterrows():
        kf.predict()
        kf.update(row["meas_x"], row["meas_z"])

        est_x, est_z = kf.get_position()

        results.append({
            "time": row["time"],
            "target": target_name,
            "true_x": row["true_x"],
            "true_z": row["true_z"],
            "meas_x": row["meas_x"],
            "meas_z": row["meas_z"],
            "est_x": est_x,
            "est_z": est_z
        })

results_df = pd.DataFrame(results)
results_df.to_csv("kalman_results.csv", index=False)

print("Kalman tracking completed.")
print("Saved: kalman_results.csv")


for target_name in results_df["target"].unique():
    target_df = results_df[results_df["target"] == target_name]

    true_pos = target_df[["true_x", "true_z"]].values
    meas_pos = target_df[["meas_x", "meas_z"]].values
    est_pos = target_df[["est_x", "est_z"]].values

    meas_rmse = np.sqrt(np.mean((true_pos - meas_pos) ** 2))
    kalman_rmse = np.sqrt(np.mean((true_pos - est_pos) ** 2))

    print(f"\n{target_name}")
    print(f"Measurement RMSE: {meas_rmse:.4f}")
    print(f"Kalman RMSE: {kalman_rmse:.4f}")

    plt.figure()
    plt.plot(target_df["true_x"], target_df["true_z"], label="True Position")
    plt.scatter(target_df["meas_x"], target_df["meas_z"], s=8, label="Noisy Measurement")
    plt.plot(target_df["est_x"], target_df["est_z"], label="Kalman Estimate")

    plt.title(f"Kalman Tracking - {target_name}")
    plt.xlabel("X Position")
    plt.ylabel("Z Position")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{target_name}_kalman_plot.png", dpi=300)
    plt.show()