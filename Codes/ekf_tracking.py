import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

DT = 0.064


class EKF2D:
    def __init__(self, dt=DT):
        self.dt = dt

        # State: [x, y, v, theta]
        self.x = np.zeros((4, 1))

        self.P = np.eye(4) * 1.0
        self.Q = np.diag([0.001, 0.001, 0.01, 0.01])
        self.R = np.eye(2) * 0.0004

    def initialize(self, meas_x, meas_y):
        self.x = np.array([[meas_x], [meas_y], [0.05], [0.0]])

    def predict(self):
        x, y, v, theta = self.x.flatten()
        dt = self.dt

        self.x = np.array([
            [x + v * np.cos(theta) * dt],
            [y + v * np.sin(theta) * dt],
            [v],
            [theta]
        ])

        F = np.array([
            [1, 0, np.cos(theta) * dt, -v * np.sin(theta) * dt],
            [0, 1, np.sin(theta) * dt,  v * np.cos(theta) * dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.P = F @ self.P @ F.T + self.Q

    def update(self, meas_x, meas_y):
        z = np.array([[meas_x], [meas_y]])

        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    def get_position(self):
        return self.x[0, 0].item(), self.x[1, 0].item()


df = pd.read_csv("tracking_log.csv")
df["target"] = df["target"].astype(str).str.strip()

results = []

for target_name in df["target"].unique():
    target_df = df[df["target"] == target_name].reset_index(drop=True)

    ekf = EKF2D()
    ekf.initialize(target_df.loc[0, "meas_x"], target_df.loc[0, "meas_z"])

    for _, row in target_df.iterrows():
        ekf.predict()
        ekf.update(row["meas_x"], row["meas_z"])

        est_x, est_y = ekf.get_position()

        results.append({
            "time": row["time"],
            "target": target_name,
            "true_x": row["true_x"],
            "true_z": row["true_z"],
            "meas_x": row["meas_x"],
            "meas_z": row["meas_z"],
            "ekf_x": est_x,
            "ekf_z": est_y
        })

results_df = pd.DataFrame(results)
results_df.to_csv("ekf_results.csv", index=False)

print("EKF tracking completed.")
print("Saved: ekf_results.csv")

for target_name in results_df["target"].unique():
    target_df = results_df[results_df["target"] == target_name]

    true_pos = target_df[["true_x", "true_z"]].values
    meas_pos = target_df[["meas_x", "meas_z"]].values
    ekf_pos = target_df[["ekf_x", "ekf_z"]].values

    meas_rmse = np.sqrt(np.mean((true_pos - meas_pos) ** 2))
    ekf_rmse = np.sqrt(np.mean((true_pos - ekf_pos) ** 2))

    print(f"\n{target_name}")
    print(f"Measurement RMSE: {meas_rmse:.4f}")
    print(f"EKF RMSE: {ekf_rmse:.4f}")

    plt.figure()
    plt.plot(target_df["true_x"], target_df["true_z"], label="True Position")
    plt.scatter(target_df["meas_x"], target_df["meas_z"], s=8, label="Noisy Measurement")
    plt.plot(target_df["ekf_x"], target_df["ekf_z"], label="EKF Estimate")

    plt.title(f"EKF Tracking - {target_name}")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{target_name}_ekf_plot.png", dpi=300)
    plt.close()