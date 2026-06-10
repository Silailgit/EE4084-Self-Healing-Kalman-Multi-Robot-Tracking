# A Self-Healing Kalman Filter Framework for Multi-Robot Tracking

## Overview

This project presents a multi-robot tracking framework developed for autonomous indoor service robots operating in a restaurant environment. The system was implemented and evaluated in the Webots simulator using noisy position measurements.

The framework includes:

* Standard Kalman Filter (KF)
* Extended Kalman Filter (EKF)
* Self-Healing Tracking Mechanism
* Multi-Robot Tracking
* RMSE-Based Performance Evaluation

The objective is to maintain accurate robot position estimation even when measurements are corrupted by noise or temporarily unavailable.

---

## Features

* Tracking of multiple autonomous service robots
* Noisy measurement simulation
* Kalman Filter based state estimation
* Extended Kalman Filter comparison
* Self-healing tracking during measurement outages
* Performance evaluation using RMSE metrics
* Webots simulation environment

---

## Simulation Setup

| Parameter             | Value     |
| --------------------- | --------- |
| Arena Size            | 2 m × 2 m |
| Number of Robots      | 2         |
| Number of Tables      | 10        |
| Measurement Noise     | 0.02 m    |
| Time Step             | 64 ms     |
| Measurement Loss Rate | 15–17%    |

---

## Project Structure

```text
.
├── controllers/
├── worlds/
├── tracking_log.csv
├── kalman_tracking.py
├── ekf_tracking.py
├── self_healing_tracking.py
├── results/
└── README.md
```

---

## Requirements

* Python 3.10+
* NumPy
* Pandas
* Matplotlib
* Webots
---

## Running the Project

1. Open the Webots world.
2. Run the simulation.
3. Generate tracking data.
4. Execute the desired tracking algorithm:

```bash
python kalman_tracking.py
```

```bash
python ekf_tracking.py
```

```bash
python self_healing_tracking.py
```

---

## Authors

* Sıla Ilgıt Kılınç – Marmara University
* Yiğit Yılmaz – Marmara University

---

## Reference

Y. Yılmaz and S. I. Kılınç, "A Self-Healing Kalman Filter Framework for Multi-Robot Tracking in Indoor Service Environments", 2026.
