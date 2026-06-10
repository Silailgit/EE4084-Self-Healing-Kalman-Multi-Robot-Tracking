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
* Kalman Filter-based state estimation
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

## Repository Structure

```text
.
├── Codes/
│   ├── kalman_tracking.py
│   ├── ekf_tracking.py
│   └── self_healing_tracking.py
│
├── laTeX/
│   ├── main.tex
│   ├── references.bib
│   ├── figures/
│   └── ...
│
├── results/
│   ├── TARGET_1_ekf_plot.png
│   ├── TARGET_1_kalman_plot.png
│   ├── TARGET_1_self_healing_plot.png
│   ├── TARGET_2_ekf_plot.png
│   ├── TARGET_2_kalman_plot.png
│   ├── TARGET_2_self_healing_plot.png
│   ├── tracking_log.csv
│   ├── kalman_results.csv
│   ├── ekf_results.csv
│   └── self_healing_results.csv
│
├── EE4084_presentation.pdf
├── kalman_conference_paper.pdf
├── world.wbt
└── README.md
```

---

## Requirements

* Python 3.10+
* NumPy
* Pandas
* Matplotlib
* Webots

Install dependencies:

```bash
pip install numpy pandas matplotlib
```

---

## Running the Project

1. Open `world.wbt` in Webots.
2. Run the simulation to generate robot position measurements.
3. Execute the desired tracking algorithm from the `Codes` folder.

Kalman Filter:

```bash
python Codes/kalman_tracking.py
```

Extended Kalman Filter:

```bash
python Codes/ekf_tracking.py
```

Self-Healing Kalman Filter:

```bash
python Codes/self_healing_tracking.py
```

---

## Results

The Kalman Filter reduced tracking error by approximately 49% compared to raw noisy measurements.

| Method          | TARGET 1 RMSE (m) | TARGET 2 RMSE (m) |
| --------------- | ----------------- | ----------------- |
| Measurements    | 0.0117            | 0.0115            |
| Kalman Filter   | 0.0060            | 0.0059            |
| Self-Healing KF | 0.0065            | 0.0062            |
| EKF             | 0.0095            | 0.0092            |

---

## Authors

* Sıla Ilgıt Kılınç – Marmara University
* Yiğit Yılmaz – Marmara University

---

## Included Materials

* IEEE Conference Paper (PDF)
* Complete LaTeX Source Files
* Python Source Code
* Webots Simulation World
* Experimental Results
* Project Presentation Slides
