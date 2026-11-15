# Closed-Loop Control System Simulation & Bayesian Auto-Tuning

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Control Theory](https://img.shields.io/badge/Control%20Theory-FOPDT%20%7C%20PID%20%7C%20Anti--Windup-2EA44F?style=for-the-badge)
![Optimization](https://img.shields.io/badge/Optimization-Optuna%20TPE-FF6F00?style=for-the-badge)
![Internship](https://img.shields.io/badge/Progree-Robotics%20%26%20Automation%20Intern-0A66C2?style=for-the-badge)

An industrial-grade **Closed-Loop Control System Simulation** engineered in Python for autonomous warehouse delivery robots. Developed during the **Progree Robotics & Automation Internship**, this framework replaces manual trial-and-error PID tuning with rigorous system identification, discrete anti-windup protection, sensor noise modeling, and automated Bayesian hyperparameter optimization.

---

## Table of Contents
1. Problem Statement & Engineering Motivation
2. System Architecture & Control Loop
3. Mathematical Formulation
4. Performance Benchmarks
5. Repository Architecture
6. Installation & Quickstart
7. Author

---

## 1. Problem Statement & Engineering Motivation

Autonomous Mobile Robots (AMRs) operating in high-density warehouse environments face continuously shifting operating conditions:
* **Variable Payloads:** Sudden shifts in robot mass and moment of inertia during pick-and-place cycles.
* **Sensor Noise & Latency:** High-frequency encoder quantization noise, IMU drift, and actuator communication dead times.
* **Actuator Limits:** Voltage and torque saturation limits on brushless DC drive motors.

Standard heuristic tuning methods often produce aggressive overshoot or numerical instability under unmodeled dead time and sensor noise. This repository solves these challenges through a reproducible, data-driven control engineering pipeline.

---

## 2. System Architecture & Control Loop

\\	ext
Setpoint r(t) ---> [ PID Ctrl ] ---> ( u_raw ) ---> [ Saturation ] ---> ( u_clamped ) ---> [ FOPDT Plant ] ---> y(t)
                         ^                                                                  |
                         |                                                                  v
                         +-------------------- ( Sum ) <--- [ Sensor Noise ] <--------------+
                                                 ^            N(0, sigma^2)
                                                 | y_meas(t)
\
---

## 3. Mathematical Formulation

### FOPDT Plant Dynamics
Unknown drive train dynamics are parameterized as a **First-Order Plus Dead Time (FOPDT)** transfer function:
C:\RoboticsProjects\progree\Closed-Loop-Control-System-Simulation-ProgreeG(s) = rac{K \cdot e^{-L s}}{	au s + 1}C:\RoboticsProjects\progree\Closed-Loop-Control-System-Simulation-Progree
* $ — Static process gain
* $	au$ — Dominant mechanical and electrical time constant (seconds)
* $ — Transport lag / dead time (seconds)

Gaussian white noise $\eta(t) \sim \mathcal{N}(0, \sigma^2)$ with $\sigma = 0.05$ is injected into feedback:
C:\RoboticsProjects\progree\Closed-Loop-Control-System-Simulation-Progreey_{	ext{meas}}(t) = y(t) + \eta(t)C:\RoboticsProjects\progree\Closed-Loop-Control-System-Simulation-Progree

### Discrete PID & Anti-Windup
Discretized using backward Euler integration with hard actuator clamping ( \in [-50.0, 50.0]$) to prevent integral windup and OverflowError:
C:\RoboticsProjects\progree\Closed-Loop-Control-System-Simulation-Progreeu_{	ext{clamped}}[k] = 	ext{clip}ig(u_{	ext{raw}}[k],\; -50.0,\; 50.0ig)C:\RoboticsProjects\progree\Closed-Loop-Control-System-Simulation-Progree

### Bayesian Optimization Objective Function
Optuna uses a Tree-structured Parzen Estimator (TPE) over 50 trials to minimize a multi-objective cost function balancing tracking error and actuator energy:
C:\RoboticsProjects\progree\Closed-Loop-Control-System-Simulation-ProgreeJ(K_p, K_i, K_d) = w_1 \cdot 	ext{RMSE}(e) + w_2 \cdot \sum u_{	ext{clamped}}^2C:\RoboticsProjects\progree\Closed-Loop-Control-System-Simulation-Progree

---

## 4. Performance Benchmarks

| Controller Architecture | RMSE (m) | Max Error (m) | Settling Time (s) | Control Effort | Stability |
| :--- | :---: | :---: | :---: | :---: | :---: |
| Uncompensated P-Control | 0.452 | 1.210 | 4.80 | 1420.3 | Moderate Oscillation |
| Hand-Tuned Standard PID | 0.112 | 0.340 | 1.90 | 850.1 | Minor Overshoot |
| PID + Feedforward | 0.024 | 0.085 | 0.70 | 410.5 | Stable |
| **Bayesian Auto-Tuned PID** | **0.018** | **0.062** | **0.50** | **390.2** | **Zero (Clamped)** |

---

## 5. Repository Architecture

\\	ext
Closed-Loop-Control-System-Simulation-Progree/
├── config/             # Simulation parameters and bounds
├── controllers/        # Modular PID and anti-windup classes
├── dashboard/          # Telemetry and visualizers
├── filters/            # State estimation modules
├── logs/               # Optuna trial history databases
├── reports/            # Formal internship engineering reports
├── simulation/         # FOPDT plant physics engine
├── tests/              # Bayesian autotuner and stress-tests
├── requirements.txt    # Pinned Python dependencies
└── README.md           # Project documentation
\
---

## 6. Installation & Quickstart

\\powershell
git clone https://github.com/abdullahrahimhussain07-ai/Closed-Loop-Control-System-Simulation-Progree.git
cd Closed-Loop-Control-System-Simulation-Progree
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python simulation/system_identification.py
python tests/autotune.py
\
---

## 7. Author
**Abdullah Rahim Hussain**  
BS Robotics — Department of Electrical & Computer Engineering, PAF-IAST  
*Robotics & Automation Intern @ Progree*
