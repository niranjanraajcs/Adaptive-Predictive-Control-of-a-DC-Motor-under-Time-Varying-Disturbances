# EKF-Enhanced Model Predictive Control for a DC Motor

This repository contains the implementation and reports for a course project on advanced state estimation and control of a nonlinear DC motor. The project explores the integration of an Extended Kalman Filter (EKF) with a nonlinear Model Predictive Controller (MPC) to achieve robust tracking performance under measurement noise, physical constraints, and model-plant mismatch.

## Project Overview

Model Predictive Control (MPC) is a powerful strategy for handling multivariable dynamics and constraints. However, its effectiveness relies heavily on accurate state information. In this DC motor system, while the angular velocity $\omega(t)$ is measurable, the armature current $i_a(t)$ is not. This project implements an EKF to reconstruct the full state vector and adapts it to estimate unknown, time-varying disturbances (load torque) to maintain optimal closed-loop control.

This project was developed as part of a course in the Automation and Robotics program at the Department of Electrical Engineering, Technische Universität Dortmund.

## System Model

The system is a nonlinear continuous-time DC motor, modeled with the following state-space representation:

$$ \dot{i}_a(t) = -\frac{R_a}{L_a}i_a(t) - \frac{k_m}{L_a}\omega(t)u(t) + \frac{u_a(t)}{L_a} + w_1(t) $$
$$ \dot{\omega}(t) = -\frac{B}{J}\omega(t) + \frac{k_m}{J}i_a(t)u(t) - \frac{\tau_l}{J} + w_2(t) $$

*   **States:** Armature current $i_a(t)$ and angular velocity $\omega(t)$
*   **Input:** Field current $u(t)$
*   **Measured Output:** $y(t) = \omega(t) + v(t)$ (with zero-mean Gaussian noise $v(t)$)

## Key Tasks & Implementations

### 1. EKF Discretization Study
We evaluated the impact of numerical discretization methods on EKF performance independently of the MPC. Methods tested include Forward/Backward Euler, Runge-Kutta 4 (RK4), Orthogonal Collocation, Heun's Method, Tustin's, and ZOH.
*   **Result:** RK4 was selected for the closed-loop implementation as it offered the best trade-off between estimation accuracy, numerical stability, and MPC compatibility.

### 2. Nonlinear MPC (Ideal Conditions)
A baseline nonlinear MPC was designed assuming full state feedback with no noise or mismatch. 
*   **Formulation:** Finite-horizon optimal control ($\Delta t = 0.05$ s, $N = 20$).
*   **Solver:** Discretized using orthogonal collocation (Radau points, $K=2$) and solved via IPOPT.
*   **Result:** Achieved smooth, constraint-aware tracking of velocity setpoints (30, -36, -175 rad/s).

### 3. EKF-Enhanced MPC (Noise & Mismatch)
The EKF and MPC were integrated to handle measurement noise and a static load torque mismatch ($\tau_l^{plant} = 1.47$ Nm, $\tau_l^{model} = 2.0$ Nm).
*   **Result:** The standard EKF effectively filtered noise, but model-plant mismatch degraded tracking. Augmenting the EKF to estimate the constant load torque online successfully eliminated the steady-state tracking error.

### 4. Additional Task: Time-Varying Load Torque Estimation
As an advanced extension, the augmented EKF framework was evaluated against a dynamically varying sinusoidal load torque:
$$ \tau_l(k) = \tau_{\text{offset}} + A \sin(fk) $$
*(where $\tau_{\text{offset}} = 1.47$ Nm, $A = 0.5$ Nm, $f = 0.05$ rad/sample).*

*   **Methodology:** The state vector was augmented to $x = [i_a, \omega, \tau_l]^T$. The process noise covariance matrix $Q_{EKF}$ was tuned (setting $Q_{33} = 0.01$) to act as a forgetting factor, allowing the EKF to track dynamic changes.
*   **Result:** The EKF successfully converged to the true time-varying torque within 2-3 seconds. Despite a small, inherent phase lag, the online parameter estimation allowed the MPC to maintain highly robust speed tracking (RMSE = 3.12 rad/s) under dynamic disturbances.

## Technologies Used
*   Nonlinear Model Predictive Control (MPC)
*   Extended Kalman Filter (EKF)
*   IPOPT Solver
*   Orthogonal Collocation on Finite Elements

## Setup and Usage
*(Instructions on how to run the simulation, required dependencies, e.g., MATLAB/Simulink, CasADi, or Python/SciPy, can be added here.)*

1. Clone the repository:
   
```bash
   git clone [https://github.com/yourusername/your-repo-name.git](https://github.com/yourusername/your-repo-name.git)
