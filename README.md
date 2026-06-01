# Autonomous Satellite Station-Keeping using Nonlinear Orbital Dynamics and LQR Control

<p align="center">
  <img src="https://img.shields.io/badge/MATLAB-Simulink-orange?style=for-the-badge">
  <img src="https://img.shields.io/badge/Control%20Systems-LQR-blue?style=for-the-badge">
  <img src="https://img.shields.io/badge/Aerospace-Orbital%20Dynamics-black?style=for-the-badge">
  <img src="https://img.shields.io/badge/University%20of%20Bordeaux-AM2AS-red?style=for-the-badge">
</p>

---

# Project Overview

This project focuses on the **autonomous station-keeping of a Low Earth Orbit (LEO) satellite** using:

* Nonlinear orbital dynamics modeling
* Relative orbital motion analysis
* Linearization of spacecraft dynamics
* State-space control synthesis
* Linear Quadratic Regulation (LQR)
* MATLAB / Simulink simulation and validation

The work was developed as part of the **DSAS (Dynamique des Systèmes Aéronautiques et Spatiaux)** engineering project from the **AM2AS** track:

> **Automatique et Mécatronique, Automobile, Aéronautique & Spatial**

within the **Master’s Degree in Complex Systems Engineering** at the **University of Bordeaux / ENSEIRB-MATMECA**.

The objective is to design a robust feedback controller capable of maintaining a satellite on its desired orbit despite disturbances caused by:

* Earth oblateness (J2 perturbation)
* Atmospheric drag
* Relative orbital drift
* Nonlinear orbital dynamics

---

# Engineering Context

Modern satellites are continuously subjected to perturbing forces that progressively alter their orbit.

Without correction:

* orbital altitude drifts,
* relative positioning errors accumulate,
* mission precision degrades,
* fuel consumption increases.

Traditional station-keeping strategies rely heavily on ground stations and intermittent corrective maneuvers.

This project instead explores an **autonomous onboard closed-loop control strategy**, where the spacecraft continuously estimates its orbital deviation and computes corrective thrust commands in real time.

This type of methodology is directly related to:

* satellite formation flying,
* autonomous rendezvous,
* orbital maintenance,
* spacecraft guidance and navigation,
* aerospace GNC (Guidance, Navigation & Control),
* proximity operations.

---

# Problem Statement

Two satellites are considered:

| Satellite        | Description             |
| ---------------- | ----------------------- |
| Target Satellite | Desired reference orbit |
| Chaser Satellite | Controlled spacecraft   |

The goal is to force the chaser satellite to remain close to the target orbit despite perturbations.

The controller must:

* minimize relative orbital error,
* stabilize relative motion,
* reject disturbances,
* reduce propulsion effort.

---

# Mathematical Modeling

# 1. Nonlinear Orbital Dynamics

The spacecraft dynamics are derived from Newton’s Second Law in an inertial geocentric frame:

[
\vec{a} = \vec{a}*{kep} + \vec{a}*{J2} + \vec{a}*{atm} + \vec{a}*{prop}
]

where:

| Term               | Description                          |
| ------------------ | ------------------------------------ |
| ( \vec{a}_{kep} )  | Keplerian gravitational acceleration |
| ( \vec{a}_{J2} )   | Earth oblateness perturbation        |
| ( \vec{a}_{atm} )  | Atmospheric drag                     |
| ( \vec{a}_{prop} ) | Propulsion acceleration              |

---

## Keplerian Acceleration

The central gravitational field is modeled as:

[
\vec{a}_{kep} = -\mu \frac{\vec{r}}{r^3}
]

where:

* ( \mu ) = Earth's gravitational parameter
* ( \vec{r} ) = satellite position vector

---

## J2 Perturbation

Earth is not perfectly spherical.

Its equatorial bulge introduces a perturbation represented by the second zonal harmonic coefficient (J_2):

[
\vec{a}*{J2} =
\frac{1}{2}\mu J_2 R*{eq}^2
\left(
\frac{3}{r^5} -
\frac{15z^2}{r^7}
\right)\vec{r}
]

The J2 perturbation causes:

* orbital plane precession,
* drift in orbital parameters,
* long-term trajectory deviation.

---

## Atmospheric Drag

Atmospheric drag is modeled as:

[
\vec{a}_{atm} =
-\frac{\rho}{2m}SC_Dv^2\frac{\vec{v}}{v}
]

where:

| Parameter | Description                    |
| --------- | ------------------------------ |
| ( \rho )  | Atmospheric density            |
| ( S )     | Satellite cross-sectional area |
| ( C_D )   | Drag coefficient               |
| ( m )     | Satellite mass                 |

Atmospheric drag progressively reduces orbital altitude and velocity.

---

# State-Space Representation

The nonlinear state vector is:

[
X =
\begin{bmatrix}
x & y & z & \dot{x} & \dot{y} & \dot{z}
\end{bmatrix}^T
]

The nonlinear system is represented as:

[
\dot{X} = f(X,U,W)
]

where:

* (U) = control input,
* (W) = perturbation vector.

---

# Relative Orbital Motion

To synthesize the controller, the nonlinear dynamics are linearized around a circular reference orbit.

The relative position between the target and chaser satellites is expressed in the local orbital frame:

[
\Delta r =
\begin{bmatrix}
\xi & \eta & \zeta
\end{bmatrix}^T
]

where:

| Coordinate | Direction   |
| ---------- | ----------- |
| ( \xi )    | Radial      |
| ( \eta )   | Along-track |
| ( \zeta )  | Cross-track |

---

# Clohessy-Wiltshire (Hill) Equations

The resulting linearized dynamics are:

[
\ddot{\xi} = 2n\dot{\eta} + 3n^2\xi + u_{\xi}
]

[
\ddot{\eta} = -2n\dot{\xi} + u_{\eta}
]

[
\ddot{\zeta} = -n^2\zeta + u_{\zeta}
]

where:

[
n = \sqrt{\frac{\mu}{a^3}}
]

is the orbital mean motion.

These equations describe the relative dynamics of two nearby spacecraft in orbit.

---

# Linear State-Space Model

The system is rewritten in state-space form:

[
\dot{x} = Ax + Bu
]

with:

[
x =
\begin{bmatrix}
\xi & \eta & \zeta & \dot{\xi} & \dot{\eta} & \dot{\zeta}
\end{bmatrix}^T
]

This linearized model is used for controller synthesis.

---

# Control Design — LQR

A Linear Quadratic Regulator (LQR) is used to compute the optimal feedback gain matrix (K).

The control law is:

[
u = -Kx
]

The controller minimizes the quadratic cost function:

[
J =
\int_0^\infty
(x^TQx + u^TRu),dt
]

which balances:

* trajectory accuracy,
* control effort,
* system stability.

---

# MATLAB Implementation

Example controller synthesis:

```matlab
A = [...];
B = [...];

Q = eye(6);
R = 1e12*eye(3);

K = lqr(A,B,Q,R);
```

---

# Simulation Architecture

The Simulink environment includes:

* Nonlinear orbital propagator
* J2 perturbation model
* Atmospheric drag model
* Relative frame transformation
* Linearized synthesis model
* LQR controller
* Closed-loop validation environment

---

# Orbital Parameters

Reference orbit used in simulations:

| Parameter           | Value   |
| ------------------- | ------- |
| Semi-major axis     | 7068 km |
| Eccentricity        | 0       |
| Inclination         | 0 rad   |
| RAAN                | 0 rad   |
| Argument of perigee | 0 rad   |
| True anomaly        | 0 rad   |

---

# Simulation Results

The simulations demonstrate:

## Open Loop

Without control:

* relative orbit diverges,
* perturbations accumulate,
* orbital drift increases.

## Closed Loop

With LQR control:

* relative motion stabilizes,
* orbital deviation is corrected,
* perturbation effects are mitigated,
* fuel-efficient station-keeping is achieved.

---

# Repository Structure

```text
├── matlab/
│   ├── orbital_conversion.m
│   ├── nonlinear_dynamics.m
│   ├── j2_perturbation.m
│   ├── atmospheric_drag.m
│   ├── relative_motion_model.m
│   ├── lqr_controller.m
│   └── simulation_scripts/
│
├── simulink/
│   ├── nonlinear_validation_model.slx
│   ├── synthesis_model.slx
│   ├── perturbation_models.slx
│   └── closed_loop_stationkeeping.slx
│
├── figures/
│   ├── open_loop_response.png
│   ├── closed_loop_response.png
│   └── orbital_trajectory.png
│
├── report/
│   └── DSAS_Report.pdf
│
└── README.md
```

---

# Key Engineering Concepts Demonstrated

This project demonstrates practical applications of:

* Aerospace dynamics
* Orbital mechanics
* State-space modeling
* Linearization techniques
* Relative motion dynamics
* Robust feedback control
* LQR optimal control
* MATLAB/Simulink engineering workflows
* Nonlinear system validation

---

# Industrial Relevance

The methodologies implemented here are directly applicable to:

* spacecraft GNC systems,
* autonomous rendezvous,
* satellite formation flying,
* orbital maintenance,
* aerospace autonomy,
* real-time embedded control systems.

These concepts are widely used in:

* ESA missions,
* NASA rendezvous systems,
* autonomous spacecraft docking,
* CubeSat formation control,
* defense aerospace applications.

---

# Academic Context

Developed within:

### University of Bordeaux

### Master in Complex Systems Engineering

### AM2AS Track

### ENSEIRB-MATMECA

Specialization areas include:

* Automatic Control
* Aerospace Systems
* Mechatronics
* Vehicle Dynamics
* Robust Control
* Embedded Systems
* Dynamic System Modeling

---

# Future Improvements

Possible extensions of this work include:

* H∞ robust control
* Model Predictive Control (MPC)
* Kalman filtering
* Nonlinear adaptive control
* Reaction wheel attitude control
* Multi-satellite coordination
* Real-time embedded implementation
* High-fidelity perturbation models
* Fault-tolerant spacecraft control

---

# References

1. J.P. Carrou — *Mécanique Spatiale*
2. Montenbruck & Gill — *Satellite Orbits*
3. M.J. Sidi — *Spacecraft Dynamics and Control*
4. Sengupta — *Satellite Relative Motion Propagation and Control*
5. Zarrouati — *Trajectoires Spatiales*

---

# Author

## Elijah Boungoueres

M.Sc. Complex Systems Engineering — AM2AS Track
University of Bordeaux / ENSEIRB-MATMECA

Fields of interest:

* Control Systems
* Aerospace Dynamics
* Embedded Systems
* Mechatronics
* Robust Control
* Power Electronics
* MATLAB/Simulink Modeling
* Autonomous Systems

---

# License

This repository is intended for:

* academic use,
* research purposes,
* engineering education,
* aerospace control system studies.
