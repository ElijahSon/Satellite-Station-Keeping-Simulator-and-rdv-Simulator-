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

## 1. Nonlinear Orbital Dynamics

The spacecraft motion is modeled using Newton’s Second Law in an inertial geocentric frame:

```text
a = a_kep + a_J2 + a_atm + a_prop
```

Where:

| Term     | Description                          |
| -------- | ------------------------------------ |
| `a_kep`  | Keplerian gravitational acceleration |
| `a_J2`   | Earth oblateness perturbation        |
| `a_atm`  | Atmospheric drag acceleration        |
| `a_prop` | Propulsion acceleration              |

---

# Keplerian Gravitational Model

The central gravitational acceleration is:

```text
a_kep = -μ r / |r|³
```

Where:

| Variable | Value                     |
| -------- | ------------------------- |
| `μ`      | 3.986004418 × 10¹⁴ m³/s²  |
| `r`      | Satellite position vector |

---

# Satellite Physical Parameters

The project uses the physical characteristics of the MICROSCOPE satellite:

| Parameter                | Value             |
| ------------------------ | ----------------- |
| Satellite Mass `m`       | 191.1 kg          |
| Drag Coefficient `C_D`   | 2                 |
| Cross-sectional Area `S` | 2 m²              |
| Atmospheric Density `ρ₀` | 6.3 × 10⁻¹³ kg/m³ |

These values were extracted directly from the project statement.

---

# Reference Orbit Parameters

The desired reference orbit is defined by:

| Orbital Parameter       | Value   |
| ----------------------- | ------- |
| Semi-major axis `a`     | 7068 km |
| Eccentricity `e`        | 0       |
| Inclination `i`         | 0 rad   |
| RAAN `Ω`                | 0 rad   |
| Argument of Perigee `ω` | 0 rad   |
| True Anomaly `ν₀`       | 0 rad   |

This corresponds to a circular Low Earth Orbit (LEO).

---

# Orbital Position Equation

The orbital radius is:

```text
r = a(1 - e²) / (1 + e cos(ν))
```

Where:

| Variable | Description          |
| -------- | -------------------- |
| `a`      | Semi-major axis      |
| `e`      | Orbital eccentricity |
| `ν`      | True anomaly         |

---

# Orbital Mean Motion

The orbital mean motion is:

```text
n = √(μ / a³)
```

For the reference orbit:

| Quantity           | Approximate Value    |
| ------------------ | -------------------- |
| Orbital Radius     | 7068 km              |
| Mean Motion `n`    | ≈ 0.00106 rad/s      |
| Orbital Period `T` | ≈ 5926 s (~98.8 min) |

---

# Atmospheric Drag Model

Atmospheric drag acceleration is modeled as:

```text
a_atm = -(ρ / 2m) S C_D v² (v / |v|)
```

Where:

| Variable | Description          |
| -------- | -------------------- |
| `ρ`      | Atmospheric density  |
| `m`      | Satellite mass       |
| `S`      | Cross-sectional area |
| `C_D`    | Drag coefficient     |
| `v`      | Relative velocity    |

Atmospheric drag causes progressive orbital decay and energy dissipation.

---

# Relative Orbital Motion

The relative position between the target satellite and the chaser satellite is expressed in the Local Orbital Frame:

```text
Δr = [ ξ  η  ζ ]ᵀ
```

Where:

| Coordinate | Direction             |
| ---------- | --------------------- |
| `ξ`        | Radial direction      |
| `η`        | Along-track direction |
| `ζ`        | Cross-track direction |

---

# Clohessy-Wiltshire (Hill) Equations

After linearization around the circular orbit, the relative dynamics become:

```text
ξ¨ = 2nη̇ + 3n²ξ + u_ξ
η¨ = -2nξ̇ + u_η
ζ¨ = -n²ζ + u_ζ
```

These equations describe the relative motion of nearby spacecraft in orbit.

---

# State-Space Representation

The system state vector is:

```text
x = [ ξ  η  ζ  ξ̇  η̇  ζ̇ ]ᵀ
```

The linearized system is written as:

```text
ẋ = Ax + Bu
```

Where:

| Matrix | Description            |
| ------ | ---------------------- |
| `A`    | System dynamics matrix |
| `B`    | Input matrix           |
| `u`    | Control input vector   |

---

# LQR Control Law

The controller is synthesized using a Linear Quadratic Regulator (LQR).

The feedback control law is:

```text
u = -Kx
```

The gain matrix `K` minimizes the cost function:

```text
J = ∫ (xᵀQx + uᵀRu) dt
```

balancing:

* trajectory precision,
* stabilization performance,
* propulsion effort.

---

# MATLAB Controller Synthesis

```matlab id="vgm2hf"
Q = eye(6);

R1 = 1e12 * eye(3);
R2 = 1e14 * eye(3);

K1 = lqr(A,B,Q,R1);
K2 = lqr(A,B,Q,R2);
```

---

# Initial Relative Orbit Test Cases

Two validation scenarios are implemented:

## 1. Eccentricity Offset

| Parameter    | Target | Chaser  |
| ------------ | ------ | ------- |
| Eccentricity | 0      | 1.41e-5 |

This generates relative elliptical motion.

---

## 2. True Anomaly Offset

| Parameter    | Target | Chaser      |
| ------------ | ------ | ----------- |
| True Anomaly | 0 rad  | 1.41e-5 rad |

This produces phase-shift relative motion along the orbit.

---

# Perturbation Scenarios

The controller is validated under:

* No perturbations
* Atmospheric drag only
* J2 perturbation only
* Combined perturbations

---

# Engineering Significance

This project combines:

* Aerospace dynamics
* Nonlinear system modeling
* Orbital mechanics
* Optimal control theory
* State-space synthesis
* MATLAB/Simulink engineering workflows

The methodologies used are directly relevant to:

* autonomous spacecraft guidance,
* orbital rendezvous,
* satellite formation flying,
* aerospace GNC systems,
* modern autonomous space missions.
