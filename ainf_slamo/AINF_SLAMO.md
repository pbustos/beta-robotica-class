# AINF_SLAMO — Technical Documentation

**Active Inference SLAM with LiDAR, MPPI Navigation and a Known Room Model**
Version 2.1 — June 2025

---

## Table of Contents

1. [Overview](#1-overview)
2. [System Architecture](#2-system-architecture)
3. [State Space](#3-state-space)
4. [Active Inference Framework](#4-active-inference-framework)
5. [Generative Model — Room SDF](#5-generative-model--room-sdf)
6. [Variational Free Energy](#6-variational-free-energy)
7. [Dual-Prior Fusion (Command + Odometry)](#7-dual-prior-fusion-command--odometry)
8. [EKF-Style Covariance Propagation](#8-ekf-style-covariance-propagation)
9. [Variational Update — Gradient Descent on F](#9-variational-update--gradient-descent-on-f)
10. [Covariance Update (Bayesian Fusion)](#10-covariance-update-bayesian-fusion)
11. [Innovation and Health Monitoring](#11-innovation-and-health-monitoring)
12. [Velocity-Adaptive Gradient Weights](#12-velocity-adaptive-gradient-weights)
13. [Pose Smoothing (EMA Filter)](#13-pose-smoothing-ema-filter)
14. [Prediction-Based Early Exit](#14-prediction-based-early-exit)
15. [Initialisation and Kidnapping Recovery](#15-initialisation-and-kidnapping-recovery)
16. [Polygon Path Planner](#16-polygon-path-planner)
17. [MPPI Trajectory Controller](#17-mppi-trajectory-controller)
    - [17.10 ESS-Based Adaptive Parameter Tuning](#1710-ess-based-adaptive-parameter-tuning)
  - [17.11 Mood Superparameter (Calm–Excited)](#1711-mood-superparameter-calmexcited)
  - [17.12 Multi-Step Command Extraction](#1712-multi-step-command-extraction)
  - [17.13 Top-K Decisive Blending](#1713-top-k-decisive-blending)
  - [17.14 Catmull-Rom Path Smoothing](#1714-catmull-rom-path-smoothing)
  - [17.15 Lateral Clearance Cost](#1715-lateral-clearance-cost)
  - [17.16 Path Blockage Detection and Obstacle Avoidance](#1716-path-blockage-detection-and-obstacle-avoidance)
18. [Complete Algorithm Loop](#18-complete-algorithm-loop)
19. [Parameters Reference](#19-parameters-reference)
20. [Key Implementation Notes](#20-key-implementation-notes)

---

## 1. Overview

**AINF_SLAMO** estimates the 2-D pose of a robot inside a room whose geometry is
known *a priori* (the *nominal model*) and navigates through it.  Sensor data
comes from a 3-D LiDAR whose returns are projected onto the ground plane.

The system has two main subsystems:

- **Localisation** — grounded in **Active Inference** (Karl Friston, 2010): the
  system minimises a *Variational Free Energy* functional that combines a sensor
  **likelihood** (how well the LiDAR hits the room walls) with a motion-model
  **prior** (where the odometry says the robot should be).  Differentiation is
  done automatically by **LibTorch** (C++ PyTorch).

- **Navigation** — an **MPPI (Model Predictive Path Integral)** trajectory
  controller generates smooth velocity commands using the LiDAR-based ESDF
  (Euclidean Signed Distance Field).  The controller uses **warm-start**,
  **AR(1) correlated noise**, and **structured exploration injections** to handle
  both open corridors and narrow doorways.

---

## 2. System Architecture

```
┌───────────────┐   Webots2Robocomp   ┌──────────────────────┐
│  Webots sim   │ ─ ─ ─ ─ ─ ─ ─ ─ ► │  read_lidar() thread │
│  (GT only,    │  (optional, for     │  • GT pose (optional) │
│   debug)      │   use_webots=true)  │  • LiDAR 3D points   │
└───────────────┘                     └──────────┬───────────┘
                                                 │ buffer_sync
                                                 ▼
  JoystickAdapter ─── cmd velocity ──►┌──────────────────────┐
                                      │   compute() @ 20 Hz  │
  FullPoseEstimationPub               │   RoomConceptAI::     │
    (encoders/IMU) ── odom velocity ─►│     update()          │
                                      └──────────┬───────────┘
                                                 │
                 ┌───────────────────────────┬────┴────┬──────────────────┐
                 ▼                           ▼         ▼                  ▼
          Dual-Prior Fusion           EKF Predict   VFE minim.    Covariance update
          (cmd ⊕ odometry)            (propagate)   (Adam optim)  (Bayesian fusion)

  Shift+Right click ──────►┌──────────────────────┐
    (set target)           │  PolygonPathPlanner   │───► path waypoints
  Ctrl+Right click ────►   │  (Visibility Graph    │     (Catmull-Rom spline)
    (cancel mission)       │   + obstacle support) │         │
                           └───────────▲──────────┘         ▼
                                       │          ┌──────────────────────┐
                          LiDAR points ┼─────────►│  TrajectoryController │
                          Robot pose ──┼─────────►│  (MPPI + ESDF)        │
                                       │          │  • Warm-start seq.    │
                                       │          │  • AR(1) sampling     │
                                       │          │  • Injection seeds    │
                                       │          │  • Lateral clearance  │
                                       │          │  • Blockage detection │
                                       │          └──────────┬───────────┘
                                       │                     │
                              replan   │    path_blocked?    │
                              ◄────────┘◄────────────────────┘
                           (cluster LiDAR →                  │
                            OBB polygon →                    ▼
                            temp obstacle)          (adv, rot) → robot
```

> **Note:** The Webots GT pose is used **only** for debug error statistics
> (GT Δxy, GT Δθ displays). The localisation algorithm relies exclusively on
> LiDAR + dual odometry priors. Set `use_webots = false` in `etc/config` for
> real robot operation.

---

## 3. State Space

The optimised state is the **robot pose in the room frame**:

$$\mathbf{s} = \begin{bmatrix} x \\ y \\ \phi \end{bmatrix} \in \mathbb{R}^2 \times [-\pi,\pi)$$

where $(x, y)$ is the robot position and $\phi$ is its heading, both expressed
relative to the room origin (centre of the nominal polygon).

The room geometry (polygon vertices) is **fixed** — only the robot pose is
optimised.

The full 5-D internal state vector, used for backward compatibility, is:

$$\mathbf{q} = [\underbrace{w,\, l}_{\text{room dims (fixed)}},\; \underbrace{x,\, y,\, \phi}_{\text{robot pose (optimised)}}]$$

---

## 4. Active Inference Framework

Active Inference casts perception as **variational Bayesian inference**.
The brain (or robot) maintains a *generative model* $p(\mathbf{o} \mid \mathbf{s})$
and a recognition density $q(\mathbf{s})$.  It minimises the
**Variational Free Energy**:

$$\mathcal{F}[q] = \underbrace{-\mathbb{E}_{q}[\ln p(\mathbf{o} \mid \mathbf{s})]}_{\text{Likelihood (sensory fit)}}
                 + \underbrace{D_{\mathrm{KL}}[q(\mathbf{s}) \,\|\, p(\mathbf{s})]}_{\text{Complexity (prior fit)}}$$

In the Laplace (Gaussian) approximation $q(\mathbf{s}) = \mathcal{N}(\pmb{\mu}, \pmb{\Sigma})$,
this reduces to:

$$\mathcal{F} = \frac{1}{2\sigma_{\mathrm{obs}}^2}\,\mathcal{L}_{\mathrm{lik}}(\pmb{\mu})
              + \frac{1}{2}(\pmb{\mu} - \pmb{\mu}_{\mathrm{prior}})^{\top}
                \pmb{\Pi}_{\mathrm{prior}}
                (\pmb{\mu} - \pmb{\mu}_{\mathrm{prior}})$$

where:

| Symbol | Meaning |
|--------|---------|
| $\pmb{\mu} = [x, y, \phi]^{\top}$ | Current mean state (robot pose) |
| $\pmb{\mu}_{\mathrm{prior}}$ | Predicted pose from odometry |
| $\pmb{\Pi}_{\mathrm{prior}} = \pmb{\Sigma}_{\mathrm{prior}}^{-1}$ | Precision (inverse predicted covariance) |
| $\sigma_{\mathrm{obs}} = 0.05\,\mathrm{m}$ | Observation noise std (5 cm) |
| $\mathcal{L}_{\mathrm{lik}}$ | Huber-robust LiDAR likelihood loss |

---

## 5. Generative Model — Room SDF

The generative model relates robot pose to expected LiDAR observations via a
**Signed Distance Function (SDF)** of the room boundary.

### 5.1 Coordinate Transform

LiDAR returns arrive in the **robot frame** $\mathbf{p}^r = (p^r_x, p^r_y)$.
They are mapped to the **room frame** by:

$$\mathbf{p}^{\mathrm{room}} = R(\phi)\,\mathbf{p}^r + \mathbf{t}$$

where $R(\phi) \in SO(2)$ is the 2-D rotation matrix for heading $\phi$ and
$\mathbf{t} = (x, y)^{\top}$ is the robot translation.

Explicitly:

$$\begin{bmatrix} p^{\mathrm{room}}_x \\ p^{\mathrm{room}}_y \end{bmatrix}
  = \begin{bmatrix} \cos\phi & -\sin\phi \\ \sin\phi & \cos\phi \end{bmatrix}
    \begin{bmatrix} p^r_x \\ p^r_y \end{bmatrix}
  + \begin{bmatrix} x \\ y \end{bmatrix}$$

### 5.2 Polygon SDF

For a room defined by a polygon with $N$ vertices
$\mathbf{v}_0, \mathbf{v}_1, \ldots, \mathbf{v}_{N-1}$, the SDF of a point
$\mathbf{p}$ is the distance to the nearest edge:

$$\mathrm{SDF}(\mathbf{p}) = \min_{i=0}^{N-1} d\!\left(\mathbf{p},\, \overline{\mathbf{v}_i \mathbf{v}_{i+1 \bmod N}}\right)$$

The distance from $\mathbf{p}$ to segment $[\mathbf{a}, \mathbf{b}]$ is:

$$d(\mathbf{p}, [\mathbf{a},\mathbf{b}]) = \left\|\mathbf{p} - \bigl(\mathbf{a} + t^*(\mathbf{b}-\mathbf{a})\bigr)\right\|_2$$

where the clamped parameter is:

$$t^* = \mathrm{clamp}\!\left(\frac{(\mathbf{p}-\mathbf{a})\cdot(\mathbf{b}-\mathbf{a})}{\|\mathbf{b}-\mathbf{a}\|^2},\; 0,\; 1\right)$$

Implemented with a small $\epsilon$ for numerical stability:

$$d = \sqrt{\min_i d_i^2 + \varepsilon}, \quad \varepsilon = 10^{-8}$$

### 5.3 Box SDF (rectangular rooms)

For an axis-aligned box with half-extents $(h_w, h_l, h_z)$, the signed distance is:

$$\mathrm{SDF}(\mathbf{p}) = \left\|\max(\mathbf{d}, \mathbf{0})\right\|_2 + \min\!\left(\max(d_x, d_y, d_z),\; 0\right)$$

where $\mathbf{d} = |\mathbf{p}| - (h_w, h_l, h_z)^{\top}$.

---

## 6. Variational Free Energy

The total loss minimised at each step is:

$$\mathcal{F} = \mathcal{L}_{\mathrm{lik}} + \mathcal{L}_{\mathrm{prior}}$$

### 6.1 Likelihood Term

A **Huber-robust** negative log-likelihood over $N$ LiDAR points:

$$\mathcal{L}_{\mathrm{lik}} = \frac{1}{2\sigma_{\mathrm{obs}}^2} \cdot
\frac{1}{N}\sum_{i=1}^{N} \ell_\delta\!\left(\mathrm{SDF}(\mathbf{p}_i^{\mathrm{room}}),\, 0\right)$$

where the Huber loss with threshold $\delta = 0.15\,\mathrm{m}$ is:

$$\ell_\delta(r) = \begin{cases}
\frac{1}{2}r^2 & |r| \le \delta \\
\delta\!\left(|r| - \frac{\delta}{2}\right) & |r| > \delta
\end{cases}$$

Parameters: $\sigma_{\mathrm{obs}} = 0.05\,\mathrm{m}$, $\delta = 0.15\,\mathrm{m}$.

The Huber loss makes the system robust to spurious LiDAR returns (furniture,
people) which would otherwise pull the robot pose towards incorrect positions.

### 6.2 Prior Term (Active Inference complexity cost)

The Mahalanobis distance between the current pose and the predicted pose:

$$\mathcal{L}_{\mathrm{prior}} = \frac{1}{2}\,(\mathbf{s} - \pmb{\mu}_{\mathrm{pred}})^{\top}
\pmb{\Pi}_{\mathrm{prior}}\,
(\mathbf{s} - \pmb{\mu}_{\mathrm{pred}})$$

where $\pmb{\Pi}_{\mathrm{prior}} = \pmb{\Sigma}_{\mathrm{pred}}^{-1}$ is the
precision matrix of the propagated covariance.

This term anchors the solution to the motion model: if the LiDAR is ambiguous
(e.g. symmetric room), the prior prevents the pose from drifting arbitrarily.

---

## 7. Dual-Prior Fusion (Command + Odometry)

Between two consecutive LiDAR scans at times $t_{k-1}$ and $t_k$, **two
independent velocity sources** are integrated to produce two motion priors:

| Source | Interface | Variables | Nature |
|--------|-----------|-----------|--------|
| **Commanded velocity** | `JoystickAdapter` | `adv_x`, `adv_y`, `rot` | Open-loop intent (before robot moves) |
| **Measured odometry** | `FullPoseEstimationPub` | `adv`, `side`, `rot` | Closed-loop feedback from encoders/IMU |

Both are in the **robot frame** (m/s, rad/s).

### 7.1 Velocity Integration

Each source is integrated over the time window $[t_{k-1}, t_k]$ using the
velocity command/reading buffer, segment-by-segment, with a running heading
$\theta(t)$:

$$\Delta\mathbf{s} = \sum_{j} \int_{t_{j}}^{t_{j+1}} \begin{bmatrix}
    v^r_x \cos\theta(t) - v^r_y \sin\theta(t) \\
    v^r_x \sin\theta(t) + v^r_y \cos\theta(t) \\
    -\omega
\end{bmatrix} dt$$

This yields two predicted poses:

$$\pmb{\mu}_{\mathrm{cmd}} = \pmb{\mu}_{k-1} + \Delta\mathbf{s}_{\mathrm{cmd}}, \qquad
\pmb{\mu}_{\mathrm{odom}} = \pmb{\mu}_{k-1} + \Delta\mathbf{s}_{\mathrm{odom}}$$

### 7.2 Per-Prior Covariance

Each prior has its own noise model with separate configurable parameters:

$$\sigma_{\mathrm{pos}} = \sigma_{\mathrm{base}} + k_{\mathrm{trans}} \cdot |\Delta\mathbf{t}|, \qquad
\sigma_{\theta} = \sigma_{\theta,\mathrm{base}} + k_{\mathrm{rot}} \cdot |\Delta\phi|$$

| Parameter | Command prior | Odometry prior |
|-----------|:---:|:---:|
| $k_{\mathrm{trans}}$ | `cmd_noise_trans` = 0.20 | `odom_noise_trans` = 0.08 |
| $k_{\mathrm{rot}}$ | `cmd_noise_rot` = 0.10 | `odom_noise_rot` = 0.04 |
| $\sigma_{\mathrm{base}}$ | `cmd_noise_base` = 0.05 m | `odom_noise_base` = 0.01 m |

The odometry prior is approximately **2.5× more precise** than the command
prior, reflecting that measured velocities are more trustworthy than
commanded velocities (which are subject to slip, motor lag, etc.).

### 7.3 Bayesian Gaussian Fusion

The two priors are fused into a single Gaussian before being passed to the
optimiser as the Active Inference prior:

$$\pmb{\Pi}_{\mathrm{fused}} = \pmb{\Sigma}_{\mathrm{cmd}}^{-1} + \pmb{\Sigma}_{\mathrm{odom}}^{-1}$$

$$\pmb{\mu}_{\mathrm{fused}} = \pmb{\Sigma}_{\mathrm{fused}} \left(
    \pmb{\Sigma}_{\mathrm{cmd}}^{-1}\,\pmb{\mu}_{\mathrm{cmd}} +
    \pmb{\Sigma}_{\mathrm{odom}}^{-1}\,\pmb{\mu}_{\mathrm{odom}}
\right)$$

$$\pmb{\Sigma}_{\mathrm{fused}} = \pmb{\Pi}_{\mathrm{fused}}^{-1}$$

The angle component is unwrapped relative to the command prior before fusion
to avoid discontinuities near $\pm\pi$.

When **no odometry readings** are available (e.g. disconnected sensor), the
system gracefully falls back to the command-only prior.

### 7.4 Odometry Noise Injection

For simulation and robustness testing, configurable Gaussian noise can be
added to the measured odometry readings:

$$v'_i = v_i + \mathcal{N}(0,\; |v_i| \cdot \alpha_{\mathrm{noise}})$$

where $\alpha_{\mathrm{noise}} =$ `ODOMETRY_NOISE_FACTOR` (default 0.1 = 10%
of reading magnitude). This allows evaluating system performance under
degraded odometry conditions.

---

## 8. EKF-Style Covariance Propagation

The uncertainty of the robot pose is tracked as a $3\times3$ covariance matrix
$\pmb{\Sigma}$.  During prediction, it is propagated by the linearised
motion model (**EKF predict** step):

$$\pmb{\Sigma}_{\mathrm{pred}} = \mathbf{F}\,\pmb{\Sigma}_{k-1}\,\mathbf{F}^{\top} + \mathbf{Q}$$

### 8.1 Motion Model Jacobian

For state $[x, y, \phi]$, the Jacobian $\mathbf{F}$ of the nonlinear motion model
$\mathbf{s}' = f(\mathbf{s}, \mathbf{u})$ w.r.t. $\mathbf{s}$ is:

$$\mathbf{F} = \mathbf{I}_3 + \begin{bmatrix}
0 & 0 & -\Delta y^l \sin\theta - \Delta x^l \cos\theta \\
0 & 0 &  \Delta y^l \cos\theta - \Delta x^l \sin\theta \\
0 & 0 & 0
\end{bmatrix}$$

where $(\Delta x^l, \Delta y^l)$ is the odometry displacement in the **robot frame**
(obtained by inverse-rotating the global delta).

### 8.2 Process Noise

The noise covariance $\mathbf{Q}$ is **anisotropic** and motion-dependent:

$$\pmb{\sigma}_{\mathrm{fwd}} = \sigma_{\mathrm{base}} + k_{\mathrm{trans}}\,|\Delta y^l|$$
$$\pmb{\sigma}_{\mathrm{lat}} = \sigma_{\mathrm{base}} + 0.3\,k_{\mathrm{trans}}\,|\Delta x^l|$$
$$\pmb{\sigma}_{\theta} = \sigma_{\theta,\mathrm{base}} + k_{\mathrm{rot}}\,|\Delta\phi|$$

with a **time-scaled** base term in prediction:

$$\sigma_{\mathrm{base}} = \texttt{cmd\_noise\_base}\cdot\sqrt{\min(1,\,4\,\Delta t)}$$

where $\Delta t$ is the lidar interval in seconds.

Transformed to the global frame:

$$\mathbf{Q} = \begin{bmatrix}
\sigma_l^2 \sin^2\!\theta + \sigma_f^2 \cos^2\!\theta &
(\sigma_f^2 - \sigma_l^2)\cos\theta\sin\theta & 0 \\
(\sigma_f^2 - \sigma_l^2)\cos\theta\sin\theta &
\sigma_l^2 \cos^2\!\theta + \sigma_f^2 \sin^2\!\theta & 0 \\
0 & 0 & \sigma_\theta^2
\end{bmatrix}$$

When the robot is **near-stationary** (low linear and angular speed), the base
noise is additionally scaled by a damping factor:

$$\sigma_{\mathrm{base}} \leftarrow \sigma_{\mathrm{base}}\cdot \texttt{stationary\_noise\_damping}$$

This keeps covariance growth slow at rest while still allowing gradual
uncertainty increase to trigger periodic correction.

Default parameters (command prior, used for EKF propagation):

| Symbol | Value | Param name |
|--------|-------|------------|
| $\sigma_{\mathrm{base}}$ | $0.05\,\mathrm{m}$ | `cmd_noise_base` |
| $k_{\mathrm{trans}}$ | $0.20$ | `cmd_noise_trans` |
| $k_{\mathrm{rot}}$ | $0.10$ | `cmd_noise_rot` |
| stationary damping | $0.70$ | `stationary_noise_damping` |

---

## 9. Variational Update — Gradient Descent on $\mathcal{F}$

Given the predicted pose as initialisation, the Adam optimiser minimises
$\mathcal{F}$ w.r.t. $(x, y, \phi)$:

$$\pmb{\mu}_{k} = \arg\min_{x,y,\phi}\;\mathcal{F}(x, y, \phi)$$

Two separate **Adam** parameter groups are used, one for position and one for
rotation, allowing different learning rates:

$$\eta_{\mathrm{pos}} = 0.05\,\mathrm{m}, \qquad \eta_{\mathrm{rot}} = 0.01\,\mathrm{rad}$$

**Early convergence** is detected when either:

- $\mathcal{F} < \tau_{\mathrm{thresh}} = 0.10$ (absolute threshold), or
- $|\mathcal{F}_{i-1} - \mathcal{F}_i| < 0.01\,\mathcal{F}_{i-1}$ (relative, after iteration 5).

Maximum iterations: $n_{\mathrm{max}} = 50$.

The gradients are scaled by **velocity-adaptive weights** $\mathbf{w}$ (see §12)
before the optimiser step.

---

## 10. Covariance Update (Bayesian Fusion)

After optimisation, the posterior covariance is updated using a
**Gauss-Newton Hessian approximation** of the likelihood:

$$\mathbf{H}_{\mathrm{lik}} \approx \mathrm{diag}\!\left(\frac{|\nabla_i \mathcal{L}_{\mathrm{lik}}| \cdot N_{\mathrm{pts}}}{\mathcal{L}_{\mathrm{lik}} + \varepsilon}\right), \quad \varepsilon = 10^{-6}$$

This is the **Fisher Information** approximation:
$\mathbf{H}_{ii} \approx \mathbb{E}\!\left[(\partial_i \ln p)^2\right]$.

The **posterior precision** fuses prior precision and likelihood curvature:

$$\pmb{\Pi}_{\mathrm{post}} = \pmb{\Pi}_{\mathrm{prior}} + \mathbf{H}_{\mathrm{lik}} + \lambda\,\mathbf{I}$$

with regularisation $\lambda = 10^{-4}$ for numerical stability.

The **posterior covariance** is:

$$\pmb{\Sigma}_{k} = \pmb{\Pi}_{\mathrm{post}}^{-1}$$

This mirrors the **Kalman gain** update:

$$\pmb{\Sigma}_{k} = (\pmb{\Sigma}_{\mathrm{pred}}^{-1} + \mathbf{H}_{\mathrm{lik}})^{-1}$$

A **condition number** check $\kappa(\pmb{\Pi}_{\mathrm{post}}) < 10^6$ guards
against ill-conditioned updates; if violated, the propagated covariance is kept.

---

## 11. Innovation and Health Monitoring

The **innovation** (Kalman residual) is the difference between the optimised
pose and the predicted pose:

$$\pmb{\nu} = \pmb{\mu}_{k} - \pmb{\mu}_{\mathrm{pred}}
= \begin{bmatrix} x_k - \hat{x} \\ y_k - \hat{y} \\ \phi_k - \hat{\phi} \end{bmatrix}$$

The angle difference is normalised to $[-\pi, \pi]$.

The **innovation norm** (position only):

$$\|\pmb{\nu}_{xy}\| = \sqrt{\nu_x^2 + \nu_y^2}$$

is displayed in the UI as a health indicator:

| Colour | Value | Meaning |
|--------|-------|---------|
| 🟢 Green | $< 5\,\mathrm{cm}$ | Prediction excellent, system healthy |
| 🟡 Yellow | $5$–$15\,\mathrm{cm}$ | Moderate prediction error |
| 🔴 Red | $> 15\,\mathrm{cm}$ | Large discrepancy — possible slip or kidnapping |

A large innovation with a small SDF loss indicates that the **LiDAR correction
overrode the odometry prediction** — a sign of odometry slip or environmental
change.

---

## 12. Velocity-Adaptive Gradient Weights

To improve responsiveness, the gradients of $\mathcal{F}$ w.r.t. $(x, y, \phi)$
are scaled by **motion-profile weights** $\mathbf{w} = (w_x, w_y, w_\phi)$
before the Adam update:

$$\tilde{g}_i \leftarrow w_i \cdot g_i, \qquad i \in \{x, y, \phi\}$$

The weights are computed from the velocity command history:

| Motion profile | $w_x$ | $w_y$ | $w_\phi$ |
|----------------|-------|-------|---------|
| Pure rotation ($\omega > \omega_{\mathrm{th}}$, $v < v_{\mathrm{th}}$) | $0.5$ | $0.5$ | $2.0$ |
| Pure forward ($v > v_{\mathrm{th}}$, $\omega < \omega_{\mathrm{th}}$, $v_y > v_x$) | $1.0$ | $2.0$ | $0.5$ |
| Pure lateral ($v_x > v_y$) | $2.0$ | $1.0$ | $0.5$ |
| Combined motion | $1.2$ | $1.2$ | $1.2$ |
| Stationary | $1.0$ | $1.0$ | $1.0$ |

Thresholds: $v_{\mathrm{th}} = 0.05\,\mathrm{m/s}$,
$\omega_{\mathrm{th}} = 0.05\,\mathrm{rad/s}$.

Transitions are smoothed by an **Exponential Moving Average (EMA)**:

$$\mathbf{w}_{k} = (1 - \alpha_w)\,\mathbf{w}_{k-1} + \alpha_w\,\mathbf{w}_{\mathrm{new}}, \qquad \alpha_w = 0.3$$

---

## 13. Pose Smoothing (EMA Filter)

The raw optimised pose $(\hat{x}_k, \hat{y}_k, \hat{\phi}_k)$ is filtered by an
**Exponential Moving Average** to reduce high-frequency jitter:

$$x_k^s = \alpha\,x_{k-1}^s + (1-\alpha)\,\hat{x}_k$$
$$y_k^s = \alpha\,y_{k-1}^s + (1-\alpha)\,\hat{y}_k$$

For the heading, the angle wrap-around is handled explicitly:

$$\delta\phi = \hat{\phi}_k - \phi_{k-1}^s, \quad \delta\phi \leftarrow \mathrm{wrap}(\delta\phi)$$
$$\phi_k^s = \phi_{k-1}^s + (1-\alpha)\,\delta\phi$$

Smoothing factor: $\alpha = 0.3$ (i.e. 70% weight on the new measurement).

The smoothed pose replaces the raw optimised values in the output and is stored
as the initialisation for the next prediction step.

---

## 14. Prediction-Based Early Exit

When the system is well-localised, the odometry prediction alone may be
accurate enough.  If the **predicted pose** has a small mean SDF error:

$$\bar{e}_{\mathrm{pred}} = \frac{1}{N}\sum_{i=1}^{N} |\mathrm{SDF}(\mathbf{p}_i^{\mathrm{room,pred}})| < \sigma_{\mathrm{sdf}} \cdot \gamma$$

the full Adam optimisation is **skipped entirely**, saving ~60 % CPU.

Conditions for early exit:

1. $k > k_{\mathrm{min}} = 20$ (system must be stable first)
2. $\mathrm{tr}(\pmb{\Sigma}_{xy}) < \epsilon_{\mathrm{cov}} = 0.1\,\mathrm{m}^2$ (low uncertainty)
3. $\bar{e}_{\mathrm{pred}} < \sigma_{\mathrm{sdf}} \cdot \gamma$ with $\sigma_{\mathrm{sdf}} = 0.15\,\mathrm{m}$, $\gamma = 0.5$ (threshold ≈ 7.5 cm)

When early exit fires, the pose is updated from the prediction alone and the
covariance from the EKF propagation (it grows slightly each frame without a
measurement update).

---

## 15. Initialisation and Kidnapping Recovery

### 15.1 Orientation Search

On the first update after initialisation, the system tests **16 candidate poses**
(4 angular offsets × 4 position mirrors) to resolve the symmetry ambiguity:

$$\Phi_{\mathrm{cand}} = \{\phi_0,\;\phi_0 + 90°,\;\phi_0 + 180°,\;\phi_0 + 270°\}$$
$$\mathbf{t}_{\mathrm{cand}} = \{(x,y),\;(-x,y),\;(x,-y),\;(-x,-y)\}$$

The best candidate minimises $\overline{\mathrm{SDF}^2}$ over a subsample of 100
LiDAR points.

### 15.2 Grid Search (Kidnapping)

If no valid initial pose is available, a **dense grid search** is performed over
the room interior with resolution $r_g = 0.5\,\mathrm{m}$ and angle resolution
$r_\phi = 45°$:

$$(\hat{x}, \hat{y}, \hat{\phi}) = \arg\min_{x \in \mathcal{G}_x,\, y \in \mathcal{G}_y,\, \phi \in \mathcal{G}_\phi}
\frac{1}{N}\sum_i \mathrm{SDF}(\mathbf{p}_i)^2$$

After the search, the best pose is set and the covariance is reset to
$\pmb{\Sigma}_0 = 0.1\,\mathbf{I}_3$.

### 15.3 Last Pose Recovery

At shutdown, the last estimated pose is written to `last_pose.json`. At startup,
if this file exists, the pose is loaded and used to warm-start the optimisation,
avoiding the grid search delay.

---

## 16. Polygon Path Planner

The `PolygonPathPlanner` class (`polygon_path_planner.h/.cpp`) provides
**grid-free shortest-path planning** inside the room layout polygon.

### 16.1 Algorithm: Visibility Graph + Dijkstra

The planner computes the **exact shortest path** between two points inside a
simple (possibly concave) polygon, without any grid discretisation.

**Pipeline:**

1. **Minkowski Inward Offset** — The room polygon is shrunk inward by the
   robot radius (`robot_radius = 0.25 m`). This guarantees that any path within
   the shrunken polygon is collision-free for the physical robot. The algorithm
   detects polygon winding (CW/CCW), computes inward normals per edge, offsets
   the edge lines, and intersects consecutive offset lines to find the new
   vertices.

2. **Visibility Graph Construction** — A graph is built where nodes are the
   vertices of the shrunken polygon. An edge connects two vertices if the
   straight segment between them lies entirely inside the polygon (i.e., they
   are *mutually visible*). Visibility is checked by: (a) no intersection with
   any polygon boundary edge, and (b) the segment midpoint is inside the
   polygon (handles concavities). Adjacent polygon edges are always connected.

3. **Path Query** — Given a start and goal position:
   - If there is direct line of sight → return `[start, goal]`.
   - Otherwise, start and goal are added as temporary nodes connected to all
     visible polygon vertices. Dijkstra's algorithm finds the shortest path.

**Complexity:** For a polygon with $n$ vertices (typically 6–20):
- Precomputation: $O(n^3)$ visibility checks — negligible for small $n$.
- Query: $O(n^2)$ to connect start/goal + $O(n \log n)$ Dijkstra.
- Memory: $O(n^2)$ for the graph — a few hundred entries.

### 16.2 Interior Obstacles

The Visibility Graph extends to **polygonal obstacles inside the room**
(furniture, columns):

- Obstacle polygons are **expanded outward** by `robot_radius` (Minkowski outward
  offset).
- Obstacle vertices are added to the visibility graph.
- The visibility check tests against **all** boundary segments (room + obstacles).

The `plan()` interface accepts obstacles via `set_obstacles()`.  At runtime,
static furniture from the room layout **and** temporary obstacles detected by the
blockage-detection system (§17.12) are both registered before each replan.

### 16.3 User Interaction

- **Shift + Right Click** on the viewer sets a navigation target.
- **Ctrl + Right Click** cancels the current navigation mission.
- The path is drawn as a **light-green polyline** with waypoint dots and a **red
  target marker**.

### 16.4 Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_radius` | 0.25 m | Minkowski inward offset for safe clearance |
| `waypoint_reached_dist` | 0.15 m | Distance threshold to consider a waypoint reached |
| `max_path_length` | 50.0 m | Reject paths longer than this |

---

## 17. MPPI Trajectory Controller

The `TrajectoryController` class (`trajectory_controller.h/.cpp`) provides
**reactive local navigation** that follows the path produced by the
`PolygonPathPlanner` while avoiding obstacles detected by the LiDAR.

It implements a proper **Model Predictive Path Integral (MPPI)** controller
(Williams et al., 2017) with four key extensions: **warm-start sequence**,
**AR(1) temporally correlated noise**, **structured exploration injections**,
and **ESS-based adaptive parameter tuning** (§17.10).

### 17.1 MPPI Overview

At each control cycle (20 Hz), the controller:

1. Builds a local **ESDF** (Euclidean Signed Distance Field) from LiDAR points
   in the robot frame.
2. Computes a **carrot** point on the path ahead of the robot.
3. **Warm-starts**: shifts the previous optimal control sequence by one step
   and blends it with a nominal control (proportional to the carrot).
4. **Samples** $K=100$ candidate trajectories as perturbations of the
   warm-started sequence plus structured injection seeds.
5. Optionally **refines** each sample with gradient-based ESDF optimization.
6. **Scores** each trajectory using an Expected Free Energy (EFE) cost.
7. Computes the **MPPI weighted average** over the **full T-step sequence**:

$$u^*_t = \frac{\sum_{k=1}^{K} w_k \cdot u^k_t}{\sum_{k=1}^{K} w_k}, \qquad
w_k = \exp\!\left(-\frac{G_k - G_{\min}}{\lambda}\right)$$

8. Stores the full optimal sequence for the next cycle's warm-start.
9. **Adapts** K, T, λ, σ, and injection count based on the ESS (§17.10).
10. Applies **EMA smoothing** and a **Gaussian brake** (advance modulated by
    rotation) to the first-step command before sending it to the robot.

### 17.2 Warm-Start Mechanism

The controller maintains `prev_optimal_[T]`, the full T-step optimal control
sequence from the previous cycle. Each cycle:

1. **Shift** left: `prev_optimal_[t] ← prev_optimal_[t+1]` for all $t$.
2. **Blend** with nominal control toward the carrot:

$$u^{\text{base}}_t = w \cdot u^{\text{prev}}_t + (1-w) \cdot u^{\text{nominal}}_t$$

with configurable weights ($w_{\text{adv}} = w_{\text{rot}} = 0.5$).

All K random samples are perturbations of this blended sequence, ensuring
temporal coherence — the plan evolves smoothly rather than being reconstructed
from scratch each cycle.

### 17.3 AR(1) Temporally Correlated Noise

Each random sample $k$ perturbs the warm-started base with an AR(1) process:

$$\varepsilon_t = \alpha\,\varepsilon_{t-1} + \sqrt{1-\alpha^2}\;\sigma\;\mathcal{N}(0,1)$$

with $\alpha = 0.75$ (default). This produces trajectories that are **smooth
curves** rather than zigzags — the noise changes slowly along the horizon,
generating natural arcs that a differential-drive robot can physically execute.

The noise sigmas ($\sigma_{\text{adv}}$, $\sigma_{\text{rot}}$) adapt online
based on the **ESS ratio** (see §17.10):

| ESS ratio | $\sigma_{\text{adv}}$ | $\sigma_{\text{rot}}$ |
|-----------|:---:|:---:|
| < 0.15 (collapsed) | ↓ 0.90× | ↑ 1.08× |
| 0.15 – 0.30 (low) | ↓ 0.95× | ↑ 1.04× |
| > 0.50 (healthy) | ↑ 1.01× | ↓ 0.97× |
| Clamped to | [0.04, 0.25] m/s | [0.08, 0.40] rad/s |

This ensures **wider lateral exploration near obstacles** (high $\sigma_{\text{rot}}$)
and **tight, nodding-free control in open space** (low $\sigma_{\text{rot}}$).

### 17.4 Structured Exploration Injections

Pure Gaussian perturbations around a forward-pointing nominal tend to all point
in roughly the same direction. When there is a wall ahead, they all collide and
the weighted average collapses.

To guarantee lateral coverage, the controller **injects deterministic seeds**
at fixed angular offsets from the carrot direction. Each injection follows a
`sqrt(phase)` blending curve from the offset angle back toward the carrot:

$$\theta^{\text{desired}}_t = \theta_{\text{carrot}} + \Delta\theta \cdot \bigl(1 - \sqrt{t/T}\bigr)$$

The number of injections scales with the **ESS ratio** (see §17.10), which
reflects how well the current sample population covers the solution space:

| ESS ratio | Injections | Offsets |
|:---:|:---:|---|
| > 0.40 (healthy) | 2 | ±30° |
| 0.25 – 0.40 | 3 | ±30°, ±45° |
| 0.15 – 0.25 | 4 | ±30°, ±60° |
| < 0.15 (collapsed) | 6 | ±30°, ±60°, ±90° |

The remaining $K - n_{\text{inject}}$ samples are AR(1) random perturbations.
This hybrid approach combines the **temporal coherence** of MPPI with the
**guaranteed coverage** of geometric seeds.

### 17.5 ESDF (Euclidean Signed Distance Field)

A local 2-D ESDF grid is built each cycle from the LiDAR points in robot frame:

- **Resolution**: 5 cm/cell, covering ±4 m around the robot (160×160 grid).
- **Construction**: Two-pass (forward + backward) distance transform on an
  occupancy grid. Each occupied cell gets distance 0; free cells get the
  minimum distance to any occupied cell, scaled by the grid resolution.
- **Querying**: Bilinear interpolation for smooth distance values.
  Gradient via central differences, normalised to unit length.

The ESDF provides:
- **Collision detection**: trajectory is cut when ESDF < `robot_radius`.
- **Obstacle cost**: quadratic hinge penalty when ESDF < `d_safe`.
- **Gradient refinement**: `optimize_seed()` uses the ESDF gradient to push
  trajectory points away from obstacles while pulling toward the carrot.

### 17.6 Trajectory Scoring (EFE)

Each trajectory sample is scored by an **Expected Free Energy** composed of:

$$G = G_{\text{goal}} + G_{\text{obs}} + G_{\text{smooth}} + G_{\text{vel}} + G_{\text{delta}} + G_{\text{progress}} + G_{\text{collision}}$$

| Component | Formula | Weight |
|-----------|---------|--------|
| **Goal** | Endpoint distance to carrot + heading alignment + backward penalty | `lambda_goal` = 5.0 |
| **Obstacle** | Discounted sum of quadratic + exponential ESDF penalties | `lambda_obstacle` = 8.0 |
| **Smooth** | $(u_0 - u^{\text{base}}_0)^2$ — continuity with warm-start | `lambda_smooth` = 0.05 |
| **Velocity** | $\sum (v^2 + 4\omega^2)$ — penalise unnecessary rotation 4× | `lambda_velocity` = 0.01 |
| **Delta** | $\sum (\Delta v^2 + 4\Delta\omega^2)$ — penalise jerky commands | `lambda_delta_vel` = 0.18 |
| **Progress** | Penalise per-step motion away from carrot (discounted) | `lambda_goal` |
| **Collision** | +400 if trajectory enters ESDF < `robot_radius` | — |

### 17.7 Gaussian Brake

Before sending the command to the robot, the advance velocity is modulated by
the rotation magnitude:

$$v_{\text{out}} = v_{\text{smooth}} \cdot \exp\!\left(-k \cdot \left(\frac{\omega}{\omega_{\max}}\right)^2\right), \qquad k = 0.6$$

With $k = 0.6$ the braking effect is mild — the robot slows to about 85 % of
full speed at half the rotational limit.  Stronger oscillation control is
handled by `lambda_delta_vel = 0.18` in the EFE scoring, so the brake acts as a
lightweight final safety net rather than the primary turn-speed limiter.

### 17.8 Integration with Localisation

When the trajectory controller is active, the velocity commands it sends to the
robot are also fed into the **velocity command history** used by the Active
Inference localisation. This closes the loop: the controller's commands serve as
a high-quality prediction prior for the next localisation update.

### 17.9 Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_samples` | 100 | K: initial trajectory samples (adapted by ESS, §17.10) |
| `trajectory_steps` | 50 | T: initial horizon steps (adapted by ESS, §17.10) |
| `trajectory_dt` | 0.10 s | Time step per horizon step |
| `mppi_lambda` | 8.0 | Initial temperature (adapted by ESS, §17.10) |
| `sigma_adv` | 0.12 m/s | Initial advance noise std (adapted by ESS) |
| `sigma_rot` | 0.25 rad/s | Initial rotation noise std (adapted by ESS) |
| `noise_alpha` | 0.80 | AR(1) temporal correlation |
| `warm_start_adv_weight` | 0.5 | Blend weight for previous adv sequence |
| `warm_start_rot_weight` | 0.3 | Blend weight for previous rot sequence |
| `d_safe` | 0.35 m | Safety distance for obstacle penalties |
| `robot_radius` | 0.3 m | Hard collision threshold |
| `carrot_lookahead` | 2.0 m | Lookahead distance on path |
| `goal_threshold` | 0.25 m | Distance to consider goal reached |
| `optim_iterations` | 0 | ESDF gradient refinement passes per seed |
| `velocity_smoothing` | 0.60 | EMA alpha for output smoothing |
| `gauss_k` | 0.6 | Gaussian brake intensity |
| `num_trajectories_to_draw` | 10 | Trajectories shown in viewer |
| `mood` | 0.5 | High-level behavior knob in [0, 1]: 0 = calm, 1 = excited |
| `enable_mood` | true | Enables runtime remapping of MPPI parameters from mood |
| `mood_speed_gain` | 0.35 | Mood influence on speed/path aggressiveness (`max_adv`, `max_rot`, lookahead) |
| `mood_exploration_gain` | 0.40 | Mood influence on exploration/optimization effort (K/T, noise, optimizer) |
| `mood_reactivity_gain` | 0.35 | Mood influence on reactivity (warm-start inertia, smoothing, brake) |
| `mood_caution_gain` | 0.30 | Mood influence on caution (safety distance and obstacle conservatism) |

> See **§17.10** for the full list of ESS adaptation parameters (K/T/λ/σ bounds,
> `ess_smoothing`, `adapt_interval`, `cpu_budget_ms`).

### 17.10 ESS-Based Adaptive Parameter Tuning

The MPPI controller adaptively tunes **five groups of parameters** at runtime
using the **Effective Sample Size (ESS)** of the MPPI importance weights.
This mechanism replaces a fixed-parameter MPPI with a self-tuning controller
that allocates compute where it matters and conserves resources when the
situation is benign.

#### 17.10.1 Effective Sample Size (ESS)

After computing the MPPI weights $w_k = \exp\!\bigl(-(G_k - G_{\min})/\lambda\bigr)$,
the ESS measures how many samples effectively contribute to the weighted average:

$$\text{ESS} = \frac{\left(\sum_{k=1}^{K} w_k\right)^2}{\sum_{k=1}^{K} w_k^2}$$

The **ESS ratio** $\hat{r} = \text{ESS}/K$ ranges from $1/K$ (one sample
dominates — weight collapse) to $1.0$ (all samples contribute equally —
redundant exploration). A low ESS ratio indicates that the sampling distribution
is poorly matched to the cost landscape: the controller must react by widening
exploration or increasing the sample count.

The raw ESS is smoothed with an **EMA filter** ($\alpha = 0.25$) to avoid
noisy single-cycle spikes from triggering unnecessary adaptation:

$$\widetilde{\text{ESS}}_t = (1 - \alpha) \cdot \widetilde{\text{ESS}}_{t-1} + \alpha \cdot \text{ESS}_t$$

#### 17.10.2 What is Adapted

Five parameter groups are adapted, each at a different rate:

| Parameter | Adaptation rate | Purpose |
|-----------|:-:|---|
| **λ** (temperature) | Every cycle | Control weight selectivity |
| **σ_adv, σ_rot** (noise stds) | Every cycle | Widen/narrow sampling spread |
| **n_inject** (injection count) | Every cycle | Guarantee lateral coverage |
| **K** (sample count) | Every 2 cycles | Increase/decrease compute |
| **T** (horizon length) | Every 2 cycles | Extend/shorten planning horizon |

#### 17.10.3 λ Adaptation (Temperature) — Every Cycle

The MPPI temperature λ controls how "selective" the weighted average is.
Low λ concentrates weight on the best few trajectories; high λ spreads weight
more evenly. The ESS directly diagnoses the need:

| ESS ratio | Action | Multiplier |
|:---:|---|:---:|
| < 0.15 | **Urgent soften** — weights collapsed, one sample dominates | λ ×= 1.15 |
| 0.15 – 0.30 | **Moderate soften** — too few effective samples | λ ×= 1.08 |
| > 0.50 | **Sharpen** — many redundant samples, be more selective | λ ×= 0.96 |

Clamped to $[\lambda_{\min}, \lambda_{\max}] = [1.0, 500.0]$.

This creates a **negative feedback loop**: when weights collapse (low ESS),
λ increases, which softens weights, which raises ESS. The system converges to
an operating point where ESS is in the "healthy" range (0.30 – 0.50).

#### 17.10.4 σ Adaptation (Noise Standard Deviations) — Every Cycle

The noise sigmas control the spatial spread of the sampled trajectories.
When ESS is low, the current samples are not diverse enough — the controller
increases rotational exploration while reducing advance noise (which tends
to produce forward-biased samples that all collide together):

| ESS ratio | σ_rot | σ_adv |
|:---:|:---:|:---:|
| < 0.15 | ↑ ×1.08 | ↓ ×0.90 |
| 0.15 – 0.30 | ↑ ×1.04 | ↓ ×0.95 |
| > 0.50 | ↓ ×0.97 | ↑ ×1.01 |

Clamped to $\sigma_{\text{adv}} \in [0.04, 0.25]$ m/s,
$\sigma_{\text{rot}} \in [0.08, 0.40]$ rad/s.

#### 17.10.5 Injection Count — Every Cycle

Structured injection seeds (§17.4) provide **deterministic lateral coverage**.
The ESS ratio determines how many are needed:

| ESS ratio | n_inject | Angular offsets |
|:---:|:---:|---|
| < 0.15 | 6 | ±30°, ±60°, ±90° |
| 0.15 – 0.25 | 4 | ±30°, ±60° |
| 0.25 – 0.40 | 3 | ±30°, ±45° |
| > 0.40 | 2 | ±30° |

Injection count is capped at 6 (±90° max) to avoid wild seeds that cause
oscillation in narrow corridors.

#### 17.10.6 K and T Adaptation — Every 2 Cycles

Sample count (K) and horizon length (T) are adapted less frequently
(`adapt_interval = 2` cycles = 100 ms) because changing them is computationally
meaningful and should be stable:

**K (sample count):**

| ESS ratio | Action | Multiplier |
|:---:|---|:---:|
| < 0.15 | Need many more samples urgently | K ×= 1.35 |
| 0.15 – 0.30 | Need more samples | K ×= 1.20 |
| > 0.60 | Can safely reduce | K ×= 0.85 |

Clamped to $[K_{\min}, K_{\max}] = [20, 300]$.

**T (horizon length):**

T adaptation uses **both** ESS ratio and the **collision ratio** (fraction of
samples that collided during forward simulation). This captures a specific
failure mode: when the horizon is too short to "see around" an obstacle, many
samples collide early, but the ESS among survivors may still look acceptable.

| ESS ratio | Collision ratio | Action | Multiplier |
|:---:|:---:|---|:---:|
| < 0.25 | > 0.50 | Obstacle blocking — need to see further | T ×= 1.15 |
| > 0.60 | < 0.20 | Open space — save compute | T ×= 0.90 |

Clamped to $[T_{\min}, T_{\max}] = [15, 120]$.

#### 17.10.7 CPU Budget Cap

After computing desired K and T, the controller estimates the CPU cost of
the next cycle based on the previous MPPI wall-clock time:

$$\hat{t}_{\text{next}} = t_{\text{prev}} \times \frac{K_{\text{desired}} \cdot T_{\text{desired}}}{K_{\text{current}} \cdot T_{\text{current}}}$$

If $\hat{t}_{\text{next}} > t_{\text{budget}}$ (default 5 ms, i.e. 10% of a
50 ms cycle), both K and T are scaled down proportionally:

$$K' = K_{\text{desired}} \cdot \sqrt{\frac{t_{\text{budget}}}{\hat{t}_{\text{next}}}}, \qquad
T' = T_{\text{desired}} \cdot \sqrt{\frac{t_{\text{budget}}}{\hat{t}_{\text{next}}}}$$

The square root distributes the reduction equally between K (sample diversity)
and T (planning horizon), preferring neither.

#### 17.10.8 Adaptation Flow Diagram

```
                     ┌──────────────────────────────────┐
                     │ MPPI cycle produces weights w_k  │
                     └──────────────┬───────────────────┘
                                    │
                                    ▼
                         ┌──────────────────┐
                         │  ESS = Σw² / (Σw)²│
                         │  r = EMA(ESS / K)  │
                         └──────────┬─────────┘
                                    │
              ┌─────────┬───────────┼───────────┬──────────┐
              ▼         ▼           ▼           ▼          ▼
          ┌───────┐ ┌───────┐ ┌─────────┐ ┌────────┐ ┌────────┐
          │   λ   │ │ σ_adv │ │ n_inject│ │   K    │ │   T    │
          │ (fast)│ │ σ_rot │ │ (fast)  │ │ (slow) │ │ (slow) │
          │ every │ │(fast) │ │ every   │ │ every  │ │ every  │
          │ cycle │ │ every │ │ cycle   │ │ 2 cyc  │ │ 2 cyc  │
          └───┬───┘ │ cycle │ └────┬────┘ └───┬────┘ └───┬────┘
              │     └───┬───┘      │          │          │
              │         │          │          └─────┬────┘
              │         │          │                │
              │         │          │         ┌──────▼──────┐
              │         │          │         │ CPU budget  │
              │         │          │         │ cap (K×T)   │
              │         │          │         └──────┬──────┘
              └─────────┴──────────┴───────────────┘
                                    │
                                    ▼
                          Next MPPI cycle uses
                          adapted parameters
```

#### 17.10.9 Adaptation Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ess_smoothing` | 0.25 | EMA alpha for ESS (higher = faster response) |
| `adapt_interval` | 2 | K/T adaptation period (cycles) |
| `K_min` / `K_max` | 20 / 300 | Adaptive sample count bounds |
| `T_min` / `T_max` | 15 / 120 | Adaptive horizon bounds |
| `lambda_min` / `lambda_max` | 1.0 / 500.0 | Adaptive temperature bounds |
| `sigma_min_adv` / `sigma_max_adv` | 0.04 / 0.25 m/s | Adaptive σ_adv clamp |
| `sigma_min_rot` / `sigma_max_rot` | 0.08 / 0.40 rad/s | Adaptive σ_rot clamp |
| `cpu_budget_ms` | 5.0 ms | Max MPPI compute per cycle |

### 17.11 Mood Superparameter (Calm–Excited)

The trajectory controller exposes a single high-level superparameter:

$$m \in [0,1]$$

where:

- $m = 0$: calm behavior (more conservative, smoother, less exploratory)
- $m = 0.5$: neutral/baseline behavior
- $m = 1$: excited behavior (faster, more reactive, more exploratory)

The controller first copies user parameters, then computes a mood-shaped scalar
using smoothstep:

$$s(m) = 3m^2 - 2m^3, \qquad n = 2s - 1 \in [-1, 1]$$

Each parameter family is rescaled with its corresponding mood gain and then
clamped to safety/consistency bounds.

Main effects by family:

- Speed family (`mood_speed_gain`): scales max advance/rotation and lookahead.
- Exploration family (`mood_exploration_gain`): scales sample/horizon baseline,
  noise amplitudes, and optimizer effort.
- Reactivity family (`mood_reactivity_gain`): reduces output smoothing and
  warm-start inertia as mood increases; also relaxes Gaussian braking.
- Caution family (`mood_caution_gain`): reduces safety margin and obstacle weight
  as mood increases (still clamped by hard safety constraints).

This allows continuous behavioral tuning from the UI with one slider while
preserving low-level parameter safety constraints.

### 17.12 Multi-Step Command Extraction

Instead of sending only the first time-step of the MPPI optimal sequence, the
controller averages the first $N=3$ steps:

$$u^* = \frac{1}{N}\sum_{t=0}^{N-1} u^{\text{opt}}_t$$

This smooths transient spikes that occasionally appear at $t=0$ (e.g. when
the warm-start shifts a large correction into the first slot) and provides a
more representative snapshot of the intended short-term manoeuvre.

### 17.13 Top-K Decisive Blending

When the ESS ratio is healthy, the standard MPPI weighted average mixes many
similar trajectories. When ESS collapses, the average is dominated by a single
best trajectory but the weighted sum can still dilute it with near-zero weight
contributions. To improve decisiveness:

- The controller selects the **top-3** trajectories by MPPI weight.
- Applies a **softmax** reweighting over only those three candidates.
- Blends the result with the full MPPI average using a factor that increases as
  ESS drops (controlled by `ess_blend_best_start` and `ess_blend_best_max`).

This ensures committed action through tight passages without losing the
exploration diversity of full MPPI in open space.

### 17.14 Catmull-Rom Path Smoothing

After the `PolygonPathPlanner` produces a visibility-graph shortest path (a
polyline with sharp corners at polygon vertices), the controller applies a
**centripetal Catmull-Rom spline** to generate a smooth, curvature-continuous
path:

1. Ghost control points are added before the first and after the last waypoint.
2. Each segment between consecutive waypoints is subdivided into interpolated
   points using the Catmull-Rom formulation with $\alpha = 0.5$ (centripetal
   parameterisation, which avoids cusps and self-intersections).
3. The resulting densified path replaces the original polyline.

The spline runs once when a new path is set (`set_path()`).

### 17.15 Lateral Clearance Cost

In narrow corridors the MPPI trajectories may hug one wall because the forward
cost landscape is symmetric. A **lateral clearance** term breaks this symmetry:

- Two side probes are placed at $\pm 0.22\,\text{m}$ from each trajectory point.
- The ESDF is queried at both probes.
- If the minimum side clearance is less than `robot_radius + lateral_clearance_margin`
  ($0.3 + 0.25 = 0.55\,\text{m}$), a quadratic penalty scaled by
  `lambda_lateral_clearance = 3.8` is added.
- An additional `lateral_closing_gain = 1.5` penalty is applied when the
  clearance is *decreasing* along the trajectory (approaching a wall).

This encourages the robot to stay centred in passages rather than drifting
toward one side.

### 17.16 Path Blockage Detection and Obstacle Avoidance

The controller monitors the ESDF values along the upcoming path waypoints to
detect dynamic obstacles that were not present when the path was planned:

1. **Detection**: At each cycle, waypoints within `blockage_lookahead_m = 1.5 m`
   are checked. A waypoint is "blocked" if its ESDF value is below
   `blockage_esdf_threshold = 0.25 m`. If at least `blockage_min_waypoints = 4`
   consecutive waypoints are blocked, a blockage candidate is registered.

2. **Confirmation**: The blockage must persist for `blockage_confirm_cycles = 15`
   consecutive cycles (~0.75 s at 20 Hz) to avoid false positives from
   transient sensor noise.

3. **Replan**: Once confirmed, the controller reports `path_blocked = true` with
   the blockage centre and approximate radius. The main loop in
   `specificworker.cpp`:
   - Clusters the nearby LiDAR points into an obstacle region.
   - Fits an OBB (oriented bounding box) polygon around the cluster.
   - Registers it as a **temporary obstacle** in the `PolygonPathPlanner`.
   - Replans the path around the obstacle.
   - Temporary obstacles expire after a configurable TTL.

4. **Cooldown**: After triggering a replan, a `blockage_cooldown_cycles = 100`
   guard prevents re-triggering while the new path stabilises.

---

## 18. Complete Algorithm Loop

```
LOOP every 50 ms:

  1. READ lidar points {p_i} in robot frame, lidar timestamp t_k
     READ current velocity commands from circular buffer
     READ measured odometry readings from circular buffer

  2. ORIENTATION SEARCH (first call only)
     ├── Test 16 (position × angle) candidate poses
     └── Keep the one with lowest mean SDF²

  3. COMMAND PRIOR  (from commanded velocity)
     ├── Δs_cmd = integrate_velocity(cmd_history, t_{k-1}, t_k)
     ├── μ_cmd = μ_{k-1} + Δs_cmd
     └── Σ_cmd = motion_covariance(Δs_cmd, cmd_noise_*)

  4. MEASURED ODOMETRY PRIOR  (from encoder/IMU feedback)
     ├── Δs_odom = integrate_odometry(odom_history, t_{k-1}, t_k)
     ├── μ_odom = μ_{k-1} + Δs_odom
     └── Σ_odom = motion_covariance(Δs_odom, odom_noise_*)

  5. DUAL-PRIOR BAYESIAN FUSION                           [§7.3]
     ├── Π_fused = Σ_cmd⁻¹ + Σ_odom⁻¹
     ├── μ_fused = Σ_fused (Σ_cmd⁻¹ μ_cmd + Σ_odom⁻¹ μ_odom)
     └── (fallback: cmd-only prior if no odometry available)

  6. EKF PREDICT
     ├── F = motion Jacobian (linearised at θ_{k-1})     [§8.1]
     ├── Q = anisotropic process noise                   [§8.2]
     └── Σ_pred = F Σ_{k-1} F^T + Q

  7. SET PRIOR for optimisation
     ├── Π_prior = Π_fused  (or Σ_pred⁻¹ if no fusion)
     └── Model.set_prediction(μ_fused, Π_prior)

  8. EARLY EXIT CHECK
     ├── Evaluate mean |SDF| at μ_fused                  [§14]
     └── IF small enough: return smoothed prediction → SKIP to 12

  9. ADAM OPTIMISATION  (minimise F = L_lik + L_prior)
     ├── Subsample lidar to ≤ 500 points
     ├── For each iteration:
     │   ├── Compute SDF for all points (auto-diff)
     │   ├── Compute F (Huber likelihood + Mahalanobis prior)
     │   ├── Backward pass → gradients
     │   ├── Scale gradients by velocity-adaptive weights [§12]
     │   ├── Adam step  (η_pos=0.05, η_rot=0.01)
     │   └── Check convergence criteria
     └── → optimal pose (x*, y*, φ*)

 10. COVARIANCE UPDATE (Bayesian fusion)
     ├── Compute ∇L_lik at optimal pose                  [§10]
     ├── H_lik ≈ diagonal Fisher information
     ├── Π_post = Π_prior + H_lik + λI
     └── Σ_k = Π_post⁻¹

 11. POSE SMOOTHING (EMA)                                [§13]
     └── (x,y,φ)_smooth = EMA((x*,y*,φ*), prev_smooth, α=0.3)

 12. INNOVATION
     ├── ν = (x,y,φ)_smooth - μ_fused                   [§11]
     └── Display ‖ν_xy‖ in UI

 13. TRAJECTORY CONTROLLER (if path active)              [§17]
      ├── Build local ESDF from lidar points (+static obstacles)
      ├── Warm-start: shift prev_optimal_ + blend nominal
      ├── Sample K trajectories (injections + AR(1) perturbations)
      ├── Score (EFE incl. lateral clearance) and compute MPPI weighted average
      ├── Top-K decisive blending                              [§17.13]
      ├── Multi-step command extraction (avg first 3 steps)    [§17.12]
      ├── Compute ESS → adapt K, T, λ, σ, n_inject           [§17.10]
      ├── Store optimal sequence for next cycle
      ├── Apply EMA smoothing + Gaussian brake
      ├── Send (adv, rot) to robot
      └── Feed commands into velocity_history for localisation

 14. BLOCKAGE DETECTION (if path active)                 [§17.16]
      ├── Check ESDF along upcoming path waypoints
      ├── IF persistent blockage confirmed:
      │   ├── Cluster LiDAR points near blockage centre
      │   ├── Fit OBB polygon → register as temp obstacle
      │   └── Replan path around obstacle
      └── Temp obstacles expire after TTL

 15. STORE for next cycle
     └── μ_{k} = smooth pose,  Σ_{k} = posterior cov
```

---

## 19. Parameters Reference

### Optimisation

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_iterations` | 50 | Max Adam iterations per cycle |
| `learning_rate_pos` | 0.05 m | Adam LR for position |
| `learning_rate_rot` | 0.01 rad | Adam LR for rotation |
| `min_loss_threshold` | 0.10 | Absolute early convergence threshold |
| `max_lidar_points` | 500 | Max LiDAR points used per optimisation |

### Sensor Model

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sigma_obs` | 0.05 m | Observation noise std (likelihood width) |
| `huber_delta` | 0.15 m | Huber loss transition point |

### Prediction / Dual-Prior Covariance

**Command prior** (open-loop, from joystick/controller):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cmd_noise_trans` | 0.20 | Position noise per metre of motion |
| `cmd_noise_rot` | 0.10 | Rotation noise per radian |
| `cmd_noise_base` | 0.05 m | Base process noise (stationary) |
| `stationary_noise_damping` | 0.70 | Extra damping multiplier when near-stationary |

**Measured odometry prior** (closed-loop, from encoders/IMU):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `odom_noise_trans` | 0.08 | Position noise per metre of motion |
| `odom_noise_rot` | 0.04 | Rotation noise per radian |
| `odom_noise_base` | 0.01 m | Base process noise (stationary) |

**Odometry noise injection** (simulation only):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ODOMETRY_NOISE_FACTOR` | 0.10 | Gaussian noise std as fraction of reading |

### Prediction-Based Early Exit

| Parameter | Default | Description |
|-----------|---------|-------------|
| `prediction_early_exit` | true | Enable early exit |
| `sigma_sdf` | 0.15 m | SDF observation noise for threshold |
| `prediction_trust_factor` | 0.5 | Threshold = σ_sdf × factor |
| `min_tracking_steps` | 20 | Stabilisation steps before early exit |
| `max_uncertainty_for_early_exit` | 0.1 m² | Max covariance trace |

### Velocity-Adaptive Weights

| Parameter | Default | Description |
|-----------|---------|-------------|
| `velocity_adaptive_weights` | true | Enable adaptive weighting |
| `linear_velocity_threshold` | 0.05 m/s | Minimum speed for "translating" |
| `angular_velocity_threshold` | 0.05 rad/s | Minimum speed for "rotating" |
| `weight_boost_factor` | 2.0 | Multiplier for active DOF |
| `weight_reduction_factor` | 0.5 | Multiplier for inactive DOF |
| `weight_smoothing_alpha` | 0.3 | EMA alpha for weight transitions |

### Pose Smoothing

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pose_smoothing` | 0.3 | EMA alpha (0 = no smoothing, 1 = no update) |

### Path Planner

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_radius` | 0.25 m | Minkowski inward offset (safe clearance) |
| `waypoint_reached_dist` | 0.15 m | Distance to consider waypoint reached |
| `max_path_length` | 50.0 m | Reject paths longer than this |

### MPPI Trajectory Controller

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_samples` | 100 | K: initial trajectory samples per cycle (adapted by ESS) |
| `trajectory_steps` | 50 | T: initial horizon steps (adapted by ESS) |
| `trajectory_dt` | 0.10 s | Time step per horizon step |
| `mood` | 0.5 | High-level calm-excited superparameter in [0, 1] |
| `enable_mood` | true | Enables mood-based runtime remapping |
| `mood_speed_gain` | 0.35 | Mood influence on speed and lookahead |
| `mood_exploration_gain` | 0.40 | Mood influence on exploration and optimizer effort |
| `mood_reactivity_gain` | 0.35 | Mood influence on smoothing and response speed |
| `mood_caution_gain` | 0.30 | Mood influence on safety-vs-aggressiveness balance |
| `mppi_lambda` | 8.0 | Initial temperature (adapted by ESS, §17.10.3) |
| `sigma_adv` | 0.12 m/s | Initial advance noise std (adapted by ESS, §17.10.4) |
| `sigma_rot` | 0.25 rad/s | Initial rotation noise std (adapted by ESS, §17.10.4) |
| `noise_alpha` | 0.80 | AR(1) temporal correlation coefficient |
| `K_min` / `K_max` | 20 / 300 | Adaptive sample count bounds |
| `T_min` / `T_max` | 15 / 120 | Adaptive horizon bounds |
| `lambda_min` / `lambda_max` | 1.0 / 500.0 | Adaptive temperature bounds |
| `sigma_min_adv` / `sigma_max_adv` | 0.04 / 0.25 m/s | Adaptive σ_adv clamp range |
| `sigma_min_rot` / `sigma_max_rot` | 0.08 / 0.40 rad/s | Adaptive σ_rot clamp range |
| `ess_smoothing` | 0.25 | EMA alpha for ESS ratio (higher = faster response) |
| `adapt_interval` | 2 | K/T adaptation period (cycles) |
| `cpu_budget_ms` | 5.0 ms | Max MPPI compute time per cycle |
| `warm_start_adv_weight` | 0.5 | Blend weight: prev vs nominal (advance) |
| `warm_start_rot_weight` | 0.3 | Blend weight: prev vs nominal (rotation) |
| `max_adv` | 0.8 m/s | Maximum forward velocity |
| `max_rot` | 0.7 rad/s | Maximum angular velocity |
| `d_safe` | 0.35 m | Safety distance for obstacle cost |
| `robot_radius` | 0.3 m | Hard collision threshold in ESDF |
| `carrot_lookahead` | 2.0 m | Path lookahead distance |
| `goal_threshold` | 0.25 m | Distance to consider goal reached |
| `optim_iterations` | 0 | ESDF gradient refinement passes |
| `optim_lr` | 0.05 | Learning rate for seed optimization |
| `velocity_smoothing` | 0.60 | EMA alpha for output smoothing |
| `gauss_k` | 0.6 | Gaussian brake intensity (rotation→advance) |
| `grid_resolution` | 0.05 m | ESDF grid cell size |
| `grid_half_size` | 4.0 m | ESDF grid extent from robot |
| `num_trajectories_to_draw` | 10 | Trajectories shown in viewer |


---

## 20. Key Implementation Notes

### LibTorch Autograd
All SDF computations are implemented as differentiable PyTorch operations.
The backward pass through `sdf_polygon()` or `sdf_box()` automatically provides
$\nabla_{(x,y,\phi)}\mathcal{F}$, including gradients through the rotation
matrix construction.

### Thread Safety
LiDAR reading and pose estimation run in **separate threads**.  A
`DoubleBufferSync` class ensures the compute loop always sees a matched
(robot\_pose, lidar\_points) pair from the same timestamp, avoiding stale data.

### Coordinate Convention
- Room frame: origin at polygon centroid, Y forward, X right.
- Robot frame: origin at robot centre, Y forward (direction of advance),
  X right, Z up.
- LiDAR points arrive in **robot frame** in metres.
- The viewer Y axis is **flipped** ($y_{\mathrm{screen}} = -y_{\mathrm{room}}$) so
  that "up on screen" = "forward in room".

### PyTorch Thread Limiting
To avoid CPU overload, LibTorch is limited to 2 intra-op threads and 1
inter-op thread:
```cpp
torch::set_num_threads(2);
torch::set_num_interop_threads(1);
```

### Dual-Prior Graceful Degradation
The dual-prior fusion in §7 degrades gracefully:
- If **no odometry** readings arrive (e.g. `FullPoseEstimationPub` is
  disconnected), the system automatically falls back to the command-only prior,
  preserving the original single-prior behaviour.
- If **no velocity commands** arrive (robot not under joystick control), the
  command prior predicts zero motion; the measured odometry still provides a
  valid prior.
- When **both** agree the robot is stationary, the covariance stays extremely
  tight, enabling aggressive early exit and minimal CPU usage.

### Path Planner — Pure Geometry
The `PolygonPathPlanner` has **no dependency on PyTorch, Qt, or Boost**. It
uses only Eigen and the STL, making it lightweight and independently testable.
The visibility graph is precomputed once when the polygon changes and reused
for all path queries.

### Trajectory Controller — Pure C++ MPPI
The `TrajectoryController` has **no dependency on PyTorch**. It uses only Eigen,
the STL, and `<random>`. The ESDF is built and queried entirely in the robot
frame using a simple 2-D grid — no GPU or tensor operations. The MPPI weighted
average is computed over raw `std::vector<float>` sequences, making it
lightweight enough to run 50 samples × 30 steps at 20 Hz on a single CPU core.

### Hybrid MPPI Design
The trajectory controller combines two sampling strategies:
- **Structured injection seeds** (2–6 deterministic lateral offsets) guarantee
  spatial coverage for obstacle avoidance and doorway navigation.
- **AR(1) Gaussian perturbations** (random samples around the warm-started
  sequence) provide temporal coherence and smooth convergence.

Additional post-processing:
- **Top-K decisive blending** (§17.13) prevents over-averaging when ESS collapses.
- **Multi-step extraction** (§17.12) smooths the output command.
- **Catmull-Rom spline** (§17.14) provides curvature-continuous paths.
- **Lateral clearance** (§17.15) centres the robot in narrow passages.
- **Blockage detection** (§17.16) triggers replanning around dynamic obstacles.

All five key parameters (K, T, λ, σ, n_inject) are **self-tuned** at runtime
using the **Effective Sample Size** of the MPPI importance weights (§17.10).
This ESS-based feedback loop allocates additional compute (more samples, longer
horizons, wider exploration) precisely when the situation demands it (obstacle
encounters, narrow passages) and relaxes in open space to save CPU.

This hybrid avoids the failure modes of both pure geometric seeds (cancel-out
near obstacles) and pure random sampling (insufficient lateral coverage with
moderate K).

---

*This document was generated from the source code of `ainf_slamo` (June 2025, v2.1).*

