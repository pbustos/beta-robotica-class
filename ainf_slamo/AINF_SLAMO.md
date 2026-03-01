# AINF_SLAMO — Technical Documentation

**Active Inference SLAM with LiDAR and a Known Room Model**
Version 1.1 — March 2026

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
17. [Complete Algorithm Loop](#17-complete-algorithm-loop)
18. [Parameters Reference](#18-parameters-reference)
19. [Key Implementation Notes](#19-key-implementation-notes)

---

## 1. Overview

**AINF_SLAMO** estimates the 2-D pose of a robot inside a room whose geometry is
known *a priori* (the *nominal model*).  Sensor data comes from a 3-D LiDAR whose
returns are projected onto the ground plane.

The approach is grounded in **Active Inference** (Karl Friston, 2010): the system
minimises a *Variational Free Energy* functional that combines a sensor
**likelihood** (how well the LiDAR hits the room walls) with a motion-model
**prior** (where the odometry says the robot should be).  Differentiation is done
automatically by **LibTorch** (C++ PyTorch), allowing gradient-based optimisation
without symbolic calculus.

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
    (set target)           │  PolygonPathPlanner   │───► path drawn on viewer
                           │  (Visibility Graph)   │
                           └──────────────────────┘
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

Transformed to the global frame:

$$\mathbf{Q} = \begin{bmatrix}
\sigma_l^2 \sin^2\!\theta + \sigma_f^2 \cos^2\!\theta &
(\sigma_f^2 - \sigma_l^2)\cos\theta\sin\theta & 0 \\
(\sigma_f^2 - \sigma_l^2)\cos\theta\sin\theta &
\sigma_l^2 \cos^2\!\theta + \sigma_f^2 \sin^2\!\theta & 0 \\
0 & 0 & \sigma_\theta^2
\end{bmatrix}$$

When **stationary** ($|\Delta\mathbf{s}| < 0.01\,\mathrm{m}$), a very tight base
noise ($\sigma_{\mathrm{base}} = 0.001\,\mathrm{m}$) prevents spurious covariance
growth.

Default parameters (command prior, used for EKF propagation):

| Symbol | Value | Param name |
|--------|-------|------------|
| $\sigma_{\mathrm{base}}$ | $0.05\,\mathrm{m}$ | `cmd_noise_base` |
| $k_{\mathrm{trans}}$ | $0.20$ | `cmd_noise_trans` |
| $k_{\mathrm{rot}}$ | $0.10$ | `cmd_noise_rot` |

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

### 16.2 Future: Interior Obstacles

The Visibility Graph extends naturally to **polygonal obstacles inside the room**
(furniture, columns). The only change is:

- Obstacle polygons are **expanded outward** by `robot_radius` (Minkowski outward
  offset).
- Obstacle vertices are added to the visibility graph.
- The visibility check tests against **all** boundary segments (room + obstacles).

The `plan()` interface remains unchanged.

### 16.3 User Interaction

- **Shift + Right Click** on the viewer sets a navigation target.
- The path is planned from the current robot pose to the target.
- The result is drawn as a **cyan polyline** with waypoint dots and a **red
  target marker**.

### 16.4 Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_radius` | 0.25 m | Minkowski inward offset for safe clearance |
| `waypoint_reached_dist` | 0.15 m | Distance threshold to consider a waypoint reached |
| `max_path_length` | 50.0 m | Reject paths longer than this |

---

## 17. Complete Algorithm Loop

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

  5. DUAL-PRIOR BAYESIAN FUSION                           [7.3]
     ├── Π_fused = Σ_cmd⁻¹ + Σ_odom⁻¹
     ├── μ_fused = Σ_fused (Σ_cmd⁻¹ μ_cmd + Σ_odom⁻¹ μ_odom)
     └── (fallback: cmd-only prior if no odometry available)

  6. EKF PREDICT
     ├── F = motion Jacobian (linearised at θ_{k-1})     [8.1]
     ├── Q = anisotropic process noise                   [8.2]
     └── Σ_pred = F Σ_{k-1} F^T + Q

  7. SET PRIOR for optimisation
     ├── Π_prior = Π_fused  (or Σ_pred⁻¹ if no fusion)
     └── Model.set_prediction(μ_fused, Π_prior)

  8. EARLY EXIT CHECK
     ├── Evaluate mean |SDF| at μ_fused                  [14]
     └── IF small enough: return smoothed prediction → END

  9. ADAM OPTIMISATION  (minimise F = L_lik + L_prior)
     ├── Subsample lidar to ≤ 500 points
     ├── For each iteration:
     │   ├── Compute SDF for all points (auto-diff)
     │   ├── Compute F (Huber likelihood + Mahalanobis prior)
     │   ├── Backward pass → gradients
     │   ├── Scale gradients by velocity-adaptive weights [12]
     │   ├── Adam step  (η_pos=0.05, η_rot=0.01)
     │   └── Check convergence criteria
     └── → optimal pose (x*, y*, φ*)

 10. COVARIANCE UPDATE (Bayesian fusion)
     ├── Compute ∇L_lik at optimal pose                  [10]
     ├── H_lik ≈ diagonal Fisher information
     ├── Π_post = Π_prior + H_lik + λI
     └── Σ_k = Π_post⁻¹

 11. POSE SMOOTHING (EMA)                                [13]
     └── (x,y,φ)_smooth = EMA((x*,y*,φ*), prev_smooth, α=0.3)

 12. INNOVATION
     ├── ν = (x,y,φ)_smooth - μ_fused                   [11]
     └── Display ‖ν_xy‖ in UI

 13. STORE for next cycle
     └── μ_{k} = smooth pose,  Σ_{k} = posterior cov
```

---

## 18. Parameters Reference

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

---

## 19. Key Implementation Notes

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

---

*This document was generated from the source code of `ainf_slamo` (March 2026).*

