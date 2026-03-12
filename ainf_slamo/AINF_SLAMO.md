# AINF_SLAMO

Mathematical Implementation Report  
Updated: March 2026

---

## 1. Scope


This report documents the actual mathematics implemented in the current software for:

- Active Inference localizer (`RoomConceptAI` + `Model`)
- Polygon visibility-graph planner (`PolygonPathPlanner`)
- MPPI/PD trajectory controller (`TrajectoryController`)

The focus is implementation-faithful equations, not a generic textbook derivation.

---

## 2. Frames, State, and Conventions


### 2.1 Robot kinematic convention


The controller and rollout model use:

- $Y^+$ = forward
- $X^+$ = right

Per-step motion model:

$$
\begin{aligned}
x_{t+1} &= x_t + v_t \sin(\vartheta_t)\,\Delta t \\
y_{t+1} &= y_t + v_t \cos(\vartheta_t)\,\Delta t \\
\vartheta_{t+1} &= \vartheta_t + \omega_t\,\Delta t
\end{aligned}
$$

### 2.2 Localization state


The room model exposes state vector:

$$
s = [w,\,l,\,x,\,y,\,\phi]^\top
$$

where $w,l$ are room dimensions and $(x,y,\phi)$ is robot pose in room frame. In the current localization update, optimization is effectively over $(x,y,\phi)$ with room geometry fixed after initialization.

---

## 3. Localizer Mathematics (Active Inference)


### 3.1 LiDAR to room transformation


Given LiDAR points in robot frame $p_i^{robot}\in\mathbb{R}^3$, the XY projection is transformed to room frame by:

$$
p_{i,xy}^{room} = R(\phi)\,p_{i,xy}^{robot} + t,
\quad t=[x,y]^\top
$$

with $R(\phi)$ the standard 2D rotation matrix.

### 3.2 SDF observation model


For polygon room mode, distance is computed to every segment and the minimum is taken:

$$
d_i = \min_j \left\|p_i - \Pi_{[a_j,b_j]}(p_i)\right\|,
$$

where $\Pi_{[a,b]}(p)$ is the segment projection with clamped interpolation parameter.

For box mode, the implemented signed box-distance expression is used in 3D with wall height.

### 3.3 Variational objective used in code


The optimized scalar is:

$$
\mathcal{F}(s)
= \underbrace{\tfrac{1}{2\sigma_{obs}^2}\,\operatorname{Huber}_\delta\!\left(d_i,0\right)_{\text{mean over }i}}_{\text{likelihood term}}
+ \underbrace{\tfrac{1}{2}(s_{pose}-\mu)^\top\Pi\,(s_{pose}-\mu)}_{\text{prior term}}
$$

with implemented constants in the loss:

- $\sigma_{obs}=0.05$ m
- $\delta=0.15$ m
- $s_{pose}=[x,y,\phi]^\top$
- $(\mu,\Pi)$ from predicted motion prior if available.

The robust UI metric reported separately is median absolute SDF:

$$
\operatorname{median}_i\left|d_i\right|.
$$

### 3.4 Motion integration priors


Two priors are integrated between LiDAR timestamps:

- Command prior from commanded $(adv_x,adv_z,rot)$
- Measured odometry prior from feedback $(adv,side,rot)$

Both integrate piecewise-constant commands over overlap windows:

$$
\Delta p = \sum_k R(\theta_k)
\begin{bmatrix}
\Delta x_k^{local} \\
\Delta y_k^{local}
\end{bmatrix},
\quad
\Delta\theta = \sum_k \Delta\theta_k.
$$

### 3.5 Dual-prior Gaussian fusion


From command prior $(\mu_c,\Sigma_c)$ and odometry prior $(\mu_o,\Sigma_o)$:

$$
\Pi_c=\Sigma_c^{-1},\quad \Pi_o=\Sigma_o^{-1}
$$
$$
\Pi_f = \Pi_c + \Pi_o
$$
$$
\mu_f = \Pi_f^{-1}(\Pi_c\mu_c + \Pi_o\mu_o)
$$

The implementation unwraps angular difference before fusion and normalizes the fused angle back to $[-\pi,\pi]$.

### 3.6 EKF-style covariance prediction


Prediction uses:

$$
P^- = FPF^\top + Q
$$

with Jacobian entries (localized state case):

$$
F_{x,\theta} = -\Delta y_{loc}\sin\theta - \Delta x_{loc}\cos\theta,
\quad
F_{y,\theta} = \Delta y_{loc}\cos\theta - \Delta x_{loc}\sin\theta.
$$

Noise is anisotropic in robot local axes (forward/lateral) then rotated to global XY. Forward uncertainty is larger than lateral uncertainty.

### 3.7 Covariance update after optimization


Posterior precision is updated by:

$$
\Lambda^+ = (P^-)^{-1} + H_{like} + \lambda I,
\quad
P^+ = (\Lambda^+)^{-1}
$$

where the implemented diagonal Fisher-style approximation is:

$$
H_{ii} \approx \left|\frac{\partial \mathcal{L}_{like}}{\partial \theta_i}\right|\cdot
\frac{N}{\mathcal{L}_{like}+\epsilon} + 10^{-4}.
$$

### 3.8 Velocity-adaptive gradient weighting


During Adam optimization, gradients are scaled by motion profile:

- Rotation-dominant motion: upweight $\theta$, downweight $(x,y)$
- Translation-dominant motion: upweight dominant translation axis, downweight $\theta$

This is equivalent to multiplying gradient components by weights $w_x,w_y,w_\theta$ before `optimizer.step()`.

### 3.9 Prediction-based early exit


Optimization is skipped if all conditions hold:

1. tracking is stable for minimum steps,
2. predicted covariance trace is below threshold,
3. mean absolute predicted SDF is below:

$$
\mathrm{tau}_{exit}=\sigma_{sdf}\cdot\text{prediction\_trust\_factor}.
$$

### 3.10 Innovation and smoothing


After update, pose is exponentially smoothed:

$$
\hat{s}_k = \alpha\hat{s}_{k-1} + (1-\alpha)s_k
$$

with wrapped-angle handling for $\phi$. Innovation is:

$$
\nu_k = s_k - \mu_k^{-}
$$

with angular component normalized to $[-\pi,\pi]$.

---

## 4. Planner Mathematics (Polygon Visibility Graph)


### 4.1 Point-in-polygon test


Uses ray-casting parity. For point $p$, each edge $(v_i,v_j)$ toggles inside/outside when:

$$
((v_i^y>p_y) \neq (v_j^y>p_y))
\land
\left(p_x < \frac{(v_j^x-v_i^x)(p_y-v_i^y)}{(v_j^y-v_i^y)} + v_i^x\right).
$$

### 4.2 Segment intersection


Proper intersection uses 2D cross products:

$$
t = \frac{(b_1-a_1)\times d_2}{d_1\times d_2},
\quad
u = \frac{(b_1-a_1)\times d_1}{d_1\times d_2}
$$

with strict interior checks $t,u\in(\epsilon,1-\epsilon)$.

### 4.3 Inward and outward polygon offsets


For each vertex, candidates are sampled on a circle of radius $r$ over 36 angles.

- Inward offset: choose candidate inside polygon maximizing clearance to boundary.
- Outward offset (obstacles): choose candidate outside polygon maximizing clearance.

If no feasible candidate exists, shorter radii are tried.

Clearance score is:

$$
c(p)=\min_{e\in\partial\mathcal{P}}\operatorname{dist}(p,e).
$$

### 4.4 Visibility predicate


A segment $(a,b)$ is visible iff all constraints hold:

1. both endpoints inside room polygon,
2. endpoints and midpoint not inside expanded obstacles,
3. no intersection with original room boundary,
4. no intersection with inner boundary,
5. no intersection with obstacle edges,
6. no sampled interior point along segment inside expanded obstacles.

### 4.5 Graph and shortest path


Nodes: inner polygon points plus obstacle navigation nodes. Edge weights are Euclidean distances:

$$
w_{ij}=\|p_i-p_j\|_2.
$$

Start and goal are inserted as temporary nodes and connected by visibility. Path is solved with Dijkstra:

$$
\operatorname{dist}(v)=\min_{u\to v}(\operatorname{dist}(u)+w_{uv}).
$$

Returned path is rejected if total length exceeds `max_path_length`.

---

## 5. Controller Mathematics (MPPI + Safety)


### 5.1 ESDF construction


An occupancy grid in robot frame is built from LiDAR plus static obstacle samples transformed from room frame. A two-pass chamfer-like propagation approximates distance transform, then distances are scaled by grid resolution.

Query uses bilinear interpolation; gradient uses centered finite differences and normalization.

### 5.2 Adaptive safety distance near goal


The effective safety distance is interpolated between far and near values:

$$
\alpha = e^{-d_{goal}/\tau},
\quad
d_{safe}^{eff}=(1-\alpha)d_{far}+\alpha d_{near}.
$$

### 5.3 Obstacle step cost


With ESDF value $d$:

Soft term (inside safety zone):

$$
g_{soft} = \left(\frac{d_{safe}^{eff}-d}{d_{safe}^{eff}-r_{robot}}\right)^2_+
$$

Hard-close term (inside close margin):

$$
g_{hard} = k_{close}\left(\frac{(r_{robot}+m_{close})-d}{m_{close}}\right)^2_+
$$

Final capped obstacle cost:

$$
G_{obs}^{step}=\min\left(\lambda_{obs}(g_{soft}+g_{hard}),\;G_{obs}^{cap}\right).
$$

### 5.4 MPPI seed generation


Sampling set includes:

- nominal seed toward carrot,
- six injected exploratory seeds with heading offsets $(\pm30^\circ,\pm60^\circ,\pm90^\circ)$,
- Gaussian-perturbed seeds.

Perturbed controls:

$$
u_t^{(k)} = u_t^{nom} + \epsilon_t^{(k)},
\quad
\epsilon_{adv}\sim\mathcal{N}(0,\sigma_{adv}^2),
\;\epsilon_{rot}\sim\mathcal{N}(0,\sigma_{rot}^2).
$$

### 5.5 Rollout objective used in implementation


For each seed, total score is:

$$
\begin{aligned}
G &= G_{goal} + G_{obs} + G_{lat} + G_{smooth} + G_{vel\_mag} + G_{vel\_delta} + G_{info} \\
&\quad + \lambda_{goal}G_{progress} + \mathbb{1}_{collision}\,P_{collision}.
\end{aligned}
$$

where:

- $G_{goal}$ uses closest approach to carrot (not terminal step), plus heading penalty,
- $G_{obs}$ is discounted sum of per-step obstacle costs,
- $G_{lat}$ penalizes low side clearances and worsening side trend,
- $G_{smooth}=\lambda_{smooth}(\Delta v_0^2+\Delta\omega_0^2)$ against previous optimal,
- $G_{vel\_mag}=\lambda_v\sum(v_t^2 + c_\omega\omega_t^2)$,
- $G_{vel\_delta}=\lambda_{\Delta v}\sum(\Delta v_t^2 + c_\omega\Delta\omega_t^2)$.

Near-term collisions (within hard horizon) are infeasible; farther collisions add a softer far-collision penalty.

### 5.6 Information-theoretic MPPI correction


For stochastic seeds, implementation adds:

$$
G_{info}=\lambda\sum_t
\left(
u_{t,adv}^{nom}\frac{\epsilon_{t,adv}}{\sigma_{adv}^2}
+
u_{t,rot}^{nom}\frac{\epsilon_{t,rot}}{\sigma_{rot}^2}
\right).
$$

### 5.7 MPPI weighting, ESS, and control extraction


For non-colliding rollouts:

$$
w_k = \exp\left(-\frac{G_k-G_{min}}{\lambda_{used}}\right),
\quad
\hat{u}_t=\sum_k \bar{w}_k u_t^{(k)},
\quad
\bar{w}_k=\frac{w_k}{\sum_j w_j}.
$$

Effective sample size:

$$
ESS = \frac{(\sum_k w_k)^2}{\sum_k w_k^2}.
$$

Issued command averages first 3 optimized steps and then applies velocity EMA.

### 5.8 Dominance-based adaptation


A dominance signal is computed as:

$$
D = p_{free}\,R_\theta,
\quad
E=1-D,
$$

where $p_{free}$ is free-rollout ratio and $R_\theta$ is steering concentration. A frontal Safety-Guard gate scales exploration influence.

Adaptation laws:

$$
\lambda \leftarrow \text{clip}(\lambda_0(1+2E),\lambda_{min},\lambda_{max}),
$$
$$
\sigma_{rot}\uparrow \text{ with }E,
\quad
\sigma_{adv}\downarrow \text{ with }E,
\quad
T\uparrow \text{ with }E.
$$

### 5.9 Safety guard forward risk simulation


A short-horizon forward simulation evaluates risk by querying ESDF along predicted states. Trigger occurs with either:

- hard threshold violation, or
- enough consecutive soft-threshold violations.

If triggered, controller attempts in order:

1. backup+turn command,
2. backup-only,
3. scaled forward reduction,
4. stop+turn fallback.

### 5.10 PD fallback mode


When PD mode is selected:

$$
e_\theta = \operatorname{atan2}(x_{carrot},y_{carrot}),
\quad
\omega = K_p e_\theta + K_d\Delta e_\theta,
$$
$$
v = v_{max}\,\max(0,\cos e_\theta)^p\,\text{dist\_factor},
$$

followed by the same smoothing, Gaussian turning brake, and ESDF safety gate.

---

## 6. Navigation Completion for `goto_object`


After path completion in object-navigation mode, the software performs a dedicated final heading phase (decoupled from trajectory tracking). The yaw error in robot frame is:

$$
e_\theta = \operatorname{atan2}(x_{obj}^{robot}, y_{obj}^{robot}).
$$

Exit conditions combine angle tolerance, distance checks, timeout cycles, and anti-oscillation guards.

---

## 7. Implementation Notes


- Optimization backend: Adam with separate learning rates for position and rotation.
- Localizer can skip optimization when prediction is trustworthy (early exit).
- Planner geometry uses robust visibility checks against both room and expanded obstacles.
- Controller integrates ESDF-based constraints at three levels: rollout cost, gradient seed optimization, and final safety gate.

---

## 8. Build and Run


```bash
cmake --fresh -S . -B build_make -G "Unix Makefiles"
make -C build_make -j20
bin/ainf_slamo etc/config
```

---

## 9. Symbol Glossary


- $x,y,\theta$: robot pose in controller/localization update space.
- $v,\omega$: advance and rotational command.
- $\Delta t$: control/update time step.
- $s=[w,l,x,y,\phi]^\top$: room-localization state.
- $d_i$: SDF distance residual for LiDAR point $i$.
- $\sigma_{obs},\delta$: observation noise scale and Huber threshold.
- $\mu,\Pi$: prior mean and prior precision.
- $P^-,P^+$: predicted and posterior covariance.
- $F,Q$: motion Jacobian and process-noise covariance.
- $\nu$: innovation residual.
- $d_{safe}^{eff}$: effective safety distance used in obstacle terms.
- $G$: total trajectory score for a rollout.
- $\lambda$: MPPI temperature (and context-dependent regularization weights).
- $ESS$: effective sample size of rollout weights.
- $D,E$: dominance and exploration signals in adaptive MPPI.

---

## 10. Equation to Implementation Mapping


### 10.1 Localizer mapping

- Variational loss $\mathcal{F}$: implemented in `loss_sdf_mse(...)` as likelihood + `m.prior_loss()`.
- Prior quadratic term: implemented in `Model::prior_loss()` using `prediction_precision_matrix`.
- Prior fusion equations: implemented in `RoomConceptAI::fuse_priors(...)`.
- Covariance prediction $P^- = FPF^T + Q$: implemented in `predict_step(...)`.
- Covariance posterior update: implemented in `update(...)` under covariance update block.
- Early-exit threshold $\tau_{exit}$: implemented with `sigma_sdf` and `prediction_trust_factor`.

### 10.2 Planner mapping

- Point-in-polygon predicate: `point_in_polygon(...)`.
- Proper segment intersection: `segments_intersect_proper(...)`.
- Inward/outward offsets: `offset_polygon_inward(...)`, `offset_polygon_outward(...)`.
- Visibility feasibility: `is_visible(...)` and crossing helpers.
- Shortest path: `dijkstra(...)` and `plan(...)`.

### 10.3 Controller mapping

- Step obstacle cost $G_{obs}^{step}$: `obstacle_step_cost(...)`.
- Repulsion strength for seed optimization: `obstacle_repulsion_strength(...)`.
- Rollout total score $G$: `simulate_and_score(...)`.
- Information correction $G_{info}$: `simulate_and_score(...)` with `use_info_correction`.
- MPPI weighted control and ESS: `compute(...)` and `compute_ess(...)`.
- Dominance adaptation laws: `adapt_from_dominance(...)`.
- Forward safety risk simulation: safety gate block in `compute(...)` and `compute_pd(...)`.
