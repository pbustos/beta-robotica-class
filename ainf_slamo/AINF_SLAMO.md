# AINF_SLAMO

Technical Report  
Current version reviewed: 12 March 2026

---

## 1. Purpose of this document

This document describes the **current implemented version** of `ainf_slamo` as a technical report, with emphasis on:

- the mathematical models used by the component,
- the geometry and coordinate transforms used across modules,
- the optimization criteria currently implemented in code,
- the planning and control cost functions,
- the point-cloud, grounding, and EM procedures used to update object hypotheses.

The report is intentionally focused on the **actual software as implemented now**, not on a generic future design.

---

## 2. Component overview

`ainf_slamo` is a RoboComp component that combines five main subsystems:

1. **Room-based localization** from LiDAR using an SDF room model and an Active-Inference-style prior.
2. **Global path planning** inside a known polygonal room with polygonal obstacles.
3. **Local trajectory control** using ESDF-based MPPI, with a PD fallback.
4. **Temporary obstacle modelling** from online LiDAR clusters to support replanning.
5. **Object grounding and EM-based ownership estimation**, including mesh-to-point-cloud fitting.

The current implementation is organized mainly across:

- `src/room_model.*`
- `src/room_concept_ai.*`
- `src/polygon_path_planner.*`
- `src/trajectory_controller.*`
- `src/pointcloud_center_estimator.*`
- `src/object_ownership_em.*`
- `src/mesh_sdf_optimizer.*`
- `src/specificworker_*.*`

---

## 3. Coordinate frames and notation

The whole component relies on a consistent set of 2D and 3D frames.

### 3.1 Room / world frame

The room polygon, furniture polygons, path planner nodes, and final localization pose are all expressed in a **room frame**.

- Position: $p^w = (x, y)$
- Robot heading: $\phi$

### 3.2 Robot frame

The local controller uses a robot-centric frame with the convention:


This convention appears explicitly in the controller rollout model:

$$
x_{t+1} = x_t + v_t \sin \theta_t\, \Delta t
$$

$$
y_{t+1} = y_t + v_t \cos \theta_t\, \Delta t
$$

$$
	heta_{t+1} = \theta_t + \omega_t\, \Delta t
$$

where $v_t$ is the forward speed and $\omega_t$ is the yaw rate.

### 3.3 Camera frame

The camera overlay and grounding code use a camera frame with:

- $x$ = lateral,
- $y$ = forward depth,
- $z$ = up.

The current pinhole projection used for overlays is:

$$
u = c_x + f_x \frac{x}{y}, \qquad
v = c_y - f_y \frac{z}{y}
$$

with current hardcoded overlay values:

- $c_x = 320$
- $c_y = 320$
- $f_x = 0.9 \cdot 640$
- $f_y = 0.9 \cdot 640$

### 3.4 Rigid transform between robot and room frames

LiDAR points are measured in robot coordinates and mapped to room coordinates through:

$$
p^w = R(\phi)\, p^r + t
$$

with:

$$
R(\phi) =
\begin{bmatrix}
\cos\phi & -\sin\phi \\
\sin\phi & \cos\phi
\end{bmatrix}
$$

and $t = (x, y)$ the robot translation in room coordinates.

---

## 4. Runtime architecture of the current version

### 4.1 Main loops

- `compute()` runs at `Period.Compute = 50 ms`.
- A background thread reads LiDAR.
- A second background thread runs localization (`RoomConceptAI`).

### 4.2 Current data flow

1. Sensor data are synchronized in a buffer.
2. Localization consumes high LiDAR plus motion history.
3. The planner operates in room coordinates with the current room polygon and obstacle polygons.
4. The local controller consumes the planned path plus low-level LiDAR and produces motion commands.
5. Camera/depth grounding can trigger an EM refinement and a mesh fitting stage.

---

## 5. Mathematical modules

## 5.1 Initial room-center estimation from point clouds

The startup stage uses `PointcloudCenterEstimator` to infer a robust room center and an oriented bounding box (OBB) from point clouds.

### 5.1.1 Filtering

Points are filtered by range:

$$
r = \lVert p \rVert
$$

and the point is kept only if:

$$
	exttt{min\_range} \le r \le \texttt{max\_range}
$$

### 5.1.2 Sector boundary extraction

The 2D cloud is divided into angular sectors. In each sector, the farthest point is selected, which approximates the room boundary.

### 5.1.3 Statistical outlier removal

For each candidate point, the average distance to its $k=5$ nearest neighbours is computed. Points whose local mean distance deviates too much from the global distribution are removed.

### 5.1.4 Convex hull and OBB

The convex hull is computed, and then a rotating-calipers-style sweep over hull edges finds the oriented rectangle with minimum area.

For each candidate orientation $\alpha$, points are rotated and the axis-aligned box in that rotated frame is measured. The chosen OBB minimizes:

$$
A(\alpha) = w(\alpha)\, h(\alpha)
$$

The returned center initializes the room center estimate, and the OBB angle initializes the yaw hypothesis.

---

## 5.2 Room representation and room SDF

`RoomConceptAI` relies on `Model`, which supports two room geometries:

1. **Box mode**: an axis-aligned rectangular room with half extents.
2. **Polygon mode**: a 2D polygonal room loaded from layout data.

The current runtime mainly uses **polygon mode**.

### 5.2.1 State vector

The internal state is stored as:

$$
s = [w,\, l,\, x,\, y,\, \phi]^T
$$

where:

- $w, l$ are the room dimensions,
- $(x,y,\phi)$ is the robot pose in the room frame.

In polygon mode, the room shape is fixed and only pose is optimized.

### 5.2.2 Box SDF

For the box case, the code uses the standard signed distance to an axis-aligned 3D box. If $d = |p| - h$, with $h$ the half-size vector, then:

$$
\operatorname{SDF}_{box}(p) = \lVert \max(d, 0) \rVert_2 + \min(\max(d_x, d_y, d_z), 0)
$$

### 5.2.3 Polygon boundary distance

For polygon rooms, the implementation computes the **minimum distance to the polygon edges** by projecting each point onto every segment and taking the minimum. For point $p$ and segment $(a,b)$:

$$
t^* = \operatorname{clip}\left(\frac{(p-a)^T(b-a)}{\lVert b-a \rVert^2}, 0, 1\right)
$$

$$
q = a + t^*(b-a)
$$

$$
d(p,ab) = \lVert p-q \rVert
$$

and then:

$$
\operatorname{SDF}_{poly}(p) = \min_j d(p, e_j)
$$

Important implementation detail: in polygon mode this is currently an **unsigned boundary distance**, not a signed inside/outside SDF. The localization objective therefore aligns LiDAR points with room boundaries.

---

## 5.3 Active-Inference-style localization

Localization is implemented in `RoomConceptAI` and is the main estimation module of the component.

### 5.3.1 Observation model

Let $\{p_i^r\}_{i=1}^N$ be the LiDAR points in robot coordinates. Under a candidate robot pose $(x,y,\phi)$, points are mapped to the room frame as:

$$
p_i^w = R(\phi) p_i^r + t
$$

The room model returns a boundary-distance value $d_i = \operatorname{SDF}(p_i^w)$.

### 5.3.2 Likelihood term

The code uses a robust Huber penalty around zero boundary distance:

$$
\mathcal{L}_{obs} = \frac{1}{2\sigma_{obs}^2} \cdot
\operatorname{HuberMean}(d_i, 0)
$$

with current compile-time constants in the loss:

- $\sigma_{obs} = 0.05\,m$
- Huber threshold $\delta = 0.15\,m$

This means the optimizer seeks poses for which the transformed LiDAR points lie close to room boundaries, while limiting the influence of outliers.

### 5.3.3 Motion prior

The model also includes a Gaussian prior around the predicted state:

$$
\mathcal{L}_{prior} = \frac{1}{2}(s - \mu)^T \Lambda (s - \mu)
$$

where:

- $\mu$ is the predicted pose,
- $\Lambda = \Sigma^{-1}$ is the precision matrix.

The total optimization objective is:

$$
\mathcal{F}(s) = \mathcal{L}_{obs}(s) + \mathcal{L}_{prior}(s)
$$

This is exactly the Active-Inference-style free-energy decomposition implemented in the current code.

### 5.3.4 Prediction step and covariance propagation

Before the optimization stage, the code performs an EKF-like prediction:

$$
P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k
$$

where:

- $F_k$ is the motion Jacobian,
- $Q_k$ is the motion-dependent process covariance.

The Jacobian corresponds to the pose-only model derived from the local-to-global integrated motion.

### 5.3.5 Velocity and odometry integration

Motion is integrated over the interval between consecutive LiDAR timestamps. The code accumulates command history and odometry history with a running heading.

For each short time segment:

$$
\Delta p^w = R(\theta_{run})\, \Delta p^r
$$

and headings are updated incrementally:

$$
	heta_{run} \leftarrow \theta_{run} + \Delta\theta
$$

This is done both for:


### 5.3.6 Motion covariance model

The implemented covariance model grows with displacement magnitude:

$$
\sigma_{pos} = \sigma_0 + k_t \lVert \Delta p \rVert
$$

$$
\sigma_{rot} = 0.01 + k_r |\Delta\theta|
$$

and the covariance becomes:

$$
\Sigma = \operatorname{diag}(\sigma_{pos}^2, \sigma_{pos}^2, \sigma_{rot}^2)
$$

The code uses tighter parameters for measured odometry than for commanded motion.

Additionally, `predict_step()` builds an anisotropic process model in the robot frame:

- larger uncertainty along the forward axis,
- smaller uncertainty in the lateral axis,
- separate rotational uncertainty.

That local covariance is then rotated into world coordinates.

### 5.3.7 Dual-prior fusion

When both command and measured odometry are available, the code fuses both Gaussian priors as:

$$
\Sigma_f^{-1} = \Sigma_c^{-1} + \Sigma_o^{-1}
$$

$$
\mu_f = \Sigma_f \left( \Sigma_c^{-1} \mu_c + \Sigma_o^{-1} \mu_o \right)
$$

with explicit angle unwrapping before fusion.

This fused prior initializes the optimizer and also defines the prior precision used in the loss.

### 5.3.8 Orientation search and kidnapping recovery

Two mechanisms are implemented to avoid orientation and kidnapping failures.

#### A. Initial orientation search

At first use, the code evaluates four orientations:

$$
\phi,\; \phi + \frac{\pi}{2},\; \phi + \pi,\; \phi + \frac{3\pi}{2}
$$

and mirrored position variants:

$$
(x,y),\; (-x,y),\; (x,-y),\; (-x,-y)
$$

The pose with minimum mean squared boundary distance is selected.

#### B. Grid search initial pose

For kidnapping recovery, a coarse search is run over:

- room $x$ and $y$,
- a set of discrete headings.

The selected pose minimizes:

$$
\frac{1}{N}\sum_i d_i^2
$$

over the search grid.

### 5.3.9 Optimization stage

The minimization is done with Adam using separate learning rates for translation and rotation:

- `learning_rate_pos = 0.05`
- `learning_rate_rot = 0.01`

The current default maximum number of iterations is `25`.

### 5.3.10 Velocity-adaptive gradient weighting

The code scales gradients according to the current motion profile:

- during pure rotation, rotational gradients are boosted and translational gradients are reduced,
- during straight motion, translational gradients are boosted,
- during combined motion, all gradients are moderately boosted.

The weighting is smoothed with an exponential moving average.

This is not a separate probabilistic model, but a practical optimization heuristic.

### 5.3.11 Prediction-based early exit

If the predicted pose already explains the LiDAR well enough, optimization is skipped.

The condition is based on:

1. low trace of the planar covariance,
2. sufficient tracking maturity,
3. small mean absolute predicted SDF.

The threshold used in code is:

$$
\bar d_{pred} < \sigma_{sdf} \cdot \texttt{prediction\_trust\_factor}
$$

with current defaults:

- $\sigma_{sdf} = 0.15\,m$
- trust factor $= 0.5$

so the practical threshold is $0.075\,m$.

### 5.3.12 Posterior covariance update

After optimization, the code fuses prior precision with an approximate likelihood curvature:

$$
P_{post} = \left(P_{prior}^{-1} + H_{like}\right)^{-1}
$$

where $H_{like}$ is a diagonal approximation obtained from gradient magnitudes.

The implementation comments describe this as a Fisher-style or Gauss-Newton-like approximation.

### 5.3.13 Innovation and health indicators

The component stores the innovation:

$$
\nu = x_{opt} - x_{pred}
$$

including heading wrap correction. The current quick health indicator is the planar norm:

$$
\lVert \nu_{xy} \rVert = \sqrt{\nu_x^2 + \nu_y^2}
$$

It also computes the condition number of the posterior precision matrix.

### 5.3.14 Pose smoothing

The exported pose is filtered with an exponential moving average:

$$
\hat x_k = \alpha \hat x_{k-1} + (1-\alpha)x_k
$$

with careful angle wrapping for heading.

The current default smoothing factor is `0.7`.

### 5.3.15 Display metric note

The current variable `sdf_mse` shown in the UI is historically named, but the code actually reports the **median absolute SDF value** as a robust fit indicator.

---

## 5.4 Polygonal global path planner

The path planner is implemented in `PolygonPathPlanner`.

### 5.4.1 Geometric model

The planner works inside a polygonal room and around polygonal obstacles.

Current planner parameters:

- `robot_radius = 0.4 m`
- `max_edge_len = 1.0 m`
- `max_path_length = 70.0 m`

### 5.4.2 Point-in-polygon test

The planner uses the classic ray-casting parity test.

### 5.4.3 Boundary inflation / deflation

Two offset operations are implemented:

1. **inward offset** for the room boundary,
2. **outward offset** for obstacle polygons.

Both are implemented with a sampling-based approximation around each vertex, selecting the candidate that maximizes clearance while respecting inside/outside conditions.

This acts as a practical Minkowski-sum approximation with a disk of radius `robot_radius`.

### 5.4.4 Polygon subdivision

Long room edges are subdivided so that no segment exceeds `max_edge_len`. This increases the density of navigation nodes and improves visibility-graph quality around long walls.

### 5.4.5 Visibility graph

The planner builds a graph whose nodes are:

- inward-offset room nodes,
- extra navigation nodes around obstacle vertices.

An edge is added between nodes $i$ and $j$ if the segment is visible, meaning it:

- stays inside the room,
- does not cross the room boundary,
- does not cross the inner boundary,
- does not intersect expanded obstacles,
- does not have midpoint or sampled points inside an obstacle.

### 5.4.6 Shortest path

With the visibility graph built, path search uses Dijkstra with Euclidean edge costs:

$$
c(i,j) = \lVert p_i - p_j \rVert
$$

If direct line of sight from start to goal exists, the path is simply:

$$
[p_{start}, p_{goal}]
$$

---

## 5.5 Path relaxation and spline smoothing

Before tracking, the controller modifies the path in two stages.

### 5.5.1 Elastic-band relaxation

The path is densified and then iteratively relaxed using two forces for each internal waypoint $p_i$:

1. **Obstacle repulsion** away from the nearest sampled wall/obstacle point.
2. **Smoothing force** toward the midpoint of neighbours.

In simplified form:

$$
p_i' = p_i + \alpha_{obs} f_{obs} + \alpha_{smooth} f_{smooth}
$$

where:

$$
f_{smooth} = \frac{p_{i-1}+p_{i+1}}{2} - p_i
$$

and $f_{obs}$ points away from the nearest obstacle whenever the waypoint is inside an influence distance.

The candidate move is accepted only if minimum clearance stays above a safety threshold.

### 5.5.2 Catmull-Rom spline smoothing

The relaxed polyline is then converted to a centripetal Catmull-Rom spline (`alpha = 0.5`) with approximately uniform spacing.

This produces continuous curvature and better MPPI rollouts.

The resulting points are again pushed away if they violate a minimum clearance.

---

## 5.6 ESDF-based MPPI local control

The local controller is the mathematically richest online module in the component.

### 5.6.1 Control modes

Two modes exist:

1. **MPPI** (default)
2. **PD carrot follower** (fallback)

The current report focuses on MPPI, since it is the primary controller.

### 5.6.2 ESDF and safety distance

The controller builds an ESDF over the local LiDAR cloud. The queried distance is used as the local clearance signal.

The effective safety distance is relaxed near the final goal through an exponential interpolation:

$$
d_{safe}^{eff}(d_g) = (1-\alpha)d_{far} + \alpha d_{near}
$$

with:

$$
\alpha = e^{-d_g/\tau}
$$

where $d_g$ is the distance to goal and $\tau$ is `goal_clearance_relax_dist`.

### 5.6.3 Obstacle step cost

The per-step obstacle penalty is a two-stage quadratic function.

If clearance falls below $d_{safe}$, a soft penalty is applied. If it falls below the hard threshold `robot_radius + close_obstacle_margin`, an additional stronger penalty is added.

In normalized form:

$$
G_{obs}(d) = \lambda_{obs}\left(\text{soft}(d)^2 + g_{close}\,\text{hard}(d)^2\right)
$$

and then clipped by `obstacle_cost_cap`.

### 5.6.4 Rollout dynamics

Each sampled trajectory is simulated with the kinematic model already shown:

$$
x_{t+1} = x_t + v_t \sin\theta_t\,\Delta t
$$

$$
y_{t+1} = y_t + v_t \cos\theta_t\,\Delta t
$$

$$
	heta_{t+1} = \theta_t + \omega_t\,\Delta t
$$

### 5.6.5 Nominal control and warm start

The controller keeps the previous optimal sequence and blends it with a nominal carrot-seeking control sequence. This creates the sampling center for the next MPPI iteration.

This is a standard warm-start structure:

$$
u^{center}_t = (1-\beta)u^{nom}_t + \beta u^{prev}_{t+1}
$$

with separate gains for forward and rotational components.

### 5.6.6 Sample generation

The controller samples $K$ trajectories with horizon $T$ around the blended center, using Gaussian perturbations with adaptive variances.

Current baseline values:

- $K = 100$
- $T = 50$
- $\Delta t = 0.1 s$

### 5.6.7 Rollout cost

The total cost for a sampled rollout is the sum of several terms:

$$
G = G_{goal} + G_{obs} + G_{lat} + G_{smooth} + G_{vel} + G_{\Delta vel} + G_{info} + \lambda_{goal} G_{progress} + G_{collision}
$$

#### A. Goal term

The controller scores the **closest approach to the carrot**, not just the final horizon endpoint. This avoids overshoot artifacts.

#### B. Progress penalty

Before the closest approach point, moving away from the carrot increases cost.

#### C. Heading penalty

The initial rotational tendency is penalized using the average angular error over the first rollout steps.

#### D. Obstacle cost

The discounted per-step obstacle cost is accumulated over the rollout.

#### E. Lateral clearance shaping

The controller samples ESDF on both sides of the predicted body center. If side clearance is low or getting worse, a lateral penalty is added. This helps avoid hugging walls in narrow corridors.

#### F. Smoothness term

The first control step is compared with the previous optimal first step:

$$
G_{smooth} = \lambda_{smooth}\left((\Delta v)^2 + (\Delta \omega)^2\right)
$$

#### G. Velocity and variation penalties

The controller penalizes control magnitude and rapid changes across the horizon.

#### H. Information-theoretic MPPI correction

The code includes the Williams-style correction:

$$
G_{info} \propto \lambda \sum_t u_t^T \Sigma^{-1}\epsilon_t
$$

where $\epsilon_t$ is the sampling perturbation relative to the nominal control.

### 5.6.8 Collision semantics

Near-term collisions are treated as hard infeasibility, while far-horizon collisions are turned into soft penalties. This keeps future-danger trajectories informative without letting them dominate immediate control.

### 5.6.9 MPPI weighting

For each rollout cost $G_k$, the weight is:

$$
w_k \propto \exp\left(-\frac{G_k - G_{min}}{\lambda}\right)
$$

The new control sequence is the weighted average of rollouts:

$$
U_{new} = \sum_k \bar w_k U_k
$$

with a best-seed fallback if weights collapse.

### 5.6.10 Effective sample size

The controller computes:

$$
ESS = \frac{(\sum_k w_k)^2}{\sum_k w_k^2}
$$

This is exported to the UI for diagnostics.

### 5.6.11 Dominance and exploration adaptation

The current version no longer adapts from many separate heuristics. Instead, it uses a single dominance signal:

$$
D = p_{free} \cdot c_{steer}
$$

where:

- $p_{free}$ = fraction of collision-free rollouts,
- $c_{steer}$ = directional concentration of survivor steering.

The exploration signal is:

$$
E = 1 - D
$$

This drives:

- MPPI temperature $\lambda$,
- angular noise,
- forward noise,
- horizon length.

### 5.6.12 Output smoothing and Gaussian brake

The first few optimized control steps are averaged to produce the command. Then a velocity EMA is applied. Finally, forward speed is reduced when angular speed is large through:

$$
	ext{brake} = \exp\left(-k\, \rho^2\right)
$$

where $\rho = \omega / \omega_{max}$.

### 5.6.13 Safety guard and blockage handling

The controller computes a frontal proximity gate from LiDAR and can report:

- safety-guard activation,
- persistent path blockage,
- blockage center and radius.

These signals are then used by `SpecificWorker` to trigger reverse recovery and replanning.

---

## 5.7 Temporary obstacle extraction and replanning

When the controller reports a persistent blockage, the worker extracts a local LiDAR cluster around the blockage center.

### 5.7.1 Cluster extraction

Points within a search radius around the blockage center are collected in robot coordinates.

### 5.7.2 Height estimation

The obstacle height estimate is:

$$
h = z_{max} - z_{min}
$$

### 5.7.3 PCA oriented bounding box

The cluster centroid is:

$$
\mu = \frac{1}{N}\sum_i p_i
$$

The covariance is:

$$
C = \frac{1}{N}\sum_i (p_i - \mu)(p_i - \mu)^T
$$

Eigenvectors of $C$ define major and minor axes. The cluster is projected onto those axes and enclosed in an OBB with a small additional margin.

This polygon is transformed back to room coordinates and added as a temporary obstacle. The planner and trajectory controller are then rebuilt using the static furniture plus all currently active temporary obstacles.

---

## 5.8 Final object-facing alignment

For `goto_object`, once the translational goal is reached, the current code stops path tracking and switches to a decoupled heading-alignment stage.

If the object center in robot coordinates is $(x_o^r, y_o^r)$, the yaw error is computed as:

$$
e_\theta = \operatorname{atan2}(x_o^r, y_o^r)
$$

The process ends when one of several conditions holds:

- angle tolerance reached,
- distance criterion reached,
- timeout reached,
- anti-oscillation sign-flip guard triggered.

This stage was introduced to eliminate endless-spin behaviour near object arrival.

---

## 5.9 Camera-grounding mathematics

The grounding subsystem combines camera observations, room geometry, and object hypotheses.

### 5.9.1 Visibility score for focused overlay selection

To decide which world object should be emphasized in the camera viewer, the current version computes a visibility score combining:

- angular centering,
- projected mesh visibility,
- world polygon area,
- proximity.

The implemented weighted score is:

$$
S_{vis} = 0.58 S_{center} + 0.22 S_{visible} + 0.15 S_{size} + 0.05 S_{prox}
$$

where:

- $S_{center}$ decreases with absolute bearing,
- $S_{visible}$ grows with the number of successfully projected points,
- $S_{size}$ grows with world footprint area,
- $S_{prox}$ decreases with depth.

### 5.9.2 Object orientation from polygon PCA

For a selected furniture polygon, the code estimates its principal axis from the planar covariance matrix and sets the yaw as:

$$
\psi = \operatorname{atan2}(v_y, v_x)
$$

where $(v_x,v_y)$ is the dominant eigenvector.

### 5.9.3 Camera-to-world association score

When matching a camera object to a world object, the current code evaluates:

- distance to robot,
- lateral displacement,
- distance between projected centers,
- bearing agreement,
- relative size consistency,
- weak label similarity.

The strict-gate score currently used is:

$$
S = 0.15 S_{pose} + 0.10 S_{lat} + 0.45 S_{center} + 0.20 S_{bearing} + 0.08 S_{size} + 0.02 S_{label}
$$

There is also a relaxed version with slightly different weights when the strict bearing gate fails.

This association stage is heuristic by design, but it is explicit and deterministic in the current code.

---

## 5.10 EM-based object ownership estimation

`ObjectOwnershipEM` implements a lightweight EM procedure to associate 3D observations to semantic object classes.

### 5.10.1 Observation and latent assignment

Given observed 3D points $p_i$ and latent class assignments $z_i \in \{1,\dots,K\}$, the algorithm alternates between:

- an **E-step** computing soft assignments,
- an **M-step** updating class states.

### 5.10.2 Robust residual model

Residuals are scored with the Charbonnier penalty:

$$
\rho(r;\sigma) = \sqrt{1 + \left(\frac{r}{\sigma}\right)^2} - 1
$$

This is less sensitive to outliers than a pure quadratic penalty.

### 5.10.3 Class energy

For a non-background class, the current code builds an energy from:

- planar distance to class position,
- vertical mismatch with expected object height,
- inactivity penalty if the state is disabled,
- optional RGB-D residual term,
- negative log prior weight.

In simplified form:

$$
E_{ik} = \rho(r_{xy}) + 0.30\,\rho(r_z) + E_{rgbd} - \log \pi_k + E_{inactive}
$$

For background, the energy depends mostly on vertical residual.

### 5.10.4 Annealed E-step

Responsibilities are computed with a temperature-annealed softmax:

$$
q_{ik} = \frac{\exp\left(-(E_{ik}-E_{min})/T\right)}{\sum_j \exp\left(-(E_{ij}-E_{min})/T\right)}
$$

with temperature linearly annealed from:

- `anneal_t_start = 2.0`
- to `anneal_t_end = 0.6`

### 5.10.5 M-step

For each active non-background class, the class position is updated by weighted averaging:

$$
\mu_k = \frac{\sum_i q_{ik} p_i}{\sum_i q_{ik}}
$$

Then a weighted 2D covariance is computed:

$$
C_k = \frac{\sum_i q_{ik}(p_i - \mu_k)(p_i - \mu_k)^T}{\sum_i q_{ik}}
$$

The dominant eigenvector of $C_k$ defines the yaw estimate.

The code also gates updates by:

- minimum support,
- maximum planar jump,
- maximum yaw jump.

### 5.10.6 Convergence criterion

EM stops early if the objective change is smaller than:

$$
\varepsilon = 10^{-3}
$$

The current default maximum number of EM iterations is `8`.

---

## 5.11 Mesh-to-point-cloud SDF fitting

`MeshSDFOptimizer` is used to adjust a polygonal mesh footprint to observed points.

### 5.11.1 Optimization variables

The current implementation optimizes:

- 2D translation,
- planar yaw.

### 5.11.2 Soft polygon distance

For a point and a polygon, the code computes the distance to every edge and then uses a soft minimum:

$$
d_{soft}(p) = -\frac{1}{\beta} \log \sum_j e^{-\beta d_j(p)}
$$

where:

- $d_j(p)$ is the distance to edge $j$,
- $\beta$ is `edge_softmin_beta`.

This gives a differentiable approximation to the minimum edge distance.

### 5.11.3 Data term

For observed object points $p_i$, the data loss is:

$$
L_{data} = \frac{1}{N}\sum_i d_{soft}(p_i)^2
$$

### 5.11.4 Monte Carlo pose-risk term

If enabled, pose uncertainty is propagated by sampling perturbations from the robot pose covariance:

$$
\delta \sim \mathcal{N}(0, \Sigma_{robot})
$$

and evaluating the data loss under each sample. The optimizer then uses:

- the mean loss,
- plus an optional risk term proportional to the standard deviation of sampled losses.

### 5.11.5 Wall penalty

If a room polygon is available, fitted vertices outside the room are penalized. The current code computes an outside mask and applies a squared boundary-distance penalty only to outside vertices.

### 5.11.6 Final objective

The effective objective in the current implementation is:

$$
L = L_{data} + \beta_{risk} \sigma_{data} + \lambda_{wall} L_{wall}
$$

The code keeps placeholders for additional regularizers, but in the present version the active terms are mainly the data term, Monte Carlo risk term, and wall penalty.

### 5.11.7 Optimization method

Optimization is performed with Adam.

Current defaults:

- `max_iterations = 80`
- `learning_rate = 0.03`
- `edge_softmin_beta = 20`
- `wall_penalty_weight = 25`

After optimization, a final geometric post-process pushes any remaining out-of-room vertices back inside the room polygon.

---

## 6. Navigation completion and recovery logic

The current version includes several practical safety and recovery layers around the mathematical core.

### 6.1 Safety-guard-triggered reverse recovery

If the local controller reports a safety-guard activation together with blocked-like behaviour, the worker enters a short reverse manoeuvre.

### 6.2 Replanning after recovery

If the recovery clears the blockage, the system tries to build a temporary obstacle from LiDAR and replan around it.

### 6.3 Timeout and expiry of temporary obstacles

Temporary obstacles are removed after a fixed timeout, after which the planner is rebuilt using only the remaining active obstacles.

---

## 7. Main current parameters

This section lists the most relevant numerical defaults of the present version.

### 7.1 Localization

- iterations: `25`
- translation LR: `0.05`
- rotation LR: `0.01`
- early-exit SDF sigma: `0.15 m`
- pose smoothing: `0.7`
- max LiDAR points per optimization: `1000`

### 7.2 Planner

- robot radius: `0.4 m`
- max polygon edge before subdivision: `1.0 m`
- max accepted path length: `70.0 m`

### 7.3 Local controller

- baseline MPPI samples: `100`
- baseline horizon: `50`
- step time: `0.1 s`
- maximum forward speed: `0.8 m/s`
- maximum rotation: `0.7 rad/s`
- safety distance: `0.35 m`
- robot radius for local clearance: `0.3 m`

### 7.4 EM grounding

- EM max iterations: `8`
- annealing: `2.0 \rightarrow 0.6`
- robust sigma: `0.25`

### 7.5 Mesh fit

- max iterations: `80`
- learning rate: `0.03`
- softmin beta: `20`
- wall penalty: `25`

---

## 8. Data products and persistent artifacts

The current version persists or updates:

- `last_pose.json`
- room/layout SVG-derived geometry
- scene graph state (`scene_graph.usda` when present)
- mission episodes under `episodic_memory/`
- UI geometry and 3D camera view through `QSettings`

---

## 9. Limitations of the current implementation

This section records important technical limitations of the version documented here.

1. **Polygon room distance is unsigned** in the localization model, so the observation model aligns points to walls but does not explicitly encode signed interior/exterior occupancy.
2. The covariance update uses a **diagonal curvature approximation**, not a full Hessian.
3. Polygon offsetting for planning is a **sampling-based approximation**, not an exact computational-geometry offset.
4. Camera grounding uses **engineered heuristic scores**, not a learned probabilistic association model.
5. Mesh fitting currently optimizes rigid 2D pose only, not full non-rigid shape deformation.

These are not defects of the document; they are properties of the current code.

---

## 10. Executive summary

The current `ainf_slamo` version is mathematically centered on the following chain:

1. **LiDAR-to-room alignment** through a robust SDF objective with Gaussian motion priors.
2. **Bayesian prior fusion** between commanded motion and measured odometry.
3. **Polygonal shortest-path planning** with visibility graphs and Dijkstra.
4. **ESDF-based MPPI** with multi-term rollout scoring, adaptive exploration, and safety gating.
5. **PCA-based obstacle boxing** for temporary replanning.
6. **Robust EM** for point-to-object ownership estimation.
7. **Differentiable soft-SDF mesh fitting** to align object footprints with perceived geometry.

This is therefore not just a navigation component: it is a combined estimation, planning, control, and semantic-grounding system whose main mathematical ingredients are:

- rigid-body geometry,
- robust optimization,
- Gaussian covariance propagation,
- graph shortest paths,
- stochastic optimal control,
- PCA and OBB fitting,
- EM with annealed soft assignments,
- differentiable soft-min distance models.

That is the current technical content of the implemented software.
