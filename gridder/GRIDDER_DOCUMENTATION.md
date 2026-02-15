# GRIDDER: Complete Technical Documentation

## Table of Contents

1. [Introduction](#1-introduction)
2. [Sparse ESDF Grid](#2-sparse-esdf-grid)
3. [Path Planning with ESDF](#3-path-planning-with-esdf)
4. [Particle Filter Localization](#4-particle-filter-localization)
5. [MPPI Controller](#5-mppi-controller)
   - [5.13 Time-Correlated Noise](#513-time-correlated-noise)
   - [5.14 Adaptive Covariance](#514-adaptive-covariance)
   - [5.15 ESDF-Based Obstacle Costs](#515-esdf-based-obstacle-costs)
   - [5.16 Covariance-Aware Margin Inflation](#516-covariance-aware-margin-inflation)
   - [5.17 Robot Footprint Sampling (8-Point)](#517-robot-footprint-sampling-8-point-collision-detection)
   - [5.18 Adaptive K (ESS-Based)](#518-adaptive-k-sample-count-based-on-ess-theory)
   - [5.19 Output Smoothing Filter](#519-output-smoothing-filter)
   - [5.20 Configuration Parameters](#520-mppi-configuration-parameters)
6. [Implementation Details](#6-implementation-details)
7. [Configuration Reference](#7-configuration-reference)

---

## 1. Introduction

The **Gridder** component is a RoboComp module that creates and maintains a 2D occupancy grid from LiDAR sensor data. It provides:

- **Sparse ESDF** (Euclidean Signed Distance Field) for memory-efficient map representation
- **A\* path planning** with ESDF-based cost function
- **Monte Carlo Localization** (AMCL) using particle filter
- **MPPI Controller** for smooth trajectory optimization with covariance-aware obstacle avoidance

### 1.1 Grid Implementation: Sparse ESDF

The component uses a **VoxBlox-inspired sparse representation**:

- **Memory efficient**: Only obstacle cells are stored (typically 1-5% of total area)
- **Fast updates**: Direct obstacle updates without ray casting
- **Built-in ESDF**: Distance field computed on-demand or precomputed for localization
- **TSDF-style weights**: Observation confidence weighting for robust updates

#### Key Features

| Feature | Description |
|---------|-------------|
| **Sparse Storage** | Only obstacle cells stored in hash map |
| **ESDF Distance Field** | Precomputed distance to nearest obstacle |
| **ESDF Gradient** | Precomputed gradient for smooth navigation costs |
| **Observation Weights** | Confidence-weighted obstacle updates |
| **A* Path Planning** | Cost-aware shortest path computation |

### 1.2 Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              GRIDDER                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────┐                                                        │
│  │   LiDAR Thread   │ ─────────────────────────────────────────┐             │
│  │  ─────────────── │                                          │             │
│  │  read_lidar()    │                                          │             │
│  │     ~50 Hz       │                                          │             │
│  └────────┬─────────┘                                          │             │
│           │                                                    │             │
│           │ LiDAR points (DoubleBuffer)                        │             │
│           ▼                                                    ▼             │
│  ┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐       │
│  │   Main Thread    │    │ Localizer Thread │    │   MPPI Thread    │       │
│  │  ─────────────── │    │  ─────────────── │    │  ─────────────── │       │
│  │  compute()       │    │  run_localizer() │    │  run_mppi()      │       │
│  │  • Visualization │◄───│     ~20 Hz       │───►│     ~20 Hz       │       │
│  │  • UI events     │    │                  │    │                  │       │
│  │  • Grid update   │    │  Reads: LiDAR,   │    │  Reads: LiDAR,   │       │
│  │    (mapping mode)│    │         Odometry │    │         Path,    │       │
│  └────────┬─────────┘    │         Grid/ESDF│    │         Grid/ESDF│       │
│           │              │                  │    │         Pose+Cov │       │
│           │              │  Outputs: Pose,  │    │                  │       │
│           │              │          Covar.  │    │  Outputs: vx,vy,ω│       │
│           │              └──────────────────┘    └────────┬─────────┘       │
│           │                                               │                  │
│           ▼                                               ▼                  │
│  ┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐       │
│  │   Grid ESDF      │    │  Path Planner    │    │   OmniRobot      │       │
│  │  (sparse map)    │◄───│     (A*)         │    │  (velocity cmd)  │       │
│  │                  │    │                  │    │                  │       │
│  │  • Obstacles     │    │  Uses Grid for   │    │  Receives MPPI   │       │
│  │  • ESDF distance │    │  cost-aware path │    │  commands        │       │
│  │  • Gradient      │    │  computation     │    │                  │       │
│  └──────────────────┘    └──────────────────┘    └──────────────────┘       │
│                                                                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                            ICE Interface                                     │
│      getPaths() | getMap() | getPose() | LineOfSightToTarget()              │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Data Flow Summary:**

```
                    ┌─────────┐
                    │  LiDAR  │
                    └────┬────┘
                         │
         ┌───────────────┼───────────────┐
         ▼               ▼               ▼
   ┌───────────┐   ┌───────────┐   ┌───────────┐
   │   Grid    │   │ Localizer │   │   MPPI    │
   │(map mode) │   │           │   │           │
   └─────┬─────┘   └─────┬─────┘   └─────┬─────┘
         │               │               │
         │               ▼               │
         │         ┌───────────┐         │
         └────────►│ Path (A*) │◄────────┘
                   └───────────┘
```

**Thread Communication:**

| Source | Destination | Mechanism | Data |
|--------|-------------|-----------|------|
| LiDAR Thread | Main/Localizer/MPPI | DoubleBuffer | Point cloud (world + local) |
| Main | Grid ESDF | Direct call | LiDAR points (mapping mode only) |
| Main | Localizer | DoubleBuffer | Odometry (from GT or encoder) |
| Localizer | Main/MPPI | DoubleBuffer | Pose + Covariance |
| Main | MPPI | Mutex + atomic | Current path, target |
| MPPI | Main | DoubleBuffer | Velocity commands (vx, vy, ω) |
| Grid ESDF | Localizer | Direct read | Distance field for observation model |
| Grid ESDF | MPPI | Direct read | Distance + gradient for obstacle cost |
| Grid ESDF | Path Planner | Direct read | Cost map for A* |

**Operating Modes:**

| Mode | Grid Update | Localizer | MPPI | Use Case |
|------|-------------|-----------|------|----------|
| **Mapping** | ✅ Active | ✅ Uses ESDF | ✅ Uses ESDF + LiDAR | Building map |
| **Localization** | ❌ Static | ✅ Uses ESDF | ✅ Uses ESDF + LiDAR | Pre-loaded map |

---

## 2. Sparse ESDF Grid

### 2.1 Data Structure

The sparse grid uses a hash map to store only occupied cells:

```cpp
using Key = std::pair<int, int>;  // (x, y) in grid coordinates
struct ObstacleCell {
    float log_odds = 2.0f;      // Probabilistic confidence (Binary Bayes)
    float hits = 1.0f;          // Observation count (TSDF-style weight)
    uint64_t last_seen = 0;     // Timestamp for temporal filtering
    QGraphicsRectItem* tile;    // Visualization
};
std::unordered_map<Key, ObstacleCell> obstacles_;
```

### 2.2 Hybrid Update Model: Log-Odds + TSDF Weights

Our implementation uses a **hybrid approach** combining:
1. **Log-odds** for probabilistic occupancy confidence
2. **Hit counts** as TSDF-style observation weights

This provides the benefits of both approaches:
- Probabilistic reasoning about occupancy (log-odds)
- Robust noise filtering through observation counting (TSDF weights)

#### 2.2.1 Log-Odds Representation

**Definition:** The log-odds ratio transforms probability to an unbounded space:
$$
l(m) = \log\left(\frac{p(m)}{1 - p(m)}\right)
$$

**Inverse (probability recovery):**
$$
p(m) = 1 - \frac{1}{1 + e^{l(m)}}
$$

#### 2.2.2 Binary Bayes Filter Derivation

For a cell $m$ with observations $z_{1:t}$, we want to estimate:
$$
p(m \mid z_{1:t})
$$

**Step 1: Apply Bayes' Theorem recursively**

Starting from Bayes' theorem and the Markov assumption:
$$
p(m \mid z_{1:t}) = \frac{p(z_t \mid m) \cdot p(m \mid z_{1:t-1})}{p(z_t \mid z_{1:t-1})}
$$

**Step 2: Compute the odds ratio**

Taking the ratio $\frac{p(m \mid z_{1:t})}{p(\neg m \mid z_{1:t})}$ and applying logarithm:

$$
l(m \mid z_{1:t}) = l(m \mid z_{1:t-1}) + l(m \mid z_t) - l_0
$$

With uniform prior ($l_0 = 0$), the **recursive update** simplifies to:
$$
l_{\text{new}} = l_{\text{old}} + l_{\text{observation}}
$$

#### 2.2.3 Observation Model

For each LiDAR hit at cell $k$:

| Event | Log-odds Update | Typical Value |
|-------|-----------------|---------------|
| **Hit** (obstacle detected) | $l_{\text{hit}} = +1.0$ | Increases confidence |
| **Miss** (ray passes through) | $l_{\text{miss}} = -0.4$ | Decreases confidence |

**Clamping** to prevent numerical overflow:
$$
l(m) \in [l_{\min}, l_{\max}] = [-2.0, +4.0]
$$

#### 2.2.4 TSDF-Style Observation Weighting

In addition to log-odds, we track **hit counts** as observation weights:

$$
h_{\text{new}} = h_{\text{old}} + 1
$$

This serves multiple purposes:
1. **Noise filtering**: Require minimum hits before confirming obstacle
2. **Confidence metric**: More observations = higher confidence
3. **Temporal weighting**: Recently seen obstacles prioritized

#### 2.2.5 Obstacle Confirmation Criteria

A cell is confirmed as an obstacle when **both** conditions are met:

$$
\text{is\_obstacle}(k) = \begin{cases}
\text{true} & \text{if } h(k) \geq h_{\min} \land l(k) \geq l_{\text{threshold}} \\
\text{false} & \text{otherwise}
\end{cases}
$$

With typical values:
- $h_{\min} = 2$ (minimum hits to confirm)
- $l_{\text{threshold}} = 1.0$ (minimum log-odds)

Additionally, **neighbor support** is required to filter isolated noise:
```cpp
bool confirmed = (hits >= min_hits) && 
                 (log_odds >= threshold) && 
                 has_neighbor_with_hits();
```

### 2.3 Update Algorithm

```
Algorithm: Hybrid Obstacle Update
─────────────────────────────────────────────────────────────
Input: LiDAR point p, timestamp t
Output: Updated obstacle map

1. Convert point to grid key: k = point_to_key(p)

2. If cell k exists in obstacles_:
   a. Update hit count: hits[k] += 1
   b. Update log-odds: log_odds[k] = min(log_odds[k] + l_hit, l_max)
   c. Update timestamp: last_seen[k] = t
   d. If not yet visualized AND confirmed:
      - Check neighbor support
      - Create visual tile if supported

3. Else (new cell):
   a. Create cell with hits=1, log_odds=l_hit
   b. No visualization until confirmed
─────────────────────────────────────────────────────────────
```

### 2.4 Comparison: Log-Odds vs TSDF vs Hybrid

| Aspect | Pure Log-Odds | Pure TSDF | Our Hybrid |
|--------|---------------|-----------|------------|
| **Probabilistic** | ✓ Yes | ✗ No | ✓ Yes |
| **Noise filtering** | Weak | ✓ Strong | ✓ Strong |
| **Confidence metric** | Probability | Weight | Both |
| **Obstacle removal** | Via miss updates | Weight decay | Via miss + timeout |
| **Isolated point filter** | ✗ No | ✗ No | ✓ Neighbor check |

### 2.5 Log-Odds Parameter Reference

| Parameter | Value | Meaning |
|-----------|-------|---------|
| $l_{\text{hit}}$ | +1.0 | Log-odds increment for hit |
| $l_{\text{miss}}$ | -0.4 | Log-odds decrement for miss |
| $l_{\min}$ | -2.0 | Minimum log-odds (≈12% occupied) |
| $l_{\max}$ | +4.0 | Maximum log-odds (≈98% occupied) |
| $l_{\text{threshold}}$ | +1.0 | Confirmation threshold (≈73% occupied) |
| $h_{\min}$ | 2 | Minimum hits to confirm |

**Log-odds to Probability Correspondence:**

| Log-odds | Occupied Probability |
|----------|---------------------|
| -2.0 | ~12% (confident free) |
| -1.0 | ~27% |
| 0.0 | 50% (unknown) |
| +1.0 | ~73% (threshold) |
| +2.0 | ~88% |
| +4.0 | ~98% (confident occupied) |

### 2.6 ESDF Computation

The Euclidean Signed Distance Field provides the distance to the nearest obstacle for any point:

$$
d_E(p) = \min_{o \in \mathcal{O}} \|p - o\|
$$

Our implementation uses a **direct search with hypot lookup table** for efficiency:

#### 2.6.1 Precomputation Algorithm

```cpp
void precompute_distance_field() {
    // 1. Build bounding box around all obstacles with margin
    
    // 2. Precompute hypot lookup table to avoid sqrt calls
    for (dx, dy in [-R, R]):
        hypot_table[dx][dy] = sqrt(dx² + dy²) * tile_size;
    
    // 3. For each cell in bounding box:
    for (cell in bounding_box):
        if (is_obstacle(cell)):
            distance[cell] = 0;
        else:
            // Search all neighbors within radius R
            min_dist = max_distance;
            for (dx, dy in [-R, R]):
                if (is_obstacle(cell + (dx,dy))):
                    dist = hypot_table[dx][dy];
                    if (dist < min_dist):
                        min_dist = dist;
            distance[cell] = min_dist;
}
```

#### 2.6.2 Complexity

| Operation | Complexity | Notes |
|-----------|------------|-------|
| Precomputation | O(N × R²) | N = cells in bounding box, R = search radius |
| Query (precomputed) | O(1) | Hash table lookup |
| Query (on-demand) | O(R²) | Search within radius |

#### 2.6.3 Optimizations

- **Hypot lookup table**: Avoids expensive `sqrt()` calls
- **Early termination**: Stop searching when closer obstacle found
- **Sparse storage**: Only confirmed obstacles checked

### 2.7 ESDF Gradient

The gradient points **away from obstacles** and is computed as:

$$
\nabla d(p) = \frac{p - p_{closest}}{\|p - p_{closest}\|}
$$

Where $p_{closest}$ is the nearest obstacle to point $p$.

The gradient is precomputed alongside the distance field by tracking which obstacle is closest to each cell.

**Uses:**
- Smooth obstacle avoidance costs in MPPI
- Covariance-aware margin inflation (projects pose uncertainty onto distance)

---

## 3. Path Planning with ESDF

### 3.1 A* Algorithm with ESDF Cost

Path planning uses the A* algorithm with an ESDF-based cost function that balances shortest path with obstacle clearance.

#### 3.1.1 Cost Function

The total cost for a cell combines distance traveled and proximity to obstacles:

$$
f(n) = g(n) + h(n)
$$

Where:
- $g(n)$ = actual cost from start to node $n$
- $h(n)$ = heuristic estimate to goal (Euclidean distance)

**Cell traversal cost:**
$$
c(n) = c_{base} + c_{safety}(d_{ESDF}(n))
$$

Where the safety cost penalizes cells near obstacles:
$$
c_{safety}(d) = \begin{cases}
\infty & \text{if } d < r_{robot} \\
w_{safety} \cdot e^{-d/\tau} & \text{otherwise}
\end{cases}
$$

#### 3.1.2 Safety Factor Parameter

The `safety_factor` parameter (0 to 1) controls the trade-off:
- **0.0**: Shortest path (may graze walls)
- **1.0**: Safest path (prefers corridor centers)

$$
c_{total} = (1 - \alpha) \cdot c_{distance} + \alpha \cdot c_{safety}
$$

Where $\alpha$ is the safety factor.

### 3.2 Line of Sight Check

Before computing A* path, check if direct path is free:

```cpp
bool is_line_of_sight_free(start, end, robot_radius) {
    for each point p along line(start, end):
        if (esdf_distance(p) < robot_radius)
            return false;
    return true;
}
```

If line of sight is clear, return direct path (much faster than A*).

### 3.3 Path Smoothing

After A* computation, the path is smoothed:

1. **Douglas-Peucker** simplification to reduce waypoints
2. **Line-of-sight** shortcuts where possible
3. **Bezier/spline** interpolation for smooth curves

### 3.4 Memory-Efficient Sparse Representation

**Key insight**: In most environments, obstacles occupy only 1-5% of the total area.

| Representation | Memory (50m × 50m, 100mm cells) | Update Speed |
|----------------|----------------------------------|--------------|
| Dense Grid | 250,000 cells (~1 MB) | Slow (ray casting) |
| Sparse ESDF | ~5,000 cells (~20 KB) | Fast (direct update) |

**Convention:**
- Cell NOT in map → FREE (implicit)
- Cell IN map → OBSTACLE (explicit)

### 3.5 ESDF Precomputation for Path Planning

For efficient path planning, ESDF distances are precomputed using **Brushfire**:

```cpp
void precompute_esdf() {
    // Initialize: obstacles have distance 0
    queue<Key> frontier;
    for (auto& [key, cell] : obstacles_) {
        esdf_cache_[key] = {0.0f, key};  // distance, closest_obstacle
        frontier.push(key);
    }
    
    // BFS propagation
    while (!frontier.empty()) {
        Key k = frontier.front();
        frontier.pop();
        
        for (auto& neighbor : get_neighbors_8(k)) {
            float new_dist = esdf_cache_[k].distance + distance(k, neighbor);
            if (new_dist < esdf_cache_[neighbor].distance) {
                esdf_cache_[neighbor] = {new_dist, esdf_cache_[k].closest};
                frontier.push(neighbor);
            }
        }
    }
}
```

**Complexity:** O(N) where N = number of cells in bounding box

For localization (many queries), precompute all distances:

```cpp
void precompute_distance_field() {
    // Build bounding box with margin
    for (x, y) in bounding_box:
        Key k = {x, y};
        if (is_obstacle(k)):
            precomputed_distances_[k] = 0;
        else:
            // Find nearest obstacle using hypot lookup table
            min_dist = search_nearest_obstacle(k);
            precomputed_distances_[k] = min_dist;
}

float get_distance_precomputed(const Key &k) const {
    auto it = precomputed_distances_.find(k);
    return (it != end) ? it->second : max_dist;  // O(1)
}
```

**Complexity:** O(1) per query after O(area × R²) precomputation

### 3.6 ESDF-Based Movement Cost

For A* path planning, the cost combines distance and obstacle proximity:

$$
\text{move\_cost}(n, n') = d(n, n') + \text{esdf\_cost}(n') \times S_f \times S_s
$$

Where:
- $d(n, n')$ = Euclidean distance (tile_size or tile_size × √2 for diagonal)
- $\text{esdf\_cost}(n')$ = exponential cost from ESDF distance
- $S_f$ = safety_factor (0.0-1.0)
- $S_s$ = safety_scale = $1 + S_f^2 \times 10$

**ESDF Cost Function:**
$$
\text{cost}(x) = \begin{cases}
C_{\text{max}} & \text{if } d_E(x) = 0 \\
C_{\text{max}} \cdot e^{-d_E(x) / \tau} & \text{if } d_E(x) < 2\tau \\
0.25 \cdot C_{\text{max}} \cdot e^{-(d_E(x) - 2\tau) / 2\tau} & \text{otherwise}
\end{cases}
$$

### 3.7 Safety Factor Visualization

```
safety_factor = 0.0              safety_factor = 1.0

    ████████                         ████████
    █      █                         █      █
  ──█──────█──  (shortest)           █  ──  █  (safest, centered)
    █      █                         █ /  \ █
    ████████                         ████████
```

---

## 4. Particle Filter Localization

### 4.1 Overview

The localizer uses **Adaptive Monte Carlo Localization (AMCL)**, a particle filter approach that:

- Estimates robot pose from LiDAR observations
- Adapts particle count based on uncertainty (KLD-sampling)
- Provides pose covariance for downstream components (MPPI)

### 4.2 Particle Representation

Each particle represents a hypothesis of the robot's pose:

```cpp
struct Particle {
    Pose2D pose;           // (x, y, θ)
    double log_weight;     // Log-space for numerical stability
    
    double getWeight() const { return exp(log_weight); }
};
```

### 4.3 Motion Model (Prediction)

Particles are propagated using odometry with added noise:

$$
\begin{aligned}
x' &= x + (\delta_x + \epsilon_x) \cos\theta - (\delta_y + \epsilon_y) \sin\theta \\
y' &= y + (\delta_x + \epsilon_x) \sin\theta + (\delta_y + \epsilon_y) \cos\theta \\
\theta' &= \theta + \delta_\theta + \epsilon_\theta
\end{aligned}
$$

**Noise model parameters:**
- $\alpha_1$ = rotation noise from rotation
- $\alpha_2$ = rotation noise from translation
- $\alpha_3$ = translation noise from translation
- $\alpha_4$ = translation noise from rotation

### 4.4 Observation Model (Correction)

Particles are weighted by how well they explain the LiDAR observations:

$$
w_i = p(z_t \mid x_i, m)
$$

Using the **beam model** with ESDF:

$$
p(z_t \mid x_i, m) = \prod_{k=1}^{K} \left( z_{\text{hit}} \cdot p_{\text{hit}}(z_k) + z_{\text{rand}} \cdot p_{\text{rand}}(z_k) \right)
$$

Where:
- $p_{\text{hit}}$ uses ESDF distance: $\mathcal{N}(d_{ESDF}, \sigma_{hit}^2)$
- $p_{\text{rand}}$ is uniform distribution for unexplained measurements

### 4.5 Resampling

When **Effective Sample Size (ESS)** falls below threshold:

$$
ESS = \frac{1}{\sum_i w_i^2}
$$

Resample using **low-variance resampling** to maintain particle diversity.

### 4.6 KLD-Adaptive Sampling

Particle count adapts to distribution complexity:

$$
N = \frac{k-1}{2\epsilon} \left(1 - \frac{2}{9(k-1)} + \sqrt{\frac{2}{9(k-1)}} \cdot z_{1-\delta}\right)^3
$$

Where $k$ = number of occupied histogram bins.

### 4.7 Pose Covariance Output

The localizer computes pose covariance from the particle distribution:

$$
\Sigma_\xi = \begin{bmatrix} 
\sigma_{xx}^2 & \sigma_{xy} & \sigma_{x\theta} \\
\sigma_{xy} & \sigma_{yy}^2 & \sigma_{y\theta} \\
\sigma_{x\theta} & \sigma_{y\theta} & \sigma_{\theta\theta}^2 
\end{bmatrix}
$$

This covariance is passed to MPPI for **covariance-aware margin inflation**.

### 4.8 Convergence Detection

Localization is considered converged when:

$$
\sqrt{\text{trace}(\Sigma_{xy})} < \sigma_{\text{pos\_threshold}}
$$
$$
\sigma_\theta < \sigma_{\theta\_threshold}
$$

---

## 5. MPPI Controller

### 5.1 Overview

**MPPI (Model Predictive Path Integral)** is a sampling-based stochastic optimal control algorithm. Unlike traditional MPC (Model Predictive Control), MPPI does not require solving a nonlinear optimization problem. Instead, it approximates the optimal solution through a weighted average of sampled trajectories.

**Key advantages:**
- **Derivative-free**: No gradients of the cost function required
- **Handles non-smooth costs**: Can use discontinuous cost functions (e.g., collision penalties)
- **Parallelizable**: K trajectories can be evaluated in parallel (GPU-friendly)
- **Robust**: Explores multiple solutions simultaneously

### 5.2 System Dynamics

The system is modeled with the following discrete-time equation:

$$
\mathbf{x}_{t+1} = f(\mathbf{x}_t, \mathbf{u}_t)
$$

Where:
- $\mathbf{x}_t \in \mathbb{R}^n$ : system state at time $t$
- $\mathbf{u}_t \in \mathbb{R}^m$ : control input at time $t$
- $f(\cdot)$ : state transition function

**For our omnidirectional robot:**

$$
\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}, \quad \mathbf{u} = \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}
$$

The kinematic model is:

$$
\begin{aligned}
x_{t+1} &= x_t + (v_x \cos\theta_t - v_y \sin\theta_t) \cdot \Delta t \\
y_{t+1} &= y_t + (v_x \sin\theta_t + v_y \cos\theta_t) \cdot \Delta t \\
\theta_{t+1} &= \theta_t + \omega \cdot \Delta t
\end{aligned}
$$

### 5.3 Cost Function

The objective is to minimize the expected cost over a horizon $T$:

$$
J(\mathbf{U}) = \mathbb{E}\left[ \phi(\mathbf{x}_T) + \sum_{t=0}^{T-1} q(\mathbf{x}_t, \mathbf{u}_t) \right]
$$

Where:
- $\mathbf{U} = [\mathbf{u}_0, \mathbf{u}_1, ..., \mathbf{u}_{T-1}]$ : control sequence
- $\phi(\mathbf{x}_T)$ : terminal cost
- $q(\mathbf{x}_t, \mathbf{u}_t)$ : instantaneous (running) cost

**Our cost function includes:**

$$
q(\mathbf{x}_t, \mathbf{u}_t) = w_p \cdot c_{\text{path}} + w_o \cdot c_{\text{obs}} + w_g \cdot c_{\text{goal}} + w_s \cdot c_{\text{smooth}} + w_v \cdot c_{\text{speed}}
$$

| Term | Formula | Description |
|------|---------|-------------|
| $c_{\text{path}}$ | $d_{\text{path}}^2$ | Squared distance to path |
| $c_{\text{obs}}$ | $\exp\left(\frac{d_{\text{margin}} - d_{\text{obs}}}{d_{\text{decay}}}\right)$ | Exponential penalty for obstacle proximity |
| $c_{\text{goal}}$ | $\|\mathbf{x} - \mathbf{x}_{\text{goal}}\|$ | Distance to goal |
| $c_{\text{smooth}}$ | $\|\mathbf{u}_t - \mathbf{u}_{t-1}\|^2$ | Control change penalty |
| $c_{\text{speed}}$ | $(v - v_{\text{desired}})^2$ | Speed deviation penalty |

### 5.4 Trajectory Sampling

MPPI generates $K$ perturbed control sequences:

$$
\mathbf{u}_t^{(k)} = \bar{\mathbf{u}}_t + \boldsymbol{\epsilon}_t^{(k)}
$$

Where:
- $\bar{\mathbf{u}}_t$ : nominal control (from previous iteration)
- $\boldsymbol{\epsilon}_t^{(k)} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma})$ : Gaussian noise

**Noise covariance matrix:**

$$
\boldsymbol{\Sigma} = \begin{bmatrix} \sigma_{v_x}^2 & 0 & 0 \\ 0 & \sigma_{v_y}^2 & 0 \\ 0 & 0 & \sigma_{\omega}^2 \end{bmatrix}
$$

### 5.5 Trajectory Evaluation

For each sample $k$, simulate the complete trajectory:

$$
\mathbf{x}_{t+1}^{(k)} = f(\mathbf{x}_t^{(k)}, \mathbf{u}_t^{(k)})
$$

And compute the total cost:

$$
S^{(k)} = \phi(\mathbf{x}_T^{(k)}) + \sum_{t=0}^{T-1} q(\mathbf{x}_t^{(k)}, \mathbf{u}_t^{(k)})
$$

### 5.6 Weight Computation (Softmin)

Weights are computed using the **softmin** function (Boltzmann distribution):

$$
w^{(k)} = \frac{\exp\left(-\frac{1}{\lambda}(S^{(k)} - S_{\min})\right)}{\sum_{j=1}^{K} \exp\left(-\frac{1}{\lambda}(S^{(j)} - S_{\min})\right)}
$$

Where:
- $\lambda > 0$ : **temperature** parameter
- $S_{\min} = \min_k S^{(k)}$ : minimum cost (for numerical stability)

**Effect of $\lambda$:**
- $\lambda \to 0$ : **greedy** behavior (only best trajectory)
- $\lambda \to \infty$ : uniform average of all trajectories

### 5.7 Optimal Control Update

The optimal control is obtained as a weighted average:

$$
\mathbf{u}_t^* = \sum_{k=1}^{K} w^{(k)} \cdot \mathbf{u}_t^{(k)}
$$

Equivalently:

$$
\mathbf{u}_t^* = \bar{\mathbf{u}}_t + \sum_{k=1}^{K} w^{(k)} \cdot \boldsymbol{\epsilon}_t^{(k)}
$$

### 5.8 Warm Starting

To improve convergence, the control sequence is shifted at each iteration:

$$
\bar{\mathbf{u}}_t^{\text{new}} = \mathbf{u}_{t+1}^{*,\text{old}} \quad \text{for } t = 0, ..., T-2
$$

$$
\bar{\mathbf{u}}_{T-1}^{\text{new}} = \mathbf{u}_{T-1}^{*,\text{old}}
$$

### 5.9 Algorithm Pseudocode

```
Algorithm: MPPI Controller
───────────────────────────────────────────────────────────
Input: current state x₀, target path, obstacles
Output: optimal control command u*

1. Warm start: shift previous control sequence
   ū ← shift(ū_prev)

2. For k = 1 to K:
   a. Generate noise sequence:
      ε^(k) ← sample(N(0, Σ), T)
   
   b. Compute perturbed control sequence:
      u^(k) ← clamp(ū + ε^(k), u_min, u_max)
   
   c. Simulate trajectory:
      x^(k)_0 ← x₀
      For t = 0 to T-1:
         x^(k)_{t+1} ← f(x^(k)_t, u^(k)_t)
   
   d. Compute total cost:
      S^(k) ← Σ_t q(x^(k)_t, u^(k)_t)

3. Find minimum cost:
   S_min ← min_k(S^(k))

4. Compute normalized weights:
   For k = 1 to K:
      w^(k) ← exp(-1/λ · (S^(k) - S_min))
   w ← w / sum(w)

5. Compute optimal control:
   u* ← Σ_k w^(k) · u^(k)

6. Store for next iteration:
   ū_prev ← u*

7. Return u*_0 (first control of sequence)
───────────────────────────────────────────────────────────
```

### 5.10 Flow Diagram

```
                    ┌─────────────────┐
                    │  Current State  │
                    │   x₀, θ₀        │
                    └────────┬────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │        Warm Start            │
              │    ū ← shift(ū_prev)         │
              └──────────────┬───────────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ▼                   ▼                   ▼
    ┌─────────┐         ┌─────────┐         ┌─────────┐
    │Sample 1 │         │Sample 2 │   ...   │Sample K │
    │ε¹~N(0,Σ)│         │ε²~N(0,Σ)│         │εᴷ~N(0,Σ)│
    └────┬────┘         └────┬────┘         └────┬────┘
         │                   │                   │
         ▼                   ▼                   ▼
    ┌─────────┐         ┌─────────┐         ┌─────────┐
    │Simulate │         │Simulate │   ...   │Simulate │
    │Trajectory│        │Trajectory│        │Trajectory│
    └────┬────┘         └────┬────┘         └────┬────┘
         │                   │                   │
         ▼                   ▼                   ▼
    ┌─────────┐         ┌─────────┐         ┌─────────┐
    │ Cost S¹ │         │ Cost S² │   ...   │ Cost Sᴷ │
    └────┬────┘         └────┬────┘         └────┬────┘
         │                   │                   │
         └───────────────────┼───────────────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │      Compute Weights         │
              │  wᵏ = exp(-Sᵏ/λ) / Σexp()   │
              └──────────────┬───────────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │     Weighted Average         │
              │     u* = Σ wᵏ · uᵏ          │
              └──────────────┬───────────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │    Apply u*₀ to robot        │
              │      (vx, vy, ω)             │
              └──────────────────────────────┘
```

### 5.11 Computational Complexity

- **Time**: $O(K \cdot T \cdot (n + m + |\text{obstacles}|))$
- **Space**: $O(K \cdot T \cdot m)$ for storing control sequences

Where:
- $K$ = number of samples
- $T$ = prediction horizon
- $n$ = state dimension
- $m$ = control dimension
- $|\text{obstacles}|$ = number of obstacle points

### 5.12 Advantages and Limitations

**Advantages:**
1. **Derivative-free**: No gradients of cost function required
2. **Non-smooth costs**: Can use discontinuous cost functions
3. **Parallelizable**: K trajectories can be evaluated in parallel (GPU)
4. **Robust**: Explores multiple solutions simultaneously
5. **Easy to implement**: No complex optimization solvers needed

**Limitations:**
1. **Sample dependency**: Needs many samples for good approximation
2. **Limited horizon**: Does not guarantee global optimality
3. **Computational cost**: Can be expensive for high-dimensional systems

### 5.13 Time-Correlated Noise

Standard MPPI uses **independent** Gaussian noise at each timestep. This produces jerky trajectories that oscillate rapidly. We use **temporally correlated noise** via an AR(1) process:

$$
\epsilon_{t+1} = \alpha \cdot \epsilon_t + \sqrt{1-\alpha^2} \cdot \eta_t
$$

Where:
- $\alpha \in [0, 1]$ : correlation factor (typically 0.8-0.95)
- $\eta_t \sim \mathcal{N}(0, \sigma^2)$ : innovation noise
- $\sqrt{1-\alpha^2}$ : scaling to maintain variance

**Effect:**
- $\alpha = 0$ : Independent noise (standard MPPI)
- $\alpha \to 1$ : Highly correlated, smooth trajectories

**Benefits:**
- Smoother sampled trajectories
- Better exploration of solution space
- Reduced jitter in optimal control

### 5.14 Adaptive Covariance

The exploration covariance $\Sigma$ is adapted online based on the **quality of sampled trajectories**:

**Gating condition** (to prevent instability):
```
allow_adapt = (valid_ratio > 0.80) AND (ESS_ratio > 0.10)
```

Where:
- `valid_ratio` = fraction of non-collision trajectories
- `ESS_ratio` = Effective Sample Size / K

**Update rule** (exponential moving average):
$$
\sigma_{new}^2 = (1-\beta) \cdot \sigma_{old}^2 + \beta \cdot \sigma_{weighted}^2
$$

Where $\sigma_{weighted}^2$ is the weighted variance of noise from successful trajectories.

**Bounds:**
```cpp
sigma_vx ∈ [sigma_min_vx, sigma_max_vx]
sigma_vy ∈ [sigma_min_vy, sigma_max_vy]  
sigma_omega ∈ [sigma_min_omega, sigma_max_omega]
```

**Emergency shrink:** When `valid_ratio < 0.5`, covariance is reduced by 10% to escape difficult situations.

### 5.15 ESDF-Based Obstacle Costs

Instead of using raw LiDAR distances, we use the **ESDF distance field** for smooth obstacle costs:

**Fused distance:**
$$
d_{fused} = \min(d_{ESDF}, d_{LiDAR})
$$

**Cost function (softplus barrier):**
$$
c_{obs} = w_{obs} \cdot \text{softplus}\left(\frac{m - d_{fused}}{\tau}\right)
$$

Where:
- $m$ : safety margin (robot radius + buffer)
- $\tau$ : decay parameter (controls steepness)
- $\text{softplus}(x) = \log(1 + e^x)$

**Advantages over LiDAR-only:**
- Smooth gradients for optimization
- No discontinuities at obstacle boundaries
- Better handling of narrow passages

### 5.16 Covariance-Aware Margin Inflation

When the robot's pose has uncertainty (from the localizer), we **inflate the safety margin** proportionally to pose uncertainty. This is the key innovation for robust navigation under localization uncertainty.

#### Mathematical Derivation

**Pose covariance:**
$$
\Sigma_\xi = \begin{bmatrix} \sigma_{xx}^2 & \sigma_{xy} & \sigma_{x\theta} \\ \sigma_{xy} & \sigma_{yy}^2 & \sigma_{y\theta} \\ \sigma_{x\theta} & \sigma_{y\theta} & \sigma_{\theta\theta}^2 \end{bmatrix}
$$

**ESDF gradient at point p:**
$$
\mathbf{g}(p) = \nabla d(p) = \begin{bmatrix} \partial d/\partial x \\ \partial d/\partial y \end{bmatrix}
$$

**Distance variance induced by pose uncertainty:**
$$
\sigma_d^2 = \mathbf{g}^T \Sigma_{xy} \mathbf{g}
$$

Where $\Sigma_{xy}$ is the 2×2 position covariance block.

**Effective margin:**
$$
m_{eff} = m + z \cdot \sigma_d
$$

Where $z$ is a risk multiplier:
- $z = 1.64$ : 95% one-sided confidence
- $z = 2.33$ : 99% one-sided confidence

**Penetration cost:**
$$
pen = \max(0, m_{eff} - d_{fused})
$$

$$
c_{obs} = w_{obs} \cdot \left(\frac{pen}{m_{eff}}\right)^2
$$

#### Algorithm

```
For each trajectory state (x, y, θ):
1. Query ESDF: d_esdf, gradient g
2. Compute d_fused = min(d_esdf, d_lidar)
3. If d_fused < gate × m:  // Only inflate near obstacles
   a. Extract Σ_xy from pose covariance
   b. Compute σ_d = sqrt(g^T × Σ_xy × g)
   c. Clamp: σ_d = min(σ_d, σ_max_clamp × m)
   d. Inflate: m_eff = m + z × σ_d
4. Else: m_eff = m
5. Compute penetration and cost
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cov_z_score` | 1.64 | Risk multiplier (1.64=95%, 2.33=99%) |
| `cov_inflation_gate` | 2.0 | Only inflate when d < gate × margin |
| `cov_sigma_max_clamp` | 0.5 | Clamp σ_d to this fraction of margin |

#### Effect

- **High pose uncertainty** → Larger effective margin → More conservative
- **Low pose uncertainty** → Normal margin → Efficient navigation
- **Only near obstacles** → No overhead in open space

### 5.17 Robot Footprint Sampling (8-Point Collision Detection)

Standard MPPI implementations model the robot as a **single point** (center) for collision detection. This is imprecise for rectangular robots, especially when:
- Navigating through narrow passages
- Rotating near obstacles
- Approaching walls at an angle

We implement **8-point footprint sampling** for precise obstacle distance computation.

#### Footprint Points Layout

```
Robot frame (X+ = right, Y+ = forward):

    P0 -------- P1 -------- P2      ← Front edge
    |                        |
    P7        CENTER        P3      ← Side edges
    |                        |
    P6 -------- P5 -------- P4      ← Rear edge

Points:
  P0 = front-left corner      P1 = front-center      P2 = front-right corner
  P3 = right-center           P4 = rear-right corner
  P5 = rear-center            P6 = rear-left corner  P7 = left-center
```

#### Coordinate Transformation

Each footprint point is transformed from robot frame to world frame:

$$
\begin{bmatrix} x_w \\ y_w \end{bmatrix} = 
\begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} x_r \\ y_r \end{bmatrix} +
\begin{bmatrix} x_{robot} \\ y_{robot} \end{bmatrix}
$$

Where $(x_r, y_r)$ are local coordinates and $(x_{robot}, y_{robot}, \theta)$ is the robot pose.

#### Distance Computation

For each trajectory state, we compute:

1. **Minimum LiDAR distance**: Check all 8 footprint points against all LiDAR obstacles
   $$d_{lidar} = \min_{i \in [0,7]} \min_{j \in obs} \|P_i - obs_j\|$$

2. **Minimum ESDF distance**: Query ESDF at each footprint point
   $$d_{esdf} = \min_{i \in [0,7]} ESDF(P_i)$$

3. **Fused distance**: Conservative minimum
   $$d_{fused} = \min(d_{esdf}, d_{lidar})$$

#### Collision Detection

A collision is detected if **any** footprint point is within the collision threshold:

```cpp
for (const auto& fp : footprint_points) {
    for (const auto& obs : obstacles) {
        if ((fp - obs).squaredNorm() < threshold_sq) {
            collision = true;
            break;
        }
    }
}
```

#### Implementation

```cpp
std::array<Eigen::Vector2f, 8> getFootprintPoints(const State& state) const {
    const float sw = params_.robot_semi_width;   // Half width
    const float sl = params_.robot_semi_length;  // Half length
    
    // 8 points in robot frame
    const std::array<Eigen::Vector2f, 8> local = {{
        {-sw,  sl}, { 0, sl}, { sw,  sl},  // Front: left, center, right
        { sw,  0},                          // Right center
        { sw, -sl}, { 0,-sl}, {-sw, -sl},  // Rear: right, center, left
        {-sw,  0}                           // Left center
    }};
    
    // Transform to world frame
    const float c = cos(state.theta), s = sin(state.theta);
    std::array<Eigen::Vector2f, 8> world;
    for (size_t i = 0; i < 8; ++i) {
        world[i] = {
            state.x + local[i].x()*c - local[i].y()*s,
            state.y + local[i].x()*s + local[i].y()*c
        };
    }
    return world;
}
```

#### Computational Cost

| Method | ESDF Queries | LiDAR Checks | Precision |
|--------|--------------|--------------|-----------|
| Single point (center) | 1 | O(N) | Low |
| **8-point footprint** | **8** | **O(8N)** | **High** |

The 8x increase in ESDF queries is acceptable because:
- ESDF queries are O(1) with precomputed distances
- Better precision prevents oscillations near walls (fewer recovery maneuvers)
- Enables tighter navigation in narrow passages

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_semi_width` | 230 mm | Half of robot width (X direction) |
| `robot_semi_length` | 240 mm | Half of robot length (Y direction) |
| `use_footprint_sampling` | true | Enable 8-point sampling |

#### Benefits

1. **Precise narrow passage navigation**: Robot correctly estimates clearance on both sides
2. **Better rotation near obstacles**: Corner points detect imminent collisions
3. **Reduced oscillations**: More accurate cost gradients lead to smoother trajectories
4. **Asymmetric robot support**: Works with non-square robots (different width/length)

### 5.18 Adaptive K (Sample Count) Based on ESS Theory

Standard MPPI uses a fixed number of samples K. However, the optimal K depends on the situation:
- **Open spaces**: Few samples needed (most trajectories are similar)
- **Near obstacles**: More samples needed for better exploration
- **Narrow passages**: More samples to find collision-free paths

We implement **adaptive K** based on **Effective Sample Size (ESS)** theory from importance sampling.

#### Theoretical Foundation

The ESS measures how many samples effectively contribute to the weighted average:

$$
ESS = \frac{1}{\sum_{k=1}^{K} w_k^2}
$$

Where $w_k$ are the normalized weights. The **ESS ratio** = ESS/K indicates sampling efficiency:

| ESS Ratio | Interpretation | Action |
|-----------|----------------|--------|
| < 0.10 | Few trajectories dominate (poor exploration) | **Increase K** |
| 0.10 - 0.50 | Normal range | Maintain K |
| > 0.50 | Many similar trajectories (over-sampling) | **Decrease K** |

#### Monte Carlo Variance Theory

The variance of the MPPI estimate decreases with ESS, not raw K:

$$
\text{Var}[\hat{u}^*] \propto \frac{1}{ESS}
$$

Therefore, to maintain constant estimation quality, we adapt K based on ESS ratio.

#### Adaptation Algorithm

```cpp
if (ess_ratio < ess_ratio_low && valid_ratio > 0.5f)
    // Poor exploration: increase K
    adaptive_K = min(K_max, K * K_increase_factor);
else if (ess_ratio > ess_ratio_high && valid_ratio > 0.9f)
    // Over-sampling: decrease K to save CPU
    adaptive_K = max(K_min, K * K_decrease_factor);
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_adaptive_K` | true | Enable/disable adaptive sample count |
| `K_min` | 50 | Minimum samples (CPU floor) |
| `K_max` | 300 | Maximum samples (CPU ceiling) |
| `ess_ratio_low` | 0.10 | Below this: increase K |
| `ess_ratio_high` | 0.50 | Above this: decrease K |
| `K_increase_factor` | 1.3 | Multiply K when ESS too low |
| `K_decrease_factor` | 0.85 | Multiply K when ESS too high |

#### Expected Behavior

| Scenario | ESS Ratio | K Trend | CPU Usage |
|----------|-----------|---------|-----------|
| Open space, straight path | ~0.6 | ↓ to K_min (~50) | Low |
| Approaching obstacle | ~0.15 | ↑ (~150) | Medium |
| Narrow passage | ~0.08 | ↑ to K_max (~300) | High |
| After clearing obstacle | ~0.5 | ↓ gradually | Decreasing |

#### References

- Owen, A. (2013). *Monte Carlo Theory, Methods and Examples* - ESS theory
- Williams, G., et al. (2017). "Information Theoretic MPC" - MPPI foundations

### 5.19 Output Smoothing Filter

MPPI inherently produces some **jitter** in velocity commands due to:
1. **Stochastic sampling**: Each iteration uses different random trajectories
2. **Weight concentration**: ESS can be low, causing few trajectories to dominate
3. **Variable computation time**: Period fluctuations affect control updates

To reduce this jitter, we apply an **Exponential Moving Average (EMA) filter** to the output:

#### Mathematical Formulation

$$
\mathbf{u}_{smoothed} = \alpha \cdot \mathbf{u}_{prev} + (1 - \alpha) \cdot \mathbf{u}_{raw}
$$

Where:
- $\alpha \in [0, 1]$ : smoothing factor
- $\mathbf{u}_{raw}$ : raw MPPI output (weighted average)
- $\mathbf{u}_{prev}$ : previous smoothed output

#### Effect of $\alpha$

| $\alpha$ | Behavior | Use Case |
|----------|----------|----------|
| 0.0 | No smoothing (raw output) | Maximum responsiveness |
| 0.2 | Light smoothing | Fast dynamic environments |
| **0.3** | **Moderate smoothing (default)** | **Balanced performance** |
| 0.5 | Heavy smoothing | Smooth motion priority |
| 0.7+ | Very heavy smoothing | May introduce lag |

#### Frequency Domain Analysis

The EMA filter acts as a **first-order low-pass filter** with cutoff frequency:

$$
f_c = \frac{f_s}{2\pi} \cdot \ln\left(\frac{1}{1-\alpha}\right)^{-1}
$$

Where $f_s$ is the MPPI sampling frequency (~20 Hz).

For $\alpha = 0.3$ at 20 Hz: $f_c \approx 4.5$ Hz

#### Implementation

```cpp
// Applied before returning the control command
ControlCommand smooth_output(const ControlCommand& raw) {
    if (first_output_ || params_.output_smoothing_alpha <= 0.0f) {
        last_smoothed_output_ = raw;
        first_output_ = false;
        return raw;
    }
    
    const float alpha = params_.output_smoothing_alpha;
    const float one_minus_alpha = 1.0f - alpha;
    
    ControlCommand smoothed;
    smoothed.vx = alpha * last_smoothed_output_.vx + one_minus_alpha * raw.vx;
    smoothed.vy = alpha * last_smoothed_output_.vy + one_minus_alpha * raw.vy;
    smoothed.omega = alpha * last_smoothed_output_.omega + one_minus_alpha * raw.omega;
    
    last_smoothed_output_ = smoothed;
    return smoothed;
}
```

#### Expected Improvement

| Metric | Without Filter | With Filter (α=0.3) |
|--------|----------------|---------------------|
| vy variation | ~40 mm/s | ~10 mm/s |
| vx oscillation | ±10 mm/s | ±2 mm/s |
| omega jitter | ±0.1 rad/s | ±0.03 rad/s |

#### Interaction with Other Components

The output smoothing filter is the **final stage** of the MPPI pipeline:

```
Trajectory Sampling → Cost Evaluation → Weight Computation 
    → Weighted Average → Output Smoothing Filter → Robot
```

It operates **after** the warm start and adaptive covariance mechanisms, so it doesn't affect the internal MPPI optimization.

#### Reset Behavior

The filter state is reset when:
- `reset()` is called (new navigation target)
- Controller is re-initialized
- First command after idle period

This prevents the filter from "remembering" old velocities when starting a new motion.

### 5.20 MPPI Configuration Parameters

```cpp
struct Params {
    // Sampling
    int K = 512;                    // Number of trajectory samples
    int T = 30;                     // Prediction horizon (steps)
    float dt = 0.05f;               // Timestep (seconds)
    float lambda = 1.0f;            // Temperature parameter
    
    // Time-correlated noise
    bool use_time_correlated_noise = true;
    float noise_alpha = 0.9f;       // Correlation factor
    
    // Adaptive covariance
    bool use_adaptive_covariance = true;
    float cov_adaptation_rate = 0.1f;
    float sigma_vx = 50.0f;         // Initial sigma for vx (mm/s)
    float sigma_vy = 150.0f;        // Initial sigma for vy (mm/s)
    float sigma_omega = 0.3f;       // Initial sigma for omega (rad/s)
    
    // Robot limits
    float max_vx = 300.0f;          // mm/s - lateral speed
    float max_vy = 800.0f;          // mm/s - forward speed
    float max_omega = 0.5f;         // rad/s - rotation
    
    // Cost weights
    float w_path = 1.0f;            // Path following
    float w_obstacle = 50.0f;       // Obstacle avoidance
    float w_goal = 5.0f;            // Goal reaching
    float w_smoothness = 1.0f;      // Control smoothness
    float w_speed = 0.001f;         // Speed preference
    
    // Safety
    float robot_radius = 250.0f;    // mm - collision threshold
    float collision_buffer = 120.0f; // mm - soft penalty band
    float safety_margin = 1000.0f;  // mm - outer cost zone
    
    // Robot footprint (8-point sampling)
    float robot_semi_width = 230.0f;   // mm - half width (X direction)
    float robot_semi_length = 240.0f;  // mm - half length (Y direction)
    bool use_footprint_sampling = true; // Enable 8-point collision detection
    
    // Covariance-aware margin inflation
    bool use_covariance_inflation = true;
    float cov_z_score = 1.64f;      // Risk multiplier
    float cov_inflation_gate = 2.0f;
    float cov_sigma_max_clamp = 0.5f;
    
    // Output smoothing (reduces jitter)
    float output_smoothing_alpha = 0.3f;  // EMA filter: 0=off, 0.3=default, 0.5=heavy
};
```

---

## 7. Implementation Details

### 7.1 Data Structures

#### Obstacle Cell (Sparse ESDF)
```cpp
struct ObstacleCell {
    float log_odds = 2.0f;
    float hits = 1.0f;
    uint64_t last_seen = 0;
    QGraphicsRectItem *tile = nullptr;
};
```

#### Particle
```cpp
struct Particle {
    Pose2D pose;
    double log_weight = 0.0;
    
    double getWeight() const { return std::exp(log_weight); }
};
```

### 7.2 Key Conversions

**Point to Key:**
```cpp
Key point_to_key(float x, float y) {
    int kx = static_cast<int>(std::floor(x / tile_size)) * tile_size;
    int ky = static_cast<int>(std::floor(y / tile_size)) * tile_size;
    return {kx, ky};
}
```

### 7.3 Thread Architecture

| Thread | Function | Synchronization |
|--------|----------|-----------------|
| LiDAR reader | `read_lidar()` | DoubleBuffer |
| Localizer | `run_localizer()` | atomic flags, buffer |
| MPPI | `run_mppi()` | atomic flags, buffer |
| Main (compute) | `compute()` | mutex, Qt event loop |

### 7.4 MRPT Map Loading

```cpp
bool Gridder_loadMRPTMap(const string& filepath) {
    // 1. Detect compression (zstd)
    // 2. Parse header (resolution, bounds)
    // 3. For each occupied cell (value > 127):
    //    - Apply rotation transform
    //    - Apply offset translation
    //    - Add as confirmed obstacle
    // 4. Adjust tile_size to match map resolution
    // 5. Precompute distance field
    return success;
}
```

---

## 8. Optimizations

### 8.1 Precomputed Distance Field

**Problem:** ESDF queries are O(R²) per call, called millions of times during localization.

**Solution:** Precompute distances for all cells in bounding box.

```cpp
void precompute_distance_field() {
    // Precompute hypot lookup table
    for (dx, dy in [-R, R]):
        hypot_table[dx][dy] = sqrt(dx² + dy²) * tile_size;
    
    // For each cell, find nearest obstacle
    for (cell in bounding_box):
        min_dist = search_with_hypot_table(cell);
        precomputed_distances_[cell] = min_dist;
}
```

**Result:** O(1) queries instead of O(R²)

### 8.2 Fused Particle Operations

**Problem:** Multiple passes over particles array (normalize, ESS, mean).

**Solution:** Single-pass fused operation.

```cpp
void normalizeAndComputeStats() {
    // Pass 1: Find max log-weight
    float max_log_w = max(particles.log_weight);
    
    // Pass 2: Everything in one loop
    for (particle in particles):
        // Normalize weight
        particle.log_weight -= max_log_w;
        float w = exp(particle.log_weight);
        sum_weights += w;
        
        // ESS accumulator
        sum_weights_sq += w * w;
        
        // Mean pose accumulators
        sum_x += w * particle.pose.x;
        sum_cos += w * cos(particle.pose.theta);
        // ...
    
    // Finalize
    cached_ess_ = 1.0 / (sum_weights_sq / sum_weights²);
    cached_mean_pose_ = {sum_x/sum_weights, ...};
}
```

**Result:** 2 passes instead of 6+

### 8.3 Float vs Double

| Variable | Type | Reason |
|----------|------|--------|
| Positions (x, y) | `float` | Sufficient precision for mm |
| Angles | `float` | Sufficient for radians |
| Log-weights | `double` | Numerical stability (very small values) |
| Weight sums | `double` | Accumulation precision |
| Trigonometry | `float` | Use `sinf`, `cosf`, `expf` |

### 8.4 LiDAR Subsampling

Process every N-th point instead of all:

```cpp
const int step = params.lidar_subsample;  // e.g., 15
for (size_t i = 0; i < points.size(); i += step):
    process_point(points[i]);
```

**Trade-off:** N=15 gives good balance (140 points from 2100)

### 8.5 Visualization Throttling

```cpp
// Particles: draw every 10 frames
static int viz_counter = 0;
if (++viz_counter % 10 == 0)
    drawParticles();

// Max 500 particles drawn (subsample if more)
const size_t max_draw = 500;
const size_t step = max(1, particles.size() / max_draw);
```

### 8.6 Complexity Summary

| Operation | Before | After |
|-----------|--------|-------|
| ESDF query | O(R²) | O(1) |
| Particle stats | O(6N) | O(2N) |
| Math functions | double | float |
| Visualization | Every frame | Every 10 frames |

---

## 9. Configuration Reference

### 9.1 Grid Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TILE_SIZE` | 100 mm | Grid cell size |
| `MAX_LIDAR_RANGE` | 15000 mm | Maximum LiDAR range |
| `max_esdf_distance` | 500 mm | ESDF search radius |
| `log_odds_hit` | +1.0 | Log-odds for obstacle hit |
| `log_odds_miss` | -0.4 | Log-odds for free space |
| `occupancy_threshold` | 2.0 | Log-odds to confirm obstacle |

### 9.2 Path Planning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ROBOT_WIDTH` | 460 mm | Robot width for collision |
| `SAFETY_FACTOR` | 0.5 | Path safety preference (0-1) |
| `max_astar_nodes` | 50000 | A* expansion limit |
| `obstacle_cost` | 100 | Cost at obstacles |
| `free_cost` | 1 | Cost in free space |

### 9.3 Localizer Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `initial_particles` | 500 | Starting particle count |
| `min_particles` | 100 | Minimum after KLD |
| `max_particles` | 1000 | Maximum particle count |
| `lidar_subsample` | 15 | Use every N-th point |
| `sigma_hit` | 100 mm | Observation noise std dev |
| `z_hit` | 0.95 | Hit model weight |
| `z_rand` | 0.05 | Random model weight |
| `resample_threshold` | 0.5 | ESS ratio for resampling |

### 9.4 MRPT Map Loading

| Parameter | Description |
|-----------|-------------|
| `MRPT_MAP_OFFSET_X` | X translation (mm) |
| `MRPT_MAP_OFFSET_Y` | Y translation (mm) |
| `MRPT_MAP_ROTATION` | Rotation angle (radians) |

### 9.5 MPPI Controller Parameters

| Parameter | Symbol | Default | Description |
|-----------|--------|---------|-------------|
| `K` | $K$ | 100 | Initial number of sampled trajectories |
| `T` | $T$ | 50 | Prediction horizon (time steps) |
| `dt` | $\Delta t$ | 0.1 s | Time step for simulation |
| `lambda` | $\lambda$ | 50.0 | Temperature (exploration vs exploitation) |
| `use_adaptive_K` | - | true | Enable ESS-based adaptive sample count |
| `K_min` | - | 50 | Minimum samples (CPU floor) |
| `K_max` | - | 300 | Maximum samples (CPU ceiling) |
| `ess_ratio_low` | - | 0.10 | ESS ratio below which K increases |
| `ess_ratio_high` | - | 0.50 | ESS ratio above which K decreases |
| `sigma_vx` | $\sigma_{v_x}$ | 80 mm/s | Noise std dev for lateral velocity |
| `sigma_vy` | $\sigma_{v_y}$ | 150 mm/s | Noise std dev for forward velocity |
| `sigma_omega` | $\sigma_\omega$ | 0.3 rad/s | Noise std dev for angular velocity |
| `max_vx` | - | 300 mm/s | Maximum lateral velocity |
| `max_vy` | - | 800 mm/s | Maximum forward velocity |
| `max_omega` | - | 0.5 rad/s | Maximum angular velocity |
| `robot_radius` | $r$ | 250 mm | Robot radius for collision detection |
| `safety_margin` | $d_{\text{margin}}$ | 1000 mm | Distance to start penalizing obstacles |
| `collision_buffer` | - | 120 mm | Soft penalty band before collision |
| `goal_tolerance` | - | 200 mm | Distance to consider goal reached |
| `w_path` | $w_p$ | 1.0 | Weight for path following cost |
| `w_obstacle` | $w_o$ | 50.0 | Weight for obstacle avoidance cost |
| `w_goal` | $w_g$ | 5.0 | Weight for goal reaching cost |
| `w_smoothness` | $w_s$ | 1.0 | Weight for control smoothness cost |
| `w_speed` | $w_v$ | 0.001 | Weight for speed maintenance cost |
| `robot_semi_width` | - | 230 mm | Half robot width for footprint (X) |
| `robot_semi_length` | - | 240 mm | Half robot length for footprint (Y) |
| `use_footprint_sampling` | - | true | Enable 8-point footprint collision |
| `output_smoothing_alpha` | $\alpha$ | 0.3 | EMA filter for output (0=off, 0.5=heavy) |

---

## 10. References

1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

2. Tipaldi, G. D., & Burgard, W. *Robot Mapping: Grid Maps* - Course slides.

3. Oleynikova, H., et al. (2017). "Voxblox: Incremental 3D Euclidean Signed Distance Fields." *IEEE/RSJ IROS*.

4. Fox, D. (2003). "KLD-Sampling: Adaptive Particle Filters." *NIPS*.

5. Felzenszwalb, P. & Huttenlocher, D. (2012). "Distance Transforms of Sampled Functions." *Theory of Computing*.

6. Williams, G., et al. (2017). "Information theoretic MPC for model-based reinforcement learning." *IEEE ICRA*.

7. Williams, G., et al. (2016). "Aggressive driving with model predictive path integral control." *IEEE ICRA*.

8. Theodorou, E., et al. (2010). "A generalized path integral control approach to reinforcement learning." *JMLR*.

---

*Document generated: 2026-02-15*
*Component version: Gridder v2.3*

**Recent Changes (v2.3):**
- Adaptive K (sample count) based on ESS theory for dynamic CPU optimization
- Corrected architecture diagram to show proper data flow
- MPPI thread sleeps when idle (goal reached) to save resources

**Changes (v2.2):**
- Added 8-point robot footprint sampling for precise collision detection
- Supports rectangular robots with configurable dimensions
- Improved narrow passage navigation
- Output smoothing filter (EMA) to reduce velocity jitter
- Eigen::Affine2f for cleaner coordinate transformations
