# GRIDDER: Complete Technical Documentation

## Table of Contents

1. [Introduction](#1-introduction)
2. [Sparse ESDF Grid](#2-sparse-esdf-grid)
3. [Path Planning with ESDF](#3-path-planning-with-esdf)
4. [Particle Filter Localization](#4-particle-filter-localization)
5. [MPPI Controller](#5-mppi-controller)
6. [Implementation Details](#6-implementation-details)
7. [Configuration Reference](#7-configuration-reference)
8. [References](#8-references)

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
┌────────────────────────────────────────────────────────���────────┐
│                          GRIDDER                                 │
├─────────────────────────────────────────────────────────────────┤
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐   │
│  │  LiDAR Thread │───▶│   Grid ESDF   │───▶│  Visualizer   │   │
│  │  (read_lidar) │    │  (sparse map) │    │  (Qt Scene)   │   │
│  └───────────────┘    └───────────────┘    └───────────────┘   │
│          │                    │                    │            │
│          ▼                    ▼                    ▼            │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐   │
│  │  Robot Pose   │    │ Path Planner  │    │   Localizer   │   │
│  │    Buffer     │    │    (A*)       │    │    (AMCL)     │   │
│  └───────────────┘    └───────────────┘    └───────────────┘   │
│          │                    │                    │            │
│          └────────────────────┼────────────────────┘            │
│                               ▼                                  │
│                    ┌───────────────────┐                        │
│                    │  MPPI Controller  │                        │
│                    │  (covariance-aware│                        │
│                    │   navigation)     │                        │
│                    └───────────────────┘                        │
├─────────────────────────────────────────────────────────────────┤
│                       ICE Interface                              │
│   getPaths() | getMap() | getPose() | LineOfSightToTarget()     │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Sparse ESDF Grid

### 2.1 Data Structure

The sparse grid uses a hash map to store only occupied cells:

```cpp
using Key = std::pair<int, int>;  // (x, y) in grid coordinates
struct Cell {
    float esdf_distance;     // Distance to nearest obstacle
    Eigen::Vector2f gradient; // Gradient of distance field
    float weight;            // Observation confidence (TSDF-style)
    QGraphicsRectItem* tile; // Visualization
};
std::unordered_map<Key, Cell> cells_;
```

### 2.2 TSDF-Style Weighted Updates

Instead of binary Bayes filter, we use a **weighted averaging** approach inspired by TSDF:

$$
w_{new} = w_{old} + w_{obs}
$$
$$
d_{new} = \frac{w_{old} \cdot d_{old} + w_{obs} \cdot d_{obs}}{w_{new}}
$$

Where:
- $w_{obs}$: Observation weight (decreases with distance from sensor)
- $d_{obs}$: Observed distance value
- Weights are clamped to prevent overflow

**Advantages over Binary Bayes:**
- More stable obstacle boundaries
- Natural handling of sensor noise
- Better temporal consistency

### 2.3 ESDF Computation

The Euclidean Signed Distance Field is computed using **Brushfire algorithm**:

1. **Initialize**: All obstacle cells have distance 0
2. **Propagate**: BFS from obstacles, updating neighbor distances
3. **Result**: Each cell contains distance to nearest obstacle

**Distance update:**
$$
d(p) = \min_{q \in N(p)} \left( d(q) + \|p - q\| \right)
$$

### 2.4 ESDF Gradient

The gradient points **away from obstacles** and is computed as:

$$
\nabla d(p) = \frac{p - p_{closest}}{\|p - p_{closest}\|}
$$

Where $p_{closest}$ is the nearest obstacle to point $p$.

**Uses:**
- Smooth obstacle avoidance costs in MPPI
- Covariance-aware margin inflation
$$

**Without prior knowledge**: $p(m) = 0.5$

### 2.3 Binary Bayes Filter Derivation

Given sensor observations $z_{1:t}$ and robot poses $x_{1:t}$, we estimate:

$$
p(m \mid z_{1:t}, x_{1:t})
$$

**Step 1: Apply Bayes' Theorem**

Starting from Bayes' theorem:
$$
p(A \mid B) = \frac{p(B \mid A) \cdot p(A)}{p(B)}
$$

For our problem with $A = m$ (cell occupied), $B = z_{1:t}$ (all observations):

$$
p(m \mid z_{1:t}) = \frac{p(z_{1:t} \mid m) \cdot p(m)}{p(z_{1:t})}
$$

Separating current observation $z_t$ from history $z_{1:t-1}$ and applying chain rule:

$$
p(m \mid z_{1:t}) = \frac{p(z_t \mid m) \cdot p(m \mid z_{1:t-1})}{p(z_t \mid z_{1:t-1})}
$$

**Step 2: Compute Odds Ratio**

Computing $\frac{p(m \mid z_{1:t})}{p(\neg m \mid z_{1:t})}$:

$$
\frac{p(m \mid z_{1:t})}{p(\neg m \mid z_{1:t})} = \frac{p(m \mid z_{1:t-1})}{p(\neg m \mid z_{1:t-1})} \cdot \frac{p(m \mid z_t)}{p(\neg m \mid z_t)} \cdot \frac{p(\neg m)}{p(m)}
$$

### 2.4 Log-Odds Representation

**Definition:**
$$
l(x) = \log\left(\frac{p(x)}{1 - p(x)}\right)
$$

**Update Formula:**
$$
l(m \mid z_{1:t}) = l(m \mid z_{1:t-1}) + l(m \mid z_t) - l_0
$$

With uniform prior ($l_0 = 0$):
$$
l_{\text{new}} = l_{\text{previous}} + l_{\text{observation}}
$$

**Conversion back to probability:**
$$
p(m) = 1 - \frac{1}{1 + e^{l(m)}}
$$

### 2.5 Inverse Sensor Model for LiDAR

For each laser ray with measured distance $z$:

| Zone | Condition | Log-odds Update |
|------|-----------|-----------------|
| **Free** | Cell distance < z | $l_{\text{free}} = -0.4$ |
| **Occupied** | Cell distance ≈ z | $l_{\text{occ}} = +1.4$ |
| **Unknown** | Cell distance > z | No update |

```
         sensor
            │
            │  "free" zone (l_free < 0)
            ▼
    ─────────────────────── ← measured point z (l_occ > 0)
            │
            │  "unknown" zone (no update)
            ▼
```

### 2.6 Log-Odds Parameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| $l_{\text{occ}}$ | +1.4 | Log-odds increment for hit |
| $l_{\text{free}}$ | -0.4 | Log-odds decrement for miss |
| $l_{\min}$ | -2.0 | Minimum limit (high confidence free) |
| $l_{\max}$ | +4.0 | Maximum limit (high confidence occupied) |
| Threshold | 2.0 | Log-odds value to confirm obstacle |

**Correspondence Table:**

| Log-odds | Occupied Probability |
|----------|---------------------|
| -2.0 | ~12% (very free) |
| -1.0 | ~27% |
| 0.0 | 50% (unknown) |
| +1.0 | ~73% |
| +2.0 | ~88% |
| +4.0 | ~98% (very occupied) |

---

## 3. ESDF Construction

### 3.1 Overview

The **Euclidean Signed Distance Field (ESDF)** provides the distance to the nearest obstacle for any point in the map:

$$
d_E(x) = \min_{o \in \mathcal{O}} \|x - o\|
$$

Where $\mathcal{O}$ is the set of obstacle cells.

### 3.2 Traditional Ray-Casting vs Sparse ESDF

#### Traditional Approach (Dense Grid with Ray-Casting)

The classical method for occupancy grid construction traces each LiDAR ray from the sensor origin to the hit point:

```
Sensor ──────────────────────────────● Hit
        ↓     ↓     ↓     ↓     ↓    ↓
       FREE  FREE  FREE  FREE  FREE  OCC
```

**Algorithm:**
```cpp
for each LiDAR point p:
    for each cell c along ray(sensor, p):
        if c is before p:
            c.log_odds += l_free    // Mark as free
        else if c is at p:
            c.log_odds += l_occ     // Mark as occupied
```

**Complexity per scan:** O(n × r) where n = LiDAR points (~2000), r = average ray length in cells (~150)
- **Total operations:** ~300,000 cell updates per scan
- **Memory:** O(width × height) - all cells stored

#### Sparse ESDF Approach (Default)

The sparse approach **only stores obstacle cells**, dramatically reducing both memory and computation:

```
Sensor ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─● Hit
        (implicit free space)        ↓
                                    OCC (only this stored)
```

**Algorithm:**
```cpp
for each LiDAR point p:
    key = point_to_key(p)
    obstacles_[key].log_odds += l_occ  // Only update hit cell
    obstacles_[key].hits++
```

**Complexity per scan:** O(n) where n = LiDAR points
- **Total operations:** ~2,000 cell updates per scan (150× fewer!)
- **Memory:** O(obstacles) - typically 1-5% of total area

#### Performance Comparison

| Aspect | Dense Ray-Casting | Sparse ESDF |
|--------|-------------------|-------------|
| Memory (50m × 50m, 100mm cells) | 250,000 cells (~1 MB) | ~5,000 cells (~20 KB) |
| Updates per scan | ~300,000 | ~2,000 |
| Free space handling | Explicit (stored) | Implicit (not in map = free) |
| Ray tracing | Required | Not needed |
| Typical FPS | 2-5 Hz | 15-30 Hz |
| Best for | Small/dense environments | Large/sparse environments |

#### Trade-offs

**When Dense Ray-Casting is better:**
- Small environments where memory is not a concern
- Need to distinguish "unknown" from "free" cells
- Very dense obstacle environments (>20% occupied)
- Situations requiring explicit free-space proof

**When Sparse ESDF is better:**
- Large environments (hallways, outdoor)
- Real-time requirements (>10 Hz)
- Memory-constrained systems
- Path planning (ESDF provides smooth gradients)

### 3.3 Sparse Representation Details (VoxBlox-style)

**Key insight**: In most environments, obstacles occupy only 1-5% of the total area.

```
Dense Grid:  O(width × height) cells stored
Sparse Grid: O(obstacles) cells stored → 20-100× memory reduction
```

**Convention:**
- Cell NOT in map → FREE (implicit)
- Cell IN map → OBSTACLE (explicit)

### 3.4 Obstacle Confirmation

To filter noise, obstacles require confirmation:

```cpp
struct ObstacleCell {
    float log_odds;       // Accumulated confidence
    float hits;           // Number of observations
    uint64_t last_seen;   // Timestamp
    QGraphicsRectItem *tile;  // Visual (only when confirmed)
};
```

**Confirmation criteria:**
$$
\text{hits} \geq 3 \quad \text{AND} \quad \text{log\_odds} \geq 2.0
$$

### 3.5 ESDF Computation Methods

#### 3.5.1 On-Demand (Lazy Evaluation)

```cpp
float get_distance(const Key &k) {
    // Check cache
    if (esdf_cache_.contains(k))
        return esdf_cache_[k].distance;
    
    // Compute: search nearby obstacles
    float min_dist = max_esdf_distance;
    for (int radius = 1; radius <= search_radius; ++radius) {
        for each neighbor at radius:
            if (is_obstacle(neighbor))
                min_dist = min(min_dist, distance(k, neighbor));
        if (min_dist < radius * tile_size)
            break;  // Early termination
    }
    
    esdf_cache_[k] = min_dist;
    return min_dist;
}
```

**Complexity:** O(R²) per query, where R = search radius in cells

#### 3.5.2 Precomputed Distance Field

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

### 3.6 Cost Function from ESDF

The ESDF enables smooth cost gradients:

$$
\text{cost}(x) = \begin{cases}
C_{\text{obstacle}} & \text{if } d_E(x) = 0 \\
C_{\text{obstacle}} \cdot e^{-d_E(x) / \tau} & \text{if } d_E(x) < 2\tau \\
0.25 \cdot C_{\text{obstacle}} \cdot e^{-(d_E(x) - 2\tau) / 2\tau} & \text{if } d_E(x) < 4\tau \\
C_{\text{free}} & \text{otherwise}
\end{cases}
$$

Where $\tau$ = tile_size (100mm default), $C_{\text{obstacle}}$ = 100, $C_{\text{free}}$ = 1.

### 3.7 Inflation Layers Visualization

```
  2 2 2 2 2
  2 1 1 1 2     ■ = Obstacle (red, cost=100)
  2 1 ■ 1 2     1 = Layer 1 (orange, cost~50)
  2 1 1 1 2     2 = Layer 2 (green, cost~25)
  2 2 2 2 2
```

---

## 4. Path Planning with ESDF

### 4.1 A* Algorithm Overview

The A* algorithm finds the optimal path by expanding nodes in order of:

$$
f(n) = g(n) + h(n)
$$

Where:
- $g(n)$ = actual cost from start to node $n$
- $h(n)$ = heuristic estimate from $n$ to goal

### 4.2 Heuristic Function

Using Euclidean distance (admissible heuristic):

$$
h(n) = \sqrt{(n_x - \text{goal}_x)^2 + (n_y - \text{goal}_y)^2}
$$

### 4.3 ESDF-Based Movement Cost

For moving from cell $n$ to neighbor $n'$:

$$
\text{move\_cost}(n, n') = d(n, n') + \text{esdf\_cost}(n') \times S_f \times S_s
$$

Where:
- $d(n, n')$ = tile_size (or tile_size × √2 for diagonal)
- $\text{esdf\_cost}(n')$ = cost from ESDF (see §3.6)
- $S_f$ = safety_factor (0.0-1.0, user parameter)
- $S_s$ = safety_scale = $1 + S_f^2 \times 10$ (range 1-11)

### 4.4 Safety Factor Effect

```
safety_factor = 0.0              safety_factor = 1.0
                                 
    ████████                         ████████
    █      █                         █      █
  ──█──────█──  (shortest)           █  ──  █  (safest)
    █      █                         █ /  \ █
    ████████                         ████/──\████
```

### 4.5 Algorithm Implementation

```cpp
vector<Vector2f> compute_path(source, target, robot_radius, safety_factor) {
    priority_queue<(f_score, Key)> open_set;
    unordered_map<Key, float> g_score;
    unordered_map<Key, Key> came_from;
    
    g_score[source] = 0;
    open_set.push({heuristic(source, target), source});
    
    while (!open_set.empty()) {
        Key current = open_set.top().second;
        open_set.pop();
        
        if (current == target)
            return reconstruct_path(came_from, current);
        
        for (neighbor in get_neighbors_8(current)) {
            if (is_occupied(neighbor, robot_radius))
                continue;
            
            float move_cost = distance(current, neighbor);
            float esdf_cost = get_cost(neighbor) * safety_factor * safety_scale;
            float tentative_g = g_score[current] + move_cost + esdf_cost;
            
            if (tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                float f = tentative_g + heuristic(neighbor, target);
                open_set.push({f, neighbor});
            }
        }
    }
    return {};  // No path found
}
```

### 4.6 Line-of-Sight Check

For direct paths without obstacles:

```cpp
bool is_line_of_sight_free(source, target, robot_radius) {
    // Bresenham's line algorithm
    for (cell in line(source, target)) {
        if (get_distance(cell) < robot_radius)
            return false;
    }
    return true;
}
```

### 4.7 Complexity

| Operation | Complexity |
|-----------|------------|
| A* worst case | O(V log V) where V = expanded nodes |
| A* typical | O(path_length × branching_factor × log V) |
| Line-of-sight | O(distance / tile_size) |

---

## 5. Particle Filter Localization

### 5.1 Monte Carlo Localization (AMCL)

The particle filter represents the probability distribution over robot poses using a set of weighted samples (particles):

$$
\text{Bel}(x_t) \approx \{(x_t^{[i]}, w_t^{[i]})\}_{i=1}^{N}
$$

Where:
- $x_t^{[i]}$ = pose of particle $i$ at time $t$
- $w_t^{[i]}$ = weight of particle $i$
- $N$ = number of particles

### 5.2 Algorithm Overview

```
AMCL Update Cycle:
1. PREDICT: Propagate particles through motion model
2. CORRECT: Weight particles by observation likelihood
3. NORMALIZE: Ensure weights sum to 1
4. RESAMPLE: Replace low-weight particles (if ESS low)
5. ESTIMATE: Compute mean pose from weighted particles
```

### 5.3 Motion Model

Given odometry $(δx, δy, δθ)$ in robot frame, propagate particle pose:

$$
\begin{pmatrix} x' \\ y' \\ \theta' \end{pmatrix} = \begin{pmatrix} x \\ y \\ \theta \end{pmatrix} + \begin{pmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{pmatrix} \begin{pmatrix} δx + \epsilon_x \\ δy + \epsilon_y \\ δθ + \epsilon_θ \end{pmatrix}
$$

Where noise terms:
$$
\epsilon_x, \epsilon_y \sim \mathcal{N}(0, \alpha_3 \cdot |δ_{trans}| + \alpha_4 \cdot |δθ|)
$$
$$
\epsilon_θ \sim \mathcal{N}(0, \alpha_1 \cdot |δθ| + \alpha_2 \cdot |δ_{trans}|)
$$

### 5.4 Observation Model (Likelihood Field)

For each LiDAR point, compute likelihood using ESDF:

$$
p(z \mid x, m) = z_{\text{hit}} \cdot p_{\text{hit}}(z \mid x, m) + z_{\text{rand}} \cdot p_{\text{rand}}
$$

Where:
$$
p_{\text{hit}}(z \mid x, m) = \frac{1}{\sigma_{\text{hit}} \sqrt{2\pi}} \exp\left(-\frac{d_E^2(z_{\text{world}})}{2\sigma_{\text{hit}}^2}\right)
$$

- $z_{\text{world}}$ = LiDAR point transformed to world frame using particle pose
- $d_E(z_{\text{world}})$ = ESDF distance at that point
- $\sigma_{\text{hit}}$ = 100mm (sensor noise standard deviation)
- $z_{\text{hit}}$ = 0.95, $z_{\text{rand}}$ = 0.05 (mixture weights)

**Total likelihood for particle:**
$$
p(\text{scan} \mid x^{[i]}) = \exp\left(\frac{1}{n} \sum_{j=1}^{n} \log p(z_j \mid x^{[i]}, m)\right)
$$

### 5.5 Weight Update

Using log-weights for numerical stability:

$$
\log w_t^{[i]} = \log w_{t-1}^{[i]} + \log p(\text{scan} \mid x_t^{[i]})
$$

### 5.6 Effective Sample Size (ESS)

Measure of particle diversity:

$$
\text{ESS} = \frac{1}{\sum_{i=1}^{N} (w^{[i]})^2}
$$

- ESS = N: All particles equal weight (maximum diversity)
- ESS = 1: One particle dominates (degenerate)

**Resample when:** ESS < 0.5 × N

### 5.7 Low-Variance Resampling

```cpp
void resample() {
    vector<Particle> new_particles;
    double step = 1.0 / N;
    double r = random(0, step);
    double cumulative = particles[0].weight;
    int idx = 0;
    
    for (int i = 0; i < N; ++i) {
        while (r > cumulative && idx < N-1)
            cumulative += particles[++idx].weight;
        new_particles.push_back(particles[idx]);
        r += step;
    }
    particles = new_particles;
}
```

### 5.8 KLD-Adaptive Sampling

Adjust particle count based on distribution complexity:

$$
N = \frac{k-1}{2\epsilon} \left(1 - \frac{2}{9(k-1)} + \sqrt{\frac{2}{9(k-1)}} \cdot z_{1-\delta}\right)^3
$$

Where:
- $k$ = number of occupied histogram bins
- $\epsilon$ = 0.02 (KL-distance bound)
- $z_{1-\delta}$ = 2.33 (for 99% confidence)

### 5.9 Mean Pose Estimation

Weighted average (circular mean for angle):

$$
\bar{x} = \sum_{i} w^{[i]} x^{[i]}, \quad \bar{y} = \sum_{i} w^{[i]} y^{[i]}
$$
$$
\bar{\theta} = \text{atan2}\left(\sum_{i} w^{[i]} \sin\theta^{[i]}, \sum_{i} w^{[i]} \cos\theta^{[i]}\right)
$$

### 5.10 Convergence Detection

Based on pose covariance:

$$
\sigma_{\text{pos}} = \sqrt{\text{trace}(\Sigma_{xy})} < \sigma_{\text{threshold}}
$$
$$
\sigma_{\theta} < \theta_{\text{threshold}}
$$

---

## 6. MPPI Controller

### 6.1 Overview

**MPPI (Model Predictive Path Integral)** is a sampling-based stochastic optimal control algorithm. Unlike traditional MPC (Model Predictive Control), MPPI does not require solving a nonlinear optimization problem. Instead, it approximates the optimal solution through a weighted average of sampled trajectories.

**Key advantages:**
- **Derivative-free**: No gradients of the cost function required
- **Handles non-smooth costs**: Can use discontinuous cost functions (e.g., collision penalties)
- **Parallelizable**: K trajectories can be evaluated in parallel (GPU-friendly)
- **Robust**: Explores multiple solutions simultaneously

### 6.2 System Dynamics

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

### 6.3 Cost Function

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

### 6.4 Trajectory Sampling

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

### 6.5 Trajectory Evaluation

For each sample $k$, simulate the complete trajectory:

$$
\mathbf{x}_{t+1}^{(k)} = f(\mathbf{x}_t^{(k)}, \mathbf{u}_t^{(k)})
$$

And compute the total cost:

$$
S^{(k)} = \phi(\mathbf{x}_T^{(k)}) + \sum_{t=0}^{T-1} q(\mathbf{x}_t^{(k)}, \mathbf{u}_t^{(k)})
$$

### 6.6 Weight Computation (Softmin)

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

### 6.7 Optimal Control Update

The optimal control is obtained as a weighted average:

$$
\mathbf{u}_t^* = \sum_{k=1}^{K} w^{(k)} \cdot \mathbf{u}_t^{(k)}
$$

Equivalently:

$$
\mathbf{u}_t^* = \bar{\mathbf{u}}_t + \sum_{k=1}^{K} w^{(k)} \cdot \boldsymbol{\epsilon}_t^{(k)}
$$

### 6.8 Warm Starting

To improve convergence, the control sequence is shifted at each iteration:

$$
\bar{\mathbf{u}}_t^{\text{new}} = \mathbf{u}_{t+1}^{*,\text{old}} \quad \text{for } t = 0, ..., T-2
$$

$$
\bar{\mathbf{u}}_{T-1}^{\text{new}} = \mathbf{u}_{T-1}^{*,\text{old}}
$$

### 6.9 Algorithm Pseudocode

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

### 6.10 Flow Diagram

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

### 6.11 Computational Complexity

- **Time**: $O(K \cdot T \cdot (n + m + |\text{obstacles}|))$
- **Space**: $O(K \cdot T \cdot m)$ for storing control sequences

Where:
- $K$ = number of samples
- $T$ = prediction horizon
- $n$ = state dimension
- $m$ = control dimension
- $|\text{obstacles}|$ = number of obstacle points

### 6.12 Advantages and Limitations

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

### 6.13 Time-Correlated Noise

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

### 6.14 Adaptive Covariance

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

### 6.15 ESDF-Based Obstacle Costs

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

### 6.16 Covariance-Aware Margin Inflation (Option A)

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

### 6.17 MPPI Configuration Parameters

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
    
    // Covariance-aware margin inflation
    bool use_covariance_inflation = true;
    float cov_z_score = 1.64f;      // Risk multiplier
    float cov_inflation_gate = 2.0f;
    float cov_sigma_max_clamp = 0.5f;
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
| `K` | $K$ | 500 | Number of sampled trajectories |
| `T` | $T$ | 20 | Prediction horizon (time steps) |
| `dt` | $\Delta t$ | 0.1 s | Time step for simulation |
| `lambda` | $\lambda$ | 1.0 | Temperature (exploration vs exploitation) |
| `sigma_vx` | $\sigma_{v_x}$ | 200 mm/s | Noise std dev for forward velocity |
| `sigma_vy` | $\sigma_{v_y}$ | 200 mm/s | Noise std dev for lateral velocity |
| `sigma_omega` | $\sigma_\omega$ | 0.3 rad/s | Noise std dev for angular velocity |
| `max_vx` | - | 800 mm/s | Maximum forward velocity |
| `max_vy` | - | 800 mm/s | Maximum lateral velocity |
| `max_omega` | - | 0.8 rad/s | Maximum angular velocity |
| `robot_radius` | $r$ | 250 mm | Robot radius for collision detection |
| `safety_margin` | $d_{\text{margin}}$ | 500 mm | Distance to start penalizing obstacles |
| `obstacle_decay` | $d_{\text{decay}}$ | 200 mm | Exponential decay rate for obstacle cost |
| `goal_tolerance` | - | 300 mm | Distance to consider goal reached |
| `w_path` | $w_p$ | 1.0 | Weight for path following cost |
| `w_obstacle` | $w_o$ | 10.0 | Weight for obstacle avoidance cost |
| `w_goal` | $w_g$ | 0.5 | Weight for goal reaching cost |
| `w_smoothness` | $w_s$ | 0.1 | Weight for control smoothness cost |
| `w_speed` | $w_v$ | 0.05 | Weight for speed maintenance cost |

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

*Document generated: 2026-02-13*
*Component version: Gridder v2.0*

