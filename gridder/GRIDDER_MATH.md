# GRIDDER_MATH: Mathematical Foundations of Grid Mapping

## 1. Introduction

This document describes the mathematical foundations of the Occupancy Grid Map system implemented in the Gridder component. The approach is based on the **Binary Bayes Filter for Static State** algorithm described in the paper "Grid Maps" by Tipaldi and Burgard.

## 2. Grid Map Representation

### 2.1 Fundamental Assumptions

The system relies on three key assumptions:

1. **Binary discretization**: Each cell in the map is either completely free or occupied
2. **Static world**: Cells do not change state by themselves
3. **Independence**: Cells are independent of each other

### 2.2 Occupancy Variable

Each cell `m` is a **binary random variable**:

```
p(m) = probability that the cell is occupied
p(¬m) = 1 - p(m) = probability that the cell is free
```

**Without prior knowledge**: `p(m) = 0.5`

## 3. Binary Bayes Filter for Static State

### 3.1 Problem Statement

Given a set of sensor observations `z₁:t` and robot poses `x₁:t`, we want to estimate the occupancy probability of each cell:

```
p(m | z₁:t, x₁:t)
```

### 3.2 Formula Derivation

Applying Bayes' theorem repeatedly and using cell independence:

**Step 1: Apply Bayes' Theorem**

We start from the basic Bayes' theorem:
```
           p(B | A) · p(A)
p(A | B) = ───────────────
                p(B)
```

We want to compute `p(m | z₁:t)`, i.e., the probability that cell `m` is occupied given all observations up to time `t`.

We apply Bayes with:
- `A = m` (cell occupied)
- `B = z₁:t` (all observations)

```
             p(z₁:t | m) · p(m)
p(m | z₁:t) = ─────────────────
                  p(z₁:t)
```

We separate the observations into `z₁:t = {z₁:t-1, zt}`:
```
              p(zt, z₁:t-1 | m) · p(m)
p(m | z₁:t) = ────────────────────────
                   p(zt, z₁:t-1)
```

Using the chain rule in the numerator:
```
p(zt, z₁:t-1 | m) = p(zt | m, z₁:t-1) · p(z₁:t-1 | m)
```

Substituting and applying Bayes again to `p(z₁:t-1 | m)`:
```
              p(zt | m, z₁:t-1) · p(z₁:t-1 | m) · p(m)
p(m | z₁:t) = ────────────────────────────────────────
                         p(zt, z₁:t-1)
```

Since `p(z₁:t-1 | m) · p(m) = p(m | z₁:t-1) · p(z₁:t-1)`:
```
              p(zt | m, z₁:t-1) · p(m | z₁:t-1) · p(z₁:t-1)
p(m | z₁:t) = ─────────────────────────────────────────────
                            p(zt, z₁:t-1)
```

Simplifying with `p(zt | z₁:t-1) = p(zt, z₁:t-1) / p(z₁:t-1)`:
```
              p(zt | m, z₁:t-1) · p(m | z₁:t-1)
p(m | z₁:t) = ─────────────────────────────────
                       p(zt | z₁:t-1)
```

**Assuming conditional independence** (the current observation only depends on the cell state, not on past observations): `p(zt | m, z₁:t-1) ≈ p(zt | m)`

**Final formula from Step 1:**
```
              p(zt | m) · p(m | z₁:t-1)
p(m | z₁:t) = ─────────────────────────
                    p(zt | z₁:t-1)
```

**Step 2: Compute the odds ratio**

Computing the ratio `p(m | z₁:t) / p(¬m | z₁:t)`:

```
p(m | z₁:t)     p(m | z₁:t-1)     p(m | zt)     p(¬m)
─────────── = ─────────────── · ─────────── · ──────
p(¬m | z₁:t)   p(¬m | z₁:t-1)   p(¬m | zt)    p(m)
```

### 3.3 Log-Odds Notation

The odds ratio is converted to a sum using logarithms:

**Log-Odds Definition:**
```
l(x) = log(p(x) / (1 - p(x)))
```

**Log-Odds Update Formula:**
```
l(m | z₁:t) = l(m | z₁:t-1) + l(m | zt) - l₀
```

Where:
- `l(m | z₁:t)` = posterior log-odds (after observation)
- `l(m | z₁:t-1)` = prior log-odds
- `l(m | zt)` = log-odds from the inverse sensor model
- `l₀` = prior log-odds (usually 0 if p(m) = 0.5)

**Simplification (with uniform prior l₀ = 0):**
```
l_new = l_previous + l_observation
```

## 4. Inverse Sensor Model for LiDAR

### 4.1 Observation Zones

For each laser ray with measured distance `z`:

| Zone | Condition | Result |
|------|-----------|--------|
| **"free"** | Cell distance < z | Cell free (miss) |
| **"occ"** | Cell distance ≈ z | Cell occupied (hit) |
| **"no info"** | Cell distance > z | No information (don't update) |

```
         sensor
            │
            │  "free" zone
            │  (l_free < 0)
            ▼
    ─────────────────────── ← measured point z ("occ" zone, l_occ > 0)
            │
            │  "no info" zone
            │  (no update)
            ▼
```

### 4.2 Model Parameters

In the current implementation:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `l_occ` | +1.4 | Log-odds increment for hit |
| `l_free` | -0.4 | Log-odds decrement for miss |
| `l_min` | -2.0 | Minimum limit (high confidence free) |
| `l_max` | +4.0 | Maximum limit (high confidence occupied) |

**Hit/miss ratio**: 1.4 / 0.4 = **3.5:1**

This means approximately 3-4 "free" observations are needed to counteract one "occupied" observation. This balance allows:
- Fast clearing of false positives when the robot moves through previously occupied areas
- Persistent obstacles that require consistent negative evidence to be cleared

## 5. Code Implementation

### 5.1 Data Structure

Each cell `T` contains:

```cpp
struct T {
    std::uint32_t id;
    bool free = true;        // Current state
    bool visited = false;    // Visit flag
    float cost = 1;          // Cost for planning
    float hits = 0;          // (legacy, not used with log-odds)
    float misses = 0;        // (legacy, not used with log-odds)
    float log_odds = 0.0f;   // Log-odds representation
    QGraphicsRectItem *tile; // Visual representation
};
```

### 5.2 add_hit() Function

```cpp
void Grid::add_hit(const Eigen::Vector2f &p) {
    constexpr float l_occ = 1.4f;    // log-odds for hit
    constexpr float l_min = -2.0f;   // lower limit
    constexpr float l_max = 4.0f;    // upper limit
    
    auto it = fmap.find(point_to_key(p));
    if(it != fmap.end()) {
        auto &v = it->second;
        
        // Log-odds update
        v.log_odds += l_occ;
        v.log_odds = std::clamp(v.log_odds, l_min, l_max);
        
        // Convert to probability
        float prob = 1.0f - 1.0f / (1.0f + std::exp(v.log_odds));
        
        // Update state if threshold exceeded
        if(prob >= params.occupancy_threshold) {
            v.free = false;
            v.cost = params.occupied_cost;
        }
    }
}
```

### 5.3 add_miss() Function

```cpp
void Grid::add_miss(const Eigen::Vector2f &p) {
    constexpr float l_free = -0.4f;  // log-odds for miss
    constexpr float l_min = -2.0f;
    constexpr float l_max = 4.0f;
    
    auto it = fmap.find(point_to_key(p));
    if(it != fmap.end()) {
        auto &v = it->second;
        
        // Log-odds update
        v.log_odds += l_free;
        v.log_odds = std::clamp(v.log_odds, l_min, l_max);
        
        // Convert to probability
        float prob = 1.0f - 1.0f / (1.0f + std::exp(v.log_odds));
        
        // Update state if below threshold
        if(prob < params.occupancy_threshold) {
            v.free = true;
            v.cost = params.free_cost;
        }
    }
}
```

## 6. Log-Odds ↔ Probability Conversion

### 6.1 From Probability to Log-Odds

```cpp
double log_odds(double prob) {
    return log(prob / (1.0 - prob));
}
```

### 6.2 From Log-Odds to Probability

```cpp
double probability(double l) {
    return 1.0 - 1.0 / (1.0 + exp(l));
}
```

### 6.3 Correspondence Table

| log_odds | Occupied Probability |
|----------|---------------------|
| -2.0 | ~12% (very free) |
| -1.0 | ~27% |
| 0.0 | 50% (unknown) |
| +1.0 | ~73% |
| +2.0 | ~88% |
| +3.5 | ~97% (very occupied) |
| +4.0 | ~98% |

## 7. Advantages of the Log-Odds Model

1. **Computational efficiency**: Only additions, no probability products
2. **Numerical stability**: Avoids issues with very small probabilities
3. **Cumulative behavior**: Each observation adds to prior knowledge
4. **Memory**: An obstacle seen many times needs many "misses" to be erased
5. **Noise robustness**: Averaging multiple observations is more robust

## 8. Parameter Tuning

### 8.1 Persistent Obstacles

If obstacles are erased too quickly:
- **Increase** `l_occ` (e.g., 1.5 → 2.0)
- **Reduce** `l_free` (e.g., -0.1 → -0.05)

### 8.2 Fast Clearing

If free areas are not marked quickly enough:
- **Reduce** `l_occ`
- **Increase** `|l_free|`

### 8.3 Sensitivity

The **occupancy threshold** (`occupancy_threshold`) determines when a cell is considered occupied based on the computed probability (typically 0.5-0.65).

## 9. References

- Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Chapters 4.2 and 9.1-9.2.
- Tipaldi, G. D., & Burgard, W. *Robot Mapping: Grid Maps* - Course slides.

## 10. Implementation Notes

### 10.1 Unvisited Cells (Unknown)

Cells that have never been observed maintain `log_odds = 0` (probability 0.5), indicating total uncertainty.

### 10.2 Clamping

The limits `l_min` and `l_max` prevent:
- Extreme log-odds values that could never be updated
- Numerical overflow in the exponential function

### 10.3 Integration with Path Planning

The `cost` field is used for planning algorithms such as A*:
- `cost = free_cost` (typically 1.0) for free cells
- `cost = occupied_cost` (typically a high value) for occupied cells

## 11. Sparse ESDF Implementation (GridESDF)

### 11.1 Motivation

The dense grid implementation has complexity **O(area)** for storage and **O(n × r)** for ray casting updates, where:
- n = number of LiDAR points per scan
- r = average ray length in cells

For large environments (e.g., 50m × 50m with 100mm cells = 250,000 cells), this becomes prohibitively slow.

### 11.2 Sparse Representation

The GridESDF class uses a **sparse representation** inspired by VoxBlox:

```
Dense Grid:  O(width × height) cells stored
Sparse Grid: O(obstacles) cells stored
```

**Key insight**: In most environments, obstacles occupy only 1-5% of the total area. By storing only obstacle cells, we achieve:
- **Memory reduction**: 20-100× less memory
- **Iteration speedup**: Loops only visit obstacle cells

### 11.3 Free Space Convention

```
If cell NOT in map → cell is FREE (implicit)
If cell IN map → cell is OBSTACLE (explicit)
```

This convention eliminates the need to store free cells, which are the vast majority.

### 11.4 Obstacle Confirmation

To filter noise, obstacles require **confirmation** before being displayed:

```cpp
struct ObstacleCell {
    float log_odds;      // Accumulated confidence
    float hits;          // Number of observations
    uint64_t last_seen;  // Timestamp
    QGraphicsRectItem *tile;  // Only created when confirmed
};
```

An obstacle is **confirmed** when:
```
hits >= min_hits_to_confirm (default: 3)
AND
log_odds >= occupancy_threshold (default: 2.5)
```

### 11.5 ESDF (Euclidean Signed Distance Field)

The ESDF provides the distance to the nearest obstacle for any point:

```
d_E(x) = min_{o ∈ Obstacles} ||x - o||
```

**Properties:**
- `d_E(x) = 0` if x is on an obstacle
- `d_E(x) > 0` if x is in free space
- Gradient `∇d_E(x)` points away from nearest obstacle

### 11.6 Lazy ESDF Computation

Instead of computing ESDF for all cells (expensive), we use **lazy evaluation**:

1. **On obstacle update**: Mark ESDF cache as dirty, but don't recompute
2. **On query**: Compute distance on-demand and cache result
3. **On path planning**: Compute ESDF only for cells in the search region

```cpp
float get_distance(const Key &k) {
    // Check cache first
    auto it = esdf_cache_.find(k);
    if (it != esdf_cache_.end())
        return it->second.distance;
    
    // Compute on demand: O(obstacles)
    float min_dist = INFINITY;
    for (const auto &[ok, _] : obstacles_)
        min_dist = min(min_dist, distance(k, ok));
    
    esdf_cache_[k] = {min_dist, ...};
    return min_dist;
}
```

### 11.7 Cost Function from ESDF

The ESDF enables smooth cost gradients for path planning:

```cpp
float get_cost(const Key &k) {
    float d = get_distance(k);
    float ts = tile_size;
    
    if (d <= 0)
        return obstacle_cost;           // Obstacle
    else if (d < 2 * ts)
        return obstacle_cost * exp(-d / ts);  // High cost near obstacles
    else if (d < 4 * ts)
        return 0.25 * obstacle_cost * exp(-(d - 2*ts) / (2*ts));  // Medium cost
    else
        return free_cost;               // Safe distance
}
```

### 11.8 Visualization Optimization

Since the map can contain thousands of obstacles, visualization is optimized:

1. **Obstacle tiles**: Created only when confirmed (not on first observation)
2. **Inflation tiles**: Recreated only when `visualization_dirty_` flag is set
3. **Dirty flag**: Set when obstacles are confirmed or removed

```cpp
void update_visualization(bool show_inflation) {
    if (!visualization_dirty_)
        return;  // Skip if no changes
    
    // Clear and rebuild inflation layers
    // ... only for confirmed obstacles ...
    
    visualization_dirty_ = false;
}
```

### 11.9 Complexity Comparison

| Operation | Dense Grid | Sparse ESDF |
|-----------|------------|-------------|
| Memory | O(area) | O(obstacles) |
| Update per scan | O(n × r) | O(n) |
| Visualization | O(area) | O(obstacles) if dirty |
| Path planning | O(area × log(area)) | O(search_region × log) |

### 11.10 When to Use Each Implementation

| Scenario | Recommended |
|----------|-------------|
| Small environment (<20m²) | Dense (DENSE mode) |
| Medium environment, accuracy needed | Dense + ESDF (DENSE_ESDF mode) |
| Large environment (>50m²) | Sparse (SPARSE_ESDF mode) |
| Real-time requirements (>10Hz) | Sparse (SPARSE_ESDF mode) |
| High obstacle density (>20%) | Dense (DENSE mode) |

## 12. Morphological Post-Processing

### 12.1 Motivation

Raw LiDAR obstacle detections often have:
- **Small holes** within solid walls
- **Isolated noise points** that passed the confirmation threshold
- **Gaps** between segments of the same obstacle

Morphological operations from image processing can address these issues.

### 12.2 Basic Operations

#### 12.2.1 Dilation

Expands obstacles by adding cells at the boundary:

```
Before:       After Dilation:
  ■ ■           ▢ ▢ ▢ ▢
  ■ ■           ▢ ■ ■ ▢
                ▢ ■ ■ ▢
                ▢ ▢ ▢ ▢
```

**Rule**: Add cell if it has ≥ N obstacle neighbors (N typically 1-3)

#### 12.2.2 Erosion

Shrinks obstacles by removing cells at the boundary:

```
Before:       After Erosion:
▢ ▢ ▢ ▢         
▢ ■ ■ ▢           ■ ■
▢ ■ ■ ▢           ■ ■
▢ ▢ ▢ ▢         
```

**Rule**: Remove cell if it has < M obstacle neighbors (M typically 4-6)

### 12.3 Compound Operations

#### 12.3.1 Closing (Dilation → Erosion)

**Effect**: Fills small holes and narrow gaps

```cpp
void morphological_close(int iterations = 1);
```

```
Before:           After Closing:
■ ■ · ■ ■         ■ ■ ■ ■ ■
■ · · · ■    →    ■ ■ ■ ■ ■
■ ■ · ■ ■         ■ ■ ■ ■ ■
```

**Use case**: Filling holes in walls caused by sensor noise

#### 12.3.2 Opening (Erosion → Dilation)

**Effect**: Removes isolated points and thin protrusions

```cpp
void morphological_open(int iterations = 1);
```

```
Before:           After Opening:
· · ■ · ·         · · · · ·
■ ■ ■ ■ ■    →    · ■ ■ ■ ·
· · ■ · ·         · · · · ·
```

**Use case**: Removing noise points that accumulated enough hits to be confirmed

### 12.4 Gap Filling

A specialized operation to bridge nearby obstacles:

```cpp
void fill_obstacle_gaps();
```

**Algorithm**:
1. For each obstacle cell, check if there's a gap of 1 cell to another obstacle
2. If yes, fill the gap

```
Before:           After:
■ ■ · ■ ■         ■ ■ ■ ■ ■
```

### 12.5 Neighbor Threshold Selection

The threshold for morphological operations affects aggressiveness:

| Dilation Threshold | Effect |
|-------------------|--------|
| ≥ 1 neighbor | Very aggressive, fills large areas |
| ≥ 3 neighbors | Moderate, fills corners |
| ≥ 5 neighbors | Conservative, only fills small holes |

| Erosion Threshold | Effect |
|------------------|--------|
| < 2 neighbors | Removes isolated points only |
| < 4 neighbors | Removes thin lines |
| < 6 neighbors | Aggressive, shrinks all edges |

### 12.6 Implementation Notes

The current implementation uses **threshold = 5** for closing, which:
- Fills single-cell holes surrounded by obstacles
- Doesn't aggressively expand obstacle boundaries
- Preserves the general shape of obstacles

```cpp
// In morphological_close():
if (obstacle_neighbors >= 5)
    to_add.push_back(c);
```

### 12.7 When to Apply

Post-processing should be applied **periodically**, not every frame:

```cpp
// Example: apply every 10 updates
static int update_count = 0;
if (++update_count % 10 == 0) {
    grid_esdf.morphological_close(1);
    grid_esdf.fill_obstacle_gaps();
}
```

### 12.8 Visualization Layers

The inflation layers are drawn as concentric rings around obstacles:

```
Layer Structure:
  2 2 2 2 2
  2 1 1 1 2
  2 1 ■ 1 2
  2 1 1 1 2
  2 2 2 2 2

■ = Obstacle (red)
1 = Layer 1 (orange) - immediate neighbors
2 = Layer 2 (green) - second ring
```

The layers are computed using set operations:
1. Collect all confirmed obstacle cells into `obstacle_set`
2. `layer1_set` = neighbors(obstacle_set) - obstacle_set
3. `layer2_set` = neighbors(layer1_set) - obstacle_set - layer1_set

This ensures continuous, uniform layers without gaps.

