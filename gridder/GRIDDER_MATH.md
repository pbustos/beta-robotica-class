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
