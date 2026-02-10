# Grid Implementation Options for Gridder

## Overview

The gridder component now supports **three different grid implementations**, each with different trade-offs between accuracy, memory usage, and performance.

## Grid Modes

### 1. DENSE (Original Ray Casting)

**File:** `grid.cpp` / `grid.h`

**Method:** `update_map()` + `update_costs()`

**Characteristics:**
- Full dense grid with all cells allocated
- Ray casting from robot to each LiDAR point
- Most accurate occupancy representation
- Slowest for large maps

**Complexity:** O(n × r) where n = LiDAR points, r = average ray length in cells

**Use case:** Small environments, high accuracy requirements

### 2. DENSE_ESDF (Dense Grid with ESDF Optimization)

**File:** `grid.cpp` / `grid.h`

**Method:** `update_map_esdf()` + `update_costs_esdf()`

**Characteristics:**
- Full dense grid (same memory as DENSE)
- Sparse ray sampling (every 2-3 cells instead of every cell)
- ESDF-based cost computation (faster inflation)
- Good balance between accuracy and speed

**Complexity:** O(n + m log m) where m = affected cells

**Use case:** Medium-large environments, good accuracy with better performance

### 3. SPARSE_ESDF (VoxBlox-style Sparse Grid)

**File:** `grid_esdf.cpp` / `grid_esdf.h`

**Method:** `GridESDF::update()` + `GridESDF::update_visualization()`

**Characteristics:**
- **Only stores obstacle cells** (huge memory savings)
- Free space is implicit (not stored)
- ESDF computed incrementally via Dijkstra multi-source
- Fastest for large environments
- Built-in A* path planning with ESDF costs

**Memory:** O(obstacles) instead of O(area)

**Use case:** Large environments, real-time requirements, memory constraints

## Configuration

In `specificworker.h`, set the grid mode:

```cpp
// In Params struct:
enum class GridMode { DENSE, DENSE_ESDF, SPARSE_ESDF };
GridMode GRID_MODE = GridMode::SPARSE_ESDF;  // Choose mode here
```

## Performance Comparison

| Mode | Memory | Update Speed | Accuracy |
|------|--------|--------------|----------|
| DENSE | High (O(area)) | Slow | Highest |
| DENSE_ESDF | High (O(area)) | Medium | High |
| SPARSE_ESDF | Low (O(obstacles)) | Fast | Good |

### Typical Performance (15m LiDAR, 100mm cells, 2000 points/scan)

| Mode | Cells Visited | Expected FPS |
|------|---------------|--------------|
| DENSE | ~300,000 | 2-5 Hz |
| DENSE_ESDF | ~10,000 | 10-15 Hz |
| SPARSE_ESDF | ~3,000 | 20+ Hz |

## Architecture

```
specificworker.cpp
├── Grid grid;           // Dense implementations (DENSE, DENSE_ESDF)
└── GridESDF grid_esdf;  // Sparse implementation (SPARSE_ESDF)

compute()
└── switch (params.GRID_MODE)
    ├── DENSE:       grid.update_map() + grid.update_costs()
    ├── DENSE_ESDF:  grid.update_map_esdf() + grid.update_costs_esdf()
    └── SPARSE_ESDF: grid_esdf.update() + grid_esdf.update_visualization()
```

## GridESDF API

```cpp
class GridESDF {
    // Update from LiDAR
    void update(points, robot_pos, max_range, timestamp);
    
    // Queries
    bool is_obstacle(key);
    bool is_free(key);          // Implicit: not in map = free
    float get_distance(key);    // ESDF distance to nearest obstacle
    float get_cost(key);        // Cost for path planning
    
    // Path planning
    vector<Vector2f> compute_path(source, target);
    bool is_line_of_sight_free(source, target, robot_radius);
    
    // Statistics
    size_t num_obstacles();     // Number of stored cells
};
```

## References

- Oleynikova, Helen, et al. "Voxblox: Incremental 3D Euclidean Signed Distance Fields." IROS 2017.
- Felzenszwalb & Huttenlocher. "Distance Transforms of Sampled Functions." ToC 2012.
