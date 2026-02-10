# Gridder - 2D Occupancy Grid with ESDF-based Path Planning

A RoboComp component that creates and maintains a 2D occupancy grid from LiDAR sensor data, providing efficient path planning capabilities using Euclidean Signed Distance Fields (ESDF).

## Features

- **Sparse ESDF Grid**: VoxBlox-style implementation that only stores obstacle cells, making it memory-efficient for large environments
- **Real-time LiDAR Integration**: Processes point clouds from multiple LiDAR sensors
- **A* Path Planning**: With ESDF-based cost function for smooth, collision-free paths
- **Safety Factor**: Configurable parameter (0-1) to control how close paths get to obstacles
- **World Coordinates**: Grid operates in absolute world coordinates with robot pose tracking
- **Dynamic Resizing**: Grid automatically expands to accommodate new obstacles
- **Network Interface**: ICE-based interface for remote path planning queries

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Gridder                               │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │ LiDAR Thread│───▶│  Grid ESDF  │───▶│  Visualizer │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                  │                                 │
│         ▼                  ▼                                 │
│  ┌─────────────┐    ┌─────────────┐                         │
│  │ Robot Pose  │    │ Path Planner│                         │
│  │   Buffer    │    │    (A*)     │                         │
│  └─────────────┘    └─────────────┘                         │
├─────────────────────────────────────────────────────────────┤
│                    ICE Interface                             │
│  getPaths() | getMap() | LineOfSightToTarget() | ...        │
└─────────────────────────────────────────────────────────────┘
```

## Grid Modes

The component supports three grid modes:

| Mode | Description | Use Case |
|------|-------------|----------|
| `SPARSE_ESDF` | Only stores obstacles (VoxBlox-style) | Large environments, memory-constrained |
| `DENSE_ESDF` | Full grid with ESDF optimization | Medium environments, fast queries |
| `DENSE` | Original ray-casting approach | Small environments, high accuracy |

## Path Planning

### Safety Factor

The `safety_factor` parameter (0.0 - 1.0) controls path behavior:

- **0.0**: Shortest path, may touch obstacle boundaries (green cells)
- **0.5**: Balanced between distance and safety
- **1.0**: Maximum safety, strongly prefers center of free space

```
safety_factor = 0.0          safety_factor = 1.0
                             
    ████████                     ████████
    █      █                     █      █
  ──█──────█──  (short path)     █  ──  █  (safe path)
    █      █                     █ /  \ █
    ████████                     ████/──\████
```

### Cost Function

The A* planner uses an ESDF-based cost:

```
cost = move_cost + esdf_cost × safety_factor × safety_scale

where:
  - move_cost = tile_size (or tile_size × √2 for diagonal)
  - esdf_cost = distance-based penalty near obstacles
  - safety_scale = 1 + safety_factor² × 10  (range 1-11x)
```

## Obstacle Detection

Obstacles are confirmed using a log-odds approach:

1. **Hit**: `log_odds += 1.0` (LiDAR hit on cell)
2. **Miss**: `log_odds -= 0.4` (LiDAR ray passes through)
3. **Threshold**: Cell becomes obstacle when `log_odds >= 2.0` AND `hits >= 3` AND has neighbor support

### Inflation Layers

```
┌───┬───┬───┬───┬───┐
│   │ G │ G │ G │   │   G = Green (cost: 100)
├───┼───┼───┼───┼───┤   O = Orange (cost: 200)  
│ G │ O │ O │ O │ G │   R = Red (cost: 255, obstacle)
├───┼───┼───┼───┼───┤
│ G │ O │ R │ O │ G │
├───┼───┼───┼───┼───┤
│ G │ O │ O │ O │ G │
├───┼───┼───┼───┼───┤
│   │ G │ G │ G │   │
└───┴───┴───┴───┴───┘
```

## ICE Interface

### Methods

| Method | Description |
|--------|-------------|
| `getPaths(source, target, maxPaths, tryClosestFreePoint, targetIsHuman, safetyFactor)` | Compute paths between two points |
| `getMap()` | Get serialized map (obstacles + inflation) |
| `LineOfSightToTarget(source, target, robotRadius)` | Check if direct path is free |
| `IsPathBlocked(path)` | Check if existing path is blocked |
| `getClosestFreePoint(source)` | Find nearest free cell to a point |
| `getDimensions()` | Get grid dimensions |
| `setGridDimensions(dimensions)` | Set grid dimensions |

### Map Serialization

The `getMap()` method returns a compact representation:

```idsl
struct TCell {
    int x;      // mm - cell x position
    int y;      // mm - cell y position  
    byte cost;  // 0=free, 255=obstacle, intermediate=inflation
};

struct Map {
    int tileSize;       // mm - size of each cell (default: 100)
    TCellVector cells;  // only cells with cost > 0
};
```

**Transmission size**: ~9 bytes per cell (only non-free cells transmitted)

## Configuration

Key parameters in `specificworker.h`:

```cpp
struct Params {
    float TILE_SIZE = 100;           // mm - grid resolution
    float ROBOT_WIDTH = 460;         // mm
    float ROBOT_LENGTH = 480;        // mm
    float MAX_LIDAR_RANGE = 15000;   // mm
    float SAFETY_FACTOR = 0.5f;      // 0=fast, 1=safe
    GridMode GRID_MODE = GridMode::SPARSE_ESDF;
};
```

## Dependencies

- Qt6 (Widgets, OpenGL)
- Eigen3
- OpenCV
- Ice (ZeroC)
- RoboComp core libraries

## Building

```bash
cd gridder
cmake -B build
make -C build -j$(nproc)
```

## Running

```bash
# Copy and edit config
cp etc/config etc/myconfig
# Edit myconfig with your LiDAR and robot proxy endpoints

# Run
bin/gridder etc/myconfig
```

## UI Controls

- **Left Click**: Set path target
- **Right Click**: Cancel current path
- **Mouse Wheel**: Zoom
- **Right Drag**: Pan view

## Performance

| Metric | SPARSE_ESDF | DENSE |
|--------|-------------|-------|
| Memory (10m×10m) | ~50KB | ~1MB |
| Update rate | 10-15 Hz | 5-10 Hz |
| Path planning | 1-5 ms | 5-20 ms |

## References

- [VoxBlox: Incremental 3D Euclidean Signed Distance Fields](https://github.com/ethz-asl/voxblox)
- [Log-odds Occupancy Grids](https://en.wikipedia.org/wiki/Occupancy_grid_mapping)

### Documentation Files

| File | Description |
|------|-------------|
| [GRIDDER_MATH.md](GRIDDER_MATH.md) | Mathematical derivation of log-odds occupancy grid update formula |
| [ESDF_IMPLEMENTATION.md](ESDF_IMPLEMENTATION.md) | VoxBlox-style ESDF implementation details and algorithms |
| [slam10-gridmaps.md](slam10-gridmaps.md) | Reference material on probabilistic grid mapping (based on Thrun's SLAM course) |
| [slam10-gridmaps.pdf](slam10-gridmaps.pdf) | Original PDF slides on grid mapping |

---

# Developer Notes

## Editable Files

- `src/*` – Component logic and implementation
- `etc/*` – Configuration files
- `README.md` – This documentation

**Do not edit** files in `generated/` - they are auto-generated.

## State Machine

The component uses RoboComp's state machine:

1. **Initialize**: Setup grids, viewer, LiDAR thread
2. **Compute**: Main loop - read LiDAR, update grid, handle requests
3. **Emergency**: Handle error conditions
4. **Restore**: Recovery from emergency

## Adding New Features

1. Grid operations: Edit `src/grid_esdf.cpp`
2. Path planning: Edit `compute_path()` in `grid_esdf.cpp`
3. Interface methods: Edit `src/specificworker.cpp`
4. Visualization: Edit `update_visualization()` in `grid_esdf.cpp`

## Debugging

Enable debug output in `compute_path()`:
```cpp
qDebug() << "[A*] Computing path..." << "safety:" << safety_factor;
```

View LiDAR points:
```cpp
params.DRAW_LIDAR_POINTS = true;
```
