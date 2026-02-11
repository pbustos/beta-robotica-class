# MRPT Map Loader - Complete Reference

## Overview

The Gridder component can load pre-computed occupancy grids from MRPT `.gridmap` files, enabling rapid initialization without real-time exploration.

## Quick Start

```cpp
// In specificworker.cpp initialize()
std::string map_file = "my_map.gridmap";
if (Gridder_loadMRPTMap(map_file)) {
    external_map_loaded = true;  // Disables dynamic LiDAR updates
}
```

## API Methods

| Method | Description |
|--------|-------------|
| `Gridder_loadMRPTMap(filepath)` | Load map into existing grid |
| `Gridder_loadAndInitializeMap(filepath)` | Load and auto-adjust grid dimensions |

## MRPT File Format

### Native Format (COccupancyGridMap2D)

Files may be **zstd compressed** (auto-detected).

```
Header (62 bytes):
[1 byte]   0x9F (indicator)
[31 bytes] "mrpt::maps::COccupancyGridMap2D"
[2 bytes]  Version
[uint32]   width, height (cells)
[float32]  x_min, x_max, y_min, y_max (meters)
[float32]  resolution (meters/cell)

Data:
[width×height bytes] Occupancy values (0-255)
```

### Occupancy Values (0-255)

| Value | Meaning | Action |
|-------|---------|--------|
| `0` | Unknown/unexplored | Ignored |
| `1-127` | Free space | Ignored (sparse) |
| `>127` | Occupied/obstacle | Added to grid |

Typical distribution: ~73% unknown, ~22% free, ~5% obstacles.

## Configuration Parameters

In `specificworker.h` → `struct Params`:

```cpp
// Grid mode (SPARSE_ESDF recommended for large maps)
GridMode GRID_MODE = GridMode::SPARSE_ESDF;

// Map alignment with Webots/simulation world
float MRPT_MAP_OFFSET_X = 0.f;      // mm - X translation
float MRPT_MAP_OFFSET_Y = 0.f;      // mm - Y translation  
float MRPT_MAP_ROTATION = 0.f;      // radians - rotation (M_PI_2 = 90° left)
```

### Transform Order
1. Rotate around origin
2. Apply translation offset

```cpp
x' = x * cos(θ) - y * sin(θ) + offset_x
y' = x * sin(θ) + y * cos(θ) + offset_y
```

## Coordinate Alignment

To align MRPT map with simulation:

1. **Find robot position** in simulation: `(Wx, Wy)`
2. **Find corresponding point** in MRPT map origin
3. **Set offsets**: 
   ```cpp
   MRPT_MAP_OFFSET_X = Wx;  // mm
   MRPT_MAP_OFFSET_Y = Wy;  // mm
   ```
4. **If rotated**, set `MRPT_MAP_ROTATION` (e.g., `M_PI_2` for 90° left)

## Behavior with External Maps

When `external_map_loaded = true`:
- LiDAR updates are **disabled** (no `check_and_resize`, no `grid_esdf.update()`)
- Only visualization is updated
- Path planning uses loaded obstacles

## Performance

| Grid Size | Load Time | Memory (SPARSE_ESDF) |
|-----------|-----------|----------------------|
| 100×100 | <5 ms | ~10 KB |
| 500×500 | ~50 ms | ~50 KB |
| 700×1580 | ~100 ms | ~170 KB |

Memory scales with **obstacle count**, not grid size.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "Unknown file format" | File may be zstd compressed - ensure `zstd` is installed |
| Map not visible | Check `add_confirmed_obstacle()` is used for external maps |
| Wrong position | Adjust `MRPT_MAP_OFFSET_X/Y` parameters |
| Wrong orientation | Adjust `MRPT_MAP_ROTATION` (try `±M_PI_2`) |
| Walls disconnected | Resolution mismatch - TILE_SIZE auto-adjusts to map resolution |

## Code Structure

```
src/mrpt_map_loader.h    - API header
src/mrpt_map_loader.cpp  - Format parsers (native, legacy, YAML)
src/specificworker.cpp   - Integration (Gridder_loadMRPTMap)
src/grid_esdf.cpp        - add_confirmed_obstacle() for external maps
```

## Example: Full Integration

```cpp
// specificworker.h - Parameters
float MRPT_MAP_OFFSET_X = -13677.7f;
float MRPT_MAP_OFFSET_Y = -7171.55f;
float MRPT_MAP_ROTATION = M_PI_2;

// specificworker.cpp - Initialize
void SpecificWorker::initialize() {
    grid_esdf.initialize(params.TILE_SIZE, &viewer->scene);
    
    if (Gridder_loadMRPTMap("mapa2.gridmap")) {
        external_map_loaded = true;
        qInfo() << "Map loaded, LiDAR updates disabled";
    }
}

// specificworker.cpp - Compute (respects external_map_loaded)
void SpecificWorker::compute() {
    if (not external_map_loaded) {
        grid.check_and_resize(points_world);
        grid_esdf.update(points_world, robot_pos, max_range, timestamp);
    }
    grid_esdf.update_visualization(true);
}
```

## Tools

```bash
# Generate test maps
python3 tools/generate_test_maps.py --size small

# Test loader
./tools/test_mrpt_loader.sh
```
