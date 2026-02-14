# Gridder

**Autonomous Navigation Component for RoboComp**

Gridder provides complete autonomous navigation capabilities combining sparse ESDF mapping, Monte Carlo localization (AMCL), and Model Predictive Path Integral (MPPI) control. Designed for both simulation (Webots) and real robot deployment.

## Features

- **Sparse ESDF Mapping**: Memory-efficient occupancy grid using Euclidean Signed Distance Fields
- **AMCL Localization**: Adaptive Monte Carlo Localization with KLD-sampling
- **A* Path Planning**: ESDF-based cost function with configurable safety factor
- **MPPI Controller**: Covariance-aware obstacle avoidance using path integral control
- **Multi-threaded Architecture**: Separate threads for LiDAR, localization, and control (50-20 Hz)
- **Dual Mode Operation**: Seamless switching between simulation and real robot deployment

## Quick Start

### Simulation (Webots)
```bash
bin/gridder etc/config
# Left click to navigate, system auto-initializes
```

### Real Robot
```bash
# 1. Set USE_GT_WARMUP = false in src/specificworker.h
# 2. Recompile: make
# 3. Run: bin/gridder etc/config
# 4. Shift+Click to initialize position
# 5. Navigate with Left Click
```

## Documentation

📘 **[USER MANUAL](USER_MANUAL.md)** - Start here!
- Operating modes (Simulation vs Real Robot)
- Mouse controls and UI
- Step-by-step configuration
- Troubleshooting guide
- Deployment checklist

📚 **[GRIDDER DOCUMENTATION](GRIDDER_DOCUMENTATION.md)** - Complete technical reference
- Sparse ESDF algorithm and implementation
- Hybrid log-odds + TSDF mapping
- Particle filter localization (AMCL)
- MPPI controller with time-correlated noise
- Covariance-aware margin inflation
- Mathematical derivations

📋 **Additional References**
- [GT_POSE_AUDIT.md](GT_POSE_AUDIT.md) - Ground truth usage verification
- [MPPI_MATH.md](MPPI_MATH.md) - MPPI mathematical foundations
- [ESDF_IMPLEMENTATION.md](ESDF_IMPLEMENTATION.md) - ESDF algorithm details

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                         GRIDDER                               │
├──────────────────────────────────────────────────────────────┤
│  Main Thread → LiDAR Thread (50Hz) → Grid ESDF               │
│             ↓  Localizer Thread (20Hz) → Pose + Covariance   │
│             ↓  MPPI Thread (20Hz) → Velocity Commands         │
│             ↓  Path Planner (A*) → Navigation Paths           │
└──────────────────────────────────────────────────────────────┘
```

**See [GRIDDER_DOCUMENTATION.md](GRIDDER_DOCUMENTATION.md) for detailed architecture diagrams.**

## Mouse Controls

| Action | Function |
|--------|----------|
| **Left Click** | Set navigation target |
| **Shift + Left Click** | Initialize robot position (real robot only) |
| **Ctrl + Right Click** | Cancel current target |
| **Right Drag** | Pan view |
| **Wheel** | Zoom |

**See [USER_MANUAL.md](USER_MANUAL.md) for complete control reference.**

## Key Parameters

**File:** `src/specificworker.h` - `Params` struct

```cpp
// Operating Mode
bool USE_GT_WARMUP = true;              // Set false for real robot

// Robot Model
RobotType ROBOT_TYPE = RobotType::OMNIDIRECTIONAL;  // or DIFFERENTIAL

// Grid
float TILE_SIZE = 100;                  // mm - grid resolution
float ROBOT_WIDTH = 460;                // mm

// Localization
bool USE_LOCALIZER = true;
size_t LOCALIZER_PARTICLES = 500;
float LOCALIZER_ODOM_NOISE = 0.1f;
int LOCALIZER_PERIOD_MS = 50;           // 20 Hz

// Navigation
float SAFETY_FACTOR = 1.0f;             // 0=shortest, 1=safest
int MPPI_PERIOD_MS = 50;                // 20 Hz
```

**See [GRIDDER_DOCUMENTATION.md](GRIDDER_DOCUMENTATION.md) Section 9 for complete parameter reference.**

## Building

```bash
cd gridder
cmake .
make -j$(nproc)
```

## Running

```bash
bin/gridder etc/config
```

**Configuration:** Edit `etc/config` to set LiDAR, OmniRobot, and Webots2Robocomp proxy endpoints.

## Dependencies

- **Qt6** (Widgets, OpenGL)
- **Eigen3** (>= 3.3)
- **Ice** (ZeroC ICE middleware)
- **RoboComp** core libraries
- **Optional:** OpenCV (for map visualization)

## ICE Interface

```idsl
interface Gridder {
    Result getPaths(TPoint source, TPoint target, ...);
    Map getMap();
    TPoint getPose();
    bool LineOfSightToTarget(TPoint source, TPoint target, float radius);
    // ... see GRIDDER_DOCUMENTATION.md for complete API
};
```

## Performance

| Metric | Sparse ESDF | Notes |
|--------|-------------|-------|
| Memory (50m × 50m) | ~20-50 KB | Only obstacles stored |
| Grid Update | 10-15 Hz | Real-time LiDAR integration |
| Path Planning | 1-10 ms | Typical A* computation |
| Localization | 20 Hz | AMCL particle filter |
| MPPI Control | 20 Hz | 512 samples, 30-step horizon |

## Version

**Version:** 2.0  
**Date:** 2026-02-14  
**License:** See LICENSE file

## Support

- **Issues:** Report via GitHub Issues
- **Documentation:** See [USER_MANUAL.md](USER_MANUAL.md) and [GRIDDER_DOCUMENTATION.md](GRIDDER_DOCUMENTATION.md)
- **Contact:** RoboComp Team

---

**Getting Started?** → Read [USER_MANUAL.md](USER_MANUAL.md)  
**Want Technical Details?** → Read [GRIDDER_DOCUMENTATION.md](GRIDDER_DOCUMENTATION.md)  
**Deploying on Real Robot?** → Check [USER_MANUAL.md](USER_MANUAL.md) Section 5
