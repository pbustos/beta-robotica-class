# Gridder User Manual

**Component:** Gridder - 2D Mapping, Localization and Navigation  
**Version:** 2.0  
**Date:** 2026-02-14

---

## Table of Contents

1. [Overview](#overview)
2. [Mouse Controls](#mouse-controls)
3. [Operating Modes](#operating-modes)
4. [Simulation Mode (Webots)](#simulation-mode-webots)
5. [Real Robot Mode](#real-robot-mode)
6. [Configuration](#configuration)
7. [Troubleshooting](#troubleshooting)
8. [Performance Optimization](#performance-optimization)

---

## Overview

The **Gridder** component provides autonomous navigation capabilities through:

- **Sparse ESDF Mapping**: Memory-efficient occupancy grid from LiDAR data
- **AMCL Localization**: Particle filter-based pose estimation
- **A* Path Planning**: ESDF-based path planning with safety factor
- **MPPI Control**: Model Predictive Path Integral controller with covariance-aware obstacle avoidance

The component can operate in two modes:
- **Simulation Mode**: With Webots simulator (automatic initialization)
- **Real Robot Mode**: On physical hardware (manual initialization required)

---

## Mouse Controls

The graphical interface provides intuitive mouse controls for navigation and setup:

| Mouse Action | Effect | Use Case |
|--------------|--------|----------|
| **Left Click** | Set navigation target | Tell robot where to go |
| **Shift + Left Click** | Initialize robot position | First-time setup on real robot |
| **Ctrl + Right Click** | Cancel current target | Stop navigation |
| **Right Click + Drag** | Pan view | Move camera around map |
| **Mouse Wheel Up/Down** | Zoom in/out | Adjust view scale |

### Detailed Mouse Actions

#### Set Navigation Target (Left Click)
1. Click anywhere on the map
2. System computes path from current position to target
3. Robot begins autonomous navigation
4. Path shown as blue line on map

#### Initialize Robot Position (Shift + Left Click)
**Only needed when `USE_GT_WARMUP = false` (real robot mode)**

1. Start the component with map loaded
2. Locate robot's approximate position on the map
3. **Shift + Left Click** at that location
4. Localizer initializes with 500 particles
5. Wait 1-3 seconds for convergence
6. Robot ready for navigation

**Notes:**
- Only works **once** - first click initializes, subsequent clicks ignored
- Position doesn't need to be exact (~1 meter accuracy is fine)
- Initial angle assumed 0° - particle filter converges quickly
- Visual feedback: robot polygon moves to clicked position

#### Cancel Navigation (Ctrl + Right Click)
1. **Ctrl + Right Click** anywhere
2. Current navigation target cleared
3. Robot stops (if moving)

---

## Operating Modes

The Gridder component supports two distinct operating modes controlled by the `USE_GT_WARMUP` parameter.

### Comparison Table

| Aspect | Simulation Mode | Real Robot Mode |
|--------|-----------------|-----------------|
| **USE_GT_WARMUP** | `true` (default) | `false` |
| Ground Truth Pose | Available from Webots | Not available |
| Odometry Source | Computed from GT delta | Wheel encoders/IMU |
| Initialization | Automatic | Manual (Shift+Click) |
| Startup Time | Immediate | 1-3 seconds after positioning |
| Webots2Robocomp | Required | Not used |
| Typical Use | Development, testing | Deployment |

---

## Simulation Mode (Webots)

### Quick Start

1. **Start Webots simulation** with robot
2. **Launch required components**:
   ```bash
   # Terminal 1: LiDAR interface
   # Terminal 2: OmniRobot interface
   # Terminal 3: Webots2Robocomp interface
   ```
3. **Start Gridder**:
   ```bash
   cd /path/to/gridder
   bin/gridder etc/config
   ```
4. **System auto-initializes** - ready to use immediately

### Features

- **Automatic initialization**: Localizer uses GT pose for initial particle placement
- **Smooth startup**: No manual intervention needed
- **GT warmup**: First few frames may use GT pose until localizer produces estimates
- **Perfect odometry**: Computed from GT pose deltas (then noise-contaminated)

### Typical Workflow

1. Component starts → Localizer auto-initializes
2. Left click to set target → Robot navigates
3. Monitor localization quality (ESS displayed)
4. Adjust parameters if needed
5. Test different scenarios

---

## Real Robot Mode

### Prerequisites

Before deploying on a real robot, you must:

1. Set `USE_GT_WARMUP = false` in code
2. Remove Webots2Robocomp dependency
3. Update odometry source to real hardware
4. Load external map (MRPT format)

### Configuration Steps

#### 1. Disable GT Warmup

**File:** `src/specificworker.h`

```cpp
// In Params struct, change:
bool USE_GT_WARMUP = false;  // ⚠️ Set to false for real robot
```

#### 2. Update Component Definition

**File:** `gridder.cdsl`

```cdsl
Component gridder
{
    Communications
    {
        implements Gridder;
        // Remove Webots2Robocomp from requirements:
        requires Lidar3D, Lidar3D, OmniRobot;  // Real robot
    };
    language Cpp11;
    gui Qt(QWidget);
};
```

#### 3. Update Configuration File

**File:** `etc/config`

```ini
# Remove Webots proxy:
Proxies.Lidar3D = "lidar3d:tcp -h localhost -p 11990"
Proxies.Lidar3D1 = "lidar3d:tcp -h localhost -p 11989"
# Proxies.Webots2Robocomp = "..."  # Comment out or remove
Proxies.OmniRobot = "omnirobot:tcp -h localhost -p 10004"
```

#### 4. Modify Odometry Source

**File:** `src/specificworker.cpp` - `read_lidar()` thread

Replace GT pose acquisition with real odometry:

```cpp
// BEFORE (Simulation):
const auto &[position, orientation] = 
    webots2robocomp_proxy->getObjectPose("shadow");
Eigen::Affine2f eig_pose;
eig_pose.translation() = Eigen::Vector2f(-position.y, position.x);
// ...

// AFTER (Real Robot):
auto base_state = omnirobot_proxy->getBaseState();
Eigen::Affine2f eig_pose;
eig_pose.translation() = Eigen::Vector2f(base_state.x, base_state.z);
eig_pose.linear() = Eigen::Rotation2Df(base_state.alpha).toRotationMatrix();
```

#### 5. Recompile

```bash
cd /path/to/gridder
cd src
rm CMakeLists.txt  # Force regeneration
cd ..
cmake .
make -j4
```

### Startup Procedure

When you start the component with `USE_GT_WARMUP = false`, you'll see:

```
═══════════════════════════════════════════════════════════════
  REAL ROBOT MODE: GT warmup disabled (USE_GT_WARMUP = false)
═══════════════════════════════════════════════════════════════

  To initialize the localizer, please:
  1. Use Shift+Left Click to place the robot at its
     approximate current position on the map
  2. The localizer will initialize at that position
  3. Wait 1-3 seconds for localization to converge

  Until initialized, the robot will NOT move.
═══════════════════════════════════════════════════════════════
```

**Console messages during startup:**
```
[compute] Waiting for manual robot positioning (Shift+Left Click)...
```

### Manual Initialization Steps

1. **Identify robot's position**: Determine where the robot actually is
2. **Find position on map**: Locate the corresponding spot in the GUI
3. **Shift + Left Click**: Click at that position on the map
4. **Wait for confirmation**:
   ```
   [ROBOT] Manual reposition to: X Y
   [Localizer] Manually initialized at: X Y theta: 0
   ```
5. **Wait for convergence** (1-3 seconds):
   ```
   [compute] Waiting for localizer to provide pose estimate...
   ```
6. **System ready**: Normal operation begins

### Convergence Indicators

Monitor these to know when localization has converged:

- **ESS (Effective Sample Size)**: Should be > 50
  - Displayed periodically: `ESS: 215`
  - Higher is better (max = number of particles)

- **Console messages**: Stop showing "Waiting for..."

- **Robot movement**: Begins responding to navigation commands

- **Covariance ellipse** (if enabled): 
  - Small ellipse = good localization
  - Large ellipse = high uncertainty

### Initial Pose Accuracy

You don't need to be perfectly precise:

| Position Error | Expected Behavior |
|----------------|-------------------|
| < 0.5 m | Converges within 1 second |
| 0.5 - 1.0 m | Converges within 2-3 seconds |
| 1.0 - 2.0 m | May converge within 5-10 seconds |
| > 2.0 m | May not converge (re-initialize) |

**Angle:** Initial angle is assumed 0° - the particle filter will correct this during convergence.

### Alternative: Global Localization

If you don't know the robot's position at all:

**File:** `src/specificworker.cpp` - `run_localizer()`

```cpp
// Instead of manual Shift+Click, use uniform initialization:
if (!localizer_initialized.load())
{
    // Spread particles across entire known map
    localizer.resetUniform(map_bounds, 5000);  // More particles needed
    localizer_initialized.store(true);
}
```

**Trade-offs:**
- ✅ No manual positioning needed
- ✅ Works from any starting position
- ❌ Requires more particles (5000+)
- ❌ Takes longer to converge (10-30 seconds)
- ❌ Higher computational cost

---

## Configuration

### Key Parameters

**File:** `src/specificworker.h` - `Params` struct

#### Localization Parameters

```cpp
bool USE_LOCALIZER = true;           // Enable AMCL localization
bool USE_GT_WARMUP = true;           // false for real robot
size_t LOCALIZER_PARTICLES = 500;    // Number of particles
float LOCALIZER_ODOM_NOISE = 0.1f;   // Odometry noise factor
int LOCALIZER_PERIOD_MS = 50;        // Localizer thread period (20 Hz)
```

#### MPPI Controller Parameters

```cpp
int MPPI_PERIOD_MS = 50;             // MPPI thread period (~20 Hz)
float SAFETY_FACTOR = 1.0f;          // Path safety (0=shortest, 1=safest)
```

#### Map Parameters

```cpp
float MRPT_MAP_OFFSET_X = 12000.0f;  // X offset to align map (mm)
float MRPT_MAP_OFFSET_Y = -2500.0f;  // Y offset to align map (mm)
float MRPT_MAP_ROTATION = -M_PI_2;   // Rotation to align map (rad)
bool MRPT_MAP_MIRROR_X = true;       // Mirror X axis if needed
```

#### Robot Dimensions

```cpp
float ROBOT_WIDTH = 460;             // mm
float ROBOT_LENGTH = 480;            // mm
float ROBOT_SEMI_WIDTH = 230;        // mm (for collision checking)
```

### UI Controls

The interface provides checkboxes to control visualization:

- **Show LiDAR Points**: Display raw LiDAR data (can impact performance)
- **Show Particles**: Display localization particles
- **Show Trajectories**: Display MPPI sampled trajectories (default ON)
- **Show Covariance**: Display pose uncertainty ellipse

---

## Troubleshooting

### "Waiting for manual robot positioning..." loops forever

**Problem:** Localizer not initialized

**Solutions:**
1. Verify you've performed **Shift+Left Click** on the map
2. Check console for confirmation message
3. Ensure map is loaded (`external_map_loaded` should be true)
4. Verify LiDAR data is being received

### Component crashes on startup

**Problem:** Trying to connect to Webots2Robocomp proxy in real robot mode

**Solutions:**
1. Remove Webots2Robocomp from `gridder.cdsl`
2. Remove proxy from `etc/config`
3. Regenerate component: `robocompdsl gridder.cdsl .`
4. Recompile

### Localization never converges

**Problem:** Poor initial estimate or insufficient particles

**Solutions:**
1. **Increase particles**: `LOCALIZER_PARTICLES = 1000`
2. **Re-initialize closer**: Shift+Click at more accurate position
3. **Check odometry**: Verify odometry noise isn't too high
4. **Verify map alignment**: Check MRPT offset/rotation parameters
5. **Inspect LiDAR quality**: Ensure clean, consistent readings

### Robot won't move after initialization

**Problem:** No valid pose estimate or navigation disabled

**Check:**
1. Console shows: `[Localizer Thread] GT error: pos= XXX mm`
2. ESS value is reasonable (> 50)
3. Navigation state is NAVIGATING
4. MPPI enabled: `mppi_enabled` flag
5. Valid path exists

### High localization error

**Problem:** Particles converged to wrong position

**Solutions:**
1. **Re-initialize**: Component restart may be needed
2. **Increase uncertainty**: Use more particles or larger initial spread
3. **Kidnapping detection**: Currently not implemented - restart needed
4. **Check map quality**: Ensure map matches environment

### Poor navigation performance

**Problem:** MPPI not finding good trajectories

**Solutions:**
1. **Adjust safety factor**: Lower for faster, riskier paths
2. **Increase samples**: More MPPI trajectories (computational cost)
3. **Check obstacle costs**: May be too conservative
4. **Verify covariance**: High uncertainty → conservative behavior

---

## Performance Optimization

### For Embedded Systems / Real Robots

Real robots typically have less computational power than development machines:

```cpp
// Reduce computational load:
size_t LOCALIZER_PARTICLES = 300;     // Down from 500
int LOCALIZER_PERIOD_MS = 100;        // 10 Hz instead of 20 Hz
int MPPI_PERIOD_MS = 100;             // 10 Hz instead of 20 Hz

// In MPPI controller:
int K = 256;                          // Down from 512 samples
int T = 15;                           // Down from 30 horizon steps
```

**Trade-offs:**
- ✅ Lower CPU usage
- ✅ Longer battery life
- ❌ Slower localization convergence
- ❌ Less smooth navigation

### For High-Performance Requirements

If you need maximum performance and have computational power:

```cpp
// Maximize performance:
size_t LOCALIZER_PARTICLES = 1000;    // More accurate
int LOCALIZER_PERIOD_MS = 33;         // 30 Hz
int MPPI_PERIOD_MS = 33;              // 30 Hz

// In MPPI controller:
int K = 1024;                         // More samples
int T = 40;                           // Longer horizon
```

**Requirements:**
- Multi-core CPU (4+ cores recommended)
- Sufficient RAM (512 MB+ for component)
- Modern GPU (optional, for future GPU-accelerated MPPI)

### Memory Usage

Typical memory consumption:

| Configuration | RAM Usage | Notes |
|---------------|-----------|-------|
| Minimal (300 particles, K=256) | ~150 MB | Embedded systems |
| Standard (500 particles, K=512) | ~385 MB | Default |
| High-perf (1000 particles, K=1024) | ~800 MB | Development machines |

---

## Deployment Checklist

Use this checklist when deploying on a real robot:

### Pre-Deployment
- [ ] `USE_GT_WARMUP = false` in `src/specificworker.h`
- [ ] Webots2Robocomp removed from `gridder.cdsl`
- [ ] Webots proxy removed from `etc/config`
- [ ] Odometry source updated to `omnirobot_proxy->getBaseState()`
- [ ] External map loaded and aligned
- [ ] Map offset/rotation parameters configured
- [ ] Component recompiled with `make clean && make`

### Startup
- [ ] All required components running (LiDAR, OmniRobot)
- [ ] Gridder starts without errors
- [ ] Startup instructions displayed
- [ ] Map visible in GUI

### Initialization
- [ ] Robot's approximate position known
- [ ] **Shift+Left Click** performed at correct location
- [ ] Confirmation message received
- [ ] Waiting for convergence (1-3 seconds)

### Operation
- [ ] Localization converged (ESS > 50)
- [ ] Navigation commands accepted
- [ ] Robot moves toward targets
- [ ] Obstacle avoidance working
- [ ] Path replanning functional

### Monitoring
- [ ] ESS values reasonable
- [ ] Position error acceptable
- [ ] No crashes or freezes
- [ ] CPU usage acceptable
- [ ] Memory usage stable

---

## Summary

### Simulation Mode (Development)
✅ Set `USE_GT_WARMUP = true`  
✅ Connect Webots2Robocomp  
✅ Start and go - automatic initialization  
✅ Perfect for testing algorithms

### Real Robot Mode (Deployment)
✅ Set `USE_GT_WARMUP = false`  
✅ Remove Webots dependencies  
✅ **Shift+Left Click** to initialize position  
✅ Wait for convergence  
✅ Start navigation

### Mouse Controls Quick Reference
- **Left Click**: Navigate here
- **Shift+Left Click**: I'm here (initialization)
- **Ctrl+Right Click**: Cancel target
- **Right Drag**: Pan view
- **Wheel**: Zoom

---

**For additional technical details, see:**
- `GRIDDER_DOCUMENTATION.md` - Complete technical reference
- `GT_POSE_AUDIT.md` - Ground truth pose usage details
- Source code documentation in header files

**Version:** 2.0  
**Last Updated:** 2026-02-14  
**Author:** RoboComp Team

