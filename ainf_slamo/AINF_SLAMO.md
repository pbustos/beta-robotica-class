# AINF_SLAMO

Current Project State Documentation  
Updated: March 2026

---

## 1. What This Component Is

AINF_SLAMO is a RoboComp component that combines:

- Active Inference based robot localization in a known room.
- Local navigation using a path planner plus MPPI trajectory control.
- 2D and 3D visualization tools.
- Camera-based object grounding and EM-based ownership/fit refinement.
- Episodic mission logging for post-run analysis.

It is currently designed for simulated workflows (Webots + LiDAR + segmentation), with switches to disable Webots-specific dependencies when moving to real robot setups.

---

## 2. Current Build and Runtime Stack

- Language: C++ (modern standard, C++23 toolchain in project build flow).
- Build system: CMake + Make.
- GUI: Qt6 (Widgets, Xml, PrintSupport, optional Qt3D modules).
- Math/linear algebra: Eigen.
- Learning/optimization backend: LibTorch (Torch C++ API).
- Middleware: RoboComp + Ice interfaces.
- Optional acceleration: CUDA (currently enabled in build configuration).

Current validated build command:

```bash
cmake --fresh -S . -B build_make -G "Unix Makefiles" && make -C build_make -j20
```

The project is currently building successfully with this command.

---

## 3. Runtime Architecture (Current)

### 3.1 Main execution loops

- `compute()` loop at 20 Hz (`Period.Compute = 50 ms`).
- Background LiDAR read thread.
- Background localization thread (`RoomConceptAI`).

### 3.2 Main processing chain

1. Read synchronized sensors (GT optional, high LiDAR, low LiDAR).
2. Read latest localization output from the localization thread.
3. Update overlays and visual outputs (2D, 3D, camera viewer).
4. Run ownership/EM step when enabled.
5. If a path is active, run trajectory control.
6. Handle obstacle recovery and replan if blocked.
7. If `goto_object` reaches path goal, run dedicated final heading alignment stage.

---

## 4. Source Layout (Current)

The codebase is now split into thematic files instead of a single monolithic worker implementation.

### Core orchestrator

- `src/specificworker.cpp`
- `src/specificworker.h`

### Split worker modules

- `src/specificworker_navigation.cpp`
- `src/specificworker_visualization.cpp`
- `src/specificworker_sensing.cpp`
- `src/specificworker_grounding.cpp`
- `src/specificworker_layout_io.cpp`
- `src/specificworker_depth.cpp`
- `src/specificworker_episode_metrics.cpp`

### Navigation and control

- `src/polygon_path_planner.cpp`
- `src/trajectory_controller.cpp`

### Localization and model

- `src/room_concept_ai.cpp`
- `src/room_model.cpp`

### EM / object ownership / scene graph

- `src/object_ownership_em.cpp`
- `src/scene_graph_model.cpp`
- `src/scene_graph_adapter.cpp`
- `src/object_models/table_analytic_model.cpp`

### Visualization and UI

- `src/viewer_3d.cpp`
- `src/camera_viewer.cpp`
- `src/scene_tree_panel.cpp`

### Memory and metrics

- `src/episodic_memory.cpp`

---

## 5. Localization (Current Behavior)

Localization is handled by `RoomConceptAI` using a room SDF-based objective and variational-style prior terms.

Current implemented characteristics:

- Robot state is optimized in room coordinates.
- Dual priors are supported (command-based and measured odometry-based).
- Robust loss handling for LiDAR residuals.
- Optional early exit if prediction already fits sufficiently.
- Covariance and innovation tracking for health estimation.
- Kidnapping initialization support via grid search.

The localization module runs in its own thread and publishes the latest `UpdateResult` consumed by `compute()`.

---

## 6. Navigation and Control (Current Behavior)

### 6.1 Path planning

- Visibility-graph style polygon planner with room and obstacle constraints.
- Static obstacles from furniture polygons.
- Temporary obstacles generated from online LiDAR clustering when blockages are detected.

### 6.2 Local control

- MPPI trajectory controller is the main mode.
- PD mode exists as selectable alternative.
- Safety guard logic and blocked-path detection are active.
- Recovery behavior includes controlled reverse and optional replan.

### 6.3 `goto_object` final orientation

Current implementation uses a dedicated post-path alignment state:

- When path goal is reached for `goto_object`, trajectory tracking is stopped.
- Final facing alignment runs in a decoupled stage.
- Object center is recomputed when possible and transformed world-to-robot each cycle.
- Yaw error in robot frame:

$$
e_\theta = \operatorname{atan2}(x_{obj}^{robot},\; y_{obj}^{robot})
$$

- Exit criteria include angle tolerance, distance threshold, cycle timeout, and anti-oscillation sign-flip guard.

This is the current mitigation for endless-spin behavior previously seen at object arrival.

---

## 7. EM / Grounding / Model Update Workflow (Current)

The camera-grounding and EM flow is now explicit and operator gated.

Current behavior:

- Camera viewer can trigger EM reassignment.
- Overlay shows phase-dependent information (wireframe + labels + fit evolution).
- EM fitting candidates are queued as pending adjustments.
- Model updates are deferred until explicit user decision.
- UI provides `Accept Fit` and `Reject Fit` controls.
- Accept applies pending adjustments to scene graph/furniture state.
- Reject discards pending updates.

Current implementation also contains a temporary explicit-accept fallback path in model application logic to ease validation during development.

---

## 8. Visualization System (Current)

### 8.1 2D viewer

- Robot, room polygon, furniture polygons, path, debug overlays.
- Interactive target setting and mission cancellation actions.

### 8.2 3D viewer

- Integrated in splitter layout with 2D view and scene panel.
- Supports object picking, floor picking, and mission cancel signaling.
- Renders room/furniture/robot and LiDAR overlays.

### 8.3 Camera viewer

- RGB and depth tabs.
- EM trigger + Accept/Reject controls.
- Wireframe and point overlays.
- Infrastructure masking context support.

---

## 9. Persistence and Data Artifacts (Current)

The component persists operational state and episodic data:

- Last robot pose (`last_pose.json`).
- Viewer camera state (QSettings keys).
- Episodic mission logs in `episodic_memory/` as JSON files.
- Layout files (`room_layout.json`, `hall_layout.json`, etc.).
- Scene graph persistence hooks are active in current integration.

---

## 10. Configuration (Current Default)

Main runtime config file: `etc/config`.

Current notable parameters:

- `Period.Compute = 50`
- `Period.Emergency = 500`
- `layout_file = "completo_unified.svg"`
- `use_webots = true`
- Camera extrinsics: `camera_tx`, `camera_ty`, `camera_tz`
- Proxy and endpoint declarations for LiDAR, OmniRobot, ImageSegmentation, and Navigator.

---

## 11. Controls and Operator Interaction (Current)

Current supported interactions include:

- 2D/3D target selection for navigation.
- Object pick in 3D to trigger object-oriented navigation.
- Mission cancellation from UI interaction paths.
- Camera popup, depth toggle, and EM workflow actions.
- Trajectory mode and behavior tuning through UI controls (including mood parameter).

---

## 12. Build and Execution Quick Start

```bash
# Configure + build
cmake --fresh -S . -B build_make -G "Unix Makefiles"
make -C build_make -j20

# Run
bin/ainf_slamo etc/config
```

---

## 13. Current Status Summary

As of March 2026, the project is in an actively evolving but build-stable state.

Stable and integrated:

- Multi-file architecture split.
- Active localization + planner + MPPI runtime loop.
- 2D/3D visualization integration.
- Camera grounding and EM decision workflow.
- Episodic logging infrastructure.

Actively tuned / under iterative refinement:

- Final `goto_object` orientation robustness in edge cases.
- EM acceptance quality thresholds and fit validation heuristics.
- Runtime behavior polish for blocked-path and recovery transitions.

---

## 14. Notes for Contributors

- Prefer editing files in `src/` and `etc/`.
- Avoid manual edits in generated artifacts.
- Rebuild with the validated `build_make` workflow.
- Keep runtime changes synchronized with this document when behavior changes (especially navigation completion and EM acceptance flow).
