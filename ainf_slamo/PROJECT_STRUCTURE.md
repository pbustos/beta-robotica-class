# AINF_SLAMO — Project Structure

Active Inference SLAM and Navigation component for RoboComp.  
Uses PyTorch/LibTorch (C++) for SDF-based room and object fitting, Qt6 for GUI, Qt3D for 3D visualization, and Ice for middleware communication.

---

## 1. Source Files (`src/`)

### Core Component

| File | Lines | Description |
|------|------:|-------------|
| `specificworker.h` | 363 | Main component class declaration; owns all subsystems (viewers, AI, managers, scene graph) |
| `specificworker.cpp` | 1541 | Core `compute()` loop, initialization, ICE interface callbacks, room capture, UI wiring |
| `specificworker_visualization.cpp` | 393 | `update_ui()` — refreshes status bar, LCD numbers, navigation state display |
| `specificworker_layout_io.cpp` | 633 | Save/load room layouts (SVG, JSON, USDA); polygon parsing; scene graph serialization |
| `specificworker_grounding.cpp` | 923 | Object grounding: ownership EM visuals, camera wireframe overlay, mesh SDF fitting, EM validation |
| `specificworker_depth.cpp` | 74 | `extract_depth_buffer_meters()` — depth image decoding from ImageSegmentation proxy |
| `specificworker_episode_metrics.cpp` | 337 | Episode lifecycle: `start_episode()`, `update_episode_metrics()`, `finish_episode()` |
| `specificworker_navigation.cpp` | 31 | `slot_new_target()` (delegates to NavigationManager), `send_velocity_command()` |
| `specificworker_sensing.cpp` | 274 | GT error display, sensor data comparison (estimated vs. Webots ground truth) |

> **Convention:** `specificworker_*.cpp` files are **partial compilation units** — they `#include "specificworker.h"` and define methods of `SpecificWorker`. They are compiled as separate TUs but all belong to the same class.

### Extracted Managers

| File | Lines | Description |
|------|------:|-------------|
| `navigation_manager.h/cpp` | 156 / 776 | **`NavigationManager`** — owns path planner, trajectory controller, current path, temp obstacles, and safeguard state. `compute_step()` runs MPPI control + safeguard recovery + obstacle avoidance + object heading alignment each cycle. |
| `em_manager.h/cpp` | 96 / 1775 | **`EMManager`** — Expectation-Maximisation camera validator: frustum culling with wall-occlusion filter, analytic + mesh SDF fitting, accept/reject UI. Also manages ownership EM. |
| `layout_manager.h/cpp` | 73 / 128 | **`LayoutManager`** — owns room polygon, furniture polygon list, and obstacle polygon helpers. Single source of truth for layout geometry. |

### Scene Graph (Model–View architecture)

| File | Lines | Description |
|------|------:|-------------|
| `scene_graph_model.h/cpp` | 124 / 726 | **`SceneGraphModel`** — authoritative tree model: Room → Floor → Objects + Walls. Serializes to JSON and USDA. Emits Qt signals on mutation. |
| `scene_graph_adapter.h/cpp` | 41 / 96 | **`SceneGraphAdapter`** — static converter between `FurniturePolygonData` ↔ `SceneGraphObject`. Bridges legacy polygon data to the graph model. |
| `scene_tree_panel.h/cpp` | 107 / 425 | **`SceneTreePanel`** — live editable QTreeWidget displaying the scene graph hierarchy. Shows per-object SDF score. Emits `objectPropertyEdited` / `removeObjectRequested`. |
| `object_palette_panel.h/cpp` | 62 / 134 | **`ObjectPalettePanel`** — UI panel to pick object types (table, chair, bench, pot) and insert them into the scene. |
| `object_footprints.h` | ~140 | Header-only 2D footprint polygon generators per furniture type (`table`, `chair`, `bench`, `pot`). Produces world-space `Vector2f` polygons from `(tx, ty, yaw, w, d)`. Authoritative 2D representation used by Viewer2D and Viewer3D. |

### Viewers

| File | Lines | Description |
|------|------:|-------------|
| `viewer_2d.h/cpp` | 171 / 553 | **`Viewer2D`** — QGraphicsScene-based 2D top-down view: robot, room polygon, furniture, covariance ellipse, paths, LiDAR points, obstacles. |
| `viewer_3d.h/cpp` | 301 / 1855 | **`Viewer3D`** — Qt3D-based 3D view: extruded walls, floor, robot box, procedural furniture (`QCuboidMesh` compositions), obstacles. Includes `WebotsStyleCameraController` for orbit/pan/zoom. |
| `camera_viewer.h/cpp` | 141 / 1158 | **`CameraViewer`** — popup QDialog for RGB/depth frame display from ImageSegmentation proxy. Supports wireframe overlays, annotations, and a raw-mode toggle that disables all processing for monitoring. |

### Room / Localization (LibTorch)

| File | Lines | Description |
|------|------:|-------------|
| `room_concept_ai.h/cpp` | 307 / 1237 | **`RoomConceptAI`** — SDF-based active inference for joint robot-room state estimation. Adam optimizer on 5 parameters `[width, length, x, y, φ]`. Runs in a dedicated thread. |
| `room_model.h/cpp` | 120 / 243 | **`Model`** (`torch::nn::Module`) — differentiable room SDF (box or polygon mode). Supports EKF-style prediction and prior loss. |

### Object Fitting (LibTorch)

| File | Lines | Description |
|------|------:|-------------|
| `mesh_sdf_optimizer.h/cpp` | 60 / 402 | **`MeshSDFOptimizer`** — gradient-based mesh-to-pointcloud fit using SDF; wall penalty; MC sampling option. |
| `object_ownership_em.h/cpp` | 92 / 278 | **`ObjectOwnershipEM`** — EM algorithm for point-to-object ownership assignment with Charbonnier robust kernel. |

#### Analytic Object Models (`src/object_models/`)

All models use LibTorch autograd for SDF-based 6-DOF fitting. Params: `[tx, ty, yaw, width, depth, height]`.

Each analytic SDF mirrors its visual counterpart (3D model + 2D footprint) so that the point-cloud fitting optimizes against the same geometric composition the user sees.

| File | Lines | Description |
|------|------:|-------------|
| `table_analytic_model.h/cpp` | 76 / 211 | **`TableAnalyticModel`** — composite SDF: tabletop slab + 4 leg cuboids. |
| `chair_analytic_model.h/cpp` | 88 / 189 | **`ChairAnalyticModel`** — composite SDF: seat slab + backrest at rear (-Y) + 4 legs. |
| `bench_analytic_model.h/cpp` | 78 / 161 | **`BenchAnalyticModel`** — composite SDF: seat slab + backrest at rear (-Y) + 2 stout side supports at ±X (park bench). |
| `pot_analytic_model.h/cpp` | 77 / 126 | **`PotAnalyticModel`** — cylinder SDF; 5 params (no depth, uses radius instead). |

### Navigation & Control

| File | Lines | Description |
|------|------:|-------------|
| `polygon_path_planner.h/cpp` | 113 / 594 | **`PolygonPathPlanner`** — visibility-graph + Dijkstra path planner inside a simple polygon with obstacle avoidance (Minkowski expansion). |
| `trajectory_controller.h/cpp` | 404 / 1945 | **`TrajectoryController`** — MPPI-based local controller with ESDF cost. Mood-modulated parameters. Warm-start + AR(1) Gaussian sampling. |

### Sensing & Memory

| File | Lines | Description |
|------|------:|-------------|
| `pointcloud_center_estimator.h/cpp` | 65 / 300 | **`PointcloudCenterEstimator`** — robust room center/OBB estimation from LiDAR sectors with statistical outlier removal. |
| `episodic_memory.h/cpp` | 123 / 367 | **`EpisodicMemory`** — records navigation episodes (target, trajectory stats, safety summary, controller config) to JSON files. |

### Shared Types

| File | Lines | Description |
|------|------:|-------------|
| `common_types.h` | 43 | `rc::VelocityCommand`, `rc::OdometryReading`, `rc::RoomState` — lightweight shared structs. |
| `furniture_types.h` | 24 | `rc::FurniturePolygonData` — legacy polygon representation for furniture items. |
| `object_geometry.h` | 95 | Header-only 2D geometry helpers: `centroid_of()`, `polygon_obb()`, `footprint_for_type()`. Used by 3D viewer, grounding, and EM manager. |

### UI Definition

| File | Description |
|------|-------------|
| `mainUI.ui` | Qt Designer form: main window layout with status LCDs, buttons, and placeholder frames for viewers. |

### Archive / Unused (`src/sources/`)

Earlier versions of subsystems, kept for reference but **not compiled** in the current build:

| File | Lines | Description |
|------|------:|-------------|
| `mppi_controller.h/cpp` | 342 / 979 | Earlier MPPI controller implementation |
| `room_concept.h/cpp` | 258 / 1005 | Earlier room concept estimator |
| `room_model.h/cpp` | 186 / 371 | Earlier room model (torch module) |
| `room_freezing_manager.h/cpp` | 231 / 362 | Room state freezing manager |
| `room_loss.h/cpp` | 33 / 16 | Room loss function |
| `room_thread.h/cpp` | 137 / 179 | Room estimation thread wrapper |
| `pointcloud_center_estimator.h/cpp` | 67 / 296 | Earlier center estimator |
| `concept_room.py` | — | Python prototype of room concept |
| `unified_controller.py` | — | Python prototype of controller |

---

## 2. Key Data Structures

| Struct / Class | Defined In | Purpose |
|----------------|-----------|---------|
| `rc::FurniturePolygonData` | `furniture_types.h` | Legacy polygon representation: id, label, 2D vertices, SDF score, yaw, height |
| `rc::SceneGraphObject` | `scene_graph_model.h` | Scene graph–compatible object: same fields as `FurniturePolygonData` |
| `rc::SceneGraphModel::Node` | `scene_graph_model.h` | Tree node: id, label, type, object_type, translation, yaw, extents, local_vertices, children |
| `rc::VelocityCommand` | `common_types.h` | Robot velocity command (adv_x, adv_y, rot) with timestamp |
| `rc::OdometryReading` | `common_types.h` | Measured encoder/IMU odometry (adv, side, rot) with timestamp |
| `rc::Model` | `room_model.h` | `torch::nn::Module` — differentiable room SDF with EKF prediction |
| `rc::MeshSDFOptimizer::Result` | `mesh_sdf_optimizer.h` | Optimization result: fitted vertices, translation, losses, iterations |
| `rc::ObjectOwnershipEM::ClassState` | `object_ownership_em.h` | Per-object state: position, yaw, scale, confidence |
| `rc::TrajectoryController::Params` | `trajectory_controller.h` | MPPI parameters: mood, kinematic limits, safety, carrot, sampling |
| `NavigationManager::StepResult` | `navigation_manager.h` | Per-cycle result: velocity command, goal/align/blocked flags, ESS, controller output |
| `NavigationManager::Context` | `navigation_manager.h` | Callback wiring: layout manager, loc state, velocity, draw path, draw obstacles |
| `NavigationManager::TempObstacle` | `navigation_manager.h` | Dynamic obstacle data: polygon vertices, center, creation time, replan count, height |
| `EMManager::Context` | `em_manager.h` | Callback wiring: camera viewer, viewers, furniture, room polygon, scene graph, labels |
| `rc::EpisodicMemory::EpisodeRecord` | `episodic_memory.h` | Full episode: target, source, context, trajectory/safety summaries, outcome |
| `ObjectPalettePanel::TypeDef` | `object_palette_panel.h` | Object type definition: label, default width/depth/height |

---

## 3. Key Classes and Their Files

| Class | Header | Implementation |
|-------|--------|---------------|
| `SpecificWorker` | `specificworker.h` | `specificworker.cpp` + 7 `specificworker_*.cpp` partials |
| `rc::Viewer2D` | `viewer_2d.h` | `viewer_2d.cpp` |
| `rc::Viewer3D` | `viewer_3d.h` | `viewer_3d.cpp` |
| `rc::WebotsStyleCameraController` | `viewer_3d.h` | `viewer_3d.cpp` |
| `CameraViewer` | `camera_viewer.h` | `camera_viewer.cpp` |
| `rc::SceneGraphModel` | `scene_graph_model.h` | `scene_graph_model.cpp` |
| `rc::SceneGraphAdapter` | `scene_graph_adapter.h` | `scene_graph_adapter.cpp` |
| `SceneTreePanel` | `scene_tree_panel.h` | `scene_tree_panel.cpp` |
| `ObjectPalettePanel` | `object_palette_panel.h` | `object_palette_panel.cpp` |
| `rc::RoomConceptAI` | `room_concept_ai.h` | `room_concept_ai.cpp` |
| `rc::Model` | `room_model.h` | `room_model.cpp` |
| `rc::MeshSDFOptimizer` | `mesh_sdf_optimizer.h` | `mesh_sdf_optimizer.cpp` |
| `rc::ObjectOwnershipEM` | `object_ownership_em.h` | `object_ownership_em.cpp` |
| `NavigationManager` | `navigation_manager.h` | `navigation_manager.cpp` |
| `EMManager` | `em_manager.h` | `em_manager.cpp` |
| `LayoutManager` | `layout_manager.h` | `layout_manager.cpp` |
| `rc::PolygonPathPlanner` | `polygon_path_planner.h` | `polygon_path_planner.cpp` |
| `rc::TrajectoryController` | `trajectory_controller.h` | `trajectory_controller.cpp` |
| `rc::PointcloudCenterEstimator` | `pointcloud_center_estimator.h` | `pointcloud_center_estimator.cpp` |
| `rc::EpisodicMemory` | `episodic_memory.h` | `episodic_memory.cpp` |
| `rc::object_models::TableAnalyticModel` | `object_models/table_analytic_model.h` | `object_models/table_analytic_model.cpp` |
| `rc::object_models::ChairAnalyticModel` | `object_models/chair_analytic_model.h` | `object_models/chair_analytic_model.cpp` |
| `rc::object_models::BenchAnalyticModel` | `object_models/bench_analytic_model.h` | `object_models/bench_analytic_model.cpp` |
| `rc::object_models::PotAnalyticModel` | `object_models/pot_analytic_model.h` | `object_models/pot_analytic_model.cpp` |

---

## 4. Data Flow

### Room Localization

```
LiDAR proxy → SpecificWorker::compute()
  → lidar points buffered → localization thread
  → RoomConceptAI::update() → Model::sdf() (LibTorch Adam)
  → UpdateResult {state[5], robot_pose, covariance, room_polygon}
  → UI thread picks up result → updates Viewer2D, Viewer3D, status
```

### Furniture Pipeline

```
USDA file (scene_graph.usda)
  → SceneGraphModel::from_usda()        // parse into Node tree
  → SceneGraphModel::objects()           // extract SceneGraphObject list
  → SceneGraphAdapter::to_furniture()    // convert to FurniturePolygonData[]
  → SpecificWorker::furniture_polygons_  // legacy polygon storage

Per frame:
  furniture_polygons_ → Viewer2D::update_furniture()    // 2D polygons
  furniture_polygons_ → Viewer3D::update_furniture()    // 3D extruded meshes + OBJ models
  SceneGraphModel     → SceneTreePanel::rebuild_from_model()  // tree view

On object edit (tree panel or palette):
  SceneTreePanel::objectPropertyEdited → SpecificWorker::set_object_property()
    → SceneGraphModel::set_object_pose() / set_object_extents()
    → SceneGraphModel::objectChanged signal → SceneTreePanel + Viewer3D update
    → SceneGraphAdapter::to_furniture() → furniture_polygons_ synced
    → save_scene_graph_to_usd()

On SDF fitting:
  LiDAR points → ObjectOwnershipEM (E-step: assign points to objects)
  → *AnalyticModel::fit_autograd() or MeshSDFOptimizer (M-step: fit shape)
  → SceneGraphAdapter::accept_fit_if_improved() → SceneGraphModel updated
  → SceneTreePanel shows per-object SDF score

Camera EM validator (button-triggered):
  depth image → point cloud
  → frustum culling (pinhole projection check)
  → wall-occlusion filter (segment–room-boundary intersection)
  → EM iteration per visible candidate object
  → accept/reject UI with pending adjustments
```

### Navigation

```
User click on 2D view → slot_new_target()
  → NavigationManager::plan_to_target()
    → PolygonPathPlanner::plan() → waypoint path
    → TrajectoryController::set_path()
  → each compute() cycle:
    → NavigationManager::compute_step()
      → MPPI control + safeguard recovery + obstacle avoidance
      → StepResult {velocity, flags, metrics}
    → SpecificWorker applies velocity, updates UI, records episode
  → EpisodicMemory records episode stats
```

---

## 5. Configuration Files

### `etc/config`

ICE middleware configuration:
- **Proxies**: Lidar3D (×2), Webots2Robocomp, OmniRobot, ImageSegmentation
- **Endpoints**: Navigator (tcp:13212), FullPoseEstimationPub, JoystickAdapter (subscription topics)
- **Periods**: Compute=50ms, Emergency=500ms
- **Custom params**: `layout_file`, `use_webots`, `camera_tx/ty/tz`

### `CMakeLists.txt` (root)

Top-level CMake: sets component paths, delegates to `generated/CMakeLists.txt`, installs `etc/config` and `meshes/` directory.

### `src/CMakeLists.txt`

Included by `generated/CMakeLists.txt`. Configures:
- Qt6 (Xml, PrintSupport, 3DCore, 3DRender, 3DExtras)
- LibTorch with CUDA support (nvcc detection, architecture selection)
- Adds all `src/*.cpp` and `src/object_models/*.cpp` as sources

### `generated/CMakeLists.txt`

RoboComp build scaffold:
- C++23 standard (g++ ≥ 12)
- ICE IDSL → ICE → C++ code generation for all interfaces
- Links RoboComp classes (rapplication, ConfigLoader, etc.)
- AutoUIC for `mainUI.ui`

---

## 6. Generated Files (`generated/`)

Auto-generated by RoboComp toolchain from `.idsl` interface definitions:

| File | Purpose |
|------|---------|
| `main.cpp` | Application entry point (RoboComp bootstrap) |
| `genericworker.h/cpp` | Base class for SpecificWorker; declares all proxy types, UI form, timer slots |
| `CameraRGBDSimple.h/cpp` | ICE stub for camera interface |
| `FullPoseEstimation.h/cpp` | ICE stub for pose estimation |
| `FullPoseEstimationPub.h/cpp` | ICE stub for pose pub/sub |
| `GenericBase.h/cpp` | ICE stub for generic base types |
| `Gridder.h/cpp` | ICE stub for grid interface |
| `ImageSegmentation.h/cpp` | ICE stub for image segmentation |
| `JoystickAdapter.h/cpp` | ICE stub for joystick input |
| `Lidar3D.h/cpp` | ICE stub for 3D LiDAR |
| `Navigator.h/cpp` | ICE stub for navigator interface |
| `OmniRobot.h/cpp` | ICE stub for omnidirectional robot |
| `Webots2Robocomp.h/cpp` | ICE stub for Webots simulator bridge |
| `navigatorI.h/cpp` | ICE servant for Navigator (incoming calls) |
| `fullposeestimationpubI.h/cpp` | ICE servant for FullPoseEstimationPub subscription |
| `joystickadapterI.h/cpp` | ICE servant for JoystickAdapter subscription |

---

## 7. Other Notable Files

| Path | Description |
|------|-------------|
| `meshes/*.obj` | 3D mesh assets: table, chair, bench, plant_pot, monitor, shadow, tree |
| `scene_graph.usda` | Persisted scene graph in Universal Scene Description (ASCII) format |
| `last_pose.json` | Last robot pose for fast restart |
| `hall_layout.json`, `room_layout.json` | Sample room layouts |
| `episodic_memory/ep_*.json` | Recorded navigation episodes |
| `ainf_slamo.cdsl` | RoboComp component definition (interfaces, language, GUI flag) |
| `mainUI.ui` | Qt Designer form for the main window |
| `bin/ainf_slamo` | Compiled executable |

---

## 8. Total Line Count Summary

| Category | Files | Lines |
|----------|------:|------:|
| SpecificWorker (all partials) | 9 | ~4,696 |
| Extracted Managers (nav, EM, layout) | 6 | ~3,004 |
| Scene Graph (model + adapter + panels) | 8 | ~1,715 |
| Viewers (2D + 3D + camera) | 6 | ~4,306 |
| Room AI + Model | 4 | ~1,907 |
| Object Models (analytic) | 8 | ~1,206 |
| Object Fitting (mesh SDF + EM) | 4 | ~832 |
| Navigation + Control | 4 | ~3,056 |
| Sensing + Memory | 4 | ~855 |
| Shared Types | 3 | ~91 |
| **Active src/ total** | **~56** | **~21,499** |
