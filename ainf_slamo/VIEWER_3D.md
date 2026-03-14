# 3D Viewer Extension — Design Document

**Status:** Planned  
**Date:** 2026-03-08  

---

## 1. Goal

Add a Qt3D-based 3D view alongside the existing 2D `AbstractGraphicViewer`.  
The room layout polygon is extruded into walls, obstacles are loaded as mesh files (`.obj`/`.stl`), and the robot has its own mesh — all updated in real time from the existing localization data.

---

## 2. Current 2D Architecture (unchanged)

| Component | Implementation |
|-----------|---------------|
| Viewer widget | `AbstractGraphicViewer` → `QGraphicsView` + `QGraphicsScene` (OpenGL-backed) |
| Room boundary | `QPolygonF` outline drawn via `viewer->scene.addPolygon()` |
| Robot | `QPainterPath` silhouette, `setPos()`/`setRotation()` each frame |
| Obstacles | `QPolygonF` items (orange outline + semi-transparent fill) |
| Path / trajectories | `QGraphicsLineItem` segments |
| Layout data | `etc/room_layout.json` — polygon vertices `{x, y}` in meters |

The 2D view remains **untouched**. The 3D view is purely additive.

---

## 3. Qt3D Integration Strategy

### 3.1 Widget embedding

```cpp
// In specificworker.cpp initialize()
auto *view3d = new Qt3DExtras::Qt3DWindow();
auto *container = QWidget::createWindowContainer(view3d, this);
// Place in a QTabWidget tab or QSplitter alongside the 2D viewer
```

### 3.2 Entity tree

```
rootEntity
├── cameraEntity  (QCamera + QOrbitCameraController)
├── lightEntity   (QDirectionalLight or QPointLight)
├── wallsEntity   (custom QGeometry with extruded quads)
├── floorEntity   (QPlaneMesh, optional)
├── robotEntity   (QMesh → robot.obj, QTransform updated each frame)
└── obstaclesEntity
    ├── obs_0     (QMesh → obstacle.obj, QTransform = position + scale)
    ├── obs_1
    └── ...
```

### 3.3 Wall extrusion algorithm

Each edge of the room polygon `(x1,y1) → (x2,y2)` becomes a vertical quad:

```
v0 = (x1, y1, 0)          v2 = (x2, y2, 0)
v1 = (x1, y1, wall_h)     v3 = (x2, y2, wall_h)

Triangle 1: v0, v2, v1
Triangle 2: v1, v2, v3
```

For N polygon vertices → N wall segments → 6N triangle vertices.  
With the current 6-vertex room polygon that's just 36 vertices — trivial.

**Implementation options (pick one):**

| Approach | Pros | Cons |
|----------|------|------|
| Custom `QGeometry` + `QBuffer` | Full control, single draw call | ~80 lines of vertex buffer code |
| One `QCuboidMesh` per edge | Simpler, no raw buffers | Need to compute center/rotation/size per segment |

**Recommended:** Custom `QGeometry` — it's cleaner for arbitrary polygons and extends naturally if the room has more vertices.

### 3.4 Meshes

```cpp
// Robot
auto *robotMesh = new Qt3DRender::QMesh();
robotMesh->setSource(QUrl::fromLocalFile("meshes/robot.obj"));

// Obstacle (reused geometry, different transforms)
auto *obstacleMesh = new Qt3DRender::QMesh();
obstacleMesh->setSource(QUrl::fromLocalFile("meshes/obstacle_box.obj"));
```

Qt3D `QMesh` supports `.obj`, `.stl`, `.ply` natively.  
Scaling is applied via `Qt3DCore::QTransform::setScale3D()` to match the OBB dimensions from the obstacle avoidance system.

### 3.5 Real-time update loop

```cpp
// Called from SpecificWorker::compute() or a dedicated slot
void Viewer3D::update_robot_pose(float x, float y, float theta)
{
    robot_transform_->setTranslation(QVector3D(x, y, robot_z_offset));
    robot_transform_->setRotationZ(qRadiansToDegrees(theta));
}

void Viewer3D::update_obstacles(const std::vector<TempObstacle>& obstacles)
{
    // Add/remove/reposition obstacle entities to match current list
}
```

---

## 4. New Files

| File | Purpose | Est. lines |
|------|---------|-----------|
| `src/viewer_3d.h` | `Viewer3D` class declaration | ~80 |
| `src/viewer_3d.cpp` | Entity tree, wall extrusion, mesh loading, update methods | ~350 |
| `meshes/robot.obj` | Robot 3D model (can start with a simple box/cylinder) | — |
| `meshes/obstacle.obj` | Generic obstacle mesh | — |

---

## 5. CMakeLists.txt Changes

```cmake
find_package(Qt6 COMPONENTS 3DCore 3DRender 3DExtras 3DInput REQUIRED)
# or Qt5 equivalents: Qt53DCore Qt53DRender Qt53DExtras Qt53DInput

target_link_libraries(${PROJECT_NAME}
    ...existing libs...
    Qt::3DCore Qt::3DRender Qt::3DExtras Qt::3DInput
)
```

Add `src/viewer_3d.h` and `src/viewer_3d.cpp` to the sources list.

---

## 6. specificworker Integration Points

### 6.1 Header additions

```cpp
#include "viewer_3d.h"

// Member
Viewer3D *viewer_3d_ = nullptr;
```

### 6.2 initialize() — create and wire

```cpp
viewer_3d_ = new Viewer3D(this, room_polygon_, wall_height);
// Place widget in UI (tab or splitter)
```

### 6.3 compute() — per-frame update

```cpp
viewer_3d_->update_robot_pose(rx, ry, robot_angle);
viewer_3d_->update_obstacles(temp_obstacles_);
// Optionally: viewer_3d_->update_path(current_path_);
```

### 6.4 Layout changes — rebuild walls

```cpp
// In slot_load_layout() or wherever room_polygon_ changes:
viewer_3d_->rebuild_walls(room_polygon_);
```

---

## 7. Camera & Interaction

- **Default:** `Qt3DExtras::QOrbitCameraController` — orbit, zoom, pan with mouse
- **Alternative:** `QFirstPersonCameraController` for FPS-style walk-through
- **Click navigation:** Use `Qt3DRender::QObjectPicker` + `QRayCaster` to detect clicks on the floor plane → emit same `new_mouse_coordinates` signal as the 2D view

---

## 8. Visual Parameters

```cpp
struct Viewer3DParams {
    float wall_height = 2.5f;          // meters
    QColor wall_color{180, 180, 180};  // light gray
    QColor floor_color{240, 240, 240}; // near-white
    float robot_z_offset = 0.0f;       // mesh origin adjustment
    float ambient_intensity = 0.3f;
    float light_intensity = 0.8f;
};
```

---

## 9. Implementation Order

- [ ] **Step 1:** Minimal `Viewer3D` class — empty scene + camera + light + floor plane
- [ ] **Step 2:** Wall extrusion from `room_polygon_` — verify geometry matches 2D view
- [ ] **Step 3:** Robot as a simple `QCuboidMesh` box — update position/rotation from localization
- [ ] **Step 4:** Wire into `specificworker` — tab or splitter layout
- [ ] **Step 5:** Load robot `.obj` mesh (replace placeholder box)
- [ ] **Step 6:** Obstacle entities from `temp_obstacles_` with generic mesh + scaling
- [ ] **Step 7:** Path visualization (extruded ribbon or tube along waypoints)
- [ ] **Step 8:** Trajectory fan visualization (optional — translucent tubes for sampled trajectories)
- [ ] **Step 9:** Click-to-navigate via raycasting (optional)

---

## 10. Key Considerations

1. **No shared rendering** — Qt3D and QGraphicsScene are separate pipelines; only data (poses, polygons) is shared
2. **Performance** — negligible at this scale (~6 wall quads + few meshes)
3. **Qt version** — Qt6 `Qt3D` modules are stable; Qt5 works too but API is slightly different
4. **Coordinate system** — Qt3D uses Y-up by default; the 2D view uses X-right Y-up. Map: 2D `(x, y)` → 3D `(x, y, z)` where z is height. Alternatively use Qt3D's Y-up with 2D `y` → 3D `z`
5. **Thread safety** — Qt3D runs its aspect engine on a separate thread; entity creation/modification must happen on the main thread (same as current code)

---

## 11. RoboComp and Qt3D Frames

This section defines the coordinate frames and the conversion used in this project.

### 11.1 RoboComp frames

- Room/localization frame (2D): `(x, y)` in meters.
- Robot frame (for local points): `(x_r, y_r)` on ground plane, with `z_r` as height.

### 11.2 Qt3D frame

- Qt3D is right-handed and Y-up.
- Axes are `(X_q, Y_q, Z_q)`.
- `Y_q` is height.

### 11.3 Project mapping (room to Qt3D)

This viewer uses an X-flip for visual consistency with the existing scene setup:

- `X_q = -x`
- `Y_q = h`
- `Z_q = y`

Where `(x, y)` is a room point and `h` is its vertical height.

### 11.4 Conversion procedure

1) Convert robot-local ground point to room frame using robot pose:

- `p_room = T_room_from_robot * p_robot_2d`

2) Map room + height to Qt3D:

- `QVector3D(-p_room.x, height, p_room.y)`

### 11.5 Applied examples in code

- Robot pose: room `(x, y)` -> Qt3D `(-x, robot_half_h, y)`.
- Walls/floor/obstacles/furniture: room points -> Qt3D `(-x, 0 or h/2, y)`.
- Lidar points: robot point -> room via pose, then Qt3D `(-x_room, z_lidar, y_room)`.
- Segmented 3D points: use robot ground `(x_r, y_r)`, keep `z_r` as height, then same Qt3D mapping.

### 11.6 Inverse conversion for picking

From Qt3D picked world point back to room coordinates:

- `x = -X_q`
- `y = Z_q`

### 11.7 Yaw / rotation convention

**Standard rotation convention (`object_footprints.h`):**

Both 2D and 3D views compute geometry using the **standard 2D rotation**:

```
world = center + R(yaw) * local
R(yaw) = [[cos θ, −sin θ], [sin θ, cos θ]]
```

- Local X = **width** axis
- Local Y = **depth** axis (facing direction)
- Width (`extents.x`) is measured along local X, depth (`extents.y`) along local Y.

Note: the `SceneGraphModel` internally uses a non-standard axis convention
(`ydir = (cos θ, sin θ)`, `xdir = (sin θ, −cos θ)`) for `local_to_world_polygon()`.
The `object_footprints.h` functions bypass this and use the standard convention
directly, so both 2D polygons and 3D meshes agree exactly.

**Qt3D frame:**

The coordinate mapping `X_q = −x` mirrors the X axis. Combined with the standard rotation convention, the Qt3D Y-axis rotation angle is:

```
α_qt3d = π + θ
```

Derivation: width axis (local X) in standard rotation maps to world `(cos θ, sin θ)`.
In Qt3D: `(−cos θ, 0, sin θ)`.
`Ry(α)` maps `(1,0,0)` → `(cos α, 0, −sin α)`, so `cos α = −cos θ`, `sin α = −sin θ` → `α = π + θ`.

In code:

```cpp
QQuaternion::fromAxisAndAngle(0, 1, 0, qRadiansToDegrees(float(M_PI) + yaw_rad));
```

**Position offsets** for sub-parts use the standard rotation `R(yaw)`:

```cpp
// Standard rotation: world = center + R(yaw) * local
const float wx = centroid.x() + cos(θ) * local_x - sin(θ) * local_y;
const float wy = centroid.y() + sin(θ) * local_x + cos(θ) * local_y;
// Qt3D position:  QVector3D(-wx, center_h, wy)
```

### 11.8 Furniture rendering (procedural)

All furniture is rendered procedurally with `QCuboidMesh` compositions — no `.obj` mesh files are loaded. The authoritative source for each object's centroid, extents (width, depth, height), and yaw is `SceneGraphModel::get_object_node()`. Type-specific builders (`make_table`, `make_chair`, `make_bench`, `make_pot`, `make_monitor`, `make_cabinet`) compose multiple cuboid parts to approximate each shape.
