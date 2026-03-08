#pragma once

#ifdef HAS_QT3D

#include <QObject>
#include <QWidget>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QCamera>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QAbstractCameraController>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Eigen/Dense>
#include <vector>

namespace rc {

/**
 * @brief Camera controller that matches Webots 3D-window navigation.
 *
 * | Gesture                   | Action                        |
 * |---------------------------|-------------------------------|
 * | Left drag                 | Orbit around view centre      |
 * | Middle drag / Alt+Left    | Pan (translate view centre)   |
 * | Right drag (up/down)      | Zoom (dolly)                  |
 * | Scroll wheel              | Zoom (dolly)                  |
 */
class WebotsStyleCameraController : public Qt3DExtras::QAbstractCameraController
{
    Q_OBJECT
public:
    explicit WebotsStyleCameraController(Qt3DCore::QNode *parent = nullptr)
        : Qt3DExtras::QAbstractCameraController(parent) {}

private:
    void moveCamera(const Qt3DExtras::QAbstractCameraController::InputState &state,
                    float dt) override;
};

/**
 * @brief Qt3D-based 3D viewer for the SLAMO component.
 *
 * Displays the room layout as extruded walls, the robot as a solid box
 * (placeholder — swap to QMesh for a .obj model), and temporary obstacles
 * as orange boxes.  Updated in real time from localization data.
 *
 * Coordinate mapping (room 2D → Qt3D 3D):
 *   room  (x, y)  →  Qt3D (x, Y_height, y)   [Qt3D Y axis = up / height]
 *
 * Usage:
 *   viewer_3d_ = new rc::Viewer3D(this, 2.5f);
 *   some_layout->addWidget(viewer_3d_->container_widget());
 *
 *   // per frame:
 *   viewer_3d_->update_robot_pose(x, y, theta_rad);
 *
 *   // when room polygon changes:
 *   viewer_3d_->rebuild_walls(room_polygon_);
 *
 *   // when obstacles change:
 *   viewer_3d_->update_obstacles(obstacle_polygon_list);
 */
class Viewer3D : public QObject
{
    Q_OBJECT
public:
    /**
     * @param parent  Qt parent for the container widget (e.g. the main window).
     * @param wall_height  Height of extruded room walls in meters (default 2.5 m).
     */
    explicit Viewer3D(QWidget* parent, float wall_height = 2.5f);
    ~Viewer3D() override = default;

    /// The QWidget to embed in the application layout.
    QWidget* container_widget() const { return container_; }

    /// Rebuild wall and floor entities from a new room polygon.
    /// Safe to call any time the room polygon changes.
    void rebuild_walls(const std::vector<Eigen::Vector2f>& poly);

    /// Called every visualization frame to update the robot entity.
    void update_robot_pose(float x, float y, float theta_rad);

    /// Update all furniture mesh entities to match the current layout.
    /// Each item is {inkscape_label, centroid_in_room_frame}.
    /// Label matching (case-insensitive substring):
    ///   "table"        → meshes/table.obj
    ///   "bench"        → meshes/bench.obj
    ///   "plant"/"pot"  → meshes/plant_pot.obj
    struct FurnitureItem {
        std::string label;
        Eigen::Vector2f centroid;  // (x, z) in room coords
        Eigen::Vector2f size;      // (width_x, depth_z) AABB in room metres
    };
    void update_furniture(const std::vector<FurnitureItem>& items);

    /**
     * @brief Update lidar point cloud visualisation.
     * Points are in robot frame (x,y = horizontal, z = height).
     * Pass empty vectors to clear the display.
     * @param robot_x/y/theta_rad  Current robot pose in room frame.
     */
    void update_lidar_points(const std::vector<Eigen::Vector3f>& points_high,
                             const std::vector<Eigen::Vector3f>& points_low,
                             float robot_x, float robot_y, float robot_theta_rad);

    /**
     * @brief Update all furniture mesh entities to match the current layout.
     * @param obstacle_polygons  Each inner vector is one obstacle polygon (room frame, meters).
     */
    void update_obstacles(const std::vector<std::vector<Eigen::Vector2f>>& obstacle_polygons);

private:
    /// (Re)build the floor plane to match the bounding box of poly.
    void build_floor(const std::vector<Eigen::Vector2f>& poly);

    /// Compute simple centroid of a polygon.
    static Eigen::Vector2f polygon_centroid(const std::vector<Eigen::Vector2f>& poly);

    // ---- Qt3D scene graph ----
    Qt3DExtras::Qt3DWindow*  window_          = nullptr;
    QWidget*                 container_       = nullptr;
    Qt3DCore::QEntity*       root_entity_     = nullptr;

    // Robot
    Qt3DCore::QTransform*    robot_transform_ = nullptr;
    QQuaternion              robot_base_rot_;   ///< mesh-space correction applied before heading

    // Room geometry
    Qt3DCore::QEntity*       floor_entity_    = nullptr;
    std::vector<Qt3DCore::QEntity*> wall_entities_;

    // Temp obstacles
    std::vector<Qt3DCore::QEntity*> obstacle_entities_;

    // Furniture meshes (one entity per SVG furniture path)
    std::vector<Qt3DCore::QEntity*> furniture_entities_;

    // Lidar point-cloud pool (pre-allocated spheres, repositioned each frame)
    static constexpr int kLidarPoolHigh = 500;
    static constexpr int kLidarPoolLow  = 250;
    std::vector<Qt3DCore::QTransform*> lidar_hi_xf_;
    std::vector<Qt3DCore::QTransform*> lidar_lo_xf_;
    void init_lidar_pool();

    // Parameters
    float wall_height_   = 2.5f;
    float robot_half_h_  = 0.15f;  ///< half-height of robot box (m)
};

} // namespace rc

#endif // HAS_QT3D
