#pragma once


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
#include <Qt3DRender/QObjectPicker>
#include <Qt3DRender/QPickEvent>
#include <QLabel>
#include <QTimer>
#include <QVector3D>
#include <Eigen/Dense>
#include <vector>

namespace rc 
{

/**
 * @brief Camera controller that matches Webots 3D-window navigation.
 *
 * | Gesture                   | Action                        |
 * |---------------------------|-------------------------------|
 * | Left drag                 | Orbit around current pivot    |
 * | Right drag                | Pan (translate camera + pivot)|
 * | Middle drag or Left+Right | Zoom (vertical) + Roll (horiz)|
 * | Scroll wheel              | Zoom (dolly)                  |
 *
 * Left-click on an entity updates the orbit pivot (camera view centre)
 * to the picked world intersection point, reproducing Webots' click-to-orbit feel.
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

    Q_SIGNALS:
        /// Emitted when the user Shift+Right-clicks a named 3D object.
        void objectPicked(const QString& name);
        /// Emitted when the user Shift+Right-clicks the floor.
        void floorPicked(float x, float y);
        /// Emitted when the user Ctrl+Right-clicks an object.
        void cancelMissionRequested();

    public:
        /// 3D camera state (position, view-centre, up-vector).
        struct CameraState
        {
            QVector3D position   {0.f, 15.f, 15.f};
            QVector3D viewCenter {0.f,  0.f,  0.f};
            QVector3D upVector   {0.f,  1.f,  0.f};
        };
        /// Read the current camera pose.
        CameraState camera_state() const;
        /// Apply a previously saved camera pose.
        void set_camera_state(const CameraState& s);

        /// Rebuild wall and floor entities from a new room polygon.
        /// Safe to call any time the room polygon changes.
        void rebuild_walls(const std::vector<Eigen::Vector2f>& poly);

        /// Called every visualization frame to update the robot entity.
        void update_robot_pose(float x, float y, float theta_rad);

        /// Update path visualization as a ribbon.
        void update_path(const std::vector<Eigen::Vector2f>& path);

        /// Update all furniture mesh entities to match the current layout.
        /// Each item is {inkscape_label, centroid_in_room_frame}.
        /// Label matching (case-insensitive substring):
        ///   "table"        → meshes/table.obj
        ///   "bench"        → meshes/bench.obj
        ///   "plant"/"pot"  → meshes/plant_pot.obj
        struct FurnitureItem {
            std::string label;
            Eigen::Vector2f centroid;  // (x, z) in room coords
            Eigen::Vector2f size;      // oriented footprint extents (local_x, local_z)
            float yaw_rad = 0.f;       // orientation around vertical axis
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
         * @brief Update segmented object 3D points.
         * Input points are in room/layout frame (x,y horizontal, z height).
         */
        void update_segmented_points(const std::vector<Eigen::Vector3f>& points_layout);

        /**
         * @brief One oriented 3D bounding box for a segmented object.
         * Center is in room/layout frame; yaw rotates around vertical axis.
         */
        struct SegmentedBoxItem {
            std::string label;
            Eigen::Vector2f center;   // (x,y) in room coords
            float center_height = 0.f;
            Eigen::Vector3f size{0.2f, 0.2f, 0.2f}; // (x,y,z) extents in box local frame
            float yaw_rad = 0.f;
        };
        void update_segmented_boxes(const std::vector<SegmentedBoxItem>& boxes);

        /**
         * @brief Update all temporary obstacle mesh entities.
         * Each item carries the 2-D polygon (room frame, meters) and the estimated
         * height derived from the LiDAR Z range.  height == 0 means "use default".
         */
        struct ObstacleItem {
            std::vector<Eigen::Vector2f> vertices;
            float height = 0.f;   ///< metres; 0 → fall back to wall_height_ * 0.85
        };
        void update_obstacles(const std::vector<ObstacleItem>& obstacles);

    private:
        /// (Re)build the floor plane to match the bounding box of poly.
        void build_floor(const std::vector<Eigen::Vector2f>& poly);

        /// Attach a QObjectPicker to entity e with the given display name.
        /// On Shift+Right-click the pick_label_ overlay fires for 3 s.
        void attach_picker(Qt3DCore::QEntity* e, const QString& name);

        /// Compute simple centroid of a polygon.
        static Eigen::Vector2f polygon_centroid(const std::vector<Eigen::Vector2f>& poly);

        // ---- Qt3D scene graph ----
        Qt3DExtras::Qt3DWindow*  window_          = nullptr;
        QWidget*                 container_       = nullptr;
        Qt3DCore::QEntity*       root_entity_     = nullptr;
        Qt3DRender::QCamera*     camera_          = nullptr;   ///< cached camera pointer for state I/O

        // Robot
        Qt3DCore::QTransform*    robot_transform_ = nullptr;
        QQuaternion              robot_base_rot_;   ///< mesh-space correction applied before heading

        // Room geometry
        Qt3DCore::QEntity*       floor_entity_    = nullptr;
        std::vector<Qt3DCore::QEntity*> wall_entities_;

        // Temp obstacles
        std::vector<Qt3DCore::QEntity*> obstacle_entities_;

        // Segmented object 3D oriented boxes (+ text labels)
        std::vector<Qt3DCore::QEntity*> segmented_box_entities_;

        // Furniture meshes (one entity per SVG furniture path)
        std::vector<Qt3DCore::QEntity*> furniture_entities_;

        // Lidar point-cloud pool (pre-allocated spheres, repositioned each frame)
        static constexpr int kLidarPoolHigh = 500;
        static constexpr int kLidarPoolLow  = 250;
        static constexpr int kSegmentedPool = 2000;
        std::vector<Qt3DCore::QTransform*> lidar_hi_xf_;
        std::vector<Qt3DCore::QTransform*> lidar_lo_xf_;
        std::vector<Qt3DCore::QTransform*> segmented_xf_;
        void init_lidar_pool();

        // Trajectory path
        Qt3DCore::QEntity*             path_entity_   = nullptr;
        Qt3DCore::QGeometry*           path_geo_      = nullptr;
        Qt3DRender::QGeometryRenderer* path_renderer_ = nullptr;
        Qt3DCore::QBuffer*             path_buf_      = nullptr;
        Qt3DCore::QAttribute*          path_attr_     = nullptr;
        void init_path_entity();

        // Hover-label overlay for object picking
        QLabel*  pick_label_ = nullptr;
        QTimer*  pick_timer_ = nullptr;

        // Parameters
        float wall_height_   = 2.5f;
        float robot_half_h_  = 0.15f;  ///< half-height of robot box (m)
    };

} // namespace rc

