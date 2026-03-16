#pragma once

#include <QImage>
#include <QColor>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <optional>
#include "furniture_types.h"

namespace rc {

/**
 * SyntheticCameraRenderer — renders a wireframe overlay of the known model
 * (furniture, walls, floor) as seen from the robot's camera.
 *
 * The renderer uses the same pinhole camera model as the real camera and
 * produces a transparent QImage that can be alpha-composited over the RGB
 * stream in CameraViewer.
 *
 * All geometry is projected via software rasterisation with QPainter.
 */
class SyntheticCameraRenderer
{
public:
    struct CameraIntrinsics
    {
        float fx = 460.f;
        float fy = 460.f;
        float cx = 320.f;
        float cy = 320.f;
        int   width  = 640;
        int   height = 640;
    };

    struct CameraExtrinsics
    {
        float tx = 0.f;   // camera offset in robot frame (lateral)
        float ty = 0.f;   // camera offset in robot frame (forward)
        float tz = 0.92f; // camera height
    };

    /// The rendered overlay plus per-object visibility metadata.
    struct RenderResult
    {
        QImage overlay;   // ARGB32_Premultiplied with transparent background

        /// Indices into the furniture_polygons vector of objects that are visible.
        std::vector<int> visible_furniture_indices;
        /// Projected screen area (pixels²) for each visible object, same order.
        std::vector<float> visible_areas;
    };

    void set_intrinsics(const CameraIntrinsics& intr) { intr_ = intr; }
    void set_extrinsics(const CameraExtrinsics& extr) { extr_ = extr; }
    void set_wall_height(float h) { wall_height_ = h; }

    /// Render the overlay for the current robot pose + model.
    RenderResult render(const Eigen::Affine2f& robot_pose,
                        const std::vector<FurniturePolygonData>& furniture,
                        const std::vector<Eigen::Vector2f>& room_polygon) const;

    // --- Visual style knobs ---
    QColor wall_color    {100, 180, 255, 100};   // translucent blue
    QColor floor_color   {100, 180, 255,  40};   // very faint blue fill
    QColor furniture_color {255, 220, 50, 200};  // yellow wireframe
    QColor selected_color  {50, 255, 100, 230};  // green for selected (future use)

private:
    CameraIntrinsics intr_;
    CameraExtrinsics extr_;
    float wall_height_ = 2.5f;

    /// Convert a world-frame 2D point + height to camera frame.
    Eigen::Vector3f world_to_camera(const Eigen::Vector2f& p_robot, float z_world) const;

    /// Project camera-frame point to pixel coordinates. Returns false if behind camera.
    bool project(const Eigen::Vector3f& p_cam, float& u, float& v) const;

    /// Draw a clipped 3D line segment onto the painter.
    bool draw_segment_3d(QPainter& painter,
                         const Eigen::Vector3f& a_cam,
                         const Eigen::Vector3f& b_cam) const;

    /// Draw the wireframe for one furniture item. Returns projected screen area.
    float draw_furniture_wireframe(QPainter& painter,
                                   const FurniturePolygonData& fp,
                                   const Eigen::Affine2f& world_to_robot,
                                   const QColor& color) const;

    /// Draw walls as wireframe quads.
    void draw_walls(QPainter& painter,
                    const std::vector<Eigen::Vector2f>& room_polygon,
                    const Eigen::Affine2f& world_to_robot) const;

    /// Draw floor outline.
    void draw_floor(QPainter& painter,
                    const std::vector<Eigen::Vector2f>& room_polygon,
                    const Eigen::Affine2f& world_to_robot) const;
};

} // namespace rc
