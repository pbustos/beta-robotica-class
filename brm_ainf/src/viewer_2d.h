#pragma once

#include <QObject>
#include <QWidget>
#include <QRectF>
#include <QTransform>
#include <QPointF>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <Eigen/Dense>
#include <vector>

#include "furniture_types.h"

class AbstractGraphicViewer;

namespace rc {

/**
 * @brief 2D scene viewer for the SLAMO component.
 *
 * Wraps AbstractGraphicViewer and owns all QGraphicsItem* scene objects:
 * robot covariance ellipse, room polygon, furniture polygons, path lines,
 * trajectory debug overlays, and temporary obstacle polygons.
 *
 * All drawing is performed via typed update methods so that SpecificWorker
 * never touches QGraphicsScene directly.
 *
 * Usage:
 *   viewer_2d_ = std::make_unique<rc::Viewer2D>(this->frame, grid_rect, true);
 *   viewer_2d_->add_robot(w, l, 0.0, 0.2, QColor("Blue"));
 *   viewer_2d_->show();
 *   layout->addWidget(viewer_2d_->get_widget());
 *
 *   // per frame:
 *   viewer_2d_->update_robot(x, y, angle_rad);
 */
class Viewer2D : public QObject
{
    Q_OBJECT
public:
    explicit Viewer2D(QWidget* parent, const QRectF& grid_dim, bool show_axis = true);
    ~Viewer2D() override = default;

    // ----- Widget / view management -----
    QWidget* get_widget() const;
    void add_robot(float w, float l, float offset, float rotation, QColor color);
    void show();
    QTransform transform() const;
    void set_transform(const QTransform& t);
    void fit_to_scene(const QRectF& r);
    void center_on(float x, float y);

    // ----- Robot -----
    void update_robot(float x, float y, float angle_rad);

    // ----- Covariance ellipse -----
    void update_covariance_ellipse(float cx, float cy,
                                   float radius_x, float radius_y,
                                   float angle_deg);

    // ----- Estimated room rect (shown when room polygon is absent) -----
    void update_estimated_room_rect(float width, float length, bool has_polygon);

    // ----- Room polygon capture vertices (yellow dots) -----
    void add_capture_vertex(QPointF pos);
    void clear_capture_vertices();

    // ----- Room polygon outline -----
    void draw_room_polygon(const std::vector<Eigen::Vector2f>& verts, bool is_capturing);
    void clear_room_polygon();
    /// Move current polygon item into backup (used when starting a new capture).
    void save_polygon_to_backup();
    /// Restore polygon from backup and discard any current polygon.
    void restore_polygon_from_backup();

    // ----- Furniture polygons -----
    void draw_furniture(const std::vector<rc::FurniturePolygonData>& fps);
    void update_furniture_item(std::size_t idx, const rc::FurniturePolygonData& fp);
    /// Override pen/brush for one furniture item (used by EM ownership visuals).
    void set_furniture_item_style(std::size_t idx, QPen pen, QBrush brush);
    std::size_t furniture_count() const;

    // ----- Temporary obstacles -----
    /// Rebuild temp-obstacle polygons from world-frame vertex lists.
    void draw_temp_obstacles(const std::vector<std::vector<Eigen::Vector2f>>& polys);

    // ----- Path -----
    struct PathDrawData
    {
        std::vector<Eigen::Vector2f> path;
        std::vector<Eigen::Vector2f> orig_poly_verts;
        std::vector<Eigen::Vector2f> inner_poly;
        std::vector<Eigen::Vector2f> nav_poly;
        std::vector<std::vector<Eigen::Vector2f>> expanded_obstacles;
    };
    void draw_path(const PathDrawData& data);
    /// Remove path lines / waypoints / expanded-obstacle overlays; hide target marker.
    void clear_path_items();

    // ----- Trajectory debug -----
    struct TrajDrawData
    {
        std::vector<std::vector<Eigen::Vector2f>> trajectories;
        int best_idx = -1;
        Eigen::Vector2f carrot = Eigen::Vector2f::Zero();
        float robot_x = 0.f;
        float robot_y = 0.f;
    };
    void draw_trajectory_debug(const TrajDrawData& data);
    /// Hide all trajectory segments, carrot marker, robot-to-carrot line.
    void hide_trajectory_debug();

    // ----- BMR candidate ghost polygons -----
    struct CandidateDrawData
    {
        std::vector<Eigen::Vector2f> verts;
        float score     = 0.f;
        bool  is_corner = false; // corner-indent → green, wall-indent → cyan
        bool  is_chosen = false; // brighter solid line for the selected challenger
        // Label fields
        int   side_id   = -1;  // wall side (0-3)
        int   corner_id = -1;  // corner index (0-3 → BL/BR/TR/TL)
        float seg_a     = 0.f; // wall segment start
        float seg_b     = 0.f; // wall segment end
    };
    /// Replace all ghost candidate polygons with the new set.
    void draw_candidate_polygons(const std::vector<CandidateDrawData>& candidates);
    /// Remove all ghost candidate polygons and their score labels.
    void clear_candidate_polygons();

    // ----- Force repaint -----
    void invalidate();

    // ----- Direct scene access (for legacy pool-based drawing, e.g. lidar points) -----
    QGraphicsScene& scene();

    // ----- Polygon backup lifecycle -----
    /// Discard the backup polygon item (call after confirmed save).
    void discard_polygon_backup();

Q_SIGNALS:
    void robot_rotate(QPointF);
    void robot_dragging(QPointF);
    void robot_drag_end(QPointF);
    void new_mouse_coordinates(QPointF);
    void right_click(QPointF);

private:
    AbstractGraphicViewer* agv_ = nullptr;

    // Covariance ellipse
    QGraphicsEllipseItem* cov_ellipse_item_ = nullptr;

    // Estimated room rect (shown when no polygon is loaded)
    QGraphicsRectItem* estimated_room_item_ = nullptr;

    // Room polygon capture
    std::vector<QGraphicsEllipseItem*> capture_vertex_items_;

    // Room polygon outline
    QGraphicsPolygonItem* polygon_item_         = nullptr;
    QGraphicsPolygonItem* polygon_item_backup_  = nullptr;

    // Furniture
    std::vector<QGraphicsPolygonItem*> furniture_draw_items_;

    // Temporary obstacles
    std::vector<QGraphicsPolygonItem*> temp_obstacle_draw_items_;

    // Path
    std::vector<QGraphicsItem*>         path_draw_items_;
    QGraphicsEllipseItem*               target_marker_        = nullptr;
    QGraphicsPolygonItem*               navigable_poly_item_  = nullptr;
    std::vector<QGraphicsPolygonItem*>  obstacle_expanded_items_;

    // Trajectory debug
    std::vector<std::vector<QGraphicsLineItem*>> traj_draw_items_;
    QGraphicsEllipseItem* traj_carrot_marker_     = nullptr;
    QGraphicsLineItem*    traj_robot_to_carrot_   = nullptr;

    // BMR candidate ghost polygons
    std::vector<QGraphicsPolygonItem*> candidate_polygon_items_;
    std::vector<QGraphicsTextItem*>    candidate_score_items_;
};

} // namespace rc
