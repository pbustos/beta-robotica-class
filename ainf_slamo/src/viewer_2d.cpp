#include "viewer_2d.h"

#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

#include <QPen>
#include <QBrush>
#include <QPolygonF>
#include <QtMath>

#include <algorithm>
#include <cstddef>

namespace rc {

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────
Viewer2D::Viewer2D(QWidget* parent, const QRectF& grid_dim, bool show_axis)
{
    agv_ = new AbstractGraphicViewer(parent, grid_dim, show_axis);

    // Forward all AGV signals as Viewer2D signals
    connect(agv_, &AbstractGraphicViewer::robot_rotate,
            this, &Viewer2D::robot_rotate);
    connect(agv_, &AbstractGraphicViewer::robot_dragging,
            this, &Viewer2D::robot_dragging);
    connect(agv_, &AbstractGraphicViewer::robot_drag_end,
            this, &Viewer2D::robot_drag_end);
    connect(agv_, &AbstractGraphicViewer::new_mouse_coordinates,
            this, &Viewer2D::new_mouse_coordinates);
    connect(agv_, &AbstractGraphicViewer::right_click,
            this, &Viewer2D::right_click);
}

// ─────────────────────────────────────────────────────────────────────────────
// Widget / view management
// ─────────────────────────────────────────────────────────────────────────────
QWidget* Viewer2D::get_widget() const { return agv_; }

void Viewer2D::add_robot(float w, float l, float offset, float rotation, QColor color)
{
    agv_->add_robot(w, l, offset, rotation, color);
}

void Viewer2D::show() { agv_->show(); }

QTransform Viewer2D::transform() const { return agv_->transform(); }

void Viewer2D::set_transform(const QTransform& t) { agv_->setTransform(t); }

void Viewer2D::fit_to_scene(const QRectF& r) { agv_->fitToScene(r); }

void Viewer2D::center_on(float x, float y) { agv_->centerOn(x, y); }

// ─────────────────────────────────────────────────────────────────────────────
// Robot
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::update_robot(float x, float y, float angle_rad)
{
    agv_->robot_poly()->setPos(x, y);
    agv_->robot_poly()->setRotation(qRadiansToDegrees(angle_rad));
}

// ─────────────────────────────────────────────────────────────────────────────
// Covariance ellipse
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::update_covariance_ellipse(float cx, float cy,
                                         float rx, float ry,
                                         float angle_deg)
{
    if (cov_ellipse_item_ == nullptr)
    {
        cov_ellipse_item_ = agv_->scene.addEllipse(
            -rx, -ry, 2.f * rx, 2.f * ry,
            QPen(QColor(255, 50, 50), 0.03),
            QBrush(QColor(255, 100, 100, 80)));
        cov_ellipse_item_->setZValue(100);
    }
    else
    {
        cov_ellipse_item_->setRect(-rx, -ry, 2.f * rx, 2.f * ry);
    }
    cov_ellipse_item_->setPos(cx, cy);
    cov_ellipse_item_->setRotation(angle_deg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Estimated room rect
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::update_estimated_room_rect(float width, float length, bool has_polygon)
{
    if (has_polygon)
    {
        if (estimated_room_item_ != nullptr)
        {
            agv_->scene.removeItem(estimated_room_item_);
            delete estimated_room_item_;
            estimated_room_item_ = nullptr;
        }
        return;
    }

    const QRectF room_rect(-width / 2.f, -length / 2.f, width, length);
    if (estimated_room_item_ == nullptr)
    {
        estimated_room_item_ = agv_->scene.addRect(
            room_rect, QPen(Qt::magenta, 0.05), QBrush(Qt::NoBrush));
        estimated_room_item_->setZValue(2);
    }
    else
    {
        estimated_room_item_->setRect(room_rect);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Room polygon capture vertices
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::add_capture_vertex(QPointF pos)
{
    constexpr float radius = 0.15f;
    auto* item = agv_->scene.addEllipse(
        -radius, -radius, 2.f * radius, 2.f * radius,
        QPen(Qt::yellow, 0.05), QBrush(Qt::yellow));
    item->setPos(pos.x(), pos.y());
    item->setZValue(10);
    capture_vertex_items_.push_back(item);
}

void Viewer2D::clear_capture_vertices()
{
    for (auto* item : capture_vertex_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    capture_vertex_items_.clear();
}

// ─────────────────────────────────────────────────────────────────────────────
// Room polygon outline
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::draw_room_polygon(const std::vector<Eigen::Vector2f>& verts, bool is_capturing)
{
    if (verts.size() < 2)
        return;

    if (polygon_item_ != nullptr)
    {
        agv_->scene.removeItem(polygon_item_);
        delete polygon_item_;
        polygon_item_ = nullptr;
    }

    QPolygonF poly;
    for (const auto& v : verts)
        poly << QPointF(v.x(), v.y());
    if (!is_capturing && verts.size() >= 3)
        poly << QPointF(verts.front().x(), verts.front().y());

    QPen pen(is_capturing ? Qt::yellow : Qt::magenta,
             is_capturing ? 0.08 : 0.15);
    polygon_item_ = agv_->scene.addPolygon(poly, pen, QBrush(Qt::NoBrush));
    polygon_item_->setZValue(8);
}

void Viewer2D::clear_room_polygon()
{
    if (polygon_item_ != nullptr)
    {
        agv_->scene.removeItem(polygon_item_);
        delete polygon_item_;
        polygon_item_ = nullptr;
    }
}

void Viewer2D::set_room_edit_corners(const std::vector<Eigen::Vector2f>& verts, bool visible)
{
    for (auto* item : room_edit_corner_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    room_edit_corner_items_.clear();

    if (!visible)
        return;

    constexpr float radius = 0.16f;
    for (const auto& v : verts)
    {
        auto* item = agv_->scene.addEllipse(
            -radius, -radius, 2.f * radius, 2.f * radius,
            QPen(QColor(255, 140, 0), 0.05),
            QBrush(QColor(255, 165, 0, 180)));
        item->setPos(v.x(), v.y());
        item->setZValue(30);
        room_edit_corner_items_.push_back(item);
    }
}

void Viewer2D::update_room_edit_corner(std::size_t idx, const Eigen::Vector2f& pos)
{
    if (idx >= room_edit_corner_items_.size() || room_edit_corner_items_[idx] == nullptr)
        return;
    room_edit_corner_items_[idx]->setPos(pos.x(), pos.y());
}

int Viewer2D::nearest_room_edit_corner(const QPointF& scene_pos, float max_dist_m) const
{
    int best_idx = -1;
    float best_d2 = max_dist_m * max_dist_m;
    for (std::size_t i = 0; i < room_edit_corner_items_.size(); ++i)
    {
        const auto* item = room_edit_corner_items_[i];
        if (!item) continue;
        const QPointF p = item->pos();
        const float dx = static_cast<float>(scene_pos.x() - p.x());
        const float dy = static_cast<float>(scene_pos.y() - p.y());
        const float d2 = dx * dx + dy * dy;
        if (d2 <= best_d2)
        {
            best_d2 = d2;
            best_idx = static_cast<int>(i);
        }
    }
    return best_idx;
}

void Viewer2D::save_polygon_to_backup()
{
    // Discard any previous backup
    if (polygon_item_backup_ != nullptr)
    {
        agv_->scene.removeItem(polygon_item_backup_);
        delete polygon_item_backup_;
        polygon_item_backup_ = nullptr;
    }
    polygon_item_backup_ = polygon_item_;
    polygon_item_ = nullptr;
}

void Viewer2D::restore_polygon_from_backup()
{
    if (polygon_item_ != nullptr)
    {
        agv_->scene.removeItem(polygon_item_);
        delete polygon_item_;
        polygon_item_ = nullptr;
    }
    polygon_item_ = polygon_item_backup_;
    polygon_item_backup_ = nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// Furniture polygons
// ─────────────────────────────────────────────────────────────────────────────

static QColor furniture_color_for(const std::string& label)
{
    // Bench → warm brown; everything else → blue
    if (label.size() >= 5 &&
        (label[0] == 'b' || label[0] == 'B') &&
        (label[1] == 'e' || label[1] == 'E') &&
        (label[2] == 'n' || label[2] == 'N') &&
        (label[3] == 'c' || label[3] == 'C') &&
        (label[4] == 'h' || label[4] == 'H'))
        return QColor(230, 140, 30);  // warm orange
    return QColor(50, 100, 255);       // default blue
}

void Viewer2D::draw_furniture(const std::vector<rc::FurniturePolygonData>& fps)
{
    for (auto* item : furniture_draw_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    furniture_draw_items_.clear();

    for (const auto& fp : fps)
    {
        if (fp.vertices.size() < 3)
            continue;

        const QColor c = furniture_color_for(fp.label);
        const QPen   pen  (c, 0.06);
        const QBrush brush(QColor(c.red(), c.green(), c.blue(), 40));

        QPolygonF qpoly;
        for (const auto& v : fp.vertices)
            qpoly << QPointF(v.x(), v.y());
        qpoly << QPointF(fp.vertices.front().x(), fp.vertices.front().y());

        auto* item = agv_->scene.addPolygon(qpoly, pen, brush);
        item->setZValue(7);
        furniture_draw_items_.push_back(item);
    }
}

void Viewer2D::update_furniture_item(std::size_t idx, const rc::FurniturePolygonData& fp)
{
    if (idx < furniture_draw_items_.size() && furniture_draw_items_[idx] != nullptr)
    {
        agv_->scene.removeItem(furniture_draw_items_[idx]);
        delete furniture_draw_items_[idx];
        furniture_draw_items_[idx] = nullptr;
    }
    else if (idx >= furniture_draw_items_.size())
    {
        furniture_draw_items_.resize(idx + 1, nullptr);
    }

    if (fp.vertices.size() < 3)
        return;

    const QColor c = furniture_color_for(fp.label);
    const QPen   pen  (c, 0.06);
    const QBrush brush(QColor(c.red(), c.green(), c.blue(), 40));

    QPolygonF qpoly;
    for (const auto& v : fp.vertices)
        qpoly << QPointF(v.x(), v.y());
    qpoly << QPointF(fp.vertices.front().x(), fp.vertices.front().y());

    auto* item = agv_->scene.addPolygon(qpoly, pen, brush);
    item->setZValue(7);
    furniture_draw_items_[idx] = item;

    // Force immediate repaint for rapid drag updates
    agv_->scene.update();
}

void Viewer2D::set_furniture_item_style(std::size_t idx, QPen pen, QBrush brush)
{
    if (idx < furniture_draw_items_.size() && furniture_draw_items_[idx] != nullptr)
    {
        furniture_draw_items_[idx]->setPen(pen);
        furniture_draw_items_[idx]->setBrush(brush);
    }
}

std::size_t Viewer2D::furniture_count() const
{
    return furniture_draw_items_.size();
}

// ─────────────────────────────────────────────────────────────────────────────
// Temporary obstacles
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::draw_temp_obstacles(const std::vector<std::vector<Eigen::Vector2f>>& polys)
{
    for (auto* item : temp_obstacle_draw_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    temp_obstacle_draw_items_.clear();

    for (const auto& verts : polys)
    {
        if (verts.empty())
            continue;

        QPolygonF poly;
        for (const auto& v : verts)
            poly << QPointF(v.x(), v.y());
        poly << QPointF(verts.front().x(), verts.front().y());

        auto* item = agv_->scene.addPolygon(
            poly,
            QPen(QColor(255, 100, 0), 0.06),
            QBrush(QColor(255, 100, 0, 60)));
        item->setZValue(5);
        temp_obstacle_draw_items_.push_back(item);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Path
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::draw_path(const PathDrawData& data)
{
    clear_path_items();

    if (data.path.size() < 2)
        return;

    // Original polygon vertex dots (green)
    for (const auto& v : data.orig_poly_verts)
    {
        constexpr float r = 0.1f;
        auto* dot = agv_->scene.addEllipse(
            -r, -r, 2.f * r, 2.f * r,
            QPen(QColor(0, 200, 0), 0.02),
            QBrush(QColor(0, 200, 0, 80)));
        dot->setPos(v.x(), v.y());
        dot->setZValue(17);
        path_draw_items_.push_back(dot);
    }

    // Inner (shrunken) polygon outline
    if (navigable_poly_item_ != nullptr)
    {
        agv_->scene.removeItem(navigable_poly_item_);
        delete navigable_poly_item_;
        navigable_poly_item_ = nullptr;
    }
    if (data.inner_poly.size() >= 3)
    {
        QPolygonF qpoly;
        for (const auto& v : data.inner_poly)
            qpoly << QPointF(v.x(), v.y());
        qpoly << QPointF(data.inner_poly.front().x(), data.inner_poly.front().y());

        navigable_poly_item_ = agv_->scene.addPolygon(
            qpoly,
            QPen(QColor(255, 140, 0, 200), 0.03, Qt::DashLine),
            Qt::NoBrush);
        navigable_poly_item_->setZValue(19);
    }

    // Expanded obstacle outlines
    for (auto* item : obstacle_expanded_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    obstacle_expanded_items_.clear();

    for (const auto& obs : data.expanded_obstacles)
    {
        if (obs.size() < 3)
            continue;

        QPolygonF qpoly;
        for (const auto& v : obs)
            qpoly << QPointF(v.x(), v.y());
        qpoly << QPointF(obs.front().x(), obs.front().y());

        auto* obs_item = agv_->scene.addPolygon(
            qpoly,
            QPen(QColor(255, 140, 0, 200), 0.03, Qt::DashLine),
            Qt::NoBrush);
        obs_item->setZValue(19);
        obstacle_expanded_items_.push_back(obs_item);
    }

    // Navigable polygon vertex dots (yellow)
    for (const auto& v : data.nav_poly)
    {
        constexpr float r = 0.08f;
        auto* dot = agv_->scene.addEllipse(
            -r, -r, 2.f * r, 2.f * r,
            QPen(QColor(255, 255, 0, 200), 0.01),
            QBrush(QColor(255, 255, 0, 120)));
        dot->setPos(v.x(), v.y());
        dot->setZValue(18);
        path_draw_items_.push_back(dot);
    }

    // Path line segments
    const QPen path_pen(QColor(100, 255, 100), 0.04);
    for (std::size_t i = 0; i + 1 < data.path.size(); ++i)
    {
        auto* line = agv_->scene.addLine(
            data.path[i].x(),     data.path[i].y(),
            data.path[i+1].x(),   data.path[i+1].y(),
            path_pen);
        line->setZValue(20);
        path_draw_items_.push_back(line);
    }

    // Intermediate waypoint dots (cyan)
    const QBrush wp_brush(QColor(0, 220, 220));
    for (std::size_t i = 1; i + 1 < data.path.size(); ++i)
    {
        constexpr float r = 0.06f;
        auto* dot = agv_->scene.addEllipse(-r, -r, 2.f * r, 2.f * r, Qt::NoPen, wp_brush);
        dot->setPos(data.path[i].x(), data.path[i].y());
        dot->setZValue(21);
        path_draw_items_.push_back(dot);
    }

    // Goal / target marker
    const auto& goal = data.path.back();
    if (target_marker_ == nullptr)
    {
        constexpr float tr = 0.12f;
        target_marker_ = agv_->scene.addEllipse(
            -tr, -tr, 2.f * tr, 2.f * tr,
            QPen(QColor(255, 50, 50), 0.03),
            QBrush(QColor(255, 50, 50, 120)));
        target_marker_->setZValue(22);
    }
    target_marker_->setPos(goal.x(), goal.y());
    target_marker_->setVisible(true);
}

void Viewer2D::clear_path_items()
{
    for (auto* item : path_draw_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    path_draw_items_.clear();

    for (auto* item : obstacle_expanded_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    obstacle_expanded_items_.clear();

    if (target_marker_ != nullptr)
        target_marker_->setVisible(false);
}

// ─────────────────────────────────────────────────────────────────────────────
// Trajectory debug
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::draw_trajectory_debug(const TrajDrawData& data)
{
    const int num_traj = static_cast<int>(data.trajectories.size());

    while (static_cast<int>(traj_draw_items_.size()) < num_traj)
        traj_draw_items_.emplace_back();

    for (int t = 0; t < static_cast<int>(traj_draw_items_.size()); ++t)
    {
        if (t >= num_traj)
        {
            for (auto* seg : traj_draw_items_[t])
                seg->setVisible(false);
            continue;
        }

        const auto& traj = data.trajectories[t];
        const bool is_best = (t == data.best_idx);
        const QColor color = is_best ? QColor(0, 220, 0) : QColor(150, 150, 150, 100);
        const float  width = is_best ? 0.06f : 0.02f;
        const int    z     = is_best ? 32 : 28;

        auto& segments = traj_draw_items_[t];
        while (segments.size() < traj.size())
        {
            auto* item = agv_->scene.addLine(0, 0, 0, 0, QPen(color, width));
            item->setZValue(z);
            segments.push_back(item);
        }

        for (std::size_t s = 0; s + 1 < traj.size(); ++s)
        {
            segments[s]->setLine(traj[s].x(), traj[s].y(),
                                 traj[s+1].x(), traj[s+1].y());
            segments[s]->setPen(QPen(color, width));
            segments[s]->setZValue(z);
            segments[s]->setVisible(true);
        }
        const std::size_t active = traj.empty() ? 0u : traj.size() - 1u;
        for (std::size_t s = active; s < segments.size(); ++s)
            segments[s]->setVisible(false);
    }

    // Carrot marker
    constexpr float cr = 0.15f;
    if (traj_carrot_marker_ == nullptr)
    {
        traj_carrot_marker_ = agv_->scene.addEllipse(
            -cr, -cr, 2.f * cr, 2.f * cr,
            QPen(QColor(255, 140, 0), 0.03f),
            QBrush(QColor(255, 140, 0, 180)));
        traj_carrot_marker_->setZValue(33);
    }
    traj_carrot_marker_->setPos(data.carrot.x(), data.carrot.y());
    traj_carrot_marker_->setVisible(true);

    // Robot-to-carrot dashed line
    if (traj_robot_to_carrot_ == nullptr)
    {
        traj_robot_to_carrot_ = agv_->scene.addLine(
            data.robot_x, data.robot_y,
            data.carrot.x(), data.carrot.y(),
            QPen(QColor(255, 165, 0, 200), 0.02f, Qt::DashLine));
        traj_robot_to_carrot_->setZValue(29);
    }
    else
    {
        traj_robot_to_carrot_->setLine(data.robot_x, data.robot_y,
                                       data.carrot.x(), data.carrot.y());
        traj_robot_to_carrot_->setVisible(true);
    }
}

void Viewer2D::hide_trajectory_debug()
{
    for (auto& segs : traj_draw_items_)
        for (auto* seg : segs)
            seg->setVisible(false);
    if (traj_carrot_marker_     != nullptr) traj_carrot_marker_->setVisible(false);
    if (traj_robot_to_carrot_   != nullptr) traj_robot_to_carrot_->setVisible(false);
}

// ─────────────────────────────────────────────────────────────────────────────
// Force repaint
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::invalidate() { agv_->scene.update(); }

QGraphicsScene& Viewer2D::scene() { return agv_->scene; }

void Viewer2D::discard_polygon_backup()
{
    if (polygon_item_backup_ != nullptr)
    {
        agv_->scene.removeItem(polygon_item_backup_);
        delete polygon_item_backup_;
        polygon_item_backup_ = nullptr;
    }
}

} // namespace rc
