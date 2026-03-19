#include "viewer_2d.h"

#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

#include <QPen>
#include <QBrush>
#include <QPolygonF>
#include <QtMath>

#include <algorithm>
#include <cstddef>
#include <limits>

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
// BMR candidate ghost polygons
// ─────────────────────────────────────────────────────────────────────────────

void Viewer2D::draw_candidate_polygons(const std::vector<CandidateDrawData>& candidates, float hx, float hy)
{
    clear_candidate_polygons();

    // Helper: map side-local coordinate t∈[-1,1] to room-frame point on the wall.
    auto side_point = [hx, hy](int side, float t) -> QPointF
    {
        t = std::clamp(t, -1.f, 1.f);
        switch (side)
        {
            case 0: return {t * hx, -hy};   // bottom
            case 1: return {hx,  t * hy};   // right
            case 2: return {t * hx,  hy};   // top
            default: return {-hx, t * hy};  // left
        }
    };

    for (const auto& c : candidates)
    {
        if (c.verts.size() < 3)
            continue;

        // wall-indent → cyan palette; corner-indent → green palette
        // chosen → more opaque solid line; runner-up → translucent dashed line
        const int alpha_fill = c.is_chosen ? 160 : 80;
        QColor pen_color = c.is_corner
            ? QColor(60, 220, 100, alpha_fill)   // green for corner
            : QColor(60, 180, 255, alpha_fill);  // cyan for wall

        QPen pen(pen_color, c.is_chosen ? 0.12f : 0.07f);
        pen.setStyle(c.is_chosen ? Qt::SolidLine : Qt::DashLine);

        QPolygonF poly;
        for (const auto& v : c.verts)
            poly << QPointF(v.x(), v.y());
        poly << QPointF(c.verts.front().x(), c.verts.front().y()); // close

        auto* pitem = agv_->scene.addPolygon(poly, pen, QBrush(Qt::NoBrush));
        pitem->setZValue(7);
        candidate_polygon_items_.push_back(pitem);

        // ── Hot-zone segment: thick line on the wall where the feature was detected ──
        const QColor hz_col = c.is_chosen
            ? QColor(255, 200, 0, 220)    // bright yellow for the chosen one
            : QColor(255, 200, 0, 100);   // dim yellow for runners-up
        const float hz_width = c.is_chosen ? 0.14f : 0.08f;
        QPen hz_pen(hz_col, hz_width, Qt::SolidLine, Qt::RoundCap);

        if (!c.is_corner && c.side_id >= 0 && c.side_id <= 3)
        {
            // Wall candidate: segment [a,b] on the wall side.
            auto* seg = agv_->scene.addLine(
                QLineF(side_point(c.side_id, c.seg_a), side_point(c.side_id, c.seg_b)),
                hz_pen);
            seg->setZValue(9);
            candidate_polygon_items_.push_back(seg);
        }
        else if (c.is_corner && c.corner_id >= 0 && c.corner_id < 4)
        {
            // Corner candidate: short segments at the corner end of two adjacent sides.
            // corner_profile_mean uses kCornerWindowBins=5 out of nside=32.
            constexpr float kFrac = 5.f / 31.f;  // fraction of [-1,1] range covered
            const int cidx = c.corner_id;
            // side_a = horizontal side, side_b = vertical side
            const int side_a = (cidx <= 1) ? 0 : 2;
            const int side_b = (cidx == 0 || cidx == 3) ? 3 : 1;
            const bool low_a = (cidx == 0 || cidx == 3);
            const bool low_b = (cidx == 0 || cidx == 1);
            const float a0 = low_a ? -1.f : (1.f - 2.f * kFrac);
            const float a1 = low_a ? (-1.f + 2.f * kFrac) : 1.f;
            const float b0 = low_b ? -1.f : (1.f - 2.f * kFrac);
            const float b1 = low_b ? (-1.f + 2.f * kFrac) : 1.f;

            auto* sega = agv_->scene.addLine(
                QLineF(side_point(side_a, a0), side_point(side_a, a1)), hz_pen);
            sega->setZValue(9);
            candidate_polygon_items_.push_back(sega);
            auto* segb = agv_->scene.addLine(
                QLineF(side_point(side_b, b0), side_point(side_b, b1)), hz_pen);
            segb->setZValue(9);
            candidate_polygon_items_.push_back(segb);
        }

        // Label for corner candidates only — wall (blue/cyan) labels are suppressed.
        // Label is placed near the corresponding room corner, not at the polygon centroid.
        if (!c.is_corner)
            continue;

        static const char* cnames[] = {"BL", "BR", "TR", "TL"};
        const char* cn = (c.corner_id >= 0 && c.corner_id < 4) ? cnames[c.corner_id] : "?";
        const QString label = QString("C%1 %2").arg(c.corner_id).arg(cn);

        // Derive room corner from the polygon bounding box.
        // The polygon clips the room corner, so the bbox extremes ARE the room corner.
        float minx = std::numeric_limits<float>::max(),  miny = minx;
        float maxx = -std::numeric_limits<float>::max(), maxy = maxx;
        for (const auto& v : c.verts)
        {
            if (v.x() < minx) minx = v.x();
            if (v.y() < miny) miny = v.y();
            if (v.x() > maxx) maxx = v.x();
            if (v.y() > maxy) maxy = v.y();
        }
        float lx, ly;
        switch (c.corner_id)
        {
            case 0:  lx = minx; ly = miny; break; // BL
            case 1:  lx = maxx; ly = miny; break; // BR
            case 2:  lx = maxx; ly = maxy; break; // TR
            default: lx = minx; ly = maxy; break; // TL (3)
        }

        auto* titem = new QGraphicsTextItem(label);
        QFont font;
        font.setBold(true);
        font.setPixelSize(16);
        titem->setFont(font);
        constexpr float label_scale = 0.020f;
        // y-mirror to counteract view's scale(1,-1) flip.
        titem->setTransform(QTransform(label_scale, 0, 0, 0, -label_scale, 0, 0, 0, 1));
        titem->setDefaultTextColor(QColor(60, 220, 100, c.is_chosen ? 255 : 160));

        // Position label just outside the room corner (beyond the magenta layout).
        const QRectF br = titem->mapToScene(titem->boundingRect()).boundingRect();
        const float W = static_cast<float>(br.width());
        const float H = static_cast<float>(br.height());
        constexpr float pad = 0.12f;
        float tx, ty;
        switch (c.corner_id)
        {
            case 0:  tx = lx - W - pad;   ty = ly - pad;     break; // BL: left & below
            case 1:  tx = lx + pad;        ty = ly - pad;     break; // BR: right & below
            case 2:  tx = lx + pad;        ty = ly + H + pad; break; // TR: right & above
            default: tx = lx - W - pad;    ty = ly + H + pad; break; // TL: left & above
        }
        titem->setPos(tx, ty);
        titem->setZValue(8);
        agv_->scene.addItem(titem);
        candidate_score_items_.push_back(titem);
    }
}

void Viewer2D::clear_candidate_polygons()
{
    for (auto* item : candidate_polygon_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    candidate_polygon_items_.clear();

    for (auto* item : candidate_score_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    candidate_score_items_.clear();
}

// ─────────────────────────────────────────────────────────────────────────────
// Top hot-zone segments (thick colored lines on walls)
// ─────────────────────────────────────────────────────────────────────────────

void Viewer2D::draw_hot_zones(const std::vector<std::array<float,4>>& zones, float hx, float hy)
{
    clear_hot_zones();

    auto side_point = [hx, hy](int side, float t) -> QPointF
    {
        t = std::clamp(t, -1.f, 1.f);
        switch (side)
        {
            case 0: return {t * hx, -hy};
            case 1: return {hx,  t * hy};
            case 2: return {t * hx,  hy};
            default: return {-hx, t * hy};
        }
    };

    // Inward offset per side, staggered by rank so lines don't overlap.
    auto inward_offset = [](int side, int rank) -> QPointF
    {
        const float off = 0.20f + 0.15f * static_cast<float>(rank);  // #1=0.20, #2=0.35, #3=0.50
        switch (side)
        {
            case 0: return {0,  off};  // bottom → up
            case 1: return {-off, 0};  // right → left
            case 2: return {0, -off};  // top → down
            default: return {off, 0};  // left → right
        }
    };

    // Colors: #1 red, #2 orange, #3 yellow.
    static const QColor colors[] = {
        QColor(255, 30, 30, 240),
        QColor(255, 160, 0, 220),
        QColor(30, 120, 255, 200)
    };

    for (std::size_t k = 0; k < zones.size() && k < 3; ++k)
    {
        const int side = static_cast<int>(zones[k][0]);
        const float a  = zones[k][1];
        const float b  = zones[k][2];

        const QPointF p0 = side_point(side, a);
        const QPointF p1 = side_point(side, b);
        const QPointF off = inward_offset(side, static_cast<int>(k));

        const float width = (k == 0) ? 0.14f : 0.09f;
        QPen pen(colors[k], width, Qt::SolidLine, Qt::RoundCap);
        auto* line = agv_->scene.addLine(
            QLineF(p0 + off, p1 + off), pen);
        line->setZValue(10);
        hot_zone_items_.push_back(line);

        // Rank label placed well inside the room, away from the line.
        // Use a large extra inward push so the text doesn't overlap the line.
        auto label_offset = [](int s, int rank) -> QPointF
        {
            const float lo = 0.45f + 0.20f * static_cast<float>(rank);
            switch (s)
            {
                case 0: return {0,  lo};
                case 1: return {-lo, 0};
                case 2: return {0, -lo};
                default: return {lo, 0};
            }
        };

        auto* label = new QGraphicsTextItem(QString("#%1").arg(k + 1));
        QFont font;
        font.setBold(true);
        font.setPixelSize(18);
        label->setFont(font);
        label->setDefaultTextColor(colors[k]);
        constexpr float lscale = 0.022f;
        label->setTransform(QTransform(lscale, 0, 0, 0, -lscale, 0, 0, 0, 1));
        const QPointF mid = (p0 + p1) * 0.5 + label_offset(side, static_cast<int>(k));
        label->setPos(mid);
        label->setZValue(11);
        agv_->scene.addItem(label);
        hot_zone_items_.push_back(label);
    }
}

void Viewer2D::clear_hot_zones()
{
    for (auto* item : hot_zone_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    hot_zone_items_.clear();
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
