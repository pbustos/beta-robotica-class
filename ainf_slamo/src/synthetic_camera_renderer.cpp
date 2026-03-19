#include "synthetic_camera_renderer.h"

#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <cmath>
#include <algorithm>
#include <array>
#include <limits>

namespace rc {

// ---------------------------------------------------------------------------
// Coordinate transforms
// ---------------------------------------------------------------------------
Eigen::Vector3f SyntheticCameraRenderer::world_to_camera(const Eigen::Vector2f& p_robot, float z_world) const
{
    // Camera frame: x=lateral, y=forward (depth), z=up.
    return Eigen::Vector3f(p_robot.x() - extr_.tx,
                           p_robot.y() - extr_.ty,
                           z_world     - extr_.tz);
}

bool SyntheticCameraRenderer::project(const Eigen::Vector3f& p_cam, float& u, float& v) const
{
    constexpr float near = 0.05f;
    const float depth = p_cam.y();
    if (depth <= near) return false;
    u = intr_.cx + intr_.fx * (p_cam.x() / depth);
    v = intr_.cy - intr_.fy * (p_cam.z() / depth);
    return std::isfinite(u) && std::isfinite(v);
}

// ---------------------------------------------------------------------------
// draw_segment_3d  – clip to near plane then draw
// ---------------------------------------------------------------------------
bool SyntheticCameraRenderer::draw_segment_3d(QPainter& painter,
                                              const Eigen::Vector3f& a_cam,
                                              const Eigen::Vector3f& b_cam) const
{
    constexpr float near = 0.05f;
    Eigen::Vector3f a = a_cam, b = b_cam;

    // Near-plane clip
    if (a.y() <= near && b.y() <= near) return false;
    if (a.y() <= near)
    {
        const float t = (near - a.y()) / (b.y() - a.y());
        a = a + t * (b - a);
    }
    else if (b.y() <= near)
    {
        const float t = (near - b.y()) / (a.y() - b.y());
        b = b + t * (a - b);
    }

    float ua, va, ub, vb;
    if (!project(a, ua, va) || !project(b, ub, vb)) return false;

    // Broad image-bounds check (with margin)
    const float margin = 200.f;
    const float w = static_cast<float>(intr_.width);
    const float h = static_cast<float>(intr_.height);
    if ((ua < -margin && ub < -margin) || (ua > w + margin && ub > w + margin) ||
        (va < -margin && vb < -margin) || (va > h + margin && vb > h + margin))
        return false;

    painter.drawLine(QPointF(ua, va), QPointF(ub, vb));
    return true;
}

// ---------------------------------------------------------------------------
// Oriented box helper
// ---------------------------------------------------------------------------
static std::array<Eigen::Vector2f, 4> box_corners(const std::vector<Eigen::Vector2f>& poly)
{
    std::array<Eigen::Vector2f, 4> out;
    if (poly.size() == 4)
    {
        for (int i = 0; i < 4; ++i) out[i] = poly[i];
        return out;
    }
    // Fallback: axis-aligned bounding box
    Eigen::Vector2f lo(1e9f, 1e9f), hi(-1e9f, -1e9f);
    for (const auto& v : poly) { lo = lo.cwiseMin(v); hi = hi.cwiseMax(v); }
    out[0] = lo;
    out[1] = {hi.x(), lo.y()};
    out[2] = hi;
    out[3] = {lo.x(), hi.y()};
    return out;
}

// ---------------------------------------------------------------------------
// draw_furniture_wireframe
// ---------------------------------------------------------------------------
float SyntheticCameraRenderer::draw_furniture_wireframe(
    QPainter& painter,
    const FurniturePolygonData& fp,
    const Eigen::Affine2f& world_to_robot,
    const QColor& color) const
{
    if (fp.vertices.size() < 3) return 0.f;

    const float h = std::max(0.2f, fp.height);
    const auto corners = box_corners(fp.vertices);

    auto cam_pt = [&](const Eigen::Vector2f& v, float z) -> Eigen::Vector3f
    {
        return world_to_camera(world_to_robot * v, z);
    };

    // Detect label type
    const QString lbl = QString::fromStdString(fp.label).toLower();
    const bool is_table = lbl.contains("mesa") || lbl.contains("table");
    const bool is_chair = lbl.contains("silla") || lbl.contains("chair");
    const bool is_bench = lbl.contains("bench") || lbl.contains("banco");

    painter.setPen(QPen(color, 1.8, Qt::SolidLine));

    int drawn = 0;
    auto draw = [&](const Eigen::Vector2f& a, float za, const Eigen::Vector2f& b, float zb) -> bool
    {
        bool ok = draw_segment_3d(painter, cam_pt(a, za), cam_pt(b, zb));
        if (ok) ++drawn;
        return ok;
    };

    const auto& c0 = corners[0];
    const auto& c1 = corners[1];
    const auto& c2 = corners[2];
    const auto& c3 = corners[3];

    if (is_table)
    {
        // Tabletop
        const float leg_h = h * 0.92f;
        draw(c0, h, c1, h); draw(c1, h, c2, h);
        draw(c2, h, c3, h); draw(c3, h, c0, h);
        // Legs
        const float t = 0.12f;
        for (int i = 0; i < 4; ++i)
        {
            const auto& ci = corners[i];
            const auto& cj = corners[(i + 1) % 4];
            const auto& ck = corners[(i + 3) % 4];
            const Eigen::Vector2f leg = ci + t * (cj - ci) + t * (ck - ci);
            draw(leg, 0.f, leg, leg_h);
        }
        // Cross brace at mid-height
        const Eigen::Vector2f mid01 = 0.5f * (c0 + c1);
        const Eigen::Vector2f mid23 = 0.5f * (c2 + c3);
        const Eigen::Vector2f mid12 = 0.5f * (c1 + c2);
        const Eigen::Vector2f mid30 = 0.5f * (c3 + c0);
        draw(mid01, 0.7f * h, mid23, 0.7f * h);
        draw(mid12, 0.7f * h, mid30, 0.7f * h);
    }
    else if (is_chair)
    {
        const float z_seat = 0.50f * h;
        const float z_back = 0.98f * h;
        const float t = 0.18f;
        const Eigen::Vector2f s0 = c0 + t * (c1 - c0) + t * (c3 - c0);
        const Eigen::Vector2f s1 = c1 + t * (c0 - c1) + t * (c2 - c1);
        const Eigen::Vector2f s2 = c2 + t * (c1 - c2) + t * (c3 - c2);
        const Eigen::Vector2f s3 = c3 + t * (c2 - c3) + t * (c0 - c3);
        // Seat
        draw(s0, z_seat, s1, z_seat); draw(s1, z_seat, s2, z_seat);
        draw(s2, z_seat, s3, z_seat); draw(s3, z_seat, s0, z_seat);
        // Legs
        draw(s0, 0.f, s0, z_seat); draw(s1, 0.f, s1, z_seat);
        draw(s2, 0.f, s2, z_seat); draw(s3, 0.f, s3, z_seat);
        // Backrest
        const Eigen::Vector2f b0 = 0.5f * (s2 + c2);
        const Eigen::Vector2f b1 = 0.5f * (s3 + c3);
        draw(b0, z_seat, b0, z_back);
        draw(b1, z_seat, b1, z_back);
        draw(b0, z_back, b1, z_back);
        draw(0.5f * (b0 + b1), z_seat, 0.5f * (b0 + b1), z_back);
    }
    else if (is_bench)
    {
        // Bench: seat slab + legs
        const float seat_h = 0.5f * h;
        draw(c0, seat_h, c1, seat_h); draw(c1, seat_h, c2, seat_h);
        draw(c2, seat_h, c3, seat_h); draw(c3, seat_h, c0, seat_h);
        draw(c0, 0.f, c0, seat_h); draw(c1, 0.f, c1, seat_h);
        draw(c2, 0.f, c2, seat_h); draw(c3, 0.f, c3, seat_h);
        // Back
        draw(c2, seat_h, c2, h); draw(c3, seat_h, c3, h);
        draw(c2, h, c3, h);
    }
    else
    {
        // Generic box wireframe
        draw(c0, 0.f, c1, 0.f); draw(c1, 0.f, c2, 0.f);
        draw(c2, 0.f, c3, 0.f); draw(c3, 0.f, c0, 0.f);
        draw(c0, h, c1, h); draw(c1, h, c2, h);
        draw(c2, h, c3, h); draw(c3, h, c0, h);
        draw(c0, 0.f, c0, h); draw(c1, 0.f, c1, h);
        draw(c2, 0.f, c2, h); draw(c3, 0.f, c3, h);
    }

    // Estimate projected area from bounding box of projected corners
    if (drawn == 0) return 0.f;

    float umin = 1e9f, umax = -1e9f, vmin = 1e9f, vmax = -1e9f;
    for (int i = 0; i < 4; ++i)
    {
        const Eigen::Vector2f vr = world_to_robot * corners[i];
        for (float z : {0.f, h})
        {
            const auto pc = world_to_camera(vr, z);
            float u, v;
            if (project(pc, u, v))
            {
                umin = std::min(umin, u); umax = std::max(umax, u);
                vmin = std::min(vmin, v); vmax = std::max(vmax, v);
            }
        }
    }
    const float area = std::max(0.f, umax - umin) * std::max(0.f, vmax - vmin);
    return area;
}

// ---------------------------------------------------------------------------
// draw_walls
// ---------------------------------------------------------------------------
void SyntheticCameraRenderer::draw_walls(QPainter& painter,
                                         const std::vector<Eigen::Vector2f>& room_polygon,
                                         const Eigen::Affine2f& world_to_robot) const
{
    if (room_polygon.size() < 3) return;

    painter.setPen(QPen(wall_color, 3.0, Qt::SolidLine));

    // Winding-aware inward normal for each wall edge.
    float signed_area = 0.f;
    for (int i = 0; i < static_cast<int>(room_polygon.size()); ++i)
    {
        const auto& p = room_polygon[i];
        const auto& q = room_polygon[(i + 1) % room_polygon.size()];
        signed_area += p.x() * q.y() - q.x() * p.y();
    }
    const bool polygon_ccw = signed_area > 0.f;

    const float h = wall_height_;
    const int n = static_cast<int>(room_polygon.size());

    struct WallPlane
    {
        Eigen::Vector3f a0;
        Eigen::Vector3f edge_u;
        float edge_u_sq;
        Eigen::Vector3f edge_v;
        float edge_v_sq;
        Eigen::Vector3f normal;
        float plane_d;
    };
    std::vector<WallPlane> wall_planes;
    wall_planes.reserve(n);

    struct EdgeData
    {
        Eigen::Vector2f ar;
        Eigen::Vector2f br;
        Eigen::Vector3f a0;
        Eigen::Vector3f b0;
        Eigen::Vector3f a1;
        Eigen::Vector3f b1;
    };
    std::vector<EdgeData> edges;
    edges.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        const auto& a = room_polygon[i];
        const auto& b = room_polygon[(i + 1) % n];
        const Eigen::Vector2f ar = world_to_robot * a;
        const Eigen::Vector2f br = world_to_robot * b;

        const auto a0 = world_to_camera(ar, 0.f);
        const auto a1 = world_to_camera(ar, h);
        const auto b0 = world_to_camera(br, 0.f);
        const auto b1 = world_to_camera(br, h);

        const Eigen::Vector3f edge_u = b0 - a0;
        const Eigen::Vector3f edge_v = a1 - a0;
        const float edge_u_sq = edge_u.squaredNorm();
        const float edge_v_sq = edge_v.squaredNorm();
        if (edge_u_sq > 1e-9f && edge_v_sq > 1e-9f)
        {
            Eigen::Vector3f normal = edge_u.cross(edge_v);
            const float n_norm = normal.norm();
            if (n_norm > 1e-9f)
            {
                normal /= n_norm;
                wall_planes.push_back(WallPlane{a0, edge_u, edge_u_sq, edge_v, edge_v_sq, normal, -normal.dot(a0)});
            }
            else
            {
                wall_planes.push_back(WallPlane{a0, edge_u, edge_u_sq, edge_v, edge_v_sq, Eigen::Vector3f::Zero(), 0.f});
            }
        }
        else
        {
            wall_planes.push_back(WallPlane{a0, edge_u, edge_u_sq, edge_v, edge_v_sq, Eigen::Vector3f::Zero(), 0.f});
        }

        edges.push_back(EdgeData{ar, br, a0, b0, a1, b1});
    }

    auto segment_visible = [&](const Eigen::Vector3f& p_mid, int owner_idx) -> bool
    {
        constexpr float near = 0.05f;
        if (p_mid.y() <= near)
            return false;

        const Eigen::Vector3f ray = p_mid; // camera origin is (0,0,0), point at t=1
        float best_t = std::numeric_limits<float>::infinity();
        int best_idx = -1;

        for (int j = 0; j < static_cast<int>(wall_planes.size()); ++j)
        {
            const auto& wp = wall_planes[j];
            if (wp.edge_u_sq < 1e-9f || wp.edge_v_sq < 1e-9f || wp.normal.squaredNorm() < 1e-9f)
                continue;

            const float den = wp.normal.dot(ray);
            if (std::abs(den) < 1e-7f)
                continue;

            const float t = -wp.plane_d / den;
            if (t <= near)
                continue;

            const Eigen::Vector3f hit = t * ray;
            const Eigen::Vector3f rel = hit - wp.a0;
            const float alpha = rel.dot(wp.edge_u) / wp.edge_u_sq;
            const float beta = rel.dot(wp.edge_v) / wp.edge_v_sq;
            if (alpha < -1e-4f || alpha > 1.f + 1e-4f || beta < -1e-4f || beta > 1.f + 1e-4f)
                continue;

            if (t < best_t)
            {
                best_t = t;
                best_idx = j;
            }
        }

        if (best_idx < 0)
            return true;
        if (best_idx == owner_idx)
            return true;

        // Another wall is strictly in front of this segment midpoint.
        return best_t >= 1.f - 1e-3f;
    };

    const Eigen::Vector2f cam_pos(extr_.tx, extr_.ty);
    auto draw_if_visible = [&](const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, int owner_idx)
    {
        const Eigen::Vector3f mid = 0.5f * (p0 + p1);
        if (segment_visible(mid, owner_idx))
            draw_segment_3d(painter, p0, p1);
    };

    for (int i = 0; i < n; ++i)
    {
        const auto& e = edges[i];

        // Keep only the inward-facing side of each wall.
        const Eigen::Vector2f edge2d = e.br - e.ar;
        Eigen::Vector2f inward_normal;
        if (polygon_ccw)
            inward_normal = Eigen::Vector2f(-edge2d.y(), edge2d.x());
        else
            inward_normal = Eigen::Vector2f(edge2d.y(), -edge2d.x());
        const Eigen::Vector2f mid2d = 0.5f * (e.ar + e.br);
        if ((cam_pos - mid2d).dot(inward_normal) <= 0.f)
            continue;

        // Bottom edge
        draw_if_visible(e.a0, e.b0, i);
        // Top edge
        draw_if_visible(e.a1, e.b1, i);
        // Vertical edges
        draw_if_visible(e.a0, e.a1, i);
    }
}

// ---------------------------------------------------------------------------
// draw_floor  – floor polygon outline
// ---------------------------------------------------------------------------
void SyntheticCameraRenderer::draw_floor(QPainter& painter,
                                         const std::vector<Eigen::Vector2f>& room_polygon,
                                         const Eigen::Affine2f& world_to_robot) const
{
    if (room_polygon.size() < 3) return;

    // Semi-transparent floor fill: project polygon and fill with QPainter
    QPolygonF floor_poly;
    bool any_in_front = false;
    for (const auto& v : room_polygon)
    {
        const auto pc = world_to_camera(world_to_robot * v, 0.f);
        float u, pv;
        if (project(pc, u, pv))
        {
            floor_poly << QPointF(u, pv);
            any_in_front = true;
        }
    }

    if (any_in_front && floor_poly.size() >= 3)
    {
        painter.setPen(Qt::NoPen);
        painter.setBrush(QBrush(floor_color));
        painter.drawPolygon(floor_poly);
    }

    // Floor outline
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(wall_color, 2.5, Qt::DotLine));
    const int n = static_cast<int>(room_polygon.size());
    for (int i = 0; i < n; ++i)
    {
        const auto& a = room_polygon[i];
        const auto& b = room_polygon[(i + 1) % n];
        const auto ac = world_to_camera(world_to_robot * a, 0.f);
        const auto bc = world_to_camera(world_to_robot * b, 0.f);
        draw_segment_3d(painter, ac, bc);
    }
}

// ---------------------------------------------------------------------------
// render – main entry point
// ---------------------------------------------------------------------------
SyntheticCameraRenderer::RenderResult SyntheticCameraRenderer::render(
    const Eigen::Affine2f& robot_pose,
    const std::vector<FurniturePolygonData>& furniture,
    const std::vector<Eigen::Vector2f>& room_polygon) const
{
    RenderResult result;
    if (intr_.width <= 0 || intr_.height <= 0 || intr_.fx <= 0.f)
        return result;
    result.overlay = QImage(intr_.width, intr_.height, QImage::Format_ARGB32_Premultiplied);
    result.overlay.fill(Qt::transparent);

    QPainter painter(&result.overlay);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const Eigen::Affine2f world_to_robot = robot_pose.inverse();

    // 1. Floor
    draw_floor(painter, room_polygon, world_to_robot);

    // 2. Walls
    draw_walls(painter, room_polygon, world_to_robot);

    struct WallPlane
    {
        Eigen::Vector3f a0;
        Eigen::Vector3f edge_u;
        float edge_u_sq;
        Eigen::Vector3f edge_v;
        float edge_v_sq;
        Eigen::Vector3f normal;
        float plane_d;
    };
    std::vector<WallPlane> wall_planes;
    if (room_polygon.size() >= 2)
    {
        wall_planes.reserve(room_polygon.size());
        for (int i = 0; i < static_cast<int>(room_polygon.size()); ++i)
        {
            const auto& a = room_polygon[i];
            const auto& b = room_polygon[(i + 1) % room_polygon.size()];
            const Eigen::Vector3f a0 = world_to_camera(world_to_robot * a, 0.f);
            const Eigen::Vector3f b0 = world_to_camera(world_to_robot * b, 0.f);
            const Eigen::Vector3f a1 = world_to_camera(world_to_robot * a, wall_height_);

            const Eigen::Vector3f edge_u = b0 - a0;
            const Eigen::Vector3f edge_v = a1 - a0;
            const float edge_u_sq = edge_u.squaredNorm();
            const float edge_v_sq = edge_v.squaredNorm();
            if (edge_u_sq < 1e-9f || edge_v_sq < 1e-9f)
                continue;

            Eigen::Vector3f normal = edge_u.cross(edge_v);
            const float n_norm = normal.norm();
            if (n_norm < 1e-9f)
                continue;
            normal /= n_norm;

            wall_planes.push_back(WallPlane{a0, edge_u, edge_u_sq, edge_v, edge_v_sq, normal, -normal.dot(a0)});
        }
    }

    auto object_visible = [&](const FurniturePolygonData& fp) -> bool
    {
        if (fp.vertices.empty())
            return false;

        Eigen::Vector2f c = Eigen::Vector2f::Zero();
        for (const auto& v : fp.vertices)
            c += v;
        c /= static_cast<float>(fp.vertices.size());

        const float h = std::max(0.2f, fp.height);
        const Eigen::Vector3f p_cam = world_to_camera(world_to_robot * c, 0.5f * h);
        constexpr float near = 0.05f;
        if (p_cam.y() <= near)
            return false;

        // Ray from camera origin to object center (p_cam = ray at t=1).
        const Eigen::Vector3f ray = p_cam;
        float best_t = std::numeric_limits<float>::infinity();
        for (const auto& wp : wall_planes)
        {
            const float den = wp.normal.dot(ray);
            if (std::abs(den) < 1e-7f)
                continue;

            const float t = -wp.plane_d / den;
            if (t <= near)
                continue;

            const Eigen::Vector3f hit = t * ray;
            const Eigen::Vector3f rel = hit - wp.a0;
            const float alpha = rel.dot(wp.edge_u) / wp.edge_u_sq;
            const float beta = rel.dot(wp.edge_v) / wp.edge_v_sq;
            if (alpha < -1e-4f || alpha > 1.f + 1e-4f)
                continue;
            if (beta < -1e-4f || beta > 1.f + 1e-4f)
                continue;

            best_t = std::min(best_t, t);
        }

        // Visible if no wall hit, or first wall is not in front of the object center.
        return !std::isfinite(best_t) || best_t >= 1.f - 1e-3f;
    };

    // 3. Furniture wireframes
    for (int i = 0; i < static_cast<int>(furniture.size()); ++i)
    {
        const auto& fp = furniture[i];
        if (!object_visible(fp))
            continue;
        const float area = draw_furniture_wireframe(painter, fp, world_to_robot, furniture_color);
        if (area > 0.f)
        {
            result.visible_furniture_indices.push_back(i);
            result.visible_areas.push_back(area);
        }
    }

    // 4. Labels for visible furniture
    painter.setPen(QPen(QColor(255, 255, 255, 220), 1.0));
    QFont font;
    font.setPixelSize(11);
    font.setBold(true);
    painter.setFont(font);
    for (std::size_t vi = 0; vi < result.visible_furniture_indices.size(); ++vi)
    {
        const auto& fp = furniture[result.visible_furniture_indices[vi]];
        // Compute centroid in camera frame
        Eigen::Vector2f c = Eigen::Vector2f::Zero();
        for (const auto& v : fp.vertices) c += v;
        if (!fp.vertices.empty()) c /= static_cast<float>(fp.vertices.size());
        const auto pc = world_to_camera(world_to_robot * c, fp.height + 0.05f);
        float u, v;
        if (project(pc, u, v) && u > 0.f && u < static_cast<float>(intr_.width) &&
            v > 0.f && v < static_cast<float>(intr_.height))
        {
            const QString text = QString::fromStdString(fp.label.empty() ? fp.id : fp.label);
            painter.drawText(QPointF(u - 30.f, v - 4.f), text);
        }
    }

    painter.end();
    return result;
}

} // namespace rc
