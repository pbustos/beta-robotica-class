#include "em_manager.h"
#include "object_geometry.h"

#include "camera_viewer.h"
#include "viewer_2d.h"
#include "object_models/table_analytic_model.h"

#include <QCoreApplication>
#include <QLabel>
#include <QString>
#include <QThread>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <unordered_map>

#include <torch/torch.h>

// ---------------------------------------------------------------------------
// model_height_from_label  (static utility)
// ---------------------------------------------------------------------------
float EMManager::model_height_from_label(const std::string& label)
{
    return rc::geometry::height_from_label(label);
}

// ---------------------------------------------------------------------------
// extract_depth_buffer_meters  (static)
// ---------------------------------------------------------------------------
bool EMManager::extract_depth_buffer_meters(const RoboCompImageSegmentation::TDepth& depth,
                                            int& width, int& height,
                                            std::vector<float>& out)
{
    width = depth.width;
    height = depth.height;
    if (width <= 0 || height <= 0)
        return false;

    const std::size_t expected = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (expected == 0)
        return false;

    const std::size_t nbytes = depth.depth.size();
    const float scale = (std::abs(depth.depthFactor) > 1e-9f) ? depth.depthFactor : 1.f;

    out.assign(expected, 0.f);

    if (nbytes >= expected * sizeof(float))
    {
        for (std::size_t i = 0; i < expected; ++i)
        {
            float v = 0.f;
            std::memcpy(&v, depth.depth.data() + i * sizeof(float), sizeof(float));
            out[i] = std::isfinite(v) ? v * scale : 0.f;
        }
    }
    else if (nbytes >= expected * sizeof(std::uint16_t))
    {
        for (std::size_t i = 0; i < expected; ++i)
        {
            std::uint16_t mm = 0;
            std::memcpy(&mm, depth.depth.data() + i * sizeof(std::uint16_t), sizeof(std::uint16_t));
            const float v = static_cast<float>(mm);
            out[i] = std::isfinite(v) ? v : 0.f;
        }
    }
    else if (nbytes >= expected)
    {
        for (std::size_t i = 0; i < expected; ++i)
        {
            const float v = static_cast<float>(depth.depth[i]);
            out[i] = std::isfinite(v) ? v : 0.f;
        }
    }
    else
        return false;

    if (nbytes < expected * sizeof(float))
    {
        for (float& d : out)
            d = std::isfinite(d) ? d * 0.001f * scale : 0.f;
    }

    return true;
}

// ---------------------------------------------------------------------------
// extract_depth_from_tdata  (static)
// ---------------------------------------------------------------------------
bool EMManager::extract_depth_from_tdata(const RoboCompImageSegmentation::TData& tdata,
                                         int& width, int& height,
                                         std::vector<float>& out)
{
    return extract_depth_buffer_meters(tdata.depth, width, height, out);
}

// ---------------------------------------------------------------------------
// rebuild_models  (ownership EM)
// ---------------------------------------------------------------------------
void EMManager::rebuild_models()
{
    if (!enabled_ || !ctx_.furniture_polygons)
        return;

    const auto& furniture_polygons = *ctx_.furniture_polygons;

    rc::ObjectOwnershipEM::Config cfg;
    cfg.max_iterations = 3;
    cfg.convergence_eps = 1e-3f;
    cfg.anneal_t_start = 2.0f;
    cfg.anneal_t_end = 0.8f;
    cfg.robust_sigma = 0.30f;
    cfg.max_pose_jump_xy = 0.30f;
    cfg.max_pose_jump_yaw = 0.20f;
    ownership_em_.configure(cfg);

    std::vector<rc::ObjectOwnershipEM::ClassModel> models;
    models.reserve(furniture_polygons.size() + 1);
    std::vector<rc::ObjectOwnershipEM::ClassState> states;
    states.reserve(furniture_polygons.size() + 1);

    auto expected_height = [](const std::string& label) -> float
    {
        return rc::geometry::height_from_label(label);
    };

    for (const auto& fp : furniture_polygons)
    {
        if (fp.vertices.size() < 3)
            continue;

        Eigen::Vector2f cen = Eigen::Vector2f::Zero();
        for (const auto& v : fp.vertices) cen += v;
        cen /= static_cast<float>(fp.vertices.size());

        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (const auto& v : fp.vertices)
        {
            const Eigen::Vector2f d = v - cen;
            cov += d * d.transpose();
        }
        cov /= static_cast<float>(fp.vertices.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
        Eigen::Vector2f axis_x(1.f, 0.f);
        if (eig.info() == Eigen::Success)
            axis_x = eig.eigenvectors().col(1).normalized();

        rc::ObjectOwnershipEM::ClassModel m;
        m.class_id = fp.label.empty() ? fp.id : fp.label;
        m.prior_weight = 1.0f;
        m.expected_height = expected_height(fp.label.empty() ? fp.id : fp.label);
        models.emplace_back(std::move(m));

        rc::ObjectOwnershipEM::ClassState s;
        s.class_id = fp.label.empty() ? fp.id : fp.label;
        s.position = Eigen::Vector3f(cen.x(), cen.y(), 0.5f * expected_height(fp.label.empty() ? fp.id : fp.label));
        s.yaw_rad = std::atan2(axis_x.y(), axis_x.x());
        s.scale = 1.0f;
        s.confidence = 0.5f;
        s.active = true;
        states.emplace_back(std::move(s));
    }

    rc::ObjectOwnershipEM::ClassModel bg;
    bg.class_id = "background";
    bg.is_background = true;
    bg.prior_weight = 0.8f;
    bg.expected_height = 0.0f;
    models.emplace_back(std::move(bg));

    rc::ObjectOwnershipEM::ClassState bg_state;
    bg_state.class_id = "background";
    bg_state.active = true;
    bg_state.confidence = 0.5f;
    states.emplace_back(std::move(bg_state));

    ownership_em_.set_models(models);
    ownership_em_.set_initial_states(states);
}

// ---------------------------------------------------------------------------
// run_ownership_step  (lidar-based, per-frame)
// ---------------------------------------------------------------------------
void EMManager::run_ownership_step(const std::vector<Eigen::Vector3f>& points_robot,
                                   const Eigen::Affine2f& robot_pose)
{
    if (!enabled_ || points_robot.empty())
        return;

    rc::ObjectOwnershipEM::ObservationBatch batch;
    batch.robot_pose = robot_pose;
    batch.timestamp_ms = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    const float c = std::cos(std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0)));
    const float s = std::sin(std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0)));
    const float rx = robot_pose.translation().x();
    const float ry = robot_pose.translation().y();

    constexpr std::size_t max_points = 1800;
    const std::size_t stride = std::max<std::size_t>(1, points_robot.size() / max_points);
    batch.points_world.reserve(points_robot.size() / stride + 1);

    for (std::size_t i = 0; i < points_robot.size(); i += stride)
    {
        const auto& p = points_robot[i];
        const float wx = c * p.x() - s * p.y() + rx;
        const float wy = s * p.x() + c * p.y() + ry;
        batch.points_world.emplace_back(wx, wy, p.z());
    }

    ownership_em_.set_observations(batch);
    if (!ownership_em_.run_single_iteration())
        return;

    apply_visuals();

    if (++log_counter_ >= 40)
    {
        log_counter_ = 0;
        const auto& m = ownership_em_.get_metrics();
        qInfo() << "[OwnershipEM] objective=" << m.objective
                << "entropy=" << m.avg_entropy
                << "bg=" << m.background_fraction;
    }
}

// ---------------------------------------------------------------------------
// apply_visuals  (ownership EM → 2D viewer)
// ---------------------------------------------------------------------------
void EMManager::apply_visuals()
{
    if (!enabled_ || !ctx_.furniture_polygons || !ctx_.viewer_2d)
        return;

    const auto& furniture_polygons = *ctx_.furniture_polygons;
    if (furniture_polygons.empty() || ctx_.viewer_2d->furniture_count() == 0)
        return;

    std::unordered_map<std::string, float> confidence_by_id;
    confidence_by_id.reserve(ownership_em_.get_states().size() * 2);
    for (const auto& st : ownership_em_.get_states())
    {
        if (!st.active)
            continue;
        confidence_by_id[st.class_id] = std::clamp(st.confidence, 0.f, 1.f);
    }

    const QColor base(50, 100, 255);
    for (std::size_t i = 0; i < furniture_polygons.size() && i < ctx_.viewer_2d->furniture_count(); ++i)
    {
        const auto& fp = furniture_polygons[i];
        float conf = 0.35f;
        if (const auto it = confidence_by_id.find(fp.label); it != confidence_by_id.end())
            conf = it->second;
        else if (const auto it2 = confidence_by_id.find(fp.id); it2 != confidence_by_id.end())
            conf = it2->second;

        const qreal width = 0.04 + 0.10 * static_cast<qreal>(conf);
        const int alpha = static_cast<int>(35.f + 165.f * conf);
        ctx_.viewer_2d->set_furniture_item_style(
            i,
            QPen(base, width),
            QBrush(QColor(base.red(), base.green(), base.blue(), alpha)));
    }
}

// ---------------------------------------------------------------------------
// clear_overlay
// ---------------------------------------------------------------------------
void EMManager::clear_overlay()
{
    if (!ctx_.camera_viewer)
        return;

    decision_pending_ = false;
    pending_adjustments_.clear();
    ctx_.camera_viewer->set_em_decision_buttons_visible(false);
    ctx_.camera_viewer->clear_wireframe_overlay();
    ctx_.camera_viewer->clear_em_points_overlay();
    overlay_persistent_ = false;
    overlay_lock_ = false;
}

// ---------------------------------------------------------------------------
// run_camera_validator  (button-triggered, ~1400 lines)
// ---------------------------------------------------------------------------
void EMManager::run_camera_validator(const Eigen::Affine2f& robot_pose,
                                     const RoboCompImageSegmentation::TData& tdata,
                                     const std::vector<int>& visible_indices)
{
    if (!ctx_.camera_viewer)
        return;

    decision_pending_ = false;
    pending_adjustments_.clear();
    ctx_.camera_viewer->set_em_decision_buttons_visible(false);

    overlay_lock_ = true;
    bool keep_overlay_lock = false;
    struct OverlayUnlock
    {
        bool &flag;
        bool &keep;
        ~OverlayUnlock() { if (!keep) flag = false; }
    } overlay_unlock{overlay_lock_, keep_overlay_lock};
    overlay_persistent_ = false;
    ctx_.camera_viewer->clear_em_points_overlay();

    if (ctx_.status_label)
        ctx_.status_label->setText("Status: running EM validator...");

    auto& furniture_polygons = *ctx_.furniture_polygons;
    const auto& room_polygon = *ctx_.room_polygon;

    auto centroid_of = [](const rc::FurniturePolygonData& fp) -> Eigen::Vector2f
    {
        Eigen::Vector2f c = Eigen::Vector2f::Zero();
        if (fp.vertices.empty())
            return c;
        for (const auto& v : fp.vertices) c += v;
        c /= static_cast<float>(fp.vertices.size());
        return c;
    };

    // --- point cloud from depth ---
    std::vector<Eigen::Vector3f> points_cam_all;
    std::vector<Eigen::Vector3f> points_world_all;
    points_cam_all.reserve(12000);
    points_world_all.reserve(12000);

    int depth_w = 0;
    int depth_h = 0;
    std::vector<float> depth_m;
    const bool have_depth = extract_depth_from_tdata(tdata, depth_w, depth_h, depth_m);

    const auto &timg = tdata.image;
    const int img_w = std::max(1, timg.width);
    const int img_h = std::max(1, timg.height);
    float fx = static_cast<float>(timg.focalx);
    float fy = static_cast<float>(timg.focaly);
    const float max_reasonable_fx = static_cast<float>(img_w) * 5.f;
    const float max_reasonable_fy = static_cast<float>(img_h) * 5.f;
    if (!std::isfinite(fx) || fx < 10.f || fx > max_reasonable_fx) fx = static_cast<float>(img_w) * 0.9f;
    if (!std::isfinite(fy) || fy < 10.f || fy > max_reasonable_fy) fy = static_cast<float>(img_h) * 0.9f;

    if (have_depth && depth_w > 0 && depth_h > 0)
    {
        const float cx_d = static_cast<float>(depth_w) * 0.5f;
        const float cy_d = static_cast<float>(depth_h) * 0.5f;
        const float fx_d = (depth_w == img_w) ? fx : (fx * static_cast<float>(depth_w) / static_cast<float>(img_w));
        const float fy_d = (depth_h == img_h) ? fy : (fy * static_cast<float>(depth_h) / static_cast<float>(img_h));

        constexpr float z_min = 0.08f;
        constexpr float z_max = 8.0f;
        const int img_channels = std::max(1, timg.depth);
        const std::size_t img_expected = static_cast<std::size_t>(img_w) * static_cast<std::size_t>(img_h) * static_cast<std::size_t>(img_channels);
        const bool has_rgb = timg.image.size() >= img_expected;

        for (int v = 0; v < depth_h; ++v)
        {
            for (int u = 0; u < depth_w; ++u)
            {
                const float d = depth_m[static_cast<std::size_t>(v) * static_cast<std::size_t>(depth_w) + static_cast<std::size_t>(u)];
                if (!std::isfinite(d) || d < z_min || d > z_max)
                    continue;

                if (has_rgb)
                {
                    const int u_img = std::clamp((u * img_w) / std::max(1, depth_w), 0, img_w - 1);
                    const int v_img = std::clamp((v * img_h) / std::max(1, depth_h), 0, img_h - 1);
                    const std::size_t pix = (static_cast<std::size_t>(v_img) * static_cast<std::size_t>(img_w)
                                           + static_cast<std::size_t>(u_img)) * static_cast<std::size_t>(img_channels);
                    if (pix + 2 < timg.image.size())
                    {
                        const unsigned char c0 = static_cast<unsigned char>(timg.image[pix + 0]);
                        const unsigned char c1 = static_cast<unsigned char>(timg.image[pix + 1]);
                        const unsigned char c2 = static_cast<unsigned char>(timg.image[pix + 2]);
                        if (c0 == 0 && c1 == 0 && c2 == 0)
                            continue;
                    }
                }

                const float x_cam = (static_cast<float>(u) - cx_d) * d / std::max(1e-5f, fx_d);
                const float y_cam = d;
                const float z_cam = (cy_d - static_cast<float>(v)) * d / std::max(1e-5f, fy_d);

                const Eigen::Vector3f p_cam(x_cam, y_cam, z_cam);
                const float x_robot = x_cam + ctx_.camera_tx;
                const float y_robot = y_cam + ctx_.camera_ty;
                const float z_robot = z_cam + ctx_.camera_tz;
                const Eigen::Vector2f p_world_xy = robot_pose * Eigen::Vector2f(x_robot, y_robot);
                const Eigen::Vector3f p_world(p_world_xy.x(), p_world_xy.y(), z_robot);
                points_cam_all.emplace_back(p_cam);
                points_world_all.emplace_back(p_world);
            }
        }
    }

    const bool enough_points = points_world_all.size() >= 30;

    std::vector<Eigen::Vector3f> points_cam = points_cam_all;
    std::vector<Eigen::Vector3f> points_world = points_world_all;

    if (furniture_polygons.empty())
    {
        if (ctx_.status_label)
            ctx_.status_label->setText("Status: no map furniture for EM classes");
        return;
    }

    const Eigen::Affine2f world_to_robot = robot_pose.inverse();
    const float cx = static_cast<float>(img_w) * 0.5f;
    const float cy = static_cast<float>(img_h) * 0.5f;
    constexpr float near_depth = 0.05f;

    // --- candidate selection (frustum-polygon intersection on the ground plane) ---
    //
    // Build the camera's horizontal frustum as a trapezoid in world coords,
    // then test each furniture footprint for proper 2-D polygon intersection.
    // This is more robust than projecting corners and checking bbox overlap
    // because it correctly handles partial overlaps and edge-only visibility.

    constexpr float frustum_near = 0.30f;   // metres – skip very close objects
    constexpr float frustum_far  = 6.0f;    // metres – maximum range

    const float left_slope  = -cx / std::max(1.f, fx);                                    // x_cam / depth for u = 0
    const float right_slope = (static_cast<float>(img_w) - cx) / std::max(1.f, fx);       // x_cam / depth for u = img_w

    // Four corners of the trapezoid in the robot frame (CCW winding).
    const Eigen::Vector2f fn_l(ctx_.camera_tx + left_slope  * frustum_near, ctx_.camera_ty + frustum_near);
    const Eigen::Vector2f fn_r(ctx_.camera_tx + right_slope * frustum_near, ctx_.camera_ty + frustum_near);
    const Eigen::Vector2f ff_r(ctx_.camera_tx + right_slope * frustum_far,  ctx_.camera_ty + frustum_far);
    const Eigen::Vector2f ff_l(ctx_.camera_tx + left_slope  * frustum_far,  ctx_.camera_ty + frustum_far);

    // Transform to world coordinates.
    const std::vector<Eigen::Vector2f> frustum_world = {
        robot_pose * fn_l, robot_pose * fn_r, robot_pose * ff_r, robot_pose * ff_l
    };

    qInfo() << "  [EM] frustum trapezoid (world):"
             << "NL(" << frustum_world[0].x() << frustum_world[0].y() << ")"
             << "NR(" << frustum_world[1].x() << frustum_world[1].y() << ")"
             << "FR(" << frustum_world[2].x() << frustum_world[2].y() << ")"
             << "FL(" << frustum_world[3].x() << frustum_world[3].y() << ")";

    // 2-D polygon–polygon intersection test.
    auto polygons_intersect_2d = [](const std::vector<Eigen::Vector2f>& A,
                                    const std::vector<Eigen::Vector2f>& B) -> bool
    {
        // Point-in-polygon (ray-casting parity test).
        auto pip = [](const Eigen::Vector2f& p, const std::vector<Eigen::Vector2f>& poly) -> bool
        {
            bool inside = false;
            for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
            {
                const auto& pi = poly[i];
                const auto& pj = poly[j];
                if (((pi.y() > p.y()) != (pj.y() > p.y())) &&
                    (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) /
                              (pj.y() - pi.y()) + pi.x()))
                    inside = !inside;
            }
            return inside;
        };

        // Proper segment–segment intersection (no touch).
        auto segs_cross = [](const Eigen::Vector2f& a1, const Eigen::Vector2f& a2,
                             const Eigen::Vector2f& b1, const Eigen::Vector2f& b2) -> bool
        {
            auto cross2 = [](const Eigen::Vector2f& u, const Eigen::Vector2f& v)
            { return u.x() * v.y() - u.y() * v.x(); };
            const Eigen::Vector2f d1 = a2 - a1, d2 = b2 - b1, d3 = b1 - a1;
            const float denom = cross2(d1, d2);
            if (std::abs(denom) < 1e-10f) return false;
            const float t = cross2(d3, d2) / denom;
            const float s = cross2(d3, d1) / denom;
            constexpr float eps = 1e-6f;
            return (t > eps && t < 1.f - eps && s > eps && s < 1.f - eps);
        };

        // Case 1: any vertex of A inside B.
        for (const auto& v : A) if (pip(v, B)) return true;
        // Case 2: any vertex of B inside A.
        for (const auto& v : B) if (pip(v, A)) return true;
        // Case 3: any edge of A crosses any edge of B.
        for (std::size_t i = 0; i < A.size(); ++i)
            for (std::size_t j = 0; j < B.size(); ++j)
                if (segs_cross(A[i], A[(i + 1) % A.size()], B[j], B[(j + 1) % B.size()]))
                    return true;
        return false;
    };

    std::vector<int> candidate_indices;
    candidate_indices.reserve(furniture_polygons.size());
    const Eigen::Vector2f robot_xy = robot_pose.translation();

    if (!visible_indices.empty())
    {
        // Use pre-computed visibility from the synthetic camera renderer.
        for (int idx : visible_indices)
        {
            if (idx >= 0 && idx < static_cast<int>(furniture_polygons.size()) &&
                furniture_polygons[idx].vertices.size() >= 3)
                candidate_indices.push_back(idx);
        }
        qInfo() << "  [EM] using synthetic renderer visibility:" << candidate_indices.size() << "candidates";
    }
    else
    {
    for (std::size_t i = 0; i < furniture_polygons.size(); ++i)
    {
        const auto& fp = furniture_polygons[i];
        if (fp.vertices.size() < 3)
            continue;

        if (polygons_intersect_2d(fp.vertices, frustum_world))
            candidate_indices.push_back(static_cast<int>(i));
        else
        {
            const Eigen::Vector2f c = centroid_of(fp);
            const float dist = (c - robot_xy).norm();
            if (dist < 8.0f)   // only log nearby rejects to reduce noise
                qInfo() << "  [EM] frustum-reject:" << QString::fromStdString(fp.label.empty() ? fp.id : fp.label)
                         << "centroid=(" << c.x() << c.y() << ") nverts=" << fp.vertices.size()
                         << "v0=(" << fp.vertices[0].x() << fp.vertices[0].y() << ")";
        }
    }
    } // else (frustum fallback)

    // --- line-of-sight filter: discard objects behind walls ---
    if (!room_polygon.empty() && room_polygon.size() >= 3)
    {
        // Proper segment–segment intersection test (same as PolygonPathPlanner::segments_intersect_proper)
        auto seg_cross = [](const Eigen::Vector2f& a1, const Eigen::Vector2f& a2,
                            const Eigen::Vector2f& b1, const Eigen::Vector2f& b2) -> bool
        {
            auto cross2 = [](const Eigen::Vector2f& u, const Eigen::Vector2f& v)
            { return u.x() * v.y() - u.y() * v.x(); };
            const Eigen::Vector2f d1 = a2 - a1, d2 = b2 - b1, d3 = b1 - a1;
            const float denom = cross2(d1, d2);
            if (std::abs(denom) < 1e-10f) return false;
            const float t = cross2(d3, d2) / denom;
            const float u = cross2(d3, d1) / denom;
            constexpr float eps = 1e-5f;
            return (t > eps && t < 1.f - eps && u > eps && u < 1.f - eps);
        };

        auto crosses_wall = [&](const Eigen::Vector2f& target) -> bool
        {
            // Use crossing-number parity: for a non-convex room polygon a
            // line between two interior points may cross an even number of
            // edges (through concavities).  An odd count means the target
            // centroid is actually outside the room (behind a wall).
            int crossings = 0;
            const int n = static_cast<int>(room_polygon.size());
            for (int i = 0; i < n; ++i)
                if (seg_cross(robot_xy, target, room_polygon[i], room_polygon[(i + 1) % n]))
                    ++crossings;
            return (crossings % 2) != 0;
        };

        candidate_indices.erase(
            std::remove_if(candidate_indices.begin(), candidate_indices.end(),
                [&](int idx)
                {
                    bool occluded = crosses_wall(centroid_of(furniture_polygons[idx]));
                    if (occluded)
                        qInfo() << "  [EM] wall-occlude:" << QString::fromStdString(
                            furniture_polygons[idx].label.empty() ? furniture_polygons[idx].id : furniture_polygons[idx].label);
                    return occluded;
                }),
            candidate_indices.end());
    }

    // Log surviving candidates
    {
        QStringList names;
        for (int idx : candidate_indices)
            names << QString::fromStdString(furniture_polygons[idx].label.empty()
                                            ? furniture_polygons[idx].id : furniture_polygons[idx].label);
        qInfo() << "  [EM] candidates (" << candidate_indices.size() << "):" << names.join(", ");
    }

    if (candidate_indices.empty())
    {
        if (ctx_.status_label)
            ctx_.status_label->setText("Status: no model objects inside camera frustum");
        return;
    }

    // --- EM class setup ---
    struct EMClass
    {
        int furniture_index = -1;
        std::string label;
        float height = 0.8f;
        float z_base = 0.f;            // floor z — objects start on the ground
        std::vector<Eigen::Vector2f> vertices;
        float last_sdf = 0.f;
        std::optional<float> prev_sdf;
    };

    std::vector<EMClass> classes;
    classes.reserve(candidate_indices.size());
    for (int idx : candidate_indices)
    {
        const auto& fp = furniture_polygons[idx];
        EMClass c;
        c.furniture_index = idx;
        c.label = fp.label.empty() ? fp.id : fp.label;
        c.height = std::max(0.2f, fp.height);
        c.z_base = 0.f;
        c.vertices = fp.vertices;
        c.prev_sdf = fp.last_fit_sdf.has_value() ? fp.last_fit_sdf : std::optional<float>(1.0f);
        classes.emplace_back(std::move(c));
    }

    const int K = static_cast<int>(classes.size());
    const int N = static_cast<int>(points_world.size());
    if (K <= 0)
        return;

    std::vector<QString> class_labels;
    std::vector<QColor> class_colors;
    class_labels.reserve(K + 1);
    class_colors.reserve(K + 1);
    QStringList visible_model_names;
    visible_model_names.reserve(K);
    for (int k = 0; k < K; ++k)
    {
        class_labels.emplace_back(QString::fromStdString(classes[static_cast<std::size_t>(k)].label));
        class_colors.emplace_back(QColor::fromHsv((47 * k) % 360, 220, 255, 255));
        visible_model_names << class_labels.back();
    }
    class_labels.emplace_back("background");
    class_colors.emplace_back(QColor(96, 96, 96, 255));

    const QString visible_names_text = visible_model_names.isEmpty()
        ? QString("Visible models: -")
        : QString("Visible models: %1").arg(visible_model_names.join(", "));
    const QColor em_wireframe_color(255, 255, 255, 255);

    // --- distance helpers ---
    auto point_seg_distance = [](const Eigen::Vector2f& p,
                                 const Eigen::Vector2f& a,
                                 const Eigen::Vector2f& b) -> float
    {
        const Eigen::Vector2f ab = b - a;
        const float ab2 = std::max(1e-9f, ab.squaredNorm());
        const float t = std::clamp((p - a).dot(ab) / ab2, 0.f, 1.f);
        const Eigen::Vector2f q = a + t * ab;
        return (p - q).norm();
    };

    auto point_polygon_distance = [&](const Eigen::Vector2f& p,
                                      const std::vector<Eigen::Vector2f>& poly) -> float
    {
        if (poly.size() < 2)
            return 1e3f;
        float best = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < poly.size(); ++i)
        {
            const auto& a = poly[i];
            const auto& b = poly[(i + 1) % poly.size()];
            best = std::min(best, point_seg_distance(p, a, b));
        }
        return best;
    };

    // --- prev_sdf estimation for tables ---
    for (auto& c : classes)
    {
        if (c.prev_sdf.has_value() && std::isfinite(c.prev_sdf.value()) && c.prev_sdf.value() > 0.f)
            continue;

        const QString label_lc = QString::fromStdString(c.label).toLower();
        if (!(label_lc.contains("mesa") || label_lc.contains("table")))
            continue;
        if (c.vertices.size() < 3)
            continue;

        Eigen::Vector2f cen = Eigen::Vector2f::Zero();
        for (const auto& v : c.vertices) cen += v;
        cen /= static_cast<float>(c.vertices.size());

        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (const auto& v : c.vertices)
        {
            const Eigen::Vector2f d = v - cen;
            cov += d * d.transpose();
        }
        cov /= static_cast<float>(std::max<std::size_t>(1, c.vertices.size()));
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
        Eigen::Vector2f axis_x(1.f, 0.f);
        if (eig.info() == Eigen::Success)
            axis_x = eig.eigenvectors().col(1).normalized();
        const float yaw = std::atan2(axis_x.y(), axis_x.x());

        Eigen::Vector2f axis_z(-axis_x.y(), axis_x.x());
        float min_u = std::numeric_limits<float>::max();
        float max_u = -std::numeric_limits<float>::max();
        float min_v = std::numeric_limits<float>::max();
        float max_v = -std::numeric_limits<float>::max();
        for (const auto& p : c.vertices)
        {
            const Eigen::Vector2f d = p - cen;
            const float u = d.dot(axis_x);
            const float v = d.dot(axis_z);
            min_u = std::min(min_u, u); max_u = std::max(max_u, u);
            min_v = std::min(min_v, v); max_v = std::max(max_v, v);
        }
        const float w = std::max(0.08f, max_u - min_u);
        const float d = std::max(0.08f, max_v - min_v);
        const float h = std::max(0.20f, c.height);

        std::vector<float> xyz;
        xyz.reserve(points_world.size() * 3);
        const float select_dist = std::max(0.35f, 0.60f * std::max(w, d));
        const float radial_gate = std::max(0.80f, 1.40f * std::max(w, d));
        for (const auto& pw : points_world)
        {
            const float dist = point_polygon_distance(pw.head<2>(), c.vertices);
            const float radial = (pw.head<2>() - cen).norm();
            if (dist > select_dist && radial > radial_gate)
                continue;
            if (pw.z() < -0.10f || pw.z() > h + 0.60f)
                continue;
            xyz.push_back(pw.x());
            xyz.push_back(pw.y());
            xyz.push_back(pw.z());
        }

        if (xyz.size() < 15 * 3)
            continue;

        auto pts = torch::from_blob(xyz.data(),
                                    {static_cast<long>(xyz.size() / 3), 3},
                                    torch::TensorOptions().dtype(torch::kFloat32)).clone();
        auto params = torch::tensor({cen.x(), cen.y(), yaw, w, d, h},
                                    torch::TensorOptions().dtype(torch::kFloat32));
        auto sdf = rc::object_models::TableAnalyticModel::forward_sdf(pts, params);
        const float mse = torch::mean(torch::square(sdf)).item<float>();
        const float mae = torch::mean(torch::abs(sdf)).item<float>();
        const float prev_est = 0.5f * mae + 0.5f * mse;
        if (std::isfinite(prev_est) && prev_est > 0.f)
            c.prev_sdf = std::max(0.01f, prev_est);
    }

    float prev_sdf_sum = 0.f;
    int prev_sdf_count = 0;
    for (const auto& c : classes)
    {
        if (c.prev_sdf.has_value() && std::isfinite(c.prev_sdf.value()) && c.prev_sdf.value() > 0.f)
        {
            prev_sdf_sum += c.prev_sdf.value();
            ++prev_sdf_count;
        }
    }
    const float prev_mean_sdf = (prev_sdf_count > 0) ? (prev_sdf_sum / static_cast<float>(prev_sdf_count)) : 0.f;

    // --- draw initial wireframes ---
    {
        auto to_camera = [&](const Eigen::Vector2f& p_robot, float z_world) -> Eigen::Vector3f
        {
            return Eigen::Vector3f(p_robot.x() - ctx_.camera_tx,
                                   p_robot.y() - ctx_.camera_ty,
                                   z_world - ctx_.camera_tz);
        };

        auto oriented_box_corners = [](const std::vector<Eigen::Vector2f>& poly) -> std::array<Eigen::Vector2f, 4>
        {
            std::array<Eigen::Vector2f, 4> out{Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()};
            if (poly.empty())
                return out;

            // Furniture vertices from the scene graph are already properly
            // oriented 4-corner rectangles — use them directly to avoid
            // PCA instability on square footprints (e.g. chairs).
            if (poly.size() == 4)
            {
                for (int i = 0; i < 4; ++i) out[i] = poly[i];
                return out;
            }

            Eigen::Vector2f c = Eigen::Vector2f::Zero();
            for (const auto& p : poly) c += p;
            c /= static_cast<float>(poly.size());

            Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
            for (const auto& p : poly)
            {
                const Eigen::Vector2f d = p - c;
                cov += d * d.transpose();
            }
            cov /= static_cast<float>(std::max<std::size_t>(1, poly.size()));

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
            Eigen::Vector2f u(1.f, 0.f);
            if (eig.info() == Eigen::Success)
                u = eig.eigenvectors().col(1).normalized();
            Eigen::Vector2f v(-u.y(), u.x());

            float umin = std::numeric_limits<float>::max();
            float umax = -std::numeric_limits<float>::max();
            float vmin = std::numeric_limits<float>::max();
            float vmax = -std::numeric_limits<float>::max();
            for (const auto& p : poly)
            {
                const Eigen::Vector2f d = p - c;
                const float pu = d.dot(u);
                const float pv = d.dot(v);
                umin = std::min(umin, pu);
                umax = std::max(umax, pu);
                vmin = std::min(vmin, pv);
                vmax = std::max(vmax, pv);
            }

            out[0] = c + u * umin + v * vmin;
            out[1] = c + u * umax + v * vmin;
            out[2] = c + u * umax + v * vmax;
            out[3] = c + u * umin + v * vmax;
            return out;
        };

        auto append_mesh_like_wireframe = [&](const std::vector<Eigen::Vector2f>& poly,
                                              const std::string& label,
                                              float h,
                                              std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& out_segments,
                                              std::vector<QColor>& out_colors,
                                              const QColor& color)
        {
            if (poly.size() < 3)
                return;

            const auto corners = oriented_box_corners(poly);
            const auto& c0 = corners[0];
            const auto& c1 = corners[1];
            const auto& c2 = corners[2];
            const auto& c3 = corners[3];

            auto add = [&](const Eigen::Vector2f& a, float za, const Eigen::Vector2f& b, float zb)
            {
                const Eigen::Vector2f ar = world_to_robot * a;
                const Eigen::Vector2f br = world_to_robot * b;
                out_segments.emplace_back(to_camera(ar, za), to_camera(br, zb));
                out_colors.emplace_back(color);
            };

            const Eigen::Vector2f center = 0.25f * (c0 + c1 + c2 + c3);
            const Eigen::Vector2f front_mid = 0.5f * (c0 + c1);
            const Eigen::Vector2f back_mid  = 0.5f * (c2 + c3);
            const Eigen::Vector2f left_mid  = 0.5f * (c0 + c3);
            const Eigen::Vector2f right_mid = 0.5f * (c1 + c2);
            const QString ql = QString::fromStdString(label).toLower();

            if (ql.contains("mesa") || ql.contains("table"))
            {
                const Eigen::Vector2f center = 0.25f * (c0 + c1 + c2 + c3);
                Eigen::Vector2f ux = c1 - c0;
                Eigen::Vector2f uy = c3 - c0;
                const float w = std::max(0.08f, ux.norm());
                const float d = std::max(0.08f, uy.norm());
                if (ux.norm() < 1e-6f) ux = Eigen::Vector2f(1.f, 0.f); else ux.normalize();
                if (uy.norm() < 1e-6f) uy = Eigen::Vector2f(-ux.y(), ux.x()); else uy.normalize();

                const float h_table = std::max(0.20f, h);
                const float top_t = std::max(0.03f, 0.08f * h_table);
                const float leg_t = std::max(0.03f, 0.12f * std::min(w, d));
                const float leg_h = std::max(0.05f, h_table - top_t);
                const float leg_dx = std::max(0.f, 0.5f * w - 0.5f * leg_t);
                const float leg_dy = std::max(0.f, 0.5f * d - 0.5f * leg_t);

                auto local_to_world = [&](float lx, float ly) -> Eigen::Vector2f
                {
                    return center + ux * lx + uy * ly;
                };

                auto add_cuboid_wire = [&](float lx, float ly, float sx, float sy, float sz, float cz)
                {
                    const float hx = 0.5f * sx;
                    const float hy = 0.5f * sy;
                    const float hz = 0.5f * sz;
                    const float z0 = cz - hz;
                    const float z1 = cz + hz;

                    const Eigen::Vector2f p00 = local_to_world(lx - hx, ly - hy);
                    const Eigen::Vector2f p10 = local_to_world(lx + hx, ly - hy);
                    const Eigen::Vector2f p11 = local_to_world(lx + hx, ly + hy);
                    const Eigen::Vector2f p01 = local_to_world(lx - hx, ly + hy);

                    add(p00, z0, p10, z0); add(p10, z0, p11, z0); add(p11, z0, p01, z0); add(p01, z0, p00, z0);
                    add(p00, z1, p10, z1); add(p10, z1, p11, z1); add(p11, z1, p01, z1); add(p01, z1, p00, z1);
                    add(p00, z0, p00, z1); add(p10, z0, p10, z1); add(p11, z0, p11, z1); add(p01, z0, p01, z1);
                };

                add_cuboid_wire(0.f, 0.f, w, d, top_t, leg_h + 0.5f * top_t);
                add_cuboid_wire( leg_dx,  leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                add_cuboid_wire( leg_dx, -leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                add_cuboid_wire(-leg_dx,  leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                add_cuboid_wire(-leg_dx, -leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                return;
            }

            if (ql.contains("silla") || ql.contains("chair"))
            {
                const float z_seat = 0.50f * h;
                const float z_back = 0.98f * h;
                const float t = 0.18f;

                const Eigen::Vector2f s0 = c0 + t * (c1 - c0) + t * (c3 - c0);
                const Eigen::Vector2f s1 = c1 + t * (c0 - c1) + t * (c2 - c1);
                const Eigen::Vector2f s2 = c2 + t * (c1 - c2) + t * (c3 - c2);
                const Eigen::Vector2f s3 = c3 + t * (c2 - c3) + t * (c0 - c3);

                add(s0, z_seat, s1, z_seat);
                add(s1, z_seat, s2, z_seat);
                add(s2, z_seat, s3, z_seat);
                add(s3, z_seat, s0, z_seat);
                add(s0, 0.f, s0, z_seat);
                add(s1, 0.f, s1, z_seat);
                add(s2, 0.f, s2, z_seat);
                add(s3, 0.f, s3, z_seat);
                const Eigen::Vector2f b0 = 0.5f * (s2 + c2);
                const Eigen::Vector2f b1 = 0.5f * (s3 + c3);
                add(b0, z_seat, b0, z_back);
                add(b1, z_seat, b1, z_back);
                add(b0, z_back, b1, z_back);
                add(0.5f * (b0 + b1), z_seat, 0.5f * (b0 + b1), z_back);
                return;
            }

            const float z_top = h;
            const float z_support = 0.92f * h;
            add(c0, z_top, c1, z_top);
            add(c1, z_top, c2, z_top);
            add(c2, z_top, c3, z_top);
            add(c3, z_top, c0, z_top);
            add(c0, 0.f, c0, z_support);
            add(c1, 0.f, c1, z_support);
            add(c2, 0.f, c2, z_support);
            add(c3, 0.f, c3, z_support);
            add(front_mid, 0.70f * h, back_mid, 0.70f * h);
            add(left_mid, 0.70f * h, right_mid, 0.70f * h);
            add(center, 0.68f * h, center, z_top);
        };

        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> init_segments;
        std::vector<QColor> init_segment_colors;
        std::vector<Eigen::Vector3f> init_ann_points;
        std::vector<QString> init_ann_texts;
        std::vector<QColor> init_ann_colors;
        for (int k = 0; k < K; ++k)
        {
            const auto& c = classes[static_cast<std::size_t>(k)];
            Eigen::Vector2f center = Eigen::Vector2f::Zero();
            for (const auto& v : c.vertices) center += v;
            center /= static_cast<float>(std::max<std::size_t>(1, c.vertices.size()));
            init_ann_points.emplace_back(to_camera(world_to_robot * center, c.height + 0.05f));
            const QString prev_txt = c.prev_sdf.has_value()
                ? QString::number(c.prev_sdf.value(), 'f', 4)
                : QString("--");
            init_ann_texts.emplace_back(QString::fromStdString(c.label) + QString(" | prev=%1 cur=--").arg(prev_txt));
            init_ann_colors.emplace_back(QColor(255, 255, 255, 255));
            append_mesh_like_wireframe(c.vertices, c.label, c.height, init_segments, init_segment_colors, em_wireframe_color);
        }
        const QString start_banner = (prev_sdf_count > 0)
            ? QString("EM start | prev=%1 cur=--").arg(prev_mean_sdf, 0, 'f', 4)
            : QString("EM start | prev=-- cur=--");
        ctx_.camera_viewer->set_wireframe_segments_camera_colored(init_segments, init_segment_colors, start_banner);
        ctx_.camera_viewer->set_wireframe_annotations_camera(init_ann_points, init_ann_texts, init_ann_colors);
        QCoreApplication::processEvents(QEventLoop::AllEvents, 15);
    }

    if (!enough_points || N <= 0)
    {
        if (ctx_.status_label)
            ctx_.status_label->setText("Status: visible models shown (insufficient 3D points for EM)");
        keep_overlay_lock = true;
        overlay_persistent_ = true;
        return;
    }

    // --- E-M iterations ---
    std::vector<float> q(static_cast<std::size_t>(N * (K + 1)), 1.f / static_cast<float>(K + 1));
    std::vector<int> point_classes(static_cast<std::size_t>(N), K);

    constexpr int max_iters = 8;
    constexpr float sigma_xy = 0.25f;
    constexpr float sigma_z = 0.40f;

    auto to_camera = [&](const Eigen::Vector2f& p_robot, float z_world) -> Eigen::Vector3f
    {
        return Eigen::Vector3f(p_robot.x() - ctx_.camera_tx,
                               p_robot.y() - ctx_.camera_ty,
                               z_world - ctx_.camera_tz);
    };

    struct PixelDepth
    {
        int u = -1;
        int v = -1;
        float depth = 0.f;
    };
    std::vector<PixelDepth> point_pixels(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i)
    {
        const auto& pc = points_cam[static_cast<std::size_t>(i)];
        const float d = pc.y();
        if (d <= near_depth)
            continue;
        const float u = cx + fx * (pc.x() / d);
        const float v = cy - fy * (pc.z() / d);
        if (std::isfinite(u) && std::isfinite(v) && u >= 0.f && u < static_cast<float>(img_w) && v >= 0.f && v < static_cast<float>(img_h))
        {
            point_pixels[static_cast<std::size_t>(i)] = PixelDepth{static_cast<int>(u), static_cast<int>(v), d};
        }
    }

    const int rw = std::max(1, img_w / 2);
    const int rh = std::max(1, img_h / 2);
    constexpr float inf_depth = std::numeric_limits<float>::infinity();

    std::vector<Eigen::Vector2f> room_robot;
    room_robot.reserve(room_polygon.size());
    for (const auto& p : room_polygon)
        room_robot.emplace_back(world_to_robot * p);

    auto point_in_polygon = [](const Eigen::Vector2f& p, const std::vector<Eigen::Vector2f>& poly) -> bool
    {
        if (poly.size() < 3)
            return false;
        bool inside = false;
        for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        {
            const auto& pi = poly[i];
            const auto& pj = poly[j];
            const bool intersect = ((pi.y() > p.y()) != (pj.y() > p.y())) &&
                                   (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) /
                                                (pj.y() - pi.y()) + pi.x());
            if (intersect)
                inside = !inside;
        }
        return inside;
    };

    float last_mean_sdf = 0.f;
    for (int iter = 0; iter < max_iters; ++iter)
    {
        // --- depth rasterization per model ---
        std::vector<std::vector<float>> class_depth_raster(static_cast<std::size_t>(K), std::vector<float>(static_cast<std::size_t>(rw * rh), inf_depth));

        auto project_uvd = [&](const Eigen::Vector3f& p_cam, float& u, float& v, float& d) -> bool
        {
            d = p_cam.y();
            if (d <= near_depth)
                return false;
            u = cx + fx * (p_cam.x() / d);
            v = cy - fy * (p_cam.z() / d);
            return std::isfinite(u) && std::isfinite(v);
        };

        auto raster_write = [&](std::vector<float>& ras, int u, int v, float d)
        {
            if (u < 0 || v < 0 || u >= rw || v >= rh || d <= near_depth)
                return;
            const std::size_t idx = static_cast<std::size_t>(v) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(u);
            ras[idx] = std::min(ras[idx], d);
        };

        auto raster_segment = [&](std::vector<float>& ras, const Eigen::Vector3f& a_cam, const Eigen::Vector3f& b_cam)
        {
            float ua = 0.f, va = 0.f, da = 0.f;
            float ub = 0.f, vb = 0.f, db = 0.f;
            if (!project_uvd(a_cam, ua, va, da) || !project_uvd(b_cam, ub, vb, db))
                return;

            const float du = ub - ua;
            const float dv = vb - va;
            const int steps = std::max(1, static_cast<int>(std::ceil(std::max(std::abs(du), std::abs(dv)) / 2.f)));
            for (int s = 0; s <= steps; ++s)
            {
                const float t = static_cast<float>(s) / static_cast<float>(steps);
                const float u = ua + t * du;
                const float v = va + t * dv;
                const float d = da + t * (db - da);
                raster_write(ras,
                             static_cast<int>(u * static_cast<float>(rw) / static_cast<float>(img_w)),
                             static_cast<int>(v * static_cast<float>(rh) / static_cast<float>(img_h)),
                             d);
            }
        };

        for (int k = 0; k < K; ++k)
        {
            const auto& c = classes[static_cast<std::size_t>(k)];
            if (c.vertices.size() < 3)
                continue;
            auto& ras = class_depth_raster[static_cast<std::size_t>(k)];
            for (std::size_t e = 0; e < c.vertices.size(); ++e)
            {
                const auto& a = c.vertices[e];
                const auto& b = c.vertices[(e + 1) % c.vertices.size()];
                const Eigen::Vector2f ar = world_to_robot * a;
                const Eigen::Vector2f br = world_to_robot * b;
                raster_segment(ras, to_camera(ar, 0.f), to_camera(br, 0.f));
                raster_segment(ras, to_camera(ar, c.height), to_camera(br, c.height));
                raster_segment(ras, to_camera(ar, 0.f), to_camera(ar, c.height));
            }
        }

        // --- infrastructure (room) depth raster ---
        std::vector<float> infra_depth_raster(static_cast<std::size_t>(rw * rh), inf_depth);
        if (room_robot.size() >= 3)
        {
            const Eigen::Vector3f cam_o(ctx_.camera_tx, ctx_.camera_ty, ctx_.camera_tz);
            constexpr float wall_h = 2.5f;

            auto raster_write_depth = [&](int u, int v, float d)
            {
                if (u < 0 || v < 0 || u >= rw || v >= rh || d <= near_depth)
                    return;
                const std::size_t idx = static_cast<std::size_t>(v) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(u);
                infra_depth_raster[idx] = std::min(infra_depth_raster[idx], d);
            };

            for (int vr = 0; vr < rh; ++vr)
            {
                for (int ur = 0; ur < rw; ++ur)
                {
                    const float u_full = (static_cast<float>(ur) + 0.5f) * static_cast<float>(img_w) / static_cast<float>(rw);
                    const float v_full = (static_cast<float>(vr) + 0.5f) * static_cast<float>(img_h) / static_cast<float>(rh);
                    const Eigen::Vector3f d_cam((u_full - cx) / std::max(1e-5f, fx),
                                                1.f,
                                                (cy - v_full) / std::max(1e-5f, fy));

                    float best_t = inf_depth;

                    if (std::abs(d_cam.z()) > 1e-6f)
                    {
                        const float t_floor = (0.f - cam_o.z()) / d_cam.z();
                        if (t_floor > near_depth)
                        {
                            const Eigen::Vector2f pxy = cam_o.head<2>() + t_floor * d_cam.head<2>();
                            if (point_in_polygon(pxy, room_robot))
                                best_t = std::min(best_t, t_floor);
                        }

                        const float t_ceil = (wall_h - cam_o.z()) / d_cam.z();
                        if (t_ceil > near_depth)
                        {
                            const Eigen::Vector2f pxy = cam_o.head<2>() + t_ceil * d_cam.head<2>();
                            if (point_in_polygon(pxy, room_robot))
                                best_t = std::min(best_t, t_ceil);
                        }
                    }

                    const Eigen::Vector2f o_xy = cam_o.head<2>();
                    const Eigen::Vector2f r_xy = d_cam.head<2>();
                    const auto cross2 = [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) -> float
                    {
                        return a.x() * b.y() - a.y() * b.x();
                    };
                    for (std::size_t e = 0; e < room_robot.size(); ++e)
                    {
                        const Eigen::Vector2f a = room_robot[e];
                        const Eigen::Vector2f b = room_robot[(e + 1) % room_robot.size()];
                        const Eigen::Vector2f s = b - a;
                        const float den = cross2(r_xy, s);
                        if (std::abs(den) < 1e-8f)
                            continue;

                        const Eigen::Vector2f qmp = a - o_xy;
                        const float t = cross2(qmp, s) / den;
                        const float seg_u = cross2(qmp, r_xy) / den;
                        if (t <= near_depth || seg_u < 0.f || seg_u > 1.f)
                            continue;

                        const float z_hit = cam_o.z() + t * d_cam.z();
                        if (z_hit >= 0.f && z_hit <= wall_h)
                            best_t = std::min(best_t, t);
                    }

                    if (std::isfinite(best_t))
                        raster_write_depth(ur, vr, best_t);
                }
            }
        }

        // --- active point filtering ---
        std::vector<unsigned char> active_points(static_cast<std::size_t>(N), 0);
        int active_count = 0;
        for (int i = 0; i < N; ++i)
        {
            const auto pix = point_pixels[static_cast<std::size_t>(i)];
            if (pix.u < 0 || pix.v < 0)
                continue;

            const int ur = pix.u * rw / std::max(1, img_w);
            const int vr = pix.v * rh / std::max(1, img_h);
            float best_obj_pred = inf_depth;
            for (int k = 0; k < K; ++k)
            {
                const auto& ras = class_depth_raster[static_cast<std::size_t>(k)];
                for (int dv = -1; dv <= 1; ++dv)
                {
                    for (int du = -1; du <= 1; ++du)
                    {
                        const int uu = ur + du;
                        const int vv = vr + dv;
                        if (uu < 0 || vv < 0 || uu >= rw || vv >= rh)
                            continue;
                        const float d_pred = ras[static_cast<std::size_t>(vv) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(uu)];
                        best_obj_pred = std::min(best_obj_pred, d_pred);
                    }
                }
            }

            float best_infra_pred = inf_depth;
            for (int dv = -1; dv <= 1; ++dv)
            {
                for (int du = -1; du <= 1; ++du)
                {
                    const int uu = ur + du;
                    const int vv = vr + dv;
                    if (uu < 0 || vv < 0 || uu >= rw || vv >= rh)
                        continue;
                    const float d_inf = infra_depth_raster[static_cast<std::size_t>(vv) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(uu)];
                    best_infra_pred = std::min(best_infra_pred, d_inf);
                }
            }

            const bool obj_supported = std::isfinite(best_obj_pred)
                && (std::abs(pix.depth - best_obj_pred) < 0.60f || pix.depth + 0.08f < best_obj_pred);
            const bool infra_close = std::isfinite(best_infra_pred)
                && (pix.depth >= best_infra_pred - 0.05f);
            const bool keep = obj_supported || !infra_close;

            if (keep)
            {
                active_points[static_cast<std::size_t>(i)] = 1;
                ++active_count;
            }
        }

        // --- E-step ---
        float objective = 0.f;
        for (int i = 0; i < N; ++i)
        {
            if (active_points[static_cast<std::size_t>(i)] == 0)
            {
                for (int k = 0; k < K; ++k)
                    q[static_cast<std::size_t>(i * (K + 1) + k)] = 0.f;
                q[static_cast<std::size_t>(i * (K + 1) + K)] = 1.f;
                point_classes[static_cast<std::size_t>(i)] = K;
                continue;
            }

            const auto& pw = points_world[static_cast<std::size_t>(i)];
            const Eigen::Vector2f pxy = pw.head<2>();

            std::vector<float> logp(static_cast<std::size_t>(K + 1), 0.f);
            float max_logp = -std::numeric_limits<float>::max();

            for (int k = 0; k < K; ++k)
            {
                const auto& c = classes[static_cast<std::size_t>(k)];
                const float dxy = point_polygon_distance(pxy, c.vertices);
                const float dz = std::abs(pw.z() - 0.5f * c.height);
                float depth_pen = 0.f;
                const auto pix = point_pixels[static_cast<std::size_t>(i)];
                if (pix.u >= 0 && pix.v >= 0)
                {
                    const int ur = pix.u * rw / std::max(1, img_w);
                    const int vr = pix.v * rh / std::max(1, img_h);
                    const auto& ras = class_depth_raster[static_cast<std::size_t>(k)];
                    float best_pred = inf_depth;
                    for (int dv = -1; dv <= 1; ++dv)
                    {
                        for (int du = -1; du <= 1; ++du)
                        {
                            const int uu = ur + du;
                            const int vv = vr + dv;
                            if (uu < 0 || vv < 0 || uu >= rw || vv >= rh)
                                continue;
                            const float d_pred = ras[static_cast<std::size_t>(vv) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(uu)];
                            best_pred = std::min(best_pred, d_pred);
                        }
                    }
                    if (std::isfinite(best_pred))
                    {
                        const float dr = std::abs(pix.depth - best_pred);
                        depth_pen = std::min(dr, 2.0f);
                        if (pix.depth + 0.10f < best_pred)
                            depth_pen += 0.60f;
                    }
                }
                const float e = (dxy * dxy) / (2.f * sigma_xy * sigma_xy)
                              + 0.35f * (dz * dz) / (2.f * sigma_z * sigma_z);
                const float l = -(e + 0.45f * depth_pen);
                logp[static_cast<std::size_t>(k)] = l;
                max_logp = std::max(max_logp, l);
            }
            const float l_bg = -0.7f * std::abs(pw.z());
            logp[static_cast<std::size_t>(K)] = l_bg;
            max_logp = std::max(max_logp, l_bg);

            float sumw = 0.f;
            for (int k = 0; k < K + 1; ++k)
            {
                const float w = std::exp(logp[static_cast<std::size_t>(k)] - max_logp);
                q[static_cast<std::size_t>(i * (K + 1) + k)] = w;
                sumw += w;
            }
            sumw = std::max(sumw, 1e-8f);

            int best_k = K;
            float best_q = -1.f;
            for (int k = 0; k < K + 1; ++k)
            {
                const float qq = q[static_cast<std::size_t>(i * (K + 1) + k)] / sumw;
                q[static_cast<std::size_t>(i * (K + 1) + k)] = qq;
                if (qq > best_q)
                {
                    best_q = qq;
                    best_k = k;
                }
            }
            point_classes[static_cast<std::size_t>(i)] = best_k;
            objective += -std::log(std::max(best_q, 1e-8f));
        }
        objective /= static_cast<float>(std::max(1, active_count));

        std::vector<float> class_likelihood(static_cast<std::size_t>(K), 0.f);
        for (int k = 0; k < K; ++k)
        {
            float sum_q = 0.f;
            for (int i = 0; i < N; ++i)
                if (active_points[static_cast<std::size_t>(i)] != 0)
                    sum_q += q[static_cast<std::size_t>(i * (K + 1) + k)];
            class_likelihood[static_cast<std::size_t>(k)] = sum_q / static_cast<float>(std::max(1, active_count));
        }

        // --- per-iteration visualization ---
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> segments;
        std::vector<QColor> segment_colors;
        std::vector<Eigen::Vector3f> annotation_points;
        std::vector<QString> annotation_texts;
        std::vector<QColor> annotation_colors;

        auto oriented_box_corners_iter = [](const std::vector<Eigen::Vector2f>& poly) -> std::array<Eigen::Vector2f, 4>
        {
            std::array<Eigen::Vector2f, 4> out{Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()};
            if (poly.empty())
                return out;

            if (poly.size() == 4)
            {
                for (int i = 0; i < 4; ++i) out[i] = poly[i];
                return out;
            }

            Eigen::Vector2f c = Eigen::Vector2f::Zero();
            for (const auto& p : poly) c += p;
            c /= static_cast<float>(poly.size());

            Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
            for (const auto& p : poly)
            {
                const Eigen::Vector2f d = p - c;
                cov += d * d.transpose();
            }
            cov /= static_cast<float>(std::max<std::size_t>(1, poly.size()));

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
            Eigen::Vector2f u(1.f, 0.f);
            if (eig.info() == Eigen::Success)
                u = eig.eigenvectors().col(1).normalized();
            Eigen::Vector2f v(-u.y(), u.x());

            float umin = std::numeric_limits<float>::max();
            float umax = -std::numeric_limits<float>::max();
            float vmin = std::numeric_limits<float>::max();
            float vmax = -std::numeric_limits<float>::max();
            for (const auto& p : poly)
            {
                const Eigen::Vector2f d = p - c;
                const float pu = d.dot(u);
                const float pv = d.dot(v);
                umin = std::min(umin, pu);
                umax = std::max(umax, pu);
                vmin = std::min(vmin, pv);
                vmax = std::max(vmax, pv);
            }

            out[0] = c + u * umin + v * vmin;
            out[1] = c + u * umax + v * vmin;
            out[2] = c + u * umax + v * vmax;
            out[3] = c + u * umin + v * vmax;
            return out;
        };

        auto append_mesh_like_wireframe_iter = [&](const std::vector<Eigen::Vector2f>& poly,
                                                   const std::string& label,
                                                   float h,
                                                   std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& out_segments,
                                                   std::vector<QColor>& out_colors,
                                                   const QColor& color)
        {
            if (poly.size() < 3)
                return;

            const auto corners = oriented_box_corners_iter(poly);
            const auto& c0 = corners[0];
            const auto& c1 = corners[1];
            const auto& c2 = corners[2];
            const auto& c3 = corners[3];

            auto add = [&](const Eigen::Vector2f& a, float za, const Eigen::Vector2f& b, float zb)
            {
                const Eigen::Vector2f ar = world_to_robot * a;
                const Eigen::Vector2f br = world_to_robot * b;
                out_segments.emplace_back(to_camera(ar, za), to_camera(br, zb));
                out_colors.emplace_back(color);
            };

            const Eigen::Vector2f center = 0.25f * (c0 + c1 + c2 + c3);
            const Eigen::Vector2f front_mid = 0.5f * (c0 + c1);
            const Eigen::Vector2f back_mid  = 0.5f * (c2 + c3);
            const Eigen::Vector2f left_mid  = 0.5f * (c0 + c3);
            const Eigen::Vector2f right_mid = 0.5f * (c1 + c2);
            const QString ql = QString::fromStdString(label).toLower();

            if (ql.contains("mesa") || ql.contains("table"))
            {
                const Eigen::Vector2f center = 0.25f * (c0 + c1 + c2 + c3);
                Eigen::Vector2f ux = c1 - c0;
                Eigen::Vector2f uy = c3 - c0;
                const float w = std::max(0.08f, ux.norm());
                const float d = std::max(0.08f, uy.norm());
                if (ux.norm() < 1e-6f) ux = Eigen::Vector2f(1.f, 0.f); else ux.normalize();
                if (uy.norm() < 1e-6f) uy = Eigen::Vector2f(-ux.y(), ux.x()); else uy.normalize();

                const float h_table = std::max(0.20f, h);
                const float top_t = std::max(0.03f, 0.08f * h_table);
                const float leg_t = std::max(0.03f, 0.12f * std::min(w, d));
                const float leg_h = std::max(0.05f, h_table - top_t);
                const float leg_dx = std::max(0.f, 0.5f * w - 0.5f * leg_t);
                const float leg_dy = std::max(0.f, 0.5f * d - 0.5f * leg_t);

                auto local_to_world = [&](float lx, float ly) -> Eigen::Vector2f
                {
                    return center + ux * lx + uy * ly;
                };

                auto add_cuboid_wire = [&](float lx, float ly, float sx, float sy, float sz, float cz)
                {
                    const float hx = 0.5f * sx;
                    const float hy = 0.5f * sy;
                    const float hz = 0.5f * sz;
                    const float z0 = cz - hz;
                    const float z1 = cz + hz;

                    const Eigen::Vector2f p00 = local_to_world(lx - hx, ly - hy);
                    const Eigen::Vector2f p10 = local_to_world(lx + hx, ly - hy);
                    const Eigen::Vector2f p11 = local_to_world(lx + hx, ly + hy);
                    const Eigen::Vector2f p01 = local_to_world(lx - hx, ly + hy);

                    add(p00, z0, p10, z0); add(p10, z0, p11, z0); add(p11, z0, p01, z0); add(p01, z0, p00, z0);
                    add(p00, z1, p10, z1); add(p10, z1, p11, z1); add(p11, z1, p01, z1); add(p01, z1, p00, z1);
                    add(p00, z0, p00, z1); add(p10, z0, p10, z1); add(p11, z0, p11, z1); add(p01, z0, p01, z1);
                };

                add_cuboid_wire(0.f, 0.f, w, d, top_t, leg_h + 0.5f * top_t);
                add_cuboid_wire( leg_dx,  leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                add_cuboid_wire( leg_dx, -leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                add_cuboid_wire(-leg_dx,  leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                add_cuboid_wire(-leg_dx, -leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                return;
            }

            if (ql.contains("silla") || ql.contains("chair"))
            {
                const float z_seat = 0.50f * h;
                const float z_back = 0.98f * h;
                const float t = 0.18f;
                const Eigen::Vector2f s0 = c0 + t * (c1 - c0) + t * (c3 - c0);
                const Eigen::Vector2f s1 = c1 + t * (c0 - c1) + t * (c2 - c1);
                const Eigen::Vector2f s2 = c2 + t * (c1 - c2) + t * (c3 - c2);
                const Eigen::Vector2f s3 = c3 + t * (c2 - c3) + t * (c0 - c3);
                add(s0, z_seat, s1, z_seat);
                add(s1, z_seat, s2, z_seat);
                add(s2, z_seat, s3, z_seat);
                add(s3, z_seat, s0, z_seat);
                add(s0, 0.f, s0, z_seat);
                add(s1, 0.f, s1, z_seat);
                add(s2, 0.f, s2, z_seat);
                add(s3, 0.f, s3, z_seat);
                const Eigen::Vector2f b0 = 0.5f * (s2 + c2);
                const Eigen::Vector2f b1 = 0.5f * (s3 + c3);
                add(b0, z_seat, b0, z_back);
                add(b1, z_seat, b1, z_back);
                add(b0, z_back, b1, z_back);
                add(0.5f * (b0 + b1), z_seat, 0.5f * (b0 + b1), z_back);
                return;
            }

            const float z_top = h;
            const float z_support = 0.92f * h;
            add(c0, z_top, c1, z_top);
            add(c1, z_top, c2, z_top);
            add(c2, z_top, c3, z_top);
            add(c3, z_top, c0, z_top);
            add(c0, 0.f, c0, z_support);
            add(c1, 0.f, c1, z_support);
            add(c2, 0.f, c2, z_support);
            add(c3, 0.f, c3, z_support);
            add(front_mid, 0.70f * h, back_mid, 0.70f * h);
            add(left_mid, 0.70f * h, right_mid, 0.70f * h);
            add(center, 0.68f * h, center, z_top);
        };

        for (const auto& c : classes)
        {
            if (c.vertices.size() < 3)
                continue;
            const int class_idx = static_cast<int>(&c - classes.data());

            Eigen::Vector2f center = Eigen::Vector2f::Zero();
            for (const auto& v : c.vertices) center += v;
            center /= static_cast<float>(std::max<std::size_t>(1, c.vertices.size()));
            const Eigen::Vector2f center_robot = world_to_robot * center;
            annotation_points.emplace_back(to_camera(center_robot, c.height + 0.05f));
            const float lk = (class_idx >= 0 && class_idx < static_cast<int>(class_likelihood.size()))
                ? class_likelihood[static_cast<std::size_t>(class_idx)] : 0.f;
            const QString prev_txt = c.prev_sdf.has_value()
                ? QString::number(c.prev_sdf.value(), 'f', 4)
                : QString("--");
            const QString cur_txt = (std::isfinite(c.last_sdf) && c.last_sdf > 0.f)
                ? QString::number(c.last_sdf, 'f', 4)
                : QString("--");
            annotation_texts.emplace_back(QString::fromStdString(c.label)
                                          + QString(" | prev=%1 cur=%2 L=%3")
                                                .arg(prev_txt)
                                                .arg(cur_txt)
                                                .arg(lk, 0, 'f', 2));
            annotation_colors.emplace_back(QColor(255, 255, 255, 255));
            append_mesh_like_wireframe_iter(c.vertices, c.label, c.height, segments, segment_colors, em_wireframe_color);
        }
        const QString em_banner = (prev_sdf_count > 0)
            ? QString("EM iter %1/%2 | prev=%3 cur=%4")
                .arg(iter + 1)
                .arg(max_iters)
                .arg(prev_mean_sdf, 0, 'f', 4)
                .arg(last_mean_sdf, 0, 'f', 4)
            : QString("EM iter %1/%2 | prev=-- cur=%3")
                .arg(iter + 1)
                .arg(max_iters)
                .arg(last_mean_sdf, 0, 'f', 4);
        ctx_.camera_viewer->set_wireframe_segments_camera_colored(segments, segment_colors, em_banner);
        ctx_.camera_viewer->set_wireframe_annotations_camera(annotation_points, annotation_texts, annotation_colors);

        const QString title = em_banner;
        ctx_.camera_viewer->set_em_points_overlay(points_cam, point_classes, class_colors, class_labels, title);

        if (ctx_.status_label)
            ctx_.status_label->setText(QString("Status: EM iter %1/%2").arg(iter + 1).arg(max_iters));
        if (ctx_.sdf_label)
            ctx_.sdf_label->setText(QString("SDF fit: mean=%1").arg(last_mean_sdf, 0, 'f', 4));

        QCoreApplication::processEvents(QEventLoop::AllEvents, 15);

        // --- M-step ---
        float sdf_acc = 0.f;
        int sdf_count = 0;
        for (int k = 0; k < K; ++k)
        {
            std::vector<Eigen::Vector3f> class_points;
            class_points.reserve(static_cast<std::size_t>(N / std::max(1, K)) + 1);
            for (int i = 0; i < N; ++i)
            {
                if (active_points[static_cast<std::size_t>(i)] == 0)
                    continue;
                const float qik = q[static_cast<std::size_t>(i * (K + 1) + k)];
                if (qik > 0.45f || point_classes[static_cast<std::size_t>(i)] == k)
                    class_points.emplace_back(points_world[static_cast<std::size_t>(i)]);
            }

            if (class_points.size() < 15)
                continue;

            const QString class_label_lc = QString::fromStdString(classes[static_cast<std::size_t>(k)].label).toLower();
            if (class_label_lc.contains("mesa") || class_label_lc.contains("table"))
            {
                auto estimate_obb = [](const std::vector<Eigen::Vector2f>& poly,
                                       Eigen::Vector2f& center,
                                       float& yaw,
                                       float& width,
                                       float& depth) -> bool
                {
                    if (poly.size() < 3)
                        return false;

                    center = Eigen::Vector2f::Zero();
                    for (const auto& p : poly) center += p;
                    center /= static_cast<float>(poly.size());

                    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
                    for (const auto& p : poly)
                    {
                        const Eigen::Vector2f d = p - center;
                        cov += d * d.transpose();
                    }
                    cov /= static_cast<float>(std::max<std::size_t>(1, poly.size()));

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
                    Eigen::Vector2f ux(1.f, 0.f);
                    if (eig.info() == Eigen::Success)
                        ux = eig.eigenvectors().col(1).normalized();
                    Eigen::Vector2f uy(-ux.y(), ux.x());

                    float min_u = std::numeric_limits<float>::max();
                    float max_u = -std::numeric_limits<float>::max();
                    float min_v = std::numeric_limits<float>::max();
                    float max_v = -std::numeric_limits<float>::max();
                    for (const auto& p : poly)
                    {
                        const Eigen::Vector2f d = p - center;
                        const float u = d.dot(ux);
                        const float v = d.dot(uy);
                        min_u = std::min(min_u, u);
                        max_u = std::max(max_u, u);
                        min_v = std::min(min_v, v);
                        max_v = std::max(max_v, v);
                    }

                    yaw = std::atan2(ux.y(), ux.x());
                    width = std::max(0.08f, max_u - min_u);
                    depth = std::max(0.08f, max_v - min_v);
                    return true;
                };

                auto make_rect = [](float tx, float ty, float yaw, float w, float d) -> std::vector<Eigen::Vector2f>
                {
                    const float hw = 0.5f * std::max(0.08f, w);
                    const float hd = 0.5f * std::max(0.08f, d);
                    const Eigen::Vector2f cx(tx, ty);
                    const Eigen::Vector2f ux(std::cos(yaw), std::sin(yaw));
                    const Eigen::Vector2f uy(-ux.y(), ux.x());

                    std::vector<Eigen::Vector2f> out;
                    out.reserve(4);
                    out.emplace_back(cx + ux * (-hw) + uy * (-hd));
                    out.emplace_back(cx + ux * ( hw) + uy * (-hd));
                    out.emplace_back(cx + ux * ( hw) + uy * ( hd));
                    out.emplace_back(cx + ux * (-hw) + uy * ( hd));
                    return out;
                };

                try
                {
                    Eigen::Vector2f cxy = Eigen::Vector2f::Zero();
                    float yaw0 = 0.f;
                    float w0 = 1.f;
                    float d0 = 0.8f;
                    const bool obb_ok = estimate_obb(classes[static_cast<std::size_t>(k)].vertices, cxy, yaw0, w0, d0);
                    if (!obb_ok)
                        throw std::runtime_error("Cannot estimate initial OBB for table class");

                    std::vector<float> xyz;
                    xyz.reserve(class_points.size() * 3);
                    for (const auto& p : class_points)
                    {
                        xyz.push_back(p.x());
                        xyz.push_back(p.y());
                        xyz.push_back(p.z());
                    }

                    auto pts = torch::from_blob(xyz.data(),
                                                {static_cast<long>(class_points.size()), 3},
                                                torch::TensorOptions().dtype(torch::kFloat32)).clone();

                    const float h0 = std::max(0.20f, classes[static_cast<std::size_t>(k)].height);
                    auto init_params = torch::tensor({cxy.x(), cxy.y(), yaw0, w0, d0, h0},
                                                     torch::TensorOptions().dtype(torch::kFloat32));

                    rc::object_models::TableAnalyticModel table_model(init_params);
                    auto fit = table_model.fit_autograd(pts, 70, 0.025f);
                    if (fit.ok)
                    {
                        const auto p = fit.params.to(torch::kCPU);
                        const float tx = p[0].item<float>();
                        const float ty = p[1].item<float>();
                        const float yaw = p[2].item<float>();
                        const float fw = std::max(0.08f, p[3].item<float>());
                        const float fd = std::max(0.08f, p[4].item<float>());
                        const float fh = std::max(0.20f, p[5].item<float>());

                        classes[static_cast<std::size_t>(k)].vertices = make_rect(tx, ty, yaw, fw, fd);
                        classes[static_cast<std::size_t>(k)].height = fh;
                        classes[static_cast<std::size_t>(k)].last_sdf = fit.final_loss;
                        sdf_acc += fit.final_loss;
                        ++sdf_count;
                        continue;
                    }
                }
                catch (const std::exception&)
                {
                }
            }

            auto result = mesh_sdf_optimizer_.optimize_mesh_with_pose(class_points,
                                                                       classes[static_cast<std::size_t>(k)].vertices,
                                                                       room_polygon);
            if (!result.ok)
                continue;

            classes[static_cast<std::size_t>(k)].vertices = result.vertices;
            classes[static_cast<std::size_t>(k)].last_sdf = result.final_data_loss;
            sdf_acc += result.final_data_loss;
            ++sdf_count;
        }

        last_mean_sdf = (sdf_count > 0) ? (sdf_acc / static_cast<float>(sdf_count)) : last_mean_sdf;
        QThread::msleep(70);
    }

    // --- post-EM: collect candidates ---
    if (ctx_.status_label)
        ctx_.status_label->setText("Status: EM validator finished");

    int valid_candidates = 0;
    int improved_candidates = 0;
    int non_improved = 0;
    pending_adjustments_.clear();
    for (const auto& c : classes)
    {
        if (c.furniture_index < 0 || c.furniture_index >= static_cast<int>(furniture_polygons.size()))
            continue;
        if (c.vertices.size() < 3)
            continue;
        if (!std::isfinite(c.last_sdf) || c.last_sdf <= 0.f)
            continue;

        const auto prev = c.prev_sdf;
        const bool improved = !prev.has_value() || (c.last_sdf + 1e-4f < prev.value());
        if (improved) ++improved_candidates;
        else ++non_improved;

        PendingAdjustment cand;
        cand.furniture_index = c.furniture_index;
        cand.vertices = c.vertices;
        cand.new_sdf = c.last_sdf;
        cand.prev_sdf = prev;
        cand.label = c.label;
        pending_adjustments_.push_back(std::move(cand));
        ++valid_candidates;
    }

    if (valid_candidates > 0)
    {
        decision_pending_ = true;
        ctx_.camera_viewer->set_em_decision_buttons_visible(true);
        if (ctx_.status_label)
            ctx_.status_label->setText(QString("Status: EM finished (%1 candidates, %2 improved). Accept/Reject pending")
                                             .arg(valid_candidates)
                                             .arg(improved_candidates));
    }
    else
    {
        decision_pending_ = false;
        ctx_.camera_viewer->set_em_decision_buttons_visible(false);
        if (ctx_.status_label)
            ctx_.status_label->setText("Status: EM validator finished (no valid fitted candidates)");
    }

    if (!points_cam.empty() && points_cam.size() == point_classes.size())
    {
        const QString final_title = (prev_sdf_count > 0)
            ? QString("EM done | prev=%1 cur=%2")
                  .arg(prev_mean_sdf, 0, 'f', 4)
                  .arg(last_mean_sdf, 0, 'f', 4)
            : QString("EM done | prev=-- cur=%1").arg(last_mean_sdf, 0, 'f', 4);
        ctx_.camera_viewer->set_em_points_overlay(points_cam, point_classes, class_colors, class_labels, final_title);
    }

    keep_overlay_lock = true;
    overlay_persistent_ = true;
}

// ---------------------------------------------------------------------------
// apply_pending_adjustments
// ---------------------------------------------------------------------------
void EMManager::apply_pending_adjustments(bool accept)
{
    if (!ctx_.camera_viewer)
        return;

    if (!decision_pending_)
    {
        ctx_.camera_viewer->set_em_decision_buttons_visible(false);
        return;
    }

    auto& furniture_polygons = *ctx_.furniture_polygons;
    int applied = 0;
    if (accept)
    {
        for (const auto& cand : pending_adjustments_)
        {
            if (cand.furniture_index < 0 || cand.furniture_index >= static_cast<int>(furniture_polygons.size()))
                continue;

            if (rc::SceneGraphAdapter::accept_fit_if_improved(*ctx_.scene_graph,
                                                               furniture_polygons[static_cast<std::size_t>(cand.furniture_index)],
                                                               cand.vertices,
                                                               cand.new_sdf))
            {
                ++applied;
            }
            else
            {
                auto& fp = furniture_polygons[static_cast<std::size_t>(cand.furniture_index)];
                fp.vertices = cand.vertices;
                fp.last_fit_sdf = cand.new_sdf;
                fp.height = std::max(0.2f, model_height_from_label(fp.label.empty() ? fp.id : fp.label));

                rc::SceneGraphObject obj;
                obj.id = fp.id;
                obj.label = fp.label;
                obj.vertices = fp.vertices;
                obj.last_fit_sdf = fp.last_fit_sdf;
                obj.frame_yaw_inward_rad = fp.frame_yaw_inward_rad;
                obj.height = fp.height;
                if (ctx_.scene_graph->upsert_object(obj))
                    ++applied;
            }
        }

        if (applied > 0)
        {
            if (ctx_.on_accepted)
                ctx_.on_accepted();
            if (ctx_.status_label)
                ctx_.status_label->setText(QString("Status: EM adjustments accepted (%1 applied)").arg(applied));
            if (ctx_.sdf_label)
                ctx_.sdf_label->setText(QString("SDF fit: accepted %1").arg(applied));
        }
        else
        {
            if (ctx_.status_label)
                ctx_.status_label->setText("Status: EM adjustments accepted (no applicable updates)");
        }
    }
    else
    {
        if (ctx_.status_label)
            ctx_.status_label->setText("Status: EM adjustments rejected");
    }

    decision_pending_ = false;
    pending_adjustments_.clear();
    ctx_.camera_viewer->set_em_decision_buttons_visible(false);
    overlay_persistent_ = false;
    overlay_lock_ = false;
    ctx_.camera_viewer->clear_em_points_overlay();
    ctx_.camera_viewer->clear_wireframe_overlay();
}
