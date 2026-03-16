#include "specificworker.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QRegularExpression>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>

int SpecificWorker::find_furniture_index_by_name(const QString& name) const
{
    auto normalize = [](QString s)
    {
        s = s.trimmed();
        const int idx = s.indexOf(" (");
        if (idx > 0)
            s = s.left(idx);
        return s.toLower();
    };

    const QString target = normalize(name);
    if (target.isEmpty())
        return -1;

    int contains_match = -1;
    for (std::size_t i = 0; i < layout_manager_.furniture().size(); ++i)
    {
        const auto& fp = layout_manager_.furniture()[i];
        const QString label = normalize(QString::fromStdString(fp.label));
        const QString id = normalize(QString::fromStdString(fp.id));
        if ((!label.isEmpty() && label == target) || (!id.isEmpty() && id == target))
            return static_cast<int>(i);

        if ((contains_match < 0) &&
            ((!label.isEmpty() && (label.contains(target) || target.contains(label))) ||
             (!id.isEmpty() && (id.contains(target) || target.contains(id)))))
            contains_match = static_cast<int>(i);
    }
    return contains_match;
}

void SpecificWorker::translate_furniture_by_name(const QString& name, float dx_room, float dy_room)
{
    const int idx = find_furniture_index_by_name(name);
    if (idx < 0 || (std::abs(dx_room) < 1e-6f && std::abs(dy_room) < 1e-6f))
        return;

    auto& fp = layout_manager_.furniture_mutable()[static_cast<std::size_t>(idx)];
    if (fp.vertices.empty())
        return;

    const Eigen::Vector2f d(dx_room, dy_room);
    for (auto& v : fp.vertices)
        v += d;

    update_furniture_draw_item(static_cast<std::size_t>(idx));
    if (viewer_3d_)
        viewer_3d_->translate_furniture_object(QString::fromStdString(fp.label), dx_room, dy_room);

    // Keep model in sync without triggering objectChanged signal (drag path).
    scene_graph_.sync_object_silent(fp.label, fp.vertices, fp.height, std::nullopt);
    if (scene_tree_)
        scene_tree_->update_object_display(QString::fromStdString(fp.label));
}

void SpecificWorker::rotate_furniture_by_name(const QString& name, float angle_rad, const QVector3D& axis)
{
    const int idx = find_furniture_index_by_name(name);
    if (idx < 0 || std::abs(angle_rad) < 1e-6f)
        return;

    auto& fp = layout_manager_.furniture_mutable()[static_cast<std::size_t>(idx)];

    // Only update the 2D polygon for vertical-axis (yaw) rotations.
    if (std::abs(axis.y()) > 0.5f && fp.vertices.size() >= 3)
    {
        Eigen::Vector2f cen = Eigen::Vector2f::Zero();
        for (const auto& v : fp.vertices)
            cen += v;
        cen /= static_cast<float>(fp.vertices.size());

        const float c = std::cos(angle_rad);
        const float s = std::sin(angle_rad);
        for (auto& v : fp.vertices)
        {
            const Eigen::Vector2f d = v - cen;
            v = cen + Eigen::Vector2f(c * d.x() - s * d.y(), s * d.x() + c * d.y());
        }
        update_furniture_draw_item(static_cast<std::size_t>(idx));
    }

    if (viewer_3d_)
        viewer_3d_->rotate_furniture_object(QString::fromStdString(fp.label), angle_rad, axis);

    // Keep model in sync and update yaw so the tree reflects the rotation.
    const float cur_yaw = scene_graph_.get_object_yaw(fp.label).value_or(fp.frame_yaw_inward_rad);
    const float new_yaw = cur_yaw + (std::abs(axis.y()) > 0.5f ? angle_rad : 0.f);
    fp.frame_yaw_inward_rad = new_yaw;
    scene_graph_.sync_object_silent(fp.label, fp.vertices, fp.height, new_yaw);
    if (scene_tree_)
        scene_tree_->update_object_display(QString::fromStdString(fp.label));
}

int SpecificWorker::wall_index_from_pick_name(const QString& name) const
{
    static const QRegularExpression wall_re(R"(^\s*Wall\s+(\d+)\s*$)",
                                            QRegularExpression::CaseInsensitiveOption);
    const auto m = wall_re.match(name);
    if (!m.hasMatch())
        return -1;
    bool ok = false;
    const int wall_num_1based = m.captured(1).toInt(&ok);
    if (!ok || wall_num_1based <= 0)
        return -1;

    const int idx = wall_num_1based - 1;
    const auto& poly = layout_manager_.room_polygon();
    if (idx < 0 || idx >= static_cast<int>(poly.size()))
        return -1;
    return idx;
}

void SpecificWorker::translate_wall_by_name(const QString& name, float dx_room, float dy_room)
{
    const int idx = wall_index_from_pick_name(name);
    if (idx < 0 || (std::abs(dx_room) < 1e-6f && std::abs(dy_room) < 1e-6f))
        return;

    auto poly = layout_manager_.room_polygon();
    if (poly.size() < 3)
        return;

    const int j = (idx + 1) % static_cast<int>(poly.size());
    const Eigen::Vector2f d(dx_room, dy_room);
    poly[static_cast<std::size_t>(idx)] += d;
    poly[static_cast<std::size_t>(j)] += d;

    layout_manager_.set_room_polygon(poly);
    rc::SceneGraphAdapter::rebuild_graph(
        scene_graph_, layout_manager_.room_polygon(), layout_manager_.furniture(),
        [](const std::string& l) { return EMManager::model_height_from_label(l); }, 2.6f);

    draw_room_polygon();
    draw_furniture();

    nav_manager_.path_planner().set_polygon(layout_manager_.room_polygon());
    nav_manager_.trajectory_controller().set_room_boundary(layout_manager_.room_polygon());
    const auto obs = layout_manager_.obstacle_polygons();
    nav_manager_.path_planner().set_obstacles(obs);
    nav_manager_.trajectory_controller().set_static_obstacles(obs);
}

void SpecificWorker::rotate_wall_by_name(const QString& name, float angle_rad, const QVector3D& axis)
{
    if (std::abs(axis.y()) <= 0.5f || std::abs(angle_rad) < 1e-6f)
        return;

    const int idx = wall_index_from_pick_name(name);
    if (idx < 0)
        return;

    auto poly = layout_manager_.room_polygon();
    if (poly.size() < 3)
        return;

    const int j = (idx + 1) % static_cast<int>(poly.size());
    const Eigen::Vector2f a = poly[static_cast<std::size_t>(idx)];
    const Eigen::Vector2f b = poly[static_cast<std::size_t>(j)];
    const Eigen::Vector2f c = 0.5f * (a + b);

    const float cs = std::cos(angle_rad);
    const float sn = std::sin(angle_rad);
    const Eigen::Vector2f da = a - c;
    const Eigen::Vector2f db = b - c;
    poly[static_cast<std::size_t>(idx)] = c + Eigen::Vector2f(cs * da.x() - sn * da.y(), sn * da.x() + cs * da.y());
    poly[static_cast<std::size_t>(j)] = c + Eigen::Vector2f(cs * db.x() - sn * db.y(), sn * db.x() + cs * db.y());

    layout_manager_.set_room_polygon(poly);
    rc::SceneGraphAdapter::rebuild_graph(
        scene_graph_, layout_manager_.room_polygon(), layout_manager_.furniture(),
        [](const std::string& l) { return EMManager::model_height_from_label(l); }, 2.6f);

    draw_room_polygon();
    draw_furniture();

    nav_manager_.path_planner().set_polygon(layout_manager_.room_polygon());
    nav_manager_.trajectory_controller().set_room_boundary(layout_manager_.room_polygon());
    const auto obs = layout_manager_.obstacle_polygons();
    nav_manager_.path_planner().set_obstacles(obs);
    nav_manager_.trajectory_controller().set_static_obstacles(obs);
}

void SpecificWorker::set_object_property(const QString& label, const QString& property, float value)
{
    const std::string key = label.toStdString();
    auto node_opt = scene_graph_.get_object_node(key);
    if (!node_opt) return;
    const auto& node = *node_opt;

    if (property == "tx" || property == "ty")
    {
        const float tx = (property == "tx") ? value : node.translation.x();
        const float ty = (property == "ty") ? value : node.translation.y();
        scene_graph_.set_object_pose(label.toStdString(), tx, ty, node.yaw_rad);
    }
    else if (property == "tz")
    {
        scene_graph_.set_object_tz(label.toStdString(), value);
    }
    else if (property == "yaw_deg")
    {
        const int idx = find_furniture_index_by_name(label);
        if (idx < 0) return;
        auto& fp = layout_manager_.furniture_mutable()[static_cast<std::size_t>(idx)];

        // Rotate footprint vertices around their centroid by the delta angle.
        // We use vertex rotation (same as mouse-drag path) to avoid the
        // set_object_pose extents issue: the model extents come from
        // rebuild() which decomposes the polygon along the "towards-room-center"
        // axis, inflating extents when the object is not aligned with that axis.
        const float new_yaw_rad = static_cast<float>(value * M_PI / 180.0);
        const float delta_rad   = new_yaw_rad - node.yaw_rad;
        if (std::abs(delta_rad) < 1e-7f) return;

        Eigen::Vector2f cen = Eigen::Vector2f::Zero();
        for (const auto& v : fp.vertices) cen += v;
        cen /= static_cast<float>(fp.vertices.size());

        const float cosA = std::cos(delta_rad), sinA = std::sin(delta_rad);
        for (auto& v : fp.vertices)
        {
            const Eigen::Vector2f d = v - cen;
            v = cen + Eigen::Vector2f(cosA * d.x() - sinA * d.y(),
                                       sinA * d.x() + cosA * d.y());
        }
        fp.frame_yaw_inward_rad = new_yaw_rad;

        update_furniture_draw_item(static_cast<std::size_t>(idx));
        if (viewer_3d_)
            viewer_3d_->rotate_furniture_object(label, delta_rad, QVector3D(0, 1, 0));

        scene_graph_.sync_object_silent(fp.label, fp.vertices, fp.height, new_yaw_rad);
        if (scene_tree_)
            scene_tree_->update_object_display(label);
        save_scene_graph_to_usd();
    }
    else if (property == "width" || property == "depth" || property == "height")
    {
        const float w = (property == "width")  ? value : node.extents.x();
        const float d = (property == "depth")  ? value : node.extents.y();
        const float h = (property == "height") ? value : node.extents.z();
        scene_graph_.set_object_extents(label.toStdString(), w, d, h);
    }
    // objectChanged signal from scene_graph_ will update layout_manager_ furniture,
    // viewer_3d_, and viewer2d_ via the connection added in specificworker.cpp.
}

void SpecificWorker::update_camera_wireframe_overlay(const Eigen::Affine2f &robot_pose)
{
    if (!camera_viewer_)
        return;

    if (layout_manager_.furniture().empty())
    {
        camera_viewer_->clear_wireframe_overlay();
        return;
    }

    const Eigen::Affine2f world_to_robot = robot_pose.inverse();

    auto to_camera = [this](const Eigen::Vector2f& p_robot, float z_world) -> Eigen::Vector3f
    {
        // Camera frame in this component: x=lateral, y=forward, z=up.
        return Eigen::Vector3f(p_robot.x() - params.CAMERA_TX,
                               p_robot.y() - params.CAMERA_TY,
                               z_world - params.CAMERA_TZ);
    };

    const float cx = 320.f;
    const float cy = 320.f;
    const float fx = 0.9f * 640.f;
    const float fy = 0.9f * 640.f;
    constexpr float near_depth = 0.05f;
    constexpr float max_select_bearing = 50.f * static_cast<float>(M_PI) / 180.f;
    constexpr float center_bearing_ref = 20.f * static_cast<float>(M_PI) / 180.f;

    auto projects_inside = [&](const Eigen::Vector3f& p_cam) -> bool
    {
        const float depth = p_cam.y();
        if (depth <= near_depth) return false;
        const float u = cx + fx * (p_cam.x() / depth);
        const float v = cy - fy * (p_cam.z() / depth);
        return std::isfinite(u) && std::isfinite(v) &&
               u >= 0.f && u < 640.f && v >= 0.f && v < 640.f;
    };

    auto polygon_area = [](const std::vector<Eigen::Vector2f>& poly) -> float
    {
        if (poly.size() < 3)
            return 0.f;
        double a = 0.0;
        for (std::size_t i = 0; i < poly.size(); ++i)
        {
            const auto& p = poly[i];
            const auto& q = poly[(i + 1) % poly.size()];
            a += static_cast<double>(p.x()) * static_cast<double>(q.y()) -
                 static_cast<double>(q.x()) * static_cast<double>(p.y());
        }
        return static_cast<float>(std::abs(a) * 0.5);
    };

    auto centroid_of = [](const std::vector<Eigen::Vector2f>& poly) -> Eigen::Vector2f
    {
        Eigen::Vector2f c = Eigen::Vector2f::Zero();
        for (const auto& v : poly) c += v;
        if (!poly.empty()) c /= static_cast<float>(poly.size());
        return c;
    };

    auto object_visibility_score = [&](int idx, float* out_abs_bearing = nullptr) -> float
    {
        if (idx < 0 || idx >= static_cast<int>(layout_manager_.furniture().size()))
            return -1.f;

        const auto& fp = layout_manager_.furniture()[idx];
        if (fp.vertices.size() < 3)
            return -1.f;

        const std::string base_name = fp.label.empty() ? fp.id : fp.label;
        const float h = EMManager::model_height_from_label(base_name);

        int projected_hits = 0;
        for (const auto& v_world : fp.vertices)
        {
            const Eigen::Vector2f v_robot = world_to_robot * v_world;
            if (projects_inside(to_camera(v_robot, 0.f))) ++projected_hits;
            if (projects_inside(to_camera(v_robot, h))) ++projected_hits;
        }

        const Eigen::Vector2f c_world = centroid_of(fp.vertices);
        const Eigen::Vector2f c_robot = world_to_robot * c_world;
        const Eigen::Vector3f c_cam = to_camera(c_robot, 0.5f * h);
        const float depth = c_cam.y();
        if (depth <= near_depth)
            return -1.f;

        const float abs_bearing = std::abs(std::atan2(c_cam.x(), c_cam.y()));
        if (out_abs_bearing) *out_abs_bearing = abs_bearing;
        if (abs_bearing > max_select_bearing)
            return -1.f;

        if (projected_hits == 0 && !projects_inside(c_cam))
            return -1.f;

        const float area_world = std::max(0.01f, polygon_area(fp.vertices));
        const float score_center = std::clamp(1.f - abs_bearing / center_bearing_ref, 0.f, 1.f);
        const float score_visible = std::clamp(static_cast<float>(projected_hits) / 6.f, 0.f, 1.f);
        const float score_size = std::clamp(std::log1p(area_world) / 2.0f, 0.f, 1.f);
        const float score_proximity = 1.f / (1.f + 0.35f * depth);

        return 0.58f * score_center +
               0.22f * score_visible +
               0.15f * score_size +
               0.05f * score_proximity;
    };

    int selected_index = focused_model_index_;
    float focused_bearing = std::numeric_limits<float>::max();
    const float focused_score = object_visibility_score(selected_index, &focused_bearing);

    int best_idx = -1;
    float best_score = -1.f;
    for (int i = 0; i < static_cast<int>(layout_manager_.furniture().size()); ++i)
    {
        const float s = object_visibility_score(i, nullptr);
        if (s > best_score)
        {
            best_score = s;
            best_idx = i;
        }
    }

    if (selected_index < 0 || selected_index >= static_cast<int>(layout_manager_.furniture().size()))
    {
        selected_index = best_idx;
    }
    else
    {
        const bool focused_not_visible = focused_score < 0.f;
        const bool focused_off_center = focused_bearing > (24.f * static_cast<float>(M_PI) / 180.f);
        const bool better_front_candidate = best_idx >= 0 && best_score > focused_score + 0.15f;
        if (focused_not_visible || (focused_off_center && better_front_candidate))
            selected_index = best_idx;
    }

    focused_model_index_ = selected_index;

    if (focused_model_index_ < 0 || focused_model_index_ >= static_cast<int>(layout_manager_.furniture().size()))
    {
        camera_viewer_->clear_wireframe_overlay();
        return;
    }

    const auto& fp = layout_manager_.furniture()[focused_model_index_];
    if (fp.vertices.size() < 3)
    {
        camera_viewer_->clear_wireframe_overlay();
        return;
    }

    Eigen::Vector2f cen = Eigen::Vector2f::Zero();
    for (const auto& v : fp.vertices) cen += v;
    cen /= static_cast<float>(fp.vertices.size());

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& v : fp.vertices)
    {
        const Eigen::Vector2f d = v - cen;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(std::max<std::size_t>(1, fp.vertices.size()));
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
    Eigen::Vector2f axis_x(1.f, 0.f);
    if (eig.info() == Eigen::Success)
        axis_x = eig.eigenvectors().col(1).normalized();
    const float yaw = std::atan2(axis_x.y(), axis_x.x());

    float min_u = std::numeric_limits<float>::max();
    float max_u = -std::numeric_limits<float>::max();
    float min_v = std::numeric_limits<float>::max();
    float max_v = -std::numeric_limits<float>::max();
    const Eigen::Vector2f axis_z(-axis_x.y(), axis_x.x());
    for (const auto& p : fp.vertices)
    {
        const Eigen::Vector2f d = p - cen;
        const float u = d.dot(axis_x);
        const float v = d.dot(axis_z);
        min_u = std::min(min_u, u); max_u = std::max(max_u, u);
        min_v = std::min(min_v, v); max_v = std::max(max_v, v);
    }
    const Eigen::Vector2f size(std::max(0.08f, max_u - min_u), std::max(0.08f, max_v - min_v));

    QString ql = QString::fromStdString(fp.label).toLower();
    QString mesh_rel;
    float sx = 1.f, sy = 1.f;
    float sz = 1.f;
    float z_offset_model = 0.f;
    if (ql.contains("mesa") || ql.contains("table"))
    {
        mesh_rel = "meshes/table.obj";
        sx = (size.x() > 0.1f) ? size.x() / 1.2f : 1.f;
        sy = (size.y() > 0.1f) ? size.y() / 0.8f : 1.f;
    }
    else if (ql.contains("banco") || ql.contains("bench"))
        mesh_rel = "meshes/bench.obj";
    else if (ql.contains("silla") || ql.contains("chair"))
    {
        mesh_rel = "meshes/chair.obj";
        sx = (size.x() > 0.1f) ? size.x() / 0.45f : 1.f;
        sy = (size.y() > 0.1f) ? size.y() / 0.45f : 1.f;
        sz = 1.20f;
    }
    else if (ql.contains("monitor") || ql.contains("pantalla") || ql.contains("screen") ||
             ql.contains("ordenador") || ql.contains("computer"))
    {
        mesh_rel = "meshes/monitor_screen.obj";
        z_offset_model = 0.45f;
    }
    else if (ql.contains("vitrina") || ql.contains("cabinet") || ql.contains("shelf"))
        mesh_rel = "meshes/bench.obj";
    else if (ql.contains("maceta") || ql.contains("plant") || ql.contains("pot"))
        mesh_rel = "meshes/pot_only.obj";

    if (mesh_rel.isEmpty())
    {
        camera_viewer_->clear_wireframe_overlay();
        return;
    }

    struct MeshWire
    {
        std::vector<Eigen::Vector3f> vertices;
        std::vector<std::pair<int,int>> edges;
    };
    static std::unordered_map<std::string, MeshWire> mesh_cache;

    const QString component_root = QDir(QCoreApplication::applicationDirPath() + "/..").absolutePath();
    const QString mesh_path_q = component_root + "/" + mesh_rel;
    const std::string mesh_path = mesh_path_q.toStdString();

    if (!mesh_cache.contains(mesh_path))
    {
        MeshWire mw;
        QFile f(mesh_path_q);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            camera_viewer_->clear_wireframe_overlay();
            return;
        }

        QTextStream in(&f);
        std::set<std::pair<int,int>> edge_set;
        while (!in.atEnd())
        {
            const QString line = in.readLine().trimmed();
            if (line.startsWith("v "))
            {
                const auto parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
                if (parts.size() >= 4)
                    mw.vertices.emplace_back(parts[1].toFloat(), parts[2].toFloat(), parts[3].toFloat());
            }
            else if (line.startsWith("f "))
            {
                const auto parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
                std::vector<int> idx;
                idx.reserve(parts.size() - 1);
                for (int i = 1; i < parts.size(); ++i)
                {
                    const QString tok = parts[i].split('/').front();
                    bool ok = false;
                    int vi = tok.toInt(&ok);
                    if (ok && vi > 0) idx.push_back(vi - 1);
                }
                for (std::size_t i = 0; i < idx.size(); ++i)
                {
                    int a = idx[i];
                    int b = idx[(i + 1) % idx.size()];
                    if (a == b) continue;
                    if (a > b) std::swap(a, b);
                    edge_set.emplace(a, b);
                }
            }
        }
        mw.edges.assign(edge_set.begin(), edge_set.end());
        mesh_cache.emplace(mesh_path, std::move(mw));
    }

    const auto& mw = mesh_cache.at(mesh_path);
    if (mw.vertices.empty() || mw.edges.empty())
    {
        camera_viewer_->clear_wireframe_overlay();
        return;
    }

    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    auto vertex_to_camera = [&](const Eigen::Vector3f& v) -> Eigen::Vector3f
    {
        const float lx = v.x() * sx;
        const float ly = v.y() * sy;
        const float lz = v.z() * sz + z_offset_model;

        // Match viewer mesh convention: base_rot = RotX(-90°), then yaw about vertical.
        // In room coordinates this is equivalent to rotating (x, -y) in the floor plane.
        const float x0 = lx;
        const float y0 = -ly;
        const Eigen::Vector2f p_world(cen.x() + c * x0 - s * y0,
                                      cen.y() + s * x0 + c * y0);
        const Eigen::Vector2f p_robot = world_to_robot * p_world;
        return to_camera(p_robot, lz);
    };

    std::vector<Eigen::Vector3f> verts_cam;
    verts_cam.reserve(mw.vertices.size());
    for (const auto& v : mw.vertices)
        verts_cam.emplace_back(vertex_to_camera(v));

    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> segments;
    segments.reserve(mw.edges.size());
    for (const auto& e : mw.edges)
    {
        if (e.first < 0 || e.second < 0 || e.first >= static_cast<int>(verts_cam.size()) || e.second >= static_cast<int>(verts_cam.size()))
            continue;
        segments.emplace_back(verts_cam[e.first], verts_cam[e.second]);
    }

    const QString wire_label = QString::fromStdString(fp.label.empty() ? fp.id : fp.label);
    std::vector<Eigen::Vector3f> ann_points;
    std::vector<QString> ann_texts;
    std::vector<QColor> ann_colors;
    ann_points.emplace_back(to_camera(world_to_robot * cen, EMManager::model_height_from_label(fp.label.empty() ? fp.id : fp.label) + 0.05f));
    ann_texts.emplace_back(wire_label);
    ann_colors.emplace_back(QColor(255, 255, 255, 255));

    camera_viewer_->set_wireframe_segments_camera(segments, "");
    camera_viewer_->set_wireframe_annotations_camera(ann_points, ann_texts, ann_colors);
}

void SpecificWorker::update_segmented_points_3d(const Eigen::Affine2f &robot_pose)
{
    (void)robot_pose;
    if (!viewer_3d_)
        return;

    // Segmented 3D object processing/projection is intentionally disabled.
    viewer_3d_->update_segmented_points({});
    viewer_3d_->update_segmented_boxes({});
    grounding_focus_points_.clear();
    grounding_focus_label_.clear();
    grounding_world_index_ = -1;
    if (grounding_status_label_ && grounding_cam_label_ && grounding_world_label_ && grounding_score_label_)
    {
        grounding_status_label_->setText("Status: segmented 3D grounding disabled");
        grounding_cam_label_->setText("Camera object: -");
        grounding_world_label_->setText("World object: -");
        grounding_score_label_->setText("Score: -");
    }
    return;

    try
    {
        const auto tdata = imagesegmentation_proxy->getAll(false);
        std::vector<Eigen::Vector3f> points_layout;
        std::vector<rc::Viewer3D::SegmentedBoxItem> boxes_layout;
        boxes_layout.reserve(1);

        struct CamObjInfo
        {
            bool valid = false;
            std::string label;
            Eigen::Vector2f center = Eigen::Vector2f::Zero();
            Eigen::Vector3f size = Eigen::Vector3f::Zero();
            float yaw_rad = 0.f;
            float img_dist = std::numeric_limits<float>::max();
            float img_area = 0.f;
        };
        std::vector<CamObjInfo> cam_infos(tdata.objects.size());
        std::vector<std::vector<Eigen::Vector3f>> obj_points_by_idx(tdata.objects.size());
        std::vector<rc::Viewer3D::SegmentedBoxItem> obj_box_by_idx(tdata.objects.size());
        std::vector<bool> obj_box_valid(tdata.objects.size(), false);

        std::size_t fallback_idx = tdata.objects.size();
        std::size_t fallback_points = 0;

        for (std::size_t obj_idx = 0; obj_idx < tdata.objects.size(); ++obj_idx)
        {
            const auto &obj = tdata.objects[obj_idx];
            std::vector<Eigen::Vector3f> obj_points_layout_all;
            obj_points_layout_all.reserve(obj.points3D.size());

            if (obj.points3D.size() > fallback_points)
            {
                fallback_points = obj.points3D.size();
                fallback_idx = obj_idx;
            }

            for (const auto &p : obj.points3D)
            {
                // points3D now come in RoboComp local frame.
                // Convert camera frame -> robot frame via translation extrinsics,
                // then robot frame -> local layout frame, then local layout -> global layout.
                const float x_robot = p.x + params.CAMERA_TX;
                const float y_robot = p.y + params.CAMERA_TY;
                const float z_robot = p.z + params.CAMERA_TZ;

                const Eigen::Vector2f p_layout_local(x_robot, y_robot);
                const Eigen::Vector2f p_layout = robot_pose * p_layout_local;
                constexpr float z_lift = 0.05f;  // keep points slightly above floor
                const Eigen::Vector3f p3(p_layout.x(), p_layout.y(), z_robot + z_lift);
                obj_points_layout_all.emplace_back(p3);
            }

            std::vector<Eigen::Vector3f> obj_points_layout;
            obj_points_layout.reserve(obj_points_layout_all.size());
            if (!obj_points_layout_all.empty())
            {
                float min_obj_z = std::numeric_limits<float>::max();
                for (const auto &p : obj_points_layout_all)
                    min_obj_z = std::min(min_obj_z, p.z());

                constexpr float floor_clip_margin = 0.06f;  // remove points near per-object floor level
                const float floor_z = min_obj_z + floor_clip_margin;
                for (const auto &p : obj_points_layout_all)
                {
                    if (p.z() > floor_z)
                        obj_points_layout.emplace_back(p);
                }

                // If filtering removed too much, keep original to avoid losing tiny objects
                if (obj_points_layout.size() < 8)
                    obj_points_layout = obj_points_layout_all;
            }

            if (obj_points_layout.size() >= 4)
            {
                Eigen::Vector2f mean_xy = Eigen::Vector2f::Zero();
                float min_z = std::numeric_limits<float>::max();
                float max_z = -std::numeric_limits<float>::max();
                for (const auto &p : obj_points_layout)
                {
                    mean_xy += Eigen::Vector2f(p.x(), p.y());
                    min_z = std::min(min_z, p.z());
                    max_z = std::max(max_z, p.z());
                }
                mean_xy /= static_cast<float>(obj_points_layout.size());

                Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
                for (const auto &p : obj_points_layout)
                {
                    const Eigen::Vector2f d = Eigen::Vector2f(p.x(), p.y()) - mean_xy;
                    cov += d * d.transpose();
                }
                cov /= static_cast<float>(obj_points_layout.size());

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
                Eigen::Vector2f major_axis(1.f, 0.f);
                if (eig.info() == Eigen::Success)
                    major_axis = eig.eigenvectors().col(1).normalized();
                const Eigen::Vector2f minor_axis(-major_axis.y(), major_axis.x());

                float min_u = std::numeric_limits<float>::max();
                float max_u = -std::numeric_limits<float>::max();
                float min_v = std::numeric_limits<float>::max();
                float max_v = -std::numeric_limits<float>::max();
                for (const auto &p : obj_points_layout)
                {
                    const Eigen::Vector2f d = Eigen::Vector2f(p.x(), p.y()) - mean_xy;
                    const float u = d.dot(major_axis);
                    const float v = d.dot(minor_axis);
                    min_u = std::min(min_u, u); max_u = std::max(max_u, u);
                    min_v = std::min(min_v, v); max_v = std::max(max_v, v);
                }

                const float c_u = 0.5f * (min_u + max_u);
                const float c_v = 0.5f * (min_v + max_v);
                const Eigen::Vector2f center_xy = mean_xy + major_axis * c_u + minor_axis * c_v;

                rc::Viewer3D::SegmentedBoxItem box;
                box.label = obj.label.empty() ? ("object_" + std::to_string(obj_idx + 1)) : obj.label;
                box.center = center_xy;
                box.center_height = 0.5f * (min_z + max_z);
                box.size = Eigen::Vector3f(
                    std::max(0.08f, max_u - min_u),
                    std::max(0.08f, max_z - min_z),
                    std::max(0.08f, max_v - min_v));
                box.yaw_rad = std::atan2(major_axis.y(), major_axis.x());

                CamObjInfo &ci = cam_infos[obj_idx];
                ci.valid = true;
                ci.label = obj.label.empty() ? ("object_" + std::to_string(obj_idx + 1)) : obj.label;
                ci.center = center_xy;
                ci.size = box.size;
                ci.yaw_rad = box.yaw_rad;

                obj_points_by_idx[obj_idx] = std::move(obj_points_layout);
                obj_box_by_idx[obj_idx] = box;
                obj_box_valid[obj_idx] = true;
            }
        }

        const Eigen::Affine2f world_to_robot = robot_pose.inverse();
        std::size_t focus_idx = tdata.objects.size();
        constexpr float min_forward_dist = 0.08f;
        constexpr float strict_center_cone = 12.f * static_cast<float>(M_PI) / 180.f;
        constexpr float relaxed_center_cone = 22.f * static_cast<float>(M_PI) / 180.f;

        auto choose_focus_in_cone = [&](float max_abs_bearing) -> std::size_t
        {
            std::size_t best_idx = tdata.objects.size();
            float best_forward = std::numeric_limits<float>::max();
            float best_abs_bearing = std::numeric_limits<float>::max();

            for (std::size_t i = 0; i < cam_infos.size(); ++i)
            {
                if (!cam_infos[i].valid) continue;

                const Eigen::Vector2f c_robot = world_to_robot * cam_infos[i].center;
                const float forward = c_robot.y();
                if (forward <= min_forward_dist) continue;  // must be in front of robot

                const float abs_bearing = std::abs(std::atan2(c_robot.x(), c_robot.y()));
                if (abs_bearing > max_abs_bearing) continue; // must be centered enough

                // First object in line of sight (nearest forward) with center tie-break.
                const bool better_forward = forward < best_forward - 1e-3f;
                const bool same_forward = std::abs(forward - best_forward) <= 1e-3f;
                const bool better_bearing = abs_bearing < best_abs_bearing;
                if (better_forward || (same_forward && better_bearing))
                {
                    best_forward = forward;
                    best_abs_bearing = abs_bearing;
                    best_idx = i;
                }
            }
            return best_idx;
        };

        focus_idx = choose_focus_in_cone(strict_center_cone);
        if (focus_idx >= tdata.objects.size())
            focus_idx = choose_focus_in_cone(relaxed_center_cone);

        if (focus_idx >= tdata.objects.size())
        {
            // Last fallback: nearest bearing among forward objects.
            float best_abs_bearing = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < cam_infos.size(); ++i)
            {
                if (!cam_infos[i].valid) continue;
                const Eigen::Vector2f c_robot = world_to_robot * cam_infos[i].center;
                if (c_robot.y() <= min_forward_dist) continue;
                const float abs_bearing = std::abs(std::atan2(c_robot.x(), c_robot.y()));
                if (abs_bearing < best_abs_bearing)
                {
                    best_abs_bearing = abs_bearing;
                    focus_idx = i;
                }
            }
        }

        if (focus_idx >= tdata.objects.size())
            focus_idx = fallback_idx;

        if (focus_idx < obj_points_by_idx.size() && obj_box_valid[focus_idx])
        {
            points_layout = obj_points_by_idx[focus_idx];
            boxes_layout.clear();
            boxes_layout.emplace_back(obj_box_by_idx[focus_idx]);
            grounding_focus_points_ = points_layout;
            grounding_focus_center_ = cam_infos[focus_idx].center;
            grounding_focus_label_ = cam_infos[focus_idx].label;
        }

        if (focus_idx < cam_infos.size() && cam_infos[focus_idx].valid)
        {
            grounding_focus_center_ = cam_infos[focus_idx].center;
            grounding_focus_label_ = cam_infos[focus_idx].label;
        }

        viewer_3d_->update_segmented_points(points_layout);
        viewer_3d_->update_segmented_boxes(boxes_layout);

        if (grounding_status_label_ && grounding_cam_label_ && grounding_world_label_ && grounding_score_label_)
        {
            if (focus_idx >= cam_infos.size() || !cam_infos[focus_idx].valid)
            {
                grounding_focus_points_.clear();
                grounding_focus_label_.clear();
                grounding_world_index_ = -1;
                grounding_status_label_->setText("Status: no valid camera box to ground");
                grounding_cam_label_->setText("Camera object: -");
                grounding_world_label_->setText("World object: -");
                grounding_score_label_->setText("Score: -");
            }
            else
            {
                const auto &cam = cam_infos[focus_idx];
                grounding_cam_label_->setText(QString("Camera object: %1").arg(QString::fromStdString(cam.label)));

                const Eigen::Vector2f robot_pos = robot_pose.translation();
                Eigen::Vector2f forward = robot_pose.linear() * Eigen::Vector2f(0.f, 1.f);
                if (forward.norm() < 1e-6f) forward = Eigen::Vector2f(0.f, 1.f);
                forward.normalize();
                const Eigen::Vector2f left(-forward.y(), forward.x());

                const Eigen::Vector2f cam_center_robot = world_to_robot * cam.center;
                const float cam_bearing = std::atan2(cam_center_robot.x(), cam_center_robot.y());
                auto wrap_angle = [](float a)
                {
                    while (a > static_cast<float>(M_PI)) a -= static_cast<float>(2.0 * M_PI);
                    while (a < static_cast<float>(-M_PI)) a += static_cast<float>(2.0 * M_PI);
                    return a;
                };
                constexpr float max_bearing_diff = 35.f * static_cast<float>(M_PI) / 180.f;

                float best_score = -1.f;
                QString best_world = "-";
                float best_dist = 0.f, best_lat = 0.f;
                float best_size_ratio = 1.f;
                float best_center_dist = 0.f;
                float best_bearing_deg = 0.f;
                int best_world_index = -1;
                bool matched_with_relaxed_gate = false;

                auto evaluate_candidates = [&](bool strict_bearing_gate)
                {
                    float local_best_score = -1.f;
                    QString local_best_world = "-";
                    float local_best_dist = 0.f, local_best_lat = 0.f;
                    float local_best_size_ratio = 1.f;
                    float local_best_center_dist = 0.f;
                    float local_best_bearing_deg = 0.f;
                    int local_best_world_index = -1;

                    for (std::size_t world_idx = 0; world_idx < layout_manager_.furniture().size(); ++world_idx)
                    {
                        const auto &fp = layout_manager_.furniture()[world_idx];
                        if (fp.vertices.size() < 3) continue;

                        Eigen::Vector2f cen = Eigen::Vector2f::Zero();
                        float min_x = fp.vertices[0].x(), max_x = fp.vertices[0].x();
                        float min_y = fp.vertices[0].y(), max_y = fp.vertices[0].y();
                        for (const auto &v : fp.vertices)
                        {
                            cen += v;
                            min_x = std::min(min_x, v.x()); max_x = std::max(max_x, v.x());
                            min_y = std::min(min_y, v.y()); max_y = std::max(max_y, v.y());
                        }
                        cen /= static_cast<float>(fp.vertices.size());

                        const Eigen::Vector2f diff = cen - robot_pos;
                        const float along = diff.dot(forward);
                        if (along <= 0.05f) continue;
                        const float lateral = std::abs(diff.dot(left));
                        const float dist = diff.norm();

                        const Eigen::Vector2f w_robot = world_to_robot * cen;
                        const float world_bearing = std::atan2(w_robot.x(), w_robot.y());
                        const float bearing_diff = std::abs(wrap_angle(world_bearing - cam_bearing));
                        const bool bearing_gate_ok = bearing_diff <= max_bearing_diff;

                        const float center_dist = (cen - cam.center).norm();

                        const float world_sx = std::max(0.08f, max_x - min_x);
                        const float world_sz = std::max(0.08f, max_y - min_y);
                        const float cam_area = std::max(0.01f, cam.size.x() * cam.size.z());
                        const float world_area = std::max(0.01f, world_sx * world_sz);
                        const float size_ratio = cam_area / world_area;

                        const float score_pose = 1.f / (1.f + dist);
                        const float score_lat = 1.f / (1.f + lateral);
                        const float score_center = 1.f / (1.f + center_dist);
                        const float score_bearing = 1.f - std::min(1.f, bearing_diff / (2.f * max_bearing_diff));
                        const float score_size = 1.f / (1.f + std::abs(std::log(size_ratio)));

                        const QString qcam = QString::fromStdString(cam.label).toLower();
                        const QString qworld = QString::fromStdString(fp.label).toLower();
                        const float score_label = (qcam.contains(qworld) || qworld.contains(qcam)) ? 1.f : 0.f;

                        const float total = strict_bearing_gate
                            ? (0.15f * score_pose + 0.10f * score_lat + 0.45f * score_center + 0.20f * score_bearing + 0.08f * score_size + 0.02f * score_label)
                            : (0.18f * score_pose + 0.12f * score_lat + 0.52f * score_center + 0.10f * score_bearing + 0.06f * score_size + 0.02f * score_label);

                        if (strict_bearing_gate && !bearing_gate_ok) continue;

                        if (total > local_best_score)
                        {
                            local_best_score = total;
                            local_best_world = QString::fromStdString(fp.label.empty() ? fp.id : fp.label);
                            local_best_dist = dist;
                            local_best_lat = lateral;
                            local_best_size_ratio = size_ratio;
                            local_best_center_dist = center_dist;
                            local_best_bearing_deg = qRadiansToDegrees(bearing_diff);
                            local_best_world_index = static_cast<int>(world_idx);
                        }
                    }

                    if (local_best_score > 0.f)
                    {
                        best_score = local_best_score;
                        best_world = local_best_world;
                        best_dist = local_best_dist;
                        best_lat = local_best_lat;
                        best_size_ratio = local_best_size_ratio;
                        best_center_dist = local_best_center_dist;
                        best_bearing_deg = local_best_bearing_deg;
                        best_world_index = local_best_world_index;
                        return true;
                    }
                    return false;
                };

                const bool strict_match = evaluate_candidates(true);
                if (!strict_match)
                {
                    const bool relaxed_match = evaluate_candidates(false);
                    matched_with_relaxed_gate = relaxed_match;
                }

                if (best_score > 0.f)
                {
                    grounding_world_index_ = best_world_index;
                    grounding_status_label_->setText(matched_with_relaxed_gate
                        ? "Status: grounded to front world object (relaxed alignment)"
                        : "Status: grounded to front world object");
                    grounding_world_label_->setText(QString("World object: %1").arg(best_world));
                    grounding_score_label_->setText(
                        QString("Score: %1 | center=%2m d=%3m lat=%4m ang=%5° areaRatio=%6")
                            .arg(best_score, 0, 'f', 2)
                            .arg(best_center_dist, 0, 'f', 2)
                            .arg(best_dist, 0, 'f', 2)
                            .arg(best_lat, 0, 'f', 2)
                            .arg(best_bearing_deg, 0, 'f', 1)
                            .arg(best_size_ratio, 0, 'f', 2));
                }
                else
                {
                    grounding_world_index_ = -1;
                    grounding_status_label_->setText("Status: no front candidate aligned with camera object");
                    grounding_world_label_->setText("World object: -");
                    grounding_score_label_->setText("Score: -");
                }
            }
        }
    }
    catch (const Ice::Exception &)
    {
        viewer_3d_->update_segmented_points({});
        viewer_3d_->update_segmented_boxes({});
        grounding_focus_points_.clear();
        grounding_focus_label_.clear();
        grounding_world_index_ = -1;
        if (grounding_status_label_ && grounding_cam_label_ && grounding_world_label_ && grounding_score_label_)
        {
            grounding_status_label_->setText("Status: segmentation unavailable");
            grounding_cam_label_->setText("Camera object: -");
            grounding_world_label_->setText("World object: -");
            grounding_score_label_->setText("Score: -");
        }
    }
}
