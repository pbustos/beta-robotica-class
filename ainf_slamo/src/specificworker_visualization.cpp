#include "specificworker.h"

#include <QMetaObject>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <limits>

#include "object_footprints.h"

void SpecificWorker::update_ui(const rc::RoomConceptAI::UpdateResult &res,
                               const rc::VelocityCommand &current_velocity,
                               int fps_val)
{
    const auto nav_status = Navigator_getStatus();
    QString nav_text = "NAV: UNKNOWN";
    QString nav_style = "background-color: #B0B0B0; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
    switch (nav_status.state)
    {
        case RoboCompNavigator::NavigationState::IDLE:
            nav_text = "NAV: IDLE";
            nav_style = "background-color: #B0B0B0; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::NAVIGATING:
            nav_text = "NAV: NAVIGATING";
            nav_style = "background-color: #90EE90; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::PAUSED:
            nav_text = "NAV: PAUSED";
            nav_style = "background-color: #FFF59D; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::REACHED:
            nav_text = "NAV: REACHED";
            nav_style = "background-color: #80CBC4; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::BLOCKED:
            nav_text = "NAV: BLOCKED";
            nav_style = "background-color: #FFB74D; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::ERROR:
            nav_text = "NAV: ERROR";
            nav_style = "background-color: #EF9A9A; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
    }
    static QString last_nav_text, last_nav_style;
    if (QLabel *nav_label = this->findChild<QLabel*>("label_navStatus"); nav_label != nullptr)
    {
        if (nav_text != last_nav_text)
        {
            nav_label->setText(nav_text);
            last_nav_text = nav_text;
        }
        if (nav_style != last_nav_style)
        {
            nav_label->setStyleSheet(nav_style);
            last_nav_style = nav_style;
        }
    }
    else
    {
        const bool webots_mode = params.USE_WEBOTS;
        const QString mode_prefix = webots_mode ? "WEBOTS" : "ROBOT";
        const QString fallback_text = mode_prefix + " | " + nav_text;
        if (label_mode->text() != fallback_text)
            label_mode->setText(fallback_text);
    }

    if (label_safetyGuard != nullptr)
    {
        const auto now = std::chrono::steady_clock::now();
        const auto sg_time = nav_manager_.last_safety_guard_trigger_time();
        const bool guard_recent = (sg_time.time_since_epoch().count() > 0)
                               && (std::chrono::duration<float>(now - sg_time).count() <= NavigationManager::SAFETY_GUARD_UI_HOLD_SEC);
        label_safetyGuard->setText(guard_recent ? "SG: ON" : "SG: OFF");
        label_safetyGuard->setStyleSheet(guard_recent
            ? "background-color: #FF8A80; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;"
            : "background-color: #CFD8DC; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
    }

    // Update QLCDNumber displays
    const float sigma_xy_cm = std::sqrt(res.covariance(0,0) + res.covariance(1,1)) * 100.0f;
    const float sdf_median_cm = res.sdf_mse * 100.0f;
    const float innovation_cm = res.innovation_norm * 100.0f;

    lcdNumber_fps->display(fps_val);
    lcdNumber_loss->display(sdf_median_cm);
    lcdNumber_sigma->display(sigma_xy_cm);
    lcdNumber_velocity->display(std::abs(current_velocity.adv_y));
    lcdNumber_innov->display(innovation_cm);

    const float ess_ratio_pct = (nav_manager_.last_ess_K() > 0) ? (nav_manager_.last_ess() / static_cast<float>(nav_manager_.last_ess_K())) * 100.f : 0.f;
    lcdNumber_ess->display(static_cast<int>(ess_ratio_pct));

    static int ui_slow_counter = 0;
    if (++ui_slow_counter >= 10)
    {
        ui_slow_counter = 0;
        const float cpu_usage = get_cpu_usage();
        lcdNumber_cpu->display(static_cast<int>(cpu_usage));

        static QString last_sigma_color, last_innov_color, last_cpu_color, last_ess_color;

        auto set_style_if_changed = [](QLCDNumber *w, const QString &style, QString &last) {
            if (style != last) { w->setStyleSheet(style); last = style; }
        };

        QString sigma_color;
        if (sigma_xy_cm < 5.0f)       sigma_color = "background-color: #90EE90;";
        else if (sigma_xy_cm < 10.0f)  sigma_color = "background-color: #FFFF00;";
        else if (sigma_xy_cm < 20.0f)  sigma_color = "background-color: #FFA500;";
        else                            sigma_color = "background-color: #FF6B6B;";
        set_style_if_changed(lcdNumber_sigma, sigma_color, last_sigma_color);

        QString innov_color;
        if (innovation_cm < 5.0f)       innov_color = "background-color: #90EE90;";
        else if (innovation_cm < 15.0f)  innov_color = "background-color: #FFFF00;";
        else                              innov_color = "background-color: #FF6B6B;";
        set_style_if_changed(lcdNumber_innov, innov_color, last_innov_color);

        QString ess_color;
        if (ess_ratio_pct > 50.f)       ess_color = "background-color: #90EE90;";
        else if (ess_ratio_pct > 25.f)  ess_color = "background-color: #FFFF00;";
        else if (ess_ratio_pct > 15.f)  ess_color = "background-color: #FFA500;";
        else                             ess_color = "background-color: #FF6B6B;";
        set_style_if_changed(lcdNumber_ess, ess_color, last_ess_color);

        QString cpu_color;
        if (cpu_usage < 30.0f)       cpu_color = "background-color: #90EE90;";
        else if (cpu_usage < 60.0f)  cpu_color = "background-color: #FFFF00;";
        else                          cpu_color = "background-color: #FF6B6B;";
        set_style_if_changed(lcdNumber_cpu, cpu_color, last_cpu_color);
    }
}

void SpecificWorker::display_robot(const Eigen::Affine2f &robot_pose, const Eigen::Matrix3f &covariance)
{
    const float display_x = robot_pose.translation().x();
    const float display_y = robot_pose.translation().y();
    const float display_angle = std::atan2(robot_pose.linear()(1,0), robot_pose.linear()(0,0));
    viewer_2d_->update_robot(display_x, display_y, display_angle);

    if (viewer_3d_)
        viewer_3d_->update_robot_pose(display_x, display_y, display_angle);

    if (auto_center_ || !initial_center_done_)
    {
        viewer_2d_->center_on(display_x, display_y);
        initial_center_done_ = true;
    }

    lcdNumber_robotX->display(static_cast<double>(display_x));
    lcdNumber_robotY->display(static_cast<double>(display_y));
    lcdNumber_robotTheta->display(static_cast<double>(qRadiansToDegrees(display_angle)));

    Eigen::Matrix2f pos_cov = covariance.block<2,2>(0,0);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(pos_cov);
    Eigen::Vector2f eigenvalues = solver.eigenvalues();
    Eigen::Matrix2f eigenvectors = solver.eigenvectors();

    constexpr float confidence_scale = 3.0f;
    const float radius_x = std::sqrt(std::max(0.001f, eigenvalues(0))) * confidence_scale;
    const float radius_y = std::sqrt(std::max(0.001f, eigenvalues(1))) * confidence_scale;

    const float ellipse_angle = std::atan2(eigenvectors(1,0), eigenvectors(0,0));

    viewer_2d_->update_covariance_ellipse(display_x, display_y,
                                          radius_x, radius_y,
                                          qRadiansToDegrees(ellipse_angle));
}

void SpecificWorker::draw_estimated_room(const Eigen::Matrix<float, 5, 1> &state)
{
    const float width  = state[0];
    const float length = state[1];
    viewer_2d_->update_estimated_room_rect(width, length, !layout_manager_.room_polygon().empty());
}

void SpecificWorker::draw_temp_obstacles()
{
    std::vector<std::vector<Eigen::Vector2f>> polys;
    const auto& temp_obs = nav_manager_.temp_obstacles();
    polys.reserve(temp_obs.size());
    for (const auto& obs : temp_obs)
        polys.push_back(obs.vertices);
    viewer_2d_->draw_temp_obstacles(polys);

    if (viewer_3d_)
    {
        std::vector<rc::Viewer3D::ObstacleItem> obs_items;
        obs_items.reserve(temp_obs.size());
        for (const auto& obs : temp_obs)
            obs_items.push_back({obs.vertices, obs.height});
        viewer_3d_->update_obstacles(obs_items);
    }
}

void SpecificWorker::draw_trajectory_debug(const rc::TrajectoryController::ControlOutput &ctrl,
                                           const Eigen::Affine2f &robot_pose)
{
    const float rx = robot_pose.translation().x();
    const float ry = robot_pose.translation().y();

    rc::Viewer2D::TrajDrawData data;
    data.trajectories = ctrl.trajectories_room;
    data.best_idx     = ctrl.best_trajectory_idx;
    data.carrot       = ctrl.carrot_room;
    data.robot_x      = rx;
    data.robot_y      = ry;
    viewer_2d_->draw_trajectory_debug(data);
}

void SpecificWorker::draw_path(const std::vector<Eigen::Vector2f>& path)
{
    clear_path(false, false);

    if (path.size() < 2) return;

    rc::Viewer2D::PathDrawData data;
    data.path              = path;
    data.orig_poly_verts   = nav_manager_.path_planner().get_original_polygon();
    data.inner_poly        = nav_manager_.path_planner().get_inner_polygon();
    data.nav_poly          = nav_manager_.path_planner().get_navigable_polygon();
    data.expanded_obstacles = nav_manager_.path_planner().get_expanded_obstacles();
    viewer_2d_->draw_path(data);

    if (viewer_3d_)
        viewer_3d_->update_path(path);
}

void SpecificWorker::draw_path_threadsafe(const std::vector<Eigen::Vector2f>& path)
{
    if (QThread::currentThread() == this->thread())
    {
        draw_path(path);
    }
    else
    {
        QMetaObject::invokeMethod(this, [this, path]() { draw_path(path); }, Qt::QueuedConnection);
    }
}

void SpecificWorker::slot_edit_layout_toggled(bool checked)
{
    if (checked)
    {
        const auto &poly = layout_manager_.room_polygon();
        if (poly.size() < 3)
        {
            qWarning() << "Edit layout: current room polygon is invalid (needs >= 3 vertices)";
            if (pushButton_editLayout)
            {
                QSignalBlocker blocker(pushButton_editLayout);
                pushButton_editLayout->setChecked(false);
            }
            return;
        }

        editing_room_layout_ = true;
        editing_corner_idx_ = -1;
        editing_room_polygon_ = poly;

        viewer_2d_->set_room_edit_corners(editing_room_polygon_, true);
        viewer_2d_->draw_room_polygon(editing_room_polygon_, false);
        if (viewer_3d_)
            viewer_3d_->rebuild_walls(editing_room_polygon_);

        if (pushButton_editLayout)
            pushButton_editLayout->setText("Save Layout");

        qInfo() << "2D layout edit mode ON";
        return;
    }

    editing_room_layout_ = false;
    editing_corner_idx_ = -1;

    if (pushButton_editLayout)
        pushButton_editLayout->setText("Edit Layout");

    viewer_2d_->set_room_edit_corners({}, false);

    if (editing_room_polygon_.size() >= 3)
    {
        layout_manager_.set_room_polygon(editing_room_polygon_);

        rc::SceneGraphAdapter::rebuild_graph(
            scene_graph_, layout_manager_.room_polygon(), layout_manager_.furniture(),
            [](const std::string& l) { return EMManager::model_height_from_label(l); }, 2.6f);

        draw_room_polygon();
        draw_furniture();

        nav_manager_.trajectory_controller().set_room_boundary(layout_manager_.room_polygon());
        const auto obs = layout_manager_.obstacle_polygons();
        nav_manager_.path_planner().set_obstacles(obs);
        nav_manager_.trajectory_controller().set_static_obstacles(obs);

        save_scene_graph_to_usd();
        qInfo() << "2D layout edit mode OFF: modified layout committed and saved";
    }
    else
    {
        qWarning() << "Edit layout: keeping previous layout because edited polygon is invalid";
        draw_room_polygon();
    }
}

void SpecificWorker::draw_room_polygon()
{
    viewer_2d_->draw_room_polygon(layout_manager_.room_polygon(), false);

    if (viewer_3d_ && layout_manager_.room_polygon().size() >= 3)
        viewer_3d_->rebuild_walls(layout_manager_.room_polygon());
}

void SpecificWorker::draw_furniture()
{
    // Recompute all footprint vertices from the authoritative model
    layout_manager_.refresh_all_from_model();

    // 2D: draw the freshly computed polygons
    viewer_2d_->draw_furniture(layout_manager_.furniture());

    em_manager_.apply_visuals();

    if (!layout_manager_.furniture().empty())
        qInfo() << "[draw_furniture] Drew" << layout_manager_.furniture().size() << "furniture polygons";

    if (scene_tree_)
        scene_tree_->rebuild_from_model();

    if (viewer_3d_)
    {
        std::vector<rc::Viewer3D::FurnitureItem> items;
        items.reserve(layout_manager_.furniture().size());
        for (const auto& fp : layout_manager_.furniture())
        {
            if (fp.vertices.empty()) continue;

            auto node_opt = scene_graph_.get_object_node(fp.label);
            if (node_opt)
            {
                const auto& nd = *node_opt;
                rc::Viewer3D::FurnitureItem fi;
                fi.label    = fp.label;
                fi.centroid = Eigen::Vector2f(nd.translation.x(), nd.translation.y());
                fi.size     = Eigen::Vector2f(std::max(0.08f, nd.extents.x()),
                                              std::max(0.08f, nd.extents.y()));
                fi.yaw_rad  = nd.yaw_rad;
                fi.height   = std::max(0.2f, nd.extents.z());
                fi.tz       = nd.translation.z();
                items.push_back(fi);
            }
        }
        viewer_3d_->update_furniture(items);
    }

    em_manager_.rebuild_models();
}

void SpecificWorker::update_furniture_draw_item(std::size_t idx)
{
    if (idx >= layout_manager_.furniture().size())
        return;

    viewer_2d_->update_furniture_item(idx, layout_manager_.furniture()[idx]);

    em_manager_.apply_visuals();
}
