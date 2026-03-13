#include "specificworker.h"

#include <QMetaObject>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <limits>

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
        const bool guard_recent = (last_safety_guard_trigger_time_.time_since_epoch().count() > 0)
                               && (std::chrono::duration<float>(now - last_safety_guard_trigger_time_).count() <= SAFETY_GUARD_UI_HOLD_SEC);
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

    const float ess_ratio_pct = (last_ess_K_ > 0) ? (last_ess_ / static_cast<float>(last_ess_K_)) * 100.f : 0.f;
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
    viewer->robot_poly()->setPos(display_x, display_y);
    viewer->robot_poly()->setRotation(qRadiansToDegrees(display_angle));

    if (viewer_3d_)
        viewer_3d_->update_robot_pose(display_x, display_y, display_angle);

    if (auto_center_ || !initial_center_done_)
    {
        viewer->centerOn(display_x, display_y);
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

    if (cov_ellipse_item_ == nullptr)
    {
        cov_ellipse_item_ = viewer->scene.addEllipse(
            -radius_x, -radius_y, 2*radius_x, 2*radius_y,
            QPen(QColor(255, 50, 50), 0.03),
            QBrush(QColor(255, 100, 100, 80)));
        cov_ellipse_item_->setZValue(100);
    }
    else
    {
        cov_ellipse_item_->setRect(-radius_x, -radius_y, 2*radius_x, 2*radius_y);
    }

    cov_ellipse_item_->setPos(display_x, display_y);
    cov_ellipse_item_->setRotation(qRadiansToDegrees(ellipse_angle));
}

void SpecificWorker::draw_estimated_room(const Eigen::Matrix<float, 5, 1> &state)
{
    static QGraphicsRectItem* estimated_room_item = nullptr;

    if (!room_polygon_.empty())
    {
        if (estimated_room_item != nullptr)
        {
            viewer->scene.removeItem(estimated_room_item);
            delete estimated_room_item;
            estimated_room_item = nullptr;
        }
        return;
    }

    const float width = state[0];
    const float length = state[1];

    QRectF room_rect(-width/2, -length/2, width, length);

    if (estimated_room_item == nullptr)
    {
        estimated_room_item = viewer->scene.addRect(room_rect, QPen(Qt::magenta, 0.05), QBrush(Qt::NoBrush));
        estimated_room_item->setZValue(2);
    }
    else
    {
        estimated_room_item->setRect(room_rect);
    }
}

void SpecificWorker::draw_temp_obstacles()
{
    for (auto* item : temp_obstacle_draw_items_)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    temp_obstacle_draw_items_.clear();

    for (const auto& obs : temp_obstacles_)
    {
        QPolygonF poly;
        for (const auto& v : obs.vertices)
            poly << QPointF(v.x(), v.y());
        poly << QPointF(obs.vertices.front().x(), obs.vertices.front().y());

        auto* item = viewer->scene.addPolygon(poly,
            QPen(QColor(255, 100, 0), 0.06),
            QBrush(QColor(255, 100, 0, 60)));
        item->setZValue(5);
        temp_obstacle_draw_items_.push_back(item);
    }

    if (viewer_3d_)
    {
        std::vector<rc::Viewer3D::ObstacleItem> obs_items;
        obs_items.reserve(temp_obstacles_.size());
        for (const auto& obs : temp_obstacles_)
            obs_items.push_back({obs.vertices, obs.height});
        viewer_3d_->update_obstacles(obs_items);
    }
}

void SpecificWorker::draw_trajectory_debug(const rc::TrajectoryController::ControlOutput &ctrl,
                                           const Eigen::Affine2f &robot_pose)
{
    const float rx = robot_pose.translation().x();
    const float ry = robot_pose.translation().y();

    const int num_traj = static_cast<int>(ctrl.trajectories_room.size());

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

        const auto& traj = ctrl.trajectories_room[t];
        const bool is_best = (t == ctrl.best_trajectory_idx);
        const QColor color = is_best ? QColor(0, 220, 0) : QColor(150, 150, 150, 100);
        const float width = is_best ? 0.06f : 0.02f;
        const int z = is_best ? 32 : 28;

        auto& segments = traj_draw_items_[t];

        while (segments.size() < traj.size())
        {
            auto* item = viewer->scene.addLine(0, 0, 0, 0, QPen(color, width));
            item->setZValue(z);
            segments.push_back(item);
        }

        for (size_t s = 0; s + 1 < traj.size(); ++s)
        {
            segments[s]->setLine(traj[s].x(), traj[s].y(), traj[s+1].x(), traj[s+1].y());
            segments[s]->setPen(QPen(color, width));
            segments[s]->setZValue(z);
            segments[s]->setVisible(true);
        }
        for (size_t s = (traj.empty() ? 0 : traj.size() - 1); s < segments.size(); ++s)
            segments[s]->setVisible(false);
    }

    {
        constexpr float cr = 0.15f;
        if (!traj_carrot_marker_)
        {
            traj_carrot_marker_ = viewer->scene.addEllipse(
                -cr, -cr, 2*cr, 2*cr,
                QPen(QColor(255, 140, 0), 0.03f),
                QBrush(QColor(255, 140, 0, 180)));
            traj_carrot_marker_->setZValue(33);
        }
        traj_carrot_marker_->setPos(ctrl.carrot_room.x(), ctrl.carrot_room.y());
        traj_carrot_marker_->setVisible(true);
    }

    {
        if (!traj_robot_to_carrot_)
        {
            traj_robot_to_carrot_ = viewer->scene.addLine(
                rx, ry, ctrl.carrot_room.x(), ctrl.carrot_room.y(),
                QPen(QColor(255, 165, 0, 200), 0.02f, Qt::DashLine));
            traj_robot_to_carrot_->setZValue(29);
        }
        else
        {
            traj_robot_to_carrot_->setLine(rx, ry, ctrl.carrot_room.x(), ctrl.carrot_room.y());
            traj_robot_to_carrot_->setVisible(true);
        }
    }
}

void SpecificWorker::draw_path(const std::vector<Eigen::Vector2f>& path)
{
    clear_path(false, false);

    if (path.size() < 2) return;

    const auto& orig_poly = path_planner_.get_original_polygon();
    for (const auto& v : orig_poly)
    {
        constexpr float r = 0.1f;
        auto* dot = viewer->scene.addEllipse(-r, -r, 2*r, 2*r,
            QPen(QColor(0, 200, 0), 0.02), QBrush(QColor(0, 200, 0, 80)));
        dot->setPos(v.x(), v.y());
        dot->setZValue(17);
        path_draw_items_.push_back(dot);
    }

    if (navigable_poly_item_)
    {
        viewer->scene.removeItem(navigable_poly_item_);
        delete navigable_poly_item_;
        navigable_poly_item_ = nullptr;
    }
    const auto& inner_poly = path_planner_.get_inner_polygon();
    if (inner_poly.size() >= 3)
    {
        QPolygonF qpoly;
        for (const auto& v : inner_poly)
            qpoly << QPointF(v.x(), v.y());
        qpoly << QPointF(inner_poly.front().x(), inner_poly.front().y());
        QPen inner_pen(QColor(255, 140, 0, 200), 0.03, Qt::DashLine);
        navigable_poly_item_ = viewer->scene.addPolygon(qpoly, inner_pen, Qt::NoBrush);
        navigable_poly_item_->setZValue(19);
    }

    for (auto* item : obstacle_expanded_items_)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    obstacle_expanded_items_.clear();
    const auto& exp_obstacles = path_planner_.get_expanded_obstacles();
    for (const auto& obs : exp_obstacles)
    {
        if (obs.size() < 3) continue;
        QPolygonF qpoly_obs;
        for (const auto& v : obs)
            qpoly_obs << QPointF(v.x(), v.y());
        qpoly_obs << QPointF(obs.front().x(), obs.front().y());
        QPen obs_pen(QColor(255, 140, 0, 200), 0.03, Qt::DashLine);
        auto* obs_item = viewer->scene.addPolygon(qpoly_obs, obs_pen, Qt::NoBrush);
        obs_item->setZValue(19);
        obstacle_expanded_items_.push_back(obs_item);
    }

    const auto& nav_poly = path_planner_.get_navigable_polygon();
    for (const auto& v : nav_poly)
    {
        constexpr float r = 0.08f;
        auto* dot = viewer->scene.addEllipse(-r, -r, 2*r, 2*r,
            QPen(QColor(255, 255, 0, 200), 0.01), QBrush(QColor(255, 255, 0, 120)));
        dot->setPos(v.x(), v.y());
        dot->setZValue(18);
        path_draw_items_.push_back(dot);
    }

    const QPen path_pen(QColor(100, 255, 100), 0.04);
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        auto* line = viewer->scene.addLine(
            path[i].x(), path[i].y(),
            path[i + 1].x(), path[i + 1].y(),
            path_pen);
        line->setZValue(20);
        path_draw_items_.push_back(line);
    }

    const QPen wp_pen(Qt::NoPen);
    const QBrush wp_brush(QColor(0, 220, 220));
    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        constexpr float r = 0.06f;
        auto* dot = viewer->scene.addEllipse(-r, -r, 2*r, 2*r, wp_pen, wp_brush);
        dot->setPos(path[i].x(), path[i].y());
        dot->setZValue(21);
        path_draw_items_.push_back(dot);
    }

    const auto& goal = path.back();
    if (target_marker_ == nullptr)
    {
        constexpr float tr = 0.12f;
        target_marker_ = viewer->scene.addEllipse(-tr, -tr, 2*tr, 2*tr,
            QPen(QColor(255, 50, 50), 0.03), QBrush(QColor(255, 50, 50, 120)));
        target_marker_->setZValue(22);
    }
    target_marker_->setPos(goal.x(), goal.y());
    target_marker_->setVisible(true);

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

void SpecificWorker::slot_capture_room_toggled(bool checked)
{
    capturing_room_polygon = checked;

    if (checked)
    {
        // Start capturing - backup existing polygon (don't remove it yet)
        room_polygon_backup_ = room_polygon_;
        room_polygon_.clear();

        // Clear previous vertex markers (these are only for capture mode)
        for (auto* item : polygon_vertex_items)
        {
            viewer->scene.removeItem(item);
            delete item;
        }
        polygon_vertex_items.clear();

        // Keep the existing polygon visible but store reference for later removal
        // Move current polygon_item to backup (will be removed when new capture succeeds)
        polygon_item_backup_ = polygon_item;
        polygon_item = nullptr;

        pushButton_captureRoom->setText("Ctrl+Click vertices...");
        qInfo() << "Room capture started. Use Ctrl+Left click on the scene to add vertices. Ctrl+Click near the first point to close.";
        qInfo() << "Existing polygon preserved until new one is completed.";
    }
    else
    {
        pushButton_captureRoom->setText("Capture Room");

        if (room_polygon_.size() >= 3)
        {
            qInfo() << "Room polygon captured with" << room_polygon_.size() << "vertices";

            // Keep old polygon visible until user saves the new one

            // Clear vertex markers (yellow circles)
            for (auto* item : polygon_vertex_items)
            {
                viewer->scene.removeItem(item);
                delete item;
            }
            polygon_vertex_items.clear();

            // Vertices are already in room frame (where user clicked)
            // Pass them to room_ai via thread-safe command queue
            push_loc_command(LocCmdSetPolygon{room_polygon_});
            path_planner_.set_polygon(room_polygon_);

            // Draw final polygon (fixed in room frame)
            draw_room_polygon();
        }
        else
        {
            qWarning() << "Need at least 3 vertices for a valid polygon. Restoring previous polygon.";

            // Restore the backup polygon
            room_polygon_ = room_polygon_backup_;
            room_polygon_backup_.clear();

            // Remove any partial new polygon drawing
            if (polygon_item)
            {
                viewer->scene.removeItem(polygon_item);
                delete polygon_item;
                polygon_item = nullptr;
            }

            // Restore backup polygon graphic as the active one
            polygon_item = polygon_item_backup_;
            polygon_item_backup_ = nullptr;

            // Clear vertex markers from failed capture
            for (auto* item : polygon_vertex_items)
            {
                viewer->scene.removeItem(item);
                delete item;
            }
            polygon_vertex_items.clear();
        }
    }
}

void SpecificWorker::on_scene_clicked(QPointF pos)
{
    // This function is now only used for polygon capture mode
    // Robot movement is handled by slot_robot_dragging and slot_robot_rotate
    if (!capturing_room_polygon)
        return;

    // Capture mode: add polygon vertices
    // The scene shows points in room frame (transformed by robot_pose)
    // We need to store vertices in room frame (where they appear on screen)
    const Eigen::Vector2f click_pos(pos.x(), pos.y());

    // Check if clicking near the first point to close the polygon
    constexpr float close_threshold = 0.3f;  // 30cm
    if (room_polygon_.size() >= 3)
    {
        const float dist_to_first = (click_pos - room_polygon_.front()).norm();
        if (dist_to_first < close_threshold)
        {
            // Close the polygon
            pushButton_captureRoom->setChecked(false);  // This triggers slot_capture_room_toggled(false)
            return;
        }
    }

    // Add new vertex in room frame coordinates
    room_polygon_.push_back(click_pos);

    // Draw vertex marker
    const float radius = 0.15f;
    auto* vertex_item = viewer->scene.addEllipse(
        -radius, -radius, 2*radius, 2*radius,
        QPen(Qt::yellow, 0.05), QBrush(Qt::yellow));
    vertex_item->setPos(pos.x(), pos.y());
    vertex_item->setZValue(10);
    polygon_vertex_items.push_back(vertex_item);

    // Draw temporary polygon outline
    draw_room_polygon();

    qInfo() << "Added vertex" << room_polygon_.size() << "at (" << pos.x() << "," << pos.y() << ")";
}

void SpecificWorker::draw_room_polygon()
{
    if (room_polygon_.size() < 2)
        return;

    if (polygon_item)
    {
        viewer->scene.removeItem(polygon_item);
        delete polygon_item;
        polygon_item = nullptr;
    }

    QPolygonF poly;
    for (const auto& v : room_polygon_)
        poly << QPointF(v.x(), v.y());

    if (!capturing_room_polygon && room_polygon_.size() >= 3)
        poly << QPointF(room_polygon_.front().x(), room_polygon_.front().y());

    QPen pen(capturing_room_polygon ? Qt::yellow : Qt::magenta, capturing_room_polygon ? 0.08 : 0.15);
    polygon_item = viewer->scene.addPolygon(poly, pen, QBrush(Qt::NoBrush));
    polygon_item->setZValue(8);

    if (viewer_3d_ && !capturing_room_polygon && room_polygon_.size() >= 3)
        viewer_3d_->rebuild_walls(room_polygon_);
}

void SpecificWorker::draw_furniture()
{
    for (auto* item : furniture_draw_items_)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    furniture_draw_items_.clear();

    const QPen furniture_pen(QColor(50, 100, 255), 0.06);
    const QBrush furniture_brush(QColor(50, 100, 255, 40));

    for (const auto& fp : furniture_polygons_)
    {
        if (fp.vertices.size() < 3) continue;

        QPolygonF qpoly;
        for (const auto& v : fp.vertices)
            qpoly << QPointF(v.x(), v.y());
        qpoly << QPointF(fp.vertices.front().x(), fp.vertices.front().y());

        auto* item = viewer->scene.addPolygon(qpoly, furniture_pen, furniture_brush);
        item->setZValue(7);
        furniture_draw_items_.push_back(item);
    }

    apply_ownership_em_visuals();

    if (!furniture_polygons_.empty())
        qInfo() << "[draw_furniture] Drew" << furniture_polygons_.size() << "furniture polygons";

    if (scene_tree_)
    {
        std::vector<SceneTreePanel::FurnitureEntry> entries;
        entries.reserve(furniture_polygons_.size());
        for (const auto& fp : furniture_polygons_)
        {
            SceneTreePanel::FurnitureEntry e;
            e.id       = fp.id;
            e.label    = fp.label;
            e.vertices = fp.vertices;
            entries.push_back(std::move(e));
        }
        scene_tree_->refresh(room_polygon_, entries);
    }

    if (viewer_3d_)
    {
        std::vector<rc::Viewer3D::FurnitureItem> items;
        items.reserve(furniture_polygons_.size());
        for (const auto& fp : furniture_polygons_)
        {
            if (fp.vertices.empty()) continue;
            Eigen::Vector2f cen = Eigen::Vector2f::Zero();
            for (const auto& v : fp.vertices)
            {
                cen += v;
            }
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
            Eigen::Vector2f axis_z(-axis_x.y(), axis_x.x());

            float min_u = std::numeric_limits<float>::max();
            float max_u = -std::numeric_limits<float>::max();
            float min_v = std::numeric_limits<float>::max();
            float max_v = -std::numeric_limits<float>::max();
            for (const auto& p : fp.vertices)
            {
                const Eigen::Vector2f d = p - cen;
                const float u = d.dot(axis_x);
                const float v = d.dot(axis_z);
                min_u = std::min(min_u, u); max_u = std::max(max_u, u);
                min_v = std::min(min_v, v); max_v = std::max(max_v, v);
            }

            const float c_u = 0.5f * (min_u + max_u);
            const float c_v = 0.5f * (min_v + max_v);
            const Eigen::Vector2f obb_center = cen + axis_x * c_u + axis_z * c_v;

            rc::Viewer3D::FurnitureItem fi;
            fi.label    = fp.label;
            fi.centroid = obb_center;
            fi.size     = Eigen::Vector2f(std::max(0.08f, max_u - min_u), std::max(0.08f, max_v - min_v));
            fi.yaw_rad  = std::atan2(axis_x.y(), axis_x.x());
            items.push_back(fi);
        }
        viewer_3d_->update_furniture(items);
    }

    rc::SceneGraphAdapter::rebuild_graph(
        scene_graph_,
        room_polygon_,
        furniture_polygons_,
        [this](const std::string& label) { return model_height_from_label(label); },
        2.6f);

    rebuild_ownership_em_models();
}

void SpecificWorker::update_furniture_draw_item(std::size_t idx)
{
    if (idx >= furniture_polygons_.size())
        return;

    if (idx < furniture_draw_items_.size() && furniture_draw_items_[idx] != nullptr)
    {
        viewer->scene.removeItem(furniture_draw_items_[idx]);
        delete furniture_draw_items_[idx];
        furniture_draw_items_[idx] = nullptr;
    }
    else if (idx >= furniture_draw_items_.size())
    {
        furniture_draw_items_.resize(furniture_polygons_.size(), nullptr);
    }

    const auto& fp = furniture_polygons_[idx];
    if (fp.vertices.size() < 3)
        return;

    const QPen furniture_pen(QColor(50, 100, 255), 0.06);
    const QBrush furniture_brush(QColor(50, 100, 255, 40));

    QPolygonF qpoly;
    for (const auto& v : fp.vertices)
        qpoly << QPointF(v.x(), v.y());
    qpoly << QPointF(fp.vertices.front().x(), fp.vertices.front().y());

    auto* item = viewer->scene.addPolygon(qpoly, furniture_pen, furniture_brush);
    item->setZValue(7);
    furniture_draw_items_[idx] = item;

    apply_ownership_em_visuals();
}
