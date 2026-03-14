#include "navigation_manager.h"
#include "layout_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <print>
#include <QString>

// ---------------------------------------------------------------------------
// plan_to_target — called from 2D-viewer click (main thread)
// ---------------------------------------------------------------------------
void NavigationManager::plan_to_target(const Eigen::Vector2f& target)
{
    if (!path_planner_.is_ready())
    {
        qWarning() << "Cannot set target: planner not ready";
        return;
    }

    const auto state = ctx_.get_loc_state();
    const Eigen::Vector2f robot_pos(state[2], state[3]);

    qInfo() << "[PathPlanner] Robot at (" << robot_pos.x() << "," << robot_pos.y()
            << ") Target at (" << target.x() << "," << target.y() << ")"
            << " Navigable polygon:" << path_planner_.get_navigable_polygon().size() << "vertices"
            << " Inside(robot):" << path_planner_.is_inside(robot_pos)
            << " Inside(target):" << path_planner_.is_inside(target);

    const auto path = path_planner_.plan(robot_pos, target);
    if (path.empty())
    {
        qWarning() << "No path found to target (" << target.x() << "," << target.y() << ")";
        clear();
        return;
    }

    current_path_ = path;
    if (ctx_.draw_path) ctx_.draw_path(path);

    float total = 0.f;
    for (size_t i = 1; i < path.size(); ++i)
        total += (path[i] - path[i - 1]).norm();

    trajectory_controller_.set_path(path);
    if (ctx_.draw_path) ctx_.draw_path(trajectory_controller_.get_path());

    qInfo() << "Path planned:" << path.size() << "waypoints," << total << "m";
}

// ---------------------------------------------------------------------------
// compute_path — query-only path computation (Navigator_getPath logic)
// ---------------------------------------------------------------------------
RoboCompNavigator::Result NavigationManager::compute_path(
    const Eigen::Vector2f& source, const Eigen::Vector2f& target, float safety)
{
    RoboCompNavigator::Result ret;
    ret.timestamp = static_cast<long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    ret.valid = false;

    if (!path_planner_.is_ready())
    {
        ret.errorMsg = "Planner not ready (layout not initialized)";
        return ret;
    }

    if (!path_planner_.is_inside(source) || !path_planner_.is_inside(target))
    {
        ret.errorMsg = "Source or target is outside navigable area";
        return ret;
    }

    if (safety > 0.f && std::fabs(path_planner_.params.robot_radius - safety) > 1e-4f)
    {
        path_planner_.params.robot_radius = safety;
        if (!ctx_.layout_manager->room_polygon().empty())
            path_planner_.set_polygon(ctx_.layout_manager->room_polygon());
        if (!ctx_.layout_manager->furniture().empty())
        {
            const auto obstacles = ctx_.layout_manager->obstacle_polygons();
            path_planner_.set_obstacles(obstacles);
        }
    }

    const auto path = path_planner_.plan(source, target);
    if (path.empty())
    {
        ret.errorMsg = "No path found";
        return ret;
    }

    ret.path.reserve(path.size());
    for (const auto& p : path)
    {
        RoboCompNavigator::TPoint pt;
        pt.x = p.x();
        pt.y = p.y();
        ret.path.push_back(pt);
    }

    if (ctx_.draw_path)
        ctx_.draw_path(path);

    ret.valid = true;
    ret.errorMsg = "ok";
    return ret;
}

// ---------------------------------------------------------------------------
// goto_point — plan + activate controller (Navigator_gotoPoint logic)
// ---------------------------------------------------------------------------
bool NavigationManager::goto_point(const Eigen::Vector2f& target)
{
    nav_target_object_center_.reset();
    nav_target_object_name_.clear();
    object_align_cycles_ = 0;

    const auto state = ctx_.get_loc_state();
    const Eigen::Vector2f source(state[2], state[3]);

    const auto res = compute_path(source, target, path_planner_.params.robot_radius);
    if (!res.valid || res.path.size() < 2)
    {
        qWarning() << "goto_point failed:" << QString::fromStdString(res.errorMsg);
        return false;
    }

    std::vector<Eigen::Vector2f> path_m;
    path_m.reserve(res.path.size());
    for (const auto& p : res.path)
        path_m.emplace_back(p.x, p.y);

    current_path_ = path_m;
    trajectory_controller_.set_path(current_path_);
    if (ctx_.draw_path) ctx_.draw_path(trajectory_controller_.get_path());

    qInfo() << "goto_point: path with" << current_path_.size() << "waypoints activated";
    return true;
}

// ---------------------------------------------------------------------------
// goto_object — find object, compute approach point, plan (Navigator_gotoObject logic)
// ---------------------------------------------------------------------------
bool NavigationManager::goto_object(const std::string& object_name)
{
    if (object_name.empty() || ctx_.layout_manager->furniture().empty())
        return false;

    const QString query = QString::fromStdString(object_name).trimmed();
    if (query.isEmpty())
        return false;

    const rc::FurniturePolygonData* selected = nullptr;

    for (const auto& fp : ctx_.layout_manager->furniture())
    {
        const QString label = QString::fromStdString(fp.label);
        const QString id = QString::fromStdString(fp.id);
        if (label.compare(query, Qt::CaseInsensitive) == 0 || id.compare(query, Qt::CaseInsensitive) == 0)
        {
            selected = &fp;
            break;
        }
    }

    if (selected == nullptr)
    {
        for (const auto& fp : ctx_.layout_manager->furniture())
        {
            const QString label = QString::fromStdString(fp.label);
            const QString id = QString::fromStdString(fp.id);
            if (label.contains(query, Qt::CaseInsensitive) || id.contains(query, Qt::CaseInsensitive))
            {
                selected = &fp;
                break;
            }
        }
    }

    if (selected == nullptr || selected->vertices.empty())
        return false;

    Eigen::Vector2f object_center = Eigen::Vector2f::Zero();
    for (const auto& v : selected->vertices)
        object_center += v;
    object_center /= static_cast<float>(selected->vertices.size());

    const auto state = ctx_.get_loc_state();
    const Eigen::Vector2f robot_pos_m(state[2], state[3]);

    Eigen::Vector2f closest = selected->vertices.front();
    float best_d2 = std::numeric_limits<float>::max();
    const size_t n = selected->vertices.size();
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector2f a = selected->vertices[i];
        const Eigen::Vector2f b = selected->vertices[(i + 1) % n];
        const Eigen::Vector2f ab = b - a;
        const float denom = ab.squaredNorm();

        float t = 0.f;
        if (denom > 1e-8f)
            t = (robot_pos_m - a).dot(ab) / denom;
        t = std::clamp(t, 0.f, 1.f);

        const Eigen::Vector2f proj = a + t * ab;
        const float d2 = (robot_pos_m - proj).squaredNorm();
        if (d2 < best_d2)
        {
            best_d2 = d2;
            closest = proj;
        }
    }

    Eigen::Vector2f outward = robot_pos_m - closest;
    if (outward.squaredNorm() < 1e-8f)
    {
        Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
        for (const auto& v : selected->vertices)
            centroid += v;
        centroid /= static_cast<float>(selected->vertices.size());
        outward = closest - centroid;
    }
    if (outward.squaredNorm() < 1e-8f)
        outward = Eigen::Vector2f(1.f, 0.f);
    outward.normalize();

    const float base_offset = std::max(0.25f, path_planner_.params.robot_radius + 0.15f);

    const Eigen::Vector2f source_m(state[2], state[3]);

    for (const float scale : {1.f, 1.5f, 2.f, 2.5f})
    {
        const Eigen::Vector2f target_m = closest + outward * (base_offset * scale);
        if (!path_planner_.is_inside(target_m))
            continue;

        const auto res = compute_path(source_m, target_m, path_planner_.params.robot_radius);
        if (res.valid && res.path.size() >= 2)
        {
            std::vector<Eigen::Vector2f> path_m;
            path_m.reserve(res.path.size());
            for (const auto& p : res.path)
                path_m.emplace_back(p.x, p.y);

            current_path_ = path_m;
            trajectory_controller_.set_path(current_path_);
            if (ctx_.draw_path) ctx_.draw_path(trajectory_controller_.get_path());

            nav_target_object_center_ = object_center;
            nav_target_object_name_ = selected->label.empty() ? selected->id : selected->label;
            object_align_cycles_ = 0;

            qInfo() << "goto_object: path with" << current_path_.size() << "waypoints for"
                    << QString::fromStdString(nav_target_object_name_);
            return true;
        }
    }

    qWarning() << "goto_object: no reachable stand-off point for"
               << QString::fromStdString(selected->label);
    return false;
}

// ---------------------------------------------------------------------------
// stop / resume / clear
// ---------------------------------------------------------------------------
void NavigationManager::stop()
{
    nav_target_object_center_.reset();
    nav_target_object_name_.clear();
    object_align_cycles_ = 0;
    object_final_align_active_ = false;

    trajectory_controller_.stop();
    current_path_.clear();
    if (ctx_.send_velocity) ctx_.send_velocity(0.f, 0.f, 0.f);
}

void NavigationManager::resume()
{
    if (!current_path_.empty())
        trajectory_controller_.set_path(current_path_);
}

bool NavigationManager::clear(bool stop_controller, bool clear_stored)
{
    object_final_align_active_ = false;
    object_align_cycles_ = 0;
    nav_target_object_center_.reset();
    nav_target_object_name_.clear();

    if (stop_controller && trajectory_controller_.is_active())
    {
        trajectory_controller_.stop();
        if (ctx_.send_velocity) ctx_.send_velocity(0.f, 0.f, 0.f);
    }

    if (ctx_.clear_path_visual) ctx_.clear_path_visual();
    if (clear_stored)
        current_path_.clear();

    return stop_controller; // caller decides whether to finish episode
}

// ---------------------------------------------------------------------------
// compute_step — called from main loop each cycle
// ---------------------------------------------------------------------------
NavigationManager::StepResult NavigationManager::compute_step(
    const std::vector<Eigen::Vector3f>& lidar_points,
    const Eigen::Affine2f& robot_pose)
{
    StepResult result;

    // ── Phase 1: MPPI trajectory control ──
    if (trajectory_controller_.is_active())
    {
        auto t2 = std::chrono::steady_clock::now();
        const auto ctrl = trajectory_controller_.compute(lidar_points, robot_pose);
        auto t3 = std::chrono::steady_clock::now();
        result.mppi_ms = std::chrono::duration<float, std::milli>(t3 - t2).count();
        result.has_ctrl_output = true;
        result.ctrl = ctrl;

        last_ess_ = ctrl.ess;
        last_ess_K_ = ctrl.ess_K;
        if (ctrl.safety_guard_triggered)
            last_safety_guard_trigger_time_ = std::chrono::steady_clock::now();
        result.ess = ctrl.ess;
        result.ess_K = ctrl.ess_K;
        result.safety_guard_triggered = ctrl.safety_guard_triggered;
        result.dist_to_goal = ctrl.dist_to_goal;

        if (ctrl.goal_reached)
        {
            const bool is_goto_object = nav_target_object_center_.has_value();

            if (is_goto_object)
            {
                trajectory_controller_.stop();
                current_path_.clear();
                object_final_align_active_ = true;
                object_align_cycles_ = 0;
                if (ctx_.send_velocity) ctx_.send_velocity(0.f, 0.f, 0.f);
                result.object_align_started = true;
                return result;
            }

            object_align_cycles_ = 0;
            nav_target_object_center_.reset();
            nav_target_object_name_.clear();
            if (ctx_.send_velocity) ctx_.send_velocity(0.f, 0.f, 0.f);
            result.goal_reached = true;
            return result;
        }

        // Not goal-reached: normal control or safeguard
        result.speed = std::hypot(ctrl.adv, ctrl.side);
        result.rot_speed = std::fabs(ctrl.rot);
        result.blocked_like = (ctrl.dist_to_goal > trajectory_controller_.params.goal_threshold)
                           && (result.speed < BLOCKED_SPEED_THRESHOLD);

        if (ctrl.path_blocked)
        {
            safeguard_blockage_center_ = ctrl.blockage_center_room;
            safeguard_blockage_radius_ = std::max(0.20f, ctrl.blockage_radius);
        }

        if (!safeguard_recovery_active_ && ctrl.safety_guard_triggered && result.blocked_like)
        {
            safeguard_recovery_active_ = true;
            safeguard_replan_pending_ = true;
            safeguard_recovery_cycles_ = 0;
            safeguard_clear_counter_ = 0;
            qWarning() << "[SafeguardRecovery] Triggered: starting backward recovery";
        }

        bool recovering_now = false;
        if (safeguard_recovery_active_)
        {
            recovering_now = true;
            safeguard_recovery_cycles_++;
            if (ctrl.safety_guard_triggered)
                safeguard_clear_counter_ = 0;
            else
                safeguard_clear_counter_++;

            const bool recovered = (safeguard_clear_counter_ >= SAFEGUARD_CLEAR_CONFIRM_CYCLES);
            const bool timeout = (safeguard_recovery_cycles_ >= SAFEGUARD_RECOVERY_MAX_CYCLES);

            if (!recovered && !timeout)
            {
                if (ctx_.send_velocity) ctx_.send_velocity(SAFEGUARD_BACKWARD_SPEED, 0.f, 0.f);
                result.adv = SAFEGUARD_BACKWARD_SPEED;
                result.recovering = true;
            }
            else
            {
                safeguard_recovery_active_ = false;
                safeguard_recovery_cycles_ = 0;
                safeguard_clear_counter_ = 0;

                if (ctx_.send_velocity) ctx_.send_velocity(0.f, 0.f, 0.f);

                if (safeguard_replan_pending_)
                {
                    float obs_height = 0.f;
                    const auto polygon = cluster_lidar_to_polygon(
                        lidar_points,
                        safeguard_blockage_center_,
                        std::min(safeguard_blockage_radius_ + 0.15f, 1.1f),
                        robot_pose,
                        obs_height);

                    if (!polygon.empty())
                        replan_around_obstacle(polygon, obs_height, safeguard_blockage_center_, robot_pose);
                    else
                        qWarning() << "[SafeguardRecovery] Recovery ended but obstacle cluster too sparse";
                }
                safeguard_replan_pending_ = false;
                recovering_now = false;
            }
        }

        if (!recovering_now)
        {
            if (ctx_.send_velocity) ctx_.send_velocity(ctrl.adv, ctrl.side, ctrl.rot);
            result.adv = ctrl.adv;
            result.side = ctrl.side;
            result.rot = ctrl.rot;
            result.navigating = true;

            if (ctrl.path_blocked)
            {
                float obs_height = 0.f;
                const auto polygon = cluster_lidar_to_polygon(
                    lidar_points,
                    ctrl.blockage_center_room,
                    std::min(ctrl.blockage_radius + 0.1f, 1.0f),
                    robot_pose,
                    obs_height);

                if (!polygon.empty())
                    replan_around_obstacle(polygon, obs_height, ctrl.blockage_center_room, robot_pose);
                else
                    qWarning() << "[ObstacleAvoid] Blockage detected but LiDAR cluster too sparse";
            }
        }

        result.blocked_like = result.blocked_like || recovering_now;
        return result;
    }

    // ── Phase 2: Object heading alignment (after trajectory controller finishes) ──
    if (object_final_align_active_ && nav_target_object_center_.has_value())
    {
        Eigen::Vector2f object_world = nav_target_object_center_.value();
        if (!nav_target_object_name_.empty())
        {
            const QString target_name = QString::fromStdString(nav_target_object_name_).trimmed();
            for (const auto& fp : ctx_.layout_manager->furniture())
            {
                const QString flabel = QString::fromStdString(fp.label).trimmed();
                const QString fid = QString::fromStdString(fp.id).trimmed();
                if ((flabel.compare(target_name, Qt::CaseInsensitive) == 0 ||
                        fid.compare(target_name, Qt::CaseInsensitive) == 0) &&
                    !fp.vertices.empty())
                {
                    Eigen::Vector2f c = Eigen::Vector2f::Zero();
                    for (const auto& v : fp.vertices) c += v;
                    c /= static_cast<float>(fp.vertices.size());
                    object_world = c;
                    nav_target_object_center_ = c;
                    break;
                }
            }
        }

        const auto loc_state = ctx_.get_loc_state();
        const float rx = loc_state[2];
        const float ry = loc_state[3];
        const float rth = loc_state[4];
        const float cr = std::cos(rth);
        const float sr = std::sin(rth);
        const Eigen::Vector2f d_world = object_world - Eigen::Vector2f(rx, ry);
        const Eigen::Vector2f object_robot(cr * d_world.x() + sr * d_world.y(),
                                           -sr * d_world.x() + cr * d_world.y());

        constexpr float kYawTol = 0.08f;
        constexpr int kMaxAlignCycles = 70;
        const float angle_err = std::atan2(object_robot.x(), object_robot.y());

        const bool done = (std::abs(angle_err) <= kYawTol)
                       || (object_robot.norm() <= 0.05f)
                       || (object_align_cycles_ >= kMaxAlignCycles)
                       || (object_align_cycles_ > 3 && std::signbit(angle_err) != std::signbit(prev_angle_err_)
                           && std::abs(angle_err) < 0.18f);

        if (done)
        {
            object_final_align_active_ = false;
            object_align_cycles_ = 0;
            nav_target_object_center_.reset();
            nav_target_object_name_.clear();
            if (ctx_.send_velocity) ctx_.send_velocity(0.f, 0.f, 0.f);
            result.object_align_done = true;
            return result;
        }

        const float kP = 1.2f;
        float rot_cmd = std::clamp(kP * angle_err, -0.45f, 0.45f);
        if (std::abs(rot_cmd) < 0.06f)
            rot_cmd = (angle_err > 0.f) ? 0.06f : -0.06f;

        if (ctx_.send_velocity) ctx_.send_velocity(0.f, 0.f, rot_cmd);
        result.rot = rot_cmd;
        prev_angle_err_ = angle_err;
        object_align_cycles_++;
    }

    return result;
}

// ---------------------------------------------------------------------------
// get_status — thread-safe status query (Navigator_getStatus logic)
// ---------------------------------------------------------------------------
RoboCompNavigator::NavigationStatus NavigationManager::get_status(float current_speed) const
{
    RoboCompNavigator::NavigationStatus status;

    const auto state = ctx_.get_loc_state();
    status.currentPosition.x = state[2];
    status.currentPosition.y = state[3];
    status.currentOrientation = state[4];

    status.currentTarget.x = 0.f;
    status.currentTarget.y = 0.f;
    status.distanceToTarget = 0.f;
    status.estimatedTime = 0.f;
    status.pathWaypointsRemaining = 0;
    status.currentSpeed = current_speed;

    if (state.isZero())
    {
        status.state = RoboCompNavigator::NavigationState::ERROR;
        status.statusMessage = "Localization not initialized";
        return status;
    }

    if (!current_path_.empty())
    {
        const auto& goal = current_path_.back();
        status.currentTarget.x = goal.x();
        status.currentTarget.y = goal.y();
        status.distanceToTarget = (goal - Eigen::Vector2f(status.currentPosition.x,
                                                           status.currentPosition.y)).norm();
        status.pathWaypointsRemaining = static_cast<int>(current_path_.size());
        status.estimatedTime = (current_speed > 1e-3f)
            ? status.distanceToTarget / current_speed : -1.f;
    }

    if (trajectory_controller_.is_active())
    {
        // Note: low_speed_block_timer state is mutable in the original; for const query
        // we only report NAVIGATING (block detection is handled in compute_step)
        status.state = RoboCompNavigator::NavigationState::NAVIGATING;
        status.statusMessage = "Navigating";
    }
    else if (!current_path_.empty())
    {
        status.state = RoboCompNavigator::NavigationState::PAUSED;
        status.statusMessage = "Path available but controller paused";
    }
    else
    {
        status.state = RoboCompNavigator::NavigationState::IDLE;
        status.statusMessage = "Idle";
    }

    return status;
}

// ---------------------------------------------------------------------------
// cluster_lidar_to_polygon — PCA-based OBB from LiDAR points near blockage
// ---------------------------------------------------------------------------
std::vector<Eigen::Vector2f> NavigationManager::cluster_lidar_to_polygon(
    const std::vector<Eigen::Vector3f>& lidar_points,
    const Eigen::Vector2f& blockage_center_room,
    float search_radius,
    const Eigen::Affine2f& robot_pose,
    float& height_out) const
{
    height_out = 0.f;

    const Eigen::Rotation2Df rot(robot_pose.rotation());
    const Eigen::Vector2f t = robot_pose.translation();
    const Eigen::Vector2f center_robot = rot.inverse() * (blockage_center_room - t);

    const float r2 = search_radius * search_radius;
    std::vector<Eigen::Vector2f> cluster;
    float z_min =  std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    for (const auto& p : lidar_points)
    {
        const Eigen::Vector2f p2d(p.x(), p.y());
        if ((p2d - center_robot).squaredNorm() < r2)
        {
            cluster.push_back(p2d);
            z_min = std::min(z_min, p.z());
            z_max = std::max(z_max, p.z());
        }
    }

    if (cluster.size() < 3)
        return {};

    if (z_max > z_min)
        height_out = z_max - z_min;

    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (const auto& p : cluster) centroid += p;
    centroid /= static_cast<float>(cluster.size());

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& p : cluster)
    {
        const Eigen::Vector2f d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(cluster.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
    const Eigen::Vector2f axis_major = eig.eigenvectors().col(1);
    const Eigen::Vector2f axis_minor = eig.eigenvectors().col(0);

    float min_major = std::numeric_limits<float>::max(), max_major = -std::numeric_limits<float>::max();
    float min_minor = std::numeric_limits<float>::max(), max_minor = -std::numeric_limits<float>::max();
    for (const auto& p : cluster)
    {
        const Eigen::Vector2f d = p - centroid;
        const float proj_maj = d.dot(axis_major);
        const float proj_min = d.dot(axis_minor);
        min_major = std::min(min_major, proj_maj);
        max_major = std::max(max_major, proj_maj);
        min_minor = std::min(min_minor, proj_min);
        max_minor = std::max(max_minor, proj_min);
    }

    const float margin = TEMP_OBSTACLE_MARGIN;
    min_major -= margin;
    max_major += margin;
    min_minor -= margin;
    max_minor += margin;

    std::vector<Eigen::Vector2f> corners_robot = {
        centroid + min_major * axis_major + min_minor * axis_minor,
        centroid + max_major * axis_major + min_minor * axis_minor,
        centroid + max_major * axis_major + max_minor * axis_minor,
        centroid + min_major * axis_major + max_minor * axis_minor
    };

    std::vector<Eigen::Vector2f> corners_room;
    corners_room.reserve(4);
    for (const auto& c : corners_robot)
        corners_room.push_back(rot * c + t);

    return corners_room;
}

// ---------------------------------------------------------------------------
// replan_around_obstacle — add temp obstacle and replan current path
// ---------------------------------------------------------------------------
bool NavigationManager::replan_around_obstacle(
    const std::vector<Eigen::Vector2f>& obstacle_polygon,
    float obstacle_height,
    const Eigen::Vector2f& center,
    const Eigen::Affine2f& robot_pose)
{
    if (obstacle_polygon.size() < 3 || current_path_.empty())
        return false;

    bool merged = false;
    for (auto& existing : temp_obstacles_)
    {
        if ((existing.center - center).norm() < TEMP_OBSTACLE_MERGE_DIST)
        {
            existing.vertices = obstacle_polygon;
            existing.center = center;
            existing.created = std::chrono::steady_clock::now();
            existing.replan_count++;
            if (obstacle_height > 0.05f) existing.height = obstacle_height;
            merged = true;
            break;
        }
    }

    if (!merged)
    {
        TempObstacle obs;
        obs.vertices = obstacle_polygon;
        obs.center = center;
        obs.created = std::chrono::steady_clock::now();
        obs.replan_count = 1;
        obs.height = obstacle_height;
        temp_obstacles_.push_back(std::move(obs));
    }

    std::vector<std::vector<Eigen::Vector2f>> all_obstacles;
    for (const auto& fp : ctx_.layout_manager->furniture())
        all_obstacles.push_back(fp.vertices);
    for (const auto& to : temp_obstacles_)
        all_obstacles.push_back(to.vertices);

    path_planner_.set_obstacles(all_obstacles);
    trajectory_controller_.set_static_obstacles(all_obstacles);

    const Eigen::Vector2f robot_pos = robot_pose.translation();
    const Eigen::Vector2f target = current_path_.back();

    const auto new_path = path_planner_.plan(robot_pos, target);
    if (new_path.empty())
    {
        qWarning() << "[ObstacleAvoid] Replan failed — no path found around obstacle";
        return false;
    }

    current_path_ = new_path;
    trajectory_controller_.set_path(new_path);

    if (ctx_.draw_path) ctx_.draw_path(trajectory_controller_.get_path());
    if (ctx_.draw_temp_obstacles) ctx_.draw_temp_obstacles();

    qInfo() << "[ObstacleAvoid] Replanned:" << new_path.size() << "waypoints around obstacle at ("
            << center.x() << "," << center.y() << ") temp_obstacles:" << temp_obstacles_.size();
    return true;
}

// ---------------------------------------------------------------------------
// cleanup_temp_obstacles — expire old obstacles
// ---------------------------------------------------------------------------
void NavigationManager::cleanup_temp_obstacles()
{
    const auto now = std::chrono::steady_clock::now();
    bool removed = false;

    temp_obstacles_.erase(
        std::remove_if(temp_obstacles_.begin(), temp_obstacles_.end(),
            [&](const TempObstacle& o)
            {
                const float age = std::chrono::duration<float>(now - o.created).count();
                if (age > TEMP_OBSTACLE_TIMEOUT_SEC)
                {
                    removed = true;
                    qInfo() << "[ObstacleAvoid] Removing expired temp obstacle at ("
                            << o.center.x() << "," << o.center.y() << ") age:" << age << "s";
                    return true;
                }
                return false;
            }),
        temp_obstacles_.end());

    if (removed)
    {
        std::vector<std::vector<Eigen::Vector2f>> all_obstacles;
        for (const auto& fp : ctx_.layout_manager->furniture())
            all_obstacles.push_back(fp.vertices);
        for (const auto& to : temp_obstacles_)
            all_obstacles.push_back(to.vertices);

        path_planner_.set_obstacles(all_obstacles);
        trajectory_controller_.set_static_obstacles(all_obstacles);
        if (ctx_.draw_temp_obstacles) ctx_.draw_temp_obstacles();
    }
}
