#include "specificworker.h"

#include <algorithm>
#include <limits>

void SpecificWorker::slot_new_target(QPointF pos)
{
    if (!loc_initialized_.load() || !path_planner_.is_ready())
    {
        qWarning() << "Cannot set target: room not initialized or planner not ready";
        return;
    }

    const Eigen::Vector2f target(pos.x(), pos.y());

    // Get current robot pose (thread-safe)
    const auto state = get_loc_state();
    const Eigen::Vector2f robot_pos(state[2], state[3]);

    qInfo() << "[PathPlanner] Robot at (" << robot_pos.x() << "," << robot_pos.y()
            << ") Target at (" << target.x() << "," << target.y() << ")"
            << " Navigable polygon:" << path_planner_.get_navigable_polygon().size() << "vertices"
            << " Inside(robot):" << path_planner_.is_inside(robot_pos)
            << " Inside(target):" << path_planner_.is_inside(target);

    // Plan path
    const auto path = path_planner_.plan(robot_pos, target);
    if (path.empty())
    {
        qWarning() << "No path found to target (" << pos.x() << "," << pos.y() << ")";
        clear_path();
        return;
    }

    current_path_ = path;
    draw_path(path);

    // Compute total path length
    float total = 0.f;
    for (size_t i = 1; i < path.size(); ++i)
        total += (path[i] - path[i - 1]).norm();

    // Activate the trajectory controller to follow the path (applies elastic relaxation)
    trajectory_controller_.set_path(path);

    // Redraw with the relaxed path so the viewer shows what the controller actually follows
    draw_path(trajectory_controller_.get_path());

    start_episode("goto_point", Eigen::Vector2f(pos.x(), pos.y()));

    qInfo() << "Path planned:" << path.size() << "waypoints," << total << "m to ("
            << pos.x() << "," << pos.y() << ")";
}

void SpecificWorker::send_velocity_command(float adv, float side, float rot)
{
    try
    {
        omnirobot_proxy->setSpeedBase(side*1000.f, adv*1000.f, rot);
    }
    catch (const Ice::Exception &e)
    {
        static int err_count = 0;
        if (++err_count % 100 == 1)
            qWarning() << "OmniRobot proxy error:" << e.what();
    }
}

std::vector<Eigen::Vector2f> SpecificWorker::cluster_lidar_to_polygon(
    const std::vector<Eigen::Vector3f>& lidar_points,
    const Eigen::Vector2f& blockage_center_room,
    float search_radius,
    const Eigen::Affine2f& robot_pose,
    float& height_out) const
{
    height_out = 0.f;

    // Transform blockage center to robot frame
    const Eigen::Rotation2Df rot(robot_pose.rotation());
    const Eigen::Vector2f t = robot_pose.translation();
    const Eigen::Vector2f center_robot = rot.inverse() * (blockage_center_room - t);

    // Collect 2D lidar points within search_radius of blockage center (robot frame)
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
        return {};  // too few points to form a polygon

    // Estimate height from Z range of clustered points
    if (z_max > z_min)
        height_out = z_max - z_min;

    // Compute centroid
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (const auto& p : cluster) centroid += p;
    centroid /= static_cast<float>(cluster.size());

    // PCA to find principal axes
    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& p : cluster)
    {
        const Eigen::Vector2f d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(cluster.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
    // Eigenvectors sorted ascending — col(1) is major axis
    const Eigen::Vector2f axis_major = eig.eigenvectors().col(1);
    const Eigen::Vector2f axis_minor = eig.eigenvectors().col(0);

    // Project points onto axes, find extents
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

    // Add small margin (planner already does Minkowski expansion by robot_radius)
    const float margin = TEMP_OBSTACLE_MARGIN;
    min_major -= margin;
    max_major += margin;
    min_minor -= margin;
    max_minor += margin;

    // Build OBB corners in robot frame
    std::vector<Eigen::Vector2f> corners_robot = {
        centroid + min_major * axis_major + min_minor * axis_minor,
        centroid + max_major * axis_major + min_minor * axis_minor,
        centroid + max_major * axis_major + max_minor * axis_minor,
        centroid + min_major * axis_major + max_minor * axis_minor
    };

    // Transform to room frame
    std::vector<Eigen::Vector2f> corners_room;
    corners_room.reserve(4);
    for (const auto& c : corners_robot)
        corners_room.push_back(rot * c + t);

    return corners_room;
}

bool SpecificWorker::replan_around_obstacle(const std::vector<Eigen::Vector2f>& obstacle_polygon,
                                            float obstacle_height,
                                            const Eigen::Vector2f& center,
                                            const Eigen::Affine2f& robot_pose)
{
    if (obstacle_polygon.size() < 3 || current_path_.empty())
        return false;

    // Check if this obstacle is near an existing temp obstacle — merge if so
    bool merged = false;
    for (auto& existing : temp_obstacles_)
    {
        if ((existing.center - center).norm() < TEMP_OBSTACLE_MERGE_DIST)
        {
            // Replace with new, larger polygon
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

    // Rebuild obstacle list: static furniture + all temp obstacles
    std::vector<std::vector<Eigen::Vector2f>> all_obstacles;
    for (const auto& fp : furniture_polygons_)
        all_obstacles.push_back(fp.vertices);
    for (const auto& to : temp_obstacles_)
        all_obstacles.push_back(to.vertices);

    path_planner_.set_obstacles(all_obstacles);
    trajectory_controller_.set_static_obstacles(all_obstacles);

    // Replan from current robot position to original target
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

    // Redraw with relaxed path
    draw_path(trajectory_controller_.get_path());
    draw_temp_obstacles();

    qInfo() << "[ObstacleAvoid] Replanned:" << new_path.size() << "waypoints around obstacle at ("
            << center.x() << "," << center.y() << ") temp_obstacles:" << temp_obstacles_.size();
    return true;
}

void SpecificWorker::cleanup_temp_obstacles()
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
        // Rebuild obstacle list with only furniture + remaining temp obstacles
        std::vector<std::vector<Eigen::Vector2f>> all_obstacles;
        for (const auto& fp : furniture_polygons_)
            all_obstacles.push_back(fp.vertices);
        for (const auto& to : temp_obstacles_)
            all_obstacles.push_back(to.vertices);

        path_planner_.set_obstacles(all_obstacles);
        trajectory_controller_.set_static_obstacles(all_obstacles);
        draw_temp_obstacles();
    }
}
