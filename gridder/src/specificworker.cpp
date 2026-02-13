/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cppitertools/filter.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/range.hpp>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

class TPointVector;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
		
	}
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	// Signal the lidar thread to stop and wait for it
	stop_lidar_thread = true;
	if(read_lidar_th.joinable())
		read_lidar_th.join();
	std::cout << "Lidar thread stopped" << std::endl;

	// Signal the localizer thread to stop and wait for it
	stop_localizer_thread = true;
	if(localizer_th.joinable())
		localizer_th.join();
	std::cout << "Localizer thread stopped" << std::endl;

	// Signal the MPPI thread to stop and wait for it
	stop_mppi_thread = true;
	if(mppi_th.joinable())
		mppi_th.join();
	std::cout << "MPPI thread stopped" << std::endl;
}
void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

//chekpoint robocompUpdater
	std::cout << "Initialize worker" << std::endl;
    const int period = 50;
	setPeriod("Compute", period);
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        // Viewer
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM, false);
        viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("LightBlue"));
        // Don't limit sceneRect - allow unlimited panning
        // viewer->setSceneRect(params.GRID_MAX_DIM);
        viewer->show();

        // Fit the view to show the initial grid area centered on robot
        QRectF initial_view(-3000, -3000, 6000, 6000);  // 6m x 6m area centered on origin
        viewer->fitToScene(initial_view);

        // Initialize Sparse ESDF grid (VoxBlox-style)
        grid_esdf.initialize(static_cast<int>(params.TILE_SIZE), &viewer->scene);

        // Configure A* parameters
        grid_esdf.params().max_astar_nodes = params.MAX_ASTAR_NODES;
        grid_esdf.params().astar_distance_factor = params.ASTAR_DISTANCE_FACTOR;

        // Load pre-computed MRPT map
        std::string map_file = "mapa_webots.gridmap";
        if (Gridder_loadMRPTMap(map_file))
        {
            qInfo() << "[MRPT] Map loaded successfully:" << map_file.c_str();
            external_map_loaded = true;  // Disable dynamic grid updates
        }
        else
            qWarning() << "[MRPT] Failed to load map:" << map_file.c_str();

        // Initialize localizer if enabled and map is loaded
        if (params.USE_LOCALIZER && external_map_loaded)
        {
            localizer.initialize(&grid_esdf, &viewer->scene);

            // Configure localizer parameters
            Localizer::Params loc_params;
            loc_params.initial_particles = params.LOCALIZER_PARTICLES;
            loc_params.min_particles = 100;
            loc_params.max_particles = 1000;
            loc_params.draw_particles = false;  // Controlled by UI checkbox
            loc_params.lidar_subsample = 10;
            localizer.setParams(loc_params);

            // Precompute distance field for fast lookups
            qInfo() << "[Localizer] Precomputing distance field...";
            grid_esdf.precompute_distance_field();

            // Mark map as ready for localization (after precomputation)
            map_ready_for_localization.store(true);
            qInfo() << "[Localizer] Initialized, waiting for first pose...";
        }

        // Lidar thread is created
        read_lidar_th = std::thread(&SpecificWorker::read_lidar,this);
        std::cout << __FUNCTION__ << " Started lidar reader" << std::endl;

        // Localizer thread is created (only if localizer is enabled)
        if (params.USE_LOCALIZER)
        {
            localizer_th = std::thread(&SpecificWorker::run_localizer, this);
            std::cout << __FUNCTION__ << " Started localizer thread" << std::endl;
        }

        // Initialize MPPI controller with default parameters from header
        // Only override robot-specific values
        MPPIController::Params mppi_params;  // Uses defaults from mppi_controller.h
        mppi_params.robot_radius = params.ROBOT_SEMI_WIDTH;
        mppi_controller.setParams(mppi_params);

        // MPPI thread is created
        mppi_th = std::thread(&SpecificWorker::run_mppi, this);
        std::cout << __FUNCTION__ << " Started MPPI thread" << std::endl;

        // Initialize CPU usage tracking
        getrusage(RUSAGE_SELF, &last_usage);
        last_cpu_time = std::chrono::steady_clock::now();

        // Connect MPPI button
        connect(pushButton_mppi, &QPushButton::toggled, this, &SpecificWorker::slot_mppi_button_toggled);

        // Connect visualization checkboxes
        connect(checkBox_lidar, &QCheckBox::toggled, this, &SpecificWorker::slot_lidar_checkbox_toggled);
        connect(checkBox_particles, &QCheckBox::toggled, this, &SpecificWorker::slot_particles_checkbox_toggled);
        connect(checkBox_trajectories, &QCheckBox::toggled, this, &SpecificWorker::slot_trajectories_checkbox_toggled);
        connect(checkBox_covariance, &QCheckBox::toggled, this, &SpecificWorker::slot_covariance_checkbox_toggled);

        // mouse
        connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, [this](QPointF p)
        {
            qInfo() << "[MOUSE] New global target arrived:" << p;
            std::lock_guard<std::mutex> lock(mutex_current_path);

            // Use current robot position in world coordinates as source
            const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            const auto &[robot, lidar_world, lidar_local] = buffer_sync.read(timestamp);
            if (not robot.has_value()) return;

            const Eigen::Vector2f source = robot.value().translation();
            const Eigen::Vector2f target{p.x(), p.y()};
            std::vector<Eigen::Vector2f> path;

            auto start_time = std::chrono::high_resolution_clock::now();

            // Use Sparse ESDF grid for path planning
            // First check if there's line of sight to target
            bool los_free = grid_esdf.is_line_of_sight_free(source, target, params.ROBOT_SEMI_WIDTH);
            qInfo() << "[PATH] Source:" << source.x() << source.y()
                    << "Target:" << target.x() << target.y()
                    << "LOS free:" << los_free
                    << "Robot width:" << params.ROBOT_SEMI_WIDTH;

            if (los_free)
            {
                // Direct path - line of sight is free
                // Interpolate points along the path for visibility
                const float step = params.TILE_SIZE;  // One point per tile
                const Eigen::Vector2f delta = target - source;
                const float length = delta.norm();
                const Eigen::Vector2f dir = delta.normalized();

                    path.push_back(source);
                    for (float t = step; t < length; t += step)
                        path.push_back(source + dir * t);
                    path.push_back(target);

                    qInfo() << "Line of sight path found (ESDF), interpolated to" << path.size() << "points";
                }
                else
                {
                    // Use A* with ESDF cost, passing robot radius and safety factor
                    path = grid_esdf.compute_path(source, target, params.ROBOT_SEMI_WIDTH, params.SAFETY_FACTOR);
                    qInfo() << "A* path computed (ESDF), path size:" << path.size();
                }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            float elapsed_sec = static_cast<float>(elapsed_ms) / 1000.f;

            if (!path.empty())
            {
                // Calculate path length (sum of segment distances)
                float path_length = 0.f;
                for (size_t i = 1; i < path.size(); ++i)
                    path_length += (path[i] - path[i-1]).norm();

                // Calculate path cost (sum of costs along the path)
                float path_cost = 0.f;
                for (const auto &pt : path)
                    path_cost += grid_esdf.get_cost(pt);

                // Update UI displays
                lcdNumber_length->display(static_cast<double>(path_length / 1000.f));  // Show in meters
                lcdNumber_cost->display(static_cast<int>(path_cost));
                lcdNumber_elapsed->display(static_cast<double>(elapsed_sec));  // Show in seconds

                qInfo() << "Path found with" << path.size() << "waypoints, length:"
                        << path_length/1000.f << "m, cost:" << path_cost << ", time:" << elapsed_sec << "s";
                draw_path(path, &viewer->scene);

                // Draw target marker (solid circle)
                static QGraphicsEllipseItem* target_marker = nullptr;
                if (target_marker)
                {
                    viewer->scene.removeItem(target_marker);
                    delete target_marker;
                }
                const float marker_size = 200.f;  // mm
                target_marker = viewer->scene.addEllipse(
                    -marker_size/2, -marker_size/2, marker_size, marker_size,
                    QPen(QColor("Red"), 30),
                    QBrush(QColor(255, 0, 0, 150)));  // Semi-transparent red fill
                target_marker->setPos(target.x(), target.y());
                target_marker->setZValue(20);  // On top of everything

                // Update MPPI navigation target and path
                current_path = path;
                current_target = target;
                nav_state = NavigationState::NAVIGATING;
                mppi_controller.reset();  // Reset for new path
            }
            else
            {
                // Clear UI on no path
                lcdNumber_length->display(0);
                lcdNumber_cost->display(0);
                lcdNumber_elapsed->display(static_cast<double>(elapsed_sec));  // Show in seconds
                qInfo() << "No path found!";
                nav_state = NavigationState::BLOCKED;
            }
        });
        connect(viewer, &AbstractGraphicViewer::right_click, [this](QPointF p)
        {
            qInfo() <<  "RIGHT CLICK. Cancelling target";
            draw_path({}, &viewer->scene, true);
            cancel_from_mouse = true;
        });
        // Shift+Left click to reposition robot (for initial placement with external maps)
        // connect(viewer, &AbstractGraphicViewer::robot_moved, [this](QPointF p)
        // {
        //     qInfo() << "[ROBOT] Manual reposition to:" << p;
        //     // Update robot visual position
        //     viewer->robot_poly()->setPos(p.x(), p.y());
        //     // Update internal robot pose (for path planning source)
        //     robot_pose.translation() = Eigen::Vector2f(p.x(), p.y());
        // });
        if(not params.DISPLAY)
            hide();

        // test robot is alive
        const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        int startup_check_counter = 0;
        std::optional<Eigen::Affine2f> robot;
        std::optional<std::vector<Eigen::Vector2f>> lidar_world;
        std::optional<std::vector<Eigen::Vector2f>> lidar_local;
        // do this 5 times and then exit is no data.
        do
        {
            qWarning() << "No data from buffer_sync: robot has value? Retrying... (" << startup_check_counter << ")";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            const auto &[r, lw, ll] = buffer_sync.read(timestamp);
            robot = r; lidar_world = lw; lidar_local = ll;
        }while (++startup_check_counter < 5 and (not robot.has_value() or not lidar_world.has_value()));
        if (not robot.has_value() or not lidar_world.has_value())
        { qWarning() << "No data from buffer_sync: robot has value?. Exiting program"; std::terminate(); };
        const auto &robot_pos = robot.value();  // Ground truth pose from simulator
        viewer->centerOn(robot_pos.translation().x(), robot_pos.translation().y());
    }
}
void SpecificWorker::compute()
{
    const auto timestamp = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const auto &[robot, lidar_world, lidar_local] = buffer_sync.read(timestamp);
    if (not robot.has_value() or not lidar_world.has_value() or not lidar_local.has_value())
        { qWarning() << "No data from buffer_sync: robot has value?"; return; };
    const auto &robot_pos = robot.value();  // Ground truth pose from simulator
    const auto &points_world = lidar_world.value();
    // Note: lidar_local is used by the localizer thread via buffer_sync

    /// Update grid with world coordinates (only if no external map loaded)
    if (not external_map_loaded)
    {
        mutex_path.lock();
        // VoxBlox-style ESDF: only stores obstacles
        grid_esdf.update(points_world, robot_pos.translation(), params.MAX_LIDAR_RANGE, timestamp);
        grid_esdf.update_visualization(true);  // Only update visualization when there are changes
        mutex_path.unlock();
    }

    // ============ UPDATE ROBOT POSE (from Localizer thread or Ground Truth) ============
    if (params.USE_LOCALIZER and external_map_loaded)
        // Try to read from localizer thread's buffer
        if (const auto& [estimated] = buffer_estimated_pose.read(timestamp); estimated.has_value())
            estimated_robot_pose = estimated.value();
        else
            estimated_robot_pose = robot_pos;  // Fallback to ground truth
    else
        estimated_robot_pose = robot_pos;  // Use ground truth directly

    // ============ Debug: draw lidar points (controlled by UI checkbox)
    if (show_lidar_points.load())
        draw_lidar_points(points_world);

    // ============ Draw localizer particles (must be in main Qt thread)
    if (show_particles.load() && params.USE_LOCALIZER)
        localizer.drawParticles();

    // ============ Draw covariance ellipse (must be in main Qt thread)
    if (show_covariance.load() && params.USE_LOCALIZER && localizer_initialized.load())
        draw_covariance_ellipse();

    // ============ Update robot visualization in the viewer using estimated pose
    const float display_x = estimated_robot_pose.translation().x();
    const float display_y = estimated_robot_pose.translation().y();
    const float display_angle = std::atan2(estimated_robot_pose.linear()(1,0), estimated_robot_pose.linear()(0,0));
    viewer->robot_poly()->setPos(display_x, display_y);
    viewer->robot_poly()->setRotation(qRadiansToDegrees(display_angle));

    // ============ MPPI NAVIGATION (read from MPPI thread) ============
    if (mppi_enabled.load())
    {
        const auto& [mppi_out] = buffer_mppi_output.read(timestamp);
        if (mppi_out.has_value() && mppi_out->valid)
            try
            {
                // MPPI returns: vx = sideways (X+ right), vy = forward (Y+), omega = rotation
                // setSpeedBase expects same convention: (sideways, forward, rotation)
                omnirobot_proxy->setSpeedBase(mppi_out->vx, mppi_out->vy, mppi_out->omega);
            }
            catch (const Ice::Exception &e)
            {
                static int error_count = 0;
                if (++error_count % 100 == 1)
                    qWarning() << "[MPPI] Failed to send speed to robot:" << e.what();
            }

        // Update MPPI trajectory visualization (from the thread's output) if enabled
        if (show_trajectories.load())
        {
            std::lock_guard<std::mutex> lock(mutex_mppi_trajectory);
            if (!last_optimal_trajectory.empty())
                draw_mppi_trajectory(last_optimal_trajectory);
        }
    }

    // ============ UPDATE UI ============
    this->hz = fps.print("FPS:", 3000);
    this->lcdNumber_hz->display(this->hz);
    this->lcdNumber_loc_hz->display(localizer_hz.load());
    this->lcdNumber_mppi_hz->display(mppi_hz.load());

    // Update CPU usage with exponential moving average to reduce flickering
    const float current_cpu = get_cpu_usage();
    cpu_usage_avg = CPU_AVG_ALPHA * current_cpu + (1.0f - CPU_AVG_ALPHA) * cpu_usage_avg;
    this->lcdNumber_cpu->display(static_cast<int>(cpu_usage_avg));
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::draw_lidar_points(const std::vector<Eigen::Vector2f> &points_world)
{
    static std::vector<QGraphicsEllipseItem*> lidar_points;
    static const QPen pen(QColor("DarkBlue"));
    static const QBrush brush(QColor("DarkBlue"));
    static constexpr float s = 40.f;

    const int stride = std::max(1, static_cast<int>(points_world.size() / params.MAX_LIDAR_DRAW_POINTS));
    const size_t num_points_to_draw = (points_world.size() + stride - 1) / stride;

    // Reuse existing items when possible, only create/delete when needed
    // Remove excess items if we have too many
    while (lidar_points.size() > num_points_to_draw)
    {
        auto *p = lidar_points.back();
        viewer->scene.removeItem(p);
        delete p;
        lidar_points.pop_back();
    }

    // Update existing items and create new ones as needed
    size_t idx = 0;
    for (size_t i = 0; i < points_world.size() && idx < num_points_to_draw; i += stride, ++idx)
    {
        const auto &p = points_world[i];
        if (idx < lidar_points.size())
        {
            // Reuse existing item - just update position
            lidar_points[idx]->setPos(p.x(), p.y());
        }
        else
        {
            // Create new item
            auto *item = viewer->scene.addEllipse(-s / 2.f, -s / 2.f, s, s, pen, brush);
            item->setPos(p.x(), p.y());
            item->setZValue(5);
            lidar_points.push_back(item);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::draw_covariance_ellipse()
{
    // Get covariance from localizer: [xx, xy, xθ, yy, yθ, θθ]
    auto cov = localizer.getCovarianceVector();
    if (cov.size() < 6) return;

    const float var_xx = cov[0];  // σ²_x
    const float var_xy = cov[1];  // σ_xy
    const float var_yy = cov[3];  // σ²_y

    // Compute eigenvalues and eigenvectors of 2x2 covariance matrix
    // | var_xx  var_xy |
    // | var_xy  var_yy |

    // Eigenvalues: λ = (trace ± sqrt(trace² - 4*det)) / 2
    const float trace = var_xx + var_yy;
    const float det = var_xx * var_yy - var_xy * var_xy;
    const float discriminant = trace * trace - 4.0f * det;

    if (discriminant < 0) return;  // Should not happen for valid covariance

    const float sqrt_disc = std::sqrt(discriminant);
    const float lambda1 = (trace + sqrt_disc) / 2.0f;  // Larger eigenvalue
    const float lambda2 = (trace - sqrt_disc) / 2.0f;  // Smaller eigenvalue

    // Standard deviations (sqrt of eigenvalues) scaled by 6 for visibility
    // Also apply minimum size so ellipse is always visible
    const float scale = 6.0f;  // 6-sigma for better visibility
    const float min_size = 200.0f;  // Minimum size in mm for visibility
    float sigma1 = scale * std::sqrt(std::max(0.0f, lambda1));
    float sigma2 = scale * std::sqrt(std::max(0.0f, lambda2));

    // Ensure minimum visible size
    sigma1 = std::max(sigma1, min_size);
    sigma2 = std::max(sigma2, min_size);

    // Angle of the principal axis (eigenvector of larger eigenvalue)
    float ellipse_angle = 0.0f;
    if (std::abs(var_xy) > 1e-6f)
        ellipse_angle = 0.5f * std::atan2(2.0f * var_xy, var_xx - var_yy);
    else if (var_yy > var_xx)
        ellipse_angle = M_PI_2;

    // Get robot pose for positioning the ellipse
    const float robot_x = estimated_robot_pose.translation().x();
    const float robot_y = estimated_robot_pose.translation().y();

    // Create or update ellipse
    if (!covariance_ellipse)
    {
        covariance_ellipse = viewer->scene.addEllipse(
            -sigma1, -sigma2, 2.0f * sigma1, 2.0f * sigma2,
            QPen(QColor(255, 0, 255, 255), 5),  // Magenta border, thicker
            QBrush(QColor(255, 0, 255, 80))     // Semi-transparent fill
        );
        covariance_ellipse->setZValue(100);  // On top of everything
    }
    else
    {
        covariance_ellipse->setRect(-sigma1, -sigma2, 2.0f * sigma1, 2.0f * sigma2);
    }

    // Position and rotate ellipse
    covariance_ellipse->setPos(robot_x, robot_y);
    covariance_ellipse->setRotation(qRadiansToDegrees(ellipse_angle));
}

///////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine2f SpecificWorker::get_robot_pose()
{
    RoboCompWebots2Robocomp::ObjectPose pose;
    Eigen::Affine2f robot_pose;
    try
    {
        pose = webots2robocomp_proxy->getObjectPose("shadow");
         robot_pose.translation() = Eigen::Vector2f(-pose.position.y, pose.position.x);
        const auto yaw = yawFromQuaternion(pose.orientation);
        robot_pose.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();
    }
    catch (const Ice::Exception &e){ std::cout<<e.what()<<std::endl; return {};}
    return robot_pose;
}

float SpecificWorker::yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat)
{
    double w = quat.w;
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;
    const auto norm = std::sqrt(w*w + x*x + y*y + z*z);
    w /= norm; x /= norm; y /= norm; z /= norm;
    return static_cast<float>(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
}

void SpecificWorker::read_lidar()
{
    auto wait_period = std::chrono::milliseconds (getPeriod("Compute"));
    while(!stop_lidar_thread)
    {
        const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        try
        {
            // Get robot pose
            const auto &[position, orientation] = webots2robocomp_proxy->getObjectPose("shadow");
            Eigen::Affine2f eig_pose;
            eig_pose.translation() = Eigen::Vector2f(-position.y, position.x);
            eig_pose.linear() = Eigen::Rotation2Df(yawFromQuaternion(orientation)).toRotationMatrix();

            // Get LiDAR data
            auto data = lidar3d1_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_LOW,
                                                                   params.MAX_LIDAR_LOW_RANGE,
                                                                   params.LIDAR_LOW_DECIMATION_FACTOR);

            // Store local points (original LiDAR frame) and transform to world frame
            std::vector<Eigen::Vector2f> points_world;
            std::vector<Eigen::Vector2f> points_local;
            points_world.reserve(data.points.size());
            points_local.reserve(data.points.size());
            for (const auto &p : data.points)
                if (std::hypot(p.y, p.x) > 100) // TODO: move to Params
                {
                    points_local.emplace_back(p.x, p.y);
                    points_world.emplace_back(eig_pose * Eigen::Vector2f{p.x, p.y});
                }

            // Put all in sync buffer with same timestamp
            buffer_sync.put<0>(std::move(eig_pose), timestamp);
            buffer_sync.put<1>(std::move(points_world), timestamp);
            buffer_sync.put<2>(std::move(points_local), timestamp);

            // Adjust period with hysteresis
            if (wait_period > std::chrono::milliseconds((long) data.period + 2)) wait_period--;
            else if (wait_period < std::chrono::milliseconds((long) data.period - 2)) wait_period++;
        }
        catch (const Ice::Exception &e)
        { std::cout << "Error reading from Lidar3D or robot pose: " << e.what() << std::endl; }
        std::this_thread::sleep_for(wait_period);
    }
} // Thread to read the lidar

/////////////////////////////////////////////////////////////////////////////////////////////////
/// Localizer Thread - runs AMCL independently at its own rate
/////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::run_localizer()
{
    const auto target_period = std::chrono::milliseconds(params.LOCALIZER_PERIOD_MS);
    auto last_fps_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    while (!stop_localizer_thread)
    {
        const auto loop_start = std::chrono::steady_clock::now();

        // Wait until map is ready
        if (!map_ready_for_localization.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        const auto timestamp = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        // Read current data from sync buffer
        const auto& [robot, lidar_world, lidar_local] = buffer_sync.read(timestamp);
        if (!robot.has_value() || !lidar_local.has_value())
        {
            std::this_thread::sleep_for(target_period);
            continue;
        }

        const auto& robot_pos = robot.value();
        const auto& points_local = lidar_local.value();

        // Get current ground truth pose
        const float gt_x = robot_pos.translation().x();
        const float gt_y = robot_pos.translation().y();
        const auto robot_rot = robot_pos.linear();
        const float gt_theta = std::atan2(robot_rot(1, 0), robot_rot(0, 0));
        Localizer::Pose2D current_gt_pose{gt_x, gt_y, gt_theta};

        // Initialize localizer on first valid pose
        if (!localizer_initialized.load())
        {
            localizer.resetGaussian(current_gt_pose, 500.f, 0.2f);
            last_ground_truth_pose = current_gt_pose;
            localizer_initialized.store(true);
            qInfo() << "[Localizer Thread] Initialized at:" << gt_x << gt_y << "theta:" << qRadiansToDegrees(gt_theta);
            continue;
        }

        // Compute odometry from ground truth poses
        auto odom = Localizer::computeOdometryDelta(last_ground_truth_pose, current_gt_pose);

        // Update localizer with odometry and LiDAR points
        auto estimated_pose_opt = localizer.update(odom, points_local);

        // Update last pose for next iteration
        last_ground_truth_pose = current_gt_pose;

        // Get the estimated pose (from update result or mean pose)
        Localizer::Pose2D estimated_pose = estimated_pose_opt.value_or(localizer.getMeanPose());

        // Sanity check and publish to buffer
        float error = std::hypot(estimated_pose.x - gt_x, estimated_pose.y - gt_y);
        if (error < 5000.f && localizer.getEffectiveSampleSize() > 10.0)
        {
            // Build Affine2f from localized pose
            Eigen::Affine2f result;
            result.translation() = Eigen::Vector2f(estimated_pose.x, estimated_pose.y);
            result.linear() = Eigen::Rotation2Df(estimated_pose.theta).toRotationMatrix();

            // Write to double buffer for main thread
            buffer_estimated_pose.put<0>(std::move(result), timestamp);
        }

        // Update FPS counter
        frame_count++;
        const auto now = std::chrono::steady_clock::now();
        const auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
        if (fps_elapsed >= 1000)
        {
            localizer_hz.store(frame_count);
            frame_count = 0;
            last_fps_time = now;
        }

        // Periodic debug
        static int debug_counter = 0;
        if (++debug_counter % 100 == 0)
        {
            auto est = localizer.getMeanPose();
            float pos_error = std::hypot(est.x - gt_x, est.y - gt_y);
            float angle_error = std::abs(Localizer::Pose2D::normalizeAngle(est.theta - gt_theta));
            qInfo() << "[Localizer Thread] GT error: pos=" << static_cast<int>(pos_error)
                    << "mm, angle=" << qRadiansToDegrees(angle_error) << "deg"
                    << "| ESS:" << static_cast<int>(localizer.getEffectiveSampleSize());
        }

        // Adaptive sleep: only sleep for remaining time to maintain target frequency
        const auto loop_end = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        if (elapsed < target_period)
            std::this_thread::sleep_for(target_period - elapsed);
        // else: loop took longer than target, don't sleep (run as fast as possible)
    }
    qInfo() << "[Localizer Thread] Stopped";
} // Thread to run localizer

void SpecificWorker::draw_path(const std::vector<Eigen::Vector2f> &path, QGraphicsScene *scene, bool erase_only)
{
    static std::vector<QGraphicsEllipseItem*> points;
    static const QColor color("blue");
    static const QPen pen(color);
    static const QBrush brush(color);
    static constexpr float s = 50;  // Smaller dots for thinner path

    for(auto p : points)
    {
        scene->removeItem(p);
        delete p;
    }
    points.clear();

    if(erase_only) return;

    points.reserve(path.size());
    for(const auto &p: path)
    {
        auto ptr = scene->addEllipse(-s/2, -s/2, s, s, pen, brush);
        ptr->setPos(p.x(), p.y());
        ptr->setZValue(10);
        points.push_back(ptr);
    }
}

void SpecificWorker::draw_paths(const std::vector<std::vector<Eigen::Vector2f>> &paths, QGraphicsScene *scene, bool erase_only)
{
    static std::vector<QGraphicsEllipseItem*> points;
    static QColor colors[] = {QColor("cyan"), QColor("blue"), QColor("red"), QColor("orange"), QColor("magenta"), QColor("cyan")};
    for(auto p : points)
    {
        scene->removeItem(p);
        delete p;  // Fix memory leak
    }
    points.clear();

    if(erase_only) return;

    float s = 80;
    for(const auto &[i, path]: paths | iter::enumerate)
    {
        // pick a consecutive color
        auto color = colors[i];
        for(const auto &p: path)
        {
            auto ptr = scene->addEllipse(-s/2.f, -s/2.f, s, s, QPen(color), QBrush(color));
            ptr->setPos(QPointF(p.x(), p.y()));
            ptr->setZValue(10);  // Draw on top of grid, obstacles, and robot
            points.push_back(ptr);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// MPPI Thread - runs controller independently at its own rate
//////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::run_mppi()
{
    const auto target_period = std::chrono::milliseconds(params.MPPI_PERIOD_MS);
    auto last_fps_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    while (!stop_mppi_thread)
    {
        const auto loop_start = std::chrono::steady_clock::now();

        // Check preconditions
        if (!mppi_enabled.load() || nav_state.load() != NavigationState::NAVIGATING)
        {
            std::this_thread::sleep_for(target_period);
            continue;
        }

        const auto timestamp = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        // Read current robot pose from estimated pose buffer
        Eigen::Affine2f robot_pose;
        {
            const auto& [estimated] = buffer_estimated_pose.read(timestamp);
            if (estimated.has_value())
                robot_pose = estimated.value();
            else
            {
                // Try to get from sync buffer as fallback
                const auto& [robot, lw, ll] = buffer_sync.read(timestamp);
                if (robot.has_value())
                    robot_pose = robot.value();
                else
                {
                    std::this_thread::sleep_for(target_period);
                    continue;
                }
            }
        }

        // Read LiDAR points for obstacle avoidance
        std::vector<Eigen::Vector2f> lidar_points;
        {
            const auto& [robot, lw, ll] = buffer_sync.read(timestamp);
            if (lw.has_value())
                lidar_points = lw.value();
        }

        // Get current path (protected by mutex)
        std::vector<Eigen::Vector2f> path_copy;
        {
            std::lock_guard<std::mutex> lock(mutex_current_path);
            path_copy = current_path;
        }

        if (path_copy.empty())
        {
            std::this_thread::sleep_for(target_period);
            continue;
        }

        // Convert robot pose to MPPI state
        MPPIController::State current_state;
        current_state.x = robot_pose.translation().x();
        current_state.y = robot_pose.translation().y();
        current_state.theta = std::atan2(robot_pose.linear()(1,0), robot_pose.linear()(0,0));

        // Check if goal reached
        if (mppi_controller.goalReached(current_state, path_copy.back()))
        {
            nav_state.store(NavigationState::GOAL_REACHED);
            qInfo() << "[MPPI] Goal reached!";

            // Send stop command
            MPPIOutput out{0.f, 0.f, 0.f, true};
            buffer_mppi_output.put<0>(std::move(out), timestamp);
            std::this_thread::sleep_for(target_period);
            continue;
        }

        // Get pose covariance from localizer (if available)
        std::vector<float> pose_cov;
        if (params.USE_LOCALIZER && localizer_initialized.load())
            pose_cov = localizer.getCovarianceVector();

        // Compute control command using MPPI with covariance-aware ESDF obstacle costs
        // LiDAR points are still used for hard collision detection (safety layer)
        auto cmd = mppi_controller.compute(current_state, path_copy, lidar_points, &grid_esdf, pose_cov);

        // Log command periodically
        static int log_counter = 0;
        if (++log_counter % 50 == 0)
        {
            qDebug() << "[MPPI Thread] Command: vx=" << cmd.vx << "mm/s, vy=" << cmd.vy
                     << "mm/s, omega=" << cmd.omega << "rad/s";
        }

        // Write to output buffer
        MPPIOutput out{cmd.vx, cmd.vy, cmd.omega, true};
        buffer_mppi_output.put<0>(std::move(out), timestamp);

        // Copy trajectory for visualization (protected by mutex)
        {
            std::lock_guard<std::mutex> lock(mutex_mppi_trajectory);
            last_optimal_trajectory = mppi_controller.getOptimalTrajectory();
        }

        // Update FPS counter
        frame_count++;
        const auto now = std::chrono::steady_clock::now();
        const auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
        if (fps_elapsed >= 1000)
        {
            mppi_hz.store(frame_count);
            frame_count = 0;
            last_fps_time = now;
        }

        // Adaptive sleep: only sleep for remaining time to maintain target frequency
        const auto loop_end = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        if (elapsed < target_period)
            std::this_thread::sleep_for(target_period - elapsed);
        // else: loop took longer than target, don't sleep (run as fast as possible)
    }
    qInfo() << "[MPPI Thread] Stopped";
}

void SpecificWorker::draw_mppi_trajectory(const std::vector<MPPIController::State>& trajectory)
{
    // Clear previous trajectory items
    for (auto* item : mppi_trajectory_items)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    mppi_trajectory_items.clear();

    // Colors for the N best trajectories (from worst to best among selected)
    static const std::vector<QColor> trajectory_colors = {
        QColor(100, 100, 255, 150),   // Light blue (transparent)
        QColor(100, 150, 255, 150),
        QColor(100, 200, 255, 160),
        QColor(50, 220, 200, 170),
        QColor(50, 255, 150, 180),
        QColor(100, 255, 100, 190),
        QColor(150, 255, 50, 200),
        QColor(200, 255, 50, 210),
        QColor(255, 200, 50, 220),
        QColor(255, 150, 50, 230)     // Orange (best among N)
    };

    // Draw N best trajectories first (so optimal is on top)
    const auto& best_trajectories = mppi_controller.getBestTrajectories();
    for (size_t traj_idx = 0; traj_idx < best_trajectories.size(); ++traj_idx)
    {
        const auto& traj = best_trajectories[traj_idx];
        if (traj.size() < 2) continue;

        // Select color based on index (better trajectories get warmer colors)
        size_t color_idx = std::min(traj_idx, trajectory_colors.size() - 1);
        QColor color = trajectory_colors[color_idx];
        QPen pen(color, 15);  // Thinner than optimal

        for (size_t i = 1; i < traj.size(); ++i)
        {
            auto* line = viewer->scene.addLine(
                traj[i-1].x, traj[i-1].y,
                traj[i].x, traj[i].y,
                pen);
            line->setZValue(10 + static_cast<int>(traj_idx));  // Stack by quality
            mppi_trajectory_items.push_back(line);
        }
    }

    // Draw optimal trajectory on top in magenta, thick
    if (trajectory.size() >= 2)
    {
        QPen pen(QColor("Magenta"), 40);
        for (size_t i = 1; i < trajectory.size(); ++i)
        {
            auto* line = viewer->scene.addLine(
                trajectory[i-1].x, trajectory[i-1].y,
                trajectory[i].x, trajectory[i].y,
                pen);
            line->setZValue(20);  // On top of all others
            mppi_trajectory_items.push_back(line);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Auxiliary methods
/////////////////////////////////////////////////////////////////////////////////////////////////

float SpecificWorker::get_cpu_usage()
{
    struct rusage current_usage;
    getrusage(RUSAGE_SELF, &current_usage);

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - last_cpu_time).count();

    if (elapsed <= 0) return 0.f;

    // Calculate CPU time used (user + system)
    long user_diff = (current_usage.ru_utime.tv_sec - last_usage.ru_utime.tv_sec) * 1000000 +
                     (current_usage.ru_utime.tv_usec - last_usage.ru_utime.tv_usec);
    long sys_diff = (current_usage.ru_stime.tv_sec - last_usage.ru_stime.tv_sec) * 1000000 +
                    (current_usage.ru_stime.tv_usec - last_usage.ru_stime.tv_usec);

    float cpu_percent = 100.f * static_cast<float>(user_diff + sys_diff) / static_cast<float>(elapsed);

    // Update for next call
    last_usage = current_usage;
    last_cpu_time = current_time;

    return cpu_percent;
}

void SpecificWorker::slot_mppi_button_toggled(bool checked)
{
    mppi_enabled.store(checked);

    if (checked)
    {
        pushButton_mppi->setText("MPPI ON");
        pushButton_mppi->setStyleSheet("background-color: green; color: white;");
        qInfo() << "[MPPI] Controller ENABLED";
    }
    else
    {
        pushButton_mppi->setText("MPPI OFF");
        pushButton_mppi->setStyleSheet("");
        // Stop the robot when disabling MPPI
        try
        {
            omnirobot_proxy->setSpeedBase(0.f, 0.f, 0.f);
        }
        catch (const Ice::Exception &e)
        {
            qWarning() << "[MPPI] Failed to stop robot:" << e.what();
        }
        nav_state = NavigationState::IDLE;
        qInfo() << "[MPPI] Controller DISABLED";
    }
}

void SpecificWorker::slot_lidar_checkbox_toggled(bool checked)
{
    show_lidar_points.store(checked);
    if (!checked)
    {
        // Clear existing lidar points from scene
        draw_lidar_points({});
    }
}

void SpecificWorker::slot_particles_checkbox_toggled(bool checked)
{
    show_particles.store(checked);
    localizer.params().draw_particles = checked;

    if (!checked)
        localizer.clearParticleVisualization();
}

void SpecificWorker::slot_trajectories_checkbox_toggled(bool checked)
{
    show_trajectories.store(checked);
    if (!checked)
    {
        // Clear existing trajectory items
        for (auto* item : mppi_trajectory_items)
        {
            viewer->scene.removeItem(item);
            delete item;
        }
        mppi_trajectory_items.clear();
    }
}

void SpecificWorker::slot_covariance_checkbox_toggled(bool checked)
{
    show_covariance.store(checked);
    if (!checked && covariance_ellipse)
    {
        viewer->scene.removeItem(covariance_ellipse);
        delete covariance_ellipse;
        covariance_ellipse = nullptr;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
/// Gridder.idsl interface implementation
/// ///////////////////////////////////////////////////////////////////////////////////////////////

RoboCompGridder::Result SpecificWorker::Gridder_getPaths(RoboCompGridder::TPoint source,
                                                         RoboCompGridder::TPoint target,
                                                         int max_paths,
                                                         bool tryClosestFreePoint,
                                                         bool targetIsHuman,
                                                         float safetyFactor)
{
    //TODO: improve this method to try to find a path even if the target is not free by using the closest free point
    //TODO: if target is human, set safe area around as free
    RoboCompGridder::Result result;
    std::vector<std::vector<Eigen::Vector2f>> paths;

    auto begin = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    qInfo() << __FUNCTION__ << " New plan request: source [" << source.x << source.y << "], target [" << target.x << target.y << "]"
            << " max_paths: " << max_paths << " safety: " << safetyFactor;

    std::lock_guard<std::mutex> lock(mutex_path);
    std::string msg;
    bool success = true;

    // Use ESDF grid for path planning
    const Eigen::Vector2f src{source.x, source.y};
    const Eigen::Vector2f tgt{target.x, target.y};

    // Check if source or target are obstacles
    if (grid_esdf.is_obstacle(grid_esdf.point_to_key(src)))
    {
        success = false;
        msg = "Source is blocked";
    }
    else if (grid_esdf.is_obstacle(grid_esdf.point_to_key(tgt)))
    {
        success = false;
        msg = "Target is blocked";
    }
    else
    {
        // Check if line of sight is free
        if (grid_esdf.is_line_of_sight_free(src, tgt, params.ROBOT_SEMI_WIDTH))
        {
            paths.push_back({src, tgt});
            msg = "VLOS path (ESDF)";
        }
        else
        {
            // Use A* with ESDF cost and safety factor from parameter
            auto path = grid_esdf.compute_path(src, tgt, params.ROBOT_SEMI_WIDTH, safetyFactor);
            if (!path.empty())
            {
                paths.push_back(path);
                msg = "A* path (ESDF)";
            }
            else
            {
                msg = "A* path not found (ESDF)";
            }
        }
    }

    result.errorMsg = msg;
    result.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // If not success return result with empty paths, error message and timestamp
    if (not success)
        return result;
    else
    {
        // fill Result with data
        result.paths.resize(paths.size());
        for (const auto &[i, path]: paths | iter::enumerate)
        {
            result.paths[i].resize(path.size());
            for (const auto &[j, point]: path | iter::enumerate)
            {
                result.paths[i][j].x = point.x();
                result.paths[i][j].y = point.y();
            }
        }
        qInfo() << __FUNCTION__ << " " << paths.size() << " paths computed in " <<
                std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count() - begin << " ms" << "Status:" << msg.c_str();
        return result;
    }
}
bool SpecificWorker::Gridder_LineOfSightToTarget(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, float robotRadius)
{
    std::lock_guard<std::mutex> lock(mutex_path);
    return grid_esdf.is_line_of_sight_free(
        Eigen::Vector2f{source.x, source.y},
        Eigen::Vector2f{target.x, target.y},
        robotRadius);
}
RoboCompGridder::TPoint SpecificWorker::Gridder_getClosestFreePoint(RoboCompGridder::TPoint source)
{
    std::lock_guard<std::mutex> lock(mutex_path);

    // In sparse grid, non-obstacle = free
    // If source is not an obstacle, return source itself
    const auto key = grid_esdf.point_to_key(source.x, source.y);
    if (!grid_esdf.is_obstacle(key))
        return source;

    // Otherwise, search neighbors for closest free cell
    const float tile_size = static_cast<float>(grid_esdf.params().tile_size);
    for (int radius = 1; radius <= 10; ++radius)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
                for (int dy = -radius; dy <= radius; ++dy)
                {
                    if (std::abs(dx) != radius && std::abs(dy) != radius) continue;  // Only check perimeter
                    const auto test_key = std::make_pair(
                        static_cast<int>(key.first + dx * tile_size),
                        static_cast<int>(key.second + dy * tile_size));
                    if (!grid_esdf.is_obstacle(test_key))
                    {
                        const auto point = grid_esdf.key_to_point(test_key);
                        return {point.x(), point.y()};
                    }
                }
            }
        }
        return {0, 0};  // No free point found
}
RoboCompGridder::TDimensions SpecificWorker::Gridder_getDimensions()
{
    return {static_cast<float>(params.GRID_MAX_DIM.x()),
            static_cast<float>(params.GRID_MAX_DIM.y()),
            static_cast<float>(params.GRID_MAX_DIM.width()),
            static_cast<float>(params.GRID_MAX_DIM.height())};
}
RoboCompGridder::Map SpecificWorker::Gridder_getMap()
{
    std::lock_guard<std::mutex> lock(mutex_path);
    RoboCompGridder::Map result;

    // Serialize sparse ESDF grid
    auto serialized = grid_esdf.serialize_map();
    result.tileSize = serialized.tile_size;
    result.cells.reserve(serialized.num_cells);

    for (const auto &cell : serialized.cells)
    {
        RoboCompGridder::TCell tc;
        tc.x = cell.x;
        tc.y = cell.y;
        tc.cost = cell.cost;
        result.cells.push_back(tc);
    }

    qInfo() << __FUNCTION__ << "Returning ESDF map with" << result.cells.size() << "cells";
    return result;
}
bool SpecificWorker::Gridder_setGridDimensions(RoboCompGridder::TDimensions dimensions)
{
    qInfo() << __FUNCTION__ << " Setting grid dimensions to [" << dimensions.left << dimensions.top << dimensions.width << dimensions.height << "]";
    params.GRID_MAX_DIM = QRectF(dimensions.left, dimensions.top, dimensions.width, dimensions.height);
    //TODO: update grid, clear and reinitialize
    return true;
}
RoboCompGridder::Result SpecificWorker::Gridder_setLocationAndGetPath(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, RoboCompGridder::TPointVector freePoints, RoboCompGridder::TPointVector obstaclePoints)
{
    // Note: Dynamic grid modification not fully supported in sparse ESDF mode
    // This function simply computes paths using the current static map
    qWarning() << "[Gridder] setLocationAndGetPath: Dynamic grid modification not supported in ESDF mode. Using static map.";
    return Gridder_getPaths(source, target, 1, true, true, params.SAFETY_FACTOR);
}
bool SpecificWorker::Gridder_IsPathBlocked(RoboCompGridder::TPath path)
{
    std::lock_guard<std::mutex> lock(mutex_path);

    // Check each segment of the path for obstacles
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Eigen::Vector2f from{path[i].x, path[i].y};
        const Eigen::Vector2f to{path[i + 1].x, path[i + 1].y};
        if (!grid_esdf.is_line_of_sight_free(from, to, params.ROBOT_SEMI_WIDTH))
            return true;  // Path is blocked
    }
    return false;  // Path is free
}
bool SpecificWorker::Gridder_loadMRPTMap(const std::string &filepath)
{
    qInfo() << "[MRPT Loader] Loading map from:" << filepath.c_str();

    // Load MRPT map file
    auto load_result = MRPTMapLoader::load_gridmap(filepath);

    if (!load_result.success)
    {
        qWarning() << "[MRPT Loader] Failed to load map:" << load_result.error_msg.c_str();
        return false;
    }

    qInfo() << MRPTMapLoader::create_summary_report(load_result).c_str();
    qInfo() << "[MRPT Loader] Cells to add:" << load_result.cells.size()
            << "(should be ~" << load_result.num_occupied << "occupied cells)";

    std::lock_guard<std::mutex> lock(mutex_path);

    qInfo() << "[MRPT Loader] Loading into SPARSE_ESDF grid";

    // Reinitialize grid with MRPT map resolution if different
    int mrpt_resolution = static_cast<int>(load_result.metadata.resolution);
        if (mrpt_resolution > 0 && mrpt_resolution != static_cast<int>(params.TILE_SIZE))
        {
            qInfo() << "[MRPT Loader] Adjusting grid tile size from" << params.TILE_SIZE
                    << "to" << mrpt_resolution << "mm (MRPT resolution)";
            params.TILE_SIZE = static_cast<float>(mrpt_resolution);
            grid_esdf.initialize(mrpt_resolution, &viewer->scene);
        }
        else
        {
            // Clear previous data
            grid_esdf.clear();
        }

        // Convert MRPT cells to sparse grid format
        // Apply rotation and offset to align MRPT map with Webots world coordinates
        const float offset_x = params.MRPT_MAP_OFFSET_X;
        const float offset_y = params.MRPT_MAP_OFFSET_Y;
        const float rotation = params.MRPT_MAP_ROTATION;
        const float cos_r = std::cos(rotation);
        const float sin_r = std::sin(rotation);
        const bool mirror_x = params.MRPT_MAP_MIRROR_X;

        if (offset_x != 0.f || offset_y != 0.f || rotation != 0.f || mirror_x)
            qInfo() << "[MRPT Loader] Applying transform: rotation=" << rotation << "rad, offset=(" << offset_x << "," << offset_y << ")mm, mirror_x=" << mirror_x;

        // Track unique keys to detect collisions (multiple MRPT cells mapping to same grid cell)
        std::unordered_set<GridESDF::Key, boost::hash<GridESDF::Key>> unique_keys;
        size_t cells_processed = 0;
        size_t cells_added = 0;
        size_t cells_collided = 0;

        for (const auto &mrpt_cell : load_result.cells)
        {
            // All cells from loader already have occupancy > 0.78 (filtered by cell_value > 200)
            cells_processed++;

            // Apply mirror if needed (negate X before rotation)
            float orig_x = static_cast<float>(mrpt_cell.x);
            const float orig_y = static_cast<float>(mrpt_cell.y);
            if (mirror_x)
                orig_x = -orig_x;

            // First rotate around origin, then apply offset
            const float cell_x = orig_x * cos_r - orig_y * sin_r + offset_x;
            const float cell_y = orig_x * sin_r + orig_y * cos_r + offset_y;
            const auto key = grid_esdf.point_to_key(Eigen::Vector2f(cell_x, cell_y));

            // Check if this key already exists (collision detection)
            if (unique_keys.find(key) == unique_keys.end())
            {
                unique_keys.insert(key);
                grid_esdf.add_confirmed_obstacle(key);
                cells_added++;
            }
            else
            {
                cells_collided++;  // Multiple MRPT cells map to same grid cell
            }
        }

        // Mark dirty once after loading all obstacles, then update visualization
        grid_esdf.mark_visualization_dirty();
        grid_esdf.update_visualization(true);

        // Report statistics
        qInfo() << "[MRPT Loader] SPARSE_ESDF Statistics:";
        qInfo() << "  - MRPT cells processed:" << cells_processed;
        qInfo() << "  - Grid cells added:" << cells_added;
        qInfo() << "  - Collisions (MRPT->same grid cell):" << cells_collided;
        qInfo() << "  - Final grid obstacles:" << grid_esdf.num_obstacles();
        if (cells_collided > 0)
        {
            float collision_pct = 100.f * cells_collided / cells_processed;
            qWarning() << "[MRPT Loader] WARNING:" << collision_pct << "% cells lost due to resolution mismatch!";
        }

    return true;
}
std::string SpecificWorker::Gridder_loadAndInitializeMap(const std::string &filepath)
{
    qInfo() << "[MRPT Loader] Loading and initializing map from:" << filepath.c_str();

    auto load_result = MRPTMapLoader::load_gridmap(filepath);

    if (!load_result.success)
    {
        std::string msg = "Error loading map: " + load_result.error_msg;
        qWarning() << msg.c_str();
        return msg;
    }

    // Calculate bounding box from loaded cells
    if (load_result.cells.empty())
        return "Error: No cells loaded from map";

    int32_t min_x = std::numeric_limits<int32_t>::max();
    int32_t max_x = std::numeric_limits<int32_t>::lowest();
    int32_t min_y = std::numeric_limits<int32_t>::max();
    int32_t max_y = std::numeric_limits<int32_t>::lowest();

    for (const auto &cell : load_result.cells)
    {
        min_x = std::min(min_x, cell.x);
        max_x = std::max(max_x, cell.x);
        min_y = std::min(min_y, cell.y);
        max_y = std::max(max_y, cell.y);
    }

    // Add margin around the bounding box
    const int32_t margin = static_cast<int32_t>(load_result.metadata.resolution * 5);  // 5 cells margin
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;

    const int32_t width = max_x - min_x;
    const int32_t height = max_y - min_y;

    qInfo() << "[MRPT Loader] Map bounds: x=[" << min_x << "," << max_x
            << "] y=[" << min_y << "," << max_y << "]"
            << "size=" << width << "x" << height;

    // Update grid dimensions
    params.GRID_MAX_DIM = QRectF(min_x, min_y, width, height);

    // Load the map using selected grid mode
    if (Gridder_loadMRPTMap(filepath))
    {
        std::string msg = "Successfully loaded map with: " +
                         std::to_string(load_result.cells.size()) + " cells";
        qInfo() << msg.c_str();
        return msg;
    }
    else
    {
        return "Failed to load MRPT map";
    }
}
RoboCompGridder::Pose SpecificWorker::Gridder_getPose()
{
    RoboCompGridder::Pose ret{};

    // Read from thread-safe buffer
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    const auto& [estimated] = buffer_estimated_pose.read(timestamp);
    if (estimated.has_value())
    {
        const auto& pose = estimated.value();
        ret.x = pose.translation().x();
        ret.y = pose.translation().y();
        ret.theta = std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));

        // Get covariance from localizer if available
        if (params.USE_LOCALIZER && localizer_initialized.load())
            ret.cov = localizer.getCovarianceVector();  // [xx, xy, xθ, yy, yθ, θθ] symmetric
    }
    // else: returns default (0, 0, 0) with empty covariance if no pose available

    return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}
