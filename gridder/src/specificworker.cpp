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
}
void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

//chekpoint robocompUpdater
	std::cout << "Initialize worker" << std::endl;
    int period = 100;
	setPeriod("Compute", period);
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        // Viewer
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        // Don't limit sceneRect - allow unlimited panning
        // viewer->setSceneRect(params.GRID_MAX_DIM);
        viewer->show();

        // Fit the view to show the initial grid area centered on robot
        QRectF initial_view(-3000, -3000, 6000, 6000);  // 6m x 6m area centered on origin
        viewer->fitToScene(initial_view);

        // Initialize grids (both available, selection via params.GRID_MODE)
        grid.initialize(params.GRID_MAX_DIM, static_cast<int>(params.TILE_SIZE), &viewer->scene);
        grid_esdf.initialize(static_cast<int>(params.TILE_SIZE), &viewer->scene);

        // Configure A* parameters
        grid_esdf.params().max_astar_nodes = params.MAX_ASTAR_NODES;
        grid_esdf.params().astar_distance_factor = params.ASTAR_DISTANCE_FACTOR;

        // Load pre-computed MRPT map
        std::string map_file = "mapa2.gridmap";
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
            loc_params.max_particles = 2000;
            loc_params.draw_particles = true;
            loc_params.lidar_subsample = 5;  // Use every 5th LiDAR point
            localizer.setParams(loc_params);

            // Reset with Gaussian distribution around origin (will be updated on first pose)
            // The actual initialization will happen when we get the first ground truth pose
            qInfo() << "[Localizer] Initialized, waiting for first pose...";
        }

        // Lidar thread is created
        read_lidar_th = std::thread(&SpecificWorker::read_lidar,this);
        std::cout << __FUNCTION__ << " Started lidar reader" << std::endl;

        // mouse
        connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, [this](QPointF p)
        {
            qInfo() << "[MOUSE] New global target arrived:" << p;
            std::lock_guard<std::mutex> lock(mutex_path);

            // Use current robot position in world coordinates as source
            const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            const auto &[robot, lidar_world, lidar_local] = buffer_sync.read(timestamp);
            if (not robot.has_value()) return;

            const Eigen::Vector2f source = robot.value().translation();
            const Eigen::Vector2f target{p.x(), p.y()};
            std::vector<Eigen::Vector2f> path;

            auto start_time = std::chrono::high_resolution_clock::now();

            // Use appropriate grid based on mode
            if (params.GRID_MODE == Params::GridMode::SPARSE_ESDF)
            {
                // Sparse ESDF mode: use grid_esdf for path planning
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
            }
            else
            {
                // Dense grid modes: use original grid for path planning
                auto [success, msg, source_key, target_key] =
                    grid.validate_source_target(source, 750, target, 750);
                if (not success) return;

                std::vector<std::vector<Eigen::Vector2f>> paths;

                // First check if there's line of sight to target
                if (grid.is_line_of_sigth_to_target_free(source_key, target_key, params.ROBOT_SEMI_WIDTH))
                {
                    // Direct path - line of sight is free
                    auto direct_path = grid.compute_path_line_of_sight(source_key, target_key, params.ROBOT_SEMI_LENGTH);
                    if (!direct_path.empty())
                    {
                        paths.push_back(direct_path);
                        qInfo() << "Line of sight path found";
                    }
                }

                // If no direct path, use Dijkstra/A*
                if (paths.empty())
                {
                    paths = grid.compute_k_paths(source_key,
                                                 target_key,
                                                 std::clamp(3, 1, params.NUM_PATHS_TO_SEARCH),
                                                 params.MIN_DISTANCE_BETWEEN_PATHS,
                                                 true, true);
                    qInfo() << "Dijkstra path computed";
                }

                if (!paths.empty())
                    path = paths.front();
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
                if (params.GRID_MODE == Params::GridMode::SPARSE_ESDF)
                {
                    for (const auto &pt : path)
                        path_cost += grid_esdf.get_cost(pt);
                }
                else
                {
                    for (const auto &pt : path)
                        path_cost += grid.get_cost(pt);
                }

                // Update UI displays
                lcdNumber_length->display(static_cast<double>(path_length / 1000.f));  // Show in meters
                lcdNumber_cost->display(static_cast<int>(path_cost));
                lcdNumber_elapsed->display(static_cast<double>(elapsed_sec));  // Show in seconds

                qInfo() << "Path found with" << path.size() << "waypoints, length:"
                        << path_length/1000.f << "m, cost:" << path_cost << ", time:" << elapsed_sec << "s";
                draw_path(path, &viewer->scene);
            }
            else
            {
                // Clear UI on no path
                lcdNumber_length->display(0);
                lcdNumber_cost->display(0);
                lcdNumber_elapsed->display(static_cast<double>(elapsed_sec));  // Show in seconds
                qInfo() << "No path found!";
            }
        });
        connect(viewer, &AbstractGraphicViewer::right_click, [this](QPointF p)
        {
            qInfo() <<  "RIGHT CLICK. Cancelling target";
            draw_path({}, &viewer->scene, true);
            cancel_from_mouse = true;
        });
        // Shift+Left click to reposition robot (for initial placement with external maps)
        connect(viewer, &AbstractGraphicViewer::robot_moved, [this](QPointF p)
        {
            qInfo() << "[ROBOT] Manual reposition to:" << p;
            // Update robot visual position
            viewer->robot_poly()->setPos(p.x(), p.y());
            // Update internal robot pose (for path planning source)
            robot_pose.translation() = Eigen::Vector2f(p.x(), p.y());
        });
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
    const auto &points_local = lidar_local.value();

    /// Update grid with world coordinates (only if no external map loaded)
    mutex_path.lock();
    if (not external_map_loaded)
    {
        // SPARSE_ESDF mode: VoxBlox-style, only stores obstacles
        grid.check_and_resize(points_world);
        grid_esdf.update(points_world, robot_pos.translation(), params.MAX_LIDAR_RANGE, timestamp);
        grid_esdf.update_visualization(true);  // Only update visualization when there are changes
    }
    // When external_map_loaded, visualization is static - no need to update
    mutex_path.unlock();

    // ============ LOCALIZER UPDATE ============
    float display_x = robot_pos.translation().x();
    float display_y = robot_pos.translation().y();
    float display_angle = std::atan2(robot_pos.linear()(1,0), robot_pos.linear()(0,0));

    if (params.USE_LOCALIZER and external_map_loaded)
    {
        if (const auto estimated = update_localizer(robot_pos, points_local); estimated.has_value())
        {
            // Sanity check: only use estimate if it's reasonable
            // (not too far from GT - max 5m error allowed)
            float error = std::hypot(estimated->x - display_x, estimated->y - display_y);
            if (error < 5000.f && localizer.getEffectiveSampleSize() > 10.0)
            {
                // Use localized pose for display
                display_x = estimated->x;
                display_y = estimated->y;
                display_angle = estimated->theta;
            }
        }
    }

    // Debug: draw lidar points to inspect noise
    if(params.DRAW_LIDAR_POINTS)
        draw_lidar_points(points_world);

    // Update robot visualization in the viewer (using localized pose if available)
    viewer->robot_poly()->setPos(display_x, display_y);
    viewer->robot_poly()->setRotation(qRadiansToDegrees(display_angle));

    this->hz = fps.print("FPS:", 3000);
    this->lcdNumber_hz->display(this->hz);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
std::optional<Localizer::Pose2D> SpecificWorker::update_localizer(const Eigen::Affine2f &robot_pos, const std::vector<Eigen::Vector2f> &points_local)
{
    // Get current ground truth pose
    const float gt_x = robot_pos.translation().x();
    const float gt_y = robot_pos.translation().y();
    const auto robot_rot = robot_pos.linear();
    const float gt_theta = std::atan2(robot_rot(1,0), robot_rot(0,0));
    Localizer::Pose2D current_gt_pose{gt_x, gt_y, gt_theta};

    // Initialize localizer on first valid pose
    if (!localizer_initialized)
    {
        // Initialize with Gaussian around ground truth (simulating initial uncertainty)
        localizer.resetGaussian(current_gt_pose, 500.f, 0.2f);  // 500mm pos, 0.2rad angle uncertainty
        last_ground_truth_pose = current_gt_pose;
        localizer_initialized = true;
        qInfo() << "[Localizer] Initialized at:" << gt_x << gt_y << "theta:" << qRadiansToDegrees(gt_theta);
        return std::nullopt;
    }

    // Compute odometry from ground truth poses (simulating odometry)
    auto odom = Localizer::computeOdometryDelta(last_ground_truth_pose, current_gt_pose);

    // Debug: check if poses are different
    float pose_diff = std::hypot(current_gt_pose.x - last_ground_truth_pose.x,
                                  current_gt_pose.y - last_ground_truth_pose.y);
    static int odom_debug = 0;
    if (++odom_debug % 50 == 0 && pose_diff > 1.0f)
    {
        qInfo() << "[Localizer ODOM] last:" << last_ground_truth_pose.x << last_ground_truth_pose.y
                << "curr:" << current_gt_pose.x << current_gt_pose.y
                << "diff:" << pose_diff << "odom:" << odom.delta_x << odom.delta_y;
    }

    // LiDAR points already come in local (robot) frame - no transformation needed
    // Update localizer directly with local points
    auto estimated_pose = localizer.update(odom, points_local);

    // Periodic debug: show GT errors and LiDAR stats every ~10 seconds
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0)
    {
        // Compute LiDAR stats using GT pose
        const float c = std::cos(gt_theta);
        const float s = std::sin(gt_theta);
        int hits = 0;
        float sum_dist = 0;
        const int sample_step = std::max(1, static_cast<int>(points_local.size() / 100));
        int count = 0;
        for (size_t i = 0; i < points_local.size(); i += sample_step)
        {
            const auto& p = points_local[i];
            const float wx = gt_x + p.x() * c - p.y() * s;
            const float wy = gt_y + p.x() * s + p.y() * c;
            auto key = grid_esdf.point_to_key(Eigen::Vector2f(wx, wy));
            float dist = grid_esdf.get_distance(key);
            sum_dist += dist;
            count++;
            if (dist < 100.f) hits++;
        }

        // Get estimated pose
        auto est = localizer.getMeanPose();
        float pos_error = std::hypot(est.x - gt_x, est.y - gt_y);
        float angle_error = std::abs(Localizer::Pose2D::normalizeAngle(est.theta - gt_theta));

        // Compute accumulated translation since init
        static float init_gt_x = gt_x, init_gt_y = gt_y;
        static float init_est_x = est.x, init_est_y = est.y;
        static bool first_debug = true;
        if (first_debug) { init_gt_x = gt_x; init_gt_y = gt_y; init_est_x = est.x; init_est_y = est.y; first_debug = false; }

        float gt_moved = std::hypot(gt_x - init_gt_x, gt_y - init_gt_y);
        float est_moved = std::hypot(est.x - init_est_x, est.y - init_est_y);

        qInfo() << "[Localizer] GT error: pos=" << static_cast<int>(pos_error) << "mm, angle=" << qRadiansToDegrees(angle_error) << "deg"
                << "| odom: dx=" << static_cast<int>(odom.delta_x) << "dy=" << static_cast<int>(odom.delta_y)
                << "| GT moved:" << static_cast<int>(gt_moved) << "mm, Est moved:" << static_cast<int>(est_moved) << "mm"
                << "| hits:" << hits << "/" << count
                << "| ESS:" << static_cast<int>(localizer.getEffectiveSampleSize());
    }

    last_ground_truth_pose = current_gt_pose;

    // Return estimated pose (or mean pose if not converged)
    return estimated_pose.has_value() ? estimated_pose : std::optional<Localizer::Pose2D>(localizer.getMeanPose());
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

//////////////////////////////////////////////////////////////////////////////////////////////
/// Clustering de puntos LiDAR para detección de obstáculos convexos
//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Agrupa puntos LiDAR en clusters usando Adaptive Breakpoint Detection
 *
 * El algoritmo funciona así:
 * 1. Los puntos del LiDAR vienen ordenados angularmente
 * 2. Dos puntos consecutivos pertenecen al mismo cluster si:
 *    - La distancia entre ellos < umbral adaptativo
 *    - El umbral crece con la distancia al robot (puntos lejanos están más espaciados)
 *
 * @param points Puntos LiDAR en coordenadas mundo
 * @param robot_pos Posición del robot en mundo
 * @param distance_threshold Umbral base de distancia (mm)
 * @param min_points Mínimo de puntos para considerar un cluster válido
 * @return Vector de clusters detectados
 */
std::vector<SpecificWorker::Cluster> SpecificWorker::cluster_lidar_points(
    const std::vector<Eigen::Vector2f> &points,
    const Eigen::Vector2f &robot_pos,
    float distance_threshold,
    int min_points)
{
    std::vector<Cluster> clusters;
    if (points.size() < static_cast<size_t>(min_points))
        return clusters;

    // Constantes para Adaptive Breakpoint Detection
    constexpr float ANGULAR_RESOLUTION = 0.004f;  // ~0.25 grados en radianes (típico para LiDAR)
    constexpr float LAMBDA = 10.0f;  // Factor de adaptación: threshold = base + lambda * sin(angular_res) * distance

    Cluster current_cluster;
    current_cluster.points.push_back(points[0]);

    for (size_t i = 1; i < points.size(); ++i)
    {
        const auto &p_prev = points[i - 1];
        const auto &p_curr = points[i];

        // Distancia entre puntos consecutivos
        const float dist_between = (p_curr - p_prev).norm();

        // Distancia adaptativa: puntos más lejanos tienen umbral mayor
        // Fórmula ABD: D_threshold = D_base + lambda * sin(delta_phi) * range
        const float range = (p_prev - robot_pos).norm();
        const float adaptive_threshold = distance_threshold + LAMBDA * std::sin(ANGULAR_RESOLUTION) * range;

        if (dist_between < adaptive_threshold)
        {
            // Mismo cluster
            current_cluster.points.push_back(p_curr);
        }
        else
        {
            // Nuevo cluster - guardar el anterior si tiene suficientes puntos
            if (current_cluster.points.size() >= static_cast<size_t>(min_points))
            {
                // Calcular centroide
                Eigen::Vector2f sum = Eigen::Vector2f::Zero();
                float min_dist = std::numeric_limits<float>::max();
                for (const auto &p : current_cluster.points)
                {
                    sum += p;
                    float d = (p - robot_pos).norm();
                    if (d < min_dist) min_dist = d;
                }
                current_cluster.centroid = sum / static_cast<float>(current_cluster.points.size());
                current_cluster.min_dist_to_robot = min_dist;

                // Calcular convex hull si hay suficientes puntos
                if (current_cluster.points.size() >= 3)
                    current_cluster.convex_hull = compute_convex_hull(current_cluster.points);

                clusters.push_back(std::move(current_cluster));
            }

            // Iniciar nuevo cluster
            current_cluster = Cluster();
            current_cluster.points.push_back(p_curr);
        }
    }

    // No olvidar el último cluster
    if (current_cluster.points.size() >= static_cast<size_t>(min_points))
    {
        Eigen::Vector2f sum = Eigen::Vector2f::Zero();
        float min_dist = std::numeric_limits<float>::max();
        for (const auto &p : current_cluster.points)
        {
            sum += p;
            float d = (p - robot_pos).norm();
            if (d < min_dist) min_dist = d;
        }
        current_cluster.centroid = sum / static_cast<float>(current_cluster.points.size());
        current_cluster.min_dist_to_robot = min_dist;

        if (current_cluster.points.size() >= 3)
            current_cluster.convex_hull = compute_convex_hull(current_cluster.points);

        clusters.push_back(std::move(current_cluster));
    }

    return clusters;
}

/**
 * @brief Calcula la envolvente convexa de un conjunto de puntos usando el algoritmo de Graham Scan
 * @param points Puntos de entrada
 * @return Puntos de la envolvente convexa en orden antihorario
 */
std::vector<Eigen::Vector2f> SpecificWorker::compute_convex_hull(const std::vector<Eigen::Vector2f> &points)
{
    if (points.size() < 3)
        return points;

    // Encontrar el punto más bajo (y menor, luego x menor)
    size_t min_idx = 0;
    for (size_t i = 1; i < points.size(); ++i)
    {
        if (points[i].y() < points[min_idx].y() ||
            (points[i].y() == points[min_idx].y() && points[i].x() < points[min_idx].x()))
        {
            min_idx = i;
        }
    }

    // Copiar puntos y poner el mínimo al principio
    std::vector<Eigen::Vector2f> sorted_points = points;
    std::swap(sorted_points[0], sorted_points[min_idx]);
    const Eigen::Vector2f pivot = sorted_points[0];

    // Ordenar por ángulo polar respecto al pivot
    std::sort(sorted_points.begin() + 1, sorted_points.end(),
              [&pivot](const Eigen::Vector2f &a, const Eigen::Vector2f &b)
              {
                  float angle_a = std::atan2(a.y() - pivot.y(), a.x() - pivot.x());
                  float angle_b = std::atan2(b.y() - pivot.y(), b.x() - pivot.x());
                  if (std::abs(angle_a - angle_b) < 1e-6f)
                  {
                      // Mismo ángulo: el más cercano primero
                      return (a - pivot).squaredNorm() < (b - pivot).squaredNorm();
                  }
                  return angle_a < angle_b;
              });

    // Graham scan
    std::vector<Eigen::Vector2f> hull;
    hull.push_back(sorted_points[0]);
    hull.push_back(sorted_points[1]);

    // Función para calcular el producto cruz (determina giro)
    auto cross = [](const Eigen::Vector2f &o, const Eigen::Vector2f &a, const Eigen::Vector2f &b) -> float
    {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };

    for (size_t i = 2; i < sorted_points.size(); ++i)
    {
        while (hull.size() > 1 && cross(hull[hull.size() - 2], hull[hull.size() - 1], sorted_points[i]) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(sorted_points[i]);
    }

    return hull;
}

/**
 * @brief Dibuja los clusters detectados en la escena
 */
void SpecificWorker::draw_clusters(const std::vector<Cluster> &clusters, QGraphicsScene *scene, bool erase_only)
{
    static std::vector<QGraphicsItem*> cluster_items;
    static QColor colors[] = {QColor("cyan"), QColor("magenta"), QColor("yellow"),
                              QColor("lime"), QColor("orange"), QColor("pink")};

    // Limpiar items anteriores
    for (auto *item : cluster_items)
    {
        scene->removeItem(item);
        delete item;  // Fix memory leak
    }
    cluster_items.clear();

    if (erase_only) return;

    int color_idx = 0;
    for (const auto &cluster : clusters)
    {
        QColor color = colors[color_idx % 6];
        color_idx++;

        // Dibujar convex hull si existe
        if (cluster.convex_hull.size() >= 3)
        {
            QPolygonF polygon;
            for (const auto &p : cluster.convex_hull)
                polygon << QPointF(p.x(), p.y());
            polygon << QPointF(cluster.convex_hull[0].x(), cluster.convex_hull[0].y());  // cerrar

            auto *poly_item = scene->addPolygon(polygon, QPen(color, 30), QBrush(color, Qt::Dense4Pattern));
            poly_item->setZValue(3);
            cluster_items.push_back(poly_item);
        }

        // Dibujar centroide
        constexpr float cs = 80.f;
        auto *centroid_item = scene->addEllipse(-cs/2, -cs/2, cs, cs, QPen(Qt::black, 20), QBrush(color));
        centroid_item->setPos(cluster.centroid.x(), cluster.centroid.y());
        centroid_item->setZValue(4);
        cluster_items.push_back(centroid_item);
    }
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

void SpecificWorker::draw_path(const std::vector<Eigen::Vector2f> &path, QGraphicsScene *scene, bool erase_only)
{
    static std::vector<QGraphicsEllipseItem*> points;
    static const QColor color("green");
    static const QPen pen(color);
    static const QBrush brush(color);
    static constexpr float s = 100;

    for(auto p : points)
    {
        scene->removeItem(p);
        delete p;  // Fix memory leak
    }
    points.clear();

    if(erase_only) return;

    points.reserve(path.size());
    for(const auto &p: path)
    {
        auto ptr = scene->addEllipse(-s/2, -s/2, s, s, pen, brush);
        ptr->setPos(p.x(), p.y());
        ptr->setZValue(10);  // Draw on top of grid, obstacles, and robot
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

///////////////////////////////////////////////////////////////////////////////////////////////////
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

    if (params.GRID_MODE == Params::GridMode::SPARSE_ESDF)
    {
        // Sparse ESDF mode: use grid_esdf for path planning
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
    }
    else
    {
        // Dense grid modes
        auto [val_success, val_msg, source_key, target_key] =
                grid.validate_source_target(Eigen::Vector2f{source.x, source.y},
                                            source.radius,
                                            Eigen::Vector2f{target.x, target.y},
                                            source.radius);
        success = val_success;
        msg = val_msg;

        if (success)
        {
            //check if is line of sight to target free
            if (grid.is_line_of_sigth_to_target_free(source_key,
                                                     target_key,
                                                     params.ROBOT_SEMI_WIDTH))
            {
                paths.emplace_back(grid.compute_path_line_of_sight(source_key, target_key, params.ROBOT_SEMI_LENGTH));
                if(paths.empty())
                    msg = "VLOS path not found";
                else
                    msg = "VLOS path";
            }
            else
            {
                paths = grid.compute_k_paths(source_key, target_key,
                                             std::clamp(max_paths, 1, params.NUM_PATHS_TO_SEARCH),
                                             params.MIN_DISTANCE_BETWEEN_PATHS,
                                             tryClosestFreePoint,
                                             targetIsHuman);
                if(paths.empty())
                    msg = "Djikstra path not found";
                else
                    msg = "Djikstra path";
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

    if (params.GRID_MODE == Params::GridMode::SPARSE_ESDF)
    {
        // Sparse ESDF mode: use grid_esdf for line of sight check
        return grid_esdf.is_line_of_sight_free(
            Eigen::Vector2f{source.x, source.y},
            Eigen::Vector2f{target.x, target.y},
            robotRadius);
    }
    else
    {
        // Dense grid modes
        auto [success, msg, source_key, target_key] =
                grid.validate_source_target(Eigen::Vector2f{source.x, source.y}, source.radius,
                                            Eigen::Vector2f{target.x, target.y}, target.radius);

        if(success)
            return grid.is_line_of_sigth_to_target_free(source_key, target_key, robotRadius);
        else
            return false;
    }
}
RoboCompGridder::TPoint SpecificWorker::Gridder_getClosestFreePoint(RoboCompGridder::TPoint source)
{
    std::lock_guard<std::mutex> lock(mutex_path);

    if (params.GRID_MODE == Params::GridMode::SPARSE_ESDF)
    {
        // Sparse ESDF mode: in sparse grid, non-obstacle = free
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
    else
    {
        // Dense grid modes
        if(const auto &p = grid.closest_free({source.x, source.y}); p.has_value())
            return {static_cast<float>(p->x()), static_cast<float>(p->y())};
        else
            return {0, 0};  // non valid closest point
    }
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
    //qInfo() << __FUNCTION__ << " Requesting map. Current grid mode: " << (params.GRID_MODE == Params::GridMode::SPARSE_ESDF ? "SPARSE_ESDF" : (params.GRID_MODE == Params::GridMode::DENSE_ESDF ? "DENSE_ESDF" : "DENSE"));
    std::lock_guard<std::mutex> lock(mutex_path);
    RoboCompGridder::Map result;

    if (params.GRID_MODE == Params::GridMode::SPARSE_ESDF)
    {
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
    }
    else
    {
        // Serialize dense grid - only cells with cost > 0
        result.tileSize = static_cast<int>(params.TILE_SIZE);

        for (const auto &[key, cell] : grid)
        {
            if (cell.cost > 1.0f)  // Skip free cells (cost=1)
            {
                RoboCompGridder::TCell tc;
                tc.x = key.first;
                tc.y = key.second;
                // Normalize cost to 0-255 range
                tc.cost = static_cast<uint8_t>(std::min(255.f, cell.cost * 2.55f));
                result.cells.push_back(tc);
            }
        }

        qInfo() << __FUNCTION__ << "Returning dense map with" << result.cells.size() << "cells";
    }

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

    std::vector<std::tuple<std::pair<int, int>, Grid::T>> submap_copy;

    //Lambda to get grid keys from TPointVector
    auto get_key_vector = [this](const RoboCompGridder::TPointVector &v)
    {
        std::vector<std::tuple<Grid::Key, float>> keys;
        for(const auto &p: v)
            keys.push_back(std::make_tuple(grid.point_to_key(Eigen::Vector2f(p.x, p.y)), p.radius));
        return keys;
    };

    //get keys from TPointVector
    auto free_keys = get_key_vector(freePoints);
    auto obstacle_keys = get_key_vector(obstaclePoints);

    mutex_path.lock();
    //Copy submap from actual grid
    // iterate over free keys and obstacle keys to copy submap
    for (auto &key: free_keys){
        auto cells = grid.copy_submap(std::get<0>(key), std::get<1>(key));
        std::move(cells.begin(), cells.end(), std::back_inserter(submap_copy));

    }
    for (auto &key: obstacle_keys){
        auto cells = grid.copy_submap(std::get<0>(key), std::get<1>(key));
        std::move(cells.begin(), cells.end(), std::back_inserter(submap_copy));

    }

    //Iterate over free keys and obstacle keys to set submap
    for (auto &key: free_keys){
        grid.set_submap(std::get<0>(key), std::get<1>(key), true);
    }
    for (auto &key: obstacle_keys){
        grid.set_submap(std::get<0>(key), std::get<1>(key), false);
    }

    //get paths
    auto result = Gridder_getPaths(source, target, 1, true, true, params.SAFETY_FACTOR);

    //restore submap
    grid.paste_submap(submap_copy);
    mutex_path.unlock();

    return result;
}
bool SpecificWorker::Gridder_IsPathBlocked(RoboCompGridder::TPath path)
{
    std::lock_guard<std::mutex> lock(mutex_path);

    if (params.GRID_MODE == Params::GridMode::SPARSE_ESDF)
    {
        // Sparse ESDF mode: check each segment of the path
        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            const Eigen::Vector2f from{path[i].x, path[i].y};
            const Eigen::Vector2f to{path[i + 1].x, path[i + 1].y};
            if (!grid_esdf.is_line_of_sight_free(from, to, params.ROBOT_SEMI_WIDTH))
                return true;  // Path is blocked
        }
        return false;  // Path is free
    }
    else
    {
        // Dense grid modes
        std::vector<Eigen::Vector2f> path_;
        for(const auto &p: path)
            path_.emplace_back(p.x, p.y);
        return grid.is_path_blocked(path_);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
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

    // Choose loading strategy based on grid mode
    if (params.GRID_MODE == Params::GridMode::SPARSE_ESDF)
    {
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

        if (offset_x != 0.f || offset_y != 0.f || rotation != 0.f)
            qInfo() << "[MRPT Loader] Applying transform: rotation=" << rotation << "rad, offset=(" << offset_x << "," << offset_y << ")mm";

        for (const auto &mrpt_cell : load_result.cells)
        {
            // Only add occupied cells (already filtered in loader, but double-check)
            if (MRPTMapLoader::is_occupied(mrpt_cell.occupancy, 0.5f))
            {
                // First rotate around origin, then apply offset
                const float orig_x = static_cast<float>(mrpt_cell.x);
                const float orig_y = static_cast<float>(mrpt_cell.y);
                const float cell_x = orig_x * cos_r - orig_y * sin_r + offset_x;
                const float cell_y = orig_x * sin_r + orig_y * cos_r + offset_y;
                const auto key = grid_esdf.point_to_key(Eigen::Vector2f(cell_x, cell_y));
                // Use add_confirmed_obstacle for external maps (creates visual immediately)
                grid_esdf.add_confirmed_obstacle(key);
            }
        }

        // Mark dirty once after loading all obstacles, then update visualization
        grid_esdf.mark_visualization_dirty();
        grid_esdf.update_visualization(true);
        qInfo() << "[MRPT Loader] SPARSE_ESDF loaded with" << grid_esdf.num_obstacles() << "obstacles";
    }
    else
    {
        qInfo() << "[MRPT Loader] Loading into DENSE grid";
        // Clear previous data
        grid.reset();

        // Convert MRPT cells to dense grid format
        for (const auto &mrpt_cell : load_result.cells)
        {
            const Grid::Key key = grid.point_to_key(Eigen::Vector2f(
                static_cast<float>(mrpt_cell.x), static_cast<float>(mrpt_cell.y)));

            if (MRPTMapLoader::is_occupied(mrpt_cell.occupancy, 0.5f))
            {
                grid.set_occupied(key);
                grid.set_cost(key, MRPTMapLoader::convert_occupancy_to_cost(mrpt_cell.occupancy));
            }
            else
            {
                grid.set_free(key);
            }
        }

        // Update costs and visualization
        grid.update_costs(params.ROBOT_SEMI_WIDTH, true);
        qInfo() << "[MRPT Loader] DENSE grid loaded with" << grid.size() << "cells";
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
