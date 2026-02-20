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
	// Load parameters from config file
	loadParams(configLoader);

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
* \brief Load parameters from ConfigLoader
*/
void SpecificWorker::loadParams(const ConfigLoader& configLoader_)
{
	qInfo() << "[Config] Loading parameters from config.toml...";

	// Store configLoader pointer for later use (loadMPPIParams)
	configLoader = &configLoader_;

	// Helper lambda to safely get config values with defaults
	auto getFloat = [&configLoader_](const std::string& key, float defaultVal) -> float {
		try { return static_cast<float>(configLoader_.get<double>("specific." + key)); }
		catch (...) { return defaultVal; }
	};
	auto getInt = [&configLoader_](const std::string& key, int defaultVal) -> int {
		try { return configLoader_.get<int>("specific." + key); }
		catch (...) { return defaultVal; }
	};
	auto getBool = [&configLoader_](const std::string& key, bool defaultVal) -> bool {
		try { return configLoader_.get<bool>("specific." + key); }
		catch (...) { return defaultVal; }
	};
	auto getString = [&configLoader_](const std::string& key, const std::string& defaultVal) -> std::string {
		try { return configLoader_.get<std::string>("specific." + key); }
		catch (...) { return defaultVal; }
	};

	// Display and visualization
	params.DISPLAY = getBool("display", params.DISPLAY);
	params.DRAW_LIDAR_POINTS = getBool("draw_lidar_points", params.DRAW_LIDAR_POINTS);
	params.MAX_LIDAR_DRAW_POINTS = getInt("max_lidar_draw_points", params.MAX_LIDAR_DRAW_POINTS);

	// Robot dimensions
	params.ROBOT_WIDTH = getFloat("robot_width", params.ROBOT_WIDTH);
	params.ROBOT_LENGTH = getFloat("robot_length", params.ROBOT_LENGTH);
	params.ROBOT_SEMI_WIDTH = params.ROBOT_WIDTH / 2.f;
	params.ROBOT_SEMI_LENGTH = params.ROBOT_LENGTH / 2.f;
	params.MIN_DISTANCE_TO_TARGET = params.ROBOT_WIDTH / 2.f;

	// Robot initial position (used when USE_GT_WARMUP is false)
	params.ROBOT_INITIAL_X = getFloat("robot_initial_x", params.ROBOT_INITIAL_X);
	params.ROBOT_INITIAL_Y = getFloat("robot_initial_y", params.ROBOT_INITIAL_Y);

	// Robot kinematic model
	std::string robot_type_str = getString("robot_type", "differential");
	if (robot_type_str == "omnidirectional" || robot_type_str == "OMNIDIRECTIONAL")
		params.ROBOT_TYPE = RobotType::OMNIDIRECTIONAL;
	else
		params.ROBOT_TYPE = RobotType::DIFFERENTIAL;

	// Grid configuration
	params.TILE_SIZE = getFloat("tile_size", params.TILE_SIZE);
	float grid_min_x = getFloat("grid_min_x", params.GRID_MAX_DIM.x());
	float grid_min_y = getFloat("grid_min_y", params.GRID_MAX_DIM.y());
	float grid_width = getFloat("grid_width", params.GRID_MAX_DIM.width());
	float grid_height = getFloat("grid_height", params.GRID_MAX_DIM.height());
	params.GRID_MAX_DIM = QRectF(grid_min_x, grid_min_y, grid_width, grid_height);

	// LiDAR configuration
	params.LIDAR_NAME_LOW = getString("lidar_name_low", params.LIDAR_NAME_LOW);
	params.LIDAR_NAME_HIGH = getString("lidar_name_high", params.LIDAR_NAME_HIGH);
	params.MAX_LIDAR_LOW_RANGE = getFloat("max_lidar_low_range", params.MAX_LIDAR_LOW_RANGE);
	params.MAX_LIDAR_HIGH_RANGE = getFloat("max_lidar_high_range", params.MAX_LIDAR_HIGH_RANGE);
	params.MAX_LIDAR_RANGE = params.MAX_LIDAR_LOW_RANGE;  // Use low range as default
	params.LIDAR_LOW_DECIMATION_FACTOR = getInt("lidar_low_decimation", params.LIDAR_LOW_DECIMATION_FACTOR);
	params.LIDAR_HIGH_DECIMATION_FACTOR = getInt("lidar_high_decimation", params.LIDAR_HIGH_DECIMATION_FACTOR);

	// Path planning
	params.SAFETY_FACTOR = getFloat("safety_factor", params.SAFETY_FACTOR);
	params.MAX_ASTAR_NODES = static_cast<size_t>(getInt("max_astar_nodes", static_cast<int>(params.MAX_ASTAR_NODES)));
	params.ASTAR_DISTANCE_FACTOR = getFloat("astar_distance_factor", params.ASTAR_DISTANCE_FACTOR);
	params.NUM_PATHS_TO_SEARCH = getInt("num_paths_to_search", params.NUM_PATHS_TO_SEARCH);
	params.MIN_DISTANCE_BETWEEN_PATHS = getFloat("min_distance_between_paths", params.MIN_DISTANCE_BETWEEN_PATHS);
	params.ELAPSED_TIME_BETWEEN_PATH_UPDATES = static_cast<unsigned int>(getInt("elapsed_time_between_path_updates",
	                                            static_cast<int>(params.ELAPSED_TIME_BETWEEN_PATH_UPDATES)));

	// MRPT map alignment
	params.MRPT_MAP_OFFSET_X = getFloat("mrpt_map_offset_x", params.MRPT_MAP_OFFSET_X);
	params.MRPT_MAP_OFFSET_Y = getFloat("mrpt_map_offset_y", params.MRPT_MAP_OFFSET_Y);
	params.MRPT_MAP_ROTATION = getFloat("mrpt_map_rotation", params.MRPT_MAP_ROTATION);
	params.MRPT_MAP_MIRROR_X = getBool("mrpt_map_mirror_x", params.MRPT_MAP_MIRROR_X);
	params.MAP_DILATION_RADIUS = getInt("map_dilation_radius", params.MAP_DILATION_RADIUS);

	// Localizer parameters (from [localizer] section)
	params.USE_LOCALIZER = getBool("localizer.enabled", params.USE_LOCALIZER);
	params.LOCALIZER_PERIOD_MS = getInt("localizer.period_ms", params.LOCALIZER_PERIOD_MS);

	// Particle filter configuration
	params.LOCALIZER_MIN_PARTICLES = static_cast<size_t>(getInt("localizer.min_particles", static_cast<int>(params.LOCALIZER_MIN_PARTICLES)));
	params.LOCALIZER_MAX_PARTICLES = static_cast<size_t>(getInt("localizer.max_particles", static_cast<int>(params.LOCALIZER_MAX_PARTICLES)));
	params.LOCALIZER_INITIAL_PARTICLES = static_cast<size_t>(getInt("localizer.initial_particles", static_cast<int>(params.LOCALIZER_INITIAL_PARTICLES)));

	// KLD-sampling parameters
	params.LOCALIZER_KLD_BIN_SIZE_XY = getFloat("localizer.kld_bin_size_xy", params.LOCALIZER_KLD_BIN_SIZE_XY);
	params.LOCALIZER_KLD_BIN_SIZE_THETA = getFloat("localizer.kld_bin_size_theta", params.LOCALIZER_KLD_BIN_SIZE_THETA);
	params.LOCALIZER_KLD_EPSILON = getFloat("localizer.kld_epsilon", params.LOCALIZER_KLD_EPSILON);

	// Motion model noise (alpha parameters)
	params.LOCALIZER_ALPHA1 = getFloat("localizer.alpha1", params.LOCALIZER_ALPHA1);
	params.LOCALIZER_ALPHA2 = getFloat("localizer.alpha2", params.LOCALIZER_ALPHA2);
	params.LOCALIZER_ALPHA3 = getFloat("localizer.alpha3", params.LOCALIZER_ALPHA3);
	params.LOCALIZER_ALPHA4 = getFloat("localizer.alpha4", params.LOCALIZER_ALPHA4);

	// Minimum diffusion noise
	params.LOCALIZER_MIN_TRANS_DIFFUSION = getFloat("localizer.min_trans_diffusion", params.LOCALIZER_MIN_TRANS_DIFFUSION);
	params.LOCALIZER_MIN_ROT_DIFFUSION = getFloat("localizer.min_rot_diffusion", params.LOCALIZER_MIN_ROT_DIFFUSION);

	// Observation model
	params.LOCALIZER_SIGMA_HIT = getFloat("localizer.sigma_hit", params.LOCALIZER_SIGMA_HIT);
	params.LOCALIZER_Z_HIT = getFloat("localizer.z_hit", params.LOCALIZER_Z_HIT);
	params.LOCALIZER_LIDAR_SUBSAMPLE = getInt("localizer.lidar_subsample", params.LOCALIZER_LIDAR_SUBSAMPLE);

	// Resampling
	params.LOCALIZER_RESAMPLE_THRESHOLD = getFloat("localizer.resample_threshold", params.LOCALIZER_RESAMPLE_THRESHOLD);

	// Convergence thresholds
	params.LOCALIZER_POSITION_STDDEV_THRESHOLD = getFloat("localizer.position_stddev_threshold", params.LOCALIZER_POSITION_STDDEV_THRESHOLD);
	params.LOCALIZER_ANGLE_STDDEV_THRESHOLD = getFloat("localizer.angle_stddev_threshold", params.LOCALIZER_ANGLE_STDDEV_THRESHOLD);

	// Ground Truth warmup
	params.USE_GT_WARMUP = getBool("use_gt_warmup", params.USE_GT_WARMUP);

	// MPPI controller
	params.MPPI_PERIOD_MS = getInt("mppi.period_ms", params.MPPI_PERIOD_MS);

	// Timing
	params.PERIOD_HYSTERESIS = static_cast<long>(getInt("period_hysteresis", static_cast<int>(params.PERIOD_HYSTERESIS)));
	params.PERIOD = getInt("Period.Compute", params.PERIOD);  // From [Period] section

	qInfo() << "[Config] Parameters loaded:";
	qInfo() << "  - Robot:" << params.ROBOT_WIDTH << "x" << params.ROBOT_LENGTH << "mm,"
	        << (params.ROBOT_TYPE == RobotType::DIFFERENTIAL ? "differential" : "omnidirectional");
	qInfo() << "  - Robot initial pos:" << params.ROBOT_INITIAL_X << "," << params.ROBOT_INITIAL_Y << "mm";
	qInfo() << "  - Grid: tile=" << params.TILE_SIZE << "mm, bounds="
	        << params.GRID_MAX_DIM.x() << "," << params.GRID_MAX_DIM.y()
	        << "," << params.GRID_MAX_DIM.width() << "," << params.GRID_MAX_DIM.height();
	qInfo() << "  - Localizer:" << (params.USE_LOCALIZER ? "enabled" : "disabled")
	        << ", particles=" << params.LOCALIZER_INITIAL_PARTICLES;
	qInfo() << "  - GT warmup:" << (params.USE_GT_WARMUP ? "enabled (simulation)" : "disabled (real robot)");
}

MPPIController::Params SpecificWorker::loadMPPIParams()
{
	MPPIController::Params p;  // Start with defaults from header

	// Helper lambdas using configLoader stored in class
	auto getFloat = [this](const std::string& key, float defaultVal) -> float {
		try { return static_cast<float>(configLoader->get<double>(key)); }
		catch (...) { return defaultVal; }
	};
	auto getInt = [this](const std::string& key, int defaultVal) -> int {
		try { return configLoader->get<int>(key); }
		catch (...) { return defaultVal; }
	};
	auto getBool = [this](const std::string& key, bool defaultVal) -> bool {
		try { return configLoader->get<bool>(key); }
		catch (...) { return defaultVal; }
	};

	// Core MPPI parameters
	p.K = getInt("mppi.K", p.K);
	p.T = getInt("mppi.T", p.T);
	p.dt = getFloat("mppi.dt", p.dt);
	p.lambda = getFloat("mppi.lambda", p.lambda);
	p.cost_scale = getFloat("mppi.cost_scale", p.cost_scale);

	// Adaptive K
	p.use_adaptive_K = getBool("mppi.use_adaptive_K", p.use_adaptive_K);
	p.K_min = getInt("mppi.K_min", p.K_min);
	p.K_max = getInt("mppi.K_max", p.K_max);
	p.ess_ratio_low = getFloat("mppi.ess_ratio_low", p.ess_ratio_low);
	p.ess_ratio_high = getFloat("mppi.ess_ratio_high", p.ess_ratio_high);
	p.K_increase_factor = getFloat("mppi.K_increase_factor", p.K_increase_factor);
	p.K_decrease_factor = getFloat("mppi.K_decrease_factor", p.K_decrease_factor);

	// Control noise
	p.sigma_vx = getFloat("mppi.sigma_vx", p.sigma_vx);
	p.sigma_vy = getFloat("mppi.sigma_vy", p.sigma_vy);
	p.sigma_omega = getFloat("mppi.sigma_omega", p.sigma_omega);

	// Time-correlated noise
	p.noise_alpha = getFloat("mppi.noise_alpha", p.noise_alpha);
	p.use_time_correlated_noise = getBool("mppi.use_time_correlated_noise", p.use_time_correlated_noise);

	// Adaptive covariance
	p.use_adaptive_covariance = getBool("mppi.use_adaptive_covariance", p.use_adaptive_covariance);
	p.cov_adaptation_rate = getFloat("mppi.cov_adaptation_rate", p.cov_adaptation_rate);
	p.sigma_min_vx = getFloat("mppi.sigma_min_vx", p.sigma_min_vx);
	p.sigma_min_vy = getFloat("mppi.sigma_min_vy", p.sigma_min_vy);
	p.sigma_min_omega = getFloat("mppi.sigma_min_omega", p.sigma_min_omega);
	p.sigma_max_vx = getFloat("mppi.sigma_max_vx", p.sigma_max_vx);
	p.sigma_max_vy = getFloat("mppi.sigma_max_vy", p.sigma_max_vy);
	p.sigma_max_omega = getFloat("mppi.sigma_max_omega", p.sigma_max_omega);

	// Robot limits
	p.max_vx = getFloat("mppi.max_vx", p.max_vx);
	p.max_vy = getFloat("mppi.max_vy", p.max_vy);
	p.max_omega = getFloat("mppi.max_omega", p.max_omega);

	// Cost weights
	p.w_path = getFloat("mppi.w_path", p.w_path);
	p.w_obstacle = getFloat("mppi.w_obstacle", p.w_obstacle);
	p.w_goal = getFloat("mppi.w_goal", p.w_goal);
	p.w_smoothness = getFloat("mppi.w_smoothness", p.w_smoothness);
	p.w_speed = getFloat("mppi.w_speed", p.w_speed);

	// Safety parameters
	p.collision_buffer = getFloat("mppi.collision_buffer", p.collision_buffer);
	p.safety_margin = getFloat("mppi.safety_margin", p.safety_margin);
	p.obstacle_decay = getFloat("mppi.obstacle_decay", p.obstacle_decay);

	// Covariance-aware margin
	p.use_covariance_inflation = getBool("mppi.use_covariance_inflation", p.use_covariance_inflation);
	p.cov_z_score = getFloat("mppi.cov_z_score", p.cov_z_score);
	p.cov_inflation_gate = getFloat("mppi.cov_inflation_gate", p.cov_inflation_gate);
	p.cov_sigma_max_clamp = getFloat("mppi.cov_sigma_max_clamp", p.cov_sigma_max_clamp);

	// Path following
	p.lookahead_distance = getFloat("mppi.lookahead_distance", p.lookahead_distance);
	p.goal_tolerance = getFloat("mppi.goal_tolerance", p.goal_tolerance);

	// Warm start
	p.warm_start_vx_weight = getFloat("mppi.warm_start_vx_weight", p.warm_start_vx_weight);
	p.warm_start_vy_weight = getFloat("mppi.warm_start_vy_weight", p.warm_start_vy_weight);
	p.warm_start_omega_weight = getFloat("mppi.warm_start_omega_weight", p.warm_start_omega_weight);

	// Obstacle cost smoothing
	p.obstacle_k_nearest = getInt("mppi.obstacle_k_nearest", p.obstacle_k_nearest);
	p.obstacle_softmin_beta = getFloat("mppi.obstacle_softmin_beta", p.obstacle_softmin_beta);

	// Nominal control parameters
	p.alignment_forward_threshold = getFloat("mppi.alignment_forward_threshold", p.alignment_forward_threshold);
	p.alignment_backward_threshold = getFloat("mppi.alignment_backward_threshold", p.alignment_backward_threshold);
	p.lateral_motion_gain = getFloat("mppi.lateral_motion_gain", p.lateral_motion_gain);
	p.nominal_slow_speed_factor = getFloat("mppi.nominal_slow_speed_factor", p.nominal_slow_speed_factor);

	// Output smoothing
	p.output_smoothing_alpha = getFloat("mppi.output_smoothing_alpha", p.output_smoothing_alpha);

	// Visualization
	p.num_trajectories_to_draw = getInt("mppi.num_trajectories_to_draw", p.num_trajectories_to_draw);

	return p;
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
        viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 0.2, QColor("Blue"));

        // Initialize robot pose to configured initial position
        estimated_robot_pose = Eigen::Affine2f::Identity();
        estimated_robot_pose.translation() = Eigen::Vector2f{params.ROBOT_INITIAL_X, params.ROBOT_INITIAL_Y};
        viewer->robot_poly()->setPos(params.ROBOT_INITIAL_X, params.ROBOT_INITIAL_Y);
        qInfo() << "[Init] Robot initial position:" << params.ROBOT_INITIAL_X << params.ROBOT_INITIAL_Y;

        // Don't limit sceneRect - allow unlimited panning
        // viewer->setSceneRect(params.GRID_MAX_DIM);
        viewer->show();

        // Fit the view to show the initial grid area centered on robot
        QRectF initial_view(params.ROBOT_INITIAL_X - 3000, params.ROBOT_INITIAL_Y - 3000, 6000, 6000);
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

            // Configure localizer parameters from config
            Localizer::Params loc_params;

            // Particle counts
            loc_params.min_particles = params.LOCALIZER_MIN_PARTICLES;
            loc_params.max_particles = params.LOCALIZER_MAX_PARTICLES;
            loc_params.initial_particles = params.LOCALIZER_INITIAL_PARTICLES;

            // KLD-sampling
            loc_params.kld_bin_size_xy = params.LOCALIZER_KLD_BIN_SIZE_XY;
            loc_params.kld_bin_size_theta = params.LOCALIZER_KLD_BIN_SIZE_THETA;
            loc_params.kld_epsilon = params.LOCALIZER_KLD_EPSILON;

            // Motion model noise
            loc_params.alpha1 = params.LOCALIZER_ALPHA1;
            loc_params.alpha2 = params.LOCALIZER_ALPHA2;
            loc_params.alpha3 = params.LOCALIZER_ALPHA3;
            loc_params.alpha4 = params.LOCALIZER_ALPHA4;

            // Minimum diffusion noise (for tracking external motion)
            loc_params.min_trans_diffusion = params.LOCALIZER_MIN_TRANS_DIFFUSION;
            loc_params.min_rot_diffusion = params.LOCALIZER_MIN_ROT_DIFFUSION;

            // Observation model
            loc_params.sigma_hit = params.LOCALIZER_SIGMA_HIT;
            loc_params.z_hit = params.LOCALIZER_Z_HIT;
            loc_params.z_rand = 1.f - params.LOCALIZER_Z_HIT;
            loc_params.lidar_subsample = params.LOCALIZER_LIDAR_SUBSAMPLE;

            // Resampling
            loc_params.resample_threshold = params.LOCALIZER_RESAMPLE_THRESHOLD;

            // Convergence
            loc_params.position_stddev_threshold = params.LOCALIZER_POSITION_STDDEV_THRESHOLD;
            loc_params.angle_stddev_threshold = params.LOCALIZER_ANGLE_STDDEV_THRESHOLD;

            // Visualization (controlled by UI checkbox)
            loc_params.draw_particles = false;

            localizer.setParams(loc_params);

            // Precompute distance field for fast lookups
            qInfo() << "[Localizer] Precomputing distance field...";
            grid_esdf.precompute_distance_field();

            // Mark map as ready for localization (after precomputation)
            map_ready_for_localization.store(true);
            qInfo() << "[Localizer] Initialized with" << loc_params.initial_particles << "particles";
            qInfo() << "[Localizer] Motion model: alpha1-4:" << loc_params.alpha1 << loc_params.alpha2
                    << loc_params.alpha3 << loc_params.alpha4;
            qInfo() << "[Localizer] Min diffusion: trans=" << loc_params.min_trans_diffusion
                    << "mm, rot=" << loc_params.min_rot_diffusion << "rad";
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

        // Initialize MPPI controller with parameters from config
        MPPIController::Params mppi_params = loadMPPIParams();

        // Override robot-specific values from [specific] section
        mppi_params.robot_radius = params.ROBOT_SEMI_WIDTH;
        mppi_params.robot_semi_width = params.ROBOT_SEMI_WIDTH;
        mppi_params.robot_semi_length = params.ROBOT_SEMI_LENGTH;
        mppi_params.use_footprint_sampling = true;  // Enable 8-point footprint collision detection

        // Set robot kinematic model
        mppi_params.robot_type = (params.ROBOT_TYPE == RobotType::DIFFERENTIAL)
            ? MPPIController::RobotType::DIFFERENTIAL
            : MPPIController::RobotType::OMNIDIRECTIONAL;

        mppi_controller.setParams(mppi_params);

        qInfo() << "[MPPI] Initialized with K=" << mppi_params.K << ", T=" << mppi_params.T
                << ", lambda=" << mppi_params.lambda;

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

            // Use estimated robot position (from localizer) as source for path planning
            const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

            Eigen::Vector2f source;
            bool source_valid = false;

            if (params.USE_LOCALIZER && localizer_initialized.load())
            {
                // Use localized pose
                const auto& [estimated] = buffer_estimated_pose.read(timestamp);
                if (estimated.has_value())
                {
                    source = estimated.value().translation();
                    source_valid = true;
                }
                else if (params.USE_GT_WARMUP)
                {
                    // Simulation only: Allow GT fallback during warmup
                    const auto &[robot, lw, ll] = buffer_sync.read(timestamp);
                    if (robot.has_value())
                    {
                        source = robot.value().translation();
                        source_valid = true;
                    }
                }
            }
            else if (params.USE_GT_WARMUP)
            {
                // Localizer disabled and GT warmup enabled (simulation mode)
                const auto &[robot, lw, ll] = buffer_sync.read(timestamp);
                if (robot.has_value())
                {
                    source = robot.value().translation();
                    source_valid = true;
                }
            }

            if (!source_valid)
            {
                qWarning() << "[PATH] No valid pose available for path planning";
                return;
            }

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

        // Shift+Left click to manually position robot (for initial localization without GT)
        connect(viewer, &AbstractGraphicViewer::robot_moved, [this](QPointF p)
        {
            qInfo() << "[ROBOT] Manual reposition to:" << p.x() << p.y();

            // Update robot visual position
            viewer->robot_poly()->setPos(p.x(), p.y());

            // Initialize localizer at this position if not yet initialized
            if (!localizer_initialized.load())
            {
                Localizer::Pose2D init_pose{static_cast<float>(p.x()),
                                           static_cast<float>(p.y()),
                                           0.0f};  // Assume 0 angle, will converge quickly
                localizer.resetGaussian(init_pose, 500.f, 0.2f);
                localizer_initialized.store(true);
                qInfo() << "[Localizer] Manually initialized at:" << p.x() << p.y() << "theta: 0";
            }
            else
            {
                qWarning() << "[ROBOT] Localizer already initialized, ignoring manual position";
            }
        });
	    // Connect robot drag and rotate signals
	    connect(viewer, &AbstractGraphicViewer::robot_dragging, this, &SpecificWorker::slot_robot_dragging);
	    connect(viewer, &AbstractGraphicViewer::robot_drag_end, this, &SpecificWorker::slot_robot_drag_end);
	    connect(viewer, &AbstractGraphicViewer::robot_rotate, this, &SpecificWorker::slot_robot_rotate);

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
	    const float rx = robot_pos.translation().x();
	    const float ry = robot_pos.translation().y();
	    // Fit a 6x6m window around the robot and center on it
        constexpr float view_side_m = 6000.f; //mm
	    viewer->fitToScene(QRectF(rx - view_side_m/2.f, ry - view_side_m/2.f, view_side_m, view_side_m));
	    viewer->centerOn(rx, ry);
        viewer->centerOn(robot_pos.translation().x(), robot_pos.translation().y());

        // Display initialization instructions based on GT warmup setting
        if (!params.USE_GT_WARMUP && params.USE_LOCALIZER)
        {
            qInfo() << "═══════════════════════════════════════════════════════════════";
            qInfo() << "  REAL ROBOT MODE: GT warmup disabled (USE_GT_WARMUP = false)";
            qInfo() << "═══════════════════════════════════════════════════════════════";
            qInfo() << "";
            qInfo() << "  To initialize the localizer, please:";
            qInfo() << "  1. Use Shift+Left Click to place the robot at its";
            qInfo() << "     approximate current position on the map";
            qInfo() << "  2. The localizer will initialize at that position";
            qInfo() << "  3. Wait 1-3 seconds for localization to converge";
            qInfo() << "";
            qInfo() << "  Until initialized, the robot will NOT move.";
            qInfo() << "═══════════════════════════════════════════════════════════════";
        }
    }
}
void SpecificWorker::compute()
{
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    // Read synchronized sensor data
    const auto &[robot, lidar_world, lidar_local] = buffer_sync.read(timestamp);
    if (not robot.has_value() or not lidar_world.has_value() or not lidar_local.has_value())
        { qWarning() << "No data from buffer_sync"; return; }

    const auto &robot_pos = robot.value();           // GT pose (only for odometry simulation)
    const auto &points_world = lidar_world.value();
    const auto &points_local = lidar_local.value();

    // Get current robot pose (from localizer or GT during warmup)
    auto current_pose_opt = get_current_pose(robot_pos, timestamp);
    if (!current_pose_opt.has_value())
        return;  // Waiting for localization

    Eigen::Affine2f current_robot_pose = current_pose_opt.value();
    estimated_robot_pose = current_robot_pose;

    // Update grid with LiDAR data (if no external map loaded)
    if (not external_map_loaded)
    {
        std::lock_guard<std::mutex> lock(mutex_path);
        grid_esdf.update(points_world, current_robot_pose.translation(), params.MAX_LIDAR_RANGE, timestamp);
        grid_esdf.update_visualization(true);
    }

    // Visualization (controlled by UI checkboxes)
    if (show_lidar_points.load())
        draw_lidar_points(points_local, estimated_robot_pose);
    if (show_particles.load() && params.USE_LOCALIZER)
        localizer.drawParticles();
    if (show_covariance.load() && params.USE_LOCALIZER && localizer_initialized.load())
        draw_covariance_ellipse();

    // Update robot visualization (skip if being dragged by mouse)
    if (!robot_being_dragged.load())
    {
        const float angle = std::atan2(estimated_robot_pose.linear()(1,0), estimated_robot_pose.linear()(0,0));
        viewer->robot_poly()->setPos(estimated_robot_pose.translation().x(), estimated_robot_pose.translation().y());
        viewer->robot_poly()->setRotation(qRadiansToDegrees(angle));
    }

    // Process MPPI output and send commands to robot
    process_mppi_output(timestamp);

    // Update UI displays (FPS, CPU, etc.)
    update_ui_displays();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Helper: Get current robot pose from localizer or GT (during warmup)
/////////////////////////////////////////////////////////////////////////////////////////////////
std::optional<Eigen::Affine2f> SpecificWorker::get_current_pose(const Eigen::Affine2f& gt_pose, std::uint64_t timestamp)
{
    // Case 1: Localizer enabled and initialized - use estimated pose
    if (params.USE_LOCALIZER && localizer_initialized.load())
    {
        if (const auto& [estimated] = buffer_estimated_pose.read(timestamp); estimated.has_value())
            return estimated.value();

        // Simulation fallback: use GT during warmup
        if (params.USE_GT_WARMUP)
            return gt_pose;

        return std::nullopt;  // Wait for localizer
    }

    // Case 2: GT warmup mode (simulation without localizer)
    if (params.USE_GT_WARMUP)
        return gt_pose;

    // Case 3: Real robot - waiting for manual initialization
    static int wait_counter = 0;
    if (++wait_counter % 50 == 0)
        qWarning() << "[compute] Waiting for manual robot positioning (Shift+Left Click)...";

    return std::nullopt;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Helper: Process MPPI output and send velocity commands
/////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::process_mppi_output(std::uint64_t timestamp)
{
    if (!mppi_enabled.load())
        return;

    // Read MPPI output and send to robot
    if (const auto& [mppi_out] = buffer_mppi_output.read(timestamp);
        mppi_out.has_value() && mppi_out->valid)
    {
        try {
            omnirobot_proxy->setSpeedBase(mppi_out->vx, mppi_out->vy, mppi_out->omega);

            // Store velocity command for odometry estimation
            {
                std::lock_guard<std::mutex> lock(mutex_velocity_command);
                last_velocity_command.vx = mppi_out->vx;
                last_velocity_command.vy = mppi_out->vy;
                last_velocity_command.omega = mppi_out->omega;
                last_velocity_command.timestamp = std::chrono::steady_clock::now();
            }
        }
        catch (const Ice::Exception &e) {
            static int error_count = 0;
            if (++error_count % 100 == 1)
                qWarning() << "[MPPI] Failed to send speed:" << e.what();
        }
    }

    // Update trajectory visualization if enabled
    if (show_trajectories.load())
    {
        std::lock_guard<std::mutex> lock(mutex_mppi_trajectory);
        if (!last_optimal_trajectory.empty())
            draw_mppi_trajectory(last_optimal_trajectory);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Helper: Update UI displays (FPS, CPU usage, etc.)
/////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::update_ui_displays()
{
    this->hz = fps.print("FPS:", 3000);
    this->lcdNumber_hz->display(this->hz);
    this->lcdNumber_loc_hz->display(localizer_hz.load());
    this->lcdNumber_mppi_hz->display(mppi_hz.load());

    // CPU usage with exponential moving average
    const float current_cpu = get_cpu_usage();
    cpu_usage_avg = CPU_AVG_ALPHA * current_cpu + (1.0f - CPU_AVG_ALPHA) * cpu_usage_avg;
    this->lcdNumber_cpu->display(static_cast<int>(cpu_usage_avg));
}

/////////////////////////////////////////////////////////////////////////////////////////////////
auto SpecificWorker::draw_lidar_points(const std::vector<Eigen::Vector2f> &points,
                                       const Eigen::Affine2f &robot_pose) -> void
{
    static std::vector<QGraphicsEllipseItem*> lidar_points;
    static const QPen pen(QColor("DarkBlue"));
    static const QBrush brush(QColor("DarkBlue"));
    static constexpr float s = 40.f;

    const int stride = std::max(1, static_cast<int>(points.size() / params.MAX_LIDAR_DRAW_POINTS));
    const size_t num_points_to_draw = (points.size() + stride - 1) / stride;

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
    for (size_t i = 0; i < points.size() && idx < num_points_to_draw; i += stride, ++idx)
    {
        const auto &p = robot_pose * points[i];
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

    // Standard deviations (sqrt of eigenvalues) scaled by 3 for 99% confidence ellipse
    // Also apply minimum size so ellipse is always visible
    const float scale = 3.0f;  // 3-sigma for 99% confidence ellipse
    const float min_size = 50.0f;  // Minimum size in mm for visibility
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
            QPen(QColor(255, 0, 255, 255), 10),  // Magenta border, thicker
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
// Utility: Convert quaternion to yaw angle
///////////////////////////////////////////////////////////////////////////////////////////////
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
            Eigen::Affine2f eig_pose;
            eig_pose = Eigen::Affine2f::Identity();
            eig_pose.translation() = Eigen::Vector2f{params.ROBOT_INITIAL_X, params.ROBOT_INITIAL_Y};  // Default initial position
            if (params.USE_GT_WARMUP)
            {
                const auto &[position, orientation] = webots2robocomp_proxy->getObjectPose("shadow");
                eig_pose.translation() = Eigen::Vector2f(-position.y, position.x);
                eig_pose.linear() = Eigen::Rotation2Df(yawFromQuaternion(orientation)).toRotationMatrix();
            }

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
                if (p.x*p.x + p.y*p.y < params.MAX_LIDAR_RANGE * params.MAX_LIDAR_RANGE)
                {
                    points_local.emplace_back(p.x, p.y);
                    points_world.emplace_back(eig_pose * Eigen::Vector2f{p.x, p.y});
                }

            // Put all in sync buffer with same timestamp
            buffer_sync.put<0>(std::move(eig_pose), timestamp);
            buffer_sync.put<1>(std::move(points_world), timestamp);
            buffer_sync.put<2>(std::move(points_local), timestamp);

            // Adjust period with hysteresis
            if (wait_period > std::chrono::milliseconds((long) data.period + 2)) --wait_period;
            else if (wait_period < std::chrono::milliseconds((long) data.period - 2)) ++wait_period;
        }
        catch (const Ice::Exception &e)
        { std::cout << "Error reading from Lidar3D or robot pose: " << e.what() << std::endl; }
        std::this_thread::sleep_for(wait_period);
    }
} // Thread to read the lidar

/////////////////////////////////////////////////////////////////////////////////////////////////
/// Localizer Thread - runs AMCL independently at its own rate
/// Uses velocity commands as odometry source (dead reckoning)
/////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::run_localizer()
{
    const auto target_period = std::chrono::milliseconds(params.LOCALIZER_PERIOD_MS);
    auto last_fps_time = std::chrono::steady_clock::now();
    auto last_odom_time = std::chrono::steady_clock::now();
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

        // Skip update if robot is being dragged
        if (robot_being_dragged.load())
        {
            last_odom_time = std::chrono::steady_clock::now();  // Reset odom timer
            std::this_thread::sleep_for(target_period);
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

        // Initialize localizer on first valid pose (use GT only for initial position if enabled)
        if (!localizer_initialized.load())
        {
            if (params.USE_GT_WARMUP)
            {
                const float gt_x = robot_pos.translation().x();
                const float gt_y = robot_pos.translation().y();
                const float gt_theta = std::atan2(robot_pos.linear()(1, 0), robot_pos.linear()(0, 0));
                Localizer::Pose2D init_pose{gt_x, gt_y, gt_theta};
                localizer.resetGaussian(init_pose, 500.f, 0.2f);
                localizer_initialized.store(true);
                qInfo() << "[Localizer Thread] Initialized from GT at:" << gt_x << gt_y
                        << "theta:" << qRadiansToDegrees(gt_theta);
            }
            // else: wait for manual initialization via Shift+Click
            last_odom_time = std::chrono::steady_clock::now();
            std::this_thread::sleep_for(target_period);
            continue;
        }

        // Compute odometry from velocity commands (dead reckoning)
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - last_odom_time).count();  // seconds
        last_odom_time = now;

        Localizer::OdometryDelta odom;
        {
            std::lock_guard<std::mutex> lock(mutex_velocity_command);
            // Convert velocity to displacement: delta = v * dt
            // Note: velocities are in mm/s, we keep mm for consistency
            odom.delta_x = last_velocity_command.vx * dt;
            odom.delta_y = last_velocity_command.vy * dt;
            odom.delta_theta = last_velocity_command.omega * dt;
        }

        // Update localizer with odometry and LiDAR points
        auto estimated_pose_opt = localizer.update(odom, points_local);

        // Get the estimated pose (from update result or mean pose)
        Localizer::Pose2D estimated_pose = estimated_pose_opt.value_or(localizer.getMeanPose());

        // Publish to buffer if ESS is good enough
        if (localizer.getEffectiveSampleSize() > 10.0)
        {
            Eigen::Affine2f result;
            result.translation() = Eigen::Vector2f(estimated_pose.x, estimated_pose.y);
            result.linear() = Eigen::Rotation2Df(estimated_pose.theta).toRotationMatrix();
            buffer_estimated_pose.put<0>(std::move(result), timestamp);
        }

        // Update FPS counter
        frame_count++;
        const auto fps_now = std::chrono::steady_clock::now();
        const auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(fps_now - last_fps_time).count();
        if (fps_elapsed >= 1000)
        {
            localizer_hz.store(frame_count);
            frame_count = 0;
            last_fps_time = fps_now;
        }

        // Periodic debug (localizer status)
        static int debug_counter = 0;
        if (++debug_counter % 100 == 0)
        {
            auto est = localizer.getMeanPose();
            Eigen::Matrix2f pos_cov;
            float theta_var;
            localizer.getCovariance(pos_cov, theta_var);
            qInfo() << "[Localizer Thread] Pose:" << static_cast<int>(est.x) << static_cast<int>(est.y)
                    << "theta:" << qRadiansToDegrees(est.theta) << "deg"
                    << "| ESS:" << static_cast<int>(localizer.getEffectiveSampleSize())
                    << "| σxy:" << static_cast<int>(std::sqrt(pos_cov(0,0))) << "mm";
        }

        // Adaptive sleep
        const auto loop_end = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        if (elapsed < target_period)
            std::this_thread::sleep_for(target_period - elapsed);
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
    const auto idle_period = std::chrono::milliseconds(200);  // Sleep longer when idle (5 Hz check)
    auto last_fps_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    while (!stop_mppi_thread)
    {
        const auto loop_start = std::chrono::steady_clock::now();

        // Check preconditions - sleep longer when not navigating to save CPU
        if (!mppi_enabled.load() || nav_state.load() != NavigationState::NAVIGATING)
        {
            mppi_hz.store(0);  // Show 0 FPS when not active
            std::this_thread::sleep_for(idle_period);
            continue;
        }

        const auto timestamp = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        // Read current robot pose from estimated pose buffer (ONLY from localizer, NO fallback to GT)
        Eigen::Affine2f robot_pose;
        {
            const auto& [estimated] = buffer_estimated_pose.read(timestamp);
            if (!estimated.has_value())
            {
                // Wait for localizer to provide a pose estimate - DO NOT use GT as fallback
                std::this_thread::sleep_for(target_period);
                continue;
            }
            robot_pose = estimated.value();
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

            // Clear trajectory visualization
            {
                std::lock_guard<std::mutex> lock(mutex_mppi_trajectory);
                last_optimal_trajectory.clear();
            }

            // Reset MPPI controller for next navigation
            mppi_controller.reset();
            mppi_hz.store(0);

            continue;  // Will sleep in idle_period at next iteration
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
        draw_lidar_points({}, {});
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

void SpecificWorker::slot_robot_dragging(QPointF pos)
{
    // Set flag to prevent localizer from overwriting pose
    robot_being_dragged.store(true);

    // Move robot to new position, keeping orientation
    estimated_robot_pose.translation() = Eigen::Vector2f{static_cast<float>(pos.x()), static_cast<float>(pos.y())};
    // Update visual representation
    viewer->robot_poly()->setPos(pos.x(), pos.y());
    qDebug() << "[MOUSE] Dragging robot to:" << pos.x() << pos.y();
}

void SpecificWorker::slot_robot_drag_end(QPointF pos)
{
    // Final position after drag
    const float new_x = static_cast<float>(pos.x());
    const float new_y = static_cast<float>(pos.y());

    // Determine orientation: face toward target if path exists, otherwise keep current
    float new_theta;
    {
        std::lock_guard<std::mutex> lock(mutex_current_path);
        if (!current_path.empty())
        {
            // Orient toward the target (last point of path)
            const auto& target = current_path.back();
            const float dx = target.x() - new_x;
            const float dy = target.y() - new_y;
            new_theta = std::atan2(dy, dx);
        }
        else
        {
            // No target, keep current orientation
            new_theta = std::atan2(estimated_robot_pose.linear()(1,0),
                                   estimated_robot_pose.linear()(0,0));
        }
    }

    // Update estimated pose with new position and orientation
    estimated_robot_pose.translation() = Eigen::Vector2f{new_x, new_y};
    estimated_robot_pose.linear() = Eigen::Rotation2Df(new_theta).toRotationMatrix();

    // Update visual representation
    viewer->robot_poly()->setPos(pos.x(), pos.y());
    viewer->robot_poly()->setRotation(qRadiansToDegrees(new_theta));

    // Reset localizer at new position to fix this as the new pose
    if (params.USE_LOCALIZER)
    {
        Localizer::Pose2D new_pose{new_x, new_y, new_theta};
        localizer.resetGaussian(new_pose, 200.f, 0.1f);  // Tight distribution around new position

        // Also write the new pose to the buffer immediately
        const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        Eigen::Affine2f new_affine;
        new_affine.translation() = Eigen::Vector2f(new_x, new_y);
        new_affine.linear() = Eigen::Rotation2Df(new_theta).toRotationMatrix();
        buffer_estimated_pose.put<0>(std::move(new_affine), timestamp);

        // Reset velocity command to zero to avoid drift after drag
        {
            std::lock_guard<std::mutex> lock(mutex_velocity_command);
            last_velocity_command.vx = 0.f;
            last_velocity_command.vy = 0.f;
            last_velocity_command.omega = 0.f;
        }

        qInfo() << "[MOUSE] Robot drag ended at:" << new_x << new_y
                << "- Localizer reset with theta:" << qRadiansToDegrees(new_theta) << "deg";
    }
    else
    {
        qInfo() << "[MOUSE] Robot drag ended at:" << new_x << new_y;
    }

    // Clear dragging flag - allow localizer to update again
    robot_being_dragged.store(false);
}

void SpecificWorker::slot_robot_rotate(QPointF pos)
{
    // Get current robot position
    const float robot_x = estimated_robot_pose.translation().x();
    const float robot_y = estimated_robot_pose.translation().y();

    // Calculate angle from robot to cursor position
    const float dx = pos.x() - robot_x;
    const float dy = pos.y() - robot_y;
    const float new_theta = std::atan2(dy, dx);

    // Update robot orientation only, keep position
    estimated_robot_pose.translation() = Eigen::Vector2f{robot_x, robot_y};
    estimated_robot_pose.linear() = Eigen::Rotation2Df(new_theta).toRotationMatrix();
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
            // Interpolate points along the line for proper path following
            // Use spacing of ~200mm between points
            const float spacing = 200.0f;
            const float dist = (tgt - src).norm();
            const int num_points = std::max(2, static_cast<int>(std::ceil(dist / spacing)) + 1);

            std::vector<Eigen::Vector2f> interpolated_path;
            interpolated_path.reserve(num_points);

            for (int i = 0; i < num_points; ++i)
            {
                float t = static_cast<float>(i) / static_cast<float>(num_points - 1);
                interpolated_path.push_back(src + t * (tgt - src));
            }

            paths.push_back(interpolated_path);
            msg = "VLOS path (ESDF) - interpolated " + std::to_string(num_points) + " points";
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

        // Draw the first path in the UI (if we have a viewer)
        if (!paths.empty() && viewer)
        {
            // Use QMetaObject::invokeMethod to safely draw from any thread
            QMetaObject::invokeMethod(this, [this, path = paths[0]]() {
                draw_path(path, &viewer->scene);
            }, Qt::QueuedConnection);
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

        // Dilate obstacles to fill gaps in walls
        if (params.MAP_DILATION_RADIUS > 0)
        {
            qInfo() << "[MRPT Loader] Dilating obstacles by" << params.MAP_DILATION_RADIUS << "cells...";
            grid_esdf.dilate_obstacles(params.MAP_DILATION_RADIUS);
            grid_esdf.mark_visualization_dirty();
            grid_esdf.update_visualization(true);
        }

        // Report statistics
        qInfo() << "[MRPT Loader] SPARSE_ESDF Statistics:";
        qInfo() << "  - MRPT cells processed:" << cells_processed;
        qInfo() << "  - Grid cells added:" << cells_added;
        qInfo() << "  - Collisions (MRPT->same grid cell):" << cells_collided;
        qInfo() << "  - Final grid obstacles (after dilation):" << grid_esdf.num_obstacles();
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

void SpecificWorker::Gridder_cancelNavigation()
{
    qInfo() << "[API] cancelNavigation called";

    // Disable MPPI controller
    mppi_enabled.store(false);

    // Clear path and target
    {
        std::lock_guard<std::mutex> lock(mutex_current_path);
        current_path.clear();
    }
    current_target = Eigen::Vector2f::Zero();

    // Set state to IDLE
    nav_state.store(NavigationState::IDLE);

    // Send stop command to robot
    try {
        omnirobot_proxy->setSpeedBase(0.f, 0.f, 0.f);
    } catch (const Ice::Exception &e) {
        qWarning() << "[API] Failed to stop robot:" << e.what();
    }

    // Reset MPPI controller
    mppi_controller.reset();

    // Clear trajectory visualization
    {
        std::lock_guard<std::mutex> lock(mutex_mppi_trajectory);
        last_optimal_trajectory.clear();
    }

    // Update UI
    QMetaObject::invokeMethod(this, [this]() {
        pushButton_mppi->setChecked(false);
        draw_path({}, &viewer->scene, true);  // Clear path drawing
    }, Qt::QueuedConnection);

    qInfo() << "[API] Navigation cancelled, target cleared";
}

float SpecificWorker::Gridder_getDistanceToTarget()
{
    // Check if we have a target
    if (current_target.norm() < 1.f)  // Near origin means no target
        return 0.f;

    // Get current robot pose
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    const auto& [estimated] = buffer_estimated_pose.read(timestamp);
    if (!estimated.has_value())
        return -1.f;  // Unknown

    const auto& robot_pose = estimated.value();
    Eigen::Vector2f robot_pos(robot_pose.translation().x(), robot_pose.translation().y());

    // Euclidean distance to target
    return (current_target - robot_pos).norm();
}

float SpecificWorker::Gridder_getEstimatedTimeToTarget()
{
    float distance = Gridder_getDistanceToTarget();
    if (distance <= 0.f)
        return -1.f;  // Unknown or no target

    // Estimate based on typical average speed (considering acceleration/deceleration)
    // Using ~300 mm/s average (conservative estimate)
    constexpr float avg_speed_mm_s = 300.f;

    return distance / avg_speed_mm_s;  // Time in seconds
}

RoboCompGridder::NavigationState SpecificWorker::Gridder_getNavigationState()
{
    // Map internal NavigationState to RoboCompGridder::NavigationState
    auto internal_state = nav_state.load();

    switch (internal_state)
    {
        case NavigationState::IDLE:
            return RoboCompGridder::NavigationState::IDLE;
        case NavigationState::NAVIGATING:
            return RoboCompGridder::NavigationState::NAVIGATING;
        case NavigationState::GOAL_REACHED:
            return RoboCompGridder::NavigationState::REACHED;
        case NavigationState::BLOCKED:
            return RoboCompGridder::NavigationState::BLOCKED;
        default:
            return RoboCompGridder::NavigationState::ERROR;
    }
}

RoboCompGridder::NavigationStatus SpecificWorker::Gridder_getNavigationStatus()
{
    RoboCompGridder::NavigationStatus status;

    // State
    status.state = Gridder_getNavigationState();

    // Current target
    status.currentTarget.x = current_target.x();
    status.currentTarget.y = current_target.y();
    status.currentTarget.radius = 0.f;

    // Current position and orientation
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    const auto& [estimated] = buffer_estimated_pose.read(timestamp);
    if (estimated.has_value())
    {
        const auto& pose = estimated.value();
        status.currentPosition.x = pose.translation().x();
        status.currentPosition.y = pose.translation().y();
        status.currentPosition.radius = params.ROBOT_SEMI_WIDTH;
        status.currentOrientation = std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
    }
    else
    {
        status.currentPosition = {0.f, 0.f, 0.f};
        status.currentOrientation = 0.f;
    }

    // Distance and time to target
    status.distanceToTarget = Gridder_getDistanceToTarget();
    status.estimatedTime = Gridder_getEstimatedTimeToTarget();

    // Current speed (from MPPI output)
    const auto& [mppi_out] = buffer_mppi_output.read(timestamp);
    if (mppi_out.has_value() && mppi_out->valid)
    {
        status.currentSpeed = std::sqrt(mppi_out->vx * mppi_out->vx + mppi_out->vy * mppi_out->vy);
    }
    else
    {
        status.currentSpeed = 0.f;
    }

    // Path waypoints remaining
    {
        std::lock_guard<std::mutex> lock(mutex_current_path);
        status.pathWaypointsRemaining = static_cast<int>(current_path.size());
    }

    // Status message
    switch (status.state)
    {
        case RoboCompGridder::NavigationState::IDLE:
            status.statusMessage = "Idle - no target set";
            break;
        case RoboCompGridder::NavigationState::NAVIGATING:
            status.statusMessage = "Navigating to target";
            break;
        case RoboCompGridder::NavigationState::PAUSED:
            status.statusMessage = "Navigation paused";
            break;
        case RoboCompGridder::NavigationState::REACHED:
            status.statusMessage = "Target reached";
            break;
        case RoboCompGridder::NavigationState::BLOCKED:
            status.statusMessage = "Path blocked - replanning needed";
            break;
        case RoboCompGridder::NavigationState::ERROR:
            status.statusMessage = "Navigation error";
            break;
    }

    return status;
}

RoboCompGridder::TPoint SpecificWorker::Gridder_getTarget()
{
    RoboCompGridder::TPoint ret;
    ret.x = current_target.x();
    ret.y = current_target.y();
    ret.radius = 0.f;
    return ret;
}

bool SpecificWorker::Gridder_hasReachedTarget()
{
    return nav_state.load() == NavigationState::GOAL_REACHED;
}

bool SpecificWorker::Gridder_replanPath()
{
    qInfo() << "[API] replanPath called";

    // Check if we have a current target
    if (current_target.norm() < 1.f)  // Near origin means no target
    {
        qWarning() << "[API] No target set, cannot replan";
        return false;
    }

    // Get current robot pose
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    const auto& [estimated] = buffer_estimated_pose.read(timestamp);
    if (!estimated.has_value())
    {
        qWarning() << "[API] No robot pose available for replanning";
        return false;
    }

    const auto& robot_pose = estimated.value();
    Eigen::Vector2f source(robot_pose.translation().x(), robot_pose.translation().y());

    // Compute new path
    auto new_path = grid_esdf.compute_path(source, current_target, params.ROBOT_SEMI_WIDTH, params.SAFETY_FACTOR);

    if (new_path.empty())
    {
        qWarning() << "[API] Replanning failed, no path found";
        nav_state.store(NavigationState::BLOCKED);
        return false;
    }

    // Update path
    {
        std::lock_guard<std::mutex> lock(mutex_current_path);
        current_path = new_path;
    }

    // Reset MPPI for new path
    mppi_controller.reset();

    qInfo() << "[API] Path replanned with" << new_path.size() << "waypoints";

    // Draw new path in UI
    QMetaObject::invokeMethod(this, [this, new_path]() {
        draw_path(new_path, &viewer->scene);
    }, Qt::QueuedConnection);

    return true;
}

bool SpecificWorker::Gridder_resumeNavigation()
{
    qInfo() << "[API] resumeNavigation called";

    // Check if we have a target to resume to
    {
        std::lock_guard<std::mutex> lock(mutex_current_path);
        if (current_path.empty())
        {
            qWarning() << "[API] No path to resume, call setTarget first";
            return false;
        }
    }

    // Check current state
    auto state = nav_state.load();
    if (state == NavigationState::NAVIGATING)
    {
        qInfo() << "[API] Already navigating";
        return true;
    }

    if (state == NavigationState::GOAL_REACHED)
    {
        qInfo() << "[API] Goal already reached, set new target";
        return false;
    }

    // Resume navigation
    mppi_enabled.store(true);
    nav_state.store(NavigationState::NAVIGATING);

    // Update UI
    QMetaObject::invokeMethod(this, [this]() {
        pushButton_mppi->setChecked(true);
    }, Qt::QueuedConnection);

    qInfo() << "[API] Navigation resumed";
    return true;
}

bool SpecificWorker::Gridder_setTarget(RoboCompGridder::TPoint target)
{
    qInfo() << "[API] setTarget called:" << target.x << target.y;

    // Check if target is valid (not blocked)
    Eigen::Vector2f target_eigen(target.x, target.y);
    auto target_key = grid_esdf.point_to_key(target_eigen);
    if (grid_esdf.is_occupied_for_planning(target_key, params.ROBOT_SEMI_WIDTH))
    {
        qWarning() << "[API] Target is blocked, searching for closest free point";
        // Use existing Gridder_getClosestFreePoint
        RoboCompGridder::TPoint closest_pt;
        closest_pt.x = target.x;
        closest_pt.y = target.y;
        closest_pt.radius = 0.f;
        auto free_pt = Gridder_getClosestFreePoint(closest_pt);
        if (free_pt.x == 0.f && free_pt.y == 0.f)
        {
            qWarning() << "[API] No free point found near target";
            nav_state.store(NavigationState::BLOCKED);
            return false;
        }
        target_eigen = Eigen::Vector2f(free_pt.x, free_pt.y);
        qInfo() << "[API] Using closest free point:" << target_eigen.x() << target_eigen.y();
    }

    // Get current robot pose
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    const auto& [estimated] = buffer_estimated_pose.read(timestamp);
    if (!estimated.has_value())
    {
        qWarning() << "[API] No robot pose available";
        return false;
    }

    const auto& robot_pose = estimated.value();
    Eigen::Vector2f source(robot_pose.translation().x(), robot_pose.translation().y());

    // Compute path
    auto path = grid_esdf.compute_path(source, target_eigen, params.ROBOT_SEMI_WIDTH, params.SAFETY_FACTOR);

    if (path.empty())
    {
        qWarning() << "[API] No path found to target";
        nav_state.store(NavigationState::BLOCKED);
        return false;
    }

    // Update navigation state
    {
        std::lock_guard<std::mutex> lock(mutex_current_path);
        current_path = path;
    }
    current_target = target_eigen;
    nav_state.store(NavigationState::NAVIGATING);
    mppi_controller.reset();

    // Enable MPPI if not already enabled
    if (!mppi_enabled.load())
    {
        mppi_enabled.store(true);
        pushButton_mppi->setChecked(true);
    }

    qInfo() << "[API] Navigation started to" << target_eigen.x() << target_eigen.y()
            << "with" << path.size() << "waypoints";

    // Draw path in UI
    QMetaObject::invokeMethod(this, [this, path]() {
        draw_path(path, &viewer->scene);
    }, Qt::QueuedConnection);

    return true;
}

bool SpecificWorker::Gridder_setTargetWithOptions(RoboCompGridder::TPoint target, RoboCompGridder::NavigationOptions options)
{
    qInfo() << "[API] setTargetWithOptions called:" << target.x << target.y
            << "maxSpeed:" << options.maxSpeed << "safetyFactor:" << options.safetyFactor;

    // Apply options to MPPI controller if needed
    if (options.maxSpeed > 0)
    {
        // Could adjust MPPI max speed here if controller supports it
        qInfo() << "[API] Max speed option:" << options.maxSpeed << "(not yet implemented in MPPI)";
    }

    // Store safety factor for path planning
    if (options.safetyFactor >= 0.f && options.safetyFactor <= 1.f)
    {
        params.SAFETY_FACTOR = options.safetyFactor;
    }

    // Use standard setTarget implementation
    return Gridder_setTarget(target);
}

bool SpecificWorker::Gridder_startNavigation()
{
    qInfo() << "[API] startNavigation called";

    // Check if we have a target
    std::lock_guard<std::mutex> lock(mutex_current_path);
    if (current_path.empty())
    {
        qWarning() << "[API] No target set, cannot start navigation";
        return false;
    }

    // Check current state
    auto state = nav_state.load();
    if (state == NavigationState::NAVIGATING)
    {
        qInfo() << "[API] Already navigating";
        return true;
    }

    // Enable MPPI controller
    mppi_enabled.store(true);
    nav_state.store(NavigationState::NAVIGATING);

    // Update UI button state
    QMetaObject::invokeMethod(this, [this]() {
        pushButton_mppi->setChecked(true);
    }, Qt::QueuedConnection);

    qInfo() << "[API] Navigation started";
    return true;
}

void SpecificWorker::Gridder_stopNavigation()
{
    qInfo() << "[API] stopNavigation called";

    // Disable MPPI controller (robot will stop)
    mppi_enabled.store(false);

    // Set state to PAUSED (keep target for resume)
    auto current_state = nav_state.load();
    if (current_state == NavigationState::NAVIGATING)
    {
        nav_state.store(NavigationState::IDLE);  // Using IDLE as PAUSED equivalent
    }

    // Send stop command to robot
    try {
        omnirobot_proxy->setSpeedBase(0.f, 0.f, 0.f);
    } catch (const Ice::Exception &e) {
        qWarning() << "[API] Failed to stop robot:" << e.what();
    }

    // Update UI button state
    QMetaObject::invokeMethod(this, [this]() {
        pushButton_mppi->setChecked(false);
    }, Qt::QueuedConnection);

    qInfo() << "[API] Navigation stopped (target preserved)";
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

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d1_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d1_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d1_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d1_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d1_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d1_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompWebots2Robocomp you can call this methods:
// RoboCompWebots2Robocomp::ObjectPose this->webots2robocomp_proxy->getObjectPose(string DEF)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->resetWebots()
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setDoorAngle(float angle)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setPathToHuman(int humanId, RoboCompGridder::TPath path)

/**************************************/
// From the RoboCompWebots2Robocomp you can use this types:
// RoboCompWebots2Robocomp::Vector3
// RoboCompWebots2Robocomp::Quaternion
// RoboCompWebots2Robocomp::ObjectPose
/**************************************/
// From the RoboCompGridder you can use this types:
// RoboCompGridder::TPoint
// RoboCompGridder::TDimensions
// RoboCompGridder::Result
// RoboCompGridder::TCell
// RoboCompGridder::Map
// RoboCompGridder::Pose
// RoboCompGridder::NavigationOptions
// RoboCompGridder::NavigationStatus
