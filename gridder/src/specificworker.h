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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "grid_esdf.h"
#include "mrpt_map_loader.h"
#include "localizer.h"
#include "mppi_controller.h"
#include "doublebuffer_sync/doublebuffer_sync.h"
#include <Eigen/Eigen>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "Gridder.h"
#include <fps/fps.h>
#include <timer/timer.h>
#include <sys/resource.h>  // For CPU usage


/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
	    /**
	     * \brief Constructor for SpecificWorker.
	     * \param configLoader Configuration loader for the component.
	     * \param tprx Tuple of proxies required for the component.
	     * \param startup_check Indicates whether to perform startup checks.
	     */
		SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

		/**
	     * \brief Destructor for SpecificWorker.
	     */
		~SpecificWorker();

	    /**
         * @brief Checks if the given path is blocked by any obstacle in the grid
         * @param path
         * @return
         */
        bool Gridder_IsPathBlocked(RoboCompGridder::TPath path);

    /**
         * @brief Checks if there is a line of sight from source to target point without any obstacle in the grid, considering the robot radius
         * @param source
         * @param target
         * @param robotRadius
         * @return
         */
        bool Gridder_LineOfSightToTarget(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, float robotRadius);

    /**
         * @brief Finds the closest free point in the grid to the given source point, if the source point is blocked by an obstacle
         * @param source
         * @return
         */
        RoboCompGridder::TPoint Gridder_getClosestFreePoint(RoboCompGridder::TPoint source);

    /**
         * @brief Returns the dimensions of the grid
         * @return
         */
        RoboCompGridder::TDimensions Gridder_getDimensions();

        /**
         * @brief Computes multiple paths from source to target point
         * @param source Starting point coordinates
         * @param target Destination point coordinates
         * @param max_paths Maximum number of alternative paths to compute
         * @param tryClosestFreePoint If true, finds closest free point when target is blocked
         * @param targetIsHuman Whether the target is a human (affects pathfinding behavior)
         * @param safetyFactor 0.0=shortest path, 1.0=safest path (prefer center of free space)
         * @return Result containing computed paths and status information
         */
        RoboCompGridder::Result Gridder_getPaths(RoboCompGridder::TPoint source,
                                                 RoboCompGridder::TPoint target,
                                                 int max_paths,
                                                 bool tryClosestFreePoint,
                                                 bool targetIsHuman,
                                                 float safetyFactor);

    /**
     * @brief Returns the current map with obstacle cells and their costs
     * @return Map structure containing cells with cost > 0
     */
        RoboCompGridder::Map Gridder_getMap();

    /**
	 * @brief Loads an MRPT gridmap file and populates the internal grid
	 * @param filepath Path to the .gridmap file
	 * @return true if successful, false otherwise
	 */
	bool Gridder_loadMRPTMap(const std::string &filepath);

    /**
	 * @brief Clears the current grid and loads a new map from MRPT format
	 * Automatically detects SPARSE_ESDF vs DENSE grid mode
	 * @param filepath Path to the .gridmap file
	 * @return Status message describing result
	 */
	std::string Gridder_loadAndInitializeMap(const std::string &filepath);

    /**
		 * @brief Sets the dimensions of the grid
		 * @param dimensions
		 * @return
		 */
		bool Gridder_setGridDimensions(RoboCompGridder::TDimensions dimensions);

		/**
		 * @brieg Sets the location of the robot and the target, updates the grid with the free and obstacle points, computes paths from source to target and returns them
		 * @param source
		 * @param target
		 * @param freePoints
		 * @param obstaclePoints
		 * @return
		 */
		RoboCompGridder::Result Gridder_setLocationAndGetPath(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, RoboCompGridder::TPointVector freePoints, RoboCompGridder::TPointVector obstaclePoints);


	/**
		 * @ brief Returns the current pose of the robot (position and orientation with covariance if available)
		 * @return
		 */
		RoboCompGridder::Pose Gridder_getPose();

		RoboCompGridder::TPoint Gridder_getTarget();
		bool Gridder_hasReachedTarget();
		bool Gridder_replanPath();
		bool Gridder_resumeNavigation();
		bool Gridder_setTarget(RoboCompGridder::TPoint target);
		bool Gridder_setTargetWithOptions(RoboCompGridder::TPoint target, RoboCompGridder::NavigationOptions options);
		bool Gridder_startNavigation();
		void Gridder_stopNavigation();
		RoboCompGridder::NavigationState Gridder_getNavigationState();
		RoboCompGridder::NavigationStatus Gridder_getNavigationStatus();
		void Gridder_cancelNavigation();
		float Gridder_getEstimatedTimeToTarget();
		float Gridder_getDistanceToTarget();
		/////////////////// END of Gridder.idsl methods ///////////////////

		/**
		 * \brief Initializes the worker one time.
		 */
		void initialize();

		/**
		 * \brief Main compute loop of the worker.
		 */
		void compute();

		/**
		 * \brief Handles the emergency state loop.
		 */
		void emergency();

		/**
		 * \brief Restores the component from an emergency state.
		 */
		void restore();

	    /**
	     * \brief Performs startup checks for the component.
	     * \return An integer representing the result of the checks.
	     */
		int startup_check();

	private:
		bool startup_check_flag;
		bool cancel_from_mouse = false;     // cancel current target from mouse right click
		bool external_map_loaded = false;   // true if an external map was loaded (disables dynamic updates)

	    //Graphics
	    AbstractGraphicViewer *viewer;

		//Robot
		Eigen::Affine2f robot_pose = Eigen::Affine2f::Identity();

	    // Robot kinematic model enum (defined outside struct for easier access)
	    enum class RobotType { OMNIDIRECTIONAL, DIFFERENTIAL };

	    struct Params
	    {
	        // Robot dimensions (mm) - loaded from config
	        float ROBOT_WIDTH = 460;
	        float ROBOT_LENGTH = 480;
	        float ROBOT_SEMI_WIDTH = ROBOT_WIDTH / 2.f;
	        float ROBOT_SEMI_LENGTH = ROBOT_LENGTH / 2.f;
	        float MIN_DISTANCE_TO_TARGET = ROBOT_WIDTH / 2.f;
	    	float ROBOT_INITIAL_X = 10694;
	    	float ROBOT_INITIAL_Y = -4101;

	        // Grid configuration - loaded from config
	        float TILE_SIZE = 100;
	        QRectF GRID_MAX_DIM{-500, -500, 1000, 1000};

	        // LiDAR configuration - loaded from config
	        std::string LIDAR_NAME_LOW = "bpearl";
	        std::string LIDAR_NAME_HIGH = "helios";
	        float MAX_LIDAR_LOW_RANGE = 100000;
	        float MAX_LIDAR_HIGH_RANGE = 100000;
	        float MAX_LIDAR_RANGE = MAX_LIDAR_LOW_RANGE;
	        int LIDAR_LOW_DECIMATION_FACTOR = 1;
	        int LIDAR_HIGH_DECIMATION_FACTOR = 1;

	        // Display settings - loaded from config
	        bool DISPLAY = true;
	        bool DRAW_LIDAR_POINTS = false;
	        int MAX_LIDAR_DRAW_POINTS = 1500;

	        // Timing - loaded from config
	        long PERIOD_HYSTERESIS = 2;
	        int PERIOD = 100;
	        unsigned int ELAPSED_TIME_BETWEEN_PATH_UPDATES = 3000;

	        // Path planning - loaded from config
	        int NUM_PATHS_TO_SEARCH = 3;
	        float MIN_DISTANCE_BETWEEN_PATHS = 500;
	        float SAFETY_FACTOR = 1.0f;
	        size_t MAX_ASTAR_NODES = 100000;
	        float ASTAR_DISTANCE_FACTOR = 100.f;

	        // Localizer parameters - loaded from config [localizer] section
	        bool USE_LOCALIZER = true;
	        int LOCALIZER_PERIOD_MS = 50;

	        // Particle filter configuration
	        size_t LOCALIZER_MIN_PARTICLES = 100;
	        size_t LOCALIZER_MAX_PARTICLES = 1000;
	        size_t LOCALIZER_INITIAL_PARTICLES = 500;

	        // KLD-sampling parameters
	        float LOCALIZER_KLD_BIN_SIZE_XY = 200.f;
	        float LOCALIZER_KLD_BIN_SIZE_THETA = 0.1f;
	        float LOCALIZER_KLD_EPSILON = 0.02f;

	        // Motion model noise (alpha parameters) - defaults for WITHOUT ODOMETRY
	        float LOCALIZER_ALPHA1 = 0.25f;  // Rotation noise from rotation
	        float LOCALIZER_ALPHA2 = 0.10f;  // Rotation noise from translation
	        float LOCALIZER_ALPHA3 = 0.25f;  // Translation noise from translation
	        float LOCALIZER_ALPHA4 = 0.10f;  // Translation noise from rotation

	        // Minimum diffusion noise - defaults for WITHOUT ODOMETRY
	        float LOCALIZER_MIN_TRANS_DIFFUSION = 50.f;  // mm per cycle (~1000mm/s at 20Hz)
	        float LOCALIZER_MIN_ROT_DIFFUSION = 0.05f;   // rad per cycle (~3 degrees)

	        // Observation model
	        float LOCALIZER_SIGMA_HIT = 100.f;
	        float LOCALIZER_Z_HIT = 0.95f;
	        int LOCALIZER_LIDAR_SUBSAMPLE = 15;

	        // Resampling
	        float LOCALIZER_RESAMPLE_THRESHOLD = 0.5f;

	        // Convergence thresholds
	        float LOCALIZER_POSITION_STDDEV_THRESHOLD = 200.f;
	        float LOCALIZER_ANGLE_STDDEV_THRESHOLD = 0.1f;

	        // Ground Truth warmup (simulation only) - loaded from config
	        bool USE_GT_WARMUP = true;

	        // MRPT map alignment - loaded from config
	        float MRPT_MAP_OFFSET_X = 12000.0f;
	        float MRPT_MAP_OFFSET_Y = -2500.0f;
	        float MRPT_MAP_ROTATION = -M_PI_2;
	        bool MRPT_MAP_MIRROR_X = false;
	        int MAP_DILATION_RADIUS = 1;  // Dilate obstacles by N cells to fill gaps (0 = disabled)

	        // MPPI controller - loaded from config
	        int MPPI_PERIOD_MS = 50;

	        // Robot kinematic model - loaded from config
	        RobotType ROBOT_TYPE = RobotType::DIFFERENTIAL;
	    };
	    Params params;

	    // Config loader (stored for deferred parameter loading)
	    const ConfigLoader* configLoader = nullptr;

	    // Load parameters from ConfigLoader
	    void loadParams(const ConfigLoader& configLoader);

	    // Load MPPI parameters from ConfigLoader
	    MPPIController::Params loadMPPIParams();

	    // Timer
	    rc::Timer<> clock;

		// Sync Buffer: robot pose, world points, local points
		BufferSync<InOut<Eigen::Affine2f, Eigen::Affine2f>,
				   InOut<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>>,
				   InOut<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>>> buffer_sync;

	    // Lidar Thread
	    std::thread read_lidar_th;
	    std::atomic<bool> stop_lidar_thread{false};
	    void read_lidar();

	    // Localizer Thread
	    std::thread localizer_th;
	    std::atomic<bool> stop_localizer_thread{false};
	    void run_localizer();

	    // Double buffer for estimated robot pose (written by localizer thread, read by main thread)
	    BufferSync<InOut<Eigen::Affine2f, Eigen::Affine2f>> buffer_estimated_pose;

	    // Grid (Sparse ESDF - VoxBlox-style)
	    GridESDF grid_esdf;

	    // Localizer (AMCL)
	    Localizer localizer;
	    std::atomic<bool> localizer_initialized{false};
	    std::atomic<bool> map_ready_for_localization{false};  // Set after map is fully loaded

	    // Odometry from velocity commands (used by localizer)
	    struct VelocityCommand {
	        float vx = 0.f;   // mm/s
	        float vy = 0.f;   // mm/s
	        float omega = 0.f; // rad/s
	        std::chrono::steady_clock::time_point timestamp;
	    };
	    VelocityCommand last_velocity_command;
	    std::mutex mutex_velocity_command;

	    // FPS
	    FPSCounter fps;
	    int hz = 0;

	    // Draw
	    void draw_paths(const std::vector<std::vector<Eigen::Vector2f>> &paths, QGraphicsScene *scene, bool erase_only=false);
	    void draw_path(const std::vector<Eigen::Vector2f> &path, QGraphicsScene *scene, bool erase_only=false);
	    void draw_lidar_points(const std::vector<Eigen::Vector2f> &points_local, const Eigen::Affine2f &robot_pose);

	    // mutex
	    std::mutex mutex_path;

	    float yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat);

    // ==================== Robot Pose Estimation ====================
    Eigen::Affine2f estimated_robot_pose;  // Current best estimate of robot pose

    // Helper methods for compute() - keep main loop compact
    std::optional<Eigen::Affine2f> get_current_pose(const Eigen::Affine2f& gt_pose, std::uint64_t timestamp);
    void process_mppi_output(std::uint64_t timestamp);
    void update_ui_displays();

	    // ==================== MPPI Controller ====================
	    MPPIController mppi_controller;
	    std::atomic<bool> mppi_enabled{false};

	    // MPPI Thread
	    std::thread mppi_th;
	    std::atomic<bool> stop_mppi_thread{false};
	    void run_mppi();

	    // Double buffer for MPPI control output (written by MPPI thread, read by main thread)
	    // Stores (vx, vy, omega, valid) - valid indicates if the command is fresh
	    struct MPPIOutput {
	        float vx = 0.f;
	        float vy = 0.f;
	        float omega = 0.f;
	        bool valid = false;
	    };
	    BufferSync<InOut<MPPIOutput, MPPIOutput>> buffer_mppi_output;

	    // Navigation state
	    enum class NavigationState { IDLE, NAVIGATING, GOAL_REACHED, BLOCKED };
	    std::atomic<NavigationState> nav_state{NavigationState::IDLE};
	    std::vector<Eigen::Vector2f> current_path;
	    std::mutex mutex_current_path;  // Protects current_path access
	    Eigen::Vector2f current_target;

	    // MPPI visualization
	    std::vector<QGraphicsItem*> mppi_trajectory_items;
	    void draw_mppi_trajectory(const std::vector<MPPIController::State>& trajectory);
	    std::mutex mutex_mppi_trajectory;  // Protects trajectory visualization data
	    std::vector<MPPIController::State> last_optimal_trajectory;  // For visualization from main thread

	    // CPU usage tracking
	    float get_cpu_usage();
	    struct rusage last_usage;
	    std::chrono::steady_clock::time_point last_cpu_time;
	    float cpu_usage_avg = 0.0f;  // Exponential moving average
	    static constexpr float CPU_AVG_ALPHA = 0.1f;  // Smoothing factor (0.1 = slow, 0.5 = fast)

	    // Thread FPS tracking
	    std::atomic<int> localizer_hz{0};
	    std::atomic<int> mppi_hz{0};

	    // Visualization flags (controlled by UI checkboxes)
	    std::atomic<bool> show_lidar_points{false};
	    std::atomic<bool> show_particles{false};
	    std::atomic<bool> show_trajectories{true};
	    std::atomic<bool> show_covariance{false};

	    // Robot dragging state - prevents localizer from overwriting pose during drag
	    std::atomic<bool> robot_being_dragged{false};

	    // Covariance ellipse visualization
	    QGraphicsEllipseItem* covariance_ellipse = nullptr;
	    void draw_covariance_ellipse();

	    // UI slots
	    void slot_mppi_button_toggled(bool checked);
	    void slot_lidar_checkbox_toggled(bool checked);
	    void slot_particles_checkbox_toggled(bool checked);
	    void slot_trajectories_checkbox_toggled(bool checked);
	    void slot_covariance_checkbox_toggled(bool checked);

	private slots:
		void slot_robot_dragging(QPointF pos);
		void slot_robot_drag_end(QPointF pos);
		void slot_robot_rotate(QPointF pos);

};

#endif
