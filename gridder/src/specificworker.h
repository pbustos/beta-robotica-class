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
#include "grid.h"
#include "grid_esdf.h"
#include "mrpt_map_loader.h"
#include "localizer.h"
#include "doublebuffer_sync/doublebuffer_sync.h"
#include <Eigen/Eigen>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "Gridder.h"
#include <fps/fps.h>
#include <timer/timer.h>


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

	    struct Params
	    {
	        float ROBOT_WIDTH = 460;  // mm
	        float ROBOT_LENGTH = 480;  // mm
	        float ROBOT_SEMI_WIDTH = ROBOT_WIDTH / 2.f;     // mm
	        float ROBOT_SEMI_LENGTH = ROBOT_LENGTH / 2.f;    // mm
	        float TILE_SIZE = 100;   // mm
	        float MIN_DISTANCE_TO_TARGET = ROBOT_WIDTH / 2.f; // mm
	        std::string LIDAR_NAME_LOW = "bpearl";
	        std::string LIDAR_NAME_HIGH = "helios";
	        float MAX_LIDAR_LOW_RANGE = 15000;  // mm
	        float MAX_LIDAR_HIGH_RANGE = 15000;  // mm
	        float MAX_LIDAR_RANGE = MAX_LIDAR_LOW_RANGE;  // mm used in the grid
	        int LIDAR_LOW_DECIMATION_FACTOR = 1;
	        int LIDAR_HIGH_DECIMATION_FACTOR = 1;
	        QRectF GRID_MAX_DIM{-500, -500, 1000, 1000};
	        long PERIOD_HYSTERESIS = 2; // to avoid oscillations in the adjustment of the lidar thread period
	        int PERIOD = 100;    // ms (20 Hz) for compute timer
	        unsigned int ELAPSED_TIME_BETWEEN_PATH_UPDATES = 3000;
	        int NUM_PATHS_TO_SEARCH = 3;
	        float MIN_DISTANCE_BETWEEN_PATHS = 500; // mm
	        bool DISPLAY = true ; //TODO: config file
	        bool DRAW_LIDAR_POINTS = false;  // debug: draw LiDAR points (can impact performance)
	        int MAX_LIDAR_DRAW_POINTS = 1500; // debug: limit number of points drawn

	    	// Grid mode selection
	        enum class GridMode { DENSE, DENSE_ESDF, SPARSE_ESDF };
	        GridMode GRID_MODE = GridMode::SPARSE_ESDF;  // Options:
            // DENSE: original ray casting (accurate, slow for large maps)
            // DENSE_ESDF: dense grid with ESDF optimization
            // SPARSE_ESDF: sparse grid, only obstacles stored (fastest, most memory efficient)
	        bool USE_ESDF_MODE = false;  // kept for backward compatibility
	        // Path planning safety factor: 0=shortest path (touch walls), 1=safest path (prefer center)
	        float SAFETY_FACTOR = 1.0f;	// 0=touch walls, 1=prefer center
	        size_t MAX_ASTAR_NODES = 100000;  // Maximum nodes to expand in A* before giving up
	        float ASTAR_DISTANCE_FACTOR = 100.f;  // Multiply path distance in cells by this factor for max nodes

	        // MRPT map offset to align with Webots world coordinates (in mm)
	        //float MRPT_MAP_OFFSET_X = 26100.7f;  // mm - X offset to apply to loaded map
	        //float MRPT_MAP_OFFSET_Y = 5600.f;  // mm - Y offset to apply to loaded map
	        //float MRPT_MAP_ROTATION = M_PI_2;   // radians - rotation to apply (90º left = PI/2)

	        // Localizer parameters
	        bool USE_LOCALIZER = true;  // Enable/disable AMCL localization
	        size_t LOCALIZER_PARTICLES = 500;  // Number of particles
	        float LOCALIZER_ODOM_NOISE = 0.1f;  // Noise factor for simulated odometry
	        float MRPT_MAP_OFFSET_X = 12000.0f; //26100.7f;  // mm - X offset to apply to loaded map
	        float MRPT_MAP_OFFSET_Y = -2500.0f;//5600.f;  // mm - Y offset to apply to loaded map
	        float MRPT_MAP_ROTATION =  -M_PI_2;   // radians - rotation to apply (90º left = PI/2)
	        bool MRPT_MAP_MIRROR_X = true;       // Mirror X axis (negate X before rotation) if map appears flipped
	    };
	    Params params;

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

	    // Grids (two implementations available)
	    Grid grid;              // Dense grid (original)
	    GridESDF grid_esdf;     // Sparse ESDF grid (VoxBlox-style)

	    // Localizer (AMCL)
	    Localizer localizer;
	    Localizer::Pose2D last_ground_truth_pose;  // For simulating odometry
	    std::atomic<bool> localizer_initialized{false};
	    std::atomic<bool> map_ready_for_localization{false};  // Set after map is fully loaded
	    std::optional<Localizer::Pose2D> update_localizer(const Eigen::Affine2f &robot_pos, const std::vector<Eigen::Vector2f> &points_local);

	    // FPS
	    FPSCounter fps;
	    int hz = 0;

	    // Draw
	    void draw_paths(const std::vector<std::vector<Eigen::Vector2f>> &paths, QGraphicsScene *scene, bool erase_only=false);
	    void draw_path(const std::vector<Eigen::Vector2f> &path, QGraphicsScene *scene, bool erase_only=false);
	    void draw_lidar_points(const std::vector<Eigen::Vector2f> &points_world);

	    // Clustering for obstacle detection
	    struct Cluster
	    {
	        std::vector<Eigen::Vector2f> points;
	        Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
	        float min_dist_to_robot = 0.f;
	        std::vector<Eigen::Vector2f> convex_hull;  // optional convex hull
	    };
	    std::vector<Cluster> cluster_lidar_points(const std::vector<Eigen::Vector2f> &points,
	                                              const Eigen::Vector2f &robot_pos,
	                                              float distance_threshold = 300.f,
	                                              int min_points = 3);
	    std::vector<Eigen::Vector2f> compute_convex_hull(const std::vector<Eigen::Vector2f> &points);
	    void draw_clusters(const std::vector<Cluster> &clusters, QGraphicsScene *scene, bool erase_only = false);

	    // mutex
	    std::mutex mutex_path;

		Eigen::Affine2f get_robot_pose();

	float yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat);

};

#endif
