/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
#include "rapplication/rapplication.h"
#include "grid_esdf.h"
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"

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


public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();
	void left_click_handler(QPointF p);

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
	        float MAX_LIDAR_LOW_RANGE = 100000;  // mm
	        float MAX_LIDAR_HIGH_RANGE = 100000;  // mm
	        float MAX_LIDAR_RANGE = MAX_LIDAR_LOW_RANGE;  // mm used in the grid
	        int LIDAR_LOW_DECIMATION_FACTOR = 1;
	        int LIDAR_HIGH_DECIMATION_FACTOR = 1;
	        QRectF GRID_MAX_DIM{-5000, -5000, 10000, 10000};
	        long PERIOD_HYSTERESIS = 2; // to avoid oscillations in the adjustment of the lidar thread period
	        int PERIOD = 100;    // ms (20 Hz) for compute timer
	        unsigned int ELAPSED_TIME_BETWEEN_PATH_UPDATES = 3000;
	        int NUM_PATHS_TO_SEARCH = 3;
	        float MIN_DISTANCE_BETWEEN_PATHS = 500; // mm
	        bool DRAW_LIDAR_POINTS = false;  // debug: draw LiDAR points (can impact performance)
	        int MAX_LIDAR_DRAW_POINTS = 1500; // debug: limit number of points drawn
			bool DISPLAY = true; // Whether to display the viewer (set false for headless operation)

	  		// Path planning safety factor: 0=shortest path (touch walls), 1=safest path (prefer center)
	        float SAFETY_FACTOR = 1.0f;	// 0=touch walls, 1=prefer center

	  		// Grid
	  		float MRPT_MAP_OFFSET_X = 0.f; // 12000.0f; //26100.7f;  // mm - X offset to apply to loaded map
	  		float MRPT_MAP_OFFSET_Y = 0.f; // -2500.0f;//5600.f;  // mm - Y offset to apply to loaded map
	  		float MRPT_MAP_ROTATION = 0.f; // -M_PI_2;   // radians - rotation to apply (90º left = PI/2)
	  		bool MRPT_MAP_MIRROR_X = false;       // Mirror X axis (negate X before rotation) if map appears flipped

	    };
	    Params params;

	//Graphics
	AbstractGraphicViewer *viewer;

	RoboCompGridder::Map map;

	// Grid (Sparse ESDF - VoxBlox-style)
	GridESDF grid_esdf;

	// Robot position variables
	float robot_x = 0.0f;
	float robot_y = 0.0f;
	float robot_theta = 0.0f;

	void update_ui();
	void initialize_grid();
	void draw_path(const std::vector<Eigen::Vector2f> &path, QGraphicsScene *scene, bool erase_only=false);

};

#endif
