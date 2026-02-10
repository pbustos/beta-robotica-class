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
#include <grid2d/grid.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <Eigen/Geometry>

// MPPI Controller - descomentar para activar
// #include "mppi_controller.h"


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

		/**
		 * \brief Handles the emergency state loop.
		 */
		void emergency();

		/**
		 * \brief Restores the component from an emergency state.
		 */
		void restore();

	private:
		bool startup_check_flag;
		int startup_check();

		struct Params
		{
			float ROBOT_WIDTH = 460;  // mm
			float ROBOT_LENGTH = 480;  // mm
			float MAX_ADV_SPEED = 1000; // mm/s
			float MAX_ROT_SPEED = 1; // rad/s
			float MAX_SIDE_SPEED = 50; // mm/s
			float MAX_TRANSLATION = 500; // mm/s
			float MAX_ROTATION = 0.2;
			float STOP_THRESHOLD = 700; // mm
			float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
			float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
			// wall
			float LIDAR_RIGHT_SIDE_SECTION = M_PI/3; // rads, 90 degrees
			float LIDAR_LEFT_SIDE_SECTION = -M_PI/3; // rads, 90 degrees
			float WALL_MIN_DISTANCE = ROBOT_WIDTH*1.2;
			// match error correction
			float MATCH_ERROR_SIGMA = 150.f; // mm
			float DOOR_REACHED_DIST = 300.f;
			std::string LIDAR_NAME_LOW = "bpearl";
			std::string LIDAR_NAME_HIGH = "helios";
			QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

			// relocalization
			float RELOCAL_CENTER_EPS = 300.f;    // mm: stop when |mean| < eps
			float RELOCAL_KP = 0.002f;           // gain to convert mean (mm) -> speed (magnitude)
			float RELOCAL_MAX_ADV = 300.f;       // mm/s cap while re-centering
			float RELOCAL_MAX_SIDE = 300.f;      // mm/s cap while re-centering
			float RELOCAL_ROT_SPEED = 0.3f;     // rad/s while aligning
			float RELOCAL_DELTA = 5.0f * M_PI/180.f; // small probe angle in radians
			float RELOCAL_MATCH_MAX_DIST = 2000.f;   // mm for Hungarian gating
			float RELOCAL_DONE_COST = 500.f;
			float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;

		};
		Params params;

		// viewer
		AbstractGraphicViewer *viewer;
		QGraphicsPolygonItem *robot_draw;
		Eigen::Affine2f robot_pose;

		Eigen::Affine2f get_robot_pose();
		RoboCompGridder::Result current_path_result = {};
		bool has_target_ = false;

		void draw_lidar (const RoboCompLidar3D::TPoints &filtered_points, const Eigen::Affine2f &robot_pose, QGraphicsScene *scene);

		//Updates robot_pose_display with the new robot coordinates each iteration (modifies robot_pose_display class attribute)
		Eigen::Affine2f update_robot_transform(const RoboCompWebots2Robocomp::ObjectPose &pose, Eigen::Affine2f &robot_transform);

		RoboCompLidar3D::TPoints get_lidar();

		double yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat);

		void compute_mppi_control(const RoboCompLidar3D::TPoints &lidar_data);

		//Transforms param local_point to the room's coordinate system by multiplying with robot_pose (which already is at the room's coordinate system)
		//Eigen::Vector2f transform_to_world(const RoboCompLidar3D::TPoint &local_point);

		//Obtains the robot's rotation from the linear part of robot_pose_display
		//float obtain_rotation();

		// =====================================================================
		// MPPI Controller - descomentar para activar
		// =====================================================================
		// MPPIController mppi_controller_;
		// std::vector<Eigen::Vector2f> current_path_;  // Path from Gridder
		// Eigen::Vector2f current_target_;             // Current target position
		// bool has_target_ = false;                    // Flag to indicate if there is an active target
		//
		// // Helper methods for MPPI
		// std::vector<Eigen::Vector2f> lidar_to_obstacles(const RoboCompLidar3D::TPoints& points);
		// void draw_mppi_trajectory(const std::vector<MPPIController::State>& trajectory, QGraphicsScene* scene);
};

#endif
