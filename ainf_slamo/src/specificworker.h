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
#include <JoystickAdapter.h>
#include <Eigen/Eigen>
#include <fps/fps.h>
#include <timer/timer.h>
#include <sys/resource.h>  // For CPU usage
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "doublebuffer_sync/doublebuffer_sync.h"
#include "room_concept_ai.h"
#include "pointcloud_center_estimator.h"
#include <boost/circular_buffer.hpp>
#include "common_types.h"

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

		void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

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

	    /**
	     * \brief Performs startup checks for the component.
	     * \return An integer representing the result of the checks.
	     */
		int startup_check();

	private:
	    bool startup_check_flag;

	//Graphics
	AbstractGraphicViewer *viewer;

	//Robot
	Eigen::Affine2f robot_pose = Eigen::Affine2f::Identity();

	struct Params
	{
		float ROBOT_WIDTH = 0.460;  // m
		float ROBOT_LENGTH = 0.480;  // m
		float ROBOT_SEMI_WIDTH = ROBOT_WIDTH / 2.f;     // m
		float ROBOT_SEMI_LENGTH = ROBOT_LENGTH / 2.f;    // m
		float MIN_DISTANCE_TO_TARGET = ROBOT_WIDTH / 2.f; // m
		std::string LIDAR_NAME_HIGH = "helios";
		float MAX_LIDAR_HIGH_RANGE = 100;  // m
		int LIDAR_LOW_DECIMATION_FACTOR = 1;
		int LIDAR_HIGH_DECIMATION_FACTOR = 1;
		float LIDAR_HIGH_MIN_HEIGHT = 1.2; // m, points below this height in the high lidar will be ignored (e.g. to filter tables)
		float LIDAR_HIGH_MAX_HEIGHT = 1.4f; // m, points above this height in the high lidar will be ignored (e.g. to filter ceiling)
		QRectF GRID_MAX_DIM{-8, -5, 16, 10};
		int MAX_LIDAR_DRAW_POINTS = 1500;
	};
	Params params;

	// Timer
	rc::Timer<> clock;
	FPSCounter fps;
	int hz = 0;

	// Sync Buffer: robot pose, world points, local points
	BufferSync<InOut<Eigen::Affine2f, Eigen::Affine2f>,
			   InOut<std::pair<std::vector<Eigen::Vector3f>, std::int64_t>,
					 std::pair<std::vector<Eigen::Vector3f>, std::int64_t>>> buffer_sync;

	// Lidar Thread
	std::thread read_lidar_th;
	std::atomic<bool> stop_lidar_thread{false};
	void read_lidar();

	// GT Room
	QRectF room_rect_gt;
	std::vector<Eigen::Vector2f> room_polygon_gt;  // Polygon vertices for GT room (in room frame)
	bool capturing_room_polygon = false;
	std::vector<QGraphicsEllipseItem*> polygon_vertex_items;
	QGraphicsPolygonItem* polygon_item = nullptr;

	// velocity commands. boost buffer is thread safe
	boost::circular_buffer<rc::VelocityCommand> velocity_history_{20};
    rc::VelocityCommand compute_current_odometry(long lidar_timestamp_ms, float delay = 0.0f);

	// Active inference room concept
	rc::RoomConceptAI room_ai;

	// Draw
	void draw_lidar_points(const std::vector<Eigen::Vector3f> &points, const Eigen::Affine2f &robot_pose);
	void draw_estimated_room(const Eigen::Matrix<float, 5, 1> &state);
	void display_robot(const Eigen::Affine2f &robot_pose_gt);
	float yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat);
	float estimate_orientation_from_points(const std::vector<Eigen::Vector3f> &pts) const;
	void update_ui(const rc::RoomConceptAI::UpdateResult &res, const rc::VelocityCommand &current_velocity, int fps_val);

	// Room polygon capture
	void draw_room_polygon();
	void on_scene_clicked(QPointF pos);

private slots:
	void slot_capture_room_toggled(bool checked);

};

#endif




