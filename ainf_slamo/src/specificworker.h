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
		bool USE_WEBOTS = true;  // When false, disables all Webots connections (for real robot)
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
		int MAX_LIDAR_DRAW_POINTS = 500;
	};
	Params params;

	// Timer
	rc::Timer<> clock;
	FPSCounter fps;
	int hz = 0;

	// Sync Buffer: robot pose (GT, for debug/stats), lidar local points
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
	std::vector<Eigen::Vector2f> room_polygon_original_;  // Original polygon before any flips
	std::vector<Eigen::Vector2f> room_polygon_backup_;  // Backup of polygon during capture
	QGraphicsPolygonItem* polygon_item_backup_ = nullptr;  // Backup of polygon graphic item
	bool capturing_room_polygon = false;
	std::vector<QGraphicsEllipseItem*> polygon_vertex_items;
	QGraphicsPolygonItem* polygon_item = nullptr;

	// Extra polygons loaded from SVG (all closed paths, with their SVG color)
	struct SvgPolygon {
		std::vector<Eigen::Vector2f> vertices;
		QColor color;
		QString id;
	};
	std::vector<SvgPolygon> svg_extra_polygons_;
	std::vector<QGraphicsPolygonItem*> svg_extra_polygon_items_;

	// Flip state tracking
	bool flip_x_applied_ = false;
	bool flip_y_applied_ = false;

	// velocity commands. boost buffer is thread safe
	boost::circular_buffer<rc::VelocityCommand> velocity_history_{20};
    rc::VelocityCommand compute_current_odometry(long lidar_timestamp_ms, float delay = 0.0f);

	// Active inference room concept
	rc::RoomConceptAI room_ai;

	// CPU usage tracking
	float get_cpu_usage();
	clock_t last_cpu_time_ = 0;
	clock_t last_sys_cpu_time_ = 0;
	clock_t last_user_cpu_time_ = 0;
	int num_processors_ = 1;

	// Covariance ellipse visualization
	QGraphicsEllipseItem* cov_ellipse_item_ = nullptr;

	// Draw
	void draw_lidar_points(const std::vector<Eigen::Vector3f> &points, const Eigen::Affine2f &robot_pose);
	void draw_estimated_room(const Eigen::Matrix<float, 5, 1> &state);
	void display_robot(const Eigen::Affine2f &robot_pose, const Eigen::Matrix3f &covariance);
	void display_gt_error(const Eigen::Affine2f &estimated_pose, const std::optional<Eigen::Affine2f> &gt_pose_opt);
	float yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat);
	float estimate_orientation_from_points(const std::vector<Eigen::Vector3f> &pts) const;
	void update_ui(const rc::RoomConceptAI::UpdateResult &res, const rc::VelocityCommand &current_velocity, int fps_val);

	// Room polygon capture
	void draw_room_polygon();
	void on_scene_clicked(QPointF pos);

	// Layout save/load
	void save_layout_to_file(const std::string& filename);
	void save_layout_to_svg(const std::string& filename);
	void load_layout_from_file(const std::string& filename, const std::string& gt_polygon_id = "");
	void load_polygon_from_file(const std::string& filename, const std::string& gt_polygon_id = "");  // Only loads vertices, doesn't init room_ai
	void load_polygon_from_svg(const QString& svg_content, const std::string& gt_polygon_id = "");    // Parse SVG path/polygon data
	// Returns list of closed polygon ids found in an SVG file (for UI selection)
	std::vector<std::string> get_svg_polygon_ids(const std::string& filename);

	// Pose persistence (for fast restart)
	void save_last_pose();
	bool load_last_pose();  // Returns true if pose was loaded successfully
	void perform_grid_search(const std::vector<Eigen::Vector3f>& lidar_points);
	static constexpr const char* LAST_POSE_FILE = "./last_pose.json";

	// GT calibration: offset from Webots frame to map frame
	Eigen::Affine2f gt_offset_ = Eigen::Affine2f::Identity();
	bool gt_calibrated_ = false;
	void calibrate_gt_offset(const Eigen::Affine2f &estimated_pose, const Eigen::Affine2f &webots_pose);

private slots:
	void slot_capture_room_toggled(bool checked);
	void slot_save_layout();
	void slot_load_layout();
	void slot_flip_x();
	void slot_flip_y();
	void slot_robot_dragging(QPointF pos);
	void slot_robot_drag_end(QPointF pos);
	void slot_robot_rotate(QPointF pos);
	void slot_calibrate_gt();

};

#endif






