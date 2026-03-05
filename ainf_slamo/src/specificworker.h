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
#include "polygon_path_planner.h"
#include "trajectory_controller.h"
#include <variant>
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

		void FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose);
		void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);
		RoboCompNavigator::LayoutData Navigator_getLayout();
		RoboCompNavigator::Result Navigator_getPath(RoboCompNavigator::TPoint source, RoboCompNavigator::TPoint target, float safety);
		RoboCompNavigator::TPose Navigator_getRobotPose();
		RoboCompNavigator::NavigationStatus Navigator_getStatus();
		RoboCompNavigator::TPoint Navigator_gotoObject(std::string object);
		RoboCompNavigator::TPoint Navigator_gotoPoint(RoboCompNavigator::TPoint target);
		void Navigator_resume();
		void Navigator_stop();

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

	// Graphics
	AbstractGraphicViewer *viewer;


	struct Params
	{
		bool USE_WEBOTS = true;  // When false, disables all Webots connections (for real robot)
		float ROBOT_WIDTH = 0.460;  // m
		float ROBOT_LENGTH = 0.480;  // m
		float ROBOT_SEMI_WIDTH = ROBOT_WIDTH / 2.f;     // m
		float ROBOT_SEMI_LENGTH = ROBOT_LENGTH / 2.f;    // m
		// High LiDAR (HELIOS) — localization + MPPI
		std::string LIDAR_NAME_HIGH = "helios";
		float MAX_LIDAR_HIGH_RANGE = 100;  // m
		int LIDAR_LOW_DECIMATION_FACTOR = 1;
		float LIDAR_HIGH_MIN_HEIGHT = 1.2; // m, points below this height in the high lidar will be ignored
		float LIDAR_HIGH_MAX_HEIGHT = 2.2f; // m, points above this height in the high lidar will be ignored
		// Low LiDAR (BPEARL) — MPPI only (obstacles: chairs, tables, people)
		std::string LIDAR_NAME_LOW = "bpearl";
		float MAX_LIDAR_LOW_RANGE = 100;    // m (short range sensor)
		float LIDAR_LOW_MIN_HEIGHT = 0.1f; // m, filter ground
		int LIDAR_LOW_DECIMATION_FACTOR_LOW = 1;
		// General
		QRectF GRID_MAX_DIM{-8, -5, 16, 10};
		int MAX_LIDAR_DRAW_POINTS = 500;
		float ODOMETRY_NOISE_FACTOR = 0.1f;  // Gaussian noise std added to odometry (fraction of reading)
	};
	Params params;

	// Timer
	rc::Timer<> clock;
	FPSCounter fps;

	// Type alias for lidar data in buffer
	using LidarData = std::pair<std::vector<Eigen::Vector3f>, std::int64_t>;

	// Sync Buffer: slot 0 = GT pose, slot 1 = HELIOS high lidar, slot 2 = BPEARL low lidar
	BufferSync<InOut<Eigen::Affine2f, Eigen::Affine2f>,
			   InOut<LidarData, LidarData>,
			   InOut<LidarData, LidarData>> buffer_sync;

	// Lidar Thread
	std::thread read_lidar_th;
	std::atomic<bool> stop_lidar_thread{false};
	void read_lidar();

	// Room model
	std::vector<Eigen::Vector2f> room_polygon_;  // Room model polygon vertices (loaded from SVG or captured)
	std::vector<Eigen::Vector2f> room_polygon_backup_;  // Backup of polygon during capture
	QGraphicsPolygonItem* polygon_item_backup_ = nullptr;  // Backup of polygon graphic item
	bool capturing_room_polygon = false;
	std::vector<QGraphicsEllipseItem*> polygon_vertex_items;
	QGraphicsPolygonItem* polygon_item = nullptr;

	// Furniture / obstacles (loaded from SVG "Furniture" layer)
	struct FurniturePolygon
	{
		std::string id;                          // SVG id
		std::string label;                       // inkscape:label
		std::vector<Eigen::Vector2f> vertices;   // polygon vertices in room frame
	};
	std::vector<FurniturePolygon> furniture_polygons_;
	std::vector<QGraphicsPolygonItem*> furniture_draw_items_;
	void draw_furniture();

	// Flip state tracking
	bool flip_x_applied_ = false;
	bool flip_y_applied_ = false;
	bool auto_center_ = false;  // Auto-center view on robot
	bool draw_lidar_ = true;    // Toggle lidar point drawing
	bool draw_trajectories_ = true;  // Toggle MPPI trajectory drawing

	// Velocity commands (thread-safe via BufferSync)
	BufferSync<InOut<rc::VelocityCommand, rc::VelocityCommand>> velocity_buffer_{20};

	// Measured odometry readings from FullPoseEstimationPub (thread-safe via BufferSync)
	BufferSync<InOut<rc::OdometryReading, rc::OdometryReading>> odometry_buffer_{20};

	// ===== Localization Thread =====
	// room_ai runs in its own thread, publishing UpdateResult to loc_result_
	std::thread localization_th_;
	std::atomic<bool> stop_localization_thread_{false};
	void run_localization();

	// Localization output: latest UpdateResult (mutex-guarded, SPSC)
	mutable std::mutex loc_result_mutex_;
	std::optional<rc::RoomConceptAI::UpdateResult> loc_result_;
	std::atomic<bool> loc_initialized_{false};  // true once room_ai has produced its first result

	// Command queue: UI slots push commands, localization thread drains them
	struct LocCmdSetPolygon { std::vector<Eigen::Vector2f> vertices; };
	struct LocCmdSetPose    { float x; float y; float theta; };
	struct LocCmdGridSearch { std::vector<Eigen::Vector3f> lidar_points; float grid_res; float angle_res; };
	using LocCommand = std::variant<LocCmdSetPolygon, LocCmdSetPose, LocCmdGridSearch>;
	std::mutex loc_cmd_mutex_;
	std::vector<LocCommand> loc_pending_commands_;
	void push_loc_command(LocCommand cmd);

	/// Thread-safe read of latest localization state [half_w, half_h, x, y, theta]
	/// Returns zeros if not yet initialized
	Eigen::Matrix<float,5,1> get_loc_state() const
	{
		std::lock_guard lock(loc_result_mutex_);
		if (loc_result_.has_value() && loc_result_->ok)
			return loc_result_->state;
		return Eigen::Matrix<float,5,1>::Zero();
	}

	// Active inference room concept (owned by localization thread after init)
	rc::RoomConceptAI room_ai;

	// Path planner
	rc::PolygonPathPlanner path_planner_;
	std::vector<Eigen::Vector2f> current_path_;
	std::vector<QGraphicsItem*> path_draw_items_;
	QGraphicsEllipseItem* target_marker_ = nullptr;
	QGraphicsPolygonItem* navigable_poly_item_ = nullptr;  // debug: shrunken polygon
	std::vector<QGraphicsPolygonItem*> obstacle_expanded_items_;  // debug: expanded obstacle boundaries
	void draw_path_threadsafe(const std::vector<Eigen::Vector2f>& path);
	void draw_path(const std::vector<Eigen::Vector2f>& path);
	void clear_path(bool stop_controller = true, bool clear_stored_path = true);

	// Trajectory controller (ESDF-based sampling local control)
	rc::TrajectoryController trajectory_controller_;
	bool low_speed_block_timer_active_ = false;
	std::chrono::steady_clock::time_point low_speed_block_start_;
	static constexpr float BLOCKED_SPEED_THRESHOLD = 0.03f;     // m/s
	static constexpr float BLOCKED_TIME_THRESHOLD_SEC = 2.5f;   // s
	float last_ess_ = 0.f;   // Latest ESS value for UI
	int   last_ess_K_ = 1;   // Latest K for ESS ratio
	std::chrono::steady_clock::time_point last_joystick_time_;
	static constexpr float JOYSTICK_TIMEOUT_SEC = 0.5f;
	void send_velocity_command(float adv, float side, float rot);
	void draw_trajectory_debug(const rc::TrajectoryController::ControlOutput &ctrl,
	                           const Eigen::Affine2f &robot_pose);
	// Trajectory debug graphic items
	std::vector<std::vector<QGraphicsLineItem*>> traj_draw_items_;  // [traj_idx][segment]
	QGraphicsEllipseItem* traj_carrot_marker_ = nullptr;
	QGraphicsLineItem* traj_robot_to_carrot_ = nullptr;

	// CPU usage tracking
	float get_cpu_usage();
	clock_t last_sys_cpu_time_ = 0;
	clock_t last_user_cpu_time_ = 0;
	int num_processors_ = 1;

	// Covariance ellipse visualization
	QGraphicsEllipseItem* cov_ellipse_item_ = nullptr;

	// Draw
	void draw_lidar_points(const std::vector<Eigen::Vector3f> &points_high,
	                       const std::vector<Eigen::Vector3f> &points_low,
	                       const Eigen::Affine2f &robot_pose);
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
    void save_layout_to_svg(const std::string& filename);
    void load_layout_from_file(const std::string& filename);
    void load_polygon_from_file(const std::string& filename);
    void load_polygon_from_svg(const QString& svg_content);

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
	void slot_new_target(QPointF pos);

};

#endif



