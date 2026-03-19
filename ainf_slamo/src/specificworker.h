/*
 *    Copyright (C) 2026 by RoboComp
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
	\brief AINF_SLAMO – Active Inference SLAM and Navigation
	@author robocomp
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
#include "viewer_2d.h"
#include "viewer_3d.h"
#include "scene_tree_panel.h"
#include "object_palette_panel.h"
#include "camera_viewer.h"
#include "furniture_types.h"
#include "object_footprints.h"
#include "scene_graph_adapter.h"
#include "scene_graph_model.h"
#include "layout_manager.h"
#include "em_manager.h"
#include "navigation_manager.h"
#include "synthetic_camera_renderer.h"
#include <QSplitter>
#include "buffer_types.h"
#include "room_concept_ai.h"
#include "pointcloud_center_estimator.h"
#include "polygon_path_planner.h"
#include "trajectory_controller.h"
#include "episodic_memory.h"
#include <variant>
#include <optional>
#include <chrono>
#include <limits>
#include <memory>
#include "common_types.h"

class QWidget;
class QLabel;
class QPushButton;

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
	std::unique_ptr<rc::Viewer2D> viewer_2d_;            // 2D scene viewer (owned by us)
	std::unique_ptr<rc::Viewer3D> viewer_3d_;            // 3D scene viewer (owned by us)
	std::unique_ptr<SceneTreePanel> scene_tree_;          // 3rd pane: scene element tree
	std::unique_ptr<CameraViewer>  camera_viewer_;        // Camera popup dialog
	rc::SyntheticCameraRenderer    synthetic_renderer_;    // Wireframe overlay renderer
	std::mutex synth_vis_mutex_;
	std::vector<int> last_visible_indices_;                  // cached from last synthetic render

	// Camera intrinsics (lazily set from first TImage via CameraViewer signal)
	struct CachedCameraIntrinsics {
		float fx = 0.f, fy = 0.f;     // zero until first TImage arrives
		float cx = 0.f, cy = 0.f;
		int   w  = 0,   h  = 0;
		bool  valid = false;
	} camera_intr_;

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
		// Camera extrinsics: camera frame origin expressed in robot frame (meters)
		float CAMERA_TX = 0.0f;
		float CAMERA_TY = -0.11f;
		float CAMERA_TZ = 0.92f;
	};
	Params params;

	// Timer
	rc::Timer<> clock;
	FPSCounter fps;

	// Type alias for lidar data in buffer (delegates to rc::LidarData from buffer_types.h)
	using LidarData = rc::LidarData;

	// Sync Buffer: slot 0 = GT pose, slot 1 = HELIOS high lidar, slot 2 = BPEARL low lidar
	rc::SensorBuffer buffer_sync;

	// Lidar Thread
	std::thread read_lidar_th;
	std::atomic<bool> stop_lidar_thread{false};
	void read_lidar();

	// Room model

	// Layout manager: owns furniture polygons, room polygon, wraps scene graph
	rc::SceneGraphModel scene_graph_;
	LayoutManager layout_manager_{scene_graph_};
	void draw_furniture();
	void update_furniture_draw_item(std::size_t idx);

	// Navigation manager: owns path planner, trajectory controller, navigation state
	NavigationManager nav_manager_;

	/// Draw temporary obstacles in the viewer
	void draw_temp_obstacles();

	/// Compute the q-th percentile of a vector (pass by value for in-place partial sort)
	static float percentile_copy(std::vector<float> values, float q);

	// Flip state tracking
	bool flip_x_applied_ = false;
	bool flip_y_applied_ = false;
	bool auto_center_ = false;       // Auto-center view on robot
	bool draw_lidar_ = false;        // Toggle lidar point rendering
	bool draw_trajectories_ = false; // Toggle trajectory debug rendering
	bool editing_room_layout_ = false; // 2D room-corner edit mode
	int editing_corner_idx_ = -1;      // currently dragged corner index
	std::vector<Eigen::Vector2f> editing_room_polygon_; // working polygon while editing
        bool initial_center_done_ = false; // Flag to center view once at start
	QSplitter* splitter_ = nullptr;  // Horizontal splitter (2D | 3D views)
	QSplitter* right_splitter_ = nullptr; // Vertical splitter in right pane (tree top | palette bottom)
	int tree_panel_saved_width_ = 220;    // last known width of the right panel (for toggle restore)
	QWidget* grounding_panel_ = nullptr;          // Hidden container for grounding status labels
	QLabel* grounding_status_label_ = nullptr;
	QLabel* grounding_cam_label_ = nullptr;
	QLabel* grounding_world_label_ = nullptr;
	QLabel* grounding_score_label_ = nullptr;
	QLabel* grounding_sdf_label_ = nullptr;
	QPushButton* grounding_fit_mesh_button_ = nullptr;
	ObjectPalettePanel* object_palette_ = nullptr;
	std::vector<Eigen::Vector3f> grounding_focus_points_;
	Eigen::Vector2f grounding_focus_center_ = Eigen::Vector2f::Zero();
	std::string grounding_focus_label_;
	int grounding_world_index_ = -1;
	int focused_model_index_ = -1;
	EMManager em_manager_;
	rc::VelocityBuffer velocity_buffer_{20};

	// Measured odometry readings from FullPoseEstimationPub (thread-safe via BufferSync)
	rc::OdometryBuffer odometry_buffer_{20};

	// ===== Localization (delegated to room_ai) =====
	// room_ai owns its own thread; these helpers forward commands and read results.

	/// Thread-safe read of latest localization state [half_w, half_h, x, y, theta]
	/// Returns zeros if not yet initialized
	Eigen::Matrix<float,5,1> get_loc_state() const { return room_ai.get_loc_state(); }

	/// Push a command to the localization thread (thin wrapper)
	void push_loc_command(rc::RoomConceptAI::Command cmd) { room_ai.push_command(std::move(cmd)); }

	// Active inference room concept (owns localization thread)
	rc::RoomConceptAI room_ai;

	void draw_path_threadsafe(const std::vector<Eigen::Vector2f>& path);
	void draw_path(const std::vector<Eigen::Vector2f>& path);
	void clear_path(bool stop_controller = true, bool clear_stored_path = true);
	rc::EpisodicMemory episodic_memory_;
	std::optional<rc::EpisodicMemory::EpisodeRecord> active_episode_;
	struct EpisodeAccum
	{
		bool has_prev_pose = false;
		Eigen::Vector2f prev_pos = Eigen::Vector2f::Zero();
		float start_to_goal_dist_m = 0.f;
		int n_cycles = 0;
		float distance_traveled_m = 0.f;
		float min_esdf_m = std::numeric_limits<float>::max();
		float blocked_time_s = 0.f;
		int n_blocked_events = 0;
		std::vector<float> speed_samples;
		std::vector<float> rot_samples;
		std::vector<float> ess_ratio_samples;
		std::vector<float> cpu_samples;
		std::vector<float> mppi_ms_samples;
		std::chrono::steady_clock::time_point last_block_sample;
		std::chrono::steady_clock::time_point last_metric_sample;
		bool has_last_metric_sample = false;
		bool was_blocked = false;
	} episode_accum_;

	std::chrono::steady_clock::time_point last_joystick_time_;
	static constexpr float JOYSTICK_TIMEOUT_SEC = 0.5f;
	void send_velocity_command(float adv, float side, float rot);
	void draw_trajectory_debug(const rc::TrajectoryController::ControlOutput &ctrl,
	                           const Eigen::Affine2f &robot_pose);
	// CPU usage tracking
	float get_cpu_usage();
	clock_t last_sys_cpu_time_ = 0;
	clock_t last_user_cpu_time_ = 0;
	int num_processors_ = 1;

	// Draw
	void draw_lidar_points(const std::vector<Eigen::Vector3f> &points_high,
	                       const std::vector<Eigen::Vector3f> &points_low,
	                       const Eigen::Affine2f &robot_pose);
	void update_camera_wireframe_overlay(const Eigen::Affine2f &robot_pose);
	int pick_attention_target(const Eigen::Affine2f& robot_pose) const;
	int find_furniture_index_by_name(const QString& name) const;
	void translate_furniture_by_name(const QString& name, float dx_room, float dy_room);
	void rotate_furniture_by_name(const QString& name, float angle_rad, const QVector3D& axis);
	int wall_index_from_pick_name(const QString& name) const;
	void translate_wall_by_name(const QString& name, float dx_room, float dy_room);
	void rotate_wall_by_name(const QString& name, float angle_rad, const QVector3D& axis);
	void set_object_property(const QString& label, const QString& property, float value);
	void update_segmented_points_3d(const Eigen::Affine2f &robot_pose);
	void draw_estimated_room(const Eigen::Matrix<float, 5, 1> &state);
	void display_robot(const Eigen::Affine2f &robot_pose, const Eigen::Matrix3f &covariance);
	void display_gt_error(const Eigen::Affine2f &estimated_pose, const std::optional<Eigen::Affine2f> &gt_pose_opt);
	float yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat);
	float estimate_orientation_from_points(const std::vector<Eigen::Vector3f> &pts) const;
	void update_ui(const rc::RoomConceptAI::UpdateResult &res, const rc::VelocityCommand &current_velocity, int fps_val);
	float compute_source_obstacle_density(const Eigen::Vector2f& source_point) const;
	void start_episode(const std::string &mission_type,
	                   const std::optional<Eigen::Vector2f> &target_point = std::nullopt,
	                   const std::string &target_object = "");
	void update_episode_metrics(const rc::RoomConceptAI::UpdateResult &res,
	                           const rc::TrajectoryController::ControlOutput *ctrl,
	                           float current_speed,
	                           float current_rot,
	                           float cpu_usage,
	                           float mppi_ms,
	                           bool blocked_state);
	void finish_episode(const std::string &status);

	void draw_room_polygon();

	// Layout save/load
    void save_layout_to_svg(const std::string& filename);
    void load_layout_from_file(const std::string& filename);
    void load_polygon_from_file(const std::string& filename);
    void load_polygon_from_svg(const QString& svg_content);
	void save_scene_graph_to_usd();
	bool load_scene_graph_from_usd();
	void push_undo_snapshot();
	void undo_last_action();

	// Pose persistence (for fast restart)
	void save_last_pose();
	void save_camera_state_to_settings() const;
	bool load_last_pose();  // Returns true if pose was loaded successfully
	void perform_grid_search(const std::vector<Eigen::Vector3f>& lidar_points);
	static constexpr const char* LAST_POSE_FILE = "./last_pose.json";
	static constexpr const char* PERSISTED_SCENE_GRAPH_FILE = "./scene_graph.usda";
	std::vector<std::string> undo_stack_;
	bool undo_group_open_ = false;
	static constexpr std::size_t MAX_UNDO_STEPS = 50;

	// GT calibration: offset from Webots frame to map frame
	Eigen::Affine2f gt_offset_ = Eigen::Affine2f::Identity();
	bool gt_calibrated_ = false;
	void calibrate_gt_offset(const Eigen::Affine2f &estimated_pose, const Eigen::Affine2f &webots_pose);

private slots:
	void slot_save_layout();
	void slot_edit_layout_toggled(bool checked);
	void slot_flip_x();
	void slot_flip_y();
	void slot_robot_dragging(QPointF pos);
	void slot_robot_drag_end(QPointF pos);
	void slot_robot_rotate(QPointF pos);
	void slot_calibrate_gt();
	void slot_new_target(QPointF pos);

};

#endif // SPECIFICWORKER_H