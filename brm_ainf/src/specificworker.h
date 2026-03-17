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

#ifdef slots
  #define RC_QT_SLOTS_WAS_DEFINED
  #undef slots
#endif
#ifdef signals
  #define RC_QT_SIGNALS_WAS_DEFINED
  #undef signals
#endif
#ifdef emit
  #define RC_QT_EMIT_WAS_DEFINED
  #undef emit
#endif

#include <torch/torch.h>

#ifdef RC_QT_SLOTS_WAS_DEFINED
  #define slots Q_SLOTS
  #undef RC_QT_SLOTS_WAS_DEFINED
#endif
#ifdef RC_QT_SIGNALS_WAS_DEFINED
  #define signals Q_SIGNALS
  #undef RC_QT_SIGNALS_WAS_DEFINED
#endif
#ifdef RC_QT_EMIT_WAS_DEFINED
  #define emit Q_EMIT
  #undef RC_QT_EMIT_WAS_DEFINED
#endif

#include <genericworker.h>

// Eigen includes for linear algebra
#include <Eigen/Dense>
#include <chrono>
#include <QGraphicsEllipseItem>
#include <QLabel>
#include <QListWidget>
#include <QSplitter>
#include "buffer_types.h"
#include "common_types.h"
#include "doublebuffer_sync/doublebuffer_sync.h"
#include "room_bootstrapper.h"
#include "room_concept_ai.h"
#include "viewer_2d.h"

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
	enum class RuntimePhase
	{
		BOOTSTRAPPING = 0,
		LOCALIZING = 1
	};

struct Params
	{
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
		// General
		QRectF GRID_MAX_DIM{-5, -5, 10, 10};
		int MAX_LIDAR_DRAW_POINTS = 500;
		// Bootstrap parameters
		int INIT_MIN_FRAMES = 8;   // LiDAR frames to accumulate before bootstrap
		int INIT_MIN_POINTS = 1200; // Minimum points required for robust rectangle estimate
		float INIT_SPAN_SCALE = 1.18f; // Multiplicative inflation to avoid conservative bootstrap
		float INIT_SPAN_MARGIN = 0.35f; // Additive margin (m) to compensate sparse wall returns
		float INIT_GRID_RESOLUTION = 0.4f; // meters for initial grid search
		float INIT_ANGLE_RESOLUTION = static_cast<float>(M_PI / 18.0); // radians (10 deg) for initial grid search
		float INIT_BELIEF_MIN = 0.50f;   // Accept bootstrap when belief exceeds this value
		float INIT_SDF_P90_MAX = 0.15f;  // Accept bootstrap when SDF p90 is below this threshold (m)
		float INIT_REVERSE_SDF_MAX = 0.12f; // Accept bootstrap when reverse SDF is below this threshold (m)
		int INIT_ACCEPT_CONSECUTIVE = 5; // Number of consecutive valid frames required
		bool BOOTSTRAP_ENABLE_ROTATION = true; // Slow in-place rotation to improve observation diversity
		bool BOOTSTRAP_ROTATE_ONLY_PRESEED = true; // Stop rotating once seeded to reduce estimator lag
		float BOOTSTRAP_ROT_SPEED = 0.1f; // rad/s
		// EFE (Expected Free Energy) policy parameters for active exploration of BMR hot-zones
		bool ENABLE_EFE_HOTZONE_POLICY = true; // Approach BMR hot-zone to gather discriminative evidence
		float EFE_DT = 0.5f; // seconds per prediction step
		int EFE_TREE_DEPTH = 4; // policy tree branching depth (|A|^D evaluated sequences, e.g. 6^4=1296)
		float EFE_DISCOUNT = 0.90f; // temporal discount factor across tree levels
		float EFE_PRIOR_SIGMA = 0.8f; // meters, goal-reaching prior covariance std
		float EFE_FORWARD_SPEED = 0.12f; // m/s
		float EFE_ROT_SPEED = 0.25f; // rad/s
		float EFE_INFO_GAIN_WEIGHT = 0.40f; // weight for epistemic info-gain term (replaces heading-error proxy)
		float EFE_CONTROL_WEIGHT = 0.05f; // control effort term weight
		float EFE_TURN_ONLY_MIN_DIST = 0.45f; // force translational arc when far from EFE target to avoid turn-in-place dithering
		float EFE_LOCAL_GOAL_MAX_DIST = 1.2f; // meters, cap far hot-zones to a reachable local subgoal
		float EFE_TARGET_SMOOTH_ALPHA = 0.30f; // low-pass filter for BMR hot-zone target to avoid left-right command jitter
		float ODOMETRY_NOISE_FACTOR = 0.1f;  // Gaussian noise std added to odometry (fraction of reading)
		// Camera extrinsics: camera frame origin expressed in robot frame (meters)
		float CAMERA_TX = 0.0f;
		float CAMERA_TY = -0.11f;
		float CAMERA_TZ = 0.92f;
	};
	Params params;

    // Measured odometry readings from FullPoseEstimationPub (thread-safe via BufferSync)
	BufferSync<InOut<rc::OdometryReading, rc::OdometryReading>> odometry_buffer_{20};

	// Velocity history
	rc::VelocityBuffer velocity_buffer_{20};
	rc::RoomVFEBMR bmr_engine_;
	RuntimePhase phase_ = RuntimePhase::BOOTSTRAPPING;
	bool loc_thread_started_ = false;
	bool bootstrap_seeded_ = false;
	bool room_bootstrapped_ = false;
	int bootstrap_accept_streak_ = 0;
	bool bootstrap_rotating_ = false;
	bool efe_hotzone_active_ = false;
	bool has_smoothed_efe_target_ = false;
	Eigen::Vector2f smoothed_efe_target_{Eigen::Vector2f::Zero()};
	rc::RoomBootstrapper room_bootstrapper_;
	std::unique_ptr<rc::Viewer2D> viewer_2d_;
	std::vector<QGraphicsEllipseItem*> lidar_draw_items_;
	QLabel*       phase_label_     = nullptr;
	QLabel*       score_label_     = nullptr;
	QListWidget*  candidate_list_  = nullptr;
	QLabel*       winner_label_    = nullptr;

   // Active inference room concept (owns localization thread)
	rc::RoomConceptAI room_ai;

	 /// Main sensor buffer: slot 0 = GT pose, slot 1 = high lidar, slot 2 = low lidar
	rc::SensorBuffer lidar_buffer{20};

	// Lidar Thread
	std::thread read_lidar_th;
	std::atomic<bool> stop_lidar_thread{false};
    void read_lidar();
    bool try_initialize_room_from_lidar(const rc::LidarData &lidar_data,
                                        const std::vector<rc::OdometryReading> &odometry_history);
	void set_bootstrap_rotation(bool enable);
	void send_base_command(float advx, float advz, float rot);
	void push_velocity_command(float advx, float advz, float rot);
	void drive_toward_hotzone_efe(const rc::RoomConceptAI::UpdateResult &res, const rc::BmrResult &bmr);
	void stop_efe_hotzone_motion();
	void set_phase(RuntimePhase new_phase);
	void set_compound_score_label(const rc::RoomConceptAI::UpdateResult &res);
	static const char *phase_to_cstr(RuntimePhase phase);
	void update_viewer(const std::vector<Eigen::Vector3f> &points, const rc::RoomConceptAI::UpdateResult &res);
	void update_viewer(const rc::LidarData &lidar_data, const rc::RoomConceptAI::UpdateResult &res);
	void update_candidate_list(const std::vector<rc::BmrResult::IndentCandidateInfo>& candidates);

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

signals:
	//void customSignal();
};

#endif
