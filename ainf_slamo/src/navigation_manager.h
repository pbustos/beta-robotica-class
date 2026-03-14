#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <vector>
#include <optional>
#include <string>
#include <chrono>
#include <limits>
#include "polygon_path_planner.h"
#include "trajectory_controller.h"
#include <Navigator.h>

class LayoutManager;

class NavigationManager
{
public:
    struct TempObstacle
    {
        std::vector<Eigen::Vector2f> vertices;
        Eigen::Vector2f center;
        std::chrono::steady_clock::time_point created;
        int replan_count = 0;
        float height = 0.f;
    };

    struct Context
    {
        LayoutManager* layout_manager = nullptr;
        std::function<Eigen::Matrix<float,5,1>()> get_loc_state;
        std::function<void(float adv, float side, float rot)> send_velocity;
        std::function<void(const std::vector<Eigen::Vector2f>&)> draw_path;
        std::function<void()> draw_temp_obstacles;
        std::function<void()> clear_path_visual;
    };

    void set_context(const Context& ctx) { ctx_ = ctx; }

    // --- Planning (called from UI slots or ICE threads) ---
    void plan_to_target(const Eigen::Vector2f& target);
    RoboCompNavigator::Result compute_path(const Eigen::Vector2f& source,
                                           const Eigen::Vector2f& target,
                                           float safety = 0.f);
    bool goto_point(const Eigen::Vector2f& target);
    bool goto_object(const std::string& object_name);

    // --- Control ---
    void stop();
    void resume();
    /// Clear navigation state. Returns true if an episode was active (caller should finish it).
    bool clear(bool stop_controller = true, bool clear_stored = true);

    // --- Compute step (called each cycle from main loop) ---
    struct StepResult
    {
        float adv = 0, side = 0, rot = 0;
        bool goal_reached = false;
        bool object_align_started = false;
        bool object_align_done = false;
        bool navigating = false;     // normal MPPI control was applied
        bool recovering = false;     // safeguard backward recovery
        float dist_to_goal = 0;
        float ess = 0;
        int ess_K = 1;
        bool safety_guard_triggered = false;
        bool blocked_like = false;
        float speed = 0;
        float rot_speed = 0;
        float mppi_ms = 0;
        bool has_ctrl_output = false;
        rc::TrajectoryController::ControlOutput ctrl{};
    };

    StepResult compute_step(const std::vector<Eigen::Vector3f>& lidar_points,
                            const Eigen::Affine2f& robot_pose);

    // --- Status / Queries ---
    bool is_active() const { return trajectory_controller_.is_active(); }
    bool is_aligning() const { return object_final_align_active_; }
    bool has_path() const { return !current_path_.empty(); }
    const std::vector<Eigen::Vector2f>& current_path() const { return current_path_; }
    const std::optional<Eigen::Vector2f>& nav_target_object_center() const { return nav_target_object_center_; }
    const std::string& nav_target_object_name() const { return nav_target_object_name_; }

    RoboCompNavigator::NavigationStatus get_status(float current_speed) const;

    // --- Obstacle management ---
    void cleanup_temp_obstacles();
    std::vector<Eigen::Vector2f> cluster_lidar_to_polygon(
        const std::vector<Eigen::Vector3f>& lidar_points,
        const Eigen::Vector2f& blockage_center_room,
        float search_radius,
        const Eigen::Affine2f& robot_pose,
        float& height_out) const;

    const std::vector<TempObstacle>& temp_obstacles() const { return temp_obstacles_; }

    // --- Direct access for initialization and settings ---
    rc::PolygonPathPlanner& path_planner() { return path_planner_; }
    const rc::PolygonPathPlanner& path_planner() const { return path_planner_; }
    rc::TrajectoryController& trajectory_controller() { return trajectory_controller_; }
    const rc::TrajectoryController& trajectory_controller() const { return trajectory_controller_; }

    // --- Safeguard state accessors for UI ---
    float last_ess() const { return last_ess_; }
    int last_ess_K() const { return last_ess_K_; }
    std::chrono::steady_clock::time_point last_safety_guard_trigger_time() const
        { return last_safety_guard_trigger_time_; }

    static constexpr float BLOCKED_SPEED_THRESHOLD = 0.03f;
    static constexpr float BLOCKED_TIME_THRESHOLD_SEC = 2.5f;
    static constexpr float SAFETY_GUARD_UI_HOLD_SEC = 0.8f;
    static constexpr float SOURCE_OBSTACLE_DENSITY_PROBE_EXTRA_RADIUS = 3.0f;

private:
    Context ctx_;

    rc::PolygonPathPlanner path_planner_;
    rc::TrajectoryController trajectory_controller_;
    std::vector<Eigen::Vector2f> current_path_;

    std::optional<Eigen::Vector2f> nav_target_object_center_;
    std::string nav_target_object_name_;
    int object_align_cycles_ = 0;
    bool object_final_align_active_ = false;
    float prev_angle_err_ = 0.f;

    std::vector<TempObstacle> temp_obstacles_;
    static constexpr float TEMP_OBSTACLE_TIMEOUT_SEC = 30.f;
    static constexpr float TEMP_OBSTACLE_MERGE_DIST = 0.8f;
    static constexpr float TEMP_OBSTACLE_MARGIN = 0.05f;

    bool low_speed_block_timer_active_ = false;
    std::chrono::steady_clock::time_point low_speed_block_start_;

    bool safeguard_recovery_active_ = false;
    bool safeguard_replan_pending_ = false;
    int safeguard_recovery_cycles_ = 0;
    int safeguard_clear_counter_ = 0;
    Eigen::Vector2f safeguard_blockage_center_ = Eigen::Vector2f::Zero();
    float safeguard_blockage_radius_ = 0.35f;
    static constexpr int SAFEGUARD_RECOVERY_MAX_CYCLES = 30;
    static constexpr int SAFEGUARD_CLEAR_CONFIRM_CYCLES = 4;
    static constexpr float SAFEGUARD_BACKWARD_SPEED = -0.12f;

    float last_ess_ = 0.f;
    int last_ess_K_ = 1;
    std::chrono::steady_clock::time_point last_safety_guard_trigger_time_{};

    bool replan_around_obstacle(const std::vector<Eigen::Vector2f>& polygon,
                                float obstacle_height,
                                const Eigen::Vector2f& center,
                                const Eigen::Affine2f& robot_pose);
};
