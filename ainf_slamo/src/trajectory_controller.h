#pragma once

#include <vector>
#include <optional>
#include <random>
#include <Eigen/Dense>

// ---- PyTorch vs Qt macros ----
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

namespace rc
{
/**
 * TrajectoryController — Multi-seed gradient-optimized local controller with ESDF.
 *
 * Adaptive seed generation based on previous-cycle score feedback:
 *   - Good best score → few seeds, narrow spread, short horizon (cruise)
 *   - Many colliding / high-G seeds → wide spread, longer horizon (explore)
 *   - Spread biased towards the direction of the best-scoring seed
 */
class TrajectoryController
{
public:
    struct Params
    {
        // Kinematic limits
        float max_adv   = 0.6f;
        float max_side  = 0.3f;
        float max_rot   = 1.0f;

        // Safety
        float d_safe       = 0.4f;
        float robot_radius = 0.3f;

        // Carrot / path following
        float carrot_lookahead = 1.5f;
        float goal_threshold   = 0.25f;

        // Trajectory seeds (base values — adapted at runtime)
        int   num_seeds        = 7;
        int   trajectory_steps = 20;
        float trajectory_dt    = 0.15f;

        // Adaptive ranges
        int   min_seeds        = 3;
        int   max_seeds        = 20;
        int   min_steps        = 15;
        int   max_steps        = 150;
        float min_spread_deg   = 15.f;    // minimum angular spread (degrees)
        float max_spread_deg   = 360.f;   // maximum angular spread (degrees)

        // Gradient optimization
        int   optim_iterations = 3;
        float optim_lr         = 0.05f;

        // EFE weights
        float lambda_goal      = 4.0f;
        float lambda_obstacle  = 10.0f;
        float lambda_smooth    = 0.5f;
        float lambda_velocity  = 0.01f;   // velocity magnitude penalty
        float lambda_delta_vel = 0.05f;   // action change penalty

        // ESDF grid
        float grid_resolution  = 0.05f;
        float grid_half_size   = 4.0f;

        // Output smoothing
        float velocity_smoothing = 0.3f;

        // MPPI temperature: lower = more selective, higher = more averaging
        float mppi_lambda       = 5.0f;
    };

    struct ControlOutput
    {
        float adv  = 0.f;
        float side = 0.f;
        float rot  = 0.f;
        bool  goal_reached = false;
        float dist_to_goal = 0.f;
        float min_esdf = 0.f;

        Eigen::Vector2f carrot_room = Eigen::Vector2f::Zero();
        int current_wp_index = 0;

        // All candidate trajectories in room frame (fresh every tick)
        std::vector<std::vector<Eigen::Vector2f>> trajectories_room;
        int best_trajectory_idx = -1;
    };

    TrajectoryController() = default;

    void set_path(const std::vector<Eigen::Vector2f>& path_room);
    ControlOutput compute(const std::vector<Eigen::Vector3f>& lidar_points,
                          const Eigen::Affine2f& robot_pose);
    void stop();

    bool is_active() const { return active_ && !path_room_.empty(); }
    int current_waypoint_index() const { return wp_index_; }
    const std::vector<Eigen::Vector2f>& get_path() const { return path_room_; }
    std::optional<Eigen::Vector2f> current_waypoint_room() const;

    Params params;

private:
    bool active_ = false;
    std::vector<Eigen::Vector2f> path_room_;
    int wp_index_ = 0;

    // ESDF
    std::vector<float> esdf_data_;
    int esdf_N_ = 0;

    Eigen::Vector3f smoothed_vel_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f prev_best_cmd_ = Eigen::Vector3f::Zero();
    bool has_prev_vel_ = false;

    // A trajectory seed: sequence of (adv, rot) commands
    struct Seed
    {
        std::vector<float> adv;
        std::vector<float> rot;
    };

    // Result of simulating a seed
    struct SimResult
    {
        std::vector<Eigen::Vector2f> positions;
        float G_total = 0.f;
        float min_esdf = 1e9f;
        bool  collides = false;
    };

    // ---------- Adaptive parameters (feedback from previous cycle) ----------
    struct AdaptiveState
    {
        int   num_seeds = 7;
        int   steps     = 20;
        float spread    = 1.0f;             // half-angle (radians)
        float drive_speed = 0.6f;
        float best_angle = 0.f;             // angle of the best seed last cycle
        float collision_ratio = 0.f;        // fraction of seeds that collided
        float best_G = 1e9f;                // best G from last cycle

        // EFE-based adaptive horizon (EMA of log(1+G))
        float efe_smoothed = 0.f;
        float efe_gamma    = 0.85f;          // EMA decay
        float efe_threshold = 5.0f;          // log-G above this → explore
        float efe_sensitivity = 0.4f;        // tanh scaling factor
    };
    AdaptiveState adaptive_;

    void update_adaptive(const std::vector<SimResult>& results,
                         const std::vector<Seed>& seeds,
                         int best_idx,
                         const Eigen::Vector2f& carrot_robot);


    // RNG for exploration noise
    std::mt19937 rng_{std::random_device{}()};

    // ---- Methods ----
    void build_esdf(const std::vector<Eigen::Vector3f>& lidar_points);
    float query_esdf(float rx, float ry) const;
    Eigen::Vector2f query_esdf_gradient(float rx, float ry) const;

    Eigen::Vector2f compute_carrot(const Eigen::Affine2f& robot_pose) const;
    void advance_waypoints(const Eigen::Affine2f& robot_pose);

    std::vector<Seed> generate_seeds(const Eigen::Vector2f& carrot_robot);
    // Geometric seed generators (no ESDF dependency — pure direction blending)
    Seed make_curve_seed(float carrot_angle, float offset_angle, int steps, float speed);
    Seed make_momentum_seed(int steps, float speed, float carrot_angle);
    Seed make_spread_seed(float target_angle, float carrot_angle, int steps, float speed);

    SimResult simulate_and_score(const Seed& seed, const Eigen::Vector2f& carrot_robot);
    void optimize_seed(Seed& seed, const Eigen::Vector2f& carrot_robot);

    static Eigen::Vector2f room_to_robot(const Eigen::Vector2f& p_room,
                                          const Eigen::Affine2f& robot_pose);
};

} // namespace rc

