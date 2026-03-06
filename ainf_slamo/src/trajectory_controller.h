#pragma once

#include <vector>
#include <optional>
#include <random>
#include <Eigen/Dense>

namespace rc
{
/**
 * TrajectoryController — MPPI-based local controller with ESDF.
 *
 * Proper MPPI: warm-start + Gaussian perturbations + AR(1) noise.
 * All K samples are perturbations of the previous optimal sequence.
 * Weighted average over the FULL T-step sequence (not just first step).
 */
class TrajectoryController
{
public:
    struct Params
    {
        // Superparameter in [0,1]: 0 = calm, 0.5 = neutral, 1 = excited
        float mood = 0.5f;
        bool  enable_mood = true;

        // Mood gains (higher values increase the effect of mood on each family)
        float mood_speed_gain = 0.35f;        // max_adv, max_rot, carrot lookahead
        float mood_exploration_gain = 0.40f;  // K, T, sigmas, optimizer effort
        float mood_reactivity_gain = 0.35f;   // warm-start inertia, output smoothing, brake
        float mood_caution_gain = 0.30f;      // calm-side boost for d_safe and obstacle conservatism

        // Kinematic limits (differential drive: adv + rot only)
        float max_adv   = 0.8f;   // m/s forward
        float max_rot   = 0.7f;   // rad/s

        // Safety
        float d_safe       = 0.5f; // minimum ESDF distance to consider a trajectory safe
        float robot_radius = 0.3f;

        // Carrot / path following
        float carrot_lookahead = 2.0f;
        float goal_threshold   = 0.25f;

        // MPPI sampling — initial / baseline values (adapted by ESS)
        int   num_samples      = 50;       // K baseline
        int   trajectory_steps = 30;       // T baseline
        float trajectory_dt    = 0.1f;     // dt per step

        // Command floors to avoid degenerate zero-motion plans
        float min_adv_cmd = 0.05f;

        // Discount and numerical epsilons
        float cost_discount = 0.95f;          // per-step discount in rollout scoring
        float weights_epsilon = 1e-10f;       // minimum normalized-weight sum / weight threshold
        float ess_den_epsilon = 1e-20f;       // ESS denominator guard

        // Debug/diagnostics
        int debug_print_period = 20;          // print MPPI diagnostics every N compute cycles

        // ESS-based adaptive ranges
        int   K_min = 20,  K_max = 120;    // adaptive K bounds
        int   T_min = 15,  T_max = 120;     // adaptive T bounds
        float lambda_min = 1.0f, lambda_max = 20.0f;  // adaptive λ bounds
        float cpu_budget_ms = 5.0f;        // max MPPI time per cycle (10% of 50ms)
        float ess_smoothing = 0.25f;       // EMA alpha for ESS (faster response to drops)
        float ess_initial_ratio = 0.5f;    // initial ESS / K used when a new path starts
        int   adapt_interval = 2;          // adapt K/T every N cycles (was 5)

        // MPPI temperature (initial, adapted by ESS)
        float mppi_lambda       = 5.0f;

        // Noise standard deviations (for Gaussian perturbations)
        float sigma_adv   = 0.12f;
        float sigma_rot   = 0.25f;

        // AR(1) temporal noise correlation
        float noise_alpha  = 0.80f;
        // Adaptive sigma limits
        float sigma_min_adv   = 0.04f;
        float sigma_min_rot   = 0.08f;
        float sigma_max_adv   = 0.25f;
        float sigma_max_rot   = 0.40f;

        // Nominal control blending (warm start weights)
        float warm_start_adv_weight = 0.5f;
        float warm_start_rot_weight = 0.5f;

        // Gradient optimization (post-processing refinement)
        int   optim_iterations = 3;
        float optim_lr         = 0.05f;
        float optimize_goal_pull_dist_cap = 1.0f;     // cap for goal-pull distance in seed refinement
        float optimize_obstacle_cap_ratio = 2.0f;     // max obstacle correction / goal correction ratio
        float optimize_goal_min_norm = 1e-4f;         // minimum goal correction norm for ratio gating
        float optimize_remaining_cap_steps = 3.0f;    // cap remaining-time Jacobian factor in dt units

        // EFE weights (scoring)
        float lambda_goal      = 5.0f;
        float lambda_obstacle  = 18.0f;
        float lambda_smooth    = 0.5f;
        float lambda_velocity  = 0.01f;
        float lambda_delta_vel = 0.05f;
        float lambda_heading   = 0.5f;   // heading term multiplier relative to lambda_goal
        float lambda_progress  = 1.0f;   // moving-away penalty multiplier relative to lambda_goal

        // Obstacle barrier details
        float close_obstacle_margin = 0.1f;   // extra margin over robot radius for exponential barrier
        float close_obstacle_gain   = 10.0f;  // gain of exponential close-obstacle penalty

        // Collision and velocity-shape penalties in rollout score
        float collision_penalty = 1000.0f;
        float rot_cost_factor = 4.0f;         // relative cost multiplier for rotational effort

        // Nominal and injected seed shaping
        float nominal_alignment_floor = 0.1f; // minimum forward alignment when turning toward carrot
        float nominal_goal_dist_scale = 1.0f; // distance at which nominal speed reaches full scale
        float injection_adv_scale = 0.7f;     // forward speed scale used by structured injection seeds

        // Structured exploration offsets (radians)
        float inject_offset_30 = 0.5f;
        float inject_offset_60 = 1.05f;
        float inject_offset_90 = 1.57f;
        float inject_offset_120 = 2.09f;
        int   inject_count_start = 2;         // initial number of structured injection seeds
        int   inject_count_very_low_ess = 6;  // injection count when ESS is very low
        int   inject_count_low_ess = 4;       // injection count when ESS is low
        int   inject_count_mid_ess = 3;       // injection count when ESS is medium
        int   inject_count_high_ess = 2;      // injection count when ESS is healthy

        // ESS adaptation thresholds and gains
        float ess_low_very = 0.15f;
        float ess_low = 0.30f;
        float ess_high = 0.50f;
        float ess_high_strong = 0.60f;
        float ess_inject_low = 0.25f;
        float ess_inject_mid = 0.40f;

        float lambda_gain_low_very = 1.15f;
        float lambda_gain_low = 1.08f;
        float lambda_gain_high = 0.96f;

        float sigma_rot_gain_low_very = 1.08f;
        float sigma_adv_gain_low_very = 0.90f;
        float sigma_rot_gain_low = 1.04f;
        float sigma_adv_gain_low = 0.95f;
        float sigma_rot_gain_high = 0.97f;
        float sigma_adv_gain_high = 1.01f;

        float adapt_K_gain_low_very = 1.35f;
        float adapt_K_gain_low = 1.20f;
        float adapt_K_gain_high = 0.85f;
        float adapt_T_gain_low = 1.15f;
        float adapt_T_gain_high = 0.90f;

        float collision_ratio_high = 0.5f;
        float collision_ratio_low = 0.2f;
        float cpu_estimation_min_ms = 0.1f;

        // ESDF grid
        float grid_resolution  = 0.05f;
        float grid_half_size   = 4.0f;
        float esdf_unknown_distance = 100.0f;   // distance returned when ESDF query is outside the grid
        float esdf_init_distance = 9999.0f;     // initial large value used during ESDF passes
        float esdf_diag_step = 1.414f;          // diagonal neighbor cost in grid units
        float esdf_grad_min_norm = 1e-4f;       // gradient norm threshold to normalize ESDF gradient

        // Path progression heuristics
        float waypoint_advance_lookahead_factor = 0.5f;
        float segment_length_epsilon = 1e-3f;

        // Goal/heading numerical thresholds
        float heading_norm_epsilon = 0.01f;

        // Output smoothing
        float velocity_smoothing = 0.45f;

        // Visualization
        int num_trajectories_to_draw = 10;

        // Gaussian brake for high rotation (to prevent oscillation)
        float gauss_k = 1.0f;  // Higher = stronger brake at high rotation
    };

    struct ControlOutput
    {
        float adv  = 0.f;
        float side = 0.f;
        float rot  = 0.f;
        bool  goal_reached = false;
        float dist_to_goal = 0.f;
        float min_esdf = 0.f;

        // ESS diagnostics for UI
        float ess = 0.f;          // current ESS value
        int   ess_K = 1;          // current K (to compute ratio)

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

    /// Set static obstacle polygons (furniture). Their edges are sampled into points
    /// and injected into every ESDF build, transformed to robot frame.
    void set_static_obstacles(const std::vector<std::vector<Eigen::Vector2f>>& obstacles_room,
                              float sample_spacing = 0.05f);

    bool is_active() const { return active_ && !path_room_.empty(); }
    int current_waypoint_index() const { return wp_index_; }
    const std::vector<Eigen::Vector2f>& get_path() const { return path_room_; }
    std::optional<Eigen::Vector2f> current_waypoint_room() const;

    Params params;

private:
    Params active_params_;

    bool active_ = false;
    std::vector<Eigen::Vector2f> path_room_;
    int wp_index_ = 0;

    // ---- ESDF ----
    std::vector<float> esdf_data_;
    int esdf_N_ = 0;

    // Static obstacles (furniture) — pre-sampled points in room frame
    std::vector<Eigen::Vector2f> static_obstacle_points_room_;

    // Output smoothing
    Eigen::Vector3f smoothed_vel_ = Eigen::Vector3f::Zero();
    bool has_prev_vel_ = false;

    // ---- MPPI state ----
    // Previous optimal control sequence (T steps of [adv, rot])
    // This is the core warm-start: each cycle shifts it and samples around it
    struct ControlStep { float adv = 0.f; float rot = 0.f; };
    std::vector<ControlStep> prev_optimal_;

    // Adaptive noise sigmas
    float adaptive_sigma_adv_;
    float adaptive_sigma_rot_;

    // ESS-based adaptive state
    int   adaptive_K_;               // current number of samples
    int   adaptive_T_;               // current horizon length
    float adaptive_lambda_;          // current MPPI temperature
    int   adaptive_n_inject_ = 2;    // current injection count
    float ess_smooth_ = 0.f;        // EMA-smoothed ESS
    float last_mppi_ms_ = 0.f;      // last MPPI wall-clock time
    int   adapt_counter_ = 0;       // counter for K/T adaptation interval

    // Compute ESS from weights and adapt K, T, λ, σ, injections
    float compute_ess(const std::vector<float>& weights, int K) const;
    void adapt_from_ess(float ess, int K, int num_collisions);
    void refresh_active_params();

    // RNG
    std::mt19937 rng_{std::random_device{}()};
    std::normal_distribution<float> normal_{0.f, 1.f};

    // A trajectory sample: sequence of (adv, rot) commands
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

    // ---- Methods ----
    void build_esdf(const std::vector<Eigen::Vector3f>& lidar_points,
                    const Eigen::Affine2f& robot_pose);
    float query_esdf(float rx, float ry) const;
    Eigen::Vector2f query_esdf_gradient(float rx, float ry) const;

    Eigen::Vector2f compute_carrot(const Eigen::Affine2f& robot_pose) const;
    void advance_waypoints(const Eigen::Affine2f& robot_pose);

    // Nominal control toward carrot (initial guess for warm-start)
    Seed compute_nominal(const Eigen::Vector2f& carrot_robot, int steps) const;

    // MPPI sampling: generate K perturbations around prev_optimal_
    std::vector<Seed> sample_trajectories(const Eigen::Vector2f& carrot_robot);

    SimResult simulate_and_score(const Seed& seed, const Eigen::Vector2f& carrot_robot);
    void optimize_seed(Seed& seed, const Eigen::Vector2f& carrot_robot);

    static Eigen::Vector2f room_to_robot(const Eigen::Vector2f& p_room,
                                          const Eigen::Affine2f& robot_pose);
};

} // namespace rc

