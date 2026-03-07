#include "trajectory_controller.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>
#include <chrono>

namespace rc
{

namespace
{
float clamp01(float x)
{
    return std::clamp(x, 0.f, 1.f);
}

float smoothstep01(float x)
{
    const float t = clamp01(x);
    return t * t * (3.f - 2.f * t);
}
} // namespace

void TrajectoryController::refresh_active_params()
{
    active_params_ = params;
    const float safety_priority = std::max(1.f, params.safety_priority_scale);

    const float base_d_safe_priority = std::max(0.01f, params.d_safe * safety_priority);
    const float base_lambda_obstacle = std::max(1e-5f, params.lambda_obstacle);
    const float base_close_obstacle_gain = std::max(1e-5f, params.close_obstacle_gain);
    const float base_collision_penalty_priority = std::max(1.f, params.collision_penalty * safety_priority);

    active_params_.d_safe = base_d_safe_priority;
    active_params_.lambda_obstacle = base_lambda_obstacle;
    active_params_.close_obstacle_gain = base_close_obstacle_gain;
    active_params_.collision_penalty = base_collision_penalty_priority;

    if (params.enable_mood)
    {
        const float mood = clamp01(params.mood);
        const float s = smoothstep01(mood);
        const float n = 2.f * s - 1.f;  // -1 calm, +1 excited

        auto scale_with_gain = [n](float base, float gain, float min_value)
        {
            return std::max(min_value, base * (1.f + gain * n));
        };

        const float speed_gain = std::max(0.f, params.mood_speed_gain);
        const float exploration_gain = std::max(0.f, params.mood_exploration_gain);
        const float reactivity_gain = std::max(0.f, params.mood_reactivity_gain);
        const float caution_gain = std::max(0.f, params.mood_caution_gain);

        active_params_.max_adv = scale_with_gain(params.max_adv, speed_gain, 0.01f);
        active_params_.max_rot = scale_with_gain(params.max_rot, speed_gain, 0.01f);
        active_params_.carrot_lookahead = scale_with_gain(params.carrot_lookahead, speed_gain, 0.05f);
        active_params_.goal_threshold = scale_with_gain(params.goal_threshold, speed_gain * 0.5f, 0.01f);

        active_params_.num_samples = std::max(1, static_cast<int>(std::round(scale_with_gain(
            static_cast<float>(params.num_samples), exploration_gain, 1.f))));
        active_params_.trajectory_steps = std::max(1, static_cast<int>(std::round(scale_with_gain(
            static_cast<float>(params.trajectory_steps), exploration_gain, 1.f))));

        active_params_.sigma_adv = scale_with_gain(params.sigma_adv, exploration_gain, 1e-4f);
        active_params_.sigma_rot = scale_with_gain(params.sigma_rot, exploration_gain, 1e-4f);
        active_params_.optim_lr = scale_with_gain(params.optim_lr, exploration_gain, 1e-5f);
        active_params_.optim_iterations = std::max(1, static_cast<int>(std::round(scale_with_gain(
            static_cast<float>(params.optim_iterations), exploration_gain * 0.5f, 1.f))));

        active_params_.velocity_smoothing = std::clamp(
            params.velocity_smoothing * (1.f - reactivity_gain * n), 0.f, 0.98f);
        active_params_.warm_start_adv_weight = std::clamp(
            params.warm_start_adv_weight * (1.f - reactivity_gain * n), 0.f, 0.98f);
        active_params_.warm_start_rot_weight = std::clamp(
            params.warm_start_rot_weight * (1.f - reactivity_gain * n), 0.f, 0.98f);
        active_params_.gauss_k = scale_with_gain(params.gauss_k, -reactivity_gain, 1e-5f);

        const float calm_factor = std::max(0.f, -n);  // 0 at neutral/excited, 1 at calm
        active_params_.d_safe = std::max(base_d_safe_priority,
                                         base_d_safe_priority * (1.f + caution_gain * calm_factor));
        active_params_.lambda_obstacle = base_lambda_obstacle;
        active_params_.close_obstacle_gain = base_close_obstacle_gain;
        active_params_.collision_penalty = std::max(base_collision_penalty_priority,
                                                    base_collision_penalty_priority * (1.f + 0.5f * caution_gain * calm_factor));
        active_params_.lambda_smooth = scale_with_gain(params.lambda_smooth, -reactivity_gain, 1e-6f);
        active_params_.lambda_velocity = scale_with_gain(params.lambda_velocity, -reactivity_gain, 1e-8f);
        active_params_.lambda_delta_vel = scale_with_gain(params.lambda_delta_vel, -reactivity_gain, 1e-8f);
        active_params_.lambda_goal = scale_with_gain(params.lambda_goal, speed_gain * 0.5f, 1e-5f);
    }

    active_params_.K_min = std::max(1, std::min(active_params_.K_min, active_params_.K_max));
    active_params_.T_min = std::max(1, std::min(active_params_.T_min, active_params_.T_max));
    active_params_.num_samples = std::clamp(active_params_.num_samples, active_params_.K_min, active_params_.K_max);
    active_params_.trajectory_steps = std::clamp(active_params_.trajectory_steps, active_params_.T_min, active_params_.T_max);
    active_params_.d_safe = std::max(active_params_.d_safe, active_params_.robot_radius + 0.01f);
    active_params_.max_back_adv = std::clamp(active_params_.max_back_adv, 0.f, active_params_.max_adv);
    active_params_.min_adv_cmd = std::clamp(active_params_.min_adv_cmd, 0.f, active_params_.max_adv);
    active_params_.mppi_lambda = std::clamp(active_params_.mppi_lambda, active_params_.lambda_min, active_params_.lambda_max);
}

float TrajectoryController::obstacle_step_cost(float esdf_val) const
{
    const float d_safe = std::max(active_params_.d_safe, active_params_.robot_radius + 1e-3f);
    const float soft_span = std::max(d_safe - active_params_.robot_radius, 1e-3f);

    float soft_penalty = 0.f;
    if (esdf_val < d_safe)
    {
        const float normalized = (d_safe - esdf_val) / soft_span;
        soft_penalty = normalized * normalized;
    }

    const float hard_margin = std::max(active_params_.close_obstacle_margin, 1e-3f);
    const float hard_threshold = active_params_.robot_radius + hard_margin;
    float hard_penalty = 0.f;
    if (esdf_val < hard_threshold)
    {
        const float normalized = (hard_threshold - esdf_val) / hard_margin;
        hard_penalty = active_params_.close_obstacle_gain * normalized * normalized;
    }

    const float g_obs = active_params_.lambda_obstacle * (soft_penalty + hard_penalty);
    return std::min(g_obs, active_params_.obstacle_cost_cap);
}

float TrajectoryController::obstacle_repulsion_strength(float esdf_val) const
{
    const float d_safe = std::max(active_params_.d_safe, active_params_.robot_radius + 1e-3f);
    const float soft_span = std::max(d_safe - active_params_.robot_radius, 1e-3f);

    float strength = 0.f;
    if (esdf_val < d_safe)
    {
        const float normalized = (d_safe - esdf_val) / soft_span;
        strength += active_params_.lambda_obstacle * normalized;
    }

    const float hard_margin = std::max(active_params_.close_obstacle_margin, 1e-3f);
    const float hard_threshold = active_params_.robot_radius + hard_margin;
    if (esdf_val < hard_threshold)
    {
        const float normalized = (hard_threshold - esdf_val) / hard_margin;
        strength += active_params_.lambda_obstacle * active_params_.close_obstacle_gain * normalized;
    }

    return strength;
}

// ============================================================================
// Public API
// ============================================================================

void TrajectoryController::set_path(const std::vector<Eigen::Vector2f>& path_room)
{
    refresh_active_params();

    path_room_ = path_room;
    wp_index_ = (path_room.size() > 1) ? 1 : 0;
    active_ = path_room.size() >= 2;

    // Push waypoints away from walls/furniture toward center of free space
    relax_path(40);

    has_prev_vel_ = false;
    smoothed_vel_ = Eigen::Vector3f::Zero();

    // Initialize MPPI state
    adaptive_K_ = active_params_.num_samples;
    adaptive_T_ = active_params_.trajectory_steps;
    adaptive_lambda_ = active_params_.mppi_lambda;
    ess_smooth_ = static_cast<float>(adaptive_K_) * active_params_.ess_initial_ratio;
    explore_ = 0.f;
    last_mppi_ms_ = 0.f;
    safety_guard_mood_cooldown_ = 0;

    prev_optimal_.assign(adaptive_T_, {0.f, 0.f});
    adaptive_sigma_adv_ = active_params_.sigma_adv;
    adaptive_sigma_rot_ = active_params_.sigma_rot;

    if (active_)
        std::cout << "[TrajectoryCtrl] Path set: " << path_room_.size() << " waypoints\n";
}

void TrajectoryController::stop()
{
    active_ = false;
    path_room_.clear();
    wp_index_ = 0;
    has_prev_vel_ = false;
    smoothed_vel_ = Eigen::Vector3f::Zero();
    prev_optimal_.clear();
    safety_guard_mood_cooldown_ = 0;
}

void TrajectoryController::set_static_obstacles(
    const std::vector<std::vector<Eigen::Vector2f>>& obstacles_room, float sample_spacing)
{
    static_obstacle_points_room_.clear();
    for (const auto& obs : obstacles_room)
    {
        if (obs.size() < 2) continue;
        const int n = static_cast<int>(obs.size());
        for (int i = 0; i < n; ++i)
        {
            const Eigen::Vector2f& a = obs[i];
            const Eigen::Vector2f& b = obs[(i + 1) % n];
            const float len = (b - a).norm();
            const int num_samples = std::max(1, static_cast<int>(len / sample_spacing));
            for (int s = 0; s <= num_samples; ++s)
            {
                const float t = static_cast<float>(s) / static_cast<float>(num_samples);
                static_obstacle_points_room_.push_back(a + t * (b - a));
            }
        }
    }
    std::cout << "[TrajectoryCtrl] Set " << obstacles_room.size() << " static obstacles → "
              << static_obstacle_points_room_.size() << " sampled points\n";
}

void TrajectoryController::set_room_boundary(const std::vector<Eigen::Vector2f>& polygon,
                                              float sample_spacing)
{
    room_boundary_points_room_.clear();
    const int n = static_cast<int>(polygon.size());
    if (n < 3) return;
    for (int i = 0; i < n; ++i)
    {
        const Eigen::Vector2f& a = polygon[i];
        const Eigen::Vector2f& b = polygon[(i + 1) % n];
        const float len = (b - a).norm();
        const int num_samples = std::max(1, static_cast<int>(len / sample_spacing));
        for (int s = 0; s <= num_samples; ++s)
        {
            const float t = static_cast<float>(s) / static_cast<float>(num_samples);
            room_boundary_points_room_.push_back(a + t * (b - a));
        }
    }
    std::cout << "[TrajectoryCtrl] Room boundary: " << n << " vertices → "
              << room_boundary_points_room_.size() << " sampled points\n";
}

// ============================================================================
// Elastic-band path relaxation: push waypoints toward center of free space
// ============================================================================
void TrajectoryController::relax_path(int iterations)
{
    if (path_room_.size() < 2) return;

    // --- Interpolate: insert points so no segment exceeds ~1 m ---
    constexpr float max_seg = 1.0f;
    std::vector<Eigen::Vector2f> dense;
    dense.push_back(path_room_.front());
    for (size_t i = 1; i < path_room_.size(); ++i)
    {
        const Eigen::Vector2f& a = path_room_[i - 1];
        const Eigen::Vector2f& b = path_room_[i];
        const float len = (b - a).norm();
        const int n_seg = std::max(1, static_cast<int>(std::ceil(len / max_seg)));
        for (int s = 1; s <= n_seg; ++s)
        {
            const float t = static_cast<float>(s) / static_cast<float>(n_seg);
            dense.push_back(a + t * (b - a));
        }
    }
    path_room_ = std::move(dense);

    const int n = static_cast<int>(path_room_.size());
    if (n < 3) return;

    // Combine room boundary + furniture points for nearest-obstacle queries
    std::vector<Eigen::Vector2f> all_obs;
    all_obs.reserve(room_boundary_points_room_.size() + static_obstacle_points_room_.size());
    all_obs.insert(all_obs.end(), room_boundary_points_room_.begin(), room_boundary_points_room_.end());
    all_obs.insert(all_obs.end(), static_obstacle_points_room_.begin(), static_obstacle_points_room_.end());

    if (all_obs.empty()) return;

    constexpr float alpha_obs    = 0.35f;   // obstacle repulsion gain per iteration
    constexpr float alpha_smooth = 0.15f;   // smoothing gain per iteration
    const float d_thresh = 1.5f;            // meters — influence radius for push
    const float min_clearance = active_params_.robot_radius + 0.05f;  // safety margin

    // Helper: minimum distance from point to any obstacle
    auto min_obs_dist = [&](const Eigen::Vector2f& p) -> float
    {
        float best = std::numeric_limits<float>::max();
        for (const auto& obs_pt : all_obs)
        {
            const float dsq = (p - obs_pt).squaredNorm();
            if (dsq < best) best = dsq;
        }
        return std::sqrt(best);
    };

    for (int iter = 0; iter < iterations; ++iter)
    {
        // Skip first and last waypoint (start & goal are fixed)
        for (int i = 1; i < n - 1; ++i)
        {
            const Eigen::Vector2f& p = path_room_[i];

            // Find nearest obstacle point
            float min_dist_sq = std::numeric_limits<float>::max();
            Eigen::Vector2f nearest = p;
            for (const auto& obs_pt : all_obs)
            {
                const float dsq = (p - obs_pt).squaredNorm();
                if (dsq < min_dist_sq) { min_dist_sq = dsq; nearest = obs_pt; }
            }
            const float min_dist = std::sqrt(min_dist_sq);

            // Obstacle repulsion: push away from nearest wall, proportional to proximity
            Eigen::Vector2f obs_force = Eigen::Vector2f::Zero();
            if (min_dist < d_thresh && min_dist > 0.01f)
            {
                const Eigen::Vector2f away = (p - nearest) / min_dist;  // unit vector away
                obs_force = away * (d_thresh - min_dist);
            }

            // Smoothing: pull toward midpoint of neighbors
            const Eigen::Vector2f midpoint = 0.5f * (path_room_[i - 1] + path_room_[i + 1]);
            const Eigen::Vector2f smooth_force = midpoint - p;

            // Candidate new position
            const Eigen::Vector2f candidate = p + alpha_obs * obs_force + alpha_smooth * smooth_force;

            // Safety check: reject move if candidate is too close to any obstacle
            if (min_obs_dist(candidate) >= min_clearance)
                path_room_[i] = candidate;
        }
    }

    std::cout << "[TrajectoryCtrl] Path relaxed: " << n << " waypoints, "
              << iterations << " iterations\n";
}

std::optional<Eigen::Vector2f> TrajectoryController::current_waypoint_room() const
{
    if (!active_ || wp_index_ >= static_cast<int>(path_room_.size()))
        return std::nullopt;
    return path_room_[wp_index_];
}

// ============================================================================
// Main compute — Proper MPPI with warm-start + Gaussian perturbations
// ============================================================================

TrajectoryController::ControlOutput TrajectoryController::compute(
    const std::vector<Eigen::Vector3f>& lidar_points,
    const Eigen::Affine2f& robot_pose)
{
    refresh_active_params();

    ControlOutput out;
    if (!active_ || path_room_.empty()) { active_ = false; out.goal_reached = true; return out; }

    const auto t_mppi_start = std::chrono::steady_clock::now();

    const int T = adaptive_T_;

    // 1. ESDF
    build_esdf(lidar_points, robot_pose);

    // 2. Advance waypoints
    advance_waypoints(robot_pose);

    // 3. Goal check
    const Eigen::Vector2f goal_robot = room_to_robot(path_room_.back(), robot_pose);
    out.dist_to_goal = goal_robot.norm();
    if (out.dist_to_goal < active_params_.goal_threshold)
    { active_ = false; out.goal_reached = true; std::cout << "[TrajectoryCtrl] Goal reached!\n"; return out; }

    // 4. Carrot
    const Eigen::Vector2f carrot_room = compute_carrot(robot_pose);
    const Eigen::Vector2f carrot_robot = room_to_robot(carrot_room, robot_pose);
    out.carrot_room = carrot_room;
    out.current_wp_index = wp_index_;
    out.min_esdf = query_esdf(0.f, 0.f);

    // ---- PD carrot-follower mode: simple proportional-derivative controller ----
    if (control_mode_ == ControlMode::PD)
        return compute_pd(out, carrot_robot, lidar_points, robot_pose);

    // 5-6. Tracking MPPI: compute the nominal (straight-to-carrot) each cycle
    //       and use it as the sampling center.  This guarantees the exploration
    //       region always covers the carrot direction, unlike warm-start which
    //       can drift to zero and never recover.
    Seed nominal_seed = compute_nominal(carrot_robot, T);

    // 7. Sample K trajectories around the nominal
    auto seeds = sample_trajectories(carrot_robot, nominal_seed);
    const int actual_K = static_cast<int>(seeds.size());

    // 8. Simulate, optimize, and score each sample
    std::vector<SimResult> results(actual_K);
    int best_idx = -1;
    float best_G = std::numeric_limits<float>::max();

    for (int k = 0; k < actual_K; ++k)
    {
        optimize_seed(seeds[k], carrot_robot);
        results[k] = simulate_and_score(seeds[k], carrot_robot, nominal_seed);
        if (results[k].G_total < best_G)
        {
            best_G = results[k].G_total;
            best_idx = k;
        }
    }

    // 9. MPPI weighted average with fixed λ
    std::vector<ControlStep> optimal(T, {0.f, 0.f});
    std::vector<ControlStep> weighted_optimal(T, {0.f, 0.f});
    float ess_current = 0.f;
    float lambda_used = adaptive_lambda_;
    int num_collisions = 0;
    {
        const float G_min = best_G;

        // Count collisions, compute cost range over non-colliding seeds
        float g_max_nc = G_min;
        for (int k = 0; k < actual_K; ++k)
        {
            if (results[k].collides)
                num_collisions++;
            else
                g_max_nc = std::max(g_max_nc, results[k].G_total);
        }
        // Adaptive floor: λ must cover the cost range so weight ratios stay ~exp(-5)
        const float g_range = g_max_nc - G_min;
        const float range_lambda = std::max(1.f, g_range / 5.f);
        lambda_used = std::max(adaptive_lambda_, range_lambda);

        std::vector<float> weights(actual_K);
        float w_sum = 0.f;

        for (int k = 0; k < actual_K; ++k)
        {
            if (results[k].collides)
            {
                weights[k] = 0.f;
            }
            else
            {
                const float exponent = std::max(-60.f, -(results[k].G_total - G_min) / std::max(lambda_used, 1e-6f));
                weights[k] = std::exp(exponent);
            }
            w_sum += weights[k];
        }

        // Compute ESS
        ess_current = compute_ess(weights, actual_K);
        ess_smooth_ = 0.75f * ess_smooth_ + 0.25f * ess_current;

        // Textbook MPPI weighted average: U_new = Σ w_k * V_k
        // Since all V_k = U + ε_k and Σ w_k = 1, this is equivalent to
        // U_new = U + Σ w_k * ε_k (perturbation-weighted update).
        if (w_sum > active_params_.weights_epsilon)
        {
            for (int k = 0; k < actual_K; ++k)
            {
                const float w = weights[k] / w_sum;
                if (w > active_params_.weights_epsilon)
                {
                    const int steps_k = static_cast<int>(seeds[k].adv.size());
                    for (int t = 0; t < std::min(T, steps_k); ++t)
                    {
                        weighted_optimal[t].adv += w * seeds[k].adv[t];
                        weighted_optimal[t].rot += w * seeds[k].rot[t];
                    }
                }
            }

            optimal = weighted_optimal;

            // Low-ESS blending with best seed:
            // when ESS/K is very low, weighted averaging tends to produce
            // indecisive slow commands in narrow passages.
            if (best_idx >= 0)
            {
                const float ess_ratio = std::clamp(ess_current / static_cast<float>(std::max(actual_K, 1)), 0.f, 1.f);
                const float blend_start = std::clamp(active_params_.ess_blend_best_start, 0.f, 1.f);
                const float blend_max = std::clamp(active_params_.ess_blend_best_max, 0.f, 1.f);
                if (blend_start > 1e-6f && ess_ratio < blend_start)
                {
                    const float low_ess_strength = (blend_start - ess_ratio) / blend_start; // 0..1
                    const float blend = std::clamp(blend_max * low_ess_strength, 0.f, 1.f);
                    const int steps_b = static_cast<int>(seeds[best_idx].adv.size());
                    const int blend_steps = std::min(T, steps_b);
                    for (int t = 0; t < blend_steps; ++t)
                    {
                        optimal[t].adv = (1.f - blend) * weighted_optimal[t].adv + blend * seeds[best_idx].adv[t];
                        optimal[t].rot = (1.f - blend) * weighted_optimal[t].rot + blend * seeds[best_idx].rot[t];
                    }
                }
            }
        }
        else if (best_idx >= 0)
        {
            // Fallback: if all weights collapsed, use best seed
            const int steps_b = static_cast<int>(seeds[best_idx].adv.size());
            for (int t = 0; t < std::min(T, steps_b); ++t)
            {
                optimal[t].adv = seeds[best_idx].adv[t];
                optimal[t].rot = seeds[best_idx].rot[t];
            }
        }
    }

    // 10. Minimal ESS-based λ adaptation
    adapt_from_ess(ess_current, actual_K, num_collisions);

    // Export ESS diagnostics
    out.ess = ess_smooth_;
    out.ess_K = adaptive_K_;
    out.explore = explore_;

    // Store optimal for smoothing reference (G_smooth term) — NOT used as
    // sampling center (that role belongs to the freshly-computed nominal).
    prev_optimal_ = optimal;

    // Wall-clock time for CPU budget tracking
    last_mppi_ms_ = std::chrono::duration<float, std::milli>(
        std::chrono::steady_clock::now() - t_mppi_start).count();

    // 11. Extract first-step command
    float cmd_adv = optimal[0].adv;
    float cmd_rot = optimal[0].rot;

    // 12. Viz: trajectories in room frame (subsample for drawing)
    {
        const Eigen::Matrix2f R = robot_pose.linear();
        const Eigen::Vector2f t_pos = robot_pose.translation();

        const int requested_draw = std::max(1, active_params_.num_trajectories_to_draw);
        const int n_draw = std::min(requested_draw, actual_K);
        out.trajectories_room.resize(n_draw);
        for (int i = 0; i < n_draw; ++i)
        {
            int k = (i == 0 && best_idx >= 0) ? best_idx
                    : (i * actual_K) / std::max(n_draw, 1);
            k = std::clamp(k, 0, actual_K - 1);
            auto& tr = out.trajectories_room[i];
            tr.reserve(results[k].positions.size() + 1);
            tr.push_back(t_pos);
            for (const auto& p : results[k].positions)
                tr.push_back(R * p + t_pos);
        }
        out.best_trajectory_idx = (n_draw > 0) ? 0 : -1;
    }

    // 13. Smooth + Gaussian brake
    const float eff_smoothing = active_params_.velocity_smoothing;

    Eigen::Vector3f raw(cmd_adv, 0.f, cmd_rot);
    if (has_prev_vel_)
        smoothed_vel_ = eff_smoothing * smoothed_vel_ + (1.f - eff_smoothing) * raw;
    else { smoothed_vel_ = raw; has_prev_vel_ = true; }

    const float rot_ratio = smoothed_vel_[2] / active_params_.max_rot;
    const float brake = std::exp(-active_params_.gauss_k * rot_ratio * rot_ratio);

    out.adv  = smoothed_vel_[0] * brake;
    out.side = smoothed_vel_[1];
    out.rot  = smoothed_vel_[2];

    // 14. Safety gate on top of MPPI output (short forward prediction on inflated ESDF)
    {
        constexpr float gate_horizon_s = 0.30f;     // shorter look-ahead to reduce false positives
        constexpr float gate_inflate_m = 0.03f;       // tighter soft margin (was 0.06)
        constexpr float gate_hard_margin_m = 0.01f;
        constexpr int gate_soft_consecutive_needed = 5; // need 5 consecutive close steps (was 3)
        const float gate_dt = std::max(0.03f, active_params_.trajectory_dt);

        if (safety_guard_mood_cooldown_ > 0)
            safety_guard_mood_cooldown_--;

        struct RiskEval
        {
            bool trigger = false;
            bool hard_collision = false;
            float min_esdf = std::numeric_limits<float>::infinity();
        };

        auto eval_risk = [&](float adv, float rot, float horizon_s)
        {
            RiskEval r;
            float x = 0.f, y = 0.f, theta = 0.f;
            const int steps = std::max(1, static_cast<int>(std::ceil(horizon_s / gate_dt)));
            const float soft_thresh = active_params_.robot_radius + gate_inflate_m;
            const float hard_thresh = active_params_.robot_radius + gate_hard_margin_m;
            int soft_consecutive = 0;

            for (int i = 0; i < steps; ++i)
            {
                x += adv * std::sin(theta) * gate_dt;
                y += adv * std::cos(theta) * gate_dt;
                theta += rot * gate_dt;

                const float d = query_esdf(x, y);
                r.min_esdf = std::min(r.min_esdf, d);

                if (d < hard_thresh)
                {
                    r.trigger = true;
                    r.hard_collision = true;
                    return r;
                }

                if (d < soft_thresh)
                {
                    soft_consecutive++;
                    if (soft_consecutive >= gate_soft_consecutive_needed)
                    {
                        r.trigger = true;
                        return r;
                    }
                }
                else
                {
                    soft_consecutive = 0;
                }
            }
            return r;
        };

        // Simpler arming from direct LiDAR measurements (robot frame):
        // trigger only when an obstacle is close in a frontal cone.
        constexpr float lidar_front_cone_rad = 0.40f; // ~23 deg (was 31.5°) — narrower frontal cone
        const float lidar_trigger_dist = active_params_.d_safe + 0.08f;  // tighter arming (was +0.20)
        const float lidar_min_valid_dist = active_params_.robot_radius + 0.08f;
        constexpr int lidar_min_points_to_arm = 5;  // need 5 points (was 3)
        bool gate_armed = false;
        int frontal_close_count = 0;
        for (const auto &p : lidar_points)
        {
            const float px = p.x();
            const float py = p.y(); // forward axis
            if (py <= 0.f) continue; // front only

            const float d = std::hypot(px, py);
            if (d < lidar_min_valid_dist) continue; // ignore near-body noise returns

            const float ang = std::abs(std::atan2(px, py));
            if (ang <= lidar_front_cone_rad && d < lidar_trigger_dist)
            {
                frontal_close_count++;
                if (frontal_close_count >= lidar_min_points_to_arm)
                {
                    gate_armed = true;
                    break;
                }
            }
        }
        const auto nominal_risk = gate_armed ? eval_risk(out.adv, out.rot, gate_horizon_s) : RiskEval{};

        if (nominal_risk.trigger)
        {
            out.safety_guard_triggered = true;
            const float base_rot = out.rot;
            bool accepted = false;

            // Back up while turning toward free space
            float preferred_sign = 1.f;
            const Eigen::Vector2f grad0 = query_esdf_gradient(0.f, 0.f);
            if (grad0.norm() > active_params_.esdf_grad_min_norm)
                preferred_sign = (grad0.x() >= 0.f) ? 1.f : -1.f;
            else if (std::abs(carrot_robot.x()) > 1e-3f)
                preferred_sign = (carrot_robot.x() >= 0.f) ? 1.f : -1.f;
            else if (std::abs(base_rot) > 1e-3f)
                preferred_sign = (base_rot >= 0.f) ? 1.f : -1.f;

            const float backup_adv = -std::max(0.05f, std::min(active_params_.max_back_adv, 0.35f * active_params_.max_adv));
            const float backup_rot = preferred_sign * std::max(0.12f, 0.35f * active_params_.max_rot);

            if (!eval_risk(backup_adv, backup_rot, gate_horizon_s).trigger)
            {
                out.adv = backup_adv;
                out.rot = backup_rot;
                accepted = true;
            }
            else if (!eval_risk(backup_adv, 0.f, gate_horizon_s).trigger)
            {
                out.adv = backup_adv;
                out.rot = 0.f;
                accepted = true;
            }

            // Try reducing forward speed instead of full backup
            for (float scale : {0.6f, 0.35f, 0.15f, 0.f})
            {
                if (accepted) break;
                const float test_adv = out.adv * scale;
                if (!eval_risk(test_adv, base_rot, gate_horizon_s).trigger)
                {
                    out.adv = test_adv;
                    out.rot = base_rot;
                    accepted = true;
                    break;
                }
            }

            if (!accepted)
            {
                out.adv = 0.f;
                out.rot = preferred_sign * std::max(0.15f, 0.7f * active_params_.max_rot);

                if (query_esdf(0.f, 0.f) < active_params_.robot_radius + 0.01f)
                    out.rot = 0.f;
            }
        }
    }

    // Debug (every ~1 second)
    static int dbg = 0;
    if (++dbg % std::max(1, active_params_.debug_print_period) == 0)
    {
        std::cout << "[MPPI] K=" << adaptive_K_ << " T=" << adaptive_T_
                  << " λ=" << std::fixed << std::setprecision(1) << adaptive_lambda_
                  << " λw=" << std::setprecision(1) << lambda_used
                  << " ESS=" << std::setprecision(0) << ess_smooth_ << "/" << adaptive_K_
                  << " E=" << std::setprecision(2) << explore_
                  << " ncol=" << num_collisions << "/" << actual_K
                  << " σr=" << std::setprecision(2) << adaptive_sigma_rot_
                  << " G=" << std::setprecision(1) << best_G
                  << " ms=" << std::setprecision(1) << last_mppi_ms_
                  << " cmd(" << std::setprecision(2) << out.adv << "," << out.rot << ")"
                  << " dist=" << std::setprecision(1) << out.dist_to_goal
                  << " esdf=" << out.min_esdf << "\n";
    }

    return out;
}

// ============================================================================
// PD carrot-follower: simple proportional-derivative controller
//
// rot = Kp * angle_error + Kd * d(angle_error)/dt
// adv = max_adv * cos(angle_error) * dist_factor
//
// Uses the same ESDF, carrot, safety gate, gaussian brake and smoothing
// as the MPPI mode but replaces all the sampling/weighting logic.
// ============================================================================

TrajectoryController::ControlOutput TrajectoryController::compute_pd(
    ControlOutput& out,
    const Eigen::Vector2f& carrot_robot,
    const std::vector<Eigen::Vector3f>& /*lidar_points*/,
    const Eigen::Affine2f& /*robot_pose*/)
{
    // Angle to carrot in robot frame (Y+ = forward, X+ = right)
    const float angle_err = std::atan2(carrot_robot.x(), carrot_robot.y());
    const float carrot_dist = carrot_robot.norm();

    // PD rotation command
    const float d_err = angle_err - prev_angle_err_;
    prev_angle_err_ = angle_err;

    float cmd_rot = active_params_.pd_Kp_rot * angle_err
                  + active_params_.pd_Kd_rot * d_err;
    cmd_rot = std::clamp(cmd_rot, -active_params_.max_rot, active_params_.max_rot);

    // Forward speed: proportional to alignment, reduced near goal
    const float alignment = std::pow(std::max(0.f, std::cos(angle_err)),
                                     active_params_.pd_speed_cos_power);
    float dist_factor = std::min(1.f, carrot_dist / std::max(active_params_.nominal_goal_dist_scale, 0.1f));
    float cmd_adv = active_params_.max_adv * alignment * dist_factor;
    cmd_adv = std::clamp(cmd_adv, active_params_.min_adv_cmd, active_params_.max_adv);

    // Smoothing + Gaussian brake (same as MPPI path)
    Eigen::Vector3f raw(cmd_adv, 0.f, cmd_rot);
    if (has_prev_vel_)
        smoothed_vel_ = active_params_.velocity_smoothing * smoothed_vel_
                      + (1.f - active_params_.velocity_smoothing) * raw;
    else { smoothed_vel_ = raw; has_prev_vel_ = true; }

    const float rot_ratio = smoothed_vel_[2] / active_params_.max_rot;
    const float brake = std::exp(-active_params_.gauss_k * rot_ratio * rot_ratio);

    out.adv  = smoothed_vel_[0] * brake;
    out.side = smoothed_vel_[1];
    out.rot  = smoothed_vel_[2];

    // Safety gate (same ESDF-based forward prediction as MPPI mode)
    {
        constexpr float gate_horizon_s = 0.30f;
        constexpr float gate_inflate_m = 0.03f;
        constexpr float gate_hard_margin_m = 0.01f;
        constexpr int gate_soft_consecutive_needed = 5;
        const float gate_dt = std::max(0.03f, active_params_.trajectory_dt);

        auto eval_risk = [&](float adv, float rot, float horizon_s)
        {
            struct R { bool trigger = false; bool hard_collision = false; float min_esdf = 1e9f; };
            R r;
            float x = 0.f, y = 0.f, theta = 0.f;
            const int steps = std::max(1, static_cast<int>(std::ceil(horizon_s / gate_dt)));
            const float soft_thresh = active_params_.robot_radius + gate_inflate_m;
            const float hard_thresh = active_params_.robot_radius + gate_hard_margin_m;
            int soft_consecutive = 0;
            for (int i = 0; i < steps; ++i)
            {
                x += adv * std::sin(theta) * gate_dt;
                y += adv * std::cos(theta) * gate_dt;
                theta += rot * gate_dt;
                const float d = query_esdf(x, y);
                r.min_esdf = std::min(r.min_esdf, d);
                if (d < hard_thresh) { r.trigger = true; r.hard_collision = true; return r; }
                if (d < soft_thresh) { if (++soft_consecutive >= gate_soft_consecutive_needed) { r.trigger = true; return r; } }
                else soft_consecutive = 0;
            }
            return r;
        };

        auto risk = eval_risk(out.adv, out.rot, gate_horizon_s);
        if (risk.trigger)
        {
            out.safety_guard_triggered = true;
            // Try reducing speed first
            for (float scale : {0.5f, 0.25f, 0.f})
            {
                if (!eval_risk(out.adv * scale, out.rot, gate_horizon_s).trigger)
                { out.adv *= scale; return out; }
            }
            // Full stop + rotate away from closest obstacle
            out.adv = 0.f;
            float sign = (out.rot >= 0.f) ? 1.f : -1.f;
            out.rot = sign * 0.5f * active_params_.max_rot;
        }
    }

    // Debug
    static int dbg_pd = 0;
    if (++dbg_pd % std::max(1, active_params_.debug_print_period) == 0)
    {
        std::cout << "[PD] angle=" << std::fixed << std::setprecision(2) << angle_err
                  << " cmd(" << out.adv << "," << out.rot << ")"
                  << " dist=" << std::setprecision(1) << out.dist_to_goal
                  << " esdf=" << out.min_esdf << "\n";
    }

    return out;
}

// ============================================================================
// Nominal control: simple proportional control toward carrot
// Used as the base for MPPI warm-start blending
// ============================================================================

TrajectoryController::Seed TrajectoryController::compute_nominal(
    const Eigen::Vector2f& carrot_robot, int steps) const
{
    Seed seed;
    seed.adv.resize(steps);
    seed.rot.resize(steps);

    const float dt = active_params_.trajectory_dt;
    // Y+ forward: atan2(x, y) gives angle from forward axis
    const float carrot_angle = std::atan2(carrot_robot.x(), carrot_robot.y());
    const float carrot_dist = carrot_robot.norm();
    float theta = 0.f;

    for (int s = 0; s < steps; ++s)
    {
        // Remaining angle to carrot from current simulated heading
        float angle_err = carrot_angle - theta;
        while (angle_err > static_cast<float>(M_PI)) angle_err -= 2.f * static_cast<float>(M_PI);
        while (angle_err < -static_cast<float>(M_PI)) angle_err += 2.f * static_cast<float>(M_PI);

        // PD-like proportional gain instead of bang-bang (angle_err/dt).
        // angle_err/dt amplifies a 6° offset to 0.7 rad/s (clamped max_rot),
        // producing 5-10x more rotation than the PD controller (Kp=2.0).
        // This was the root cause of mean_rot ≈ 0.3 in all MPPI variants:
        // ALL seeds are centered around an overly-aggressive nominal.
        const float Kp_nom = 2.5f;  // slightly above PD's Kp=2.0 for MPPI margin
        float rot_cmd = std::clamp(Kp_nom * angle_err,
                                   -active_params_.max_rot, active_params_.max_rot);

        // Forward speed: proportional to alignment with carrot
        float alignment = std::cos(angle_err);
        float adv_cmd = active_params_.max_adv * std::max(active_params_.nominal_alignment_floor, alignment);
        // Reduce speed near goal
        float dist_factor = std::min(1.f, carrot_dist / std::max(active_params_.nominal_goal_dist_scale, 1e-6f));
        adv_cmd *= dist_factor;
        adv_cmd = std::clamp(adv_cmd, active_params_.min_adv_cmd, active_params_.max_adv);

        seed.adv[s] = adv_cmd;
        seed.rot[s] = rot_cmd;

        theta += rot_cmd * dt;
    }
    return seed;
}

// ============================================================================
// Sample K trajectories — Tracking MPPI
//
// The sampling center is the freshly-computed nominal (straight-to-carrot),
// NOT the shifted previous solution.  This guarantees that:
//   1. The exploration region always contains the carrot direction
//   2. No warm-start death spiral (prev_optimal_ starting at zero)
//   3. The nominal adapts instantly to carrot/path changes (corners)
//
// Structure:
//   Seed 0         = nominal (zero perturbation)
//   Seeds 1..N_inj = structured exploration at wide angles (±30°, ±60°, ±90°)
//   Seeds N_inj+1..K-1 = nominal + i.i.d. Gaussian perturbations
//
// The exploration seeds use the same PD-like gain as the nominal (Kp=2.5)
// to avoid the bang-bang oscillation that angle_err/dt caused.
// In open space, rot_cost_factor=40 ensures they get near-zero weight.
// Near obstacles, they provide the wide-angle coverage needed to find detours.
// ============================================================================

std::vector<TrajectoryController::Seed> TrajectoryController::sample_trajectories(
    const Eigen::Vector2f& carrot_robot, const Seed& nominal)
{
    const int K = adaptive_K_;
    const int T = adaptive_T_;
    const int nom_T = static_cast<int>(nominal.adv.size());
    const float dt = active_params_.trajectory_dt;
    const float carrot_angle = std::atan2(carrot_robot.x(), carrot_robot.y());
    const float carrot_dist = carrot_robot.norm();

    std::vector<Seed> seeds;
    seeds.reserve(K);

    // --- Seed 0: the nominal itself (zero perturbation) --------------------
    Seed nominal_copy = nominal;
    nominal_copy.use_info_correction = false;
    seeds.push_back(std::move(nominal_copy));

    // --- Structured exploration seeds at wide angles -----------------------
    // Always inject 6 seeds: ±30°, ±60°, ±90° offsets from carrot direction.
    // These provide coarse coverage for detour discovery around obstacles.
    // With rot_cost_factor=40 they are heavily penalized in open space
    // (near-zero weight), so they don't cause oscillation.
    const std::array<float, 6> offsets = {
        active_params_.inject_offset_30, -active_params_.inject_offset_30,
        active_params_.inject_offset_60, -active_params_.inject_offset_60,
        active_params_.inject_offset_90, -active_params_.inject_offset_90
    };

    const float Kp_nom = 2.5f;  // same PD-like gain as compute_nominal

    for (float offset : offsets)
    {
        Seed s;
        s.adv.resize(T);
        s.rot.resize(T);
        s.use_info_correction = false;
        float theta = 0.f;

        for (int t = 0; t < T; ++t)
        {
            // Target heading: gradually reduce offset over time (converge to carrot)
            float phase = static_cast<float>(t) / static_cast<float>(std::max(T - 1, 1));
            float target_heading = carrot_angle + offset * (1.f - phase);

            float angle_err = target_heading - theta;
            while (angle_err > static_cast<float>(M_PI)) angle_err -= 2.f * static_cast<float>(M_PI);
            while (angle_err < -static_cast<float>(M_PI)) angle_err += 2.f * static_cast<float>(M_PI);

            // PD-like proportional gain — NOT bang-bang (angle_err/dt)
            float rot_cmd = std::clamp(Kp_nom * angle_err,
                                       -active_params_.max_rot, active_params_.max_rot);

            // Reduced forward speed for exploration
            float alignment = std::cos(angle_err);
            float adv_cmd = active_params_.max_adv * active_params_.injection_adv_scale
                          * std::max(active_params_.nominal_alignment_floor, alignment);
            float dist_factor = std::min(1.f, carrot_dist / std::max(active_params_.nominal_goal_dist_scale, 1e-6f));
            adv_cmd *= dist_factor;
            adv_cmd = std::clamp(adv_cmd, active_params_.min_adv_cmd, active_params_.max_adv);

            s.adv[t] = adv_cmd;
            s.rot[t] = rot_cmd;
            theta += rot_cmd * dt;
        }
        seeds.push_back(std::move(s));
    }

    // --- Remaining seeds: nominal + i.i.d. Gaussian perturbations ----------
    const int n_random = std::max(0, K - static_cast<int>(seeds.size()));
    for (int k = 0; k < n_random; ++k)
    {
        Seed s;
        s.adv.resize(T);
        s.rot.resize(T);
        s.use_info_correction = true;
        for (int t = 0; t < T; ++t)
        {
            const float base_adv = (t < nom_T) ? nominal.adv[t] : 0.f;
            const float base_rot = (t < nom_T) ? nominal.rot[t] : 0.f;

            const float eps_adv = normal_(rng_) * adaptive_sigma_adv_;
            const float eps_rot = normal_(rng_) * adaptive_sigma_rot_;

            s.adv[t] = std::clamp(base_adv + eps_adv,
                                  active_params_.min_adv_cmd, active_params_.max_adv);
            s.rot[t] = std::clamp(base_rot + eps_rot,
                                  -active_params_.max_rot, active_params_.max_rot);
        }
        seeds.push_back(std::move(s));
    }

    return seeds;
}

// ============================================================================
// Forward simulate a seed and compute EFE score
// ============================================================================

TrajectoryController::SimResult TrajectoryController::simulate_and_score(
    const Seed& seed, const Eigen::Vector2f& carrot_robot, const Seed& nominal)
{
    SimResult res;
    const int steps = static_cast<int>(seed.adv.size());
    const float dt = active_params_.trajectory_dt;
    res.positions.reserve(steps);

    float x = 0.f, y = 0.f, theta = 0.f;
    float G_obs_total = 0.f;
    float G_progress = 0.f;
    const float discount = active_params_.cost_discount;
    float discount_acc = 1.f;
    float prev_dist_to_carrot = carrot_robot.norm();
    int actual_steps = 0;

    // Fix 1: Compute the scoring horizon — stop accumulating goal/progress cost
    // once the trajectory would overshoot the carrot.  We still simulate all T
    // steps for obstacle scoring and positions, but the goal-related costs use
    // the *closest-approach* point as the effective endpoint.
    float best_dist_to_carrot = carrot_robot.norm();
    int best_step = 0;  // step index of closest approach to carrot

    for (int s = 0; s < steps; ++s)
    {
        // Differential-drive kinematics (Y+ = forward, X+ = right)
        x += seed.adv[s] * std::sin(theta) * dt;
        y += seed.adv[s] * std::cos(theta) * dt;
        theta += seed.rot[s] * dt;

        res.positions.emplace_back(x, y);
        actual_steps = s + 1;

        // Track closest approach to carrot
        float cur_dist = (Eigen::Vector2f(x, y) - carrot_robot).norm();
        if (cur_dist < best_dist_to_carrot)
        {
            best_dist_to_carrot = cur_dist;
            best_step = s;
        }

        // Per-step progress: only penalize moving away BEFORE closest approach
        if (s <= best_step)
        {
            float step_progress = prev_dist_to_carrot - cur_dist;
            G_progress += discount_acc * std::max(0.f, -step_progress);
        }
        prev_dist_to_carrot = cur_dist;

        float esdf_val = query_esdf(x, y);
        res.min_esdf = std::min(res.min_esdf, esdf_val);

        const float G_obs = obstacle_step_cost(esdf_val);

        G_obs_total += discount_acc * G_obs;
        discount_acc *= discount;

        // Cut trajectory at collision
        if (esdf_val < active_params_.robot_radius)
        {
            res.collides = true;
            break;
        }
    }

    // G_goal: use the closest-approach point, not the far-future endpoint.
    // This prevents overshoot from dominating the cost when horizon >> carrot distance.
    const float initial_dist = carrot_robot.norm();
    const float progress = initial_dist - best_dist_to_carrot;
    float G_goal = active_params_.lambda_goal
                 * (best_dist_to_carrot + active_params_.lambda_progress * std::max(0.f, -progress));

    // Fix 2: Heading cost — penalize initial angular divergence from carrot direction.
    // The old heading-at-endpoint was broken: trajectories that overshoot the carrot
    // face backward (heading error ≈ π), making the term non-discriminating.
    // Instead, measure how well each seed initially steers toward the carrot.
    const float carrot_angle = std::atan2(carrot_robot.x(), carrot_robot.y()); // Y+ forward
    {
        // Weighted average of angular error over the first few steps (not just step 0)
        // to capture the seed's rotational tendency, not just a single-step noise sample.
        const int heading_window = std::min(5, actual_steps);
        float heading_err_acc = 0.f;
        float hw_theta = 0.f;
        for (int s = 0; s < heading_window; ++s)
        {
            hw_theta += seed.rot[s] * dt;
            float err = std::abs(carrot_angle - hw_theta);
            if (err > static_cast<float>(M_PI)) err = 2.f * static_cast<float>(M_PI) - err;
            heading_err_acc += err;
        }
        heading_err_acc /= std::max(1, heading_window);
        G_goal += active_params_.lambda_heading * active_params_.lambda_goal * heading_err_acc;
    }

    // G_smooth: continuity with warm-started baseline
    float dv = seed.adv[0] - (prev_optimal_.empty() ? 0.f : prev_optimal_[0].adv);
    float dr = seed.rot[0] - (prev_optimal_.empty() ? 0.f : prev_optimal_[0].rot);
    float G_smooth = active_params_.lambda_smooth * (dv * dv + dr * dr);

    // G_velocity: magnitude regularization + action change penalty
    float G_vel_mag = 0.f;
    float G_vel_delta = 0.f;
    for (int s = 0; s < actual_steps; ++s)
    {
        G_vel_mag += seed.adv[s] * seed.adv[s]
                   + active_params_.rot_cost_factor * seed.rot[s] * seed.rot[s];
        if (s > 0)
        {
            float da = seed.adv[s] - seed.adv[s - 1];
            float dro = seed.rot[s] - seed.rot[s - 1];
            G_vel_delta += da * da + active_params_.rot_cost_factor * dro * dro;
        }
    }
    G_vel_mag *= active_params_.lambda_velocity;
    G_vel_delta *= active_params_.lambda_delta_vel;

    // Information-theoretic correction (Williams et al. 2017):
    // S_k += λ · Σ_t  u_t^T · Σ^{-1} · ε_t
    // where u_t is the sampling mean (nominal), and ε_t = seed_t - nominal_t.
    // IMPORTANT: using seed_t here (u+ε) introduces an extra ε²/σ² bias term,
    // which can dominate costs and collapse ESS in narrow passages.
    float G_info = 0.f;
    if (seed.use_info_correction)
    {
        const float inv_var_adv = 1.f / std::max(adaptive_sigma_adv_ * adaptive_sigma_adv_, 1e-8f);
        const float inv_var_rot = 1.f / std::max(adaptive_sigma_rot_ * adaptive_sigma_rot_, 1e-8f);
        const int nom_steps = static_cast<int>(nominal.adv.size());
        for (int s = 0; s < actual_steps; ++s)
        {
            const float nom_adv = (s < nom_steps) ? nominal.adv[s] : 0.f;
            const float nom_rot = (s < nom_steps) ? nominal.rot[s] : 0.f;
            const float eps_adv = seed.adv[s] - nom_adv;
            const float eps_rot = seed.rot[s] - nom_rot;
            G_info += nom_adv * eps_adv * inv_var_adv
                    + nom_rot * eps_rot * inv_var_rot;
        }
        G_info *= adaptive_lambda_;  // λ temperature
    }

    res.G_total = G_goal + G_obs_total + G_smooth + G_vel_mag + G_vel_delta + G_info
                + active_params_.lambda_goal * G_progress
                + (res.collides ? active_params_.collision_penalty : 0.f);
    return res;
}

// ============================================================================
// Gradient optimization of a seed using ESDF gradient
// ============================================================================

void TrajectoryController::optimize_seed(Seed& seed, const Eigen::Vector2f& carrot_robot)
{
    const int steps = static_cast<int>(seed.adv.size());
    const float dt = active_params_.trajectory_dt;
    const float lr = active_params_.optim_lr;

    for (int iter = 0; iter < active_params_.optim_iterations; ++iter)
    {
        std::vector<float> px(steps), py(steps), ptheta(steps);
        float x = 0.f, y = 0.f, theta = 0.f;

        for (int s = 0; s < steps; ++s)
        {
            x += seed.adv[s] * std::sin(theta) * dt;
            y += seed.adv[s] * std::cos(theta) * dt;
            theta += seed.rot[s] * dt;
            px[s] = x; py[s] = y; ptheta[s] = theta;
        }

        for (int s = 0; s < steps; ++s)
        {
            Eigen::Vector2f correction = Eigen::Vector2f::Zero();

            // Goal: pull towards carrot
            Eigen::Vector2f to_carrot = carrot_robot - Eigen::Vector2f(px[s], py[s]);
            float dist_c = to_carrot.norm();
            if (dist_c > active_params_.heading_norm_epsilon)
                correction += active_params_.lambda_goal * to_carrot.normalized()
                           * std::min(dist_c, active_params_.optimize_goal_pull_dist_cap);

            // ESDF: push away from obstacles
            // Cap the obstacle correction magnitude so it cannot overpower the goal pull.
            // This prevents trajectories from being pushed backward at narrow passages.
            float esdf_val = query_esdf(px[s], py[s]);
            if (esdf_val < active_params_.d_safe)
            {
                Eigen::Vector2f grad = query_esdf_gradient(px[s], py[s]);
                Eigen::Vector2f obs_correction = obstacle_repulsion_strength(esdf_val) * grad;
                // Cap obstacle correction to at most 2x the goal correction magnitude
                float goal_mag = correction.norm();
                float obs_mag = obs_correction.norm();
                if (obs_mag > goal_mag * active_params_.optimize_obstacle_cap_ratio
                    && goal_mag > active_params_.optimize_goal_min_norm)
                    obs_correction *= (goal_mag * active_params_.optimize_obstacle_cap_ratio) / obs_mag;
                correction += obs_correction;
            }

            // Jacobians (Y+ forward, X+ right)
            float ct = std::cos(ptheta[s]);
            float st = std::sin(ptheta[s]);
            Eigen::Vector2f d_pos_d_adv(st * dt, ct * dt);
            // Limit the remaining-time factor to avoid wild corrections on early steps
            float remaining = std::min(static_cast<float>(steps - s) * dt,
                                       active_params_.optimize_remaining_cap_steps * dt);
            Eigen::Vector2f d_pos_d_rot(seed.adv[s] * ct * remaining * dt,
                                        -seed.adv[s] * st * remaining * dt);

            seed.adv[s] += lr * correction.dot(d_pos_d_adv);
            seed.rot[s] += lr * correction.dot(d_pos_d_rot);

            // Keep advance strictly positive (no backward motion)
            seed.adv[s] = std::clamp(seed.adv[s], active_params_.min_adv_cmd, active_params_.max_adv);
            seed.rot[s] = std::clamp(seed.rot[s], -active_params_.max_rot, active_params_.max_rot);
        }
    }
}

// ============================================================================
// Carrot computation
// ============================================================================

Eigen::Vector2f TrajectoryController::compute_carrot(const Eigen::Affine2f& robot_pose) const
{
    const Eigen::Vector2f robot_pos = robot_pose.translation();
    float min_dist_sq = std::numeric_limits<float>::max();
    int closest_seg = 0;
    float closest_t = 0.f;

    for (int i = 0; i + 1 < static_cast<int>(path_room_.size()); ++i)
    {
        const Eigen::Vector2f a = path_room_[i];
        const Eigen::Vector2f ab = path_room_[i + 1] - a;
        const float ab_sq = ab.squaredNorm();
        float t = 0.f;
        if (ab_sq > 1e-6f)
            t = std::clamp((robot_pos - a).dot(ab) / ab_sq, 0.f, 1.f);
        const Eigen::Vector2f proj = a + t * ab;
        const float d_sq = (robot_pos - proj).squaredNorm();
        if (d_sq < min_dist_sq) { min_dist_sq = d_sq; closest_seg = i; closest_t = t; }
    }

    float remaining = active_params_.carrot_lookahead;
    {
        const Eigen::Vector2f ab = path_room_[closest_seg + 1] - path_room_[closest_seg];
        const float seg_remaining = (1.f - closest_t) * ab.norm();
        if (seg_remaining >= remaining)
            return path_room_[closest_seg] + std::min(closest_t + remaining / std::max(ab.norm(), 1e-6f), 1.f) * ab;
        remaining -= seg_remaining;
    }
    for (int i = closest_seg + 1; i + 1 < static_cast<int>(path_room_.size()); ++i)
    {
        const Eigen::Vector2f ab = path_room_[i + 1] - path_room_[i];
        const float seg_len = ab.norm();
        if (seg_len >= remaining) return path_room_[i] + (remaining / seg_len) * ab;
        remaining -= seg_len;
    }
    return path_room_.back();
}

// ============================================================================
// Waypoint advancement
// ============================================================================

void TrajectoryController::advance_waypoints(const Eigen::Affine2f& robot_pose)
{
    const Eigen::Vector2f robot_pos = robot_pose.translation();
    const int last = static_cast<int>(path_room_.size()) - 1;
    while (wp_index_ < last)
    {
        const float dist = (robot_pos - path_room_[wp_index_]).norm();
        if (dist < active_params_.carrot_lookahead * active_params_.waypoint_advance_lookahead_factor) { wp_index_++; continue; }
        if (wp_index_ > 0)
        {
            const Eigen::Vector2f seg = path_room_[wp_index_] - path_room_[wp_index_ - 1];
            const float seg_len = seg.norm();
            if (seg_len > active_params_.segment_length_epsilon)
            {
                const float proj = (robot_pos - path_room_[wp_index_ - 1]).dot(seg / seg_len);
                if (proj > seg_len) { wp_index_++; continue; }
            }
        }
        break;
    }
}

// ============================================================================
// ESDF
// ============================================================================

void TrajectoryController::build_esdf(const std::vector<Eigen::Vector3f>& lidar_points,
                                      const Eigen::Affine2f& robot_pose)
{
    const float res = active_params_.grid_resolution;
    const float half = active_params_.grid_half_size;
    const int N = static_cast<int>(2.f * half / res);
    esdf_N_ = N;

    std::vector<int> occ(N * N, 0);

    // Mark lidar points (already in robot frame)
    for (const auto& p : lidar_points)
    {
        const int ci = static_cast<int>((p.x() + half) / res);
        const int cj = static_cast<int>((p.y() + half) / res);
        if (ci >= 0 && ci < N && cj >= 0 && cj < N)
            occ[cj * N + ci] = 1;
    }

    // Inject static obstacle points (furniture) — transform from room frame to robot frame
    if (!static_obstacle_points_room_.empty())
    {
        const Eigen::Affine2f robot_inv = robot_pose.inverse();
        for (const auto& p_room : static_obstacle_points_room_)
        {
            const Eigen::Vector2f p_robot = robot_inv * p_room;
            const int ci = static_cast<int>((p_robot.x() + half) / res);
            const int cj = static_cast<int>((p_robot.y() + half) / res);
            if (ci >= 0 && ci < N && cj >= 0 && cj < N)
                occ[cj * N + ci] = 1;
        }
    }

    esdf_data_.assign(N * N, active_params_.esdf_init_distance);

    // Forward pass
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i)
        {
            const int idx = j * N + i;
            if (occ[idx]) { esdf_data_[idx] = 0.f; continue; }
            float d = active_params_.esdf_init_distance;
            if (i > 0)            d = std::min(d, esdf_data_[idx - 1] + 1.f);
            if (j > 0)            d = std::min(d, esdf_data_[(j-1)*N + i] + 1.f);
            if (i > 0 && j > 0)   d = std::min(d, esdf_data_[(j-1)*N + (i-1)] + active_params_.esdf_diag_step);
            if (i < N-1 && j > 0) d = std::min(d, esdf_data_[(j-1)*N + (i+1)] + active_params_.esdf_diag_step);
            esdf_data_[idx] = d;
        }

    // Backward pass
    for (int j = N-1; j >= 0; --j)
        for (int i = N-1; i >= 0; --i)
        {
            const int idx = j * N + i;
            float d = esdf_data_[idx];
            if (i < N-1)              d = std::min(d, esdf_data_[idx + 1] + 1.f);
            if (j < N-1)              d = std::min(d, esdf_data_[(j+1)*N + i] + 1.f);
            if (i < N-1 && j < N-1)   d = std::min(d, esdf_data_[(j+1)*N + (i+1)] + active_params_.esdf_diag_step);
            if (i > 0   && j < N-1)   d = std::min(d, esdf_data_[(j+1)*N + (i-1)] + active_params_.esdf_diag_step);
            esdf_data_[idx] = d;
        }

    for (auto& d : esdf_data_) d *= res;
}

float TrajectoryController::query_esdf(float rx, float ry) const
{
    if (esdf_data_.empty()) return active_params_.esdf_unknown_distance;
    const float res = active_params_.grid_resolution;
    const float half = active_params_.grid_half_size;
    const int N = esdf_N_;

    const float gx = (rx + half) / res;
    const float gy = (ry + half) / res;
    const int ix = static_cast<int>(std::floor(gx));
    const int iy = static_cast<int>(std::floor(gy));
    if (ix < 0 || ix >= N-1 || iy < 0 || iy >= N-1) return active_params_.esdf_unknown_distance;

    const float fx = gx - static_cast<float>(ix);
    const float fy = gy - static_cast<float>(iy);
    const float d00 = esdf_data_[iy * N + ix];
    const float d10 = esdf_data_[iy * N + ix + 1];
    const float d01 = esdf_data_[(iy+1) * N + ix];
    const float d11 = esdf_data_[(iy+1) * N + ix + 1];
    return (1.f-fx)*(1.f-fy)*d00 + fx*(1.f-fy)*d10 + (1.f-fx)*fy*d01 + fx*fy*d11;
}

Eigen::Vector2f TrajectoryController::query_esdf_gradient(float rx, float ry) const
{
    const float h = active_params_.grid_resolution;
    const float dx = query_esdf(rx + h, ry) - query_esdf(rx - h, ry);
    const float dy = query_esdf(rx, ry + h) - query_esdf(rx, ry - h);
    Eigen::Vector2f grad(dx / (2.f * h), dy / (2.f * h));
    const float mag = grad.norm();
    if (mag > active_params_.esdf_grad_min_norm) grad /= mag;
    else grad = Eigen::Vector2f::Zero();
    return grad;
}

// ============================================================================
// Coordinate transform
// ============================================================================

Eigen::Vector2f TrajectoryController::room_to_robot(const Eigen::Vector2f& p_room,
                                                     const Eigen::Affine2f& robot_pose)
{
    return robot_pose.linear().transpose() * (p_room - robot_pose.translation());
}

// ============================================================================
// ESS computation
// ============================================================================

float TrajectoryController::compute_ess(const std::vector<float>& weights, int K) const
{
    float w_sum = 0.f, w_sq_sum = 0.f;
    for (int k = 0; k < K; ++k)
    {
        w_sum += weights[k];
        w_sq_sum += weights[k] * weights[k];
    }
    if (w_sq_sum < active_params_.ess_den_epsilon) return 1.f;  // degenerate: all zero
    return (w_sum * w_sum) / w_sq_sum;
}

// ============================================================================
// ESS-based adaptation of K, T, λ, σ, and injection count
//
// Runs every cycle for λ/σ (fast), every adapt_interval cycles for K/T (slow).
// CPU budget caps K*T to keep MPPI within cpu_budget_ms.
// ============================================================================

void TrajectoryController::adapt_from_ess(float ess, int K, int /*num_collisions*/)
{
    // 1. EMA-smoothed ESS
    const float alpha = active_params_.ess_smoothing;
    ess_smooth_ = (1.f - alpha) * ess_smooth_ + alpha * ess;

    const float ess_ratio = std::clamp(ess_smooth_ / static_cast<float>(std::max(K, 1)), 0.f, 1.f);

    // 2. Continuous exploration signal: explore ∈ [0, 1]
    //    Maps ess_ratio linearly: ratio >= ess_high → explore=0, ratio <= ess_low_very → explore=1
    //    Smoothed to avoid jitter.
    const float raw_explore = std::clamp(
        (active_params_.ess_high - ess_ratio) / std::max(active_params_.ess_high - active_params_.ess_low_very, 0.01f),
        0.f, 1.f);
    explore_ = 0.8f * explore_ + 0.2f * raw_explore;

    // 3. λ adaptation (unchanged): low ESS → soften weights; high ESS → sharpen
    if (ess_ratio < active_params_.ess_low_very)
        adaptive_lambda_ *= active_params_.lambda_gain_low_very;
    else if (ess_ratio < active_params_.ess_low)
        adaptive_lambda_ *= active_params_.lambda_gain_low;
    else if (ess_ratio > active_params_.ess_high)
        adaptive_lambda_ *= active_params_.lambda_gain_high;
    adaptive_lambda_ = std::clamp(adaptive_lambda_, active_params_.lambda_min, active_params_.lambda_max);

    // 4. σ_rot driven by explore: wider angular search when ESS drops
    const float sigma_rot_target = active_params_.sigma_rot
        + explore_ * (active_params_.sigma_max_rot - active_params_.sigma_rot);
    adaptive_sigma_rot_ = 0.8f * adaptive_sigma_rot_ + 0.2f * sigma_rot_target;
    adaptive_sigma_rot_ = std::clamp(adaptive_sigma_rot_, active_params_.sigma_min_rot, active_params_.sigma_max_rot);

    // σ_adv slightly reduced when exploring (focus on turning, not speeding)
    const float sigma_adv_target = active_params_.sigma_adv
        * (1.f - 0.3f * explore_);
    adaptive_sigma_adv_ = 0.8f * adaptive_sigma_adv_ + 0.2f * sigma_adv_target;
    adaptive_sigma_adv_ = std::clamp(adaptive_sigma_adv_, active_params_.sigma_min_adv, active_params_.sigma_max_adv);
}

} // namespace rc

