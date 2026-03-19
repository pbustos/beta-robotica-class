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
        const float reactivity_gain = std::max(0.f, params.mood_reactivity_gain);
        const float caution_gain = std::max(0.f, params.mood_caution_gain);

        // Speed family: kinematic limits and lookahead
        active_params_.max_adv = scale_with_gain(params.max_adv, speed_gain, 0.01f);
        active_params_.max_rot = scale_with_gain(params.max_rot, speed_gain, 0.01f);
        active_params_.carrot_lookahead = scale_with_gain(params.carrot_lookahead, speed_gain, 0.05f);
        active_params_.goal_threshold = scale_with_gain(params.goal_threshold, speed_gain * 0.5f, 0.01f);

        // Reactivity family: output smoothing, warm-start inertia, brake
        active_params_.velocity_smoothing = std::clamp(
            params.velocity_smoothing * (1.f - reactivity_gain * n), 0.f, 0.98f);
        active_params_.warm_start_adv_weight = std::clamp(
            params.warm_start_adv_weight * (1.f - reactivity_gain * n), 0.f, 0.98f);
        active_params_.warm_start_rot_weight = std::clamp(
            params.warm_start_rot_weight * (1.f - reactivity_gain * n), 0.f, 0.98f);
        active_params_.gauss_k = scale_with_gain(params.gauss_k, -reactivity_gain, 1e-5f);

        // Caution family: safety margins (calm side only)
        const float calm_factor = std::max(0.f, -n);  // 0 at neutral/excited, 1 at calm
        active_params_.d_safe = std::max(base_d_safe_priority,
                                         base_d_safe_priority * (1.f + caution_gain * calm_factor));
        active_params_.collision_penalty = std::max(base_collision_penalty_priority,
                                                    base_collision_penalty_priority * (1.f + 0.5f * caution_gain * calm_factor));
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

float TrajectoryController::effective_d_safe_for_goal_dist(float goal_dist) const
{
    const float near_safe = std::max(active_params_.robot_radius + active_params_.goal_obstacle_margin,
                                     active_params_.robot_radius + 0.005f);
    const float far_safe = std::max(active_params_.d_safe, near_safe);
    const float tau = std::max(active_params_.goal_clearance_relax_dist, 1e-3f);
    const float alpha = std::exp(-std::max(goal_dist, 0.f) / tau); // 1 near goal, 0 far
    return std::clamp((1.f - alpha) * far_safe + alpha * near_safe, near_safe, far_safe);
}

float TrajectoryController::obstacle_step_cost(float esdf_val, float d_safe_eff) const
{
    const float d_safe = std::max(d_safe_eff, active_params_.robot_radius + 1e-3f);
    const float soft_span = std::max(d_safe - active_params_.robot_radius, 1e-3f);
    const float hard_margin = std::max(active_params_.close_obstacle_margin, 1e-3f);
    const float hard_threshold = active_params_.robot_radius + hard_margin;

    float soft_penalty = 0.f;
    if (esdf_val < d_safe)
    {
        const float normalized = (d_safe - esdf_val) / soft_span;
        soft_penalty = normalized * normalized;
    }

    float hard_penalty = 0.f;
    if (esdf_val < hard_threshold)
    {
        const float normalized = (hard_threshold - esdf_val) / hard_margin;
        hard_penalty = active_params_.close_obstacle_gain * normalized * normalized;
    }

    const float g_obs = active_params_.lambda_obstacle * (soft_penalty + hard_penalty);
    return std::min(g_obs, active_params_.obstacle_cost_cap);
}

float TrajectoryController::obstacle_repulsion_strength(float esdf_val, float d_safe_eff) const
{
    const float d_safe = std::max(d_safe_eff, active_params_.robot_radius + 1e-3f);
    const float soft_span = std::max(d_safe - active_params_.robot_radius, 1e-3f);
    const float hard_margin = std::max(active_params_.close_obstacle_margin, 1e-3f);
    const float hard_threshold = active_params_.robot_radius + hard_margin;

    float strength = 0.f;
    if (esdf_val < d_safe)
    {
        const float normalized = (d_safe - esdf_val) / soft_span;
        strength += active_params_.lambda_obstacle * normalized;
    }

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

    // Smooth path with Catmull-Rom spline interpolation for continuous curvature
    smooth_path_spline();

    has_prev_vel_ = false;
    smoothed_vel_ = Eigen::Vector3f::Zero();

    // Initialize MPPI state
    adaptive_K_ = active_params_.num_samples;
    adaptive_T_ = active_params_.trajectory_steps;
    adaptive_lambda_ = active_params_.mppi_lambda;
    ess_smooth_ = static_cast<float>(adaptive_K_) * active_params_.ess_initial_ratio;
    dominance_smooth_ = 0.5f;
    explore_ = 0.f;
    last_mppi_ms_ = 0.f;
    safety_guard_mood_cooldown_ = 0;
    carrot_curve_active_ = false;

    prev_optimal_.assign(adaptive_T_, {0.f, 0.f});
    adaptive_sigma_adv_ = active_params_.sigma_adv;
    adaptive_sigma_rot_ = active_params_.sigma_rot;

    // Reset blockage detection
    blockage_streak_ = 0;
    blockage_cooldown_ = 0;

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
    carrot_curve_active_ = false;
    blockage_streak_ = 0;
    blockage_cooldown_ = 0;
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
// Catmull-Rom spline path smoothing
// Replaces the piecewise-linear (relaxed) path with a smooth curve that
// preserves start/end points and passes through all original waypoints.
// The output spacing is uniform (~spline_spacing meters between points).
// ============================================================================

void TrajectoryController::smooth_path_spline()
{
    const int n = static_cast<int>(path_room_.size());
    if (n < 3) return;  // need at least 3 points for meaningful spline

    constexpr float spline_spacing = 0.15f;  // output resolution in meters
    constexpr float alpha = 0.5f;             // centripetal Catmull-Rom (0.5)

    // Evaluate one Catmull-Rom segment between P1 and P2
    // (P0, P1, P2, P3 are the four control points)
    auto catmull_rom = [alpha](const Eigen::Vector2f& P0, const Eigen::Vector2f& P1,
                               const Eigen::Vector2f& P2, const Eigen::Vector2f& P3,
                               float t) -> Eigen::Vector2f
    {
        // Knot parameterization
        auto knot = [alpha](float ti, const Eigen::Vector2f& a, const Eigen::Vector2f& b)
        {
            float d = (b - a).squaredNorm();
            return ti + std::pow(std::max(d, 1e-8f), alpha * 0.5f);
        };
        float t0 = 0.f;
        float t1 = knot(t0, P0, P1);
        float t2 = knot(t1, P1, P2);
        float t3 = knot(t2, P2, P3);

        // Map t in [0,1] to [t1, t2]
        float u = t1 + t * (t2 - t1);

        auto lerp = [](const Eigen::Vector2f& a, const Eigen::Vector2f& b, float ta, float tb, float tu)
        {
            float f = (tu - ta) / std::max(tb - ta, 1e-8f);
            return (1.f - f) * a + f * b;
        };

        Eigen::Vector2f A1 = lerp(P0, P1, t0, t1, u);
        Eigen::Vector2f A2 = lerp(P1, P2, t1, t2, u);
        Eigen::Vector2f A3 = lerp(P2, P3, t2, t3, u);
        Eigen::Vector2f B1 = lerp(A1, A2, t0, t2, u);
        Eigen::Vector2f B2 = lerp(A2, A3, t1, t3, u);
        return lerp(B1, B2, t1, t2, u);
    };

    std::vector<Eigen::Vector2f> smooth;
    smooth.push_back(path_room_.front());

    for (int i = 0; i < n - 1; ++i)
    {
        // Clamp control points at boundaries
        const Eigen::Vector2f& P0 = path_room_[std::max(0, i - 1)];
        const Eigen::Vector2f& P1 = path_room_[i];
        const Eigen::Vector2f& P2 = path_room_[std::min(n - 1, i + 1)];
        const Eigen::Vector2f& P3 = path_room_[std::min(n - 1, i + 2)];

        float seg_len = (P2 - P1).norm();
        int n_sub = std::max(1, static_cast<int>(std::ceil(seg_len / spline_spacing)));

        for (int s = 1; s <= n_sub; ++s)
        {
            float t = static_cast<float>(s) / static_cast<float>(n_sub);
            smooth.push_back(catmull_rom(P0, P1, P2, P3, t));
        }
    }

    // Safety: verify spline points stay inside free space (min clearance from obstacles)
    if (!room_boundary_points_room_.empty() || !static_obstacle_points_room_.empty())
    {
        const float min_clearance = active_params_.robot_radius + 0.03f;
        std::vector<Eigen::Vector2f> all_obs;
        all_obs.insert(all_obs.end(), room_boundary_points_room_.begin(), room_boundary_points_room_.end());
        all_obs.insert(all_obs.end(), static_obstacle_points_room_.begin(), static_obstacle_points_room_.end());

        for (auto& pt : smooth)
        {
            float min_dist_sq = std::numeric_limits<float>::max();
            Eigen::Vector2f nearest = pt;
            for (const auto& obs_pt : all_obs)
            {
                float dsq = (pt - obs_pt).squaredNorm();
                if (dsq < min_dist_sq) { min_dist_sq = dsq; nearest = obs_pt; }
            }
            float dist = std::sqrt(min_dist_sq);
            if (dist < min_clearance && dist > 1e-4f)
            {
                // Push point away from obstacle to maintain clearance
                Eigen::Vector2f away = (pt - nearest) / dist;
                pt = nearest + away * min_clearance;
            }
        }
    }

    path_room_ = std::move(smooth);
    std::cout << "[TrajectoryCtrl] Path spline-smoothed: " << path_room_.size() << " points\n";
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

    out.min_esdf = query_esdf(0.f, 0.f);

    // Safety-Guard proximity proxy from frontal LiDAR distance (continuous in [0,1]).
    // Exploration should ramp up only near SG activation distance.
    float sg_gate = 0.f;
    float frontal_min_dist = std::numeric_limits<float>::infinity();
    {
        constexpr float sg_front_cone_rad = 0.45f;
        const float sg_on_dist = active_params_.d_safe + 0.08f;
        const float sg_center = sg_on_dist + active_params_.sg_explore_pre_distance;
        const float sg_width = std::max(1e-3f, active_params_.sg_explore_sigmoid_width);

        for (const auto &p : lidar_points)
        {
            const float px = p.x();
            const float py = p.y();
            if (py <= 0.f) continue;

            const float d = std::hypot(px, py);
            if (d < active_params_.robot_radius + 0.05f) continue;

            const float ang = std::abs(std::atan2(px, py));
            if (ang <= sg_front_cone_rad)
                frontal_min_dist = std::min(frontal_min_dist, d);
        }

        if (std::isfinite(frontal_min_dist))
        {
            const float x = (frontal_min_dist - sg_center) / sg_width;
            const float raw = 1.f / (1.f + std::exp(x));
            sg_explore_gate_smooth_ = 0.8f * sg_explore_gate_smooth_ + 0.2f * std::clamp(raw, 0.f, 1.f);
            sg_gate = std::clamp(sg_explore_gate_smooth_, 0.f, 1.f);
        }
        else
        {
            sg_explore_gate_smooth_ *= 0.8f;
            sg_gate = std::clamp(sg_explore_gate_smooth_, 0.f, 1.f);
        }
    }

    // 4. Carrot (fixed lookahead, no displacement/bias)
    const Eigen::Vector2f carrot_room = compute_carrot(robot_pose);
    const Eigen::Vector2f carrot_robot = room_to_robot(carrot_room, robot_pose);
    out.carrot_room = carrot_room;
    out.current_wp_index = wp_index_;

    // ---- PD carrot-follower mode: simple proportional-derivative controller ----
    if (control_mode_ == ControlMode::PD)
        return compute_pd(out, carrot_robot, lidar_points, robot_pose);

    // 5-6. Tracking MPPI: compute nominal toward carrot and blend it with
    //       the shifted previous optimum to create the actual sampling center.
    //       This preserves carrot coverage while adding stronger temporal momentum.
    Seed nominal_seed = compute_nominal(carrot_robot, T);
    Seed sampling_center = nominal_seed;
    float ws_adv_eff = 0.f;
    float ws_rot_eff = 0.f;
    {
        const float ws_boost = 1.f + 0.75f * sg_gate + 0.50f * explore_;
        ws_adv_eff = std::clamp(active_params_.warm_start_adv_weight * ws_boost, 0.f, 0.97f);
        ws_rot_eff = std::clamp(active_params_.warm_start_rot_weight * ws_boost, 0.f, 0.97f);

        if (!prev_optimal_.empty())
        {
            const int prev_T = static_cast<int>(prev_optimal_.size());
            for (int t = 0; t < T; ++t)
            {
                const int p = std::min(t + 1, prev_T - 1);  // shifted warm start
                const float prev_adv = prev_optimal_[p].adv;
                const float prev_rot = prev_optimal_[p].rot;

                sampling_center.adv[t] = std::clamp(
                    (1.f - ws_adv_eff) * nominal_seed.adv[t] + ws_adv_eff * prev_adv,
                    active_params_.min_adv_cmd, active_params_.max_adv);
                sampling_center.rot[t] = std::clamp(
                    (1.f - ws_rot_eff) * nominal_seed.rot[t] + ws_rot_eff * prev_rot,
                    -active_params_.max_rot, active_params_.max_rot);
            }
        }
    }

    // 7. Sample K trajectories around the blended center
    auto seeds = sample_trajectories(carrot_robot, sampling_center);
    const int actual_K = static_cast<int>(seeds.size());

    // 8. Simulate, optimize, and score each sample
    std::vector<SimResult> results(actual_K);
    int best_idx = -1;
    float best_G = std::numeric_limits<float>::max();

    for (int k = 0; k < actual_K; ++k)
    {
        optimize_seed(seeds[k], carrot_robot);
        results[k] = simulate_and_score(seeds[k], carrot_robot, goal_robot, sampling_center);
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
    float dominance_current = 0.f;
    float p_free_current = 0.f;
    float steering_concentration_current = 0.f;
    float clearance_quality_current = 0.f;
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

        // Compute ESS (diagnostics)
        ess_current = compute_ess(weights, actual_K);
        ess_smooth_ = 0.75f * ess_smooth_ + 0.25f * ess_current;

        // Dominance D = free-survival mass * steering concentration
        // D≈0 -> no viable dominant bypass (explore more)
        // D≈1 -> clear viable direction dominates (exploit more)
        const int num_survivors = actual_K - num_collisions;
        const float p_free = static_cast<float>(std::max(0, num_survivors))
                           / static_cast<float>(std::max(actual_K, 1));
        float steering_concentration = 0.f;
        float clearance_quality = 0.f;
        if (num_survivors > 0)
        {
            float cos_acc = 0.f;
            float sin_acc = 0.f;
            float clear_acc = 0.f;
            float w_acc = 0.f;
            const bool use_soft_weights = (w_sum > active_params_.weights_epsilon);
            const float clear_denom = std::max(active_params_.d_safe - active_params_.robot_radius, 1e-3f);
            for (int k = 0; k < actual_K; ++k)
            {
                if (results[k].collides)
                    continue;

                const float wk = use_soft_weights
                    ? (weights[k] / w_sum)
                    : (1.f / static_cast<float>(num_survivors));

                const int steps_k = static_cast<int>(seeds[k].rot.size());
                const int heading_window = std::min(8, steps_k);
                float steering_angle = 0.f;
                for (int s = 0; s < heading_window; ++s)
                    steering_angle += seeds[k].rot[s] * active_params_.trajectory_dt;

                const float clearance_norm = std::clamp(
                    (results[k].min_esdf - active_params_.robot_radius) / clear_denom,
                    0.f, 1.f);

                cos_acc += wk * std::cos(steering_angle);
                sin_acc += wk * std::sin(steering_angle);
                clear_acc += wk * clearance_norm;
                w_acc += wk;
            }
            if (w_acc > active_params_.weights_epsilon)
            {
                steering_concentration = std::sqrt(cos_acc * cos_acc + sin_acc * sin_acc) / w_acc;
                clearance_quality = clear_acc / w_acc;
            }
        }
        p_free_current = p_free;
        steering_concentration_current = steering_concentration;
        clearance_quality_current = clearance_quality;

        // Dominance: feasibility × directional concentration (clearance excluded
        // to avoid over-exploration in narrow passages).
        dominance_current = std::clamp(p_free * steering_concentration, 0.f, 1.f);

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

    // 10. Single-metric adaptation from rollout dominance gated by SG proximity
    adapt_from_dominance(dominance_current, actual_K, sg_gate);

    // Top-K decisive blending: when exploration is high, blend weighted mean
    // toward the weighted average of the top-3 feasible seeds rather than the
    // single best.  This eliminates frame-to-frame sign flips when two
    // competing seeds alternate as "best" near doorways.
    {
        constexpr int top_k_count = 3;
        const float decisive = std::clamp(explore_ * explore_, 0.f, 0.85f);
        if (decisive > 1e-4f && actual_K > 0)
        {
            // Collect indices of non-colliding seeds sorted by cost
            std::vector<int> free_indices;
            free_indices.reserve(actual_K);
            for (int k = 0; k < actual_K; ++k)
                if (!results[k].collides) free_indices.push_back(k);
            // Fall back to all seeds if none are free
            if (free_indices.empty())
                for (int k = 0; k < actual_K; ++k) free_indices.push_back(k);

            std::sort(free_indices.begin(), free_indices.end(),
                      [&](int a, int b) { return results[a].G_total < results[b].G_total; });
            const int n_top = std::min(top_k_count, static_cast<int>(free_indices.size()));

            // Softmax weights over top-K costs (temperature = lambda_used)
            float top_w_sum = 0.f;
            std::vector<float> top_w(n_top);
            const float G_top_min = results[free_indices[0]].G_total;
            for (int i = 0; i < n_top; ++i)
            {
                const float exp_arg = std::max(-30.f, -(results[free_indices[i]].G_total - G_top_min)
                                                       / std::max(lambda_used, 1e-6f));
                top_w[i] = std::exp(exp_arg);
                top_w_sum += top_w[i];
            }
            if (top_w_sum > 1e-10f)
            {
                for (int t = 0; t < T; ++t)
                {
                    float blend_adv = 0.f, blend_rot = 0.f;
                    for (int i = 0; i < n_top; ++i)
                    {
                        const int ki = free_indices[i];
                        const float w = top_w[i] / top_w_sum;
                        const int steps_k = static_cast<int>(seeds[ki].adv.size());
                        if (t < steps_k)
                        {
                            blend_adv += w * seeds[ki].adv[t];
                            blend_rot += w * seeds[ki].rot[t];
                        }
                    }
                    optimal[t].adv = (1.f - decisive) * optimal[t].adv + decisive * blend_adv;
                    optimal[t].rot = (1.f - decisive) * optimal[t].rot + decisive * blend_rot;
                }
            }
        }
    }

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

    float cmd_adv = 0.f, cmd_rot = 0.f;
    // 11. Extract command: average first 3 steps of the optimal sequence.
    // Step 0 has the highest perturbation variance; averaging [0..2] is a
    // standard MPPI technique that smooths the output without adding lag.
    {
        constexpr int cmd_window = 3;
        const int n_avg = std::min(cmd_window, T);
        float sum_adv = 0.f, sum_rot = 0.f;
        for (int t = 0; t < n_avg; ++t)
        {
            sum_adv += optimal[t].adv;
            sum_rot += optimal[t].rot;
        }
        cmd_adv = sum_adv / static_cast<float>(n_avg);
        cmd_rot = sum_rot / static_cast<float>(n_avg);
    }

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
        constexpr float gate_inflate_m = 0.03f;       // soft margin around d_safe
        constexpr float gate_hard_margin_m = 0.01f;
        constexpr int gate_soft_consecutive_needed = 5; // consecutive close steps to trigger
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
        constexpr float lidar_front_cone_rad = 0.40f; // ~23 deg frontal cone
        const float lidar_trigger_dist = active_params_.d_safe + 0.08f;
        const float lidar_min_valid_dist = active_params_.robot_radius + 0.08f;
        constexpr int lidar_min_points_to_arm = 5;
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
                    out.adv = -std::min(active_params_.max_back_adv, 0.05f);
            }
        }
    }

    // ── 15. Path blockage detection ──────────────────────────────────
    // Look ahead along the planned path in room frame and query ESDF at each
    // waypoint.  If several consecutive waypoints are blocked (ESDF < threshold)
    // for many consecutive compute cycles, signal the caller to replan.
    out.path_blocked = false;
    if (blockage_cooldown_ > 0)
    {
        --blockage_cooldown_;
    }
    else
    {
        const float look_m = active_params_.blockage_lookahead_m;
        const float thr = active_params_.blockage_esdf_threshold;
        const int   min_wp = active_params_.blockage_min_waypoints;

        int   consec_blocked = 0;
        Eigen::Vector2f block_sum = Eigen::Vector2f::Zero();
        int   block_count = 0;

        float accum_dist = 0.f;
        for (int i = wp_index_; i < static_cast<int>(path_room_.size()); ++i)
        {
            if (i > wp_index_)
                accum_dist += (path_room_[i] - path_room_[i - 1]).norm();
            if (accum_dist > look_m) break;

            const Eigen::Vector2f p_rob = room_to_robot(path_room_[i], robot_pose);
            const float esdf_val = query_esdf(p_rob.x(), p_rob.y());

            if (esdf_val < thr)
            {
                ++consec_blocked;
                block_sum += path_room_[i];
                ++block_count;
            }
            else
            {
                // reset streak only if we had fewer than required
                if (consec_blocked < min_wp)
                {
                    consec_blocked = 0;
                    block_sum = Eigen::Vector2f::Zero();
                    block_count = 0;
                }
            }
        }

        if (consec_blocked >= min_wp)
        {
            ++blockage_streak_;
            if (blockage_streak_ >= active_params_.blockage_confirm_cycles)
            {
                out.path_blocked = true;
                out.blockage_center_room = block_sum / static_cast<float>(block_count);
                // radius = max distance from center to any blocked waypoint + robot radius
                // Recompute max dist from final center
                float max_r = 0.f;
                for (int i = wp_index_; i < static_cast<int>(path_room_.size()); ++i)
                {
                    const Eigen::Vector2f p_rob = room_to_robot(path_room_[i], robot_pose);
                    if (query_esdf(p_rob.x(), p_rob.y()) < thr)
                        max_r = std::max(max_r, (path_room_[i] - out.blockage_center_room).norm());
                }
                out.blockage_radius = max_r + active_params_.robot_radius;
                blockage_streak_ = 0;
                blockage_cooldown_ = active_params_.blockage_cooldown_cycles;
                std::cout << "[TrajectoryCtrl] PATH BLOCKED at ("
                          << out.blockage_center_room.x() << ", " << out.blockage_center_room.y()
                          << ") r=" << out.blockage_radius << "\n";
            }
        }
        else
        {
            blockage_streak_ = std::max(0, blockage_streak_ - 1); // decay slowly
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
                  << " Draw=" << std::setprecision(2) << dominance_current
                  << " Ds=" << std::setprecision(2) << dominance_smooth_
                  << " Pf=" << std::setprecision(2) << p_free_current
                  << " Rθ=" << std::setprecision(2) << steering_concentration_current
                  << " C=" << std::setprecision(2) << clearance_quality_current
                  << " E=" << std::setprecision(2) << explore_
                  << " SG=" << std::setprecision(2) << sg_gate
                  << " Ws(" << std::setprecision(2) << ws_adv_eff << "," << ws_rot_eff << ")"
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
        // Kp=1.8 saturates at ~22° heading error (vs 16° at 2.5), reducing
        // unnecessary max-rot commands during small course corrections while
        // still reaching max_rot at doors/corners (>22° error).
        const float Kp_nom = 1.8f;
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
    const Eigen::Vector2f& carrot_robot,
    const Seed& nominal)
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

    const float Kp_nom = 1.8f;  // same PD-like gain as compute_nominal

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
    const Seed& seed,
    const Eigen::Vector2f& carrot_robot,
    const Eigen::Vector2f& goal_robot,
    const Seed& nominal)
{
    SimResult res;
    const int steps = static_cast<int>(seed.adv.size());
    const float dt = active_params_.trajectory_dt;
    res.positions.reserve(steps);

    float x = 0.f, y = 0.f, theta = 0.f;
    float G_obs_total = 0.f;
    float G_lat_total = 0.f;
    float G_cbf_total = 0.f;
    float G_progress = 0.f;
    const float discount = active_params_.cost_discount;
    float discount_acc = 1.f;
    float prev_dist_to_carrot = carrot_robot.norm();
    [[maybe_unused]] float prev_side_min = std::numeric_limits<float>::infinity();
    // CBF state: h(x,v) = d_ESDF - r - v²/(2*a_max)
    float prev_h_cbf = std::numeric_limits<float>::infinity();
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

        const float dist_goal_step = (Eigen::Vector2f(x, y) - goal_robot).norm();
        const float d_safe_eff = effective_d_safe_for_goal_dist(dist_goal_step);
        const float G_obs = obstacle_step_cost(esdf_val, d_safe_eff);

        // Lateral-clearance shaping (continuous, pre-SG):
        // sample ESDF on both sides of the predicted body center and penalize
        // low side clearance and worsening side clearance trend.
        {
            const float probe_offset = std::max(0.f, active_params_.lateral_probe_offset);
            const float ct = std::cos(theta);
            const float st = std::sin(theta);

            const float x_right = x + probe_offset * ct;
            const float y_right = y - probe_offset * st;
            const float x_left  = x - probe_offset * ct;
            const float y_left  = y + probe_offset * st;

            const float d_right = query_esdf(x_right, y_right);
            const float d_left  = query_esdf(x_left, y_left);
            const float side_min = std::min(d_left, d_right);

            const float side_target = active_params_.robot_radius
                                    + std::max(0.f, active_params_.lateral_clearance_margin);
            const float side_span = std::max(active_params_.lateral_clearance_margin, 1e-3f);

            float G_lat_step = 0.f;
            if (side_min < side_target)
            {
                const float deficit = (side_target - side_min) / side_span;
                G_lat_step += active_params_.lambda_lateral_clearance * deficit * deficit;
            }

            // Lateral closing-gain term — disabled; replaced by CBF below.
            // if (std::isfinite(prev_side_min) && side_min < prev_side_min)
            // {
            //     const float closing = (prev_side_min - side_min) / side_span;
            //     G_lat_step += active_params_.lambda_lateral_clearance
            //                 * active_params_.lateral_closing_gain
            //                 * std::max(0.f, closing);
            // }

            G_lat_total += discount_acc * G_lat_step;
            prev_side_min = side_min;
        }

        // Control Barrier Function cost:
        // h(x,v) = d_ESDF - r_robot - v²/(2*a_max)
        // Penalise ḣ + α·h < 0  (barrier decaying faster than class-K allows)
        if (active_params_.enable_cbf)
        {
            const float v        = seed.adv[s];
            const float a_max    = std::max(active_params_.cbf_max_decel, 1e-3f);
            const float h_curr   = esdf_val - active_params_.robot_radius
                                 - (v * v) / (2.f * a_max);
            if (std::isfinite(prev_h_cbf))
            {
                const float h_dot    = (h_curr - prev_h_cbf) / dt;
                const float cbf_cond = h_dot + active_params_.cbf_alpha * h_curr;
                if (cbf_cond < 0.f)
                {
                    const float viol = cbf_cond * cbf_cond;
                    G_cbf_total += discount_acc
                                 * std::min(active_params_.lambda_cbf * viol,
                                            active_params_.cbf_cost_cap);
                }
            }
            prev_h_cbf = h_curr;
        }

        G_obs_total += discount_acc * G_obs;
        discount_acc *= discount;

        // Collision semantics with finite hard horizon:
        // - near-term collision => hard infeasible (weight zero)
        // - far collision      => soft penalty only (keeps rollout useful)
        if (esdf_val < active_params_.robot_radius + active_params_.close_obstacle_margin)
        {
            const float ttc_s = static_cast<float>(s + 1) * dt;
            if (ttc_s <= active_params_.hard_collision_horizon_s)
            {
                res.collides = true;
                break;
            }
            else
            {
                G_obs_total += discount_acc * active_params_.far_collision_penalty_scale * active_params_.collision_penalty;
                break;
            }
        }
    }

    // G_goal: use the closest-approach point, not the far-future endpoint.
    // This prevents overshoot from dominating the cost when horizon >> carrot distance.
    const float initial_dist = carrot_robot.norm();
    const float progress = initial_dist - best_dist_to_carrot;
    float G_goal = active_params_.lambda_goal
                 * (best_dist_to_carrot + active_params_.lambda_progress * std::max(0.f, -progress));

    // Heading cost: penalize initial angular divergence from carrot direction.
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

    res.G_total = G_goal + G_obs_total + G_lat_total + G_cbf_total + G_smooth + G_vel_mag + G_vel_delta + G_info
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
            const float d_safe_eff = effective_d_safe_for_goal_dist(dist_c);
            if (esdf_val < d_safe_eff)
            {
                Eigen::Vector2f grad = query_esdf_gradient(px[s], py[s]);
                Eigen::Vector2f obs_correction = obstacle_repulsion_strength(esdf_val, d_safe_eff) * grad;
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

Eigen::Vector2f TrajectoryController::compute_carrot(const Eigen::Affine2f& robot_pose)
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

    float lookahead_eff = active_params_.carrot_lookahead;
    if (active_params_.carrot_curve_adaptation_enabled)
    {
        const int n = static_cast<int>(path_room_.size());
        if (n >= 3)
        {
            const int v = std::clamp(closest_seg + 1, 1, n - 2);
            const Eigen::Vector2f seg_prev = path_room_[v] - path_room_[v - 1];
            const Eigen::Vector2f seg_next = path_room_[v + 1] - path_room_[v];
            const float len_prev = seg_prev.norm();
            const float len_next = seg_next.norm();

            if (len_prev > active_params_.segment_length_epsilon && len_next > active_params_.segment_length_epsilon)
            {
                const Eigen::Vector2f t_prev = seg_prev / len_prev;
                const Eigen::Vector2f t_next = seg_next / len_next;
                float dpsi = std::acos(std::clamp(t_prev.dot(t_next), -1.f, 1.f));

                const float enter_th = std::max(active_params_.carrot_curve_min_heading_change, 1e-3f);
                const float release_th = std::clamp(active_params_.carrot_curve_release_heading_change,
                                                    0.f,
                                                    enter_th);

                if (!carrot_curve_active_)
                {
                    if (dpsi >= enter_th)
                        carrot_curve_active_ = true;
                }
                else
                {
                    if (dpsi <= release_th)
                        carrot_curve_active_ = false;
                }

                if (carrot_curve_active_)
                    lookahead_eff = std::clamp(active_params_.carrot_curve_lookahead_min,
                                               0.05f,
                                               active_params_.carrot_lookahead);
            }
        }
        else
        {
            carrot_curve_active_ = false;
        }
    }
    else
    {
        carrot_curve_active_ = false;
    }

    float remaining = lookahead_eff;
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
// Dominance-based adaptation (single metric)
//
// Dominance D in [0,1] captures both:
//  - free-survival mass of rollouts
//  - directional concentration of surviving steering
// Explore signal is E = 1 - D.
// ============================================================================

void TrajectoryController::adapt_from_dominance(float dominance, int /*K*/, float sg_gate)
{
    const float alpha = active_params_.ess_smoothing;
    const float D = std::clamp(dominance, 0.f, 1.f);
    dominance_smooth_ = (1.f - alpha) * dominance_smooth_ + alpha * D;

    // Control adaptation should react quickly to feasibility collapse,
    // so it is driven mostly by instantaneous dominance, with a small
    // contribution of the smoothed state to avoid jitter.
    const float dominance_for_control = std::clamp(0.8f * D + 0.2f * dominance_smooth_, 0.f, 1.f);
    const float base_explore = std::clamp(1.f - dominance_for_control, 0.f, 1.f);
    const float sg = std::clamp(sg_gate, 0.f, 1.f);
    explore_ = std::clamp(base_explore * sg, 0.f, 1.f);

    // Single-law modulation: low dominance -> soften weighting (higher λ)
    const float lambda_target = active_params_.mppi_lambda * (1.f + 2.f * explore_);
    adaptive_lambda_ = 0.8f * adaptive_lambda_ + 0.2f * lambda_target;
    adaptive_lambda_ = std::clamp(adaptive_lambda_, active_params_.lambda_min, active_params_.lambda_max);

    // Wider angular search and slightly reduced speed perturbation when exploring
    const float sigma_rot_target = active_params_.sigma_rot
        + explore_ * (active_params_.sigma_max_rot - active_params_.sigma_rot);
    adaptive_sigma_rot_ = 0.8f * adaptive_sigma_rot_ + 0.2f * sigma_rot_target;
    adaptive_sigma_rot_ = std::clamp(adaptive_sigma_rot_, active_params_.sigma_min_rot, active_params_.sigma_max_rot);

    const float sigma_adv_target = active_params_.sigma_adv * (1.f - 0.3f * explore_);
    adaptive_sigma_adv_ = 0.8f * adaptive_sigma_adv_ + 0.2f * sigma_adv_target;
    adaptive_sigma_adv_ = std::clamp(adaptive_sigma_adv_, active_params_.sigma_min_adv, active_params_.sigma_max_adv);

    // Expand horizon under low dominance to search around unexpected obstacles
    const float target_T = static_cast<float>(active_params_.trajectory_steps)
                         + explore_ * static_cast<float>(active_params_.T_max - active_params_.trajectory_steps);
    adaptive_T_ = std::clamp(static_cast<int>(std::lround(0.8f * static_cast<float>(adaptive_T_) + 0.2f * target_T)),
                             active_params_.T_min, active_params_.T_max);

    // Keep sample count at configured value (clamped), avoiding extra heuristics
    adaptive_K_ = std::clamp(active_params_.num_samples, active_params_.K_min, active_params_.K_max);
}

} // namespace rc

