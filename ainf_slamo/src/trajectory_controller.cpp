#include "trajectory_controller.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>
#include <chrono>

namespace rc
{

// ============================================================================
// Public API
// ============================================================================

void TrajectoryController::set_path(const std::vector<Eigen::Vector2f>& path_room)
{
    path_room_ = path_room;
    wp_index_ = (path_room.size() > 1) ? 1 : 0;
    active_ = path_room.size() >= 2;
    has_prev_vel_ = false;
    smoothed_vel_ = Eigen::Vector3f::Zero();

    // Initialize MPPI state
    adaptive_K_ = params.num_samples;
    adaptive_T_ = params.trajectory_steps;
    adaptive_lambda_ = params.mppi_lambda;
    adaptive_n_inject_ = 2;
    ess_smooth_ = static_cast<float>(adaptive_K_) * 0.5f;  // start at healthy ESS
    last_mppi_ms_ = 0.f;
    adapt_counter_ = 0;

    prev_optimal_.assign(adaptive_T_, {0.f, 0.f});
    adaptive_sigma_adv_ = params.sigma_adv;
    adaptive_sigma_rot_ = params.sigma_rot;

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
    if (out.dist_to_goal < params.goal_threshold)
    { active_ = false; out.goal_reached = true; std::cout << "[TrajectoryCtrl] Goal reached!\n"; return out; }

    // 4. Carrot
    const Eigen::Vector2f carrot_room = compute_carrot(robot_pose);
    const Eigen::Vector2f carrot_robot = room_to_robot(carrot_room, robot_pose);
    out.carrot_room = carrot_room;
    out.current_wp_index = wp_index_;
    out.min_esdf = query_esdf(0.f, 0.f);

    // 5. Warm-start: resize prev_optimal_ if T changed, then shift
    if (static_cast<int>(prev_optimal_.size()) != T)
    {
        std::vector<ControlStep> resized(T, {0.f, 0.f});
        const int copy_n = std::min(T, static_cast<int>(prev_optimal_.size()));
        for (int t = 0; t < copy_n; ++t) resized[t] = prev_optimal_[t];
        prev_optimal_ = std::move(resized);
    }

    // Shift left: discard step 0, duplicate last
    for (int t = 0; t < T - 1; ++t)
        prev_optimal_[t] = prev_optimal_[t + 1];

    // 6. Compute nominal control and blend with warm-started sequence
    Seed nominal = compute_nominal(carrot_robot, T);
    for (int t = 0; t < T; ++t)
    {
        const float wa = params.warm_start_adv_weight;
        const float wr = params.warm_start_rot_weight;
        prev_optimal_[t].adv = wa * prev_optimal_[t].adv + (1.f - wa) * nominal.adv[t];
        prev_optimal_[t].rot = wr * prev_optimal_[t].rot + (1.f - wr) * nominal.rot[t];
    }

    // 7. Sample K trajectories
    auto seeds = sample_trajectories(carrot_robot);
    const int actual_K = static_cast<int>(seeds.size());

    // 8. Simulate, optimize, and score each sample
    std::vector<SimResult> results(actual_K);
    int best_idx = -1;
    float best_G = std::numeric_limits<float>::max();

    for (int k = 0; k < actual_K; ++k)
    {
        optimize_seed(seeds[k], carrot_robot);
        results[k] = simulate_and_score(seeds[k], carrot_robot);
        if (results[k].G_total < best_G)
        {
            best_G = results[k].G_total;
            best_idx = k;
        }
    }

    // 9. MPPI weighted average with adaptive λ
    std::vector<ControlStep> optimal(T, {0.f, 0.f});
    float ess_current = 0.f;
    {
        const float lambda = adaptive_lambda_;
        const float G_min = best_G;

        std::vector<float> weights(actual_K);
        float w_sum = 0.f;
        int num_collisions = 0;

        for (int k = 0; k < actual_K; ++k)
        {
            if (results[k].collides)
            {
                weights[k] = 0.f;
                num_collisions++;
            }
            else
            {
                weights[k] = std::exp(-(results[k].G_total - G_min) / lambda);
            }
            w_sum += weights[k];
        }

        // Compute ESS
        ess_current = compute_ess(weights, actual_K);

        if (w_sum > 1e-10f)
        {
            for (int k = 0; k < actual_K; ++k)
            {
                const float w = weights[k] / w_sum;
                if (w > 1e-10f)
                {
                    const int steps_k = static_cast<int>(seeds[k].adv.size());
                    for (int t = 0; t < std::min(T, steps_k); ++t)
                    {
                        optimal[t].adv += w * seeds[k].adv[t];
                        optimal[t].rot += w * seeds[k].rot[t];
                    }
                }
            }
        }
        else if (best_idx >= 0)
        {
            const int steps_b = static_cast<int>(seeds[best_idx].adv.size());
            for (int t = 0; t < std::min(T, steps_b); ++t)
            {
                optimal[t].adv = seeds[best_idx].adv[t];
                optimal[t].rot = seeds[best_idx].rot[t];
            }
        }

        // 10. ESS-based adaptation of all parameters
        adapt_from_ess(ess_current, actual_K, num_collisions);
    }

    // Export ESS diagnostics
    out.ess = ess_smooth_;
    out.ess_K = adaptive_K_;

    // Store optimal sequence for next cycle's warm-start
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

        const int n_draw = std::min(params.num_trajectories_to_draw, actual_K);
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
        out.best_trajectory_idx = 0;
    }

    // 13. Smooth + Gaussian brake
    Eigen::Vector3f raw(cmd_adv, 0.f, cmd_rot);
    if (has_prev_vel_)
        smoothed_vel_ = params.velocity_smoothing * smoothed_vel_ + (1.f - params.velocity_smoothing) * raw;
    else { smoothed_vel_ = raw; has_prev_vel_ = true; }

    const float rot_ratio = smoothed_vel_[2] / params.max_rot;
    const float brake = std::exp(-params.gauss_k * rot_ratio * rot_ratio);

    out.adv  = smoothed_vel_[0] * brake;
    out.side = smoothed_vel_[1];
    out.rot  = smoothed_vel_[2];

    // Debug (every ~1 second)
    static int dbg = 0;
    if (++dbg % 20 == 0)
    {
        std::cout << "[MPPI] K=" << adaptive_K_ << " T=" << adaptive_T_
                  << " λ=" << std::fixed << std::setprecision(1) << adaptive_lambda_
                  << " ESS=" << std::setprecision(0) << ess_smooth_ << "/" << adaptive_K_
                  << " inj=" << adaptive_n_inject_
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
// Nominal control: simple proportional control toward carrot
// Used as the base for MPPI warm-start blending
// ============================================================================

TrajectoryController::Seed TrajectoryController::compute_nominal(
    const Eigen::Vector2f& carrot_robot, int steps) const
{
    Seed seed;
    seed.adv.resize(steps);
    seed.rot.resize(steps);

    const float dt = params.trajectory_dt;
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

        float rot_cmd = std::clamp(angle_err / dt, -params.max_rot, params.max_rot);

        // Forward speed: proportional to alignment with carrot
        float alignment = std::cos(angle_err);
        float adv_cmd = params.max_adv * std::max(0.1f, alignment);
        // Reduce speed near goal
        float dist_factor = std::min(1.f, carrot_dist / 1.0f);
        adv_cmd *= dist_factor;
        adv_cmd = std::clamp(adv_cmd, 0.05f, params.max_adv);

        seed.adv[s] = adv_cmd;
        seed.rot[s] = rot_cmd;

        theta += rot_cmd * dt;
    }
    return seed;
}

// ============================================================================
// Sample K trajectories: MPPI perturbations + structured exploration injections
//
// Injection count is ESS-driven via adaptive_n_inject_ (2-8 seeds).
// K and T are ESS-driven via adaptive_K_ and adaptive_T_.
// ============================================================================

std::vector<TrajectoryController::Seed> TrajectoryController::sample_trajectories(
    const Eigen::Vector2f& carrot_robot)
{
    const int K = adaptive_K_;
    const int T = adaptive_T_;
    const float alpha = params.noise_alpha;
    const float innovation = std::sqrt(std::max(0.f, 1.f - alpha * alpha));
    const float dt = params.trajectory_dt;
    const float carrot_angle = std::atan2(carrot_robot.x(), carrot_robot.y());

    std::vector<Seed> seeds;
    seeds.reserve(K);

    // --- Structured exploration injections (count set by ESS adaptation) ---
    // Build offset list based on adaptive_n_inject_ (always in pairs ±)
    std::vector<float> inject_offsets;
    if (adaptive_n_inject_ >= 2) { inject_offsets.push_back(0.5f); inject_offsets.push_back(-0.5f); }   // ±30°
    if (adaptive_n_inject_ >= 4) { inject_offsets.push_back(1.05f); inject_offsets.push_back(-1.05f); }  // ±60°
    if (adaptive_n_inject_ >= 6) { inject_offsets.push_back(1.57f); inject_offsets.push_back(-1.57f); }  // ±90°
    if (adaptive_n_inject_ >= 8) { inject_offsets.push_back(2.09f); inject_offsets.push_back(-2.09f); }  // ±120°

    // Generate injection seeds
    for (float offset : inject_offsets)
    {
        Seed s;
        s.adv.resize(T);
        s.rot.resize(T);
        float theta = 0.f;

        for (int t = 0; t < T; ++t)
        {
            float phase = static_cast<float>(t) / static_cast<float>(std::max(T - 1, 1));
            float desired = carrot_angle + offset * (1.f - std::sqrt(phase));

            float angle_err = desired - theta;
            while (angle_err > static_cast<float>(M_PI)) angle_err -= 2.f * static_cast<float>(M_PI);
            while (angle_err < -static_cast<float>(M_PI)) angle_err += 2.f * static_cast<float>(M_PI);

            float rot_cmd = std::clamp(angle_err / dt, -params.max_rot, params.max_rot);
            float adv_cmd = std::clamp(params.max_adv * 0.7f, 0.05f, params.max_adv);

            s.adv[t] = adv_cmd;
            s.rot[t] = rot_cmd;
            theta += rot_cmd * dt;
        }
        seeds.push_back(std::move(s));
    }

    // --- Random MPPI perturbations of warm-started sequence ---
    const int n_random = K - static_cast<int>(seeds.size());
    for (int k = 0; k < n_random; ++k)
    {
        Seed s;
        s.adv.resize(T);
        s.rot.resize(T);

        float eps_adv = normal_(rng_) * adaptive_sigma_adv_;
        float eps_rot = normal_(rng_) * adaptive_sigma_rot_;

        const int prev_T = static_cast<int>(prev_optimal_.size());
        for (int t = 0; t < T; ++t)
        {
            // Use prev_optimal_ if available, else zero base
            const float base_adv = (t < prev_T) ? prev_optimal_[t].adv : 0.f;
            const float base_rot = (t < prev_T) ? prev_optimal_[t].rot : 0.f;

            float adv = base_adv + eps_adv;
            float rot = base_rot + eps_rot;

            s.adv[t] = std::clamp(adv, 0.05f, params.max_adv);
            s.rot[t] = std::clamp(rot, -params.max_rot, params.max_rot);

            eps_adv = alpha * eps_adv + innovation * adaptive_sigma_adv_ * normal_(rng_);
            eps_rot = alpha * eps_rot + innovation * adaptive_sigma_rot_ * normal_(rng_);
        }
        seeds.push_back(std::move(s));
    }

    return seeds;
}

// ============================================================================
// Forward simulate a seed and compute EFE score
// ============================================================================

TrajectoryController::SimResult TrajectoryController::simulate_and_score(
    const Seed& seed, const Eigen::Vector2f& carrot_robot)
{
    SimResult res;
    const int steps = static_cast<int>(seed.adv.size());
    const float dt = params.trajectory_dt;
    res.positions.reserve(steps);

    float x = 0.f, y = 0.f, theta = 0.f;
    float G_obs_total = 0.f;
    float G_progress = 0.f;  // per-step progress toward carrot
    const float discount = 0.95f;
    float discount_acc = 1.f;
    float prev_dist_to_carrot = carrot_robot.norm();
    int actual_steps = 0;

    for (int s = 0; s < steps; ++s)
    {
        // Differential-drive kinematics (Y+ = forward, X+ = right)
        x += seed.adv[s] * std::sin(theta) * dt;
        y += seed.adv[s] * std::cos(theta) * dt;
        theta += seed.rot[s] * dt;

        res.positions.emplace_back(x, y);
        actual_steps = s + 1;

        // Per-step progress: reward getting closer to carrot each step
        float cur_dist = (Eigen::Vector2f(x, y) - carrot_robot).norm();
        float step_progress = prev_dist_to_carrot - cur_dist;  // positive = good
        G_progress += discount_acc * std::max(0.f, -step_progress);  // penalize moving away
        prev_dist_to_carrot = cur_dist;

        float esdf_val = query_esdf(x, y);
        res.min_esdf = std::min(res.min_esdf, esdf_val);

        float G_obs = 0.f;
        if (esdf_val < params.d_safe)
        {
            float penetration = params.d_safe - esdf_val;
            G_obs = params.lambda_obstacle * penetration * penetration;
        }
        if (esdf_val < params.robot_radius + 0.1f)
            G_obs += params.lambda_obstacle * 10.f * std::exp(params.robot_radius - esdf_val);

        G_obs_total += discount_acc * G_obs;
        discount_acc *= discount;

        // Cut trajectory at collision: stop simulating past the lidar barrier
        if (esdf_val < params.robot_radius)
        {
            res.collides = true;
            break;
        }
    }

    // G_goal: endpoint distance + heading alignment + backward penalty
    const Eigen::Vector2f endpoint(x, y);
    const float final_dist = (endpoint - carrot_robot).norm();
    const float initial_dist = carrot_robot.norm();
    const float progress = initial_dist - final_dist;
    // Stronger penalty for moving away from carrot (negative progress)
    float G_goal = params.lambda_goal * (final_dist + 1.0f * std::max(0.f, -progress));

    Eigen::Vector2f to_carrot = carrot_robot - endpoint;
    if (to_carrot.norm() > 0.01f)
    {
        float desired = std::atan2(to_carrot.x(), to_carrot.y());  // Y+ forward
        float err = std::abs(desired - theta);
        if (err > static_cast<float>(M_PI)) err = 2.f * static_cast<float>(M_PI) - err;
        G_goal += 0.5f * params.lambda_goal * err;
    }

    // G_smooth: continuity with warm-started baseline
    float dv = seed.adv[0] - (prev_optimal_.empty() ? 0.f : prev_optimal_[0].adv);
    float dr = seed.rot[0] - (prev_optimal_.empty() ? 0.f : prev_optimal_[0].rot);
    float G_smooth = params.lambda_smooth * (dv * dv + dr * dr);

    // G_velocity: magnitude regularization + action change penalty
    // Only over actual simulated steps
    // Rotation is weighted 4x more than advance to penalize unnecessary turning
    float G_vel_mag = 0.f;
    float G_vel_delta = 0.f;
    for (int s = 0; s < actual_steps; ++s)
    {
        G_vel_mag += seed.adv[s] * seed.adv[s] + 4.f * seed.rot[s] * seed.rot[s];
        if (s > 0)
        {
            float da = seed.adv[s] - seed.adv[s - 1];
            float dro = seed.rot[s] - seed.rot[s - 1];
            G_vel_delta += da * da + 4.f * dro * dro;
        }
    }
    G_vel_mag *= params.lambda_velocity;
    G_vel_delta *= params.lambda_delta_vel;

    res.G_total = G_goal + G_obs_total + G_smooth + G_vel_mag + G_vel_delta
                + params.lambda_goal * G_progress
                + (res.collides ? 1000.f : 0.f);
    return res;
}

// ============================================================================
// Gradient optimization of a seed using ESDF gradient
// ============================================================================

void TrajectoryController::optimize_seed(Seed& seed, const Eigen::Vector2f& carrot_robot)
{
    const int steps = static_cast<int>(seed.adv.size());
    const float dt = params.trajectory_dt;
    const float lr = params.optim_lr;

    for (int iter = 0; iter < params.optim_iterations; ++iter)
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
            if (dist_c > 0.01f)
                correction += params.lambda_goal * to_carrot.normalized() * std::min(dist_c, 1.f);

            // ESDF: push away from obstacles
            // Cap the obstacle correction magnitude so it cannot overpower the goal pull.
            // This prevents trajectories from being pushed backward at narrow passages.
            float esdf_val = query_esdf(px[s], py[s]);
            if (esdf_val < params.d_safe)
            {
                Eigen::Vector2f grad = query_esdf_gradient(px[s], py[s]);
                Eigen::Vector2f obs_correction = params.lambda_obstacle * (params.d_safe - esdf_val) * grad;
                // Cap obstacle correction to at most 2x the goal correction magnitude
                float goal_mag = correction.norm();
                float obs_mag = obs_correction.norm();
                if (obs_mag > goal_mag * 2.f && goal_mag > 1e-4f)
                    obs_correction *= (goal_mag * 2.f) / obs_mag;
                correction += obs_correction;
            }

            // Jacobians (Y+ forward, X+ right)
            float ct = std::cos(ptheta[s]);
            float st = std::sin(ptheta[s]);
            Eigen::Vector2f d_pos_d_adv(st * dt, ct * dt);
            // Limit the remaining-time factor to avoid wild corrections on early steps
            float remaining = std::min(static_cast<float>(steps - s) * dt, 3.f * dt);
            Eigen::Vector2f d_pos_d_rot(seed.adv[s] * ct * remaining * dt,
                                        -seed.adv[s] * st * remaining * dt);

            seed.adv[s] += lr * correction.dot(d_pos_d_adv);
            seed.rot[s] += lr * correction.dot(d_pos_d_rot);

            // Keep advance strictly positive (no backward motion)
            seed.adv[s] = std::clamp(seed.adv[s], 0.05f, params.max_adv);
            seed.rot[s] = std::clamp(seed.rot[s], -params.max_rot, params.max_rot);
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

    float remaining = params.carrot_lookahead;
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
        if (dist < params.carrot_lookahead * 0.5f) { wp_index_++; continue; }
        if (wp_index_ > 0)
        {
            const Eigen::Vector2f seg = path_room_[wp_index_] - path_room_[wp_index_ - 1];
            const float seg_len = seg.norm();
            if (seg_len > 1e-3f)
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
    const float res = params.grid_resolution;
    const float half = params.grid_half_size;
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

    esdf_data_.assign(N * N, 9999.f);

    // Forward pass
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i)
        {
            const int idx = j * N + i;
            if (occ[idx]) { esdf_data_[idx] = 0.f; continue; }
            float d = 9999.f;
            if (i > 0)            d = std::min(d, esdf_data_[idx - 1] + 1.f);
            if (j > 0)            d = std::min(d, esdf_data_[(j-1)*N + i] + 1.f);
            if (i > 0 && j > 0)   d = std::min(d, esdf_data_[(j-1)*N + (i-1)] + 1.414f);
            if (i < N-1 && j > 0) d = std::min(d, esdf_data_[(j-1)*N + (i+1)] + 1.414f);
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
            if (i < N-1 && j < N-1)   d = std::min(d, esdf_data_[(j+1)*N + (i+1)] + 1.414f);
            if (i > 0   && j < N-1)   d = std::min(d, esdf_data_[(j+1)*N + (i-1)] + 1.414f);
            esdf_data_[idx] = d;
        }

    for (auto& d : esdf_data_) d *= res;
}

float TrajectoryController::query_esdf(float rx, float ry) const
{
    if (esdf_data_.empty()) return 100.f;
    const float res = params.grid_resolution;
    const float half = params.grid_half_size;
    const int N = esdf_N_;

    const float gx = (rx + half) / res;
    const float gy = (ry + half) / res;
    const int ix = static_cast<int>(std::floor(gx));
    const int iy = static_cast<int>(std::floor(gy));
    if (ix < 0 || ix >= N-1 || iy < 0 || iy >= N-1) return 100.f;

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
    const float h = params.grid_resolution;
    const float dx = query_esdf(rx + h, ry) - query_esdf(rx - h, ry);
    const float dy = query_esdf(rx, ry + h) - query_esdf(rx, ry - h);
    Eigen::Vector2f grad(dx / (2.f * h), dy / (2.f * h));
    const float mag = grad.norm();
    if (mag > 1e-4f) grad /= mag;
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
    if (w_sq_sum < 1e-20f) return 1.f;  // degenerate: all zero
    return (w_sum * w_sum) / w_sq_sum;
}

// ============================================================================
// ESS-based adaptation of K, T, λ, σ, and injection count
//
// Runs every cycle for λ/σ (fast), every adapt_interval cycles for K/T (slow).
// CPU budget caps K*T to keep MPPI within cpu_budget_ms.
// ============================================================================

void TrajectoryController::adapt_from_ess(float ess, int K, int num_collisions)
{
    // 1. EMA-smoothed ESS
    const float alpha = params.ess_smoothing;
    ess_smooth_ = (1.f - alpha) * ess_smooth_ + alpha * ess;

    const float ess_ratio = ess_smooth_ / static_cast<float>(std::max(K, 1));  // 0..1

    // 2. λ adaptation (every cycle — fast response)
    //    Low ESS → increase λ (soften weights, let more samples contribute)
    //    High ESS → decrease λ (be more selective)
    if (ess_ratio < 0.15f)
        adaptive_lambda_ *= 1.15f;   // very low → aggressive soften
    else if (ess_ratio < 0.30f)
        adaptive_lambda_ *= 1.08f;   // low → moderate soften
    else if (ess_ratio > 0.5f)
        adaptive_lambda_ *= 0.96f;   // healthy → sharpen
    adaptive_lambda_ = std::clamp(adaptive_lambda_, params.lambda_min, params.lambda_max);

    // 3. σ adaptation (every cycle)
    //    Low ESS → increase σ_rot (explore laterals), decrease σ_adv
    //    High ESS → tighten σ_rot (reduce nodding), relax σ_adv slightly
    if (ess_ratio < 0.15f)
    {
        adaptive_sigma_rot_ *= 1.08f;  // very low → faster lateral exploration
        adaptive_sigma_adv_ *= 0.90f;
    }
    else if (ess_ratio < 0.30f)
    {
        adaptive_sigma_rot_ *= 1.04f;  // low → moderate growth
        adaptive_sigma_adv_ *= 0.95f;
    }
    else if (ess_ratio > 0.5f)
    {
        adaptive_sigma_rot_ *= 0.97f;
        adaptive_sigma_adv_ *= 1.01f;
    }
    adaptive_sigma_adv_ = std::clamp(adaptive_sigma_adv_, params.sigma_min_adv, params.sigma_max_adv);
    adaptive_sigma_rot_ = std::clamp(adaptive_sigma_rot_, params.sigma_min_rot, params.sigma_max_rot);

    // 4. Injection count (every cycle — cheap to change)
    //    Low ESS → more injections (guaranteed lateral coverage)
    //    High ESS → fewer injections (random samples suffice)
    //    Cap at 6 (±90° max) to avoid wild ±120° seeds that cause oscillation
    if (ess_ratio < 0.15f)       adaptive_n_inject_ = 6;   // ±30°, ±60°, ±90°
    else if (ess_ratio < 0.25f)  adaptive_n_inject_ = 4;   // ±30°, ±60°
    else if (ess_ratio < 0.40f)  adaptive_n_inject_ = 3;   // ±30°, ±45°
    else                         adaptive_n_inject_ = 2;   // ±30° only

    // 5. K and T adaptation (every adapt_interval cycles — slow, with CPU budget)
    if (++adapt_counter_ >= params.adapt_interval)
    {
        adapt_counter_ = 0;

        // --- K adaptation ---
        // Low ESS → need more samples; High ESS → can reduce
        int desired_K = adaptive_K_;
        if (ess_ratio < 0.15f)
            desired_K = static_cast<int>(adaptive_K_ * 1.35f);  // +35% (urgent)
        else if (ess_ratio < 0.30f)
            desired_K = static_cast<int>(adaptive_K_ * 1.20f);  // +20%
        else if (ess_ratio > 0.6f)
            desired_K = static_cast<int>(adaptive_K_ * 0.85f); // -15%

        // --- T adaptation ---
        // Low ESS + many collisions → horizon too short to bypass obstacle → increase T
        // High ESS → horizon may be overkill → decrease T
        int desired_T = adaptive_T_;
        const float collision_ratio = static_cast<float>(num_collisions) / static_cast<float>(std::max(K, 1));

        if (ess_ratio < 0.25f && collision_ratio > 0.5f)
            desired_T = static_cast<int>(adaptive_T_ * 1.15f);  // +15%: need to see further
        else if (ess_ratio > 0.6f && collision_ratio < 0.2f)
            desired_T = static_cast<int>(adaptive_T_ * 0.90f);  // -10%: save compute

        // Clamp to configured ranges
        desired_K = std::clamp(desired_K, params.K_min, params.K_max);
        desired_T = std::clamp(desired_T, params.T_min, params.T_max);

        // --- CPU budget cap ---
        // Estimate cost as proportional to K*T. Scale from last measurement.
        if (last_mppi_ms_ > 0.1f)
        {
            const float current_KT = static_cast<float>(adaptive_K_ * adaptive_T_);
            const float desired_KT = static_cast<float>(desired_K * desired_T);
            const float estimated_ms = last_mppi_ms_ * (desired_KT / std::max(current_KT, 1.f));

            if (estimated_ms > params.cpu_budget_ms)
            {
                // Over budget: scale down K*T proportionally, prefer reducing K
                const float scale = params.cpu_budget_ms / estimated_ms;
                desired_K = std::max(params.K_min,
                    static_cast<int>(desired_K * std::sqrt(scale)));  // K gets sqrt reduction
                desired_T = std::max(params.T_min,
                    static_cast<int>(desired_T * std::sqrt(scale)));  // T gets sqrt reduction
            }
        }

        adaptive_K_ = desired_K;
        adaptive_T_ = desired_T;
    }
}

} // namespace rc

