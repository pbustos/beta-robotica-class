#include "trajectory_controller.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>

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

    // Initialize prev_optimal_ to zeros (no prior plan)
    prev_optimal_.assign(params.trajectory_steps, {0.f, 0.f});
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

    const int T = params.trajectory_steps;
    const int K = params.num_samples;

    // 1. ESDF
    build_esdf(lidar_points);

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

    // 5. Warm-start: shift previous optimal sequence by one step
    if (static_cast<int>(prev_optimal_.size()) != T)
        prev_optimal_.assign(T, {0.f, 0.f});

    // Shift left: discard step 0, duplicate last
    for (int t = 0; t < T - 1; ++t)
        prev_optimal_[t] = prev_optimal_[t + 1];
    // Last step: keep same (will be overwritten by sampling)

    // 6. Compute nominal control and blend with warm-started sequence
    Seed nominal = compute_nominal(carrot_robot, T);
    for (int t = 0; t < T; ++t)
    {
        const float wa = params.warm_start_adv_weight;
        const float wr = params.warm_start_rot_weight;
        prev_optimal_[t].adv = wa * prev_optimal_[t].adv + (1.f - wa) * nominal.adv[t];
        prev_optimal_[t].rot = wr * prev_optimal_[t].rot + (1.f - wr) * nominal.rot[t];
    }

    // 7. Sample K trajectories as perturbations of prev_optimal_ with AR(1) noise
    auto seeds = sample_trajectories(carrot_robot);

    // 8. Simulate, optimize, and score each sample
    std::vector<SimResult> results(K);
    int best_idx = -1;
    float best_G = std::numeric_limits<float>::max();

    for (int k = 0; k < K; ++k)
    {
        optimize_seed(seeds[k], carrot_robot);
        results[k] = simulate_and_score(seeds[k], carrot_robot);
        if (results[k].G_total < best_G)
        {
            best_G = results[k].G_total;
            best_idx = k;
        }
    }

    // 9. MPPI weighted average over the FULL T-step sequence
    //    w_k = exp(-(G_k - G_min) / λ)
    //    u*_t = Σ w_k · u_k_t   for each t in [0, T)
    std::vector<ControlStep> optimal(T, {0.f, 0.f});
    {
        const float lambda = params.mppi_lambda;
        const float G_min = best_G;

        std::vector<float> weights(K);
        float w_sum = 0.f;
        int num_collisions = 0;

        for (int k = 0; k < K; ++k)
        {
            if (results[k].collides)
            {
                weights[k] = 0.f;  // colliding trajectories get zero weight
                num_collisions++;
            }
            else
            {
                weights[k] = std::exp(-(results[k].G_total - G_min) / lambda);
            }
            w_sum += weights[k];
        }

        if (w_sum > 1e-10f)
        {
            for (int k = 0; k < K; ++k)
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
            // Fallback: use best trajectory
            const int steps_b = static_cast<int>(seeds[best_idx].adv.size());
            for (int t = 0; t < std::min(T, steps_b); ++t)
            {
                optimal[t].adv = seeds[best_idx].adv[t];
                optimal[t].rot = seeds[best_idx].rot[t];
            }
        }

        // 10. Adaptive sigma based on collision ratio and ESDF
        const float valid_ratio = 1.f - static_cast<float>(num_collisions) / static_cast<float>(K);
        const float esdf_at_robot = query_esdf(0.f, 0.f);
        const bool near_obstacle = (esdf_at_robot < params.d_safe * 1.5f);

        if (valid_ratio < 0.5f)
        {
            // Many collisions: reduce advance (don't push into walls)
            // but INCREASE rotation (explore lateral escape routes)
            adaptive_sigma_adv_ *= 0.85f;
            adaptive_sigma_rot_ *= 1.1f;
        }
        else if (valid_ratio > 0.9f && !near_obstacle)
        {
            // Free space: slightly reduce rotation sigma (reduce nodding)
            // keep advance sigma moderate
            adaptive_sigma_adv_ *= 1.01f;
            adaptive_sigma_rot_ *= 0.95f;
        }
        adaptive_sigma_adv_ = std::clamp(adaptive_sigma_adv_, params.sigma_min_adv, params.sigma_max_adv);
        adaptive_sigma_rot_ = std::clamp(adaptive_sigma_rot_, params.sigma_min_rot, params.sigma_max_rot);
    }

    // Store optimal sequence for next cycle's warm-start
    prev_optimal_ = optimal;

    // 11. Extract first-step command
    float cmd_adv = optimal[0].adv;
    float cmd_rot = optimal[0].rot;

    // 12. Viz: trajectories in room frame (subsample for drawing)
    {
        const Eigen::Matrix2f R = robot_pose.linear();
        const Eigen::Vector2f t_pos = robot_pose.translation();

        // Draw a subset of trajectories for visualization
        const int n_draw = std::min(params.num_trajectories_to_draw, K);
        out.trajectories_room.resize(n_draw);
        // Pick evenly spaced + best
        for (int i = 0; i < n_draw; ++i)
        {
            int k = (i == 0 && best_idx >= 0) ? best_idx
                    : (i * K) / std::max(n_draw, 1);
            k = std::clamp(k, 0, K - 1);
            auto& tr = out.trajectories_room[i];
            tr.reserve(results[k].positions.size() + 1);
            tr.push_back(t_pos);
            for (const auto& p : results[k].positions)
                tr.push_back(R * p + t_pos);
        }
        out.best_trajectory_idx = 0;  // first drawn is always best
    }

    // 13. Smooth + Gaussian brake
    Eigen::Vector3f raw(cmd_adv, 0.f, cmd_rot);
    if (has_prev_vel_)
        smoothed_vel_ = params.velocity_smoothing * smoothed_vel_ + (1.f - params.velocity_smoothing) * raw;
    else { smoothed_vel_ = raw; has_prev_vel_ = true; }

    constexpr float gauss_k = 2.0f;
    const float rot_ratio = smoothed_vel_[2] / params.max_rot;
    const float brake = std::exp(-gauss_k * rot_ratio * rot_ratio);

    out.adv  = smoothed_vel_[0] * brake;
    out.side = smoothed_vel_[1];
    out.rot  = smoothed_vel_[2];

    // Debug
    static int dbg = 0;
    if (++dbg % 20 == 0)
    {
        std::cout << "[TrajCtrl] MPPI K=" << K
                  << " T=" << T
                  << " σ_adv=" << std::fixed << std::setprecision(3) << adaptive_sigma_adv_
                  << " σ_rot=" << adaptive_sigma_rot_
                  << " cmd(adv=" << cmd_adv << " rot=" << cmd_rot << ")"
                  << " sent(adv=" << out.adv << " rot=" << out.rot << ")"
                  << " bestG=" << std::setprecision(1) << best_G
                  << " carrot=" << static_cast<int>(std::atan2(carrot_robot.x(), carrot_robot.y()) * 180.f / M_PI) << "°"
                  << " dist=" << out.dist_to_goal
                  << " esdf=" << out.min_esdf
                  << " brake=" << static_cast<int>(brake * 100) << "%\n";
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
// Core: K-n_inject random samples as AR(1) perturbations of prev_optimal_
// Injections: n_inject deterministic seeds at fixed angular offsets from carrot
//   to guarantee lateral coverage (obstacle avoidance, door navigation)
//
// The injection count scales with proximity to obstacles:
//   far from obstacles → 2 injections (just ±30°)
//   near obstacles → 8 injections (±30°, ±60°, ±90°, ±120°)
// ============================================================================

std::vector<TrajectoryController::Seed> TrajectoryController::sample_trajectories(
    const Eigen::Vector2f& carrot_robot)
{
    const int K = params.num_samples;
    const int T = params.trajectory_steps;
    const float alpha = params.noise_alpha;
    const float innovation = std::sqrt(std::max(0.f, 1.f - alpha * alpha));
    const float dt = params.trajectory_dt;
    const float carrot_angle = std::atan2(carrot_robot.x(), carrot_robot.y());

    std::vector<Seed> seeds;
    seeds.reserve(K);

    // --- Structured exploration injections ---
    // Determine how many based on ESDF at robot position
    const float esdf_here = query_esdf(0.f, 0.f);
    const float esdf_ratio = std::clamp(esdf_here / params.d_safe, 0.f, 2.f);  // 0=collision, 2=very free

    // Angular offsets for injected seeds (radians)
    // Always inject ±30°; add wider angles when near obstacles
    std::vector<float> inject_offsets = {0.5f, -0.5f};  // ±30° always
    if (esdf_ratio < 1.5f)
    {
        inject_offsets.push_back(1.05f);   // +60°
        inject_offsets.push_back(-1.05f);  // -60°
    }
    if (esdf_ratio < 1.0f)
    {
        inject_offsets.push_back(1.57f);   // +90°
        inject_offsets.push_back(-1.57f);  // -90°
    }
    if (esdf_ratio < 0.6f)
    {
        inject_offsets.push_back(2.09f);   // +120°
        inject_offsets.push_back(-2.09f);  // -120°
    }

    // Generate injection seeds: constant rotation offset that decays toward carrot
    for (float offset : inject_offsets)
    {
        Seed s;
        s.adv.resize(T);
        s.rot.resize(T);
        float theta = 0.f;

        for (int t = 0; t < T; ++t)
        {
            float phase = static_cast<float>(t) / static_cast<float>(std::max(T - 1, 1));
            // Blend from offset direction to carrot over the trajectory (sqrt for slow convergence)
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

        // Initialize AR(1) state
        float eps_adv = normal_(rng_) * adaptive_sigma_adv_;
        float eps_rot = normal_(rng_) * adaptive_sigma_rot_;

        for (int t = 0; t < T; ++t)
        {
            // Perturbed control = warm-started base + correlated noise
            float adv = prev_optimal_[t].adv + eps_adv;
            float rot = prev_optimal_[t].rot + eps_rot;

            // Clamp to kinematic limits
            s.adv[t] = std::clamp(adv, 0.05f, params.max_adv);
            s.rot[t] = std::clamp(rot, -params.max_rot, params.max_rot);

            // AR(1) update: noise evolves smoothly
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

void TrajectoryController::build_esdf(const std::vector<Eigen::Vector3f>& lidar_points)
{
    const float res = params.grid_resolution;
    const float half = params.grid_half_size;
    const int N = static_cast<int>(2.f * half / res);
    esdf_N_ = N;

    std::vector<int> occ(N * N, 0);
    for (const auto& p : lidar_points)
    {
        const int ci = static_cast<int>((p.x() + half) / res);
        const int cj = static_cast<int>((p.y() + half) / res);
        if (ci >= 0 && ci < N && cj >= 0 && cj < N)
            occ[cj * N + ci] = 1;
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

} // namespace rc

