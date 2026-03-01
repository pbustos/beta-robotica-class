#include "trajectory_controller.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <limits>
#include <random>

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
    prev_best_cmd_ = Eigen::Vector3f::Zero();
    // Reset adaptive to base params
    adaptive_.num_seeds = params.num_seeds;
    adaptive_.steps = params.trajectory_steps;
    adaptive_.spread = 1.0f;
    adaptive_.drive_speed = params.max_adv;
    adaptive_.best_angle = 0.f;
    adaptive_.collision_ratio = 0.f;
    adaptive_.best_G = 1e9f;
    adaptive_.efe_smoothed = 0.f;
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
    prev_best_cmd_ = Eigen::Vector3f::Zero();
}

std::optional<Eigen::Vector2f> TrajectoryController::current_waypoint_room() const
{
    if (!active_ || wp_index_ >= static_cast<int>(path_room_.size()))
        return std::nullopt;
    return path_room_[wp_index_];
}

// ============================================================================
// Main compute
// ============================================================================

TrajectoryController::ControlOutput TrajectoryController::compute(
    const std::vector<Eigen::Vector3f>& lidar_points,
    const Eigen::Affine2f& robot_pose)
{
    ControlOutput out;
    if (!active_ || path_room_.empty()) { active_ = false; out.goal_reached = true; return out; }

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

    // 5. ALWAYS generate, optimize and score seeds (for viz + adaptive feedback)
    auto seeds = generate_seeds(carrot_robot);
    std::vector<SimResult> results(seeds.size());
    int best_idx = -1;
    float best_G = std::numeric_limits<float>::max();

    for (size_t i = 0; i < seeds.size(); ++i)
    {
        optimize_seed(seeds[i], carrot_robot);
        results[i] = simulate_and_score(seeds[i], carrot_robot);
        if (results[i].G_total < best_G)
        {
            best_G = results[i].G_total;
            best_idx = static_cast<int>(i);
        }
    }

    // Update adaptive state for next cycle
    update_adaptive(results, seeds, best_idx, carrot_robot);

    // 6. MPPI weighted average: softmax over all seeds' first-step commands
    //    w_i = exp(-(G_i - G_min) / λ)   normalized so Σ w_i = 1
    //    u* = Σ w_i · u_i
    float cmd_adv = 0.f, cmd_rot = 0.f;
    {
        const float lambda = params.mppi_lambda;
        const float G_min = best_G;  // for numerical stability

        // Compute unnormalized weights
        std::vector<float> weights(seeds.size());
        float w_sum = 0.f;
        for (size_t i = 0; i < seeds.size(); ++i)
        {
            weights[i] = std::exp(-(results[i].G_total - G_min) / lambda);
            w_sum += weights[i];
        }

        // Weighted average of first-step commands
        if (w_sum > 1e-10f)
        {
            for (size_t i = 0; i < seeds.size(); ++i)
            {
                const float w = weights[i] / w_sum;
                cmd_adv += w * seeds[i].adv[0];
                cmd_rot += w * seeds[i].rot[0];
            }
        }
        else if (best_idx >= 0)
        {
            // Fallback: all weights collapsed → use best
            cmd_adv = seeds[best_idx].adv[0];
            cmd_rot = seeds[best_idx].rot[0];
        }
    }

    // 7. Viz: all trajectories in room frame (always fresh every tick)
    const Eigen::Matrix2f R = robot_pose.linear();
    const Eigen::Vector2f t = robot_pose.translation();
    out.trajectories_room.resize(seeds.size());
    for (size_t i = 0; i < results.size(); ++i)
    {
        auto& tr = out.trajectories_room[i];
        tr.reserve(results[i].positions.size() + 1);
        tr.push_back(t);
        for (const auto& p : results[i].positions)
            tr.push_back(R * p + t);
    }
    out.best_trajectory_idx = best_idx;

    // 8. Smooth + Gaussian brake
    Eigen::Vector3f raw(cmd_adv, 0.f, cmd_rot);
    if (has_prev_vel_)
        smoothed_vel_ = params.velocity_smoothing * smoothed_vel_ + (1.f - params.velocity_smoothing) * raw;
    else { smoothed_vel_ = raw; has_prev_vel_ = true; }
    prev_best_cmd_ = raw;

    // Gaussian brake: modulate advance with rotation.
    // adv_out = adv * exp(-k * (rot / max_rot)²)
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
        const float relu_d = std::max(0.f, adaptive_.efe_smoothed - adaptive_.efe_threshold);
        const float diff = std::tanh(adaptive_.efe_sensitivity * relu_d);
        std::cout << "[TrajCtrl] MPPI"
                  << " seeds=" << adaptive_.num_seeds
                  << " steps=" << adaptive_.steps
                  << " spread=" << static_cast<int>(adaptive_.spread * 180.f / M_PI) << "°"
                  << " diff=" << static_cast<int>(diff * 100) << "%"
                  << " collR=" << static_cast<int>(adaptive_.collision_ratio * 100) << "%"
                  << " λ=" << std::fixed << std::setprecision(1) << params.mppi_lambda
                  << " cmd(adv=" << cmd_adv << " rot=" << cmd_rot << ")"
                  << " sent(adv=" << out.adv << " rot=" << out.rot << ")"
                  << " bestG=" << best_G
                  << " carrot=" << static_cast<int>(std::atan2(carrot_robot.x(), carrot_robot.y()) * 180.f / M_PI) << "°"
                  << " dist=" << out.dist_to_goal
                  << " esdf=" << out.min_esdf
                  << " brake=" << static_cast<int>(brake * 100) << "%\n";
    }

    return out;
}

// ============================================================================
// Adaptive feedback — EFE-based smooth adaptation
//
// Uses EMA of log(1+G_best) to smoothly transition between regimes:
//   T(t) = T_base + (T_max - T_base) · tanh(β · ReLU(Ĝ(t) - G₀))
//
// Same formula applied to num_seeds, steps, and spread.
// When EFE is low (clear path) → few seeds, short horizon, narrow spread
// When EFE is high (stuck/blocked) → many seeds, long horizon, wide spread
// ============================================================================

void TrajectoryController::update_adaptive(const std::vector<SimResult>& results,
                                            const std::vector<Seed>& seeds,
                                            int best_idx,
                                            const Eigen::Vector2f& carrot_robot)
{
    const int N = static_cast<int>(results.size());
    if (N == 0) return;

    const float carrot_dist = carrot_robot.norm();

    // Count collisions and compute score stats
    int num_collisions = 0;
    float G_best = std::numeric_limits<float>::max();
    for (const auto& r : results)
    {
        if (r.collides) num_collisions++;
        G_best = std::min(G_best, r.G_total);
    }

    const float coll_ratio = static_cast<float>(num_collisions) / static_cast<float>(N);
    adaptive_.collision_ratio = coll_ratio;
    adaptive_.best_G = G_best;

    // Best seed angle (direction it took)
    if (best_idx >= 0 && !seeds[best_idx].rot.empty())
    {
        float angle = 0.f;
        for (float r : seeds[best_idx].rot)
            angle += r * params.trajectory_dt;
        adaptive_.best_angle = angle;
    }

    // ---- EFE-based smooth adaptation ----
    // EMA of log(1 + G_best) as difficulty metric
    const float log_G = std::log(1.f + G_best);
    adaptive_.efe_smoothed = adaptive_.efe_gamma * adaptive_.efe_smoothed
                           + (1.f - adaptive_.efe_gamma) * log_G;

    // Difficulty factor: 0 (easy/cruise) to 1 (hard/explore)
    // tanh(sensitivity * ReLU(efe_smoothed - threshold))
    const float relu_diff = std::max(0.f, adaptive_.efe_smoothed - adaptive_.efe_threshold);
    const float difficulty = std::tanh(adaptive_.efe_sensitivity * relu_diff);

    // Collision boost: extra difficulty if many seeds collide
    const float coll_boost = std::max(0.f, (coll_ratio - 0.3f) / 0.7f);  // 0..1
    const float effective_difficulty = std::min(1.f, difficulty + 0.5f * coll_boost);

    // Interpolate between min and max for each parameter
    const float min_spread = params.min_spread_deg * static_cast<float>(M_PI) / 180.f;
    const float max_spread = params.max_spread_deg * static_cast<float>(M_PI) / 180.f;

    adaptive_.num_seeds = params.min_seeds +
        static_cast<int>(effective_difficulty * static_cast<float>(params.max_seeds - params.min_seeds));
    adaptive_.steps = params.min_steps +
        static_cast<int>(effective_difficulty * static_cast<float>(params.max_steps - params.min_steps));
    adaptive_.spread = min_spread + effective_difficulty * (max_spread - min_spread);

    // Drive speed: slower when exploring (high difficulty), faster when cruising
    adaptive_.drive_speed = params.max_adv * (1.f - 0.6f * effective_difficulty);
    adaptive_.drive_speed = std::clamp(adaptive_.drive_speed,
                                        0.1f,
                                        std::min(params.max_adv, std::max(0.15f, carrot_dist * 0.5f)));
}

// ============================================================================
// Seed generation — Pure geometric approach (Python reference style)
//
// Seeds are generated using only direction blending — NO ESDF queries.
// The ESDF is used later in optimize_seed() to push seeds away from
// obstacles, and in simulate_and_score() to evaluate them.
//
// Seed types:
//   - Seed 0:   Warm start from previous best command
//   - Seed 1:   Direct to carrot (turn then go straight)
//   - Seed 2,3: Left and right curves (offset angle decaying to goal)
//   - Seed 4:   Momentum (continue previous velocity, blend to goal)
//   - Seeds 5..K-1: Spread seeds (angular fan around goal with sqrt blending)
// ============================================================================

std::vector<TrajectoryController::Seed> TrajectoryController::generate_seeds(
    const Eigen::Vector2f& carrot_robot)
{
    const int K = adaptive_.num_seeds;
    const int steps = adaptive_.steps;
    // Y+ forward convention: atan2(x, y) gives angle from Y+ axis
    const float carrot_angle = std::atan2(carrot_robot.x(), carrot_robot.y());
    const float drive_speed = adaptive_.drive_speed;
    const float spread = adaptive_.spread;

    std::vector<Seed> seeds;
    seeds.reserve(K);

    // ---- Seed 0: Warm start from previous best command ----
    if (has_prev_vel_)
    {
        seeds.push_back(make_momentum_seed(steps, drive_speed, carrot_angle));
    }

    // ---- Seed 1: Direct to carrot (smooth turn + advance) ----
    // Always advance at full speed; rotation computed to reach carrot angle.
    // The Gaussian brake at output will modulate advance based on rotation.
    seeds.push_back(make_spread_seed(carrot_angle, carrot_angle, steps, drive_speed));

    // ---- Seeds 2,3: Left and right curves ----
    // Offset ±45° decaying to goal direction over the trajectory
    seeds.push_back(make_curve_seed(carrot_angle, +static_cast<float>(M_PI) / 4.f, steps, drive_speed));
    seeds.push_back(make_curve_seed(carrot_angle, -static_cast<float>(M_PI) / 4.f, steps, drive_speed));

    // ---- Remaining seeds: angular spread fan with blending toward goal ----
    // Blend centre: 70% carrot direction + 30% adaptive best direction
    const float centre_angle = 0.7f * carrot_angle + 0.3f * adaptive_.best_angle;

    const int num_spread = K - static_cast<int>(seeds.size());
    if (num_spread > 0)
    {
        // Effective spread: clamp so no seed starts more than ±90° from carrot
        const float max_half_spread = static_cast<float>(M_PI) * 0.5f;
        const float effective_spread = std::min(spread, max_half_spread);

        for (int k = 0; k < num_spread; ++k)
        {
            float frac = (num_spread == 1) ? 0.5f
                         : static_cast<float>(k) / static_cast<float>(num_spread - 1);
            float target_angle = centre_angle + effective_spread * (2.f * frac - 1.f);
            seeds.push_back(make_spread_seed(target_angle, carrot_angle, steps, drive_speed));
        }
    }

    return seeds;
}

// ============================================================================
// Curve seed: starts with offset_angle from carrot, decays linearly to carrot.
// Simulates diff-drive kinematics step by step.
// Python equivalent: offset_angles = side * pi/4 * (1 - phases)
// ============================================================================

TrajectoryController::Seed TrajectoryController::make_curve_seed(
    float carrot_angle, float offset_angle, int steps, float speed)
{
    Seed seed;
    seed.adv.resize(steps);
    seed.rot.resize(steps);

    const float dt = params.trajectory_dt;
    float theta = 0.f;

    for (int s = 0; s < steps; ++s)
    {
        float phase = static_cast<float>(s) / static_cast<float>(std::max(steps - 1, 1));
        // Desired direction: offset decays linearly to zero
        float desired_angle = carrot_angle + offset_angle * (1.f - phase);

        // Rotation to reach desired angle
        float angle_err = desired_angle - theta;
        while (angle_err > static_cast<float>(M_PI)) angle_err -= 2.f * static_cast<float>(M_PI);
        while (angle_err < -static_cast<float>(M_PI)) angle_err += 2.f * static_cast<float>(M_PI);

        float rot_cmd = std::clamp(angle_err / dt, -params.max_rot, params.max_rot);
        float adv_cmd = std::clamp(speed * 0.85f, 0.05f, params.max_adv);

        seed.adv[s] = adv_cmd;
        seed.rot[s] = rot_cmd;

        // Simulate forward
        theta += rot_cmd * dt;
    }
    return seed;
}

// ============================================================================
// Momentum seed: continues previous velocity and blends toward goal.
// Python equivalent: (1 - blend*0.5) * prev_dir + blend*0.5 * goal_dir
// ============================================================================

TrajectoryController::Seed TrajectoryController::make_momentum_seed(
    int steps, float speed, float carrot_angle)
{
    Seed seed;
    seed.adv.resize(steps);
    seed.rot.resize(steps);

    const float dt = params.trajectory_dt;
    float prev_adv = prev_best_cmd_[0];
    float prev_rot = prev_best_cmd_[2];

    // Estimate previous heading from prev_rot
    float prev_heading = prev_rot * dt * 3.f;  // rough estimate of where we were heading
    float theta = 0.f;

    for (int s = 0; s < steps; ++s)
    {
        float phase = static_cast<float>(s) / static_cast<float>(std::max(steps - 1, 1));
        // Blend from previous heading to goal direction
        float blend = phase * 0.5f;
        float desired_angle = (1.f - blend) * prev_heading + blend * carrot_angle;

        float angle_err = desired_angle - theta;
        while (angle_err > static_cast<float>(M_PI)) angle_err -= 2.f * static_cast<float>(M_PI);
        while (angle_err < -static_cast<float>(M_PI)) angle_err += 2.f * static_cast<float>(M_PI);

        float rot_cmd = std::clamp(angle_err / dt, -params.max_rot, params.max_rot);
        // Speed: blend from previous speed to target speed
        float adv_cmd = std::max(0.05f, (1.f - phase) * prev_adv + phase * speed * 0.8f);
        adv_cmd = std::clamp(adv_cmd, 0.05f, params.max_adv);

        seed.adv[s] = adv_cmd;
        seed.rot[s] = rot_cmd;

        theta += rot_cmd * dt;
    }
    return seed;
}

// ============================================================================
// Spread seed: starts heading in target_angle, blends toward carrot_angle
// using sqrt(phase) blending (slower convergence, more exploration early).
// Python equivalent: blends = (t_vals / T).pow(0.5)
//   blended = (1-blends)*direction + blends*goal_dir
// ============================================================================

TrajectoryController::Seed TrajectoryController::make_spread_seed(
    float target_angle, float carrot_angle, int steps, float speed)
{
    Seed seed;
    seed.adv.resize(steps);
    seed.rot.resize(steps);

    const float dt = params.trajectory_dt;
    float theta = 0.f;

    for (int s = 0; s < steps; ++s)
    {
        float phase = static_cast<float>(s) / static_cast<float>(std::max(steps - 1, 1));
        // sqrt blending: slow convergence early, faster late
        float blend = std::sqrt(phase);
        float desired_angle = (1.f - blend) * target_angle + blend * carrot_angle;

        float angle_err = desired_angle - theta;
        while (angle_err > static_cast<float>(M_PI)) angle_err -= 2.f * static_cast<float>(M_PI);
        while (angle_err < -static_cast<float>(M_PI)) angle_err += 2.f * static_cast<float>(M_PI);

        float rot_cmd = std::clamp(angle_err / dt, -params.max_rot, params.max_rot);
        float adv_cmd = std::clamp(speed * 0.85f, 0.05f, params.max_adv);

        seed.adv[s] = adv_cmd;
        seed.rot[s] = rot_cmd;

        theta += rot_cmd * dt;
    }

    return seed;
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

    // G_smooth: continuity with previous command
    float dv = seed.adv[0] - prev_best_cmd_[0];
    float dr = seed.rot[0] - prev_best_cmd_[2];
    float G_smooth = params.lambda_smooth * (dv * dv + dr * dr);

    // G_velocity: magnitude regularization + action change penalty
    // Only over actual simulated steps
    float G_vel_mag = 0.f;
    float G_vel_delta = 0.f;
    for (int s = 0; s < actual_steps; ++s)
    {
        G_vel_mag += seed.adv[s] * seed.adv[s] + seed.rot[s] * seed.rot[s];
        if (s > 0)
        {
            float da = seed.adv[s] - seed.adv[s - 1];
            float dro = seed.rot[s] - seed.rot[s - 1];
            G_vel_delta += da * da + dro * dro;
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


