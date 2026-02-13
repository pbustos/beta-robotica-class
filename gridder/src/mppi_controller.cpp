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

#include "mppi_controller.h"
#include "grid_esdf.h"  // For ESDF-based obstacle costs
#include <iostream>
#include <QDebug>     // For debug output
#include <algorithm>  // For std::shuffle, std::partial_sort

// ============================================================================
// Constructors
// ============================================================================

MPPIController::MPPIController()
    : MPPIController(Params{})
{
}

MPPIController::MPPIController(const Params& params)
    : params_(params)
    , rng_(std::random_device{}())
    , noise_vx_(0.0f, 1.0f)  // Standard normal, we scale by adaptive sigma
    , noise_vy_(0.0f, 1.0f)
    , noise_omega_(0.0f, 1.0f)
    , adaptive_sigma_vx_(params.sigma_vx)
    , adaptive_sigma_vy_(params.sigma_vy)
    , adaptive_sigma_omega_(params.sigma_omega)
    , adaptive_lambda_(params.lambda)
{
    // Initialize previous controls to zero
    prev_controls_.resize(params_.T);
    for (auto& ctrl : prev_controls_)
    {
        ctrl = ControlCommand{0.0f, 0.0f, 0.0f};
    }

    // Pre-allocate buffers for performance
    noise_buffer_vx_.resize(params_.K * params_.T);
    noise_buffer_vy_.resize(params_.K * params_.T);
    noise_buffer_omega_.resize(params_.K * params_.T);
}

// ============================================================================
// Public Methods
// ============================================================================

MPPIController::ControlCommand MPPIController::compute(
    const State& current_state,
    const std::vector<Eigen::Vector2f>& path,
    const std::vector<Eigen::Vector2f>& obstacles)
{
    // If no path, return zero control
    if (path.empty())
    {
        return ControlCommand{0.0f, 0.0f, 0.0f};
    }

    // Check if goal reached
    if (goalReached(current_state, path.back()))
    {
        return ControlCommand{0.0f, 0.0f, 0.0f};
    }

    // Compute nominal control towards the path (provides a good initial guess)
    ControlCommand nominal_control = computeNominalControl(current_state, path);

    // Warm start: shift previous control sequence
    warmStart();

    // Blend warm-started controls with nominal control
    // Use higher weight for nominal on vx to prevent lateral drift
    for (int t = 0; t < params_.T; ++t)
    {
        prev_controls_[t].vx = 0.2f * prev_controls_[t].vx + 0.8f * nominal_control.vx;  // Strong bias to nominal for lateral
        prev_controls_[t].vy = 0.5f * prev_controls_[t].vy + 0.5f * nominal_control.vy;
        prev_controls_[t].omega = 0.5f * prev_controls_[t].omega + 0.5f * nominal_control.omega;
    }

    // =========================================================================
    // NOISE GENERATION: AR(1) time-correlated OR i.i.d. (configurable)
    // AR(1): epsilon_t = alpha * epsilon_{t-1} + eta_t
    // eta_t ~ N(0, (1-alpha^2) * sigma^2) to preserve marginal variance
    // =========================================================================
    const int total_samples = params_.K * params_.T;
    if (static_cast<int>(noise_buffer_vx_.size()) != total_samples)
    {
        noise_buffer_vx_.resize(total_samples);
        noise_buffer_vy_.resize(total_samples);
        noise_buffer_omega_.resize(total_samples);
    }

    if (params_.use_time_correlated_noise)
    {
        // AR(1) correlated noise generation
        const float alpha = params_.noise_alpha;
        const float innovation_scale = std::sqrt(std::max(0.0f, 1.0f - alpha * alpha));

        for (int k = 0; k < params_.K; ++k)
        {
            // Initialize AR(1) state for this trajectory
            float eps_vx = noise_vx_(rng_) * adaptive_sigma_vx_;
            float eps_vy = noise_vy_(rng_) * adaptive_sigma_vy_;
            float eps_omega = noise_omega_(rng_) * adaptive_sigma_omega_;

            const int noise_base = k * params_.T;
            for (int t = 0; t < params_.T; ++t)
            {
                // Store current noise value
                noise_buffer_vx_[noise_base + t] = eps_vx;
                noise_buffer_vy_[noise_base + t] = eps_vy;
                noise_buffer_omega_[noise_base + t] = eps_omega;

                // AR(1) update for next timestep
                eps_vx = alpha * eps_vx + innovation_scale * adaptive_sigma_vx_ * noise_vx_(rng_);
                eps_vy = alpha * eps_vy + innovation_scale * adaptive_sigma_vy_ * noise_vy_(rng_);
                eps_omega = alpha * eps_omega + innovation_scale * adaptive_sigma_omega_ * noise_omega_(rng_);
            }
        }
    }
    else
    {
        // Standard i.i.d. noise (fallback)
        for (int i = 0; i < total_samples; ++i)
        {
            noise_buffer_vx_[i] = noise_vx_(rng_) * adaptive_sigma_vx_;
            noise_buffer_vy_[i] = noise_vy_(rng_) * adaptive_sigma_vy_;
            noise_buffer_omega_[i] = noise_omega_(rng_) * adaptive_sigma_omega_;
        }
    }

    // =========================================================================
    // Pre-compute obstacle squared distances threshold
    // =========================================================================
    const float robot_radius_sq = params_.robot_radius * params_.robot_radius;
    const float buffer_radius = params_.robot_radius + params_.collision_buffer;
    const float buffer_radius_sq = buffer_radius * buffer_radius;

    // =========================================================================
    // OPTIMIZATION 3: Subsample obstacles for faster collision checking
    // =========================================================================
    std::vector<Eigen::Vector2f> subsampled_obstacles;
    const size_t max_obstacles = 200;  // Limit number of obstacles to check
    if (obstacles.size() > max_obstacles)
    {
        subsampled_obstacles.reserve(max_obstacles);
        const size_t step = obstacles.size() / max_obstacles;
        for (size_t i = 0; i < obstacles.size(); i += step)
        {
            subsampled_obstacles.push_back(obstacles[i]);
        }
    }
    const auto& obs_to_use = (obstacles.size() > max_obstacles) ? subsampled_obstacles : obstacles;

    // Storage for sampled trajectories and costs
    std::vector<std::vector<ControlCommand>> all_controls(params_.K);
    std::vector<std::vector<State>> all_trajectories(params_.K);
    std::vector<float> costs(params_.K);

    // Precompute sin/cos for initial state
    const float cos_theta_init = std::cos(current_state.theta);
    const float sin_theta_init = std::sin(current_state.theta);

    // Cost breakdown structure for debug
    struct CostBreakdown {
        float path_cost = 0.f;
        float obstacle_cost = 0.f;
        float smoothness_cost = 0.f;
        float speed_cost = 0.f;
        float goal_cost = 0.f;
        float total() const { return path_cost + obstacle_cost + smoothness_cost + speed_cost + goal_cost; }
    };
    std::vector<CostBreakdown> cost_breakdowns(params_.K);

    // Sample K trajectories
    for (int k = 0; k < params_.K; ++k)
    {
        std::vector<ControlCommand> controls(params_.T);
        std::vector<State> trajectory;
        trajectory.reserve(params_.T + 1);
        trajectory.push_back(current_state);

        State state = current_state;
        float cos_theta = cos_theta_init;
        float sin_theta = sin_theta_init;

        bool collision = false;
        float traj_cost = 0.0f;
        CostBreakdown& breakdown = cost_breakdowns[k];

        // Generate control sequence with noise
        const int noise_base = k * params_.T;
        for (int t = 0; t < params_.T && !collision; ++t)
        {
            // Add noise to previous (warm-started) controls
            ControlCommand ctrl;
            ctrl.vx = std::clamp(prev_controls_[t].vx + noise_buffer_vx_[noise_base + t],
                                 -params_.max_vx, params_.max_vx);
            ctrl.vy = std::clamp(prev_controls_[t].vy + noise_buffer_vy_[noise_base + t],
                                 -params_.max_vy, params_.max_vy);
            ctrl.omega = std::clamp(prev_controls_[t].omega + noise_buffer_omega_[noise_base + t],
                                    -params_.max_omega, params_.max_omega);
            controls[t] = ctrl;

            // =========================================================================
            // OPTIMIZATION 6: Inline simulateStep to avoid function call overhead
            // =========================================================================
            float vx_world = ctrl.vx * cos_theta - ctrl.vy * sin_theta;
            float vy_world = ctrl.vx * sin_theta + ctrl.vy * cos_theta;

            state.x += vx_world * params_.dt;
            state.y += vy_world * params_.dt;
            state.theta -= ctrl.omega * params_.dt;  // INVERTED: try negative omega

            // Fast theta normalization
            if (state.theta > M_PI) state.theta -= 2.0f * M_PI;
            else if (state.theta < -M_PI) state.theta += 2.0f * M_PI;

            // Update sin/cos for next iteration
            cos_theta = std::cos(state.theta);
            sin_theta = std::sin(state.theta);

            trajectory.push_back(state);

            // =========================================================================
            // Smooth obstacle distance using softmin over k nearest points
            // d_softmin = -1/β * log(Σ exp(-β * d_i))
            // This reduces discontinuities from raw LiDAR point min
            // =========================================================================
            constexpr int k_nearest = 10;  // Number of nearest points for softmin
            constexpr float softmin_beta = 0.02f;  // Smoothing parameter (1/mm scale)

            // Collect distances to all obstacles (squared for efficiency)
            thread_local std::vector<float> distances_sq;
            distances_sq.clear();
            distances_sq.reserve(obs_to_use.size());

            bool hard_collision = false;
            bool in_buffer_zone = false;
            float min_dist_sq_raw = std::numeric_limits<float>::max();

            for (const auto& obs : obs_to_use)
            {
                float dx = state.x - obs.x();
                float dy = state.y - obs.y();
                float dist_sq = dx * dx + dy * dy;
                distances_sq.push_back(dist_sq);

                if (dist_sq < min_dist_sq_raw)
                    min_dist_sq_raw = dist_sq;

                // Hard collision: d < robot_radius → invalid
                if (dist_sq < robot_radius_sq)
                {
                    hard_collision = true;
                    break;
                }
                // Buffer zone: robot_radius < d < robot_radius + buffer → huge finite cost
                else if (dist_sq < buffer_radius_sq)
                {
                    in_buffer_zone = true;
                }
            }

            if (hard_collision)
            {
                collision = true;
            }
            else if (!distances_sq.empty())
            {
                // Add huge cost if in buffer zone (but still valid)
                if (in_buffer_zone)
                {
                    float min_dist = std::sqrt(min_dist_sq_raw);
                    float buffer_pen = buffer_radius - min_dist;  // How deep into buffer
                    // Exponential cost that grows rapidly as we approach robot_radius
                    float buffer_cost = params_.w_obstacle * 50.0f * std::exp(buffer_pen / 30.0f);
                    traj_cost += buffer_cost;
                    breakdown.obstacle_cost += buffer_cost;
                }

                // Partial sort to get k smallest distances
                int k = std::min(k_nearest, static_cast<int>(distances_sq.size()));
                std::partial_sort(distances_sq.begin(), distances_sq.begin() + k, distances_sq.end());

                // Compute softmin: d_softmin = -1/β * log(Σ exp(-β * d_i))
                // For numerical stability, factor out the minimum:
                // softmin = d_min - 1/β * log(Σ exp(-β * (d_i - d_min)))
                float d_min = std::sqrt(distances_sq[0]);
                float sum_exp = 0.0f;
                for (int i = 0; i < k; ++i)
                {
                    float d_i = std::sqrt(distances_sq[i]);
                    sum_exp += std::exp(-softmin_beta * (d_i - d_min));
                }
                float softmin_dist = d_min - (1.0f / softmin_beta) * std::log(sum_exp);

                // Use softmin distance for obstacle cost
                if (softmin_dist < params_.safety_margin)
                {
                    float pen = params_.safety_margin - softmin_dist;
                    pen = std::clamp(pen, 0.f, params_.safety_margin - params_.robot_radius);

                    // Softplus with zero baseline: k_sp * (softplus(x) - log(2))
                    // This makes obstacle cost = 0 at pen = 0
                    float x = pen / params_.obstacle_decay;
                    float sp = std::log1p(std::exp(std::min(x, 20.0f)));
                    constexpr float sp0 = 0.693147f;  // log(2)
                    constexpr float k_sp = 1.44f;
                    float softplus_term = k_sp * (sp - sp0);  // 0 at pen=0

                    float obs_c = params_.w_obstacle * (pen * pen / 1000.0f + softplus_term);
                    traj_cost += obs_c;
                    breakdown.obstacle_cost += obs_c;
                }
            }

            // Other costs (only if no collision)
            if (!collision)
            {
                // =========================================================================
                // Path following cost - use LINEAR distance, not squared
                // =========================================================================
                // =========================================================================
                float path_min_dist_sq = std::numeric_limits<float>::max();
                for (const auto& wp : path)
                {
                    float dx = state.x - wp.x();
                    float dy = state.y - wp.y();
                    float d_sq = dx * dx + dy * dy;
                    if (d_sq < path_min_dist_sq) path_min_dist_sq = d_sq;
                }
                // Use linear distance to avoid dominating obstacle cost
                float path_c = params_.w_path * std::sqrt(path_min_dist_sq);
                traj_cost += path_c;
                breakdown.path_cost += path_c;

                // Smoothness cost
                if (t > 0)
                {
                    float dvx = (ctrl.vx - controls[t-1].vx) / params_.max_vx;
                    float dvy = (ctrl.vy - controls[t-1].vy) / params_.max_vy;
                    float domega = (ctrl.omega - controls[t-1].omega) / params_.max_omega;
                    float smooth_c = params_.w_smoothness * (dvx*dvx + dvy*dvy + domega*domega);
                    traj_cost += smooth_c;
                    breakdown.smoothness_cost += smooth_c;
                }

                // Speed cost
                float speed_sq = ctrl.vx * ctrl.vx + ctrl.vy * ctrl.vy;
                float desired_speed = params_.max_vy * 0.5f;
                float speed_diff = std::sqrt(speed_sq) - desired_speed;
                float speed_c = params_.w_speed * speed_diff * speed_diff;
                traj_cost += speed_c;
                breakdown.speed_cost += speed_c;
            }
        }

        // Goal cost for final state
        if (!collision)
        {
            const Eigen::Vector2f& goal = path.back();
            float dx = state.x - goal.x();
            float dy = state.y - goal.y();
            float goal_c = params_.w_goal * std::sqrt(dx * dx + dy * dy);
            traj_cost += goal_c;
            breakdown.goal_cost += goal_c;
        }

        all_controls[k] = std::move(controls);
        all_trajectories[k] = std::move(trajectory);
        // Apply cost scaling to get values in MPPI-friendly range (10-1000 instead of 10k-1M)
        // Mark collisions with infinity temporarily - will be reassigned after finding min valid
        costs[k] = collision ? std::numeric_limits<float>::infinity() : (traj_cost / params_.cost_scale);
    }

    // =========================================================================
    // Finite penalty for invalid (collision) rollouts
    // Instead of infinity, use: S_invalid = S_min_valid + c * lambda
    // This keeps invalids strongly worse but doesn't destroy numeric scale
    // =========================================================================

    // First pass: find minimum valid cost
    float min_valid_cost = std::numeric_limits<float>::max();
    int num_invalid = 0;
    for (int k = 0; k < params_.K; ++k)
    {
        if (std::isinf(costs[k]))
            num_invalid++;
        else if (costs[k] < min_valid_cost)
            min_valid_cost = costs[k];
    }

    // If all are invalid, return zero control
    if (std::isinf(min_valid_cost))
    {
        return ControlCommand{0.0f, 0.0f, 0.0f};
    }

    // Compute valid ratio for later use in adaptive sigma
    const float valid_ratio = static_cast<float>(params_.K - num_invalid) / static_cast<float>(params_.K);

    // Second pass: assign finite penalty to invalid rollouts
    // invalid_bonus = c * lambda, where c=30 makes invalids ~exp(-30) ≈ 0 weight relative to best
    constexpr float invalid_cost_multiplier = 30.0f;
    const float invalid_cost = min_valid_cost + invalid_cost_multiplier * adaptive_lambda_;

    for (int k = 0; k < params_.K; ++k)
    {
        if (std::isinf(costs[k]))
            costs[k] = invalid_cost;
    }

    // Now min_cost is min_valid_cost (all costs are finite)
    float min_cost = min_valid_cost;

    // =========================================================================
    // OPTIMIZATION 9: Fused weight computation and normalization
    // =========================================================================
    const float inv_lambda = -1.0f / adaptive_lambda_;
    float weight_sum = 0.0f;
    std::vector<float> weights(params_.K);

    // Track statistics for debug
    float max_cost = min_cost;
    int valid_count = params_.K - num_invalid;  // Already computed above

    for (int k = 0; k < params_.K; ++k)
    {
        weights[k] = std::exp(inv_lambda * (costs[k] - min_cost));
        weight_sum += weights[k];
        if (costs[k] > max_cost) max_cost = costs[k];
    }

    if (weight_sum < 1e-10f)
    {
        return ControlCommand{0.0f, 0.0f, 0.0f};
    }

    const float inv_weight_sum = 1.0f / weight_sum;

    // =========================================================================
    // Compute Effective Sample Size (ESS) = 1 / sum(w_i^2)
    // ESS close to 1 means one trajectory dominates (bad)
    // ESS close to K means uniform weights (good exploration)
    // =========================================================================
    float sum_w_squared = 0.0f;
    for (int k = 0; k < params_.K; ++k)
    {
        float w_norm = weights[k] * inv_weight_sum;
        sum_w_squared += w_norm * w_norm;
    }
    const float ess = (sum_w_squared > 1e-10f) ? (1.0f / sum_w_squared) : static_cast<float>(valid_count);

    // Adaptive lambda: if ESS is too low, increase lambda to flatten the distribution
    // ESS < 5% of valid samples indicates poor diversity
    const float ess_ratio = ess / static_cast<float>(std::max(1, valid_count));

    if (params_.use_adaptive_covariance)  // Use same flag for adaptive lambda
    {
        if (ess_ratio < 0.05f && adaptive_lambda_ < 20.0f * params_.lambda)
        {
            // ESS very low (<5%) - increase lambda more aggressively
            adaptive_lambda_ *= 1.2f;
        }
        else if (ess_ratio > 0.2f && adaptive_lambda_ > params_.lambda)
        {
            // ESS healthy (>20%) - slowly restore original lambda
            adaptive_lambda_ *= 0.95f;
        }
    }

    // =========================================================================
    // Adaptive sigma based on valid rollout ratio AND ESS
    // Two-stage logic:
    //   1. allow_adapt gate: only learn sigma when conditions are favorable
    //   2. Emergency multipliers: always apply safety corrections
    // =========================================================================
    if (params_.use_adaptive_covariance)
    {
        // Gate: only allow covariance learning when conditions are favorable
        // This prevents "learning wrong sigma" when obstacles dominate
        const bool allow_adapt = (valid_ratio > 0.80f && ess_ratio > 0.10f);

        if (allow_adapt)
        {
            // ✅ Conditions favorable: allow weighted sigma update
            // Only increase exploration very slowly when very safe
            if (valid_ratio > 0.95f && ess_ratio > 0.30f)
            {
                adaptive_sigma_vx_ *= 1.005f;
                adaptive_sigma_vy_ *= 1.005f;
                adaptive_sigma_omega_ *= 1.005f;
            }
        }
        // else: ❌ skip covariance learning this cycle

        // Emergency multipliers: always apply regardless of allow_adapt
        if (valid_ratio < 0.70f)
        {
            // Too many collisions -> reduce exploration NOW
            adaptive_sigma_vx_ *= 0.70f;
            adaptive_sigma_vy_ *= 0.70f;
            adaptive_sigma_omega_ *= 0.70f;
        }
        else if (valid_ratio < 0.80f)
        {
            // Warning zone -> gentle reduction
            adaptive_sigma_vx_ *= 0.90f;
            adaptive_sigma_vy_ *= 0.90f;
            adaptive_sigma_omega_ *= 0.90f;
        }

        // Always clamp to safe ranges
        adaptive_sigma_vx_ = std::clamp(adaptive_sigma_vx_, params_.sigma_min_vx, params_.sigma_max_vx);
        adaptive_sigma_vy_ = std::clamp(adaptive_sigma_vy_, params_.sigma_min_vy, params_.sigma_max_vy);
        adaptive_sigma_omega_ = std::clamp(adaptive_sigma_omega_, params_.sigma_min_omega, params_.sigma_max_omega);
    }

    // Debug: show cost, weight, and ESS statistics
    static int stats_counter = 0;
    if (++stats_counter % 100 == 0)
    {
        // Find best (lowest cost) and a typical (median cost) trajectory for debug breakdown
        int best_idx = -1;
        int median_idx = -1;

        std::vector<std::pair<float, int>> valid_costs;
        for (int k = 0; k < params_.K; ++k)
        {
            if (!std::isinf(costs[k]))
            {
                valid_costs.emplace_back(costs[k], k);
            }
        }

        if (!valid_costs.empty())
        {
            std::sort(valid_costs.begin(), valid_costs.end());
            best_idx = valid_costs.front().second;
            median_idx = valid_costs[valid_costs.size() / 2].second;
        }

        qDebug() << "[MPPI Stats] Costs(scaled): min=" << min_cost << "max=" << max_cost
                 << "| ESS:" << ess << "/" << valid_count << "(" << (ess_ratio * 100) << "%)"
                 << "| lambda:" << adaptive_lambda_
                 << "| Valid:" << valid_count << "/" << params_.K << "(" << (valid_ratio * 100) << "%)";

        // Print cost breakdown for best and median trajectories (scaled)
        const float inv_scale = 1.0f / params_.cost_scale;
        if (best_idx >= 0)
        {
            const auto& best = cost_breakdowns[best_idx];
            qDebug() << "[MPPI Best ] path=" << (best.path_cost * inv_scale)
                     << "obs=" << (best.obstacle_cost * inv_scale)
                     << "smooth=" << (best.smoothness_cost * inv_scale)
                     << "speed=" << (best.speed_cost * inv_scale)
                     << "goal=" << (best.goal_cost * inv_scale)
                     << "TOTAL=" << (best.total() * inv_scale);
        }
        if (median_idx >= 0 && median_idx != best_idx)
        {
            const auto& med = cost_breakdowns[median_idx];
            qDebug() << "[MPPI Med  ] path=" << (med.path_cost * inv_scale)
                     << "obs=" << (med.obstacle_cost * inv_scale)
                     << "smooth=" << (med.smoothness_cost * inv_scale)
                     << "speed=" << (med.speed_cost * inv_scale)
                     << "goal=" << (med.goal_cost * inv_scale)
                     << "TOTAL=" << (med.total() * inv_scale);
        }
    }

    // =========================================================================
    // OPTIMIZATION 10: Fused weight normalization and control averaging
    // =========================================================================
    std::vector<ControlCommand> optimal_controls(params_.T, ControlCommand{0.0f, 0.0f, 0.0f});

    for (int k = 0; k < params_.K; ++k)
    {
        const float w = weights[k] * inv_weight_sum;
        if (w > 1e-10f)  // Skip zero-weight trajectories
        {
            for (int t = 0; t < params_.T; ++t)
            {
                optimal_controls[t].vx += w * all_controls[k][t].vx;
                optimal_controls[t].vy += w * all_controls[k][t].vy;
                optimal_controls[t].omega += w * all_controls[k][t].omega;
            }
        }
    }

    // Check for NaN
    for (int t = 0; t < params_.T; ++t)
    {
        if (std::isnan(optimal_controls[t].vx)) optimal_controls[t].vx = 0.0f;
        if (std::isnan(optimal_controls[t].vy)) optimal_controls[t].vy = 0.0f;
        if (std::isnan(optimal_controls[t].omega)) optimal_controls[t].omega = 0.0f;
    }

    // =========================================================================
    // ADAPTIVE COVARIANCE UPDATE (Safe Diagonal Version with Horizon Averaging)
    // sigma_i^2 = (1 - beta) * sigma_i^2 + beta * (1/T) * sum_t sum_k w_k * epsilon_{t,i}^2
    //
    // GATE: Only update sigma from weighted noise when conditions are favorable.
    // This prevents "σ jumps to max" moments caused by learning from poor distributions.
    // =========================================================================
    // Recompute allow_adapt gate (same logic as the multiplier section above)
    const bool allow_weighted_sigma_update = (valid_ratio > 0.80f && ess_ratio > 0.10f);

    if (params_.use_adaptive_covariance && allow_weighted_sigma_update)
    {
        float weighted_var_vx = 0.0f;
        float weighted_var_vy = 0.0f;
        float weighted_var_omega = 0.0f;
        float total_weight = 0.0f;

        for (int k = 0; k < params_.K; ++k)
        {
            const float w = weights[k] * inv_weight_sum;
            if (w > 1e-10f)
            {
                const int noise_base = k * params_.T;
                for (int t = 0; t < params_.T; ++t)
                {
                    float eps_vx = noise_buffer_vx_[noise_base + t];
                    float eps_vy = noise_buffer_vy_[noise_base + t];
                    float eps_omega = noise_buffer_omega_[noise_base + t];

                    weighted_var_vx += w * eps_vx * eps_vx;
                    weighted_var_vy += w * eps_vy * eps_vy;
                    weighted_var_omega += w * eps_omega * eps_omega;
                }
                total_weight += w * params_.T;
            }
        }

        // Horizon averaging for stability
        if (total_weight > 1e-10f)
        {
            weighted_var_vx /= static_cast<float>(params_.T);
            weighted_var_vy /= static_cast<float>(params_.T);
            weighted_var_omega /= static_cast<float>(params_.T);

            // Update adaptive sigmas with very slow exponential moving average (safe version)
            const float beta = params_.cov_adaptation_rate;  // 0.01 = very slow
            const float one_minus_beta = 1.0f - beta;

            float new_sigma_vx = std::sqrt(one_minus_beta * adaptive_sigma_vx_ * adaptive_sigma_vx_ + beta * weighted_var_vx);
            float new_sigma_vy = std::sqrt(one_minus_beta * adaptive_sigma_vy_ * adaptive_sigma_vy_ + beta * weighted_var_vy);
            float new_sigma_omega = std::sqrt(one_minus_beta * adaptive_sigma_omega_ * adaptive_sigma_omega_ + beta * weighted_var_omega);

            // Critical: clamp to safe range (narrower bounds for stability)
            adaptive_sigma_vx_ = std::clamp(new_sigma_vx, params_.sigma_min_vx, params_.sigma_max_vx);
            adaptive_sigma_vy_ = std::clamp(new_sigma_vy, params_.sigma_min_vy, params_.sigma_max_vy);
            adaptive_sigma_omega_ = std::clamp(new_sigma_omega, params_.sigma_min_omega, params_.sigma_max_omega);
        }
    }
    // else: ❌ skip weighted covariance update this cycle (conditions unfavorable)

    // Periodic debug: show adaptive sigma values
    static int cov_debug_counter = 0;
    if (++cov_debug_counter % 200 == 0)
    {
        qDebug() << "[MPPI] Adaptive sigma: vx=" << adaptive_sigma_vx_
                 << "vy=" << adaptive_sigma_vy_ << "omega=" << adaptive_sigma_omega_
                 << "(AR1=" << params_.use_time_correlated_noise
                 << ", adapt=" << params_.use_adaptive_covariance << ")";
    }

    // Store for next iteration (warm start)
    prev_controls_ = optimal_controls;

    // Compute optimal trajectory for visualization
    optimal_trajectory_.clear();
    optimal_trajectory_.reserve(params_.T + 1);
    State state = current_state;
    optimal_trajectory_.push_back(state);
    for (int t = 0; t < params_.T; ++t)
    {
        state = simulateStep(state, optimal_controls[t]);
        optimal_trajectory_.push_back(state);
    }

    // Store N random trajectories for visualization (to show diversity)
    best_trajectories_.clear();
    if (params_.num_trajectories_to_draw > 0)
    {
        // Select N random valid trajectories to show the actual sampling diversity
        std::vector<int> valid_indices;
        valid_indices.reserve(params_.K);
        for (int k = 0; k < params_.K; ++k)
        {
            if (!std::isinf(costs[k]) && !all_trajectories[k].empty())
            {
                valid_indices.push_back(k);
            }
        }

        // Shuffle and take first N
        std::shuffle(valid_indices.begin(), valid_indices.end(), rng_);
        const int n_to_draw = std::min(params_.num_trajectories_to_draw, static_cast<int>(valid_indices.size()));

        best_trajectories_.reserve(n_to_draw);
        for (int i = 0; i < n_to_draw; ++i)
        {
            best_trajectories_.push_back(std::move(all_trajectories[valid_indices[i]]));
        }
    }

    // Return first control command
    return optimal_controls[0];
}

// ============================================================================
// ESDF-based compute (smooth obstacle costs)
// ============================================================================

MPPIController::ControlCommand MPPIController::compute(
    const State& current_state,
    const std::vector<Eigen::Vector2f>& path,
    const std::vector<Eigen::Vector2f>& obstacles,
    const GridESDF* esdf)
{
    // Fallback to LiDAR-only version if no ESDF available
    if (esdf == nullptr || !esdf->has_precomputed_distances())
    {
        return compute(current_state, path, obstacles);
    }

    // If no path, return zero control
    if (path.empty())
        return ControlCommand{0.0f, 0.0f, 0.0f};

    // Check if goal reached
    if (goalReached(current_state, path.back()))
        return ControlCommand{0.0f, 0.0f, 0.0f};

    // Compute nominal control towards the path
    ControlCommand nominal_control = computeNominalControl(current_state, path);

    // Warm start: shift previous control sequence
    warmStart();

    // Blend warm-started controls with nominal control
    for (int t = 0; t < params_.T; ++t)
    {
        prev_controls_[t].vx = 0.2f * prev_controls_[t].vx + 0.8f * nominal_control.vx;
        prev_controls_[t].vy = 0.5f * prev_controls_[t].vy + 0.5f * nominal_control.vy;
        prev_controls_[t].omega = 0.5f * prev_controls_[t].omega + 0.5f * nominal_control.omega;
    }

    // Generate noise (same as original)
    const int total_samples = params_.K * params_.T;
    if (static_cast<int>(noise_buffer_vx_.size()) != total_samples)
    {
        noise_buffer_vx_.resize(total_samples);
        noise_buffer_vy_.resize(total_samples);
        noise_buffer_omega_.resize(total_samples);
    }

    if (params_.use_time_correlated_noise)
    {
        const float alpha = params_.noise_alpha;
        const float innovation_scale = std::sqrt(std::max(0.0f, 1.0f - alpha * alpha));

        for (int k = 0; k < params_.K; ++k)
        {
            float eps_vx = noise_vx_(rng_) * adaptive_sigma_vx_;
            float eps_vy = noise_vy_(rng_) * adaptive_sigma_vy_;
            float eps_omega = noise_omega_(rng_) * adaptive_sigma_omega_;

            const int noise_base = k * params_.T;
            for (int t = 0; t < params_.T; ++t)
            {
                noise_buffer_vx_[noise_base + t] = eps_vx;
                noise_buffer_vy_[noise_base + t] = eps_vy;
                noise_buffer_omega_[noise_base + t] = eps_omega;

                eps_vx = alpha * eps_vx + innovation_scale * adaptive_sigma_vx_ * noise_vx_(rng_);
                eps_vy = alpha * eps_vy + innovation_scale * adaptive_sigma_vy_ * noise_vy_(rng_);
                eps_omega = alpha * eps_omega + innovation_scale * adaptive_sigma_omega_ * noise_omega_(rng_);
            }
        }
    }
    else
    {
        for (int i = 0; i < total_samples; ++i)
        {
            noise_buffer_vx_[i] = noise_vx_(rng_) * adaptive_sigma_vx_;
            noise_buffer_vy_[i] = noise_vy_(rng_) * adaptive_sigma_vy_;
            noise_buffer_omega_[i] = noise_omega_(rng_) * adaptive_sigma_omega_;
        }
    }

    // Pre-compute for LiDAR hard collision check
    const float robot_radius_sq = params_.robot_radius * params_.robot_radius;

    // Subsample obstacles for faster collision checking
    std::vector<Eigen::Vector2f> subsampled_obstacles;
    const size_t max_obstacles = 200;
    if (obstacles.size() > max_obstacles)
    {
        subsampled_obstacles.reserve(max_obstacles);
        const size_t step = obstacles.size() / max_obstacles;
        for (size_t i = 0; i < obstacles.size(); i += step)
            subsampled_obstacles.push_back(obstacles[i]);
    }
    const auto& obs_to_use = (obstacles.size() > max_obstacles) ? subsampled_obstacles : obstacles;

    // Storage for trajectories and costs
    std::vector<std::vector<ControlCommand>> all_controls(params_.K);
    std::vector<std::vector<State>> all_trajectories(params_.K);
    std::vector<float> costs(params_.K);

    // Precompute sin/cos for initial state
    const float cos_theta_init = std::cos(current_state.theta);
    const float sin_theta_init = std::sin(current_state.theta);

    // Cost breakdown for debug
    struct CostBreakdown {
        float path_cost = 0.f;
        float obstacle_cost = 0.f;
        float smoothness_cost = 0.f;
        float speed_cost = 0.f;
        float goal_cost = 0.f;
        float total() const { return path_cost + obstacle_cost + smoothness_cost + speed_cost + goal_cost; }
    };
    std::vector<CostBreakdown> cost_breakdowns(params_.K);

    // ========================================================================
    // SAMPLE K TRAJECTORIES WITH ESDF-BASED OBSTACLE COSTS
    // ========================================================================
    for (int k = 0; k < params_.K; ++k)
    {
        std::vector<ControlCommand> controls(params_.T);
        std::vector<State> trajectory;
        trajectory.reserve(params_.T + 1);
        trajectory.push_back(current_state);

        State state = current_state;
        float cos_theta = cos_theta_init;
        float sin_theta = sin_theta_init;

        bool collision = false;
        float traj_cost = 0.0f;
        CostBreakdown& breakdown = cost_breakdowns[k];

        const int noise_base = k * params_.T;
        for (int t = 0; t < params_.T && !collision; ++t)
        {
            // Add noise to controls
            ControlCommand ctrl;
            ctrl.vx = std::clamp(prev_controls_[t].vx + noise_buffer_vx_[noise_base + t],
                                 -params_.max_vx, params_.max_vx);
            ctrl.vy = std::clamp(prev_controls_[t].vy + noise_buffer_vy_[noise_base + t],
                                 -params_.max_vy, params_.max_vy);
            ctrl.omega = std::clamp(prev_controls_[t].omega + noise_buffer_omega_[noise_base + t],
                                    -params_.max_omega, params_.max_omega);
            controls[t] = ctrl;

            // Simulate step
            float vx_world = ctrl.vx * cos_theta - ctrl.vy * sin_theta;
            float vy_world = ctrl.vx * sin_theta + ctrl.vy * cos_theta;

            state.x += vx_world * params_.dt;
            state.y += vy_world * params_.dt;
            state.theta -= ctrl.omega * params_.dt;

            if (state.theta > M_PI) state.theta -= 2.0f * M_PI;
            else if (state.theta < -M_PI) state.theta += 2.0f * M_PI;

            cos_theta = std::cos(state.theta);
            sin_theta = std::sin(state.theta);

            trajectory.push_back(state);

            // ==============================================================
            // HARD COLLISION CHECK (LiDAR safety layer)
            // Also compute minimum LiDAR distance for cost fusion
            // ==============================================================
            float min_lidar_dist_sq = std::numeric_limits<float>::max();
            for (const auto& obs : obs_to_use)
            {
                float dx = state.x - obs.x();
                float dy = state.y - obs.y();
                float dist_sq = dx * dx + dy * dy;

                // Hard collision check
                if (dist_sq < robot_radius_sq)
                {
                    collision = true;
                    break;
                }

                // Track minimum distance for cost fusion
                if (dist_sq < min_lidar_dist_sq)
                    min_lidar_dist_sq = dist_sq;
            }

            if (collision)
                break;

            // ==============================================================
            // FUSED OBSTACLE COST: d_fused = min(d_esdf, d_lidar)
            // - ESDF: smooth field from static map
            // - LiDAR: detects dynamic/unmapped obstacles
            // ==============================================================
            float d_esdf = std::numeric_limits<float>::max();
            GridESDF::ESDFQuery esdf_query = esdf->query_esdf(state.x, state.y);
            if (esdf_query.valid)
                d_esdf = esdf_query.distance;

            float d_lidar = std::sqrt(min_lidar_dist_sq);

            // Fused distance: conservative (minimum of both)
            const float d_fused = std::min(d_esdf, d_lidar);

            // Quadratic hinge cost: c = w * max(0, margin - d)^2
            if (d_fused < params_.safety_margin)
            {
                float pen = params_.safety_margin - d_fused;
                // Normalize penetration to get reasonable cost values
                float pen_normalized = pen / params_.safety_margin;
                float obs_c = params_.w_obstacle * pen_normalized * pen_normalized * 100.0f;
                traj_cost += obs_c;
                breakdown.obstacle_cost += obs_c;
            }

            // Extra penalty when very close to collision
            if (d_fused < params_.robot_radius + params_.collision_buffer)
            {
                float critical_pen = (params_.robot_radius + params_.collision_buffer) - d_fused;
                float critical_c = params_.w_obstacle * 10.0f * std::exp(critical_pen / 50.0f);
                traj_cost += critical_c;
                breakdown.obstacle_cost += critical_c;
            }

            // ==============================================================
            // PATH FOLLOWING COST (linear distance)
            // ==============================================================
            float path_min_dist_sq = std::numeric_limits<float>::max();
            for (const auto& wp : path)
            {
                float dx = state.x - wp.x();
                float dy = state.y - wp.y();
                float d_sq = dx * dx + dy * dy;
                if (d_sq < path_min_dist_sq) path_min_dist_sq = d_sq;
            }
            float path_c = params_.w_path * std::sqrt(path_min_dist_sq);
            traj_cost += path_c;
            breakdown.path_cost += path_c;

            // ==============================================================
            // SMOOTHNESS COST
            // ==============================================================
            if (t > 0)
            {
                float dvx = (ctrl.vx - controls[t-1].vx) / params_.max_vx;
                float dvy = (ctrl.vy - controls[t-1].vy) / params_.max_vy;
                float domega = (ctrl.omega - controls[t-1].omega) / params_.max_omega;
                float smooth_c = params_.w_smoothness * (dvx*dvx + dvy*dvy + domega*domega);
                traj_cost += smooth_c;
                breakdown.smoothness_cost += smooth_c;
            }

            // ==============================================================
            // SPEED COST
            // ==============================================================
            float speed_sq = ctrl.vx * ctrl.vx + ctrl.vy * ctrl.vy;
            float desired_speed = params_.max_vy * 0.5f;
            float speed_diff = std::sqrt(speed_sq) - desired_speed;
            float speed_c = params_.w_speed * speed_diff * speed_diff;
            traj_cost += speed_c;
            breakdown.speed_cost += speed_c;
        }

        // Goal cost for final state
        if (!collision)
        {
            const Eigen::Vector2f& goal = path.back();
            float dx = state.x - goal.x();
            float dy = state.y - goal.y();
            float goal_c = params_.w_goal * std::sqrt(dx * dx + dy * dy);
            traj_cost += goal_c;
            breakdown.goal_cost += goal_c;
        }

        all_controls[k] = std::move(controls);
        all_trajectories[k] = std::move(trajectory);
        costs[k] = collision ? std::numeric_limits<float>::infinity() : (traj_cost / params_.cost_scale);
    }

    // ========================================================================
    // WEIGHT COMPUTATION (same as original)
    // ========================================================================
    float min_valid_cost = std::numeric_limits<float>::max();
    int num_invalid = 0;
    for (int k = 0; k < params_.K; ++k)
    {
        if (std::isinf(costs[k]))
            num_invalid++;
        else if (costs[k] < min_valid_cost)
            min_valid_cost = costs[k];
    }

    if (std::isinf(min_valid_cost))
        return ControlCommand{0.0f, 0.0f, 0.0f};

    const float valid_ratio = static_cast<float>(params_.K - num_invalid) / static_cast<float>(params_.K);

    constexpr float invalid_cost_multiplier = 30.0f;
    const float invalid_cost = min_valid_cost + invalid_cost_multiplier * adaptive_lambda_;

    for (int k = 0; k < params_.K; ++k)
    {
        if (std::isinf(costs[k]))
            costs[k] = invalid_cost;
    }

    float min_cost = min_valid_cost;
    const float inv_lambda = -1.0f / adaptive_lambda_;
    float weight_sum = 0.0f;
    std::vector<float> weights(params_.K);

    float max_cost = min_cost;
    int valid_count = params_.K - num_invalid;

    for (int k = 0; k < params_.K; ++k)
    {
        weights[k] = std::exp(inv_lambda * (costs[k] - min_cost));
        weight_sum += weights[k];
        if (costs[k] > max_cost) max_cost = costs[k];
    }

    if (weight_sum < 1e-10f)
        return ControlCommand{0.0f, 0.0f, 0.0f};

    const float inv_weight_sum = 1.0f / weight_sum;

    // ESS computation
    float sum_w_squared = 0.0f;
    for (int k = 0; k < params_.K; ++k)
    {
        float w_norm = weights[k] * inv_weight_sum;
        sum_w_squared += w_norm * w_norm;
    }
    const float ess = (sum_w_squared > 1e-10f) ? (1.0f / sum_w_squared) : static_cast<float>(valid_count);
    const float ess_ratio = ess / static_cast<float>(std::max(1, valid_count));

    // Adaptive lambda
    if (params_.use_adaptive_covariance)
    {
        if (ess_ratio < 0.05f && adaptive_lambda_ < 20.0f * params_.lambda)
            adaptive_lambda_ *= 1.2f;
        else if (ess_ratio > 0.2f && adaptive_lambda_ > params_.lambda)
            adaptive_lambda_ *= 0.95f;
    }

    // Adaptive sigma
    if (params_.use_adaptive_covariance)
    {
        const bool allow_adapt = (valid_ratio > 0.80f && ess_ratio > 0.10f);

        if (allow_adapt)
        {
            if (valid_ratio > 0.95f && ess_ratio > 0.30f)
            {
                adaptive_sigma_vx_ *= 1.005f;
                adaptive_sigma_vy_ *= 1.005f;
                adaptive_sigma_omega_ *= 1.005f;
            }
        }

        if (valid_ratio < 0.70f)
        {
            adaptive_sigma_vx_ *= 0.70f;
            adaptive_sigma_vy_ *= 0.70f;
            adaptive_sigma_omega_ *= 0.70f;
        }
        else if (valid_ratio < 0.80f)
        {
            adaptive_sigma_vx_ *= 0.90f;
            adaptive_sigma_vy_ *= 0.90f;
            adaptive_sigma_omega_ *= 0.90f;
        }

        adaptive_sigma_vx_ = std::clamp(adaptive_sigma_vx_, params_.sigma_min_vx, params_.sigma_max_vx);
        adaptive_sigma_vy_ = std::clamp(adaptive_sigma_vy_, params_.sigma_min_vy, params_.sigma_max_vy);
        adaptive_sigma_omega_ = std::clamp(adaptive_sigma_omega_, params_.sigma_min_omega, params_.sigma_max_omega);
    }

    // ========================================================================
    // COMPUTE OPTIMAL CONTROLS
    // ========================================================================
    std::vector<ControlCommand> optimal_controls(params_.T, ControlCommand{0.0f, 0.0f, 0.0f});

    for (int k = 0; k < params_.K; ++k)
    {
        const float w = weights[k] * inv_weight_sum;
        if (w > 1e-10f)
        {
            for (int t = 0; t < params_.T; ++t)
            {
                optimal_controls[t].vx += w * all_controls[k][t].vx;
                optimal_controls[t].vy += w * all_controls[k][t].vy;
                optimal_controls[t].omega += w * all_controls[k][t].omega;
            }
        }
    }

    // NaN check
    for (int t = 0; t < params_.T; ++t)
    {
        if (std::isnan(optimal_controls[t].vx)) optimal_controls[t].vx = 0.0f;
        if (std::isnan(optimal_controls[t].vy)) optimal_controls[t].vy = 0.0f;
        if (std::isnan(optimal_controls[t].omega)) optimal_controls[t].omega = 0.0f;
    }

    // Adaptive covariance update (gated)
    const bool allow_weighted_sigma_update = (valid_ratio > 0.80f && ess_ratio > 0.10f);
    if (params_.use_adaptive_covariance && allow_weighted_sigma_update)
    {
        float weighted_var_vx = 0.0f;
        float weighted_var_vy = 0.0f;
        float weighted_var_omega = 0.0f;
        float total_weight = 0.0f;

        for (int k = 0; k < params_.K; ++k)
        {
            const float w = weights[k] * inv_weight_sum;
            if (w > 1e-10f)
            {
                const int noise_base = k * params_.T;
                for (int t = 0; t < params_.T; ++t)
                {
                    float eps_vx = noise_buffer_vx_[noise_base + t];
                    float eps_vy = noise_buffer_vy_[noise_base + t];
                    float eps_omega = noise_buffer_omega_[noise_base + t];

                    weighted_var_vx += w * eps_vx * eps_vx;
                    weighted_var_vy += w * eps_vy * eps_vy;
                    weighted_var_omega += w * eps_omega * eps_omega;
                }
                total_weight += w * params_.T;
            }
        }

        if (total_weight > 1e-10f)
        {
            weighted_var_vx /= static_cast<float>(params_.T);
            weighted_var_vy /= static_cast<float>(params_.T);
            weighted_var_omega /= static_cast<float>(params_.T);

            const float beta = params_.cov_adaptation_rate;
            const float one_minus_beta = 1.0f - beta;

            float new_sigma_vx = std::sqrt(one_minus_beta * adaptive_sigma_vx_ * adaptive_sigma_vx_ + beta * weighted_var_vx);
            float new_sigma_vy = std::sqrt(one_minus_beta * adaptive_sigma_vy_ * adaptive_sigma_vy_ + beta * weighted_var_vy);
            float new_sigma_omega = std::sqrt(one_minus_beta * adaptive_sigma_omega_ * adaptive_sigma_omega_ + beta * weighted_var_omega);

            adaptive_sigma_vx_ = std::clamp(new_sigma_vx, params_.sigma_min_vx, params_.sigma_max_vx);
            adaptive_sigma_vy_ = std::clamp(new_sigma_vy, params_.sigma_min_vy, params_.sigma_max_vy);
            adaptive_sigma_omega_ = std::clamp(new_sigma_omega, params_.sigma_min_omega, params_.sigma_max_omega);
        }
    }

    // Store for warm start
    prev_controls_ = optimal_controls;

    // Compute optimal trajectory for visualization
    optimal_trajectory_.clear();
    optimal_trajectory_.reserve(params_.T + 1);
    State state = current_state;
    optimal_trajectory_.push_back(state);
    for (int t = 0; t < params_.T; ++t)
    {
        state = simulateStep(state, optimal_controls[t]);
        optimal_trajectory_.push_back(state);
    }

    // Store trajectories for visualization
    best_trajectories_.clear();
    if (params_.num_trajectories_to_draw > 0)
    {
        std::vector<int> valid_indices;
        valid_indices.reserve(params_.K);
        for (int k = 0; k < params_.K; ++k)
        {
            if (!std::isinf(costs[k]) && !all_trajectories[k].empty())
                valid_indices.push_back(k);
        }

        std::shuffle(valid_indices.begin(), valid_indices.end(), rng_);
        const int n_to_draw = std::min(params_.num_trajectories_to_draw, static_cast<int>(valid_indices.size()));

        best_trajectories_.reserve(n_to_draw);
        for (int i = 0; i < n_to_draw; ++i)
            best_trajectories_.push_back(std::move(all_trajectories[valid_indices[i]]));
    }

    return optimal_controls[0];
}

bool MPPIController::goalReached(const State& current_state, const Eigen::Vector2f& goal) const
{
    float dx = current_state.x - goal.x();
    float dy = current_state.y - goal.y();
    return std::sqrt(dx * dx + dy * dy) < params_.goal_tolerance;
}

void MPPIController::reset()
{
    for (auto& ctrl : prev_controls_)
    {
        ctrl = ControlCommand{0.0f, 0.0f, 0.0f};
    }
    optimal_trajectory_.clear();

    // Reset adaptive covariance to initial values
    adaptive_sigma_vx_ = params_.sigma_vx;
    adaptive_sigma_vy_ = params_.sigma_vy;
    adaptive_sigma_omega_ = params_.sigma_omega;

    // Reset adaptive lambda
    adaptive_lambda_ = params_.lambda;
}

void MPPIController::setParams(const Params& params)
{
    params_ = params;
    // Use standard normal distributions (we scale by adaptive sigma)
    noise_vx_ = std::normal_distribution<float>(0.0f, 1.0f);
    noise_vy_ = std::normal_distribution<float>(0.0f, 1.0f);
    noise_omega_ = std::normal_distribution<float>(0.0f, 1.0f);

    // Initialize adaptive sigmas from params
    adaptive_sigma_vx_ = params.sigma_vx;
    adaptive_sigma_vy_ = params.sigma_vy;
    adaptive_sigma_omega_ = params.sigma_omega;

    // Initialize adaptive lambda
    adaptive_lambda_ = params.lambda;

    // Resize control sequence if horizon changed
    if (static_cast<int>(prev_controls_.size()) != params_.T)
    {
        prev_controls_.resize(params_.T);
        for (auto& ctrl : prev_controls_)
        {
            ctrl = ControlCommand{0.0f, 0.0f, 0.0f};
        }
    }
}

// ============================================================================
// Private Methods
// ============================================================================

MPPIController::State MPPIController::simulateStep(const State& state, const ControlCommand& control) const
{
    // =======================================================================
    // COORDINATE SYSTEM CONVENTION:
    // - Robot frame: X+ = right (vx), Y+ = forward (vy)
    // - World frame: standard X-Y
    // - theta = angle of robot's X-axis from world X-axis
    // - When theta=0: robot's X+ points to world X+, robot's Y+ points to world Y+
    // =======================================================================

    float cos_theta = std::cos(state.theta);
    float sin_theta = std::sin(state.theta);

    // Robot's X-axis (right, vx) in world: (cos(theta), sin(theta))
    // Robot's Y-axis (forward, vy) in world: (-sin(theta), cos(theta))
    //
    // World velocity = vx * robot_X_in_world + vy * robot_Y_in_world
    float vx_world = control.vx * cos_theta + control.vy * (-sin_theta);
    float vy_world = control.vx * sin_theta + control.vy * cos_theta;

    State next;
    next.x = state.x + vx_world * params_.dt;
    next.y = state.y + vy_world * params_.dt;
    next.theta = state.theta - control.omega * params_.dt;  // INVERTED: negative omega

    // Normalize theta to [-pi, pi]
    while (next.theta > M_PI) next.theta -= 2.0f * static_cast<float>(M_PI);
    while (next.theta < -M_PI) next.theta += 2.0f * static_cast<float>(M_PI);

    return next;
}

std::pair<float, size_t> MPPIController::distanceToPath(
    const State& state,
    const std::vector<Eigen::Vector2f>& path) const
{
    float min_dist = std::numeric_limits<float>::max();
    size_t closest_idx = 0;

    Eigen::Vector2f pos(state.x, state.y);

    for (size_t i = 0; i < path.size(); ++i)
    {
        float dist = (pos - path[i]).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_idx = i;
        }
    }

    return {min_dist, closest_idx};
}


MPPIController::ControlCommand MPPIController::computeNominalControl(
    const State& current_state,
    const std::vector<Eigen::Vector2f>& path) const
{
    ControlCommand nominal{0.0f, 0.0f, 0.0f};

    if (path.empty()) return nominal;

    // Find the closest point on path and look ahead
    auto [dist, closest_idx] = distanceToPath(current_state, path);

    // Find lookahead point
    size_t target_idx = closest_idx;
    float accumulated_dist = 0.0f;

    for (size_t i = closest_idx; i < path.size() - 1; ++i)
    {
        accumulated_dist += (path[i + 1] - path[i]).norm();
        if (accumulated_dist >= params_.lookahead_distance)
        {
            target_idx = i + 1;
            break;
        }
        target_idx = i + 1;
    }

    // Target point in world frame
    const Eigen::Vector2f& target = path[target_idx];

    // Vector from robot to target in world frame
    float dx_world = target.x() - current_state.x;
    float dy_world = target.y() - current_state.y;
    float dist_to_target = std::sqrt(dx_world * dx_world + dy_world * dy_world);

    if (dist_to_target < 1.0f) return nominal;

    // =======================================================================
    // SIMPLE APPROACH: Transform target to robot frame and drive towards it
    // Robot frame: X+ = right, Y+ = forward
    // World to robot transformation (rotate by -theta):
    // [dx_robot]   [cos(theta)  sin(theta)] [dx_world]
    // [dy_robot] = [-sin(theta) cos(theta)] [dy_world]
    // =======================================================================

    float cos_theta = std::cos(current_state.theta);
    float sin_theta = std::sin(current_state.theta);

    // Transform target delta from world frame to robot frame
    float dx_robot = dx_world * cos_theta + dy_world * sin_theta;    // lateral (+ = right)
    float dy_robot = -dx_world * sin_theta + dy_world * cos_theta;   // forward (+ = front)

    // Angle to target in robot frame (0 = straight ahead, positive = right)
    float angle_to_target_robot = std::atan2(dx_robot, dy_robot);  // atan2(x,y) because Y+ is forward

    // =======================================================================
    // CONTROL STRATEGY:
    // - Rotate to face target (angle_to_target_robot -> 0)
    // - Move forward when aligned
    // =======================================================================

    const float desired_speed = params_.max_vy * 0.6f;

    // Rotation: proportional to angle error in robot frame
    nominal.omega = std::clamp(angle_to_target_robot * 1.0f, -params_.max_omega, params_.max_omega);

    // Forward speed: full when aligned, reduced when turning
    // cos(angle) gives 1 when aligned, 0 when perpendicular, -1 when backwards
    float alignment = std::cos(angle_to_target_robot);

    if (alignment > 0.3f)
    {
        // Target is in front: move forward
        nominal.vy = desired_speed * alignment;
        nominal.vx = 0.0f;
    }
    else if (alignment < -0.3f)
    {
        // Target is behind: rotate in place (no forward motion)
        nominal.vy = 0.0f;
        nominal.vx = 0.0f;
        // Increase rotation when target is behind
        nominal.omega = std::clamp(angle_to_target_robot * 1.5f, -params_.max_omega, params_.max_omega);
    }
    else
    {
        // Target is to the side: slow forward + rotate
        nominal.vy = desired_speed * 0.3f;
        nominal.vx = 0.0f;
    }

    return nominal;
}


void MPPIController::warmStart()
{
    // Shift control sequence: discard first, duplicate last
    if (prev_controls_.size() > 1)
    {
        for (size_t t = 0; t < prev_controls_.size() - 1; ++t)
        {
            prev_controls_[t] = prev_controls_[t + 1];
        }
        // Last control stays the same (or could be set to zero)
    }
}
