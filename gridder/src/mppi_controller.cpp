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
#include <iostream>
#include <execution>  // For parallel execution policies
#include <QDebug>     // For debug output
#include <algorithm>  // For std::shuffle

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

    // Pre-allocate AR(1) state buffers (one per trajectory)
    ar1_state_vx_.resize(params_.K, 0.0f);
    ar1_state_vy_.resize(params_.K, 0.0f);
    ar1_state_omega_.resize(params_.K, 0.0f);
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
    const float safety_margin_sq = params_.safety_margin * params_.safety_margin;

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

    // =========================================================================
    // OPTIMIZATION 4: Cache path distances for faster lookup
    // =========================================================================
    std::vector<float> path_cumulative_dist(path.size(), 0.0f);
    for (size_t i = 1; i < path.size(); ++i)
    {
        path_cumulative_dist[i] = path_cumulative_dist[i-1] + (path[i] - path[i-1]).norm();
    }

    // =========================================================================
    // OPTIMIZATION 5: Precompute sin/cos for prev_controls theta updates
    // =========================================================================
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
            // OPTIMIZATION 7: Early termination on collision (inline obstacle check)
            // =========================================================================
            float min_dist_sq = std::numeric_limits<float>::max();
            for (const auto& obs : obs_to_use)
            {
                float dx = state.x - obs.x();
                float dy = state.y - obs.y();
                float dist_sq = dx * dx + dy * dy;
                if (dist_sq < min_dist_sq) min_dist_sq = dist_sq;

                // Early exit if collision detected
                if (dist_sq < robot_radius_sq)
                {
                    collision = true;
                    break;
                }
            }

            if (!collision)
            {
                // Obstacle cost (only if within safety margin)
                // Use very aggressive exponential penalty
                if (min_dist_sq < safety_margin_sq)
                {
                    float min_dist = std::sqrt(min_dist_sq);
                    float penetration = params_.safety_margin - min_dist;
                    // Quadratic + exponential for very aggressive avoidance
                    float obs_c = params_.w_obstacle * (penetration * penetration / 1000.0f + std::exp(penetration / params_.obstacle_decay));
                    traj_cost += obs_c;
                    breakdown.obstacle_cost += obs_c;
                }

                // =========================================================================
                // Path following cost - use LINEAR distance, not squared
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
        costs[k] = collision ? std::numeric_limits<float>::infinity() : (traj_cost / params_.cost_scale);
    }

    // Find minimum cost for numerical stability
    float min_cost = std::numeric_limits<float>::max();
    for (int k = 0; k < params_.K; ++k)
    {
        if (costs[k] < min_cost) min_cost = costs[k];
    }

    // If all costs are infinite, return zero control
    if (std::isinf(min_cost))
    {
        return ControlCommand{0.0f, 0.0f, 0.0f};
    }

    // =========================================================================
    // OPTIMIZATION 9: Fused weight computation and normalization
    // =========================================================================
    const float inv_lambda = -1.0f / adaptive_lambda_;
    float weight_sum = 0.0f;
    std::vector<float> weights(params_.K);

    // Track statistics for debug
    float max_cost = min_cost;
    int valid_count = 0;

    for (int k = 0; k < params_.K; ++k)
    {
        if (std::isinf(costs[k]))
        {
            weights[k] = 0.0f;
        }
        else
        {
            weights[k] = std::exp(inv_lambda * (costs[k] - min_cost));
            weight_sum += weights[k];
            if (costs[k] > max_cost) max_cost = costs[k];
            valid_count++;
        }
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

    // Debug: show cost, weight, and ESS statistics
    static int stats_counter = 0;
    if (++stats_counter % 100 == 0)
    {
        // Find min/max normalized weights and best trajectory index
        float min_weight = std::numeric_limits<float>::max();
        float max_weight = 0.0f;
        int best_idx = -1;
        int median_idx = -1;

        // Find best (lowest cost) and a typical (median cost) trajectory
        std::vector<std::pair<float, int>> valid_costs;
        for (int k = 0; k < params_.K; ++k)
        {
            float w = weights[k] * inv_weight_sum;
            if (w > 1e-10f)
            {
                if (w < min_weight) min_weight = w;
                if (w > max_weight) max_weight = w;
            }
            if (!std::isinf(costs[k]))
            {
                valid_costs.push_back({costs[k], k});
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
                 << "| Valid:" << valid_count << "/" << params_.K;

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
    // =========================================================================
    if (params_.use_adaptive_covariance)
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

    // Reset AR(1) states
    std::fill(ar1_state_vx_.begin(), ar1_state_vx_.end(), 0.0f);
    std::fill(ar1_state_vy_.begin(), ar1_state_vy_.end(), 0.0f);
    std::fill(ar1_state_omega_.begin(), ar1_state_omega_.end(), 0.0f);
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

    // Resize AR(1) state buffers if K changed
    if (static_cast<int>(ar1_state_vx_.size()) != params_.K)
    {
        ar1_state_vx_.resize(params_.K, 0.0f);
        ar1_state_vy_.resize(params_.K, 0.0f);
        ar1_state_omega_.resize(params_.K, 0.0f);
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

float MPPIController::computeTrajectoryCost(
    const std::vector<State>& trajectory,
    const std::vector<ControlCommand>& controls,
    const std::vector<Eigen::Vector2f>& path,
    const std::vector<Eigen::Vector2f>& obstacles) const
{
    float total_cost = 0.0f;

    for (size_t t = 0; t < trajectory.size(); ++t)
    {
        const State& state = trajectory[t];

        // Path following cost
        total_cost += params_.w_path * pathFollowingCost(state, path);

        // Obstacle avoidance cost
        float obs_cost = obstacleCost(state, obstacles);
        if (std::isinf(obs_cost))
        {
            return std::numeric_limits<float>::infinity();
        }
        total_cost += params_.w_obstacle * obs_cost;

        // Goal cost (only for last state, weighted more)
        if (t == trajectory.size() - 1)
        {
            total_cost += params_.w_goal * goalCost(state, path);
        }

        // Smoothness cost
        if (t < controls.size())
        {
            ControlCommand prev_ctrl = (t > 0) ? controls[t - 1] : ControlCommand{0.0f, 0.0f, 0.0f};
            total_cost += params_.w_smoothness * smoothnessCost(controls[t], prev_ctrl);

            // Speed cost - encourage maintaining some forward speed
            float speed = std::sqrt(controls[t].vx * controls[t].vx + controls[t].vy * controls[t].vy);
            float desired_speed = params_.max_vx * 0.5f; // 50% of max speed
            total_cost += params_.w_speed * std::pow(speed - desired_speed, 2);
        }
    }

    return total_cost;
}

float MPPIController::pathFollowingCost(const State& state,
                                        const std::vector<Eigen::Vector2f>& path) const
{
    auto [dist, _] = distanceToPath(state, path);
    return dist * dist; // Quadratic cost
}

float MPPIController::obstacleCost(const State& state,
                                   const std::vector<Eigen::Vector2f>& obstacles) const
{
    float min_dist = minDistanceToObstacles(state, obstacles);

    // Collision - infinite cost
    if (min_dist < params_.robot_radius)
    {
        return std::numeric_limits<float>::infinity();
    }

    // Within safety margin - exponential penalty
    if (min_dist < params_.safety_margin)
    {
        float penetration = params_.safety_margin - min_dist;
        return std::exp(penetration / params_.obstacle_decay);
    }

    // Safe - no cost
    return 0.0f;
}

float MPPIController::goalCost(const State& state,
                               const std::vector<Eigen::Vector2f>& path) const
{
    if (path.empty()) return 0.0f;

    const Eigen::Vector2f& goal = path.back();
    float dx = state.x - goal.x();
    float dy = state.y - goal.y();
    return std::sqrt(dx * dx + dy * dy);
}

float MPPIController::smoothnessCost(const ControlCommand& control,
                                     const ControlCommand& prev_control) const
{
    float dvx = control.vx - prev_control.vx;
    float dvy = control.vy - prev_control.vy;
    float domega = control.omega - prev_control.omega;

    // Normalize by max values
    dvx /= params_.max_vx;
    dvy /= params_.max_vy;
    domega /= params_.max_omega;

    return dvx * dvx + dvy * dvy + domega * domega;
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

float MPPIController::minDistanceToObstacles(const State& state,
                                              const std::vector<Eigen::Vector2f>& obstacles) const
{
    if (obstacles.empty())
    {
        return std::numeric_limits<float>::max();
    }

    float min_dist = std::numeric_limits<float>::max();
    Eigen::Vector2f pos(state.x, state.y);

    for (const auto& obs : obstacles)
    {
        float dist = (pos - obs).norm();
        min_dist = std::min(min_dist, dist);
    }

    return min_dist;
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

MPPIController::ControlCommand MPPIController::clampControl(const ControlCommand& control) const
{
    ControlCommand clamped;
    clamped.vx = std::clamp(control.vx, -params_.max_vx, params_.max_vx);
    clamped.vy = std::clamp(control.vy, -params_.max_vy, params_.max_vy);
    clamped.omega = std::clamp(control.omega, -params_.max_omega, params_.max_omega);
    return clamped;
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
