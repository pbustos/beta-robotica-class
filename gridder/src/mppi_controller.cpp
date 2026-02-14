/*
 *    Copyright (C) 2026 by RoboComp Team
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
    // Redirect to full version without ESDF or covariance
    std::vector<float> empty_cov;
    return compute(current_state, path, obstacles, nullptr, empty_cov);
}

// ============================================================================
// ESDF-based compute (redirects to full version)
// ============================================================================

MPPIController::ControlCommand MPPIController::compute(
    const State& current_state,
    const std::vector<Eigen::Vector2f>& path,
    const std::vector<Eigen::Vector2f>& obstacles,
    const GridESDF* esdf)
{
    // Redirect to full version without covariance
    std::vector<float> empty_cov;
    return compute(current_state, path, obstacles, esdf, empty_cov);
}

// ============================================================================
// ESDF-based compute with Covariance-Aware Margin Inflation (Option A)
// This is the MAIN implementation - all other compute() overloads redirect here
// ============================================================================

MPPIController::ControlCommand MPPIController::compute(
    const State& current_state,
    const std::vector<Eigen::Vector2f>& path,
    const std::vector<Eigen::Vector2f>& obstacles,
    const GridESDF* esdf,
    const std::vector<float>& pose_cov)
{
    // Fallback to non-covariance version if covariance not provided or disabled
    if (!params_.use_covariance_inflation || pose_cov.size() < 6 || esdf == nullptr)
    {
        return compute(current_state, path, obstacles, esdf);
    }

    // Extract pose covariance matrix elements: [σ²_xx, σ_xy, σ_xθ, σ²_yy, σ_yθ, σ²_θθ]
    // Build 3x3 covariance matrix
    Eigen::Matrix3f Sigma_pose;
    Sigma_pose << pose_cov[0], pose_cov[1], pose_cov[2],
                  pose_cov[1], pose_cov[3], pose_cov[4],
                  pose_cov[2], pose_cov[4], pose_cov[5];

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

    // Generate noise (same as base version)
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
            // HARD COLLISION CHECK & DISTANCE COMPUTATION
            // Uses 8-point footprint sampling for precise obstacle distances
            // ==============================================================
            float min_lidar_dist_sq = std::numeric_limits<float>::max();
            float min_esdf_dist = std::numeric_limits<float>::max();

            if (params_.use_footprint_sampling)
            {
                // Get 8 footprint points in world frame
                auto footprint_points = getFootprintPoints(state);

                // Check each footprint point against LiDAR obstacles
                for (const auto& fp : footprint_points)
                {
                    for (const auto& obs : obs_to_use)
                    {
                        float dx = fp.x() - obs.x();
                        float dy = fp.y() - obs.y();
                        float dist_sq = dx * dx + dy * dy;

                        // Hard collision: any footprint point inside obstacle
                        if (dist_sq < 100.0f * 100.0f)  // 100mm threshold for LiDAR points
                        {
                            collision = true;
                            break;
                        }

                        if (dist_sq < min_lidar_dist_sq)
                            min_lidar_dist_sq = dist_sq;
                    }
                    if (collision) break;
                }

                // Query ESDF for each footprint point (find minimum)
                if (!collision && esdf != nullptr)
                {
                    for (const auto& fp : footprint_points)
                    {
                        auto esdf_query = esdf->query_esdf(fp.x(), fp.y());
                        if (esdf_query.valid && esdf_query.distance < min_esdf_dist)
                            min_esdf_dist = esdf_query.distance;
                    }
                }
            }
            else
            {
                // Fallback: single point (center) check
                for (const auto& obs : obs_to_use)
                {
                    float dx = state.x - obs.x();
                    float dy = state.y - obs.y();
                    float dist_sq = dx * dx + dy * dy;

                    if (dist_sq < robot_radius_sq)
                    {
                        collision = true;
                        break;
                    }

                    if (dist_sq < min_lidar_dist_sq)
                        min_lidar_dist_sq = dist_sq;
                }

                // Single ESDF query at center
                if (!collision && esdf != nullptr)
                {
                    auto esdf_query = esdf->query_esdf(state.x, state.y);
                    if (esdf_query.valid)
                        min_esdf_dist = esdf_query.distance;
                }
            }

            if (collision)
                break;

            // ==============================================================
            // FUSED OBSTACLE COST: d_fused = min(d_esdf, d_lidar)
            // Now using minimum distance from all footprint points
            // ==============================================================
            float d_lidar = std::sqrt(min_lidar_dist_sq);
            float d_fused = std::min(min_esdf_dist, d_lidar);

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

            // Extra penalty when very close to collision (any footprint point)
            // Note: with footprint sampling, collision_buffer applies to the footprint edge
            if (d_fused < params_.collision_buffer)
            {
                float critical_pen = params_.collision_buffer - d_fused;
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

    // ========================================================================
    // OUTPUT SMOOTHING (EMA filter to reduce jitter)
    // smoothed = alpha * previous + (1 - alpha) * new
    // ========================================================================
    ControlCommand raw_output = optimal_controls[0];
    ControlCommand smoothed_output;

    if (first_output_ || params_.output_smoothing_alpha <= 0.0f)
    {
        // No smoothing on first output or if disabled
        smoothed_output = raw_output;
        first_output_ = false;
    }
    else
    {
        const float alpha = params_.output_smoothing_alpha;
        const float one_minus_alpha = 1.0f - alpha;

        smoothed_output.vx = alpha * last_smoothed_output_.vx + one_minus_alpha * raw_output.vx;
        smoothed_output.vy = alpha * last_smoothed_output_.vy + one_minus_alpha * raw_output.vy;
        smoothed_output.omega = alpha * last_smoothed_output_.omega + one_minus_alpha * raw_output.omega;
    }

    last_smoothed_output_ = smoothed_output;

    return smoothed_output;
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

    // Reset output smoothing filter
    last_smoothed_output_ = ControlCommand{0.0f, 0.0f, 0.0f};
    first_output_ = true;
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
    const Eigen::Vector2f delta_world(target.x() - current_state.x, target.y() - current_state.y);
    const float dist_to_target = delta_world.norm();

    if (dist_to_target < 1.0f) return nominal;

    // =======================================================================
    // Transform target delta from world frame to robot frame using Eigen::Affine2f
    // Robot frame: X+ = right, Y+ = forward
    // World to robot = inverse of robot-to-world = Rotation(-theta)
    // =======================================================================
    const Eigen::Rotation2Df R_world_to_robot(-current_state.theta);
    const Eigen::Vector2f delta_robot = R_world_to_robot * delta_world;

    const float dx_robot = delta_robot.x();  // lateral (+ = right)
    const float dy_robot = delta_robot.y();  // forward (+ = front)

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

std::array<Eigen::Vector2f, 8> MPPIController::getFootprintPoints(const State& state) const
{
    // Robot footprint points in robot frame (X+ = right, Y+ = forward)
    // 8 points: 4 corners + 4 edge midpoints
    //
    //     P0 -------- P1 -------- P2
    //     |                        |
    //     P7        CENTER        P3
    //     |                        |
    //     P6 -------- P5 -------- P4

    const float sw = params_.robot_semi_width;   // Half width (X)
    const float sl = params_.robot_semi_length;  // Half length (Y)

    // Footprint points in robot frame (local coordinates)
    const std::array<Eigen::Vector2f, 8> local_points = {{
        {-sw,  sl},   // P0: front-left corner
        { 0.f, sl},   // P1: front-center
        { sw,  sl},   // P2: front-right corner
        { sw,  0.f},  // P3: right-center
        { sw, -sl},   // P4: rear-right corner
        { 0.f,-sl},   // P5: rear-center
        {-sw, -sl},   // P6: rear-left corner
        {-sw,  0.f}   // P7: left-center
    }};

    // Build robot-to-world transform: T = Translation(x,y) * Rotation(theta)
    Eigen::Affine2f T_robot_to_world = Eigen::Translation2f(state.x, state.y)
                                     * Eigen::Rotation2Df(state.theta);

    // Transform all points to world frame
    std::array<Eigen::Vector2f, 8> world_points;
    for (size_t i = 0; i < 8; ++i)
        world_points[i] = T_robot_to_world * local_points[i];

    return world_points;
}

