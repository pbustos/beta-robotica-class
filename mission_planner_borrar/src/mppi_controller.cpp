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
    , noise_vx_(0.0f, params.sigma_vx)
    , noise_vy_(0.0f, params.sigma_vy)
    , noise_omega_(0.0f, params.sigma_omega)
{
    // Initialize previous controls to zero
    prev_controls_.resize(params_.T);
    for (auto& ctrl : prev_controls_)
    {
        ctrl = ControlCommand{0.0f, 0.0f, 0.0f};
    }
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

    // Warm start: shift previous control sequence
    warmStart();

    // Storage for sampled trajectories and costs
    std::vector<std::vector<ControlCommand>> all_controls(params_.K);
    std::vector<float> costs(params_.K);

    // Sample K trajectories
    for (int k = 0; k < params_.K; ++k)
    {
        std::vector<ControlCommand> controls(params_.T);
        std::vector<State> trajectory;
        trajectory.reserve(params_.T + 1);
        trajectory.push_back(current_state);

        State state = current_state;

        // Generate control sequence with noise
        for (int t = 0; t < params_.T; ++t)
        {
            // Add noise to previous (warm-started) controls
            ControlCommand ctrl;
            ctrl.vx = prev_controls_[t].vx + noise_vx_(rng_);
            ctrl.vy = prev_controls_[t].vy + noise_vy_(rng_);
            ctrl.omega = prev_controls_[t].omega + noise_omega_(rng_);

            // Clamp to robot limits
            ctrl = clampControl(ctrl);
            controls[t] = ctrl;

            // Simulate forward
            state = simulateStep(state, ctrl);
            trajectory.push_back(state);
        }

        all_controls[k] = controls;
        costs[k] = computeTrajectoryCost(trajectory, controls, path, obstacles);
    }

    // Find minimum cost for numerical stability
    float min_cost = *std::min_element(costs.begin(), costs.end());

    // Compute weights using softmin: w_k = exp(-1/lambda * (cost_k - min_cost))
    std::vector<float> weights(params_.K);
    float weight_sum = 0.0f;

    for (int k = 0; k < params_.K; ++k)
    {
        weights[k] = std::exp(-1.0f / params_.lambda * (costs[k] - min_cost));
        weight_sum += weights[k];
    }

    // Normalize weights
    for (int k = 0; k < params_.K; ++k)
    {
        weights[k] /= weight_sum;
    }

    // Compute weighted average of control sequences
    std::vector<ControlCommand> optimal_controls(params_.T);
    for (int t = 0; t < params_.T; ++t)
    {
        optimal_controls[t] = ControlCommand{0.0f, 0.0f, 0.0f};
        for (int k = 0; k < params_.K; ++k)
        {
            optimal_controls[t].vx += weights[k] * all_controls[k][t].vx;
            optimal_controls[t].vy += weights[k] * all_controls[k][t].vy;
            optimal_controls[t].omega += weights[k] * all_controls[k][t].omega;
        }
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
}

void MPPIController::setParams(const Params& params)
{
    params_ = params;
    noise_vx_ = std::normal_distribution<float>(0.0f, params.sigma_vx);
    noise_vy_ = std::normal_distribution<float>(0.0f, params.sigma_vy);
    noise_omega_ = std::normal_distribution<float>(0.0f, params.sigma_omega);

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
    // Omnidirectional robot kinematic model
    // Velocities are in robot frame, need to transform to world frame
    float cos_theta = std::cos(state.theta);
    float sin_theta = std::sin(state.theta);

    State next;
    // Transform velocities from robot frame to world frame
    float vx_world = control.vx * cos_theta - control.vy * sin_theta;
    float vy_world = control.vx * sin_theta + control.vy * cos_theta;

    next.x = state.x + vx_world * params_.dt;
    next.y = state.y + vy_world * params_.dt;
    next.theta = state.theta + control.omega * params_.dt;

    // Normalize theta to [-pi, pi]
    while (next.theta > M_PI) next.theta -= 2.0f * M_PI;
    while (next.theta < -M_PI) next.theta += 2.0f * M_PI;

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
