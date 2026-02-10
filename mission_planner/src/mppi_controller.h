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

#ifndef MPPI_CONTROLLER_H
#define MPPI_CONTROLLER_H

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <limits>

/**
 * @brief MPPI (Model Predictive Path Integral) Controller for omnidirectional robot
 *
 * This controller generates optimal velocity commands (vx, vy, omega) by:
 * 1. Sampling K trajectories with random control perturbations
 * 2. Evaluating each trajectory with a cost function
 * 3. Computing weighted average of controls based on trajectory costs
 */
class MPPIController
{
public:
    // Control output structure
    struct ControlCommand
    {
        float vx = 0.0f;    // Forward velocity (mm/s)
        float vy = 0.0f;    // Lateral velocity (mm/s)
        float omega = 0.0f; // Angular velocity (rad/s)
    };

    // Robot state structure
    struct State
    {
        float x = 0.0f;     // Position X (mm)
        float y = 0.0f;     // Position Y (mm)
        float theta = 0.0f; // Orientation (rad)
    };

    // Configuration parameters
    struct Params
    {
        // MPPI parameters
        int K = 1000;              // Number of sampled trajectories
        int T = 30;                // Prediction horizon (time steps)
        float dt = 0.05f;          // Time step (seconds)
        float lambda = 1.0f;       // Temperature parameter (lower = more greedy)

        // Control noise standard deviations
        float sigma_vx = 200.0f;   // mm/s
        float sigma_vy = 200.0f;   // mm/s
        float sigma_omega = 0.3f;  // rad/s

        // Robot limits
        float max_vx = 1000.0f;    // mm/s
        float max_vy = 1000.0f;    // mm/s
        float max_omega = 1.0f;    // rad/s

        // Cost weights
        float w_path = 1.0f;           // Weight for path following
        float w_obstacle = 10.0f;      // Weight for obstacle avoidance
        float w_goal = 0.5f;           // Weight for goal reaching
        float w_smoothness = 0.1f;     // Weight for control smoothness
        float w_speed = 0.05f;         // Weight for maintaining desired speed

        // Safety parameters
        float robot_radius = 250.0f;       // mm
        float safety_margin = 400.0f;      // mm - start penalizing at this distance
        float obstacle_decay = 200.0f;     // mm - exponential decay rate

        // Path following
        float lookahead_distance = 500.0f; // mm - how far ahead to look on path
        float goal_tolerance = 200.0f;     // mm - consider goal reached
    };

    /**
     * @brief Constructor with default parameters
     */
    MPPIController();

    /**
     * @brief Constructor with custom parameters
     */
    explicit MPPIController(const Params& params);

    /**
     * @brief Compute optimal control command
     * @param current_state Current robot state (x, y, theta) in world frame
     * @param path Path to follow as vector of waypoints (world frame, mm)
     * @param obstacles Obstacle points from lidar (world frame, mm)
     * @return Optimal control command (vx, vy, omega)
     */
    ControlCommand compute(const State& current_state,
                          const std::vector<Eigen::Vector2f>& path,
                          const std::vector<Eigen::Vector2f>& obstacles);

    /**
     * @brief Check if the goal has been reached
     * @param current_state Current robot state
     * @param goal Goal position
     * @return true if within goal_tolerance of the goal
     */
    bool goalReached(const State& current_state, const Eigen::Vector2f& goal) const;

    /**
     * @brief Reset the controller (clear previous control sequence)
     */
    void reset();

    /**
     * @brief Set new parameters
     */
    void setParams(const Params& params);

    /**
     * @brief Get current parameters
     */
    const Params& getParams() const { return params_; }

    /**
     * @brief Get the last computed optimal trajectory for visualization
     */
    const std::vector<State>& getOptimalTrajectory() const { return optimal_trajectory_; }

private:
    Params params_;
    std::mt19937 rng_;
    std::normal_distribution<float> noise_vx_;
    std::normal_distribution<float> noise_vy_;
    std::normal_distribution<float> noise_omega_;

    // Previous control sequence for warm-starting
    std::vector<ControlCommand> prev_controls_;

    // Optimal trajectory for visualization
    std::vector<State> optimal_trajectory_;

    /**
     * @brief Simulate robot motion using kinematic model
     * @param state Current state
     * @param control Control input
     * @return Next state after dt
     */
    State simulateStep(const State& state, const ControlCommand& control) const;

    /**
     * @brief Compute trajectory cost
     * @param trajectory Sequence of states
     * @param controls Sequence of controls
     * @param path Target path
     * @param obstacles Obstacle points
     * @return Total cost of the trajectory
     */
    float computeTrajectoryCost(const std::vector<State>& trajectory,
                                const std::vector<ControlCommand>& controls,
                                const std::vector<Eigen::Vector2f>& path,
                                const std::vector<Eigen::Vector2f>& obstacles) const;

    /**
     * @brief Compute cost for path following
     */
    float pathFollowingCost(const State& state,
                           const std::vector<Eigen::Vector2f>& path) const;

    /**
     * @brief Compute cost for obstacle avoidance
     */
    float obstacleCost(const State& state,
                      const std::vector<Eigen::Vector2f>& obstacles) const;

    /**
     * @brief Compute cost for reaching the goal
     */
    float goalCost(const State& state,
                  const std::vector<Eigen::Vector2f>& path) const;

    /**
     * @brief Compute cost for control smoothness
     */
    float smoothnessCost(const ControlCommand& control,
                        const ControlCommand& prev_control) const;

    /**
     * @brief Find closest point on path and compute distance
     */
    std::pair<float, size_t> distanceToPath(const State& state,
                                            const std::vector<Eigen::Vector2f>& path) const;

    /**
     * @brief Find minimum distance to any obstacle
     */
    float minDistanceToObstacles(const State& state,
                                 const std::vector<Eigen::Vector2f>& obstacles) const;

    /**
     * @brief Clamp control to robot limits
     */
    ControlCommand clampControl(const ControlCommand& control) const;

    /**
     * @brief Shift previous controls for warm-starting
     */
    void warmStart();
};

#endif // MPPI_CONTROLLER_H
