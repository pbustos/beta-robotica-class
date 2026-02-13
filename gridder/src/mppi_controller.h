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
    // Robot frame convention: X+ = right (lateral), Y+ = forward
    struct ControlCommand
    {
        float vx = 0.0f;    // Lateral velocity - positive = right (mm/s)
        float vy = 0.0f;    // Forward velocity - positive = forward (mm/s)
        float omega = 0.0f; // Angular velocity - positive = counterclockwise (rad/s)
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
        int K = 100;               // Number of sampled trajectories
        int T = 50;                // Prediction horizon (time steps)
        float dt = 0.1f;           // Time step (seconds) - larger for stability
        float lambda = 1.0f;       // Temperature parameter

        // Control noise standard deviations
        float sigma_vx = 80.0f;    // mm/s - lateral exploration
        float sigma_vy = 150.0f;   // mm/s - forward speed variation
        float sigma_omega = 0.3f;  // rad/s - rotation exploration

        // Robot limits
        float max_vx = 300.0f;     // mm/s - limited lateral
        float max_vy = 800.0f;     // mm/s - main forward speed
        float max_omega = 0.5f;    // rad/s - smooth turning

        // Cost weights
        float w_path = 5.0f;           // Weight for path following (increased to keep trajectories on path)
        float w_obstacle = 10.0f;      // Weight for obstacle avoidance
        float w_goal = 2.0f;           // Weight for goal reaching
        float w_smoothness = 3.0f;     // Weight for control smoothness
        float w_speed = 0.1f;          // Weight for maintaining desired speed
        float w_heading = 3.0f;        // Weight for heading alignment

        // Safety parameters
        float robot_radius = 250.0f;       // mm
        float safety_margin = 500.0f;      // mm - start penalizing at this distance (increased)
        float obstacle_decay = 150.0f;     // mm - exponential decay rate (faster decay)

        // Path following
        float lookahead_distance = 500.0f; // mm - how far ahead to look on path
        float goal_tolerance = 200.0f;     // mm - consider goal reached

        // Visualization
        int num_trajectories_to_draw = 10; // Number of best trajectories to visualize
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

    /**
     * @brief Get the N best trajectories for visualization
     */
    const std::vector<std::vector<State>>& getBestTrajectories() const { return best_trajectories_; }

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

    // N best trajectories for visualization
    std::vector<std::vector<State>> best_trajectories_;

    // Pre-allocated noise buffers for performance
    mutable std::vector<float> noise_buffer_vx_;
    mutable std::vector<float> noise_buffer_vy_;
    mutable std::vector<float> noise_buffer_omega_;

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

    /**
     * @brief Compute nominal control towards path (initial guess for sampling)
     */
    ControlCommand computeNominalControl(const State& state,
                                         const std::vector<Eigen::Vector2f>& path) const;
};

#endif // MPPI_CONTROLLER_H
