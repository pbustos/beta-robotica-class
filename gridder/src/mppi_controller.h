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

#ifndef MPPI_CONTROLLER_H
#define MPPI_CONTROLLER_H

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <limits>

// Forward declaration
class GridESDF;

/**
 * @brief MPPI (Model Predictive Path Integral) Controller for omnidirectional robot
 *
 * This controller generates optimal velocity commands (vx, vy, omega) by:
 * 1. Sampling K trajectories with random control perturbations
 * 2. Each trajectory propagates the robot mean pose and covariance
 * 3. To answer the question: What is the cost when my state is uncertain?
 * 4. Computing weighted average of controls based on trajectory costs
 */
class MPPIController
{
public:
    // Robot type enumeration
    enum class RobotType
    {
        OMNIDIRECTIONAL,  // Holonomic robot (vx, vy, omega independent)
        DIFFERENTIAL      // Non-holonomic robot (only vy, omega; vx = 0)
    };

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
        // Robot model
        RobotType robot_type = RobotType::OMNIDIRECTIONAL;  // Robot kinematic model

        // MPPI parameters
        int K = 100;               // Number of sampled trajectories
        int T = 50;                // Prediction horizon (time steps)
        float dt = 0.1f;           // Time step (seconds) - larger for stability
        float lambda = 50.0f;      // Temperature - should be ~10-100 after cost rescaling
        float cost_scale = 1000.0f; // Divide all costs by this to get totals in range 10-1000

        // Control noise standard deviations (initial values, adapted online)
        float sigma_vx = 80.0f;    // mm/s - lateral exploration
        float sigma_vy = 150.0f;   // mm/s - forward speed variation
        float sigma_omega = 0.3f;  // rad/s - rotation exploration

        // Time-correlated noise (AR(1) process) - CONSERVATIVE settings
        float noise_alpha = 0.8f;  // Temporal correlation [0.5-0.7] - lower = more reactive
        bool use_time_correlated_noise = true;  // Enable/disable AR(1) noise

        // Adaptive covariance parameters - CONSERVATIVE settings
        float cov_adaptation_rate = 0.01f;  // Beta: very slow adaptation for stability
        bool use_adaptive_covariance = true;  // Enable/disable covariance adaptation
        float sigma_min_vx = 40.0f;         // Minimum sigma for vx (mm/s)
        float sigma_min_vy = 80.0f;         // Minimum sigma for vy (mm/s)
        float sigma_min_omega = 0.1f;       // Minimum sigma for omega (rad/s)
        float sigma_max_vx = 100.0f;        // Maximum sigma for vx (mm/s) - reduced from 120
        float sigma_max_vy = 180.0f;        // Maximum sigma for vy (mm/s) - reduced from 250
        float sigma_max_omega = 0.4f;       // Maximum sigma for omega (rad/s) - reduced from 0.5

        // Robot limits
        float max_vx = 300.0f;     // mm/s - limited lateral
        float max_vy = 800.0f;     // mm/s - main forward speed
        float max_omega = 0.5f;    // rad/s - smooth turning

        // Cost weights (TUNED based on cost breakdown analysis)
        float w_path = 1.0f;           // Weight for path following
        float w_obstacle = 50.0f;      // Weight for obstacle avoidance (reduced - was dominating)
        float w_goal = 5.0f;           // Weight for goal reaching (increased)
        float w_smoothness = 1.0f;     // Weight for control smoothness
        float w_speed = 0.001f;        // Weight for speed - VERY LOW (was dominating total cost)

        // Safety parameters
        float robot_radius = 250.0f;       // mm - hard collision threshold
        float collision_buffer = 120.0f;   // mm - soft penalty band before collision
        float safety_margin = 1000.0f;     // mm - outer cost zone
        float obstacle_decay = 100.0f;     // mm - softplus decay parameter

        // Covariance-aware margin inflation (Option A)
        bool use_covariance_inflation = true;   // Enable covariance-aware margin inflation
        float cov_z_score = 1.64f;              // Risk multiplier (1.64 = 95%, 2.33 = 99%)
        float cov_inflation_gate = 2.0f;        // Only inflate when d < gate * safety_margin
        float cov_sigma_max_clamp = 0.5f;       // Clamp sigma_max to this fraction of safety_margin

        // Path following
        float lookahead_distance = 500.0f; // mm - how far ahead to look on path
        float goal_tolerance = 200.0f;     // mm - consider goal reached

        // Warm start blending weights (blend previous controls with nominal)
        float warm_start_vx_weight = 0.2f;     // Weight for previous vx (0-1), rest goes to nominal
        float warm_start_vy_weight = 0.5f;     // Weight for previous vy
        float warm_start_omega_weight = 0.5f;  // Weight for previous omega

        // Obstacle cost smoothing (softmin aggregation)
        int obstacle_k_nearest = 10;           // Number of nearest obstacles for softmin
        float obstacle_softmin_beta = 0.02f;   // Softmin smoothing parameter (1/mm)

        // Nominal control parameters
        float alignment_forward_threshold = 0.3f;   // cos(angle) > this → move forward
        float alignment_backward_threshold = -0.3f; // cos(angle) < this → rotate in place
        float lateral_motion_gain = 0.5f;           // Gain for lateral motion in omnidirectional mode
        float nominal_slow_speed_factor = 0.3f;     // Speed factor when strafing or turning

        // Output smoothing (reduces jitter in velocity commands)
        float output_smoothing_alpha = 0.3f;  // EMA filter: 0 = no smoothing, 0.5 = heavy smoothing
                                               // output = alpha * prev + (1-alpha) * new

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
     * @param obstacles Obstacle points from lidar (world frame, mm) - used for hard collision check
     * @return Optimal control command (vx, vy, omega)
     */
    ControlCommand compute(const State& current_state,
                          const std::vector<Eigen::Vector2f>& path,
                          const std::vector<Eigen::Vector2f>& obstacles);

    /**
     * @brief Compute optimal control command using ESDF for smooth obstacle costs
     * @param current_state Current robot state (x, y, theta) in world frame
     * @param path Path to follow as vector of waypoints (world frame, mm)
     * @param obstacles Obstacle points from lidar (world frame, mm) - used for hard collision check only
     * @param esdf Grid ESDF for smooth obstacle cost computation
     * @return Optimal control command (vx, vy, omega)
     *
     * This version uses the ESDF distance field for smooth obstacle costs while
     * keeping LiDAR points for hard collision detection (safety layer).
     */
    ControlCommand compute(const State& current_state,
                          const std::vector<Eigen::Vector2f>& path,
                          const std::vector<Eigen::Vector2f>& obstacles,
                          const GridESDF* esdf);

    /**
     * @brief Compute optimal control command using ESDF with covariance-aware margin inflation
     * @param current_state Current robot state (x, y, theta) in world frame
     * @param path Path to follow as vector of waypoints (world frame, mm)
     * @param obstacles Obstacle points from lidar (world frame, mm) - used for hard collision check only
     * @param esdf Grid ESDF for smooth obstacle cost computation
     * @param pose_cov Pose covariance [σ²_xx, σ_xy, σ_xθ, σ²_yy, σ_yθ, σ²_θθ] (upper triangular 3x3)
     * @return Optimal control command (vx, vy, omega)
     *
     * This version inflates the safety margin based on pose uncertainty using the ESDF gradient.
     * Higher pose uncertainty → larger effective safety margin → more conservative behavior.
     */
    ControlCommand compute(const State& current_state,
                          const std::vector<Eigen::Vector2f>& path,
                          const std::vector<Eigen::Vector2f>& obstacles,
                          const GridESDF* esdf,
                          const std::vector<float>& pose_cov);

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

    // Adaptive covariance (diagonal) - updated online based on elite trajectories
    float adaptive_sigma_vx_;
    float adaptive_sigma_vy_;
    float adaptive_sigma_omega_;

    // Adaptive lambda (temperature) - adjusted based on ESS
    float adaptive_lambda_;

    // Output smoothing (EMA filter to reduce jitter)
    mutable ControlCommand last_smoothed_output_{0.0f, 0.0f, 0.0f};
    mutable bool first_output_ = true;

    /**
     * @brief Simulate robot motion using kinematic model
     * @param state Current state
     * @param control Control input
     * @return Next state after dt
     */
    State simulateStep(const State& state, const ControlCommand& control) const;

    /**
     * @brief Find closest point on path and compute distance
     */
    std::pair<float, size_t> distanceToPath(const State& state,
                                            const std::vector<Eigen::Vector2f>& path) const;


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
