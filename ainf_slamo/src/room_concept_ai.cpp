#include "room_concept_ai.h"
#include <iostream>
#include <QDebug>

namespace rc
{
    // Static initialization: limit PyTorch threads to avoid CPU overload
    static bool torch_threads_initialized = []() {
        torch::set_num_threads(2);  // Limit to 2 threads
        torch::set_num_interop_threads(1);
        return true;
    }();

    float RoomConceptAI::find_best_initial_orientation(const std::vector<Eigen::Vector3f>& lidar_points,
                                                        float x, float y, float base_phi)
    {
        if (model_ == nullptr || lidar_points.empty())
            return base_phi;

        // Test 4 orientations: base, +90°, +180°, +270° (covers symmetries)
        const std::vector<float> angle_offsets = {0.0f, M_PI_2, M_PI, 3.0f * M_PI_2};

        // Also test mirrored positions (x, y) and (-x, y) with all rotations
        // This handles axis-mirroring ambiguity
        const std::vector<std::pair<float, float>> position_variants = {
            {x, y},      // Original
            {-x, y},     // Mirror X
            {x, -y},     // Mirror Y
            {-x, -y}     // Mirror both
        };

        // Subsample points for faster evaluation
        std::vector<Eigen::Vector3f> sample_points;
        const int max_samples = 100;
        const int stride = std::max(1, static_cast<int>(lidar_points.size()) / max_samples);
        for (size_t i = 0; i < lidar_points.size(); i += stride)
            sample_points.push_back(lidar_points[i]);

        const torch::Tensor points_tensor = points_to_tensor_xyz(sample_points);

        float best_phi = base_phi;
        float best_x = x;
        float best_y = y;
        float best_loss = std::numeric_limits<float>::infinity();

        // Test all combinations of position variants and angle offsets
        for (const auto& [test_x, test_y] : position_variants)
        {
            for (float offset : angle_offsets)
            {
                float test_phi = base_phi + offset;
                // Normalize to [-pi, pi]
                while (test_phi > M_PI) test_phi -= 2.0f * M_PI;
                while (test_phi < -M_PI) test_phi += 2.0f * M_PI;

                // Temporarily set the pose
                model_->robot_pos.data().copy_(torch::tensor({test_x, test_y},
                    torch::TensorOptions().device(get_device())));
                model_->robot_theta.data().copy_(torch::tensor({test_phi},
                    torch::TensorOptions().device(get_device())));

                // Evaluate SDF loss (without gradient)
                torch::NoGradGuard no_grad;
                const auto sdf_vals = model_->sdf(points_tensor);
                const float loss = torch::mean(torch::square(sdf_vals)).item<float>();

                qDebug() << "  Testing pos=(" << test_x << "," << test_y << ") phi="
                         << qRadiansToDegrees(test_phi) << "° -> loss=" << loss;

                if (loss < best_loss)
                {
                    best_loss = loss;
                    best_phi = test_phi;
                    best_x = test_x;
                    best_y = test_y;
                }
            }
        }

        // Set the best position found
        model_->robot_pos.data().copy_(torch::tensor({best_x, best_y},
            torch::TensorOptions().device(get_device())));

        qInfo() << "Best initial pose: (" << best_x << "," << best_y << ") phi="
                << qRadiansToDegrees(best_phi) << "° (loss=" << best_loss << ")";
        return best_phi;
    }

    bool RoomConceptAI::grid_search_initial_pose(const std::vector<Eigen::Vector3f>& lidar_points,
                                                  float grid_resolution,
                                                  float angle_resolution)
    {
        if (model_ == nullptr || lidar_points.empty())
            return false;

        qInfo() << "Starting grid search for initial pose...";

        // Get room bounds from polygon or half_extents
        float min_x, max_x, min_y, max_y;
        if (model_->use_polygon && model_->polygon_vertices.defined())
        {
            auto verts_cpu = model_->polygon_vertices.to(torch::kCPU);
            auto acc = verts_cpu.accessor<float, 2>();
            min_x = max_x = acc[0][0];
            min_y = max_y = acc[0][1];
            for (int i = 1; i < verts_cpu.size(0); i++)
            {
                min_x = std::min(min_x, acc[i][0]);
                max_x = std::max(max_x, acc[i][0]);
                min_y = std::min(min_y, acc[i][1]);
                max_y = std::max(max_y, acc[i][1]);
            }
        }
        else
        {
            auto he_cpu = model_->half_extents.to(torch::kCPU);
            float hw = he_cpu[0].item<float>();
            float hh = he_cpu[1].item<float>();
            min_x = -hw; max_x = hw;
            min_y = -hh; max_y = hh;
        }

        // Add margin inside the room (robot shouldn't be at the walls)
        const float margin = 0.3f;  // 30cm from walls
        min_x += margin; max_x -= margin;
        min_y += margin; max_y -= margin;

        // Subsample lidar points for faster evaluation
        std::vector<Eigen::Vector3f> sample_points;
        const int max_samples = 150;
        const int stride = std::max(1, static_cast<int>(lidar_points.size()) / max_samples);
        for (size_t i = 0; i < lidar_points.size(); i += stride)
            sample_points.push_back(lidar_points[i]);

        const torch::Tensor points_tensor = points_to_tensor_xyz(sample_points, get_device());

        // Generate angle candidates
        std::vector<float> angles;
        for (float a = -M_PI; a < M_PI; a += angle_resolution)
            angles.push_back(a);

        float best_x = 0, best_y = 0, best_theta = 0;
        float best_loss = std::numeric_limits<float>::infinity();
        int total_tests = 0;

        // Grid search
        for (float x = min_x; x <= max_x; x += grid_resolution)
        {
            for (float y = min_y; y <= max_y; y += grid_resolution)
            {
                for (float theta : angles)
                {
                    // Set test pose
                    model_->robot_pos.data().copy_(torch::tensor({x, y},
                        torch::TensorOptions().device(get_device())));
                    model_->robot_theta.data().copy_(torch::tensor({theta},
                        torch::TensorOptions().device(get_device())));

                    // Evaluate SDF loss
                    torch::NoGradGuard no_grad;
                    const auto sdf_vals = model_->sdf(points_tensor);
                    const float loss = torch::mean(torch::square(sdf_vals)).item<float>();

                    if (loss < best_loss)
                    {
                        best_loss = loss;
                        best_x = x;
                        best_y = y;
                        best_theta = theta;
                    }
                    total_tests++;
                }
            }
        }

        // Set the best pose found
        model_->robot_pos.data().copy_(torch::tensor({best_x, best_y},
            torch::TensorOptions().device(get_device())));
        model_->robot_theta.data().copy_(torch::tensor({best_theta},
            torch::TensorOptions().device(get_device())));

        // Initialize smoothed pose
        smoothed_pose_ = Eigen::Vector3f(best_x, best_y, best_theta);
        has_smoothed_pose_ = true;

        // Reset tracking state
        needs_orientation_search_ = false;  // We already found the best orientation
        tracking_step_count_ = 0;
        current_covariance = Eigen::Matrix3f::Identity() * 0.1f;

        qInfo() << "Grid search complete:" << total_tests << "poses tested";
        qInfo() << "Best pose: (" << best_x << "," << best_y << ") theta="
                << qRadiansToDegrees(best_theta) << "° (loss=" << best_loss << ")";

        // Return true if we found a reasonable pose (loss < threshold)
        return best_loss < 1.0f;  // Threshold for "good" pose
    }

    void RoomConceptAI::set_initial_state(float width, float length, float x, float y, float phi)
    {
        model_ = std::make_shared<Model>();
        model_->set_device(get_device());  // Set device before init
        model_->init_from_state(width, length, x, y, phi, params.wall_height);
        needs_orientation_search_ = true;  // Will search for best orientation on first update
        has_smoothed_pose_ = false;  // Reset smoothing
        tracking_step_count_ = 0;  // Reset early exit tracking
        prediction_early_exits_ = 0;
        current_velocity_weights_ = Eigen::Vector3f::Ones();  // Reset velocity weights

        qInfo() << "RoomConceptAI using device:" << (get_device() == torch::kCUDA ? "CUDA" : "CPU");
    }


    void RoomConceptAI::set_polygon_room(const std::vector<Eigen::Vector2f>& polygon_vertices)
    {
        if (polygon_vertices.size() < 3)
        {
            std::cerr << "set_polygon_room: Need at least 3 vertices" << std::endl;
            return;
        }

        // Vertices are in room frame (where user clicked on the viewer)
        // Keep current robot pose if we have one, otherwise start at origin
        float init_x = 0.0f;
        float init_y = 0.0f;
        float init_phi = 0.0f;

        if (model_ != nullptr)
        {
            // Preserve current robot pose
            const auto state = model_->get_state();
            init_x = state[2];
            init_y = state[3];
            init_phi = state[4];
        }

        model_ = std::make_shared<Model>();
        model_->set_device(get_device());  // Set device before init
        model_->init_from_polygon(polygon_vertices, init_x, init_y, init_phi, params.wall_height);

        // Reset state but keep covariance reasonable
        last_lidar_timestamp = 0;
        last_update_result = UpdateResult{};
        current_covariance = Eigen::Matrix3f::Identity() * 0.1f;
        needs_orientation_search_ = true;  // Will search for best orientation on first update
        has_smoothed_pose_ = false;  // Reset smoothing
        tracking_step_count_ = 0;  // Reset early exit tracking
        prediction_early_exits_ = 0;
        current_velocity_weights_ = Eigen::Vector3f::Ones();  // Reset velocity weights

        qInfo() << "RoomConceptAI initialized with polygon room:" << polygon_vertices.size()
                << "vertices. Robot at (" << init_x << "," << init_y << "," << init_phi << ")";
    }

    void RoomConceptAI::set_robot_pose(float x, float y, float theta)
    {
        if (model_ == nullptr)
        {
            qWarning() << "Cannot set robot pose: model not initialized";
            return;
        }

        // Directly update robot pose tensors
        model_->robot_pos = torch::tensor({x, y}, torch::TensorOptions().dtype(torch::kFloat32).device(model_->device_).requires_grad(true));
        model_->robot_theta = torch::tensor({theta}, torch::TensorOptions().dtype(torch::kFloat32).device(model_->device_).requires_grad(true));

        // Reset smoothed pose to new position
        smoothed_pose_ = Eigen::Vector3f(x, y, theta);
        has_smoothed_pose_ = true;

        // Reset covariance to reasonable value (we're uncertain after manual placement)
        current_covariance = Eigen::Matrix3f::Identity() * 0.1f;

        // Reset tracking counters and prediction state
        tracking_step_count_ = 0;
        needs_orientation_search_ = false;  // User explicitly set orientation
        last_lidar_timestamp = 0;  // Force fresh start

        // Skip optimization for a few frames to let the new pose "settle"
        manual_reset_frames_ = 5;

        // Clear any previous prediction
        model_->has_prediction = false;
        model_->robot_prev_pose = std::nullopt;

        // Reset last update result with new pose
        last_update_result = UpdateResult{};
        last_update_result.robot_pose.translation() = Eigen::Vector2f(x, y);
        last_update_result.robot_pose.linear() = Eigen::Rotation2Df(theta).toRotationMatrix();
        last_update_result.ok = true;
    }

    Eigen::Vector3f RoomConceptAI::compute_velocity_adaptive_weights(const OdometryPrior& odometry_prior)
    {
        /**
         * Compute velocity-adaptive precision weights for [x, y, theta].
         *
         * Based on the current velocity profile:
         * - If rotating (high angular, low linear): boost theta weight, reduce x,y
         * - If moving straight (high linear, low angular): boost x,y, reduce theta
         * - If stationary: use base weights (uniform)
         *
         * The weights scale gradients during optimization, making the system
         * more responsive to parameters expected to change based on motion.
         */
        if (!params.velocity_adaptive_weights || !odometry_prior.valid)
        {
            return Eigen::Vector3f::Ones();
        }

        // Get velocities from odometry prior
        const float linear_speed = odometry_prior.delta_pose.head<2>().norm() /
                                   std::max(odometry_prior.dt, 0.001f);
        const float angular_speed = std::abs(odometry_prior.delta_pose[2]) /
                                    std::max(odometry_prior.dt, 0.001f);

        // Determine motion profile
        const bool is_rotating = angular_speed > params.angular_velocity_threshold;
        const bool is_translating = linear_speed > params.linear_velocity_threshold;

        float w_x, w_y, w_theta;

        if (is_rotating && !is_translating)
        {
            // Pure rotation: emphasize theta, de-emphasize x, y
            w_x = params.weight_reduction_factor;
            w_y = params.weight_reduction_factor;
            w_theta = params.weight_boost_factor;
        }
        else if (is_translating && !is_rotating)
        {
            // Pure translation: emphasize x, y based on direction
            // Get velocity direction in robot frame
            const float vx = odometry_prior.delta_pose[0] / std::max(odometry_prior.dt, 0.001f);
            const float vy = odometry_prior.delta_pose[1] / std::max(odometry_prior.dt, 0.001f);

            if (std::abs(vy) > std::abs(vx))
            {
                // Mostly forward/backward motion - emphasize y (forward axis)
                w_x = 1.0f;
                w_y = params.weight_boost_factor;
            }
            else
            {
                // Mostly lateral motion - emphasize x
                w_x = params.weight_boost_factor;
                w_y = 1.0f;
            }
            w_theta = params.weight_reduction_factor;
        }
        else if (is_rotating && is_translating)
        {
            // Combined motion: moderate boost for all
            w_x = 1.2f;
            w_y = 1.2f;
            w_theta = 1.2f;
        }
        else
        {
            // Stationary: use base weights
            w_x = 1.0f;
            w_y = 1.0f;
            w_theta = 1.0f;
        }

        Eigen::Vector3f new_weights(w_x, w_y, w_theta);

        // Smooth transition using exponential moving average
        const float alpha = params.weight_smoothing_alpha;
        current_velocity_weights_ = (1.0f - alpha) * current_velocity_weights_ + alpha * new_weights;

        return current_velocity_weights_;
    }

    RoomConceptAI::UpdateResult RoomConceptAI::update(
                const std::pair<std::vector<Eigen::Vector3f>, int64_t> &lidar,
                const boost::circular_buffer<VelocityCommand> &velocity_history)
    {
        UpdateResult res;
        if(lidar.first.empty())
            return res;

        if(model_ == nullptr)
            return res;

        // ===== MANUAL RESET: Skip optimization and use current pose =====
        if (manual_reset_frames_ > 0)
        {
            manual_reset_frames_--;

            // Just return current pose without optimization
            const auto state = model_->get_state();
            res.ok = true;
            res.state = state;
            res.robot_pose.translation() = Eigen::Vector2f(state[2], state[3]);
            res.robot_pose.linear() = Eigen::Rotation2Df(state[4]).toRotationMatrix();
            res.covariance = current_covariance;
            res.final_loss = 0.0f;
            res.sdf_mse = 0.0f;

            last_update_result = res;
            last_lidar_timestamp = lidar.second;

            qInfo() << "Manual reset: skipping optimization (" << manual_reset_frames_ << " frames remaining)";
            return res;
        }

        // ===== ORIENTATION SEARCH ON FIRST UPDATE =====
        // Test multiple orientations (0°, 90°, 180°, 270°) to avoid symmetry traps
        if (needs_orientation_search_)
        {
            const auto state = model_->get_state();
            const float best_phi = find_best_initial_orientation(lidar.first, state[2], state[3], state[4]);
            model_->robot_theta.data().copy_(torch::tensor({best_phi},
                torch::TensorOptions().device(get_device())));
            needs_orientation_search_ = false;
            qInfo() << "Initial orientation search complete. Using phi=" << qRadiansToDegrees(best_phi) << "°";
        }

        // ===== ODOMETRY PRIOR BETWEEN LIDAR MEASUREMENTS =====
        auto odometry_prior = compute_odometry_prior(velocity_history, lidar);

        // ===== PREDICT STEP =====
        // Propagates state: x_pred = x_prev + f(u, dt)
        // Propagates covariance: P_pred = F*P*F^T + Q where F is motion model Jacobian
        const PredictionState prediction = predict_step(model_, odometry_prior, true);

        // ===== UPDATE PRIOR COVARIANCE WITH PROPAGATED VALUE =====
        // EKF predict: P_prior = F * P_prev * F^T + Q
        if (prediction.have_propagated && prediction.propagated_cov.defined())
        {
            // Convert propagated covariance from torch tensor to Eigen (must be on CPU for accessor)
            auto cov_cpu = prediction.propagated_cov.to(torch::kCPU);
            auto cov_acc = cov_cpu.accessor<float, 2>();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    current_covariance(i, j) = cov_acc[i][j];
        }

        // ===== ACTIVE INFERENCE: Set prediction as prior for optimization =====
        Eigen::Vector2f pred_pos;
        float pred_theta;

        if (last_update_result.ok && odometry_prior.valid)
        {
            // Compute predicted pose: x_pred = x_prev + delta
            pred_pos = last_update_result.robot_pose.translation()
                     + odometry_prior.delta_pose.head<2>();
            pred_theta = std::atan2(last_update_result.robot_pose.linear()(1, 0),
                                    last_update_result.robot_pose.linear()(0, 0))
                       + odometry_prior.delta_pose[2];

            // Normalize angle to [-pi, pi]
            while (pred_theta > M_PI) pred_theta -= 2.0f * M_PI;
            while (pred_theta < -M_PI) pred_theta += 2.0f * M_PI;

            // Initialize optimizer state with prediction (crucial for Active Inference)
            // Use data().copy_() to modify values without breaking the computation graph
            model_->robot_pos.data().copy_(torch::tensor({pred_pos.x(), pred_pos.y()},
                torch::TensorOptions().device(get_device())));
            model_->robot_theta.data().copy_(torch::tensor({pred_theta},
                torch::TensorOptions().device(get_device())));

            // Compute precision matrix (inverse of PROPAGATED covariance)
            Eigen::Matrix3f prior_precision = current_covariance.inverse();

            // Set prediction for prior loss computation
            model_->set_prediction(pred_pos, pred_theta, prior_precision);
        }

        // ===== MINIMISATION STEP (Minimize Variational Free Energy) =====
        // Subsample lidar points for speed
        const auto& all_points = lidar.first;
        std::vector<Eigen::Vector3f> sampled_points;
        if (static_cast<int>(all_points.size()) > params.max_lidar_points)
        {
            const int stride = static_cast<int>(all_points.size()) / params.max_lidar_points;
            sampled_points.reserve(params.max_lidar_points);
            for (size_t i = 0; i < all_points.size(); i += stride)
                sampled_points.push_back(all_points[i]);
        }
        else
        {
            sampled_points = all_points;
        }

        const torch::Tensor points_tensor = points_to_tensor_xyz(sampled_points, get_device());

        // Increment tracking step counter
        tracking_step_count_++;

        // ===== PREDICTION-BASED EARLY EXIT =====
        // If the predicted pose already has low SDF error, skip optimization entirely.
        // This saves significant CPU when the robot moves smoothly.
        if (params.prediction_early_exit &&
            last_update_result.ok &&
            odometry_prior.valid &&
            tracking_step_count_ > params.min_tracking_steps)
        {
            // Check pose uncertainty (trace of position covariance)
            const float pose_uncertainty = current_covariance(0,0) + current_covariance(1,1);

            if (pose_uncertainty < params.max_uncertainty_for_early_exit)
            {
                // Evaluate SDF at predicted pose (no gradients needed)
                torch::NoGradGuard no_grad;
                const auto sdf_pred = model_->sdf(points_tensor);
                const float mean_sdf_pred = torch::mean(torch::abs(sdf_pred)).item<float>();

                // Early exit threshold
                const float prediction_trust_threshold = params.sigma_sdf * params.prediction_trust_factor;

                if (mean_sdf_pred < prediction_trust_threshold)
                {
                    // Prediction is good enough - skip optimization entirely
                    prediction_early_exits_++;

                    res.ok = true;
                    res.final_loss = mean_sdf_pred;
                    res.sdf_mse = mean_sdf_pred;
                    res.iterations_used = 0;
                    res.state = model_->get_state();

                    // Build pose from predicted values (already set in model)
                    const float x = res.state[2];
                    const float y = res.state[3];
                    const float phi = res.state[4];

                    // Apply smoothing even for early exit
                    float smoothed_x = x, smoothed_y = y, smoothed_phi = phi;
                    if (has_smoothed_pose_)
                    {
                        const float alpha = params.pose_smoothing;
                        smoothed_x = alpha * smoothed_pose_[0] + (1.0f - alpha) * x;
                        smoothed_y = alpha * smoothed_pose_[1] + (1.0f - alpha) * y;
                        float angle_diff = phi - smoothed_pose_[2];
                        while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
                        while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
                        smoothed_phi = smoothed_pose_[2] + (1.0f - alpha) * angle_diff;
                    }
                    smoothed_pose_ = Eigen::Vector3f(smoothed_x, smoothed_y, smoothed_phi);
                    has_smoothed_pose_ = true;

                    res.state[2] = smoothed_x;
                    res.state[3] = smoothed_y;
                    res.state[4] = smoothed_phi;

                    Eigen::Affine2f pose = Eigen::Affine2f::Identity();
                    pose.translation() = Eigen::Vector2f{smoothed_x, smoothed_y};
                    pose.linear() = Eigen::Rotation2Df(smoothed_phi).toRotationMatrix();
                    res.robot_pose = pose;

                    // Use PROPAGATED covariance (uncertainty grows when we skip optimization)
                    // current_covariance was already updated with prediction.propagated_cov above
                    res.covariance = current_covariance;
                    res.innovation = Eigen::Vector3f::Zero();  // No correction applied
                    res.innovation_norm = 0.0f;

                    // Update current_covariance to reflect the propagated uncertainty
                    // (This is already done above in the prediction step, but we store it in result)

                    // Update model with smoothed pose
                    model_->robot_pos.data().copy_(torch::tensor({smoothed_x, smoothed_y},
                        torch::TensorOptions().device(get_device())));
                    model_->robot_theta.data().copy_(torch::tensor({smoothed_phi},
                        torch::TensorOptions().device(get_device())));
                    model_->robot_prev_pose = res.robot_pose;
                    model_->has_prediction = false;

                    last_update_result = res;
                    last_lidar_timestamp = lidar.second;

                    // Debug: log early exits periodically
                    if (prediction_early_exits_ % 50 == 0)
                    {
                        qDebug() << "[EARLY_EXIT] Skipped optimization. SDF:" << mean_sdf_pred
                                 << "m, threshold:" << prediction_trust_threshold
                                 << "m, total early exits:" << prediction_early_exits_;
                    }

                    return res;
                }
            }
        }

        // Use different learning rates for position vs rotation
        std::vector<torch::optim::OptimizerParamGroup> param_groups;
        param_groups.emplace_back(
            std::vector<torch::Tensor>{model_->robot_pos},
            std::make_unique<torch::optim::AdamOptions>(params.learning_rate_pos)
        );
        param_groups.emplace_back(
            std::vector<torch::Tensor>{model_->robot_theta},
            std::make_unique<torch::optim::AdamOptions>(params.learning_rate_rot)
        );
        torch::optim::Adam optimizer(param_groups);

        // Compute velocity-adaptive weights for gradient scaling
        const Eigen::Vector3f velocity_weights = compute_velocity_adaptive_weights(odometry_prior);

        // Determine number of iterations
        const int max_iters = params.num_iterations;

        float last_loss = std::numeric_limits<float>::infinity();
        float prev_loss = std::numeric_limits<float>::infinity();
        int iterations = 0;
        for(int i=0; i < max_iters; ++i)
        {
            optimizer.zero_grad();
            const torch::Tensor loss = loss_sdf_mse(points_tensor, *model_);
            loss.backward();

            // Apply velocity-adaptive gradient weighting
            // This emphasizes parameters that are expected to change based on motion
            {
                torch::NoGradGuard no_grad;
                if (model_->robot_pos.grad().defined())
                {
                    model_->robot_pos.mutable_grad().index({0}) *= velocity_weights[0];  // x weight
                    model_->robot_pos.mutable_grad().index({1}) *= velocity_weights[1];  // y weight
                }
                if (model_->robot_theta.grad().defined())
                {
                    model_->robot_theta.mutable_grad() *= velocity_weights[2];   // theta weight
                }
            }

            optimizer.step();

            prev_loss = last_loss;
            last_loss = loss.item<float>();
            iterations = i + 1;

            // Early exit: absolute threshold or relative convergence
            if(last_loss < params.min_loss_threshold)
                break;
            // If loss changed by less than 1%, we've converged
            if (i > 5 && std::abs(prev_loss - last_loss) < 0.01f * prev_loss)
                break;
        }

        // ===== COVARIANCE UPDATE =====
        // Active Inference / EKF update: P_posterior = (P_prior^-1 + H_likelihood)^-1
        // Using diagonal Hessian approximation for efficiency (Gauss-Newton style)
        try {
            // Compute gradients for diagonal Hessian approximation
            const torch::Tensor sdf_vals = model_->sdf(points_tensor);
            constexpr float huber_delta = 0.15f;
            const torch::Tensor likelihood_loss = torch::nn::functional::huber_loss(
                sdf_vals,
                torch::zeros_like(sdf_vals),
                torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(huber_delta)
            );

            // Get parameters
            std::vector<torch::Tensor> params_list = model_->optim_parameters();

            // Compute first-order gradients
            auto grads = torch::autograd::grad({likelihood_loss}, params_list,
                                               /*grad_outputs=*/{},
                                               /*retain_graph=*/false,
                                               /*create_graph=*/false);

            // Diagonal Hessian approximation: H_ii ≈ (∂L/∂θ_i)² / L
            // This is a Fisher Information approximation
            torch::Tensor grad_flat = torch::cat({grads[0].flatten(), grads[1].flatten()}, 0);
            float loss_val = likelihood_loss.item<float>();

            // Compute diagonal Hessian approximation (must be on CPU for accessor)
            Eigen::Matrix3f H_likelihood = Eigen::Matrix3f::Zero();
            auto grad_cpu = grad_flat.to(torch::kCPU);
            auto g_acc = grad_cpu.accessor<float, 1>();

            // Use gradient magnitude as proxy for curvature
            // Scale by number of points for proper normalization
            float scale = static_cast<float>(points_tensor.size(0)) / (loss_val + 1e-6f);
            for (int i = 0; i < 3; i++) {
                H_likelihood(i, i) = std::abs(g_acc[i]) * scale + 1e-4f;  // Ensure positive
            }

            // ===== BAYESIAN FUSION: P_posterior = (P_prior^-1 + H_likelihood)^-1 =====
            Eigen::Matrix3f prior_precision = current_covariance.inverse();

            // Posterior precision = prior precision + likelihood precision (Hessian)
            // Add small regularization for numerical stability
            constexpr float lambda = 1e-4f;
            Eigen::Matrix3f posterior_precision = prior_precision + H_likelihood
                                                 + lambda * Eigen::Matrix3f::Identity();

            // Compute posterior covariance
            Eigen::Matrix3f new_cov = posterior_precision.inverse();

            // Compute condition number for diagnostics
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(posterior_precision);
            const auto eigenvalues = solver.eigenvalues();
            const float max_ev = eigenvalues.maxCoeff();
            const float min_ev = eigenvalues.minCoeff();
            res.condition_number = (min_ev > 1e-8f) ? (max_ev / min_ev) : 1e8f;

            // Validate covariance
            if (new_cov.allFinite() && new_cov.determinant() > 1e-10f && res.condition_number < 1e6f) {
                current_covariance = new_cov;
                res.covariance = new_cov;
            } else {
                // Keep propagated covariance if fusion fails
                res.covariance = current_covariance;
            }

        } catch (const std::exception &e) {
            // Hessian computation failed, use propagated covariance
            std::cerr << "Covariance update failed: " << e.what() << std::endl;
            res.covariance = current_covariance;
            res.condition_number = -1.0f; // Signal failure
        }

        res.ok = true;
        res.final_loss = last_loss;
        res.sdf_mse = compute_sdf_mse_unscaled(points_tensor, *model_);  // Unscaled for UI
        res.iterations_used = iterations;
        res.state = model_->get_state();
        {
            float x = res.state[2];
            float y = res.state[3];
            float phi = res.state[4];

            // ===== POSE SMOOTHING to reduce jitter =====
            // Apply exponential moving average filter
            if (has_smoothed_pose_)
            {
                const float alpha = params.pose_smoothing;

                x = alpha * smoothed_pose_[0] + (1.0f - alpha) * x;
                y = alpha * smoothed_pose_[1] + (1.0f - alpha) * y;

                // Smooth angle carefully (handle wrap-around)
                float angle_diff = phi - smoothed_pose_[2];
                while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
                phi = smoothed_pose_[2] + (1.0f - alpha) * angle_diff;
                while (phi > M_PI) phi -= 2.0f * M_PI;
                while (phi < -M_PI) phi += 2.0f * M_PI;
            }

            // Store smoothed pose for next iteration
            smoothed_pose_ = Eigen::Vector3f(x, y, phi);
            has_smoothed_pose_ = true;


            // Update model with smoothed values
            model_->robot_pos.data().copy_(torch::tensor({x, y},
                torch::TensorOptions().device(get_device())));
            model_->robot_theta.data().copy_(torch::tensor({phi},
                torch::TensorOptions().device(get_device())));

            // Update result state with smoothed values
            res.state[2] = x;
            res.state[3] = y;
            res.state[4] = phi;

            Eigen::Affine2f pose = Eigen::Affine2f::Identity();
            pose.translation() = Eigen::Vector2f{x, y};
            pose.linear() = Eigen::Rotation2Df(phi).toRotationMatrix();
            res.robot_pose = pose;

            // ===== COMPUTE INNOVATION (Kalman residual) =====
            // Innovation = optimized_state - predicted_state
            if (model_->has_prediction)
            {
                res.innovation[0] = x - model_->predicted_pos[0].item<float>();
                res.innovation[1] = y - model_->predicted_pos[1].item<float>();
                float pred_theta = model_->predicted_theta[0].item<float>();
                res.innovation[2] = phi - pred_theta;
                // Normalize angle difference to [-pi, pi]
                while (res.innovation[2] > M_PI) res.innovation[2] -= 2.0f * M_PI;
                while (res.innovation[2] < -M_PI) res.innovation[2] += 2.0f * M_PI;

                // Compute norm (position only for simplicity)
                res.innovation_norm = std::sqrt(res.innovation[0]*res.innovation[0] +
                                                res.innovation[1]*res.innovation[1]);
            }
        }

        // ===== UPDATE MODEL STATE FOR NEXT ITERATION =====
        // Store current pose for next prediction step
        model_->robot_prev_pose = res.robot_pose;

        // Store current covariance as tensor for next prediction
        // Create on CPU first for accessor, then move to device
        auto cov_cpu = torch::zeros({3, 3}, torch::kFloat32);
        auto cov_acc = cov_cpu.accessor<float, 2>();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cov_acc[i][j] = res.covariance(i, j);
        model_->prev_cov = cov_cpu.to(get_device());

        // Reset prediction flag for next cycle
        model_->has_prediction = false;

        last_update_result = res;
        return res;
    }

    RoomConceptAI::OdometryPrior RoomConceptAI::compute_odometry_prior(
             const boost::circular_buffer<VelocityCommand>& velocity_history,
             const std::pair<std::vector<Eigen::Vector3f>, std::int64_t> &lidar)
    {
         OdometryPrior prior;
         prior.valid = false;
         const auto &[points, lidar_timestamp] = lidar;

         if (last_lidar_timestamp == 0)
         {
             last_lidar_timestamp = lidar_timestamp;
             return prior;
         }

         // Calculate dt
         const auto dt = lidar_timestamp - last_lidar_timestamp;
         if (dt <= 0)
        {
             last_lidar_timestamp = lidar_timestamp;
             return prior;
         }
         prior.dt = dt;

        if (!velocity_history.empty() && last_update_result.ok)
            prior.delta_pose = integrate_velocity_over_window(last_update_result.robot_pose,
                                                              velocity_history,
                                                        last_lidar_timestamp,
                                                        lidar_timestamp);
        else
            // If no history or no valid previous pose, assume STATIONARY (Zero motion)
            // This protects us when sitting still!
            prior.delta_pose = Eigen::Vector3f::Zero();

        prior.valid = true; // ALWAYS valid now


        // Compute covariance
        Eigen::Matrix3f cov_eigen = compute_motion_covariance(prior);
        prior.covariance = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32).device(get_device()));
        prior.covariance[0][0] = cov_eigen(0, 0);
        prior.covariance[1][1] = cov_eigen(1, 1);
        prior.covariance[2][2] = cov_eigen(2, 2);

        last_lidar_timestamp = lidar_timestamp;
        return prior;
    }

    // ===== HELPER METHOD: Compute motion-based covariance =====
    /**
     * Compute motion-based covariance consistently
     * σ = base + k * distance
     */
    Eigen::Matrix3f RoomConceptAI::compute_motion_covariance(const OdometryPrior &odometry_prior)
    {
        float motion_magnitude = std::sqrt(
            odometry_prior.delta_pose[0] * odometry_prior.delta_pose[0] +
            odometry_prior.delta_pose[1] * odometry_prior.delta_pose[1]
        );

        // Proper motion model: uncertainty grows with distance
        // BUT: when stationary, use much tighter uncertainty to prevent drift
        float base_uncertainty;
        if (motion_magnitude < 0.01f) {
            // Stationary: Very tight constraint (1mm)
            base_uncertainty = 0.001f;  // 1mm when not moving
        } else {
            // Moving: Normal base uncertainty
            base_uncertainty = 0.005f;  // 5mm base when moving
        }

        float noise_per_meter = prediction_params.NOISE_TRANS;  // Use configured value
        float position_std = base_uncertainty + noise_per_meter * motion_magnitude;

        float base_rot_std = 0.01f;  // 10 mrad base
        float noise_per_radian = prediction_params.NOISE_ROT;
        float rotation_std = base_rot_std + noise_per_radian * std::abs(odometry_prior.delta_pose[2]);

        Eigen::Matrix3f cov = Eigen::Matrix3f::Identity();
        cov(0, 0) = position_std * position_std;
        cov(1, 1) = position_std * position_std;
        cov(2, 2) = rotation_std * rotation_std;

        return cov;
    }

     Eigen::Vector3f RoomConceptAI::integrate_velocity_over_window(
                const Eigen::Affine2f& robot_pose,
                const boost::circular_buffer<VelocityCommand> &velocity_history,
                const std::int64_t &t_start_ms,
                const std::int64_t &t_end_ms)
    {
        using clock = std::chrono::high_resolution_clock;
        const auto t_start = clock::time_point(std::chrono::milliseconds(t_start_ms));
        const auto t_end = clock::time_point(std::chrono::milliseconds(t_end_ms));

        Eigen::Vector3f total_delta = Eigen::Vector3f::Zero();

        float running_theta = std::atan2(robot_pose.linear()(1,0), robot_pose.linear()(0,0));

        // Integrate over all velocity commands in [t_start, t_end]
        for (size_t i = 0; i < velocity_history.size(); ++i) {
            const auto&[adv_x, adv_z, rot, timestamp] = velocity_history[i];

            // Get time window for this command
            auto cmd_start = timestamp;
            auto cmd_end = (i + 1 < velocity_history.size())
                           ? velocity_history[i + 1].timestamp
                           : t_end;

            // Clip to [t_start, t_end]
            if (cmd_end < t_start) continue;
            if (cmd_start > t_end) break;

            auto effective_start = std::max(cmd_start, t_start);
            auto effective_end = std::min(cmd_end, t_end);

            const float dt = std::chrono::duration<float>(effective_end - effective_start).count();
            if (dt <= 0) continue;

            // Integrate this segment
            const float dx_local = (adv_x * dt);
            const float dy_local = (adv_z * dt);
            const float dtheta = -rot * dt;  // Negative for right-hand rule

            // Transform to global frame using RUNNING theta
            total_delta[0] += dx_local * std::cos(running_theta) - dy_local * std::sin(running_theta);
            total_delta[1] += dx_local * std::sin(running_theta) + dy_local * std::cos(running_theta);
            total_delta[2] += dtheta;

            // Update running theta for next segment
            running_theta += dtheta;
        }

        return total_delta;
    }

    RoomConceptAI::PredictionState RoomConceptAI::predict_step(
                                std::shared_ptr<Model> &room,
                                const OdometryPrior &odometry_prior,
                                bool is_localized)
    {
        int dim = is_localized ? 3 : 5;  // 3 for localized [x,y,theta], 5 for full state [w,h,x,y,theta]
        PredictionState prediction;
        const auto device = get_device();

        // Ensure prev_cov is properly initialized
        if (!room->prev_cov.defined() || room->prev_cov.numel() == 0)
        {
            room->prev_cov = 0.1f * torch::eye(dim, torch::TensorOptions().dtype(torch::kFloat32).device(device));
        }

        // Get current pose for Jacobian computation
        if (not room->robot_prev_pose.has_value())
        {
            // Fallback: simple additive noise
            prediction.propagated_cov = room->prev_cov + prediction_params.NOISE_TRANS * prediction_params.NOISE_TRANS *
                torch::eye(dim, torch::TensorOptions().dtype(torch::kFloat32).device(device));
            return prediction;
        }

        auto robot_prev_pose = room->robot_prev_pose.value();
        const float theta = std::atan2(robot_prev_pose.linear()(1, 0), robot_prev_pose.linear()(0, 0));

        // Transform global delta back to robot frame for noise computation
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        float dx_global = odometry_prior.delta_pose[0];
        float dy_global = odometry_prior.delta_pose[1];
        float dtheta = odometry_prior.delta_pose[2];

        // Inverse rotation: robot_frame = R^T * global_frame
        float dx_local = dx_global * cos_t + dy_global * sin_t;
        float dy_local = -dx_global * sin_t + dy_global * cos_t;

        // ===== MOTION MODEL JACOBIAN =====
        // State: [x, y, theta] (for localized) or [w, h, x, y, theta] (for mapping)

        torch::Tensor F = torch::eye(dim, torch::TensorOptions().dtype(torch::kFloat32).device(device));

        if (is_localized) {
            // Jacobian for robot pose only [x, y, theta]
            // ∂x'/∂θ = -dy_local*sin(θ) - dx_local*cos(θ)
            // ∂y'/∂θ =  dy_local*cos(θ) - dx_local*sin(θ)

            F[0][2] = -dy_local * sin_t - dx_local * cos_t;
            F[1][2] =  dy_local * cos_t - dx_local * sin_t;
        } else {
            // Jacobian for full state [w, h, x, y, theta]
            F[2][4] = -dy_local * sin_t - dx_local * cos_t;
            F[3][4] =  dy_local * cos_t - dx_local * sin_t;
        }

        // ===== PROCESS NOISE =====
        // ANISOTROPIC: For Y=forward, X=right coordinate system:
        //   dy_local = FORWARD motion (should get larger noise)
        //   dx_local = LATERAL motion (should get smaller noise)

        float forward_motion = std::abs(dy_local);   // Forward (Y in robot frame)
        float lateral_motion = std::abs(dx_local);   // Lateral (X in robot frame)

        // Base noise ensures covariance grows even when stationary
        float base_trans_noise = prediction_params.NOISE_BASE;  // Base uncertainty (2cm default)

        // Forward uncertainty: grows with forward motion
        float forward_noise = base_trans_noise + prediction_params.NOISE_TRANS * forward_motion;

        // Lateral uncertainty: smaller for differential drive (30% of forward)
        float lateral_noise = base_trans_noise + 0.3f * prediction_params.NOISE_TRANS * lateral_motion;

        // Rotation noise: base + motion-dependent
        float base_rot_noise = prediction_params.NOISE_BASE * 0.5f;  // Half of trans noise for rotation
        float rot_noise = base_rot_noise + prediction_params.NOISE_ROT * std::abs(dtheta);

        torch::Tensor Q = torch::zeros({dim, dim}, torch::TensorOptions().dtype(torch::kFloat32).device(device));

        if (is_localized) {
            // Build noise covariance in robot frame: Q = diag(lateral², forward², theta²)
            float sigma_x = lateral_noise;   // X = right/lateral (smaller)
            float sigma_y = forward_noise;   // Y = forward (larger)
            float sigma_theta = rot_noise;

            // Transform to global frame: Q_global = R * Q_local * R^T
            // For X=right, Y=forward:
            Q[0][0] = sigma_x*sigma_x * sin_t*sin_t + sigma_y*sigma_y * cos_t*cos_t;
            Q[0][1] = (sigma_y*sigma_y - sigma_x*sigma_x) * cos_t * sin_t;
            Q[1][0] = Q[0][1];
            Q[1][1] = sigma_x*sigma_x * cos_t*cos_t + sigma_y*sigma_y * sin_t*sin_t;
            Q[2][2] = sigma_theta * sigma_theta;
        } else {
            // Full state: room doesn't accumulate noise, only robot pose
            float sigma_x = lateral_noise;
            float sigma_y = forward_noise;
            float sigma_theta = rot_noise;

            Q[2][2] = sigma_y*sigma_y * cos_t*cos_t + sigma_x*sigma_x * sin_t*sin_t;
            Q[2][3] = (sigma_y*sigma_y - sigma_x*sigma_x) * cos_t * sin_t;
            Q[3][2] = Q[2][3];
            Q[3][3] = sigma_y*sigma_y * sin_t*sin_t + sigma_x*sigma_x * cos_t*cos_t;
            Q[4][4] = sigma_theta * sigma_theta;
        }

        // ===== EKF PREDICTION =====
        // P_pred = F * P_prev * F^T + Q
        torch::Tensor propagated = torch::matmul(torch::matmul(F, room->prev_cov), F.t()) + Q;

        prediction.propagated_cov = propagated;
        prediction.have_propagated = true;
        return prediction;
    }
} // namespace rc
