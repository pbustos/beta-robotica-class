#include "room_concept_ai.h"
#include <iostream>
#include <QDebug>

namespace rc
{

    void RoomConceptAI::set_initial_state(float width, float length, float x, float y, float phi)
    {
        model_ = std::make_shared<Model>();
        model_->init_from_state(width, length, x, y, phi, params.wall_height);
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
        model_->init_from_polygon(polygon_vertices, init_x, init_y, init_phi, params.wall_height);

        // Reset state but keep covariance reasonable
        last_lidar_timestamp = 0;
        last_update_result = UpdateResult{};
        current_covariance = Eigen::Matrix3f::Identity() * 0.1f;

        qInfo() << "RoomConceptAI initialized with polygon room:" << polygon_vertices.size()
                << "vertices. Robot at (" << init_x << "," << init_y << "," << init_phi << ")";
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
            // Convert propagated covariance from torch tensor to Eigen
            auto cov_acc = prediction.propagated_cov.accessor<float, 2>();
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
            model_->robot_pos.data().copy_(torch::tensor({pred_pos.x(), pred_pos.y()}));
            model_->robot_theta.data().copy_(torch::tensor({pred_theta}));

            // Compute precision matrix (inverse of PROPAGATED covariance)
            Eigen::Matrix3f prior_precision = current_covariance.inverse();

            // Set prediction for prior loss computation
            model_->set_prediction(pred_pos, pred_theta, prior_precision);
        }

        // ===== MINIMISATION STEP (Minimize Variational Free Energy) =====
        const torch::Tensor points_tensor = points_to_tensor_xyz(lidar.first);
        const std::vector<torch::Tensor> optim_params = model_->optim_parameters();
        torch::optim::Adam optimizer(optim_params, torch::optim::AdamOptions(params.learning_rate));

        float last_loss = std::numeric_limits<float>::infinity();
        int iterations = 0;
        for(int i=0; i<params.num_iterations; ++i)
        {
            optimizer.zero_grad();
            const torch::Tensor loss = loss_sdf_mse(points_tensor, *model_);
            loss.backward();
            optimizer.step();

            last_loss = loss.item<float>();
            iterations = i + 1;
            if(last_loss < params.min_loss_threshold)
                break;
        }

        // ===== COVARIANCE UPDATE =====
        // Active Inference / EKF update: P_posterior = (P_prior^-1 + H_likelihood)^-1
        // where H_likelihood is the Hessian of the observation loss
        try {
            // Compute Huber loss for Hessian calculation (consistent with optimization)
            // Note: We use the same loss function as in optimization for consistency
            const torch::Tensor sdf_vals = model_->sdf(points_tensor);
            constexpr float huber_delta = 0.15f;
            const torch::Tensor likelihood_loss = torch::nn::functional::huber_loss(
                sdf_vals,
                torch::zeros_like(sdf_vals),
                torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(huber_delta)
            );

            // Get parameters as vector
            std::vector<torch::Tensor> params_list = model_->optim_parameters();

            // Compute gradients (first order)
            auto first_grads = torch::autograd::grad({likelihood_loss}, params_list,
                                                     /*grad_outputs=*/{},
                                                     /*retain_graph=*/true,
                                                     /*create_graph=*/true);

            // Flatten gradients to vector [3]
            torch::Tensor grad_flat = torch::cat({first_grads[0].flatten(), first_grads[1].flatten()}, 0);

            // Compute Hessian matrix [3x3] via Jacobian of gradients
            std::vector<torch::Tensor> hessian_rows;
            for(int i = 0; i < 3; i++) {
                auto grad_i = grad_flat[i];
                auto second_grads = torch::autograd::grad({grad_i}, params_list,
                                                          /*grad_outputs=*/{},
                                                          /*retain_graph=*/true,
                                                          /*create_graph=*/false);
                torch::Tensor row = torch::cat({second_grads[0].flatten(), second_grads[1].flatten()}, 0);
                hessian_rows.push_back(row);
            }
            torch::Tensor hessian_likelihood = torch::stack(hessian_rows, 0); // [3, 3]

            // Convert Hessian to Eigen
            Eigen::Matrix3f H_likelihood;
            auto h_acc = hessian_likelihood.accessor<float, 2>();
            for(int i = 0; i < 3; i++)
                for(int j = 0; j < 3; j++)
                    H_likelihood(i, j) = h_acc[i][j];

            // ===== BAYESIAN FUSION: P_posterior = (P_prior^-1 + H_likelihood)^-1 =====
            // Get prior precision (inverse of propagated covariance)
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
            const float x = res.state[2];
            const float y = res.state[3];
            const float phi = res.state[4];
            Eigen::Affine2f pose = Eigen::Affine2f::Identity();
            pose.translation() = Eigen::Vector2f{x, y};
            pose.linear() = Eigen::Rotation2Df(phi).toRotationMatrix();
            res.robot_pose = pose;
        }

        // ===== UPDATE MODEL STATE FOR NEXT ITERATION =====
        // Store current pose for next prediction step
        model_->robot_prev_pose = res.robot_pose;

        // Store current covariance as tensor for next prediction
        model_->prev_cov = torch::zeros({3, 3}, torch::kFloat32);
        auto cov_acc = model_->prev_cov.accessor<float, 2>();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cov_acc[i][j] = res.covariance(i, j);

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
        prior.covariance = torch::eye(3, torch::kFloat32);
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

        // Ensure prev_cov is properly initialized
        if (!room->prev_cov.defined() || room->prev_cov.numel() == 0)
        {
            room->prev_cov = 0.1f * torch::eye(dim, torch::kFloat32);
        }

        // Get current pose for Jacobian computation
        if (not room->robot_prev_pose.has_value())
        {
            // Fallback: simple additive noise
            prediction.propagated_cov = room->prev_cov + prediction_params.NOISE_TRANS * prediction_params.NOISE_TRANS * torch::eye(dim, torch::kFloat32);
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

        torch::Tensor F = torch::eye(dim, torch::kFloat32);

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

        float base_trans_noise = 0.002f;  // 2mm base uncertainty

        // Forward uncertainty: grows with forward motion
        float forward_noise = base_trans_noise + prediction_params.NOISE_TRANS * forward_motion;

        // Lateral uncertainty: much smaller (differential drive, 10% of forward)
        float lateral_noise = base_trans_noise + 0.1f * prediction_params.NOISE_TRANS * lateral_motion;

        float base_rot_noise = 0.005f;  // 5 mrad base uncertainty
        float rot_noise = base_rot_noise + prediction_params.NOISE_ROT * std::abs(dtheta);

        torch::Tensor Q = torch::zeros({dim, dim}, torch::kFloat32);

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
