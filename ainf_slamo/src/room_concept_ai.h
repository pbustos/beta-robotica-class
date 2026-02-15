#pragma once

#include <memory>
#include <vector>
#include <limits>

// ---- PyTorch vs Qt macros (slots/signals/emit) ----
// Qt uses 'slots' as a macro. PyTorch/libtorch has methods named slots(), which breaks compilation.
// We temporarily undefine Qt macros *only* while including torch headers, then restore them.
#ifdef slots
  #define RC_QT_SLOTS_WAS_DEFINED
  #undef slots
#endif
#ifdef signals
  #define RC_QT_SIGNALS_WAS_DEFINED
  #undef signals
#endif
#ifdef emit
  #define RC_QT_EMIT_WAS_DEFINED
  #undef emit
#endif

#include <torch/torch.h>

#ifdef RC_QT_SLOTS_WAS_DEFINED
  #define slots Q_SLOTS
  #undef RC_QT_SLOTS_WAS_DEFINED
#endif
#ifdef RC_QT_SIGNALS_WAS_DEFINED
  #define signals Q_SIGNALS
  #undef RC_QT_SIGNALS_WAS_DEFINED
#endif
#ifdef RC_QT_EMIT_WAS_DEFINED
  #define emit Q_EMIT
  #undef RC_QT_EMIT_WAS_DEFINED
#endif

#include <Eigen/Dense>
#include <Lidar3D.h>
#include "common_types.h"
namespace rc
{
/**
 * RoomConceptAI
 * =============
 * Implementación mínima para estimar (o refinar) el estado conjunto robot-habitación usando SDF.
 *
 * Estado (5): [width, length, x, y, phi]
 *  - width/length: dimensiones completas (m)
 *  - x,y,phi: pose del robot respecto al centro de la habitación (habitación centrada en (0,0))
 *
 * Flujo previsto en esta fase:
 *  - set_initial_state(...) con el GT / hipótesis inicial de la habitación y pose aproximada.
 *  - update(...) optimiza esos 5 parámetros minimizando mean(SDF^2) vía Adam.
 */

// ---------------- Minimal Room Model (inline) ----------------
// Supports both axis-aligned box and polygon rooms
class Model : public torch::nn::Module
{
    public:
        // Device for tensor operations (CPU or CUDA)
        torch::Device device_ = torch::kCPU;

        // Room parameters are fixed for now
        torch::Tensor half_extents;  // [half_width, half_length] (no grad) - for box mode
        torch::Tensor polygon_vertices;  // [N, 2] polygon vertices in room frame (no grad)
        bool use_polygon = false;

        // Robot pose wrt room center (optimized)
        torch::Tensor robot_pos;     // [x, y]
        torch::Tensor robot_theta;   // [phi]

        float half_height = 1.2f;    // half of wall height (fixed)

        // --- NEW: Prediction fields for prior loss ---
        torch::Tensor predicted_pos;     // [x, y] from motion model
        torch::Tensor predicted_theta;   // [phi] from motion model
        torch::Tensor prediction_precision_matrix; // Inverse covariance of prediction [3x3] or diagonal [3]
        bool has_prediction = false;

        torch::Tensor prev_cov;
        std::optional<Eigen::Affine2f> robot_prev_pose;

        // Set the device for all tensors
        void set_device(torch::Device device)
        {
            device_ = device;
        }

        void init_from_state(float width, float length, float x, float y, float phi, float wall_height)
        {
            use_polygon = false;
            const float half_w = width * 0.5f;
            const float half_l = length * 0.5f;
            half_height = wall_height * 0.5f;

            // fixed room size
            half_extents = torch::tensor({half_w, half_l}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));

            // optimized robot pose
            robot_pos = torch::tensor({x, y}, torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));
            robot_theta = torch::tensor({phi}, torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));

            register_parameter("robot_pos", robot_pos);
            register_parameter("robot_theta", robot_theta);

            init_common();
        }

        // Pre-computed segment data for faster SDF (computed once in init_from_polygon)
        torch::Tensor seg_a_;      // [num_segs, 2] segment start points
        torch::Tensor seg_b_;      // [num_segs, 2] segment end points
        torch::Tensor seg_ab_;     // [num_segs, 2] segment vectors (b - a)
        torch::Tensor seg_ab_sq_;  // [num_segs] squared length of each segment

        void init_from_polygon(const std::vector<Eigen::Vector2f>& vertices, float x, float y, float phi, float wall_height)
        {
            use_polygon = true;
            half_height = wall_height * 0.5f;

            // Store polygon vertices as tensor [N, 2]
            std::vector<float> verts_flat;
            verts_flat.reserve(vertices.size() * 2);
            for (const auto& v : vertices)
            {
                verts_flat.push_back(v.x());
                verts_flat.push_back(v.y());
            }
            polygon_vertices = torch::from_blob(verts_flat.data(),
                {static_cast<long>(vertices.size()), 2}, torch::kFloat32).clone().to(device_);

            // Pre-compute segment data for faster SDF
            const int64_t num_verts = polygon_vertices.size(0);
            auto indices_a = torch::arange(num_verts, torch::TensorOptions().dtype(torch::kLong).device(device_));
            auto indices_b = (indices_a + 1) % num_verts;
            seg_a_ = polygon_vertices.index_select(0, indices_a).contiguous();
            seg_b_ = polygon_vertices.index_select(0, indices_b).contiguous();
            seg_ab_ = (seg_b_ - seg_a_).contiguous();
            seg_ab_sq_ = torch::sum(seg_ab_ * seg_ab_, /*dim=*/1).contiguous();

            // Compute bounding box for half_extents (for compatibility)
            float min_x = vertices[0].x(), max_x = vertices[0].x();
            float min_y = vertices[0].y(), max_y = vertices[0].y();
            for (const auto& v : vertices)
            {
                min_x = std::min(min_x, v.x()); max_x = std::max(max_x, v.x());
                min_y = std::min(min_y, v.y()); max_y = std::max(max_y, v.y());
            }
            half_extents = torch::tensor({(max_x - min_x) / 2.f, (max_y - min_y) / 2.f},
                torch::TensorOptions().dtype(torch::kFloat32).device(device_));

            // optimized robot pose
            robot_pos = torch::tensor({x, y}, torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));
            robot_theta = torch::tensor({phi}, torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));

            register_parameter("robot_pos", robot_pos);
            register_parameter("robot_theta", robot_theta);

            init_common();
        }

    private:
        void init_common()
        {
            // Initialize prediction tensors (detached from graph)
            predicted_pos = torch::zeros({2}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
            predicted_theta = torch::zeros({1}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
            prediction_precision_matrix = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
            has_prediction = false;

            // Initialize covariance for EKF prediction (moderate initial uncertainty)
            prev_cov = 0.1f * torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32).device(device_));

            // Initialize previous pose
            robot_prev_pose = std::nullopt;  // Will be set after first update
        }

    public:
        void set_prediction(const Eigen::Vector2f &pred_pos, float pred_theta, const Eigen::Matrix3f &precision)
        {
             predicted_pos = torch::tensor({pred_pos.x(), pred_pos.y()},
                 torch::TensorOptions().dtype(torch::kFloat32).device(device_));
             predicted_theta = torch::tensor({pred_theta},
                 torch::TensorOptions().dtype(torch::kFloat32).device(device_));

             // Convert Eigen Matrix to Tensor
             prediction_precision_matrix = torch::from_blob(
                 const_cast<float*>(precision.data()),
                 {3, 3},
                 torch::kFloat32).clone().to(device_);

             has_prediction = true;
        }

        torch::Tensor prior_loss() const
        {
            if (!has_prediction) return torch::tensor(0.0f);

            // Current state vector [x, y, theta]
            auto current_state = torch::cat({robot_pos, robot_theta}, 0); // [3]
            auto pred_state = torch::cat({predicted_pos, predicted_theta}, 0); // [3]

            auto diff = (current_state - pred_state).unsqueeze(1); // [3, 1]

            // (s - mu)^T * Sigma^-1 * (s - mu)
            // [1, 3] * [3, 3] * [3, 1] -> [1, 1]
            auto loss = torch::matmul(diff.t(), torch::matmul(prediction_precision_matrix, diff));

            return 0.5f * loss.squeeze();
        }

        // Differentiable distance from point to line segment
        // Returns squared distance for numerical stability
        static torch::Tensor point_to_segment_distance_sq(
            const torch::Tensor& points,  // [N, 2]
            const torch::Tensor& a,       // [2] segment start
            const torch::Tensor& b)       // [2] segment end
        {
            // Vector from a to b
            auto ab = b - a;  // [2]
            // Vector from a to each point
            auto ap = points - a;  // [N, 2]

            // Project point onto line: t = dot(ap, ab) / dot(ab, ab)
            auto ab_sq = torch::sum(ab * ab);  // scalar
            auto t = torch::sum(ap * ab, /*dim=*/1) / (ab_sq + 1e-8f);  // [N]

            // Clamp t to [0, 1] to stay on segment
            t = torch::clamp(t, 0.0f, 1.0f);  // [N]

            // Closest point on segment: a + t * ab
            auto closest = a + t.unsqueeze(1) * ab;  // [N, 2]

            // Distance squared
            auto diff = points - closest;  // [N, 2]
            return torch::sum(diff * diff, /*dim=*/1);  // [N]
        }

        torch::Tensor sdf(const torch::Tensor &points_robot) const
        {
            // Transform points from robot frame to room frame
            // All operations must be on the same device as input points
            const auto device = points_robot.device();
            const auto pxy = points_robot.index({torch::indexing::Slice(), torch::indexing::Slice(0,2)});

            // Ensure robot_theta and robot_pos are on the same device as points
            const auto theta = robot_theta.to(device);
            const auto pos = robot_pos.to(device);

            const auto c = torch::cos(theta);
            const auto s = torch::sin(theta);

            // Build rotation matrix on the same device
            auto rot = torch::zeros({2, 2}, pxy.options());
            rot[0][0] = c.squeeze();
            rot[0][1] = -s.squeeze();
            rot[1][0] = s.squeeze();
            rot[1][1] = c.squeeze();

            // Map points to room frame: p_room = R(phi) * p_robot + t
            const auto points_room_xy = torch::matmul(pxy, rot.transpose(0,1)) + pos;

            if (use_polygon)
            {
                return sdf_polygon(points_room_xy);
            }
            else
            {
                return sdf_box(points_robot, points_room_xy);
            }
        }

    private:
        torch::Tensor sdf_box(const torch::Tensor& points_robot, const torch::Tensor& points_room_xy) const
        {
            const auto device = points_room_xy.device();
            const auto pz = points_robot.index({torch::indexing::Slice(), 2}).reshape({-1,1});
            const auto points_room = torch::cat({points_room_xy, pz.to(device)}, 1);

            const auto hz = torch::full({1}, half_height, points_room.options());
            const auto half_ext = half_extents.to(device);
            const auto half_sizes = torch::cat({half_ext, hz}, 0);

            const auto abs_points = torch::abs(points_room);
            const auto d = abs_points - half_sizes;

            const auto outside = torch::norm(torch::max(d, torch::zeros_like(d)), 2, 1);
            const auto inside = torch::clamp_max(
                torch::max(torch::max(d.select(1,0), d.select(1,1)), d.select(1,2)), 0.0);
            return outside + inside;
        }

        torch::Tensor sdf_polygon(const torch::Tensor& points_room_xy) const
        {
            // OPTIMIZED FOR CPU: Process segment by segment to avoid large intermediate tensors
            // This keeps autograd working while being memory-efficient

            const int64_t num_points = points_room_xy.size(0);
            const int64_t num_segs = seg_a_.size(0);

            // Ensure segment data is on the same device as points
            const auto seg_a = seg_a_.to(points_room_xy.device());
            const auto seg_ab = seg_ab_.to(points_room_xy.device());
            const auto seg_ab_sq = seg_ab_sq_.to(points_room_xy.device());

            // Initialize with large distance
            auto min_dist_sq = torch::full({num_points}, 1e10f, points_room_xy.options());

            // Process each segment - keeps tensors small and autograd intact
            for (int64_t i = 0; i < num_segs; ++i)
            {
                const auto a = seg_a[i];      // [2]
                const auto ab = seg_ab[i];    // [2]
                const float ab_sq = seg_ab_sq[i].item<float>();

                // ap = points - a  [N, 2]
                const auto ap = points_room_xy - a;

                // t = clamp(dot(ap, ab) / |ab|², 0, 1)  [N]
                auto t = torch::sum(ap * ab, /*dim=*/1) / (ab_sq + 1e-8f);
                t = torch::clamp(t, 0.0f, 1.0f);

                // closest = a + t * ab  [N, 2]
                const auto closest = a + t.unsqueeze(1) * ab;

                // dist² = |points - closest|²  [N]
                const auto diff = points_room_xy - closest;
                const auto dist_sq = torch::sum(diff * diff, /*dim=*/1);

                // Update minimum
                min_dist_sq = torch::minimum(min_dist_sq, dist_sq);
            }

            // Return sqrt with epsilon for numerical stability
            return torch::sqrt(min_dist_sq + 1e-8f);
        }

    public:
        Eigen::Matrix<float,5,1> get_state() const
        {
            // Must convert to CPU for accessor
            auto ext_cpu = half_extents.to(torch::kCPU);
            auto pos_cpu = robot_pos.to(torch::kCPU);
            auto th_cpu  = robot_theta.to(torch::kCPU);
            const auto ext = ext_cpu.accessor<float,1>();
            const auto pos = pos_cpu.accessor<float,1>();
            const auto th  = th_cpu.accessor<float,1>();
            Eigen::Matrix<float,5,1> s;
            s << 2.f*ext[0], 2.f*ext[1], pos[0], pos[1], th[0];
            return s;
        }

        std::vector<torch::Tensor> optim_parameters() const
        {
            return {robot_pos, robot_theta};
        }
};

class RoomConceptAI
{
public:
    struct Params
    {
        int num_iterations = 100;          // Balance between speed and convergence
        float learning_rate_pos = 0.05f;  // Moderate LR for position
        float learning_rate_rot = 0.01f; // Higher LR for rotation - more reactive
        float min_loss_threshold = 0.1f;  // Early exit threshold

        float wall_thickness = 0.1f;
        float wall_height = 2.4f;        // meters
        int max_lidar_points = 200;      // Subsample for speed
        int static_iterations = 10;      // Fewer iterations when stationary
        float pose_smoothing_base = 0.3f;     // More aggressive smoothing
        float pose_smoothing_max = 0.6f;      // Higher max smoothing

        // Adaptive rotation LR based on jitter
        float jitter_threshold_low = 0.002f;   // ~0.1° - below this, use full LR
        float jitter_threshold_high = 0.02f;   // ~1.1° - above this, use minimum LR
        float lr_rot_min_scale = 0.3f;         // Don't reduce LR too much

        // GPU/CPU selection
        bool use_cuda = true;  // Set to true to use GPU if available
    };

    // Get the torch device based on params
    torch::Device get_device() const
    {
        if (params.use_cuda && torch::cuda::is_available())
            return torch::kCUDA;
        return torch::kCPU;
    }

    struct UpdateResult
    {
        bool ok = false;
        float final_loss = 0.f;      // Scaled loss (for optimization)
        float sdf_mse = 0.f;         // Unscaled SDF MSE (for display: sqrt gives avg error in meters)
        Eigen::Matrix<float,5,1> state = Eigen::Matrix<float,5,1>::Zero();
        Eigen::Affine2f robot_pose = Eigen::Affine2f::Identity();
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Identity();
        float condition_number = 0.f;
        int iterations_used = 0;

        // Innovation: difference between optimized pose and prediction (Kalman innovation)
        // Low values indicate good prediction, high values indicate model mismatch
        Eigen::Vector3f innovation = Eigen::Vector3f::Zero();  // [dx, dy, dtheta]
        float innovation_norm = 0.f;  // ||innovation|| for quick health check
    };

    struct OdometryPrior
    {
        bool valid = false;
        Eigen::Vector3f delta_pose;      // [dx, dy, dtheta] in meters & radians
        torch::Tensor covariance;        // 3x3 covariance matrix
        VelocityCommand velocity_cmd;    // ADD: The actual velocity command
        float dt;                        // ADD: Time delta
        float prior_weight = 1.0f;      // How much to trust this prior

        OdometryPrior()
            : delta_pose(Eigen::Vector3f::Zero())
            , covariance(torch::zeros({3,3}, torch::kFloat32))
            , dt(0.0f)
        {}
    };

    RoomConceptAI() = default;

    void set_initial_state(float width, float length, float x, float y, float phi);
    void set_polygon_room(const std::vector<Eigen::Vector2f>& polygon_vertices);
    bool is_initialized() const { return model_ != nullptr; }

    Eigen::Matrix<float,5,1> get_current_state() const
    {
        if (model_) return model_->get_state();
        return Eigen::Matrix<float,5,1>::Zero();
    }

    UpdateResult update(const std::pair<std::vector<Eigen::Vector3f>, std::int64_t> &lidar,
                        const boost::circular_buffer<rc::VelocityCommand> &velocity_history);

    Params params;

    // Process noise covariance (diagonal [x, y, theta])
    Eigen::Vector3f process_noise = {0.01f, 0.01f, 0.01f};

    // Current covariance estimate [3x3]
    Eigen::Matrix3f current_covariance = Eigen::Matrix3f::Identity() * 0.1f;

private:
   std::shared_ptr<Model> model_;
   std::int64_t last_lidar_timestamp = 0;
   UpdateResult last_update_result;
   bool needs_orientation_search_ = true;  // Search for best orientation on first update

   // Smoothed pose to reduce jitter
   Eigen::Vector3f smoothed_pose_ = Eigen::Vector3f::Zero();  // [x, y, theta]
   bool has_smoothed_pose_ = false;

   // Jitter measurement for adaptive learning rate
   static constexpr int JITTER_WINDOW_SIZE = 10;
   std::vector<float> theta_history_;  // Last N theta values
   float current_jitter_ = 0.0f;       // Current measured jitter (std dev)
   float adaptive_lr_rot_ = 0.01f;     // Adaptive learning rate for rotation

   void update_jitter_estimate(float theta);
   float compute_adaptive_lr_rot() const;

    struct PredictionParameters
   {
       float NOISE_TRANS = 0.02f;  // 2cm stddev per meter
       float NOISE_ROT = 0.01f;     // 0.1 rad
   };
    PredictionParameters prediction_params;

    struct PredictionState
    {
        torch::Tensor propagated_cov;  // Predicted covariance
        bool have_propagated = false;   // Whether prediction was performed
        std::vector<float> previous_pose;  // Robot pose BEFORE prediction (for prior loss)
        std::vector<float> predicted_pose; // Robot pose after prediction
    };

    Eigen::Matrix3f compute_motion_covariance(const OdometryPrior &odometry_prior);
    RoomConceptAI::OdometryPrior compute_odometry_prior(
                    const boost::circular_buffer<VelocityCommand>& velocity_history,
                    const std::pair<std::vector<Eigen::Vector3f>, std::int64_t> &lidar);
    Eigen::Vector3f integrate_velocity_over_window(const Eigen::Affine2f &robot_pose,
                                                   const boost::circular_buffer<VelocityCommand> &velocity_history, const int64_t &t_start_ms, const int64_t &t_end_ms);

    PredictionState predict_step( std::shared_ptr<Model> &room,
                                  const OdometryPrior &odometry_prior,
                                  bool is_localized);

    // Find best initial orientation by testing multiple candidates (0°, 90°, 180°, 270°)
    // Returns the orientation with lowest SDF loss
    float find_best_initial_orientation(const std::vector<Eigen::Vector3f>& lidar_points,
                                        float x, float y, float base_phi);

    // points to tensor [N,3] - overload for Eigen::Vector3f
    static torch::Tensor points_to_tensor_xyz(const std::vector<Eigen::Vector3f> &points,
                                               torch::Device device = torch::kCPU)
    {
        std::vector<float> data;
        data.reserve(points.size()*3);
        for(const auto &p : points)
        {
            data.push_back(p.x());
            data.push_back(p.y());
            data.push_back(p.z());
        }
        return torch::from_blob(data.data(), {static_cast<long>(points.size()), 3}, torch::kFloat32).clone().to(device);
    }

    // points to tensor [N,3] - overload for RoboCompLidar3D::TPoints
    static torch::Tensor points_to_tensor_xyz(const RoboCompLidar3D::TPoints &points,
                                               torch::Device device = torch::kCPU)
    {
        std::vector<float> data;
        data.reserve(points.size()*3);
        for(const auto &p : points)
        {
            data.push_back(p.x);
            data.push_back(p.y);
            data.push_back(p.z);
        }
        return torch::from_blob(data.data(), {static_cast<long>(points.size()), 3}, torch::kFloat32).clone().to(device);
    }

    static torch::Tensor loss_sdf_mse(const torch::Tensor &points_xyz, const Model &m)
    {
        const auto sdf_vals = m.sdf(points_xyz);

        // ===== ACTIVE INFERENCE: Variational Free Energy =====
        // Using Huber loss instead of MSE for robustness to outliers
        // (points not on the rectangular walls, e.g., furniture, door frames)
        constexpr float sigma_obs = 0.05f;  // 5cm observation noise
        constexpr float huber_delta = 0.15f; // 15cm threshold: beyond this, use linear penalty
        const float inv_var = 1.0f / (sigma_obs * sigma_obs);

        // Huber loss: quadratic for small errors, linear for large (outliers)
        const auto huber_loss = torch::nn::functional::huber_loss(
            sdf_vals,
            torch::zeros_like(sdf_vals),
            torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(huber_delta)
        );

        const auto likelihood_loss = 0.5f * inv_var * huber_loss;

        // Prior term: Mahalanobis distance from predicted state
        const auto prior_term = m.prior_loss();

        return likelihood_loss + prior_term;
    }

    // Returns median absolute SDF error for UI display (robust to outliers)
    static float compute_sdf_mse_unscaled(const torch::Tensor &points_xyz, const Model &m)
    {
        const auto sdf_vals = m.sdf(points_xyz);
        // Use median absolute error for display (more robust to outliers)
        const auto abs_sdf = torch::abs(sdf_vals);
        return torch::median(abs_sdf).item<float>();
    }
};

} // namespace rc

