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

        void init_from_state(float width, float length, float x, float y, float phi, float wall_height)
        {
            use_polygon = false;
            const float half_w = width * 0.5f;
            const float half_l = length * 0.5f;
            half_height = wall_height * 0.5f;

            // fixed room size
            half_extents = torch::tensor({half_w, half_l}, torch::kFloat32);

            // optimized robot pose
            robot_pos = torch::tensor({x, y}, torch::requires_grad(true));
            robot_theta = torch::tensor({phi}, torch::requires_grad(true));

            register_parameter("robot_pos", robot_pos);
            register_parameter("robot_theta", robot_theta);

            init_common();
        }

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
                {static_cast<long>(vertices.size()), 2}, torch::kFloat32).clone();

            // Compute bounding box for half_extents (for compatibility)
            float min_x = vertices[0].x(), max_x = vertices[0].x();
            float min_y = vertices[0].y(), max_y = vertices[0].y();
            for (const auto& v : vertices)
            {
                min_x = std::min(min_x, v.x()); max_x = std::max(max_x, v.x());
                min_y = std::min(min_y, v.y()); max_y = std::max(max_y, v.y());
            }
            half_extents = torch::tensor({(max_x - min_x) / 2.f, (max_y - min_y) / 2.f}, torch::kFloat32);

            // optimized robot pose
            robot_pos = torch::tensor({x, y}, torch::requires_grad(true));
            robot_theta = torch::tensor({phi}, torch::requires_grad(true));

            register_parameter("robot_pos", robot_pos);
            register_parameter("robot_theta", robot_theta);

            init_common();
        }

    private:
        void init_common()
        {
            // Initialize prediction tensors (detached from graph)
            predicted_pos = torch::zeros({2}, torch::kFloat32);
            predicted_theta = torch::zeros({1}, torch::kFloat32);
            prediction_precision_matrix = torch::eye(3, torch::kFloat32);
            has_prediction = false;

            // Initialize covariance for EKF prediction (moderate initial uncertainty)
            prev_cov = 0.1f * torch::eye(3, torch::kFloat32);

            // Initialize previous pose
            robot_prev_pose = std::nullopt;  // Will be set after first update
        }

    public:
        void set_prediction(const Eigen::Vector2f &pred_pos, float pred_theta, const Eigen::Matrix3f &precision)
        {
             predicted_pos = torch::tensor({pred_pos.x(), pred_pos.y()}, torch::kFloat32);
             predicted_theta = torch::tensor({pred_theta}, torch::kFloat32);

             // Convert Eigen Matrix to Tensor
             auto options = torch::TensorOptions().dtype(torch::kFloat32);
             prediction_precision_matrix = torch::from_blob(
                 const_cast<float*>(precision.data()),
                 {3, 3},
                 options).clone(); // Clone to own memory

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
            const auto pxy = points_robot.index({torch::indexing::Slice(), torch::indexing::Slice(0,2)});

            const auto c = torch::cos(robot_theta);
            const auto s = torch::sin(robot_theta);
            const auto rot = torch::stack({
                torch::stack({c.squeeze(), -s.squeeze()}),
                torch::stack({s.squeeze(),  c.squeeze()})
            });

            // Map points to room frame: p_room = R(phi) * p_robot + t
            const auto points_room_xy = torch::matmul(pxy, rot.transpose(0,1)) + robot_pos;

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
            const auto pz = points_robot.index({torch::indexing::Slice(), 2}).reshape({-1,1});
            const auto points_room = torch::cat({points_room_xy, pz}, 1);

            const auto hz = torch::full({1}, half_height, points_room.options());
            const auto half_sizes = torch::cat({half_extents.to(points_room.options()), hz}, 0);

            const auto abs_points = torch::abs(points_room);
            const auto d = abs_points - half_sizes;

            const auto outside = torch::norm(torch::max(d, torch::zeros_like(d)), 2, 1);
            const auto inside = torch::clamp_max(
                torch::max(torch::max(d.select(1,0), d.select(1,1)), d.select(1,2)), 0.0);
            return outside + inside;
        }

        torch::Tensor sdf_polygon(const torch::Tensor& points_room_xy) const
        {
            // Compute minimum distance to any edge of the polygon
            // For each point, find distance to all segments and take minimum
            const int64_t num_points = points_room_xy.size(0);
            const int64_t num_vertices = polygon_vertices.size(0);

            // Initialize with large distance
            auto min_dist_sq = torch::full({num_points}, 1e10f, points_room_xy.options());

            // Loop through polygon edges
            for (int64_t i = 0; i < num_vertices; ++i)
            {
                const int64_t j = (i + 1) % num_vertices;
                const auto a = polygon_vertices[i];  // [2]
                const auto b = polygon_vertices[j];  // [2]

                auto dist_sq = point_to_segment_distance_sq(points_room_xy, a, b);
                min_dist_sq = torch::minimum(min_dist_sq, dist_sq);
            }

            // Return distance (sqrt of squared distance)
            // Adding small epsilon for numerical stability in backward pass
            return torch::sqrt(min_dist_sq + 1e-8f);
        }

    public:
        Eigen::Matrix<float,5,1> get_state() const
        {
            const auto ext = half_extents.accessor<float,1>();
            const auto pos = robot_pos.accessor<float,1>();
            const auto th  = robot_theta.accessor<float,1>();
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
        int num_iterations = 100;      // Increased for better convergence
        float learning_rate = 0.05f;   // Increased for faster convergence
        float min_loss_threshold = 1e-4f;
        float wall_thickness = 0.1f;
        float wall_height = 2.4f;   // meters
    };

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

    // points to tensor [N,3] - overload for Eigen::Vector3f
    static torch::Tensor points_to_tensor_xyz(const std::vector<Eigen::Vector3f> &points)
    {
        std::vector<float> data;
        data.reserve(points.size()*3);
        for(const auto &p : points)
        {
            data.push_back(p.x());
            data.push_back(p.y());
            data.push_back(p.z());
        }
        return torch::from_blob(data.data(), {static_cast<long>(points.size()), 3}, torch::kFloat32).clone();
    }

    // points to tensor [N,3] - overload for RoboCompLidar3D::TPoints
    static torch::Tensor points_to_tensor_xyz(const RoboCompLidar3D::TPoints &points)
    {
        std::vector<float> data;
        data.reserve(points.size()*3);
        for(const auto &p : points)
        {
            data.push_back(p.x);
            data.push_back(p.y);
            data.push_back(p.z);
        }
        return torch::from_blob(data.data(), {static_cast<long>(points.size()), 3}, torch::kFloat32).clone();
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

