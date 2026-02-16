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
#include "room_model.h"

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

class RoomConceptAI
{
public:
    struct Params
    {
        int num_iterations = 50;          // Balance between speed and convergence
        float learning_rate_pos = 0.05f;  // Moderate LR for position
        float learning_rate_rot = 0.01f;  // LR for rotation
        float min_loss_threshold = 0.1f;  // Early exit threshold

        float wall_thickness = 0.1f;
        float wall_height = 2.4f;        // meters
        int max_lidar_points = 500;      // Subsample for speed
        float pose_smoothing = 0.3f;     // EMA smoothing factor (0=no smoothing, 1=full smoothing)

        // GPU/CPU selection
        // Note: For small tensors (~200 points), CPU is faster due to GPU transfer overhead
        bool use_cuda = false;

        // ===== Prediction-based Early Exit =====
        // If predicted pose already has low SDF error, skip optimization entirely
        // This saves CPU when motion model is accurate (smooth motion)
        bool prediction_early_exit = true;
        float sigma_sdf = 0.15f;              // SDF observation noise (15cm)
        float prediction_trust_factor = 0.5f; // Threshold = sigma_sdf * factor (~7.5cm)
        int min_tracking_steps = 20;          // Wait for system to stabilize before early exit
        float max_uncertainty_for_early_exit = 0.1f;  // Max pose uncertainty to allow early exit

        // ===== Velocity-Adaptive Gradient Weights =====
        // Adjust optimization emphasis based on current motion profile
        bool velocity_adaptive_weights = true;
        float linear_velocity_threshold = 0.05f;   // m/s - below this = "not moving linearly"
        float angular_velocity_threshold = 0.05f;  // rad/s - below this = "not rotating"
        float weight_boost_factor = 2.0f;          // Multiplier for emphasized parameters
        float weight_reduction_factor = 0.5f;      // Multiplier for de-emphasized parameters
        float weight_smoothing_alpha = 0.3f;       // EMA smoothing for weight transitions
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
        Eigen::Vector3f innovation = Eigen::Vector3f::Zero();  // [dx, dy, dtheta]
        float innovation_norm = 0.f;  // ||innovation|| for quick health check
    };

    struct OdometryPrior
    {
        bool valid = false;
        Eigen::Vector3f delta_pose;      // [dx, dy, dtheta] in meters & radians
        torch::Tensor covariance;        // 3x3 covariance matrix
        VelocityCommand velocity_cmd;    // The actual velocity command
        float dt;                        // Time delta
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
    void set_robot_pose(float x, float y, float theta);  // Set robot pose manually (e.g., from UI click)
    bool is_initialized() const { return model_ != nullptr; }

    // Grid search for initial pose (solves kidnapping problem)
    // Returns true if a good pose was found, false otherwise
    bool grid_search_initial_pose(const std::vector<Eigen::Vector3f>& lidar_points,
                                   float grid_resolution = 0.5f,  // meters
                                   float angle_resolution = M_PI_4);  // 45 degrees

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

   // Manual pose reset - skip optimization for a few frames
   int manual_reset_frames_ = 0;  // Counter to skip optimization after manual reset

   // Smoothed pose to reduce jitter
   Eigen::Vector3f smoothed_pose_ = Eigen::Vector3f::Zero();  // [x, y, theta]
   bool has_smoothed_pose_ = false;

   // Prediction-based early exit tracking
   int tracking_step_count_ = 0;        // Number of tracking steps since init
   int prediction_early_exits_ = 0;     // Counter for statistics

   // Velocity-adaptive gradient weights [x, y, theta]
   Eigen::Vector3f current_velocity_weights_ = Eigen::Vector3f::Ones();

   // Compute velocity-adaptive weights based on motion profile
   Eigen::Vector3f compute_velocity_adaptive_weights(const OdometryPrior& odometry_prior);

    struct PredictionParameters
   {
       // Process noise - controls how fast covariance grows during prediction
       // Higher values = faster covariance growth = more frequent optimizations
       // These are realistic values for a differential drive robot with encoder noise
       float NOISE_TRANS = 0.20f;  // 20% error per meter of motion (realistic wheel slip)
       float NOISE_ROT = 0.10f;    // ~6° stddev per radian of rotation
       float NOISE_BASE = 0.05f;   // Base noise even when stationary (5cm drift)
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
                                                   const boost::circular_buffer<VelocityCommand> &velocity_history,
                                                   const int64_t &t_start_ms, const int64_t &t_end_ms);

    PredictionState predict_step(std::shared_ptr<Model> &room,
                                  const OdometryPrior &odometry_prior,
                                  bool is_localized);

    // Find best initial orientation by testing multiple candidates (0°, 90°, 180°, 270°)
    float find_best_initial_orientation(const std::vector<Eigen::Vector3f>& lidar_points,
                                        float x, float y, float base_phi);

    // points to tensor [N,3] - overload for Eigen::Vector3f
    static torch::Tensor points_to_tensor_xyz(const std::vector<Eigen::Vector3f> &points,
                                               torch::Device device = torch::kCPU)
    {
        std::vector<float> data(points.size() * 3);
        for (size_t i = 0; i < points.size(); ++i)
        {
            const auto &p = points[i];
            const size_t idx = i * 3;
            data[idx] = p.x();
            data[idx + 1] = p.y();
            data[idx + 2] = p.z();
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

        // Active Inference: Variational Free Energy with Huber loss for robustness
        constexpr float sigma_obs = 0.05f;  // 5cm observation noise
        constexpr float huber_delta = 0.15f; // 15cm threshold
        const float inv_var = 1.0f / (sigma_obs * sigma_obs);

        const auto huber_loss = torch::nn::functional::huber_loss(
            sdf_vals,
            torch::zeros_like(sdf_vals),
            torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(huber_delta)
        );

        const auto likelihood_loss = 0.5f * inv_var * huber_loss;
        const auto prior_term = m.prior_loss();

        return likelihood_loss + prior_term;
    }

    // Returns median absolute SDF error for UI display (robust to outliers)
    static float compute_sdf_mse_unscaled(const torch::Tensor &points_xyz, const Model &m)
    {
        const auto sdf_vals = m.sdf(points_xyz);
        const auto abs_sdf = torch::abs(sdf_vals);
        return torch::median(abs_sdf).item<float>();
    }
};

} // namespace rc

