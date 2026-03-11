#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace rc
{

class ObjectOwnershipEM
{
public:
    struct Config
    {
        int max_iterations = 8;
        float convergence_eps = 1e-3f;
        float anneal_t_start = 2.0f;
        float anneal_t_end = 0.6f;
        float robust_sigma = 0.25f;
        float rgbd_residual_weight = 0.0f;
        float max_pose_jump_xy = 0.50f;
        float max_pose_jump_yaw = 0.35f;
    };

    struct ClassModel
    {
        std::string class_id;
        std::string mesh_path;
        bool is_background = false;
        float prior_weight = 1.0f;
        float expected_height = 0.0f;
    };

    struct ClassState
    {
        std::string class_id;
        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        float yaw_rad = 0.0f;
        float scale = 1.0f;
        float confidence = 0.0f;
        bool active = true;
    };

    struct ObservationBatch
    {
        std::vector<Eigen::Vector3f> points_world;
        // Optional camera-frame points and residual hints for depth-aware E-step terms.
        std::vector<Eigen::Vector3f> points_camera;
        std::vector<float> rgbd_residual_hint;
        Eigen::Affine2f robot_pose = Eigen::Affine2f::Identity();
        std::int64_t timestamp_ms = 0;
    };

    struct Metrics
    {
        float objective = 0.0f;
        float avg_entropy = 0.0f;
        float background_fraction = 0.0f;
        std::vector<float> class_residuals;
        std::vector<float> class_confidences;
    };

    using OwnershipMatrix = Eigen::MatrixXf;

    ObjectOwnershipEM() = default;

    void configure(const Config& cfg);
    void set_models(const std::vector<ClassModel>& models);
    void set_observations(const ObservationBatch& observations);
    void set_initial_states(const std::vector<ClassState>& states);

    bool run_single_iteration();
    bool run_em(int max_iters = -1);
    
    const std::vector<ClassState>& get_states() const;
    const OwnershipMatrix& get_ownership() const;
    const Metrics& get_metrics() const;

private:
    static float robust_charbonnier(float r, float sigma);

    Config config_;
    std::vector<ClassModel> models_;
    ObservationBatch observations_;
    std::vector<ClassState> states_;
    OwnershipMatrix ownership_;
    Metrics metrics_;
    int em_iteration_ = 0;
};

} // namespace rc
