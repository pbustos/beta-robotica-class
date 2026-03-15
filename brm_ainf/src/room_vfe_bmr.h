#pragma once

#include <Eigen/Dense>
#include <array>
#include <optional>
#include <vector>

namespace rc
{

struct VFEComponents
{
    float accuracy = 0.f;
    float complexity = 0.f;
    float total = 0.f;
    float trace_term = 0.f;
    float mahal_sq = 0.f;
    float logdet_ratio = 0.f;
    bool valid = false;
};

struct BmrResult
{
    enum class ProposalType
    {
        NONE = 0,
        EXPAND_L_SHAPE = 1,
        ADD_TWO_POINT_INDENT = 2
    };

    float log_bf_01 = 0.f;
    float posterior_z_sq = 0.f;
    float logdet_shrinkage = 0.f;
    Eigen::Vector3f ext_post_mean = Eigen::Vector3f::Zero();  // [L2, W2, s]
    Eigen::Matrix3f ext_post_cov = Eigen::Matrix3f::Identity();

    ProposalType proposal = ProposalType::NONE;

    // Bayesian Reduction structural hypothesis:
    // - 10 Manhattan vertices total.
    // - First 4 are mandatory rectangle corners (high precision).
    // - Remaining 6 are latent structural vertices (start at zero precision).
    std::array<float, 10> vertex_precision = {1.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    int activated_vertex_a = -1; // latent vertex index activated by BMR
    int activated_vertex_b = -1; // latent vertex index activated by BMR

    int indent_side = -1;   // 0=bottom,1=right,2=top,3=left
    float indent_a = 0.f;   // segment start (side-local coordinate)
    float indent_b = 0.f;   // segment end (side-local coordinate)
    float indent_depth = 0.f;

    bool expand = false;
    bool valid = false;
};

class RoomVFEBMR
{
public:
    float bmr_threshold = -2.0f;
    float ext_prior_sigma_dim = 3.0f;
    float ext_prior_sigma_pos = 2.0f;

    /// End-to-end periodic BMR check from robot-frame lidar + current state.
    /// Returns nullopt when disabled or not due yet.
    std::optional<BmrResult> maybe_evaluate_from_lidar(
        const std::vector<Eigen::Vector3f> &points_robot_xyz,
        const Eigen::Matrix<float,5,1> &state,
        bool enable_bmr,
        int bmr_check_period,
        int bmr_min_frames_before_check);

    void reset_cycle_counter() { bmr_frame_counter_ = 0; }

    BmrResult evaluate_bmr_from_points(
        const std::vector<Eigen::Vector2f> &points_room_xy,
        float L1,
        float W1) const;

private:
    int bmr_frame_counter_ = 0;
    static float log_det_spd(const Eigen::Matrix3f &m);
    Eigen::Matrix3f make_ext_prior_cov() const;
};

} // namespace rc
