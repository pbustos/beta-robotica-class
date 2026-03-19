#pragma once

#include <Eigen/Dense>
#include <array>
#include <functional>
#include <optional>
#include <vector>

namespace rc
{
/// Callback for scoring a candidate room polygon by SDF free energy.
/// Input: polygon vertices in room frame. Returns: VFE accuracy (lower = better evidence).
using CandidateScorer = std::function<float(const std::vector<Eigen::Vector2f>&)>;


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
        ADD_TWO_POINT_INDENT = 2,
        ADD_CORNER_INDENT = 3
    };

    float log_bf_01 = 0.f;
    float posterior_z_sq = 0.f;
    float logdet_shrinkage = 0.f;
    Eigen::Vector3f ext_post_mean = Eigen::Vector3f::Zero();  // [L2, W2, s]
    Eigen::Matrix3f ext_post_cov = Eigen::Matrix3f::Identity();

    ProposalType proposal = ProposalType::NONE;

    // An evaluated indent candidate (wall or corner) exposed for visualization.
    struct IndentCandidateInfo
    {
        ProposalType proposal = ProposalType::NONE;
        int side   = -1;   // wall-indent: 0=bottom,1=right,2=top,3=left; corner: -1
        int corner = -1;   // corner-indent: 0=BL,1=BR,2=TR,3=TL; wall: -1
        float a     = 0.f; // segment start (side-local [-1,1])
        float b     = 0.f; // segment end
        float depth = 0.f;   // wall: single depth; corner: legacy compat (= depth_x)
        float depth_x = 0.f; // corner: indent along the horizontal side
        float depth_y = 0.f; // corner: indent along the vertical side
        float score = 0.f;
        float raw_score = 0.f; // VFE score without proximity (for diagnostics)
        bool  is_chosen = false; // true for the candidate that wins the round
    };

    // All evaluated candidates from the latest check (wall_best + corner_best).
    std::vector<IndentCandidateInfo> all_candidates;

    // Top hot-zone segments from the distance profile (pre-VFE, pre-proximity).
    struct HotZoneSegment
    {
        int   side  = -1;    // 0=bottom,1=right,2=top,3=left
        int   i0    = 0;     // first bin index
        int   i1    = 0;     // last bin index
        float a     = 0.f;   // segment start in side-local [-1,1]
        float b     = 0.f;   // segment end
        float mean_dist = 0.f;  // mean distance from base wall
        float raw_score = 0.f;  // segment_score (mean_dist × span_ratio), no proximity
        float prox  = 0.f;   // proximity weight applied to this zone
        float score = 0.f;   // raw_score × prox
    };
    std::vector<HotZoneSegment> hot_zones;  // up to 3, sorted best-first

    // Raw distance profiles (32 bins per side) for debugging.
    // dists[side][bin] = distance from nearest lidar point to that wall sample.
    std::array<std::array<float, 32>, 4> dist_profiles{};

    // Bayesian Reduction structural hypothesis:
    // - 10 Manhattan vertices total.
    // - First 4 are mandatory rectangle corners (high precision).
    // - Remaining 6 are latent structural vertices (start at zero precision).
    std::array<float, 10> vertex_precision = {1.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    int activated_vertex_a = -1; // latent vertex index activated by BMR
    int activated_vertex_b = -1; // latent vertex index activated by BMR

    int indent_side = -1;   // 0=bottom,1=right,2=top,3=left
    int indent_corner = -1; // 0=bottom-left,1=bottom-right,2=top-right,3=top-left
    float indent_a = 0.f;   // segment start (side-local coordinate)
    float indent_b = 0.f;   // segment end (side-local coordinate)
    float indent_depth = 0.f;   // legacy single depth (= indent_depth_x for corners)
    float indent_depth_x = 0.f;  // corner: indent along the horizontal side
    float indent_depth_y = 0.f;  // corner: indent along the vertical side
    float indent_score = 0.f;
    float current_indent_score = 0.f;
    float switch_log_bf_01 = 0.f;

    // Identity of the currently accepted indent hypothesis (if any).
    ProposalType current_proposal = ProposalType::NONE;
    int current_side = -1;
    int current_corner = -1;
    float current_depth = 0.f;
    float current_depth_x = 0.f;
    float current_depth_y = 0.f;

    bool expand = false;
    bool valid = false;
};

class RoomVFEBMR
{
public:
    float bmr_threshold = -2.0f;
    float bmr_saturation = 5.0f;  // clamp cumulative log-BF to ±this value
    float ext_prior_sigma_dim = 3.0f;
    float ext_prior_sigma_pos = 2.0f;
    // Prior std for the Laplace Occam factor — one per parameter type.
    // Occam = ½ log det(I + Σ₀ H)  where Σ₀ = diag(σ_d², σ_a², σ_b²).
    // Separate sigmas because depth (metres) and segment endpoints (room-scale) live on
    // different scales: conflating them biases the log-determinant Occam factor.
    float depth_prior_sigma = 0.5f;  // σ_d: depth in [0, half_room_extent], ~0.5 m
    float ab_prior_sigma    = 1.0f;  // σ_a,b: segment endpoints span room-scale, ~1 m

    /// End-to-end periodic BMR check from robot-frame lidar + current state.
    /// Returns nullopt when disabled or not due yet.
    std::optional<BmrResult> maybe_evaluate_from_lidar(
        const std::vector<Eigen::Vector3f> &points_robot_xyz,
        const Eigen::Matrix<float,5,1> &state,
        bool enable_bmr,
        int bmr_check_period,
        int bmr_min_frames_before_check,
        CandidateScorer scorer = {});

    void reset_cycle_counter();

    BmrResult evaluate_bmr_from_points(
        const std::vector<Eigen::Vector2f> &points_room_xy,
        float L1,
        float W1,
        float robot_x,
        float robot_y,
        float robot_phi,
        CandidateScorer scorer = {}) const;

private:
    struct IndentHypothesis
    {
        bool valid = false;
        BmrResult::ProposalType proposal = BmrResult::ProposalType::NONE;
        int side = -1;
        int corner = -1;
        float a = 0.f;
        float b = 0.f;
        float depth = 0.f;
        float depth_x = 0.f;
        float depth_y = 0.f;
        float score = 0.f;
        int activated_vertex_a = -1;
        int activated_vertex_b = -1;
    };

    int bmr_frame_counter_ = 0;
    float indent_log_bf_01_cumulative_ = 0.f;
    int indent_evidence_count_ = 0;
    float indent_switch_log_bf_01_cumulative_ = 0.f;
    int indent_switch_evidence_count_ = 0;
    int activated_vertex_count_ = 4; // starts with the mandatory rectangle vertices
    IndentHypothesis current_indent_hypothesis_;

    static float log_det_spd(const Eigen::Matrix3f &m);
    Eigen::Matrix3f make_ext_prior_cov() const;
};

} // namespace rc
