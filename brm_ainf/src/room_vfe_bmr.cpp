#include "room_vfe_bmr.h"

#include <algorithm>
#include <cmath>

namespace rc
{

std::optional<BmrResult> RoomVFEBMR::maybe_evaluate_from_lidar(
    const std::vector<Eigen::Vector3f> &points_robot_xyz,
    const Eigen::Matrix<float,5,1> &state,
    bool enable_bmr,
    int bmr_check_period,
    int bmr_min_frames_before_check)
{
    if (!enable_bmr)
        return std::nullopt;

    ++bmr_frame_counter_;
    const bool do_check = bmr_frame_counter_ >= bmr_min_frames_before_check
        && (bmr_frame_counter_ % std::max(1, bmr_check_period) == 0);
    if (!do_check)
        return std::nullopt;

    const float x = state[2];
    const float y = state[3];
    const float phi = state[4];
    const float c = std::cos(phi);
    const float s = std::sin(phi);

    std::vector<Eigen::Vector2f> points_room_xy;
    points_room_xy.reserve(points_robot_xyz.size());
    for (const auto &p : points_robot_xyz)
    {
        const float qx = c * p.x() - s * p.y() + x;
        const float qy = s * p.x() + c * p.y() + y;
        points_room_xy.emplace_back(qx, qy);
    }

    const float L1 = std::max(0.5f, state[0]);
    const float W1 = std::max(0.5f, state[1]);
    return evaluate_bmr_from_points(points_room_xy, L1, W1);
}

float RoomVFEBMR::log_det_spd(const Eigen::Matrix3f &m)
{
    const Eigen::Matrix3f sym = 0.5f * (m + m.transpose()) + 1e-6f * Eigen::Matrix3f::Identity();
    Eigen::LLT<Eigen::Matrix3f> llt(sym);
    if (llt.info() == Eigen::Success)
    {
        const auto L = llt.matrixL();
        return 2.f * std::log(std::max(1e-10f, L(0, 0)))
             + 2.f * std::log(std::max(1e-10f, L(1, 1)))
             + 2.f * std::log(std::max(1e-10f, L(2, 2)));
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(sym);
    float out = 0.f;
    for (int i = 0; i < 3; ++i)
        out += std::log(std::max(es.eigenvalues()[i], 1e-10f));
    return out;
}

Eigen::Matrix3f RoomVFEBMR::make_ext_prior_cov() const
{
    Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
    S(0, 0) = ext_prior_sigma_dim * ext_prior_sigma_dim;
    S(1, 1) = ext_prior_sigma_dim * ext_prior_sigma_dim;
    S(2, 2) = ext_prior_sigma_pos * ext_prior_sigma_pos;
    return S;
}

BmrResult RoomVFEBMR::evaluate_bmr_from_points(
    const std::vector<Eigen::Vector2f> &points_room_xy,
    float L1,
    float W1) const
{
    BmrResult out;
    if (points_room_xy.empty()) return out;

    // Bayesian Reduction prior structure: 4 fixed corners + 6 latent vertices at zero precision.
    out.vertex_precision = {1.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    out.activated_vertex_a = -1;
    out.activated_vertex_b = -1;

    const float hx = std::max(0.25f, 0.5f * L1);
    const float hy = std::max(0.25f, 0.5f * W1);

    // Build tensors as plain vectors: side-local nearest distance profiles.
    constexpr int nside = 32;
    std::array<std::array<float, nside>, 4> dists{}; // 0=bottom,1=right,2=top,3=left
    for (auto &side : dists)
        side.fill(1e6f);

    auto side_point = [hx, hy](int side, float t) -> Eigen::Vector2f
    {
        t = std::clamp(t, -1.f, 1.f);
        switch (side)
        {
            case 0: return {t * hx, -hy};
            case 1: return {hx, t * hy};
            case 2: return {t * hx, hy};
            default: return {-hx, t * hy};
        }
    };

    for (int s = 0; s < 4; ++s)
    {
        for (int i = 0; i < nside; ++i)
        {
            const float t = -1.f + 2.f * static_cast<float>(i) / static_cast<float>(nside - 1);
            const auto w = side_point(s, t);
            float best = 1e6f;
            for (const auto &p : points_room_xy)
                best = std::min(best, (p - w).norm());
            dists[s][i] = best;
        }
    }

    // Find strongest contiguous high-residual segment on any side.
    const float min_depth = 0.18f;
    const int min_bins = 4;
    int best_side = -1;
    int best_i0 = -1;
    int best_i1 = -1;
    float best_score = 0.f;
    float best_depth = 0.f;

    for (int s = 0; s < 4; ++s)
    {
        int i = 0;
        while (i < nside)
        {
            while (i < nside && dists[s][i] <= min_depth) ++i;
            const int start = i;
            while (i < nside && dists[s][i] > min_depth) ++i;
            const int end = i - 1;

            if (end >= start)
            {
                const int len = end - start + 1;
                if (len >= min_bins)
                {
                    float mean_d = 0.f;
                    for (int k = start; k <= end; ++k) mean_d += dists[s][k];
                    mean_d /= static_cast<float>(len);

                    const float span_ratio = static_cast<float>(len) / static_cast<float>(nside);
                    const float score = mean_d * span_ratio;
                    if (score > best_score)
                    {
                        best_score = score;
                        best_side = s;
                        best_i0 = start;
                        best_i1 = end;
                        best_depth = mean_d;
                    }
                }
            }
        }
    }

    // Convert residual segment evidence into BMR-style decision.
    if (best_side >= 0)
    {
        const float t0 = -1.f + 2.f * static_cast<float>(best_i0) / static_cast<float>(nside - 1);
        const float t1 = -1.f + 2.f * static_cast<float>(best_i1) / static_cast<float>(nside - 1);
        const float a = std::min(t0, t1);
        const float b = std::max(t0, t1);

        // Heuristic evidence score -> log-BF surrogate (more negative => more support for richer model).
        out.log_bf_01 = -8.0f * best_score;
        out.posterior_z_sq = best_depth * best_depth;
        out.logdet_shrinkage = best_score;
        out.proposal = BmrResult::ProposalType::ADD_TWO_POINT_INDENT;
        out.indent_side = best_side;
        out.indent_a = a;
        out.indent_b = b;
        out.indent_depth = std::clamp(best_depth, 0.1f, 0.8f * std::min(hx, hy));
        out.expand = out.log_bf_01 < bmr_threshold;

        // Activate exactly two latent vertices when evidence supports richer structure.
        // Latent pool indices: [4..9], six vertices initially at zero precision.
        if (out.expand)
        {
            out.activated_vertex_a = 4;
            out.activated_vertex_b = 5;
            out.vertex_precision[4] = 1.f;
            out.vertex_precision[5] = 1.f;
        }

        out.valid = true;
        return out;
    }

    // Estimate extension evidence from points beyond the base top/right boundaries.
    // Origin is assumed fixed at the stable corner (0,0).
    float sum_dx = 0.f;
    float sum_dy = 0.f;
    float sum_s = 0.f;
    float sum_dx2 = 0.f;
    float sum_dy2 = 0.f;
    float sum_s2 = 0.f;
    int count = 0;

    for (const auto &p : points_room_xy)
    {
        const float dx = std::max(0.f, p.x() - L1);
        const float dy = std::max(0.f, p.y() - W1);
        if (dx > 0.f || dy > 0.f)
        {
            const float s = std::clamp(p.y(), 0.f, W1);
            sum_dx += dx;
            sum_dy += dy;
            sum_s += s;
            sum_dx2 += dx * dx;
            sum_dy2 += dy * dy;
            sum_s2 += s * s;
            ++count;
        }
    }

    if (count < 20)
    {
        out.valid = true;
        out.expand = false;
        return out;
    }

    const float inv_n = 1.f / static_cast<float>(count);
    const float mu_L2 = sum_dx * inv_n;
    const float mu_W2 = sum_dy * inv_n;
    const float mu_s = sum_s * inv_n;
    out.ext_post_mean = Eigen::Vector3f(mu_L2, mu_W2, mu_s);

    const float var_L2 = std::max(1e-4f, sum_dx2 * inv_n - mu_L2 * mu_L2);
    const float var_W2 = std::max(1e-4f, sum_dy2 * inv_n - mu_W2 * mu_W2);
    const float var_s = std::max(1e-3f, sum_s2 * inv_n - mu_s * mu_s);
    out.ext_post_cov = Eigen::Matrix3f::Zero();
    out.ext_post_cov(0, 0) = var_L2;
    out.ext_post_cov(1, 1) = var_W2;
    out.ext_post_cov(2, 2) = var_s;

    const Eigen::Matrix3f S_prior = make_ext_prior_cov();
    const Eigen::Matrix3f S_post = 0.5f * (out.ext_post_cov + out.ext_post_cov.transpose()) + 1e-6f * Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f Pi_post = S_post.inverse();

    out.posterior_z_sq = out.ext_post_mean.transpose() * Pi_post * out.ext_post_mean;
    out.logdet_shrinkage = 0.5f * (log_det_spd(S_prior) - log_det_spd(S_post));
    out.log_bf_01 = -0.5f * out.posterior_z_sq + out.logdet_shrinkage;

    // Expand only with both evidence and physically meaningful ext dimensions.
    out.expand = (out.log_bf_01 < bmr_threshold) && (mu_L2 > 0.2f || mu_W2 > 0.2f);
    out.valid = true;
    return out;
}

} // namespace rc
