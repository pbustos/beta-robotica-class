#include "room_vfe_bmr.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace rc
{

namespace
{
constexpr int kNumMandatoryVertices = 4;
constexpr int kNumTotalVertices = 10;
constexpr int kMinIndentBins = 4;
constexpr int kNumIndentGuesses = 3;
constexpr int kCornerWindowBins = 5;

float segment_score_from_profile(const std::array<float, 32> &profile, int i0, int i1)
{
    const int nside = static_cast<int>(profile.size());
    i0 = std::clamp(i0, 0, nside - 1);
    i1 = std::clamp(i1, 0, nside - 1);
    if (i1 < i0)
        std::swap(i0, i1);
    const int len = i1 - i0 + 1;
    if (len < kMinIndentBins)
        return 0.f;

    float mean_d = 0.f;
    for (int i = i0; i <= i1; ++i)
        mean_d += profile[i];
    mean_d /= static_cast<float>(len);
    
    const float span_ratio = static_cast<float>(len) / static_cast<float>(nside);
    return mean_d * span_ratio;
}

std::pair<float, float> bins_to_segment_ab(int i0, int i1, int nside)
{
    const float t0 = -1.f + 2.f * static_cast<float>(i0) / static_cast<float>(nside - 1);
    const float t1 = -1.f + 2.f * static_cast<float>(i1) / static_cast<float>(nside - 1);
    return {std::min(t0, t1), std::max(t0, t1)};
}

std::pair<float, float> corner_profile_mean(const std::array<float, 32> &profile, bool at_low_end)
{
    const int n = static_cast<int>(profile.size());
    const int i0 = at_low_end ? 0 : std::max(0, n - kCornerWindowBins);
    const int i1 = at_low_end ? std::min(n - 1, kCornerWindowBins - 1) : (n - 1);

    float mean = 0.f;
    int cnt = 0;
    for (int i = i0; i <= i1; ++i)
    {
        mean += profile[i];
        ++cnt;
    }
    mean /= std::max(1, cnt);
    return {mean, static_cast<float>(cnt) / static_cast<float>(n)};
}

/// Build the indented polygon for a wall or corner candidate (room-frame vertices).
/// Returns an empty vector when parameters are degenerate.
std::vector<Eigen::Vector2f> build_indent_polygon(
    BmrResult::ProposalType proposal,
    int side, int corner,
    float a, float b, float depth,
    float hx, float hy,
    float depth_x = 0.f, float depth_y = 0.f)
{
    a     = std::clamp(std::min(a, b), -0.95f, 0.95f);
    b     = std::clamp(std::max(a, b), -0.95f, 0.95f);
    depth = std::clamp(depth, 0.1f, 0.8f * std::min(hx, hy));
    if (proposal == BmrResult::ProposalType::ADD_TWO_POINT_INDENT)
    {
        switch (side)
        {
            case 0: { const float x1 = a*hx, x2 = b*hx;
                return {{-hx,-hy},{x1,-hy},{x1,-hy+depth},{x2,-hy+depth},{x2,-hy},{hx,-hy},{hx,hy},{-hx,hy}}; }
            case 1: { const float y1 = a*hy, y2 = b*hy;
                return {{-hx,-hy},{hx,-hy},{hx,y1},{hx-depth,y1},{hx-depth,y2},{hx,y2},{hx,hy},{-hx,hy}}; }
            case 2: { const float x1 = a*hx, x2 = b*hx;
                return {{-hx,-hy},{hx,-hy},{hx,hy},{x2,hy},{x2,hy-depth},{x1,hy-depth},{x1,hy},{-hx,hy}}; }
            default: { const float y1 = a*hy, y2 = b*hy;
                return {{-hx,-hy},{hx,-hy},{hx,hy},{-hx,hy},{-hx,y2},{-hx+depth,y2},{-hx+depth,y1},{-hx,y1}}; }
        }
    }
    if (proposal == BmrResult::ProposalType::ADD_CORNER_INDENT)
    {
        // 2-DOF corner: depth_x (horizontal) and depth_y (vertical) are independent.
        // Fall back to the single 'depth' when both are zero (legacy callers).
        const float dx = (depth_x > 0.01f || depth_y > 0.01f)
            ? std::clamp(depth_x, 0.1f, 0.8f * hx)
            : depth;
        const float dy = (depth_x > 0.01f || depth_y > 0.01f)
            ? std::clamp(depth_y, 0.1f, 0.8f * hy)
            : depth;
        switch (corner)
        {
            case 0: return {{-hx+dx,-hy},{hx,-hy},{hx,hy},{-hx,hy},{-hx,-hy+dy},{-hx+dx,-hy+dy}};   // BL
            case 1: return {{-hx,-hy},{hx-dx,-hy},{hx-dx,-hy+dy},{hx,-hy+dy},{hx,hy},{-hx,hy}};      // BR
            case 2: return {{-hx,-hy},{hx,-hy},{hx,hy-dy},{hx-dx,hy-dy},{hx-dx,hy},{-hx,hy}};        // TR
            default: return {{-hx,-hy},{hx,-hy},{hx,hy},{-hx+dx,hy},{-hx+dx,hy-dy},{-hx,hy-dy}};     // TL
        }
    }
    return {};
}

} // namespace

void RoomVFEBMR::reset_cycle_counter()
{
    bmr_frame_counter_ = 0;
    indent_log_bf_01_cumulative_ = 0.f;
    indent_evidence_count_ = 0;
    indent_switch_log_bf_01_cumulative_ = 0.f;
    indent_switch_evidence_count_ = 0;
    activated_vertex_count_ = 4;
    current_indent_hypothesis_ = IndentHypothesis{};
}

std::optional<BmrResult> RoomVFEBMR::maybe_evaluate_from_lidar(
    const std::vector<Eigen::Vector3f> &points_robot_xyz,
    const Eigen::Matrix<float,5,1> &state,
    bool enable_bmr,
    int bmr_check_period,
    int bmr_min_frames_before_check,
    CandidateScorer scorer)
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
    auto out = evaluate_bmr_from_points(points_room_xy, L1, W1, x, y, phi, scorer);

    const bool indent_proposal =
        out.proposal == BmrResult::ProposalType::ADD_TWO_POINT_INDENT
        || out.proposal == BmrResult::ProposalType::ADD_CORNER_INDENT;

    // Cumulative posterior-odds update for the richer indent model family.
    if (indent_proposal)
    {
        // No current structural belief yet: accumulate evidence against null model.
        if (!current_indent_hypothesis_.valid)
        {
            indent_log_bf_01_cumulative_ += out.log_bf_01;
            indent_log_bf_01_cumulative_ = std::clamp(indent_log_bf_01_cumulative_,
                -bmr_saturation, bmr_saturation);
            ++indent_evidence_count_;
            out.log_bf_01 = indent_log_bf_01_cumulative_;
            out.switch_log_bf_01 = 0.f;

            out.expand = (indent_log_bf_01_cumulative_ < bmr_threshold);
            if (out.expand)
            {
                const int next_a = std::clamp(activated_vertex_count_, kNumMandatoryVertices, kNumTotalVertices - 2);
                const int next_b = next_a + 1;
                out.activated_vertex_a = next_a;
                out.activated_vertex_b = next_b;

                activated_vertex_count_ = std::min(kNumTotalVertices, activated_vertex_count_ + 2);
                for (int i = 0; i < activated_vertex_count_; ++i)
                    out.vertex_precision[i] = 1.f;

                current_indent_hypothesis_.valid = true;
                current_indent_hypothesis_.proposal = out.proposal;
                current_indent_hypothesis_.side = out.indent_side;
                current_indent_hypothesis_.corner = out.indent_corner;
                current_indent_hypothesis_.a = out.indent_a;
                current_indent_hypothesis_.b = out.indent_b;
                current_indent_hypothesis_.depth = out.indent_depth;
                current_indent_hypothesis_.depth_x = out.indent_depth_x;
                current_indent_hypothesis_.depth_y = out.indent_depth_y;
                current_indent_hypothesis_.score = out.indent_score;
                current_indent_hypothesis_.activated_vertex_a = next_a;
                current_indent_hypothesis_.activated_vertex_b = next_b;

                indent_log_bf_01_cumulative_ = 0.f;
                indent_evidence_count_ = 0;
                indent_switch_log_bf_01_cumulative_ = 0.f;
                indent_switch_evidence_count_ = 0;
            }
        }
        else
        {
            // Keep current activated-set belief and evaluate challenger switching odds.
            out.activated_vertex_a = current_indent_hypothesis_.activated_vertex_a;
            out.activated_vertex_b = current_indent_hypothesis_.activated_vertex_b;
            for (int i = 0; i < activated_vertex_count_; ++i)
                out.vertex_precision[i] = 1.f;

            // Expose current hypothesis identity for UI display.
            out.current_proposal = current_indent_hypothesis_.proposal;
            out.current_side     = current_indent_hypothesis_.side;
            out.current_corner   = current_indent_hypothesis_.corner;
            out.current_depth    = current_indent_hypothesis_.depth;
            out.current_depth_x  = current_indent_hypothesis_.depth_x;
            out.current_depth_y  = current_indent_hypothesis_.depth_y;

            // Positive delta means challenger explains data better (both VFE and heuristic modes).
            // VFE: scores = baseline−cand_vfe; −1×delta gives correct LLR sign (natural nats).
            // Heuristic: scores = profile goodness; −8×delta rescales to approximate LLR units.
            const float delta_score = out.indent_score - out.current_indent_score;
            const float llr_switch = scorer ? -delta_score : -8.0f * delta_score;

            const bool same_hypothesis =
                current_indent_hypothesis_.valid
                && current_indent_hypothesis_.proposal == out.proposal
                && current_indent_hypothesis_.side == out.indent_side
                && current_indent_hypothesis_.corner == out.indent_corner
                && std::abs(current_indent_hypothesis_.a - out.indent_a) < 0.10f
                && std::abs(current_indent_hypothesis_.b - out.indent_b) < 0.10f
                && std::abs(current_indent_hypothesis_.depth - out.indent_depth) < 0.15f;

            if (same_hypothesis)
            {
                indent_switch_log_bf_01_cumulative_ = 0.f;
                indent_switch_evidence_count_ = 0;
            }
            else
            {
                indent_switch_log_bf_01_cumulative_ += llr_switch;
                indent_switch_log_bf_01_cumulative_ = std::clamp(indent_switch_log_bf_01_cumulative_,
                    -bmr_saturation, bmr_saturation);
                ++indent_switch_evidence_count_;
            }

            out.switch_log_bf_01 = indent_switch_log_bf_01_cumulative_;
            out.log_bf_01 = indent_switch_log_bf_01_cumulative_;
            out.expand = (indent_switch_log_bf_01_cumulative_ < bmr_threshold);

            if (out.expand)
            {
                const int next_a = std::clamp(activated_vertex_count_, kNumMandatoryVertices, kNumTotalVertices - 2);
                const int next_b = next_a + 1;
                out.activated_vertex_a = next_a;
                out.activated_vertex_b = next_b;

                activated_vertex_count_ = std::min(kNumTotalVertices, activated_vertex_count_ + 2);
                for (int i = 0; i < activated_vertex_count_; ++i)
                    out.vertex_precision[i] = 1.f;

                current_indent_hypothesis_.valid = true;
                current_indent_hypothesis_.proposal = out.proposal;
                current_indent_hypothesis_.side = out.indent_side;
                current_indent_hypothesis_.corner = out.indent_corner;
                current_indent_hypothesis_.a = out.indent_a;
                current_indent_hypothesis_.b = out.indent_b;
                current_indent_hypothesis_.depth = out.indent_depth;
                current_indent_hypothesis_.depth_x = out.indent_depth_x;
                current_indent_hypothesis_.depth_y = out.indent_depth_y;
                current_indent_hypothesis_.score = out.indent_score;
                current_indent_hypothesis_.activated_vertex_a = next_a;
                current_indent_hypothesis_.activated_vertex_b = next_b;

                indent_switch_log_bf_01_cumulative_ = 0.f;
                indent_switch_evidence_count_ = 0;
            }
        }
    }
    else
    {
        // No indent proposal this frame. Instead of hard-resetting,
        // decay the accumulator toward zero so occasional "no data" frames
        // don't destroy all built-up evidence.
        constexpr float kDecayRate = 0.85f;  // retain 85% per empty frame
        indent_log_bf_01_cumulative_ *= kDecayRate;
        indent_switch_log_bf_01_cumulative_ *= kDecayRate;
    }

    return out;
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
    float W1,
    float robot_x,
    float robot_y,
    float robot_phi,
    CandidateScorer scorer) const
{
    BmrResult out;
    if (points_room_xy.empty()) return out;

    // Bayesian Reduction prior structure: 4 fixed corners + 6 latent vertices at zero precision.
    out.vertex_precision = {1.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    out.activated_vertex_a = -1;
    out.activated_vertex_b = -1;
    out.indent_corner = -1;

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
            // If no lidar point is within max_indent_dist of this wall sample,
            // it means the lidar doesn't see that region — treat as no data (0),
            // not as evidence of an indent.
            constexpr float kMaxIndentDist = 1.5f;
            dists[s][i] = (best <= kMaxIndentDist) ? best : 0.f;
        }
    }

    // Mask distance-profile bins that belong to the already-accepted indent.
    // After commitment the lidar conforms to the indented wall, but profiles
    // still measure against the base rectangle — creating ghost residuals that
    // block detection of new features on other walls.
    if (current_indent_hypothesis_.valid)
    {
        const auto& hyp = current_indent_hypothesis_;
        if (hyp.proposal == BmrResult::ProposalType::ADD_CORNER_INDENT
            && hyp.corner >= 0 && hyp.corner < 4)
        {
            const int cidx = hyp.corner;
            const bool low_a = (cidx == 0 || cidx == 3);
            const bool low_b = (cidx == 0 || cidx == 1);
            const int side_a = (cidx <= 1) ? 0 : 2;
            const int side_b = (cidx == 0 || cidx == 3) ? 3 : 1;

            // Extent in t-space ([-1,1]).  1.5× margin absorbs edge effects.
            const float ext_a = std::min(1.8f, 1.5f * hyp.depth_x / hx);
            const float ext_b = std::min(1.8f, 1.5f * hyp.depth_y / hy);

            for (int i = 0; i < nside; ++i)
            {
                const float t = -1.f + 2.f * static_cast<float>(i) / static_cast<float>(nside - 1);
                if (( low_a && t < -1.f + ext_a) ||
                    (!low_a && t >  1.f - ext_a))
                    dists[side_a][i] = 0.f;
                if (( low_b && t < -1.f + ext_b) ||
                    (!low_b && t >  1.f - ext_b))
                    dists[side_b][i] = 0.f;
            }
        }
        else if (hyp.proposal == BmrResult::ProposalType::ADD_TWO_POINT_INDENT
                 && hyp.side >= 0 && hyp.side < 4)
        {
            const float ta = std::min(hyp.a, hyp.b);
            const float tb = std::max(hyp.a, hyp.b);
            for (int i = 0; i < nside; ++i)
            {
                const float t = -1.f + 2.f * static_cast<float>(i) / static_cast<float>(nside - 1);
                if (t >= ta && t <= tb)
                    dists[hyp.side][i] = 0.f;
            }
        }
    }

    // Proximity + heading weighting for candidate scores.
    // Lidar noise grows with distance, inflating residuals on far walls.
    // Two multiplicative factors:
    //   1. Quartic distance decay:  w_d = 1 / (1 + (r/r_ref)⁴)
    //      r_ref = min(hx,hy) — the shorter room half-extent.
    //      At r_ref the weight is 0.5; at 1.5× r_ref it drops to ~0.16;
    //      at 2× r_ref ~0.06.  Quartic overcomes the d² noise growth
    //      that neutralised the earlier quadratic version.
    //   2. Forward-facing bonus:  w_f = 1 + 0.5 * max(0, cosα)
    //      where α is the angle between the robot heading and the
    //      direction to the zone.  Zones ahead of the robot get up to
    //      1.5× weight; zones behind are unpenalised (w_f = 1).
    const Eigen::Vector2f robot_pos(robot_x, robot_y);
    const Eigen::Vector2f robot_fwd(std::cos(robot_phi), std::sin(robot_phi));
    const float r_ref = std::min(hx, hy);
    const float r_ref2 = r_ref * r_ref;

    // Combined proximity·heading weight for a point in room frame.
    auto zone_weight = [&](const Eigen::Vector2f& pt) -> float
    {
        const Eigen::Vector2f d = pt - robot_pos;
        const float r2 = d.squaredNorm();
        const float ratio = r2 / r_ref2;
        const float w_dist = 1.f / (1.f + ratio * ratio);  // quartic decay
        const float cos_a = (r2 > 1e-4f) ? d.dot(robot_fwd) / std::sqrt(r2) : 0.f;
        const float w_fwd = 1.f + 0.5f * std::max(0.f, cos_a);  // [1.0, 1.5]
        return w_dist * w_fwd;
    };

    // Side proximity at the segment midpoint.
    auto side_proximity = [&](int side, float ta, float tb) -> float
    {
        const float tc = 0.5f * (ta + tb);
        return zone_weight(side_point(side, tc));
    };

    // Corner proximity.
    auto corner_proximity = [&](int cidx) -> float
    {
        const float cx = (cidx == 0 || cidx == 3) ? -hx : hx;
        const float cy = (cidx == 0 || cidx == 1) ? -hy : hy;
        return zone_weight(Eigen::Vector2f(cx, cy));
    };

    // Find strongest contiguous high-residual segments on all sides.
    const float min_depth = 0.18f;
    const int min_bins = kMinIndentBins;
    int best_side = -1;
    int best_i0 = -1;
    int best_i1 = -1;
    float best_score = 0.f;

    // Collect ALL qualifying segments across all sides for hot-zone debugging.
    // Ranked by raw profile score only (no proximity) so we see what the data says.
    struct HZSeg { int side; int i0; int i1; float mean_d; float raw; };
    std::vector<HZSeg> all_hz_segs;

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

                    const float raw = segment_score_from_profile(dists[s], start, end);
                    all_hz_segs.push_back({s, start, end, mean_d, raw});

                    // Best segment uses proximity for candidate selection only.
                    const auto [ta, tb] = bins_to_segment_ab(start, end, nside);
                    const float prox = side_proximity(s, ta, tb);
                    const float score = raw * prox;
                    if (score > best_score)
                    {
                        best_score = score;
                        best_side = s;
                        best_i0 = start;
                        best_i1 = end;
                    }
                }
            }
        }
    }

    // Expose top-3 hot-zone segments ranked by RAW score (no proximity).
    std::sort(all_hz_segs.begin(), all_hz_segs.end(),
        [](const HZSeg& a, const HZSeg& b){ return a.raw > b.raw; });
    {
        const int n_hz = std::min(static_cast<int>(all_hz_segs.size()), 3);
        out.hot_zones.resize(n_hz);
        for (int k = 0; k < n_hz; ++k)
        {
            const auto& hz = all_hz_segs[k];
            auto& o = out.hot_zones[k];
            o.side = hz.side;
            o.i0 = hz.i0;
            o.i1 = hz.i1;
            const auto [ta, tb] = bins_to_segment_ab(hz.i0, hz.i1, nside);
            o.a = ta;
            o.b = tb;
            o.mean_dist = hz.mean_d;
            o.raw_score = hz.raw;
            // Compute proximity for info only, not for ranking.
            o.prox = side_proximity(hz.side, ta, tb);
            o.score = hz.raw * o.prox;
        }
    }

    // Copy raw distance profiles for debug output.
    out.dist_profiles = dists;

    struct IndentCandidate
    {
        BmrResult::ProposalType proposal = BmrResult::ProposalType::NONE;
        int side = -1;
        int corner = -1;
        float a = 0.f;
        float b = 0.f;
        float depth = 0.f;     // wall depth; corner: legacy (= depth_x)
        float depth_x = 0.f;   // corner: indent along the horizontal side
        float depth_y = 0.f;   // corner: indent along the vertical side
        float score = 0.f;
        float raw_score = 0.f; // VFE score without proximity (for switch comparison)
    };

    IndentCandidate wall_best;
    std::vector<IndentCandidate> all_wall_cands;  // all 5 evaluated wall guesses

    // Build best wall-indent candidate.
    if (best_side >= 0)
    {
        // Systematic low-frequency exploration around the strongest hot-zone segment.
        // Only 3 guesses: the detected segment plus ±1-bin symmetric shifts.
        // Walls have 3 free params (more complex model) so we sample them more sparsely
        // than corners (2 free params), letting the Occam penalty handle complexity.
        struct Candidate { int i0; int i1; float score; float depth; };
        std::array<Candidate, kNumIndentGuesses> cands = {{
            {best_i0, best_i1, 0.f, 0.f},
            {best_i0 - 1, best_i1 + 1, 0.f, 0.f},
            {best_i0 + 1, best_i1 - 1, 0.f, 0.f}
        }};

        Candidate best_cand = cands[0];
        best_cand.score = -1.f;
        for (auto &c : cands)
        {
            c.i0 = std::clamp(c.i0, 0, nside - 1);
            c.i1 = std::clamp(c.i1, 0, nside - 1);
            if (c.i1 < c.i0)
                std::swap(c.i0, c.i1);
            if (c.i1 - c.i0 + 1 < min_bins)
                continue;

            const auto [ca, cb] = bins_to_segment_ab(c.i0, c.i1, nside);
            const float prox = side_proximity(best_side, ca, cb);
            c.score = segment_score_from_profile(dists[best_side], c.i0, c.i1) * prox;
            float mean_d = 0.f;
            for (int i = c.i0; i <= c.i1; ++i)
                mean_d += dists[best_side][i];
            c.depth = mean_d / static_cast<float>(c.i1 - c.i0 + 1);

            if (c.score > best_cand.score)
                best_cand = c;

            // Collect every valid wall guess for visualization.
            IndentCandidate wc;
            wc.proposal = BmrResult::ProposalType::ADD_TWO_POINT_INDENT;
            wc.side  = best_side;
            wc.corner = -1;
            wc.a     = ca;
            wc.b     = cb;
            wc.depth = std::clamp(c.depth, 0.1f, 0.8f * std::min(hx, hy));
            wc.score = std::max(0.f, c.score);
            all_wall_cands.push_back(wc);
        }

        const auto [a, b] = bins_to_segment_ab(best_cand.i0, best_cand.i1, nside);

        wall_best.proposal = BmrResult::ProposalType::ADD_TWO_POINT_INDENT;
        wall_best.side = best_side;
        wall_best.corner = -1;
        wall_best.a = a;
        wall_best.b = b;
        wall_best.depth = std::clamp(best_cand.depth, 0.1f, 0.8f * std::min(hx, hy));
        wall_best.score = std::max(0.f, best_cand.score);
    }

    // Build corner-indent candidates (all 4 corners explored each check).
    IndentCandidate corner_best;
    corner_best.score = -1.f;
    std::vector<IndentCandidate> all_corner_cands;  // all 4 evaluated corners
    for (int cidx = 0; cidx < 4; ++cidx)
    {
        // Corner mapping to adjacent side profile ends:
        // 0 BL: bottom low, left low
        // 1 BR: bottom high, right low
        // 2 TR: top high, right high
        // 3 TL: top low, left high
        const bool low_a = (cidx == 0 || cidx == 3);
        const bool low_b = (cidx == 0 || cidx == 1);
        const int side_a = (cidx <= 1) ? 0 : 2;
        const int side_b = (cidx == 0 || cidx == 3) ? 3 : 1;

        const auto [ma, ra] = corner_profile_mean(dists[side_a], low_a);
        const auto [mb, rb] = corner_profile_mean(dists[side_b], low_b);
        const float depth = 0.5f * (ma + mb);
        const float prox = corner_proximity(cidx);
        const float score = 0.6f * std::sqrt(std::max(0.f, ma * mb)) * (0.5f * (ra + rb)) * prox;

        // Independent per-side depths: side_a is the horizontal side, side_b the vertical side.
        // For corners 0(BL)/1(BR), side_a=bottom → dx along x, side_b=left/right → dy along y.
        // For corners 2(TR)/3(TL), side_a=top → dx along x, side_b=right/left → dy along y.
        const float seed_dx = std::clamp(ma, 0.1f, 0.8f * hx);
        const float seed_dy = std::clamp(mb, 0.1f, 0.8f * hy);

        IndentCandidate cc;
        cc.proposal = BmrResult::ProposalType::ADD_CORNER_INDENT;
        cc.corner   = cidx;
        cc.side     = -1;
        cc.depth    = std::clamp(depth, 0.1f, 0.8f * std::min(hx, hy));
        cc.depth_x  = seed_dx;
        cc.depth_y  = seed_dy;
        cc.score    = std::max(0.f, score);
        all_corner_cands.push_back(cc);

        if (score > corner_best.score)
            corner_best = cc;
    }

    // ===== VFE-BASED STRUCTURAL SCORING WITH LAPLACE OCCAM FACTOR =================
    // score = log p(data|template) - log p(data|rect)
    //       = [F_rect - F_map(theta*)] - Occam(template)
    //
    // Occam factor (Laplace approximation over free continuous parameters):
    //   For each free parameter theta_i with prior sigma_i:
    //     Occam_i = 0.5 * log(1 + sigma_i^2 * H_ii)
    //   where H_ii = d^2 F / d theta_i^2  at the MAP, estimated by central FD.
    //
    // Free parameters:
    //   Wall  (ADD_TWO_POINT_INDENT): depth(1) + segment endpoints a,b(2) = 3 params
    //   Corner (ADD_CORNER_INDENT):  depth(1)                              = 1 param
    //
    // Both wall and corner find MAP depth via a 5-point multiplicative depth search.
    // Wall additionally searches H_a and H_b via central FD scorer calls — no proxy.
    //
    // This correctly addresses all three Occam issues:
    //   1. Marginal likelihood (not MAP): Laplace integral over all free params.
    //   2. Free-parameter count: 3 (wall) vs 1 (corner), not vertex count.
    //   3. Prior-derived penalty: σ_d for depth, σ_a,b for segment endpoints (different units).
    //      Wall Occam uses the full 3×3 Hessian log-det (includes cross-terms);
    //      corner Occam is scalar (1 free param, no cross-terms).
    float baseline_vfe = 0.f;
    if (scorer)
    {
        baseline_vfe = scorer({{-hx,-hy},{hx,-hy},{hx,hy},{-hx,hy}});
        std::cerr << "[VFE] baseline_vfe=" << baseline_vfe
            << " N_pts=" << points_room_xy.size()
            << " hx=" << hx << " hy=" << hy << '\n';
        const float depth_sigma2 = depth_prior_sigma * depth_prior_sigma;
        const float ab_sigma2    = ab_prior_sigma    * ab_prior_sigma;
        // Finite-difference step for depth Hessian estimation.
        constexpr float kHfd = 0.05f;  // 5 cm

        // --- Wall: depth search for MAP; full 3×3 Laplace log-det Occam ---
        // θ = (d, a, b).  Σ₀ = diag(σ_d², σ_a², σ_b²).
        // Occam = ½ log det(I + Σ₀ H)  where H is the full 3×3 Hessian at the MAP.
        // Diagonal terms via 2-point central FD; cross-terms via 4-point mixed FD.
        if (!all_wall_cands.empty())
        {
            constexpr std::array<float, 3> kWallDepthMults = {0.6f, 1.0f, 1.6f};
            for (auto& wc : all_wall_cands)
            {
                const float d0 = wc.depth;

                // ---- MAP depth search ----
                struct DepthF { float d; float F; };
                std::vector<DepthF> dsamples;
                float best_F = std::numeric_limits<float>::infinity();
                float best_d = d0;
                for (const float mult : kWallDepthMults)
                {
                    const float td = std::clamp(d0 * mult, 0.1f, 0.8f * std::min(hx, hy));
                    const auto poly = build_indent_polygon(wc.proposal, wc.side, wc.corner, wc.a, wc.b, td, hx, hy);
                    if (poly.empty()) continue;
                    const float Fv = scorer(poly);
                    dsamples.push_back({td, Fv});
                    if (Fv < best_F) { best_F = Fv; best_d = td; }
                }
                wc.depth = best_d;
                const float F_map = best_F;
                if (F_map >= std::numeric_limits<float>::infinity()) { wc.score = -1e9f; continue; }

                // FD pivot offsets.
                const float dim   = (wc.side == 0 || wc.side == 2) ? hx : hy;
                const float kHseg = std::max(kHfd, 0.01f * dim);  // ~1% of side length
                const float dp_   = std::clamp(best_d + kHfd, 0.1f, 0.8f * std::min(hx, hy));
                const float dm_   = std::clamp(best_d - kHfd, 0.1f, 0.8f * std::min(hx, hy));
                const float hd    = (dp_ - dm_) * 0.5f;  // actual half-step (may differ from kHfd)
                const float ap_   = wc.a + kHseg;
                const float am_   = wc.a - kHseg;
                const float bp_   = wc.b + kHseg;
                const float bm_   = wc.b - kHseg;

                // Validity guards: segment must not be degenerate after perturbation.
                const bool a_ok  = (ap_ < wc.b && am_ < wc.b);
                const bool b_ok  = (bp_ > wc.a && bm_ > wc.a);
                const bool ab_ok = (ap_ < bm_);  // tightest constraint for simultaneous a+,b- shift

                // Scorer wrapper — returns F_map on degenerate or empty polygon.
                auto eval = [&](float d, float a, float b) -> float
                {
                    if (a >= b) return F_map;
                    const auto poly = build_indent_polygon(wc.proposal, wc.side, wc.corner, a, b, d, hx, hy);
                    return poly.empty() ? F_map : scorer(poly);
                };

                // ---- Diagonal Hessians ----
                // H_dd: reuse depth-search samples when MAP is bracketed; else direct FD.
                float H_dd = 0.f;
                {
                    std::sort(dsamples.begin(), dsamples.end(), [](const DepthF& x, const DepthF& y){ return x.d < y.d; });
                    for (std::size_t i = 1; i + 1 < dsamples.size(); ++i)
                    {
                        if (std::abs(dsamples[i].d - best_d) < 1e-4f)
                        {
                            const float h = 0.5f * (dsamples[i+1].d - dsamples[i-1].d);
                            if (h > 1e-4f)
                                H_dd = std::max(0.f, (dsamples[i+1].F - 2.f*dsamples[i].F + dsamples[i-1].F) / (h*h));
                            break;
                        }
                    }
                    if (H_dd == 0.f && hd > 1e-4f)
                        H_dd = std::max(0.f, (eval(dp_,wc.a,wc.b) - 2.f*F_map + eval(dm_,wc.a,wc.b)) / (hd*hd));
                }
                // H_aa, H_bb — also cache the perturbed F values for reuse in cross-terms.
                float Fa_p = F_map, Fa_m = F_map;
                float Fb_p = F_map, Fb_m = F_map;
                float H_aa = 0.f, H_bb = 0.f;
                if (a_ok)
                {
                    Fa_p = eval(best_d, ap_, wc.b);
                    Fa_m = eval(best_d, am_, wc.b);
                    H_aa = std::max(0.f, (Fa_p - 2.f*F_map + Fa_m) / (kHseg*kHseg));
                }
                if (b_ok)
                {
                    Fb_p = eval(best_d, wc.a, bp_);
                    Fb_m = eval(best_d, wc.a, bm_);
                    H_bb = std::max(0.f, (Fb_p - 2.f*F_map + Fb_m) / (kHseg*kHseg));
                }

                // ---- Off-diagonal Hessians via 4-point mixed FD ----
                // H_xy = (F(x+,y+) - F(x+,y-) - F(x-,y+) + F(x-,y-)) / (4·hx·hy)
                float H_da = 0.f, H_db = 0.f, H_ab = 0.f;
                if (hd > 1e-4f && a_ok)
                    H_da = (eval(dp_,ap_,wc.b) - eval(dp_,am_,wc.b)
                          - eval(dm_,ap_,wc.b) + eval(dm_,am_,wc.b)) / (4.f*hd*kHseg);
                if (hd > 1e-4f && b_ok)
                    H_db = (eval(dp_,wc.a,bp_) - eval(dp_,wc.a,bm_)
                          - eval(dm_,wc.a,bp_) + eval(dm_,wc.a,bm_)) / (4.f*hd*kHseg);
                if (ab_ok)
                    H_ab = (eval(best_d,ap_,bp_) - eval(best_d,ap_,bm_)
                          - eval(best_d,am_,bp_) + eval(best_d,am_,bm_)) / (4.f*kHseg*kHseg);

                // ---- Full 3×3 Occam = ½ log det(I + Σ₀ H) ----
                // (Σ₀ H)_ij = σ_row_i² · H_ij  since Σ₀ is diagonal.
                Eigen::Matrix3f M;
                M(0,0) = 1.f + depth_sigma2 * H_dd;   M(0,1) = depth_sigma2 * H_da;   M(0,2) = depth_sigma2 * H_db;
                M(1,0) = ab_sigma2    * H_da;          M(1,1) = 1.f + ab_sigma2 * H_aa; M(1,2) = ab_sigma2    * H_ab;
                M(2,0) = ab_sigma2    * H_db;          M(2,1) = ab_sigma2    * H_ab;    M(2,2) = 1.f + ab_sigma2 * H_bb;

                const float occam = 0.5f * std::log(std::max(1.f, M.determinant()));
                const float raw = (baseline_vfe - F_map) - occam;
                const float prox = side_proximity(wc.side, wc.a, wc.b);
                wc.raw_score = raw;
                wc.score = raw > 0.f ? raw * prox : raw;

                std::cerr << "[VFE-wall] side=" << wc.side
                    << " seg=[" << wc.a << "," << wc.b << "] d=" << wc.depth
                    << "  baseline=" << baseline_vfe << " F_map=" << F_map
                    << " fit=" << (baseline_vfe - F_map) << " occam=" << occam
                    << " raw=" << raw << " prox=" << prox << " score=" << wc.score << '\n';
            }
            wall_best = *std::max_element(
                all_wall_cands.begin(), all_wall_cands.end(),
                [](const IndentCandidate& lhs, const IndentCandidate& rhs)
                { return lhs.score < rhs.score; });
        }

        // --- Corner candidates: 2D grid search over (dx, dy); 2×2 Laplace Occam ---
        // θ = (dx, dy).  Σ₀ = diag(σ_d², σ_d²).
        // Occam = ½ log det(I₂ + Σ₀ H₂ₓ₂).
        if (!all_corner_cands.empty())
        {
            constexpr std::array<float, 5> kMults = {0.5f, 0.75f, 1.0f, 1.5f, 2.0f};
            for (auto& cc : all_corner_cands)
            {
                const float dx0 = cc.depth_x;
                const float dy0 = cc.depth_y;
                float best_F  = std::numeric_limits<float>::infinity();
                float best_dx = dx0;
                float best_dy = dy0;

                // 2D grid: 5×5 = 25 scorer evaluations.
                struct GridSample { float dx; float dy; float F; };
                std::vector<GridSample> samples;
                samples.reserve(25);
                for (const float mx : kMults)
                {
                    const float tdx = std::clamp(dx0 * mx, 0.1f, 0.8f * hx);
                    for (const float my : kMults)
                    {
                        const float tdy = std::clamp(dy0 * my, 0.1f, 0.8f * hy);
                        const auto poly = build_indent_polygon(
                            cc.proposal, cc.side, cc.corner, cc.a, cc.b, 0.f, hx, hy, tdx, tdy);
                        if (poly.empty()) continue;
                        const float Fv = scorer(poly);
                        samples.push_back({tdx, tdy, Fv});
                        if (Fv < best_F) { best_F = Fv; best_dx = tdx; best_dy = tdy; }
                    }
                }
                cc.depth_x = best_dx;
                cc.depth_y = best_dy;
                cc.depth   = 0.5f * (best_dx + best_dy);  // legacy compat
                const float F_map = best_F;
                if (F_map >= std::numeric_limits<float>::infinity()) { cc.score = -1e9f; continue; }

                // ---- 2×2 Hessian via central FD at the MAP ----
                auto eval_corner = [&](float dx, float dy) -> float
                {
                    const auto poly = build_indent_polygon(
                        cc.proposal, cc.side, cc.corner, cc.a, cc.b, 0.f, hx, hy, dx, dy);
                    return poly.empty() ? F_map : scorer(poly);
                };

                const float dx_p = std::clamp(best_dx + kHfd, 0.1f, 0.8f * hx);
                const float dx_m = std::clamp(best_dx - kHfd, 0.1f, 0.8f * hx);
                const float dy_p = std::clamp(best_dy + kHfd, 0.1f, 0.8f * hy);
                const float dy_m = std::clamp(best_dy - kHfd, 0.1f, 0.8f * hy);
                const float hx_half = (dx_p - dx_m) * 0.5f;
                const float hy_half = (dy_p - dy_m) * 0.5f;

                float H_xx = 0.f, H_yy = 0.f, H_xy = 0.f;
                if (hx_half > 1e-4f)
                    H_xx = std::max(0.f,
                        (eval_corner(dx_p, best_dy) - 2.f * F_map + eval_corner(dx_m, best_dy)) / (hx_half * hx_half));
                if (hy_half > 1e-4f)
                    H_yy = std::max(0.f,
                        (eval_corner(best_dx, dy_p) - 2.f * F_map + eval_corner(best_dx, dy_m)) / (hy_half * hy_half));
                if (hx_half > 1e-4f && hy_half > 1e-4f)
                    H_xy = (eval_corner(dx_p, dy_p) - eval_corner(dx_p, dy_m)
                          - eval_corner(dx_m, dy_p) + eval_corner(dx_m, dy_m))
                          / (4.f * hx_half * hy_half);

                // ---- Occam = ½ log det(I₂ + Σ₀ H₂ₓ₂) ----
                Eigen::Matrix2f M;
                M(0,0) = 1.f + depth_sigma2 * H_xx;   M(0,1) = depth_sigma2 * H_xy;
                M(1,0) = depth_sigma2 * H_xy;          M(1,1) = 1.f + depth_sigma2 * H_yy;
                const float occam = 0.5f * std::log(std::max(1.f, M.determinant()));
                const float raw = (baseline_vfe - F_map) - occam;
                const float prox = corner_proximity(cc.corner);
                cc.raw_score = raw;
                cc.score = raw > 0.f ? raw * prox : raw;
            }
            corner_best = *std::max_element(
                all_corner_cands.begin(), all_corner_cands.end(),
                [](const IndentCandidate& lhs, const IndentCandidate& rhs)
                { return lhs.score < rhs.score; });
        }

        // ===== ADAM REFINEMENT OF BEST CANDIDATES ==============================
        // After the coarse grid search, refine wall_best and corner_best using a
        // few steps of Adam (implemented with finite-difference gradients on the
        // scorer).  This squeezes additional VFE improvement that the discrete
        // grid may miss, producing more accurate MAP estimates and Occam factors.
        constexpr int   kAdamSteps  = 8;
        constexpr float kAdamLR     = 0.02f;
        constexpr float kAdamBeta1  = 0.9f;
        constexpr float kAdamBeta2  = 0.999f;
        constexpr float kAdamEps    = 1e-8f;
        constexpr float kGradH      = 0.02f;  // FD step for gradient estimation

        // Helper: one-parameter Adam state.
        struct AdamState1 { float m = 0.f; float v = 0.f; };

        // --- Refine wall_best (depth only; a,b are discrete bin positions) ---
        if (wall_best.proposal != BmrResult::ProposalType::NONE)
        {
            AdamState1 st;
            float d = wall_best.depth;
            float best_F = scorer(build_indent_polygon(
                wall_best.proposal, wall_best.side, wall_best.corner,
                wall_best.a, wall_best.b, d, hx, hy));

            for (int step = 0; step < kAdamSteps; ++step)
            {
                const float dp = std::clamp(d + kGradH, 0.1f, 0.8f * std::min(hx, hy));
                const float dm = std::clamp(d - kGradH, 0.1f, 0.8f * std::min(hx, hy));
                const float h2 = dp - dm;
                if (h2 < 1e-4f) break;
                const auto pp = build_indent_polygon(wall_best.proposal, wall_best.side, wall_best.corner,
                    wall_best.a, wall_best.b, dp, hx, hy);
                const auto pm = build_indent_polygon(wall_best.proposal, wall_best.side, wall_best.corner,
                    wall_best.a, wall_best.b, dm, hx, hy);
                if (pp.empty() || pm.empty()) break;
                const float grad = (scorer(pp) - scorer(pm)) / h2;  // dF/dd

                st.m = kAdamBeta1 * st.m + (1.f - kAdamBeta1) * grad;
                st.v = kAdamBeta2 * st.v + (1.f - kAdamBeta2) * grad * grad;
                const float mhat = st.m / (1.f - std::pow(kAdamBeta1, step + 1));
                const float vhat = st.v / (1.f - std::pow(kAdamBeta2, step + 1));
                d -= kAdamLR * mhat / (std::sqrt(vhat) + kAdamEps);
                d = std::clamp(d, 0.1f, 0.8f * std::min(hx, hy));

                const auto cur_poly = build_indent_polygon(wall_best.proposal, wall_best.side, wall_best.corner,
                    wall_best.a, wall_best.b, d, hx, hy);
                if (!cur_poly.empty())
                {
                    const float Fv = scorer(cur_poly);
                    if (Fv < best_F) { best_F = Fv; wall_best.depth = d; }
                }
            }
            // Recompute wall score and Occam with the refined depth.
            {
                const float F_map = best_F;
                const float dim   = (wall_best.side == 0 || wall_best.side == 2) ? hx : hy;
                const float kHseg = std::max(kHfd, 0.01f * dim);
                const float dp_   = std::clamp(wall_best.depth + kHfd, 0.1f, 0.8f * std::min(hx, hy));
                const float dm_   = std::clamp(wall_best.depth - kHfd, 0.1f, 0.8f * std::min(hx, hy));
                const float hd    = (dp_ - dm_) * 0.5f;
                auto eval = [&](float dd, float a, float b) -> float {
                    if (a >= b) return F_map;
                    const auto poly = build_indent_polygon(wall_best.proposal, wall_best.side, wall_best.corner, a, b, dd, hx, hy);
                    return poly.empty() ? F_map : scorer(poly);
                };
                float H_dd = 0.f;
                if (hd > 1e-4f)
                    H_dd = std::max(0.f, (eval(dp_, wall_best.a, wall_best.b) - 2.f*F_map + eval(dm_, wall_best.a, wall_best.b)) / (hd*hd));
                float H_aa = 0.f, H_bb = 0.f;
                const float ap_ = wall_best.a + kHseg, am_ = wall_best.a - kHseg;
                const float bp_ = wall_best.b + kHseg, bm_ = wall_best.b - kHseg;
                if (ap_ < wall_best.b && am_ < wall_best.b)
                    H_aa = std::max(0.f, (eval(wall_best.depth, ap_, wall_best.b) - 2.f*F_map + eval(wall_best.depth, am_, wall_best.b)) / (kHseg*kHseg));
                if (bp_ > wall_best.a && bm_ > wall_best.a)
                    H_bb = std::max(0.f, (eval(wall_best.depth, wall_best.a, bp_) - 2.f*F_map + eval(wall_best.depth, wall_best.a, bm_)) / (kHseg*kHseg));
                float H_da = 0.f, H_db = 0.f, H_ab = 0.f;
                if (hd > 1e-4f && ap_ < wall_best.b && am_ < wall_best.b)
                    H_da = (eval(dp_,ap_,wall_best.b) - eval(dp_,am_,wall_best.b)
                          - eval(dm_,ap_,wall_best.b) + eval(dm_,am_,wall_best.b)) / (4.f*hd*kHseg);
                if (hd > 1e-4f && bp_ > wall_best.a && bm_ > wall_best.a)
                    H_db = (eval(dp_,wall_best.a,bp_) - eval(dp_,wall_best.a,bm_)
                          - eval(dm_,wall_best.a,bp_) + eval(dm_,wall_best.a,bm_)) / (4.f*hd*kHseg);
                if (ap_ < bm_)
                    H_ab = (eval(wall_best.depth,ap_,bp_) - eval(wall_best.depth,ap_,bm_)
                          - eval(wall_best.depth,am_,bp_) + eval(wall_best.depth,am_,bm_)) / (4.f*kHseg*kHseg);

                Eigen::Matrix3f M;
                M(0,0) = 1.f + depth_sigma2*H_dd;  M(0,1) = depth_sigma2*H_da;      M(0,2) = depth_sigma2*H_db;
                M(1,0) = ab_sigma2*H_da;            M(1,1) = 1.f + ab_sigma2*H_aa;   M(1,2) = ab_sigma2*H_ab;
                M(2,0) = ab_sigma2*H_db;            M(2,1) = ab_sigma2*H_ab;          M(2,2) = 1.f + ab_sigma2*H_bb;
                const float occam = 0.5f * std::log(std::max(1.f, M.determinant()));
                const float raw = (baseline_vfe - F_map) - occam;
                const float prox = side_proximity(wall_best.side, wall_best.a, wall_best.b);
                wall_best.raw_score = raw;
                wall_best.score = raw > 0.f ? raw * prox : raw;
            }
        }

        // --- Refine corner_best (dx, dy) ---
        if (corner_best.proposal != BmrResult::ProposalType::NONE)
        {
            AdamState1 st_x, st_y;
            float dx = corner_best.depth_x;
            float dy = corner_best.depth_y;
            float best_F = scorer(build_indent_polygon(
                corner_best.proposal, corner_best.side, corner_best.corner,
                corner_best.a, corner_best.b, 0.f, hx, hy, dx, dy));

            for (int step = 0; step < kAdamSteps; ++step)
            {
                // gradient w.r.t. dx
                const float dxp = std::clamp(dx + kGradH, 0.1f, 0.8f * hx);
                const float dxm = std::clamp(dx - kGradH, 0.1f, 0.8f * hx);
                const float hx2 = dxp - dxm;
                // gradient w.r.t. dy
                const float dyp = std::clamp(dy + kGradH, 0.1f, 0.8f * hy);
                const float dym = std::clamp(dy - kGradH, 0.1f, 0.8f * hy);
                const float hy2 = dyp - dym;
                if (hx2 < 1e-4f || hy2 < 1e-4f) break;

                auto ev = [&](float ddx, float ddy) -> float {
                    const auto poly = build_indent_polygon(
                        corner_best.proposal, corner_best.side, corner_best.corner,
                        corner_best.a, corner_best.b, 0.f, hx, hy, ddx, ddy);
                    return poly.empty() ? best_F : scorer(poly);
                };
                const float gx = (ev(dxp, dy) - ev(dxm, dy)) / hx2;
                const float gy = (ev(dx, dyp) - ev(dx, dym)) / hy2;

                st_x.m = kAdamBeta1 * st_x.m + (1.f - kAdamBeta1) * gx;
                st_x.v = kAdamBeta2 * st_x.v + (1.f - kAdamBeta2) * gx * gx;
                st_y.m = kAdamBeta1 * st_y.m + (1.f - kAdamBeta1) * gy;
                st_y.v = kAdamBeta2 * st_y.v + (1.f - kAdamBeta2) * gy * gy;
                const float t_corr = static_cast<float>(step + 1);
                const float mx = st_x.m / (1.f - std::pow(kAdamBeta1, t_corr));
                const float vx = st_x.v / (1.f - std::pow(kAdamBeta2, t_corr));
                const float my = st_y.m / (1.f - std::pow(kAdamBeta1, t_corr));
                const float vy = st_y.v / (1.f - std::pow(kAdamBeta2, t_corr));

                dx -= kAdamLR * mx / (std::sqrt(vx) + kAdamEps);
                dy -= kAdamLR * my / (std::sqrt(vy) + kAdamEps);
                dx = std::clamp(dx, 0.1f, 0.8f * hx);
                dy = std::clamp(dy, 0.1f, 0.8f * hy);

                const float Fv = ev(dx, dy);
                if (Fv < best_F) { best_F = Fv; corner_best.depth_x = dx; corner_best.depth_y = dy; }
            }
            corner_best.depth = 0.5f * (corner_best.depth_x + corner_best.depth_y);
            // Recompute corner score and Occam with refined depths.
            {
                const float F_map = best_F;
                auto eval_corner = [&](float ddx, float ddy) -> float {
                    const auto poly = build_indent_polygon(
                        corner_best.proposal, corner_best.side, corner_best.corner,
                        corner_best.a, corner_best.b, 0.f, hx, hy, ddx, ddy);
                    return poly.empty() ? F_map : scorer(poly);
                };
                const float dx_p = std::clamp(corner_best.depth_x + kHfd, 0.1f, 0.8f * hx);
                const float dx_m = std::clamp(corner_best.depth_x - kHfd, 0.1f, 0.8f * hx);
                const float dy_p = std::clamp(corner_best.depth_y + kHfd, 0.1f, 0.8f * hy);
                const float dy_m = std::clamp(corner_best.depth_y - kHfd, 0.1f, 0.8f * hy);
                const float hx_half = (dx_p - dx_m) * 0.5f;
                const float hy_half = (dy_p - dy_m) * 0.5f;
                float H_xx = 0.f, H_yy = 0.f, H_xy = 0.f;
                if (hx_half > 1e-4f)
                    H_xx = std::max(0.f, (eval_corner(dx_p, corner_best.depth_y) - 2.f*F_map + eval_corner(dx_m, corner_best.depth_y)) / (hx_half*hx_half));
                if (hy_half > 1e-4f)
                    H_yy = std::max(0.f, (eval_corner(corner_best.depth_x, dy_p) - 2.f*F_map + eval_corner(corner_best.depth_x, dy_m)) / (hy_half*hy_half));
                if (hx_half > 1e-4f && hy_half > 1e-4f)
                    H_xy = (eval_corner(dx_p, dy_p) - eval_corner(dx_p, dy_m)
                          - eval_corner(dx_m, dy_p) + eval_corner(dx_m, dy_m)) / (4.f*hx_half*hy_half);
                Eigen::Matrix2f M;
                M(0,0) = 1.f + depth_sigma2*H_xx;  M(0,1) = depth_sigma2*H_xy;
                M(1,0) = depth_sigma2*H_xy;         M(1,1) = 1.f + depth_sigma2*H_yy;
                const float occam = 0.5f * std::log(std::max(1.f, M.determinant()));
                const float raw = (baseline_vfe - F_map) - occam;
                const float prox = corner_proximity(corner_best.corner);
                corner_best.raw_score = raw;
                corner_best.score = raw > 0.f ? raw * prox : raw;

                std::cerr << "[VFE-corner] c=" << corner_best.corner
                    << " dx=" << corner_best.depth_x << " dy=" << corner_best.depth_y
                    << "  baseline=" << baseline_vfe << " F_map=" << F_map
                    << " fit=" << (baseline_vfe - F_map) << " occam=" << occam
                    << " raw=" << raw << " prox=" << prox << " score=" << corner_best.score << '\n';
            }
        }
    }

    // Select best challenger among wall/corner indents.
    // Only propose if the best candidate has a positive score (beats the rectangle).
    IndentCandidate chosen;
    if (wall_best.proposal != BmrResult::ProposalType::NONE
        && wall_best.score > 0.f && wall_best.score >= corner_best.score)
        chosen = wall_best;
    else if (corner_best.score > 0.f)
        chosen = corner_best;
    // else: chosen stays NONE — no indent beats the rectangle.

    // Expose all evaluated candidates for visualization.
    // Order: wall guesses (up to 5, sorted best-first) then all 4 corners (sorted best-first).
    {
        // Sort wall candidates by score descending.
        std::sort(all_wall_cands.begin(), all_wall_cands.end(),
            [](const IndentCandidate& x, const IndentCandidate& y){ return x.score > y.score; });
        // Sort corner candidates by score descending.
        std::sort(all_corner_cands.begin(), all_corner_cands.end(),
            [](const IndentCandidate& x, const IndentCandidate& y){ return x.score > y.score; });

        const bool wall_chosen   = (chosen.proposal == BmrResult::ProposalType::ADD_TWO_POINT_INDENT);
        const bool corner_chosen = (chosen.proposal == BmrResult::ProposalType::ADD_CORNER_INDENT);

        for (std::size_t i = 0; i < all_wall_cands.size(); ++i)
        {
            const auto& wc = all_wall_cands[i];
            BmrResult::IndentCandidateInfo info;
            info.proposal  = wc.proposal;
            info.side      = wc.side;
            info.corner    = -1;
            info.a         = wc.a;
            info.b         = wc.b;
            info.depth     = wc.depth;
            info.score     = wc.score;
            info.raw_score = wc.raw_score;
            info.is_chosen = (wall_chosen && i == 0);  // only the best wall guess can be chosen
            out.all_candidates.push_back(info);
        }
        for (std::size_t i = 0; i < all_corner_cands.size(); ++i)
        {
            const auto& cc = all_corner_cands[i];
            BmrResult::IndentCandidateInfo info;
            info.proposal  = cc.proposal;
            info.side      = -1;
            info.corner    = cc.corner;
            info.a         = 0.f;
            info.b         = 0.f;
            info.depth     = cc.depth;
            info.depth_x   = cc.depth_x;
            info.depth_y   = cc.depth_y;
            info.score     = cc.score;
            info.raw_score = cc.raw_score;
            info.is_chosen = (corner_chosen && i == 0);  // only the best corner can be chosen
            out.all_candidates.push_back(info);
        }
    }

    // Convert selected evidence into BMR-style decision.
    if (chosen.proposal != BmrResult::ProposalType::NONE)
    {
        out.indent_score = chosen.raw_score;

        out.current_indent_score = 0.f;
        if (scorer && current_indent_hypothesis_.valid)
        {
            // VFE: rebuild current hypothesis polygon and score it for switch-log-BF comparison
            const auto cur_poly = build_indent_polygon(
                current_indent_hypothesis_.proposal, current_indent_hypothesis_.side,
                current_indent_hypothesis_.corner, current_indent_hypothesis_.a,
                current_indent_hypothesis_.b, current_indent_hypothesis_.depth, hx, hy,
                current_indent_hypothesis_.depth_x, current_indent_hypothesis_.depth_y);
            out.current_indent_score = cur_poly.empty() ? 0.f : (baseline_vfe - scorer(cur_poly));
        }
        else if (current_indent_hypothesis_.valid)
        {
            if (current_indent_hypothesis_.proposal == BmrResult::ProposalType::ADD_TWO_POINT_INDENT
                && current_indent_hypothesis_.side >= 0 && current_indent_hypothesis_.side < 4)
            {
                const float ta = std::clamp(current_indent_hypothesis_.a, -1.f, 1.f);
                const float tb = std::clamp(current_indent_hypothesis_.b, -1.f, 1.f);
                int ci0 = static_cast<int>(std::round((ta + 1.f) * 0.5f * static_cast<float>(nside - 1)));
                int ci1 = static_cast<int>(std::round((tb + 1.f) * 0.5f * static_cast<float>(nside - 1)));
                if (ci1 < ci0)
                    std::swap(ci0, ci1);
                out.current_indent_score = segment_score_from_profile(
                    dists[current_indent_hypothesis_.side], ci0, ci1);
            }
            else if (current_indent_hypothesis_.proposal == BmrResult::ProposalType::ADD_CORNER_INDENT
                     && current_indent_hypothesis_.corner >= 0 && current_indent_hypothesis_.corner < 4)
            {
                const int cidx = current_indent_hypothesis_.corner;
                const bool low_a = (cidx == 0 || cidx == 3);
                const bool low_b = (cidx == 0 || cidx == 1);
                const int side_a = (cidx <= 1) ? 0 : 2;
                const int side_b = (cidx == 0 || cidx == 3) ? 3 : 1;
                const auto [ma, ra] = corner_profile_mean(dists[side_a], low_a);
                const auto [mb, rb] = corner_profile_mean(dists[side_b], low_b);
                out.current_indent_score = 0.6f * std::sqrt(std::max(0.f, ma * mb)) * (0.5f * (ra + rb));
            }
        }

        // VFE: log_bf_01 = cand_vfe − baseline_vfe = −(baseline_vfe − cand_vfe) = −indent_score
        // Heuristic: log_bf_01 = −8 × indent_score
        out.log_bf_01 = scorer ? -out.indent_score : -8.0f * out.indent_score;
        out.posterior_z_sq = chosen.depth * chosen.depth;
        out.logdet_shrinkage = out.indent_score;
        out.proposal = chosen.proposal;
        out.indent_side = chosen.side;
        out.indent_corner = chosen.corner;
        out.indent_a = chosen.a;
        out.indent_b = chosen.b;
        out.indent_depth = chosen.depth;
        out.indent_depth_x = chosen.depth_x;
        out.indent_depth_y = chosen.depth_y;
        out.expand = false;

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
