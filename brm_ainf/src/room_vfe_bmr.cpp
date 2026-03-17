#include "room_vfe_bmr.h"

#include <algorithm>
#include <cmath>

namespace rc
{

namespace
{
constexpr int kNumMandatoryVertices = 4;
constexpr int kNumTotalVertices = 10;
constexpr int kMinIndentBins = 4;
constexpr int kNumIndentGuesses = 5;
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
    float hx, float hy)
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
        switch (corner)
        {
            case 0: return {{-hx+depth,-hy},{hx,-hy},{hx,hy},{-hx,hy},{-hx,-hy+depth},{-hx+depth,-hy+depth}};
            case 1: return {{-hx,-hy},{hx-depth,-hy},{hx-depth,-hy+depth},{hx,-hy+depth},{hx,hy},{-hx,hy}};
            case 2: return {{-hx,-hy},{hx,-hy},{hx,hy-depth},{hx-depth,hy-depth},{hx-depth,hy},{-hx,hy}};
            default: return {{-hx,-hy},{hx,-hy},{hx,hy},{-hx+depth,hy},{-hx+depth,hy-depth},{-hx,hy-depth}};
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
    auto out = evaluate_bmr_from_points(points_room_xy, L1, W1, scorer);

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
        indent_log_bf_01_cumulative_ = 0.f;
        indent_evidence_count_ = 0;
        indent_switch_log_bf_01_cumulative_ = 0.f;
        indent_switch_evidence_count_ = 0;
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
            dists[s][i] = best;
        }
    }

    // Find strongest contiguous high-residual segment on any side.
    const float min_depth = 0.18f;
    const int min_bins = kMinIndentBins;
    int best_side = -1;
    int best_i0 = -1;
    int best_i1 = -1;
    float best_score = 0.f;

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

                    const float score = segment_score_from_profile(dists[s], start, end);
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

    struct IndentCandidate
    {
        BmrResult::ProposalType proposal = BmrResult::ProposalType::NONE;
        int side = -1;
        int corner = -1;
        float a = 0.f;
        float b = 0.f;
        float depth = 0.f;
        float score = 0.f;
    };

    IndentCandidate wall_best;
    std::vector<IndentCandidate> all_wall_cands;  // all 5 evaluated wall guesses

    // Build best wall-indent candidate.
    if (best_side >= 0)
    {
        // Systematic low-frequency exploration around the strongest hot-zone segment.
        struct Candidate { int i0; int i1; float score; float depth; };
        std::array<Candidate, kNumIndentGuesses> cands = {{
            {best_i0, best_i1, 0.f, 0.f},
            {best_i0 - 1, best_i1 - 1, 0.f, 0.f},
            {best_i0 + 1, best_i1 + 1, 0.f, 0.f},
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

            c.score = segment_score_from_profile(dists[best_side], c.i0, c.i1);
            float mean_d = 0.f;
            for (int i = c.i0; i <= c.i1; ++i)
                mean_d += dists[best_side][i];
            c.depth = mean_d / static_cast<float>(c.i1 - c.i0 + 1);

            if (c.score > best_cand.score)
                best_cand = c;

            // Collect every valid wall guess for visualization.
            const auto [ca, cb] = bins_to_segment_ab(c.i0, c.i1, nside);
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
        const float score = 0.6f * std::sqrt(std::max(0.f, ma * mb)) * (0.5f * (ra + rb));

        IndentCandidate cc;
        cc.proposal = BmrResult::ProposalType::ADD_CORNER_INDENT;
        cc.corner   = cidx;
        cc.side     = -1;
        cc.depth    = std::clamp(depth, 0.1f, 0.8f * std::min(hx, hy));
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
            constexpr std::array<float, 5> kWallDepthMults = {0.5f, 0.75f, 1.0f, 1.5f, 2.0f};
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
                wc.score = (baseline_vfe - F_map) - occam;
            }
            wall_best = *std::max_element(
                all_wall_cands.begin(), all_wall_cands.end(),
                [](const IndentCandidate& lhs, const IndentCandidate& rhs)
                { return lhs.score < rhs.score; });
        }

        // --- Corner candidates: depth search to find MAP; then Laplace Occam over 1 param ---
        if (!all_corner_cands.empty())
        {
            constexpr std::array<float, 5> kDepthMults = {0.5f, 0.75f, 1.0f, 1.5f, 2.0f};
            for (auto& cc : all_corner_cands)
            {
                const float d0 = cc.depth;
                float best_F = std::numeric_limits<float>::infinity();
                float best_d = d0;
                // Collect (depth, F) pairs for curvature estimation.
                struct DepthF { float d; float F; };
                std::vector<DepthF> samples;
                for (const float mult : kDepthMults)
                {
                    const float td = std::clamp(d0 * mult, 0.1f, 0.8f * std::min(hx, hy));
                    const auto poly = build_indent_polygon(
                        cc.proposal, cc.side, cc.corner, cc.a, cc.b, td, hx, hy);
                    if (poly.empty()) continue;
                    const float Fv = scorer(poly);
                    samples.push_back({td, Fv});
                    if (Fv < best_F) { best_F = Fv; best_d = td; }
                }
                cc.depth = best_d;

                // Estimate H_depth from the three samples nearest to best_d (central FD if available).
                float H_depth = 0.f;
                if (samples.size() >= 3)
                {
                    // Sort by depth and find bracketing samples around best_d.
                    std::sort(samples.begin(), samples.end(), [](const DepthF& x, const DepthF& y){ return x.d < y.d; });
                    for (std::size_t i = 1; i + 1 < samples.size(); ++i)
                    {
                        if (samples[i].d == best_d || std::abs(samples[i].d - best_d) < 1e-4f)
                        {
                            const float h = 0.5f * (samples[i+1].d - samples[i-1].d);
                            if (h > 1e-4f)
                                H_depth = std::max(0.f,
                                    (samples[i+1].F - 2.f*samples[i].F + samples[i-1].F) / (h*h));
                            break;
                        }
                    }
                }

                // Laplace Occam: 1 free param (depth only).
                const float occam = 0.5f * std::log(1.f + depth_sigma2 * H_depth);
                cc.score = (baseline_vfe - best_F) - occam;
            }
            corner_best = *std::max_element(
                all_corner_cands.begin(), all_corner_cands.end(),
                [](const IndentCandidate& lhs, const IndentCandidate& rhs)
                { return lhs.score < rhs.score; });
        }
    }

    // Select best challenger among wall/corner indents.
    IndentCandidate chosen;
    if (wall_best.proposal != BmrResult::ProposalType::NONE && wall_best.score >= corner_best.score)
        chosen = wall_best;
    else
        chosen = corner_best;

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
            info.score     = cc.score;
            info.is_chosen = (corner_chosen && i == 0);  // only the best corner can be chosen
            out.all_candidates.push_back(info);
        }
    }

    // Convert selected evidence into BMR-style decision.
    if (chosen.proposal != BmrResult::ProposalType::NONE)
    {
        out.indent_score = chosen.score;

        out.current_indent_score = 0.f;
        if (scorer && current_indent_hypothesis_.valid)
        {
            // VFE: rebuild current hypothesis polygon and score it for switch-log-BF comparison
            const auto cur_poly = build_indent_polygon(
                current_indent_hypothesis_.proposal, current_indent_hypothesis_.side,
                current_indent_hypothesis_.corner, current_indent_hypothesis_.a,
                current_indent_hypothesis_.b, current_indent_hypothesis_.depth, hx, hy);
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
