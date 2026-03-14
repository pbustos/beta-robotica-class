#include "pot_analytic_model.h"
#include "../object_geometry.h"

#include <algorithm>

namespace rc::object_models
{
namespace
{
torch::Tensor to_row_major_xyz(const torch::Tensor& pts)
{
    if (pts.dim() != 2 || pts.size(1) != 3)
        throw std::runtime_error("PotAnalyticModel expects points tensor with shape [N,3]");
    return pts.to(torch::kFloat32).contiguous();
}

torch::Tensor normalize_params(const torch::Tensor& p)
{
    auto params = p.to(torch::kFloat32).contiguous();
    if (params.dim() != 1 || params.size(0) != PotAnalyticModel::kParamSize)
        throw std::runtime_error("PotAnalyticModel expects params tensor with shape [5]");
    return params;
}
} // namespace

PotAnalyticModel::PotAnalyticModel(const torch::Tensor& initial_params)
{
    const auto p = normalize_params(initial_params);
    state_ = torch::zeros({kParamSize}, torch::kFloat32);
    state_[0] = p[0]; state_[1] = p[1]; state_[2] = p[2];
    state_[3] = torch::log(torch::clamp_min(p[3], 1e-4f));  // log_r
    state_[4] = torch::log(torch::clamp_min(p[4], 1e-4f));  // log_h
}

torch::Tensor PotAnalyticModel::forward_sdf(const torch::Tensor& points_xyz,
                                            const torch::Tensor& params_5)
{
    const auto pts    = to_row_major_xyz(points_xyz);
    const auto p      = normalize_params(params_5);

    const auto tx     = p[0];
    const auto ty     = p[1];
    const auto yaw    = p[2];
    const auto radius = torch::clamp_min(p[3], 1e-4f);   // top (larger) radius
    const auto height = torch::clamp_min(p[4], 1e-4f);

    // Rotate & translate into local frame
    auto x = pts.index({torch::indexing::Slice(), 0}) - tx;
    auto y = pts.index({torch::indexing::Slice(), 1}) - ty;
    auto z = pts.index({torch::indexing::Slice(), 2});

    const auto c  = torch::cos(yaw);
    const auto s  = torch::sin(yaw);
    const auto xl = c * x + s * y;
    const auto yl = -s * x + c * y;

    // ---- Exact SDF for a capped cone (truncated cone / frustum) ----
    // Reference: Inigo Quilez, "distance functions",
    //   https://iquilezles.org/articles/distfunctions/
    //   sdCappedCone(p, h, r1, r2)
    //
    // r1 = bottom radius at z=0, r2 = top radius at z=h.
    // Works in the 2D (q.x = radial distance, q.y = z) half-plane.
    const auto r1 = radius * rc::geometry::pot::taper_ratio;   // bottom (smaller) radius
    const auto r2 = radius;                                   // top (larger) radius
    const auto h  = height;

    const auto qr = torch::sqrt(xl * xl + yl * yl);   // radial distance [N]
    const auto qz = z;                                 // axial coordinate [N]

    // k1 = (r2, h),  k2 = (r2 - r1, 2h)
    const auto k1x = r2;
    const auto k1y = h;
    const auto k2x = r2 - r1;
    const auto k2y = 2.0f * h;

    // ca = closest point on cap edges
    // For the bottom cap (qz < 0 side): clamp to r1; for top cap: clamp to r2.
    const auto cap_r = torch::where(qz < 0.0f, torch::clamp_max(qr, r1),
                                                torch::clamp_max(qr, r2));
    const auto ca_x = qr - cap_r;
    const auto ca_y = torch::abs(qz) - h;

    // cb = closest point on the slanted lateral surface (as a line segment
    // from (r1, 0) to (r2, h) projected in the (r, z) half-plane).
    // t = clamp( dot(k1 - q, k2) / dot(k2, k2), 0, 1 )
    const auto dot_num = k1x * (k1x - qr) + k1y * (k1y - qz);   // broadcast over [N]
    const auto dot_den = k2x * k2x + k2y * k2y;                  // scalar
    const auto t       = torch::clamp(dot_num / dot_den, 0.0f, 1.0f);  // [N]
    const auto cb_x    = qr - k1x + k2x * t;     // [N]
    const auto cb_y    = qz - k1y + k2y * t;     // [N]

    // sign: negative inside (both ca and cb components must be < 0)
    const auto s_sign = torch::where((cb_x < 0.0f) & (ca_y < 0.0f),
                                     torch::full_like(qr, -1.0f),
                                     torch::full_like(qr, 1.0f));

    return s_sign * torch::sqrt(
        torch::minimum(ca_x * ca_x + ca_y * ca_y,
                       cb_x * cb_x + cb_y * cb_y));
}

torch::Tensor PotAnalyticModel::forward(const torch::Tensor& points_xyz) const
{
    auto p = torch::zeros({kParamSize}, state_.options());
    p[0] = state_[0]; p[1] = state_[1]; p[2] = state_[2];
    p[3] = torch::exp(state_[3]); p[4] = torch::exp(state_[4]);
    return forward_sdf(points_xyz, p);
}

torch::Tensor PotAnalyticModel::score_points(const torch::Tensor& points_xyz, float beta) const
{
    return torch::exp(-std::max(1e-3f, beta) * torch::abs(forward(points_xyz)));
}

PotAnalyticModel::FitResult PotAnalyticModel::fit_autograd(const torch::Tensor& points_xyz,
                                                           int max_iterations, float learning_rate)
{
    FitResult out;
    auto state = state_.clone().detach();
    state.set_requires_grad(true);
    torch::optim::Adam optimizer({state}, torch::optim::AdamOptions(std::max(1e-5f, learning_rate)));

    const auto pts = to_row_major_xyz(points_xyz);
    auto eval_loss = [&](const torch::Tensor& s) -> torch::Tensor {
        auto p = torch::zeros({kParamSize}, s.options());
        p[0] = s[0]; p[1] = s[1]; p[2] = s[2];
        p[3] = torch::exp(s[3]); p[4] = torch::exp(s[4]);
        return torch::mean(torch::square(forward_sdf(pts, p)));
    };

    out.initial_loss = eval_loss(state).item<float>();
    float final_loss = out.initial_loss;
    for (int i = 0; i < std::max(1, max_iterations); ++i)
    {
        optimizer.zero_grad();
        const auto loss = eval_loss(state);
        loss.backward();
        optimizer.step();
        final_loss = loss.item<float>();
        out.iterations = i + 1;
        if (!std::isfinite(final_loss)) break;
    }
    state_ = state.detach();
    out.ok = std::isfinite(final_loss);
    out.final_loss = final_loss;
    out.params = params();
    return out;
}

torch::Tensor PotAnalyticModel::params() const
{
    auto p = torch::zeros({kParamSize}, torch::TensorOptions().dtype(torch::kFloat32));
    const auto s = state_.detach().to(torch::kCPU);
    p[0] = s[0]; p[1] = s[1]; p[2] = s[2];
    p[3] = torch::exp(s[3]); p[4] = torch::exp(s[4]);
    return p;
}

} // namespace rc::object_models
