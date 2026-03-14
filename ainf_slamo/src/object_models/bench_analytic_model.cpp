#include "bench_analytic_model.h"
#include "../object_geometry.h"

#include <algorithm>

namespace rc::object_models
{
namespace
{
torch::Tensor to_row_major_xyz(const torch::Tensor& pts)
{
    if (pts.dim() != 2 || pts.size(1) != 3)
        throw std::runtime_error("BenchAnalyticModel expects points tensor with shape [N,3]");
    return pts.to(torch::kFloat32).contiguous();
}

torch::Tensor normalize_params(const torch::Tensor& p)
{
    auto params = p.to(torch::kFloat32).contiguous();
    if (params.dim() != 1 || params.size(0) != BenchAnalyticModel::kParamSize)
        throw std::runtime_error("BenchAnalyticModel expects params tensor with shape [6]");
    return params;
}

torch::Tensor sdf_box_local(const torch::Tensor& local_xyz, const torch::Tensor& half_extents)
{
    const auto q       = torch::abs(local_xyz) - half_extents.unsqueeze(0);
    const auto outside = torch::norm(torch::relu(q), 2, 1);
    const auto qmax    = std::get<0>(torch::max(q, 1));
    const auto inside  = torch::minimum(qmax, torch::zeros_like(qmax));
    return outside + inside;
}

torch::Tensor sdf_oriented_box(const torch::Tensor& pts,
                               const torch::Tensor& tx, const torch::Tensor& ty,
                               const torch::Tensor& yaw, const torch::Tensor& cz,
                               const torch::Tensor& half_extents)
{
    auto x  = pts.index({torch::indexing::Slice(), 0}) - tx;
    auto y  = pts.index({torch::indexing::Slice(), 1}) - ty;
    auto z  = pts.index({torch::indexing::Slice(), 2}) - cz;
    const auto c  = torch::cos(yaw);
    const auto s  = torch::sin(yaw);
    const auto xl = c * x + s * y;
    const auto yl = -s * x + c * y;
    return sdf_box_local(torch::stack({xl, yl, z}, 1), half_extents);
}
} // namespace

BenchAnalyticModel::BenchAnalyticModel(const torch::Tensor& initial_params)
{
    const auto p = normalize_params(initial_params);
    state_ = torch::zeros({kParamSize}, torch::kFloat32);
    state_[0] = p[0]; state_[1] = p[1]; state_[2] = p[2];
    state_[3] = torch::log(torch::clamp_min(p[3], 1e-4f));
    state_[4] = torch::log(torch::clamp_min(p[4], 1e-4f));
    state_[5] = torch::log(torch::clamp_min(p[5], 1e-4f));
}

torch::Tensor BenchAnalyticModel::forward_sdf(const torch::Tensor& points_xyz,
                                              const torch::Tensor& params_6)
{
    const auto pts    = to_row_major_xyz(points_xyz);
    const auto p      = normalize_params(params_6);

    const auto tx     = p[0];
    const auto ty     = p[1];
    const auto yaw    = p[2];
    const auto width  = torch::clamp_min(p[3], 1e-4f);
    const auto depth  = torch::clamp_min(p[4], 1e-4f);
    const auto height = torch::clamp_min(p[5], 1e-4f);

    namespace G = rc::geometry::bench;
    const auto seat_t  = torch::clamp_min(G::seat_thickness_ratio * height, G::seat_thickness_min);
    const auto seat_h  = G::seat_height_ratio * height;
    const auto back_t  = torch::clamp_min(G::backrest_thickness_ratio * depth, G::backrest_thickness_min);
    const auto back_h  = torch::clamp_min(height - seat_h - seat_t, G::backrest_height_min);
    const auto leg_t   = torch::clamp_min(G::support_thickness_ratio * width, G::support_thickness_min);

    const auto c = torch::cos(yaw);
    const auto s = torch::sin(yaw);

    // ---- 1. Seat slab ----
    const auto seat_cz   = seat_h + seat_t * 0.5f;
    const auto seat_half = torch::stack({width * 0.5f, depth * 0.5f, seat_t * 0.5f});
    auto sdf = sdf_oriented_box(pts, tx, ty, yaw, seat_cz, seat_half);

    // ---- 2. Backrest at rear (-Y local) edge ----
    const auto bdy      = -(depth * 0.5f - back_t * 0.5f);
    const auto back_tx  = tx + (-s) * bdy;
    const auto back_ty  = ty + c * bdy;
    const auto back_cz  = seat_h + seat_t + back_h * 0.5f;
    const auto back_half = torch::stack({width * 0.5f, back_t * 0.5f, back_h * 0.5f});
    sdf = torch::minimum(sdf,
        sdf_oriented_box(pts, back_tx, back_ty, yaw, back_cz, back_half));

    // ---- 3. Two stout side supports at ±X ends ----
    const auto sup_half = torch::stack({leg_t * 0.5f, depth * G::support_depth_ratio, seat_h * 0.5f});
    const auto sup_cz   = seat_h * 0.5f;
    const auto dx       = width * G::support_spread;

    auto add_support = [&](const torch::Tensor& sign)
    {
        const auto lx     = sign * dx;
        const auto sup_tx = tx + c * lx;
        const auto sup_ty = ty + s * lx;
        sdf = torch::minimum(sdf,
            sdf_oriented_box(pts, sup_tx, sup_ty, yaw, sup_cz, sup_half));
    };

    add_support(torch::tensor(1.f,  p.options()));
    add_support(torch::tensor(-1.f, p.options()));

    return sdf;
}

torch::Tensor BenchAnalyticModel::forward(const torch::Tensor& points_xyz) const
{
    auto p = torch::zeros({kParamSize}, state_.options());
    p[0] = state_[0]; p[1] = state_[1]; p[2] = state_[2];
    p[3] = torch::exp(state_[3]); p[4] = torch::exp(state_[4]); p[5] = torch::exp(state_[5]);
    return forward_sdf(points_xyz, p);
}

torch::Tensor BenchAnalyticModel::score_points(const torch::Tensor& points_xyz, float beta) const
{
    return torch::exp(-std::max(1e-3f, beta) * torch::abs(forward(points_xyz)));
}

BenchAnalyticModel::FitResult BenchAnalyticModel::fit_autograd(const torch::Tensor& points_xyz,
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
        p[3] = torch::exp(s[3]); p[4] = torch::exp(s[4]); p[5] = torch::exp(s[5]);
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

torch::Tensor BenchAnalyticModel::params() const
{
    auto p = torch::zeros({kParamSize}, torch::TensorOptions().dtype(torch::kFloat32));
    const auto s = state_.detach().to(torch::kCPU);
    p[0] = s[0]; p[1] = s[1]; p[2] = s[2];
    p[3] = torch::exp(s[3]); p[4] = torch::exp(s[4]); p[5] = torch::exp(s[5]);
    return p;
}

} // namespace rc::object_models
