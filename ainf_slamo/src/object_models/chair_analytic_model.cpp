#include "chair_analytic_model.h"
#include "../object_geometry.h"

#include <algorithm>

namespace rc::object_models
{
namespace
{
torch::Tensor to_row_major_xyz(const torch::Tensor& points_xyz)
{
    auto pts = points_xyz;
    if (pts.dim() != 2 || pts.size(1) != 3)
        throw std::runtime_error("ChairAnalyticModel expects points tensor with shape [N,3]");
    return pts.to(torch::kFloat32).contiguous();
}

torch::Tensor normalize_params(const torch::Tensor& p)
{
    auto params = p.to(torch::kFloat32).contiguous();
    if (params.dim() != 1 || params.size(0) != ChairAnalyticModel::kParamSize)
        throw std::runtime_error("ChairAnalyticModel expects params tensor with shape [6]");
    return params;
}

// Smooth box SDF in local coordinates.
torch::Tensor sdf_box_local(const torch::Tensor& local_xyz, const torch::Tensor& half_extents)
{
    const auto q       = torch::abs(local_xyz) - half_extents.unsqueeze(0);
    const auto outside = torch::norm(torch::relu(q), 2, 1);
    const auto qmax    = std::get<0>(torch::max(q, 1));
    const auto inside  = torch::minimum(qmax, torch::zeros_like(qmax));
    return outside + inside;
}

// Axis-aligned box with pose (tx, ty, yaw) and vertical centre cz.
torch::Tensor sdf_oriented_box(const torch::Tensor& pts,
                               const torch::Tensor& tx,
                               const torch::Tensor& ty,
                               const torch::Tensor& yaw,
                               const torch::Tensor& cz,
                               const torch::Tensor& half_extents)
{
    auto x = pts.index({torch::indexing::Slice(), 0}) - tx;
    auto y = pts.index({torch::indexing::Slice(), 1}) - ty;
    auto z = pts.index({torch::indexing::Slice(), 2}) - cz;

    const auto c  = torch::cos(yaw);
    const auto s  = torch::sin(yaw);
    const auto xl = c * x + s * y;
    const auto yl = -s * x + c * y;

    return sdf_box_local(torch::stack({xl, yl, z}, 1), half_extents);
}

} // namespace

ChairAnalyticModel::ChairAnalyticModel(const torch::Tensor& initial_params)
{
    const auto p = normalize_params(initial_params);
    state_ = torch::zeros({kParamSize}, torch::kFloat32);
    state_[0] = p[0];
    state_[1] = p[1];
    state_[2] = p[2];
    state_[3] = torch::log(torch::clamp_min(p[3], 1e-4f));
    state_[4] = torch::log(torch::clamp_min(p[4], 1e-4f));
    state_[5] = torch::log(torch::clamp_min(p[5], 1e-4f));
}

torch::Tensor ChairAnalyticModel::forward_sdf(const torch::Tensor& points_xyz,
                                              const torch::Tensor& params_6)
{
    const auto pts = to_row_major_xyz(points_xyz);
    const auto p   = normalize_params(params_6);

    const auto tx          = p[0];
    const auto ty          = p[1];
    const auto yaw         = p[2];
    const auto width       = torch::clamp_min(p[3], 1e-4f);
    const auto depth       = torch::clamp_min(p[4], 1e-4f);
    const auto seat_height = torch::clamp_min(p[5], 1e-4f);

    // ---- Derived dimensions ----
    namespace G = rc::geometry::chair;
    const auto seat_t  = torch::clamp_min(G::seat_thickness_ratio * seat_height, G::seat_thickness_min);
    const auto back_h  = torch::clamp_min(G::backrest_height_ratio * seat_height, G::backrest_height_min);
    const auto back_t  = torch::clamp_min(G::backrest_thickness_ratio * depth, G::backrest_thickness_min);
    const auto leg_sq  = torch::clamp_min(G::leg_size_ratio * torch::minimum(width, depth), G::leg_size_min);
    const auto leg_h   = torch::clamp_min(seat_height - seat_t, G::leg_height_min);

    // ---- 1. Seat slab ----
    const auto seat_cz   = seat_height - seat_t * 0.5f;
    const auto seat_half = torch::stack({width * 0.5f, depth * 0.5f, seat_t * 0.5f});
    auto sdf = sdf_oriented_box(pts, tx, ty, yaw, seat_cz, seat_half);

    // ---- 2. Backrest at rear (-Y local) edge ----
    const auto c   = torch::cos(yaw);
    const auto s   = torch::sin(yaw);
    const auto bdy = -(depth * 0.5f - back_t * 0.5f);   // local -Y offset (rear)
    const auto back_tx   = tx + (-s) * bdy;
    const auto back_ty   = ty + c * bdy;
    const auto back_cz   = seat_height + back_h * 0.5f;
    const auto back_half = torch::stack({width * 0.5f, back_t * 0.5f, back_h * 0.5f});
    sdf = torch::minimum(sdf, sdf_oriented_box(pts, back_tx, back_ty, yaw, back_cz, back_half));

    // ---- 3. Four legs ----
    const auto leg_half = torch::stack({leg_sq * 0.5f, leg_sq * 0.5f, leg_h * 0.5f});
    const auto leg_cz   = leg_h * 0.5f;
    const auto dx       = width * 0.5f - leg_sq * 0.5f;
    const auto dy       = depth * 0.5f - leg_sq * 0.5f;

    auto add_leg = [&](const torch::Tensor& sx, const torch::Tensor& sy)
    {
        const auto lx     = sx * dx;
        const auto ly     = sy * dy;
        const auto leg_tx = tx + c * lx - s * ly;
        const auto leg_ty = ty + s * lx + c * ly;
        sdf = torch::minimum(sdf, sdf_oriented_box(pts, leg_tx, leg_ty, yaw, leg_cz, leg_half));
    };

    add_leg(torch::tensor(1.f,  p.options()), torch::tensor(1.f,  p.options()));
    add_leg(torch::tensor(1.f,  p.options()), torch::tensor(-1.f, p.options()));
    add_leg(torch::tensor(-1.f, p.options()), torch::tensor(1.f,  p.options()));
    add_leg(torch::tensor(-1.f, p.options()), torch::tensor(-1.f, p.options()));

    return sdf;
}

torch::Tensor ChairAnalyticModel::forward(const torch::Tensor& points_xyz) const
{
    auto p = torch::zeros({kParamSize}, state_.options());
    p[0] = state_[0]; p[1] = state_[1]; p[2] = state_[2];
    p[3] = torch::exp(state_[3]); p[4] = torch::exp(state_[4]); p[5] = torch::exp(state_[5]);
    return forward_sdf(points_xyz, p);
}

torch::Tensor ChairAnalyticModel::score_points(const torch::Tensor& points_xyz, float beta) const
{
    const auto sdf = forward(points_xyz);
    const float b  = std::max(1e-3f, beta);
    return torch::exp(-b * torch::abs(sdf));
}

ChairAnalyticModel::FitResult ChairAnalyticModel::fit_autograd(const torch::Tensor& points_xyz,
                                                               int   max_iterations,
                                                               float learning_rate)
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
    const int iters = std::max(1, max_iterations);
    float final_loss = out.initial_loss;
    for (int i = 0; i < iters; ++i)
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

torch::Tensor ChairAnalyticModel::params() const
{
    auto p = torch::zeros({kParamSize}, torch::TensorOptions().dtype(torch::kFloat32));
    const auto s = state_.detach().to(torch::kCPU);
    p[0] = s[0]; p[1] = s[1]; p[2] = s[2];
    p[3] = torch::exp(s[3]); p[4] = torch::exp(s[4]); p[5] = torch::exp(s[5]);
    return p;
}

} // namespace rc::object_models
