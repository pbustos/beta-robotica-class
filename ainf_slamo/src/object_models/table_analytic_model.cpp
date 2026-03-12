#include "table_analytic_model.h"

#include <algorithm>

namespace rc::object_models
{
namespace
{
torch::Tensor to_row_major_xyz(const torch::Tensor& points_xyz)
{
    auto pts = points_xyz;
    if (pts.dim() != 2 || pts.size(1) != 3)
        throw std::runtime_error("TableAnalyticModel expects points tensor with shape [N,3]");
    return pts.to(torch::kFloat32).contiguous();
}

torch::Tensor normalize_params(const torch::Tensor& p)
{
    auto params = p.to(torch::kFloat32).contiguous();
    if (params.dim() != 1 || params.size(0) != TableAnalyticModel::kParamSize)
        throw std::runtime_error("TableAnalyticModel expects params tensor with shape [6]");
    return params;
}

// 3D box SDF in local coordinates, shape [N,3] and half extents [3].
torch::Tensor sdf_box_local(const torch::Tensor& local_xyz, const torch::Tensor& half_extents)
{
    const auto q = torch::abs(local_xyz) - half_extents.unsqueeze(0);  // [N,3]
    const auto outside = torch::norm(torch::relu(q), 2, 1);            // [N]
    const auto qmax = std::get<0>(torch::max(q, 1));                   // [N]
    const auto inside = torch::minimum(qmax, torch::zeros_like(qmax)); // [N]
    return outside + inside;
}

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

    const auto c = torch::cos(yaw);
    const auto s = torch::sin(yaw);

    // Rotate around vertical axis by -yaw to align with local box axes.
    const auto xl = c * x + s * y;
    const auto yl = -s * x + c * y;

    const auto local_xyz = torch::stack({xl, yl, z}, 1); // [N,3]
    return sdf_box_local(local_xyz, half_extents);
}

} // namespace

TableAnalyticModel::TableAnalyticModel(const torch::Tensor& initial_params)
{
    const auto p = normalize_params(initial_params);

    // Store dimensions in log space to keep them positive during optimization.
    state_ = torch::zeros({kParamSize}, torch::kFloat32);
    state_[0] = p[0];
    state_[1] = p[1];
    state_[2] = p[2];
    state_[3] = torch::log(torch::clamp_min(p[3], 1e-4f));
    state_[4] = torch::log(torch::clamp_min(p[4], 1e-4f));
    state_[5] = torch::log(torch::clamp_min(p[5], 1e-4f));
}

torch::Tensor TableAnalyticModel::forward_sdf(const torch::Tensor& points_xyz,
                                              const torch::Tensor& params_6)
{
    const auto pts = to_row_major_xyz(points_xyz);
    const auto p = normalize_params(params_6);

    const auto tx = p[0];
    const auto ty = p[1];
    const auto yaw = p[2];
    const auto width = torch::clamp_min(p[3], 1e-4f);
    const auto depth = torch::clamp_min(p[4], 1e-4f);
    const auto height = torch::clamp_min(p[5], 1e-4f);

    // Composed table SDF: tabletop cuboid + 4 leg cuboids (union via min).
    // All parts share the same (tx,ty,yaw) and sit on z=0 floor.

    const auto top_thickness = torch::clamp_min(0.08f * height, 0.03f);
    const auto leg_thickness = torch::clamp_min(0.12f * torch::minimum(width, depth), 0.03f);
    const auto leg_height = torch::clamp_min(height - top_thickness, 0.05f);

    // Tabletop center and half extents.
    const auto top_cz = height - 0.5f * top_thickness;
    const auto top_half = torch::stack({0.5f * width, 0.5f * depth, 0.5f * top_thickness});
    auto sdf = sdf_oriented_box(pts, tx, ty, yaw, top_cz, top_half);

    // Leg centers in the table local XY frame (near corners).
    const auto leg_hx = 0.5f * leg_thickness;
    const auto leg_hy = 0.5f * leg_thickness;
    const auto leg_hz = 0.5f * leg_height;
    const auto leg_cz = 0.5f * leg_height;
    const auto leg_half = torch::stack({leg_hx, leg_hy, leg_hz});

    const auto dx = 0.5f * width - leg_hx;
    const auto dy = 0.5f * depth - leg_hy;
    const auto c = torch::cos(yaw);
    const auto s = torch::sin(yaw);

    auto add_leg = [&](const torch::Tensor& sx, const torch::Tensor& sy)
    {
        const auto lx = sx * dx;
        const auto ly = sy * dy;
        const auto leg_tx = tx + c * lx - s * ly;
        const auto leg_ty = ty + s * lx + c * ly;
        const auto leg_sdf = sdf_oriented_box(pts, leg_tx, leg_ty, yaw, leg_cz, leg_half);
        sdf = torch::minimum(sdf, leg_sdf);
    };

    add_leg(torch::tensor(1.f, p.options()),  torch::tensor(1.f, p.options()));
    add_leg(torch::tensor(1.f, p.options()),  torch::tensor(-1.f, p.options()));
    add_leg(torch::tensor(-1.f, p.options()), torch::tensor(1.f, p.options()));
    add_leg(torch::tensor(-1.f, p.options()), torch::tensor(-1.f, p.options()));

    return sdf;
}

torch::Tensor TableAnalyticModel::forward(const torch::Tensor& points_xyz) const
{
    auto p = torch::zeros({kParamSize}, state_.options());
    p[0] = state_[0];
    p[1] = state_[1];
    p[2] = state_[2];
    p[3] = torch::exp(state_[3]);
    p[4] = torch::exp(state_[4]);
    p[5] = torch::exp(state_[5]);
    return forward_sdf(points_xyz, p);
}

torch::Tensor TableAnalyticModel::score_points(const torch::Tensor& points_xyz, float beta) const
{
    const auto sdf = forward(points_xyz);
    const float b = std::max(1e-3f, beta);
    return torch::exp(-b * torch::abs(sdf));
}

TableAnalyticModel::FitResult TableAnalyticModel::fit_autograd(const torch::Tensor& points_xyz,
                                                               int max_iterations,
                                                               float learning_rate)
{
    FitResult out;

    auto state = state_.clone().detach();
    state.set_requires_grad(true);

    torch::optim::Adam optimizer({state}, torch::optim::AdamOptions(std::max(1e-5f, learning_rate)));

    const auto pts = to_row_major_xyz(points_xyz);

    auto eval_loss = [&](const torch::Tensor& s) -> torch::Tensor
    {
        auto p = torch::zeros({kParamSize}, s.options());
        p[0] = s[0];
        p[1] = s[1];
        p[2] = s[2];
        p[3] = torch::exp(s[3]);
        p[4] = torch::exp(s[4]);
        p[5] = torch::exp(s[5]);

        const auto sdf = forward_sdf(pts, p);
        return torch::mean(torch::square(sdf));
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
        if (!std::isfinite(final_loss))
            break;
    }

    state_ = state.detach();

    out.ok = std::isfinite(final_loss);
    out.final_loss = final_loss;
    out.params = params();
    return out;
}

torch::Tensor TableAnalyticModel::params() const
{
    auto p = torch::zeros({kParamSize}, torch::TensorOptions().dtype(torch::kFloat32));
    const auto s = state_.detach().to(torch::kCPU);
    p[0] = s[0];
    p[1] = s[1];
    p[2] = s[2];
    p[3] = torch::exp(s[3]);
    p[4] = torch::exp(s[4]);
    p[5] = torch::exp(s[5]);
    return p;
}

}