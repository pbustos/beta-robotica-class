#include "room_model.h"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace rc
{

namespace
{
Eigen::Vector2f dominant_axis_dir(const Eigen::Vector2f& e)
{
    if (std::abs(e.x()) >= std::abs(e.y()))
    {
        const float sx = (e.x() >= 0.f) ? 1.f : -1.f;
        return {sx, 0.f};
    }
    const float sy = (e.y() >= 0.f) ? 1.f : -1.f;
    return {0.f, sy};
}

bool orthogonal_dirs(const Eigen::Vector2f& a, const Eigen::Vector2f& b)
{
    return std::abs(a.dot(b)) < 1e-5f;
}

template<typename T>
std::vector<T> rotate_left(const std::vector<T>& v, int shift)
{
    if (v.empty())
        return v;
    const int n = static_cast<int>(v.size());
    const int s = ((shift % n) + n) % n;
    std::vector<T> out;
    out.reserve(v.size());
    for (int i = 0; i < n; ++i)
        out.push_back(v[(i + s) % n]);
    return out;
}
} // namespace

void Model::set_device(torch::Device device)
{
    device_ = device;
}

void Model::init_from_state(float width, float length, float x, float y, float phi, float wall_height)
{
    use_polygon = false;
    const float half_w = width * 0.5f;
    const float half_l = length * 0.5f;
    half_height = wall_height * 0.5f;

    // Room size can be adapted online during bootstrap/localization refinement.
    half_extents = torch::tensor({half_w, half_l},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));

    // optimized robot pose
    robot_pos = torch::tensor({x, y},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));
    robot_theta = torch::tensor({phi},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));

    register_parameter("half_extents", half_extents);
    register_parameter("robot_pos", robot_pos);
    register_parameter("robot_theta", robot_theta);

    init_common();
}

void Model::init_from_polygon(const std::vector<Eigen::Vector2f>& vertices,
                               float x, float y, float phi, float wall_height)
{
    use_polygon = true;
    half_height = wall_height * 0.5f;

    if (vertices.size() < 3)
        return;

    // Preserve the exact BMR-proposed polygon vertices to avoid flattening dents.
    const auto& verts10 = vertices;

    // Build a Manhattan-by-construction manifold from the initial polygon:
    // fixed axis-aligned edge directions + trainable edge lengths.
    std::vector<Eigen::Vector2f> edge_dirs;
    std::vector<float> edge_lengths;
    edge_dirs.reserve(verts10.size());
    edge_lengths.reserve(verts10.size());
    for (size_t i = 0; i < verts10.size(); ++i)
    {
        const auto& a = verts10[i];
        const auto& b = verts10[(i + 1) % verts10.size()];
        const Eigen::Vector2f e = b - a;
        const Eigen::Vector2f d = dominant_axis_dir(e);
        const float len = std::max(0.01f, std::abs(e.dot(d)));
        edge_dirs.push_back(d);
        edge_lengths.push_back(len);
    }

    // Rotate the edge sequence so the last two closure directions are orthogonal.
    int shift = 0;
    for (int i = 0; i < static_cast<int>(edge_dirs.size()); ++i)
    {
        const int j = (i + 1) % static_cast<int>(edge_dirs.size());
        if (orthogonal_dirs(edge_dirs[i], edge_dirs[j]))
        {
            shift = (i + 2) % static_cast<int>(edge_dirs.size());
            break;
        }
    }
    const auto verts_rot = rotate_left(verts10, shift);
    const auto dirs_rot = rotate_left(edge_dirs, shift);
    const auto lens_rot = rotate_left(edge_lengths, shift);

    std::vector<float> dirs_flat;
    dirs_flat.reserve(dirs_rot.size() * 2);
    for (const auto& d : dirs_rot)
    {
        dirs_flat.push_back(d.x());
        dirs_flat.push_back(d.y());
    }
    polygon_edge_dirs = torch::from_blob(dirs_flat.data(),
        {static_cast<long>(dirs_rot.size()), 2}, torch::kFloat32).clone().to(device_);

    polygon_anchor = torch::tensor({verts_rot.front().x(), verts_rot.front().y()},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    polygon_ref_lengths = torch::from_blob(const_cast<float*>(lens_rot.data()),
        {static_cast<long>(lens_rot.size())}, torch::kFloat32).clone().to(device_);

    const int n = static_cast<int>(lens_rot.size());
    const int n_free = std::max(1, n - 2);
    std::vector<float> logits;
    logits.reserve(static_cast<size_t>(n_free));
    for (int i = 0; i < n_free; ++i)
    {
        const float y = std::max(1e-4f, lens_rot[i] - polygon_min_edge);
        logits.push_back(std::log(std::exp(y) - 1.0f));  // inverse softplus
    }
    polygon_edge_logits = torch::from_blob(logits.data(),
        {static_cast<long>(logits.size())}, torch::kFloat32).clone().to(device_);
    polygon_edge_logits = polygon_edge_logits.set_requires_grad(true);

    // Expose the current reconstructed polygon for visualization/debug consumers.
    polygon_vertices = current_polygon_vertices();

    // Compute bounding box for half_extents (for compatibility)
    float min_x = verts10[0].x(), max_x = verts10[0].x();
    float min_y = verts10[0].y(), max_y = verts10[0].y();
    for (const auto& v : verts10)
    {
        min_x = std::min(min_x, v.x()); max_x = std::max(max_x, v.x());
        min_y = std::min(min_y, v.y()); max_y = std::max(max_y, v.y());
    }
    half_extents = torch::tensor({(max_x - min_x) / 2.f, (max_y - min_y) / 2.f},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    // optimized robot pose
    robot_pos = torch::tensor({x, y},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));
    robot_theta = torch::tensor({phi},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));

    register_parameter("polygon_edge_logits", polygon_edge_logits);
    register_parameter("robot_pos", robot_pos);
    register_parameter("robot_theta", robot_theta);

    init_common();
}

void Model::init_common()
{
    // Initialize prediction tensors (detached from graph)
    predicted_pos = torch::zeros({2},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    predicted_theta = torch::zeros({1},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    prediction_precision_matrix = torch::eye(3,
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    has_prediction = false;

    // Initialize covariance for EKF prediction (moderate initial uncertainty)
    prev_cov = 0.1f * torch::eye(3,
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    // Initialize previous pose
    robot_prev_pose = std::nullopt;
}

void Model::set_prediction(const Eigen::Vector2f& pred_pos, float pred_theta,
                           const Eigen::Matrix3f& precision)
{
    predicted_pos = torch::tensor({pred_pos.x(), pred_pos.y()},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    predicted_theta = torch::tensor({pred_theta},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    // Convert Eigen Matrix to Tensor
    prediction_precision_matrix = torch::from_blob(
        const_cast<float*>(precision.data()),
        {3, 3},
        torch::kFloat32).clone().to(device_);

    has_prediction = true;
}

torch::Tensor Model::prior_loss() const
{
    if (!has_prediction)
        return torch::tensor(0.0f, torch::TensorOptions().device(device_));

    // Current state vector [x, y, theta]
    auto current_state = torch::cat({robot_pos, robot_theta}, 0); // [3]
    auto pred_state = torch::cat({predicted_pos, predicted_theta}, 0); // [3]

    auto diff = (current_state - pred_state).unsqueeze(1); // [3, 1]

    // (s - mu)^T * Sigma^-1 * (s - mu)
    // [1, 3] * [3, 3] * [3, 1] -> [1, 1]
    auto loss = torch::matmul(diff.t(), torch::matmul(prediction_precision_matrix, diff));

    return 0.5f * loss.squeeze();
}

torch::Tensor Model::sdf(const torch::Tensor& points_robot) const
{
    // Transform points from robot frame to room frame
    // Use the device of input points as the reference
    const auto device = points_robot.device();
    const auto pxy = points_robot.index({torch::indexing::Slice(), torch::indexing::Slice(0,2)});

    // Move model parameters to the same device as points (no-op if already there)
    const auto theta = robot_theta.to(device);
    const auto pos = robot_pos.to(device);

    const auto c = torch::cos(theta);
    const auto s = torch::sin(theta);

    // Build rotation matrix on the same device as points
    auto rot = torch::zeros({2, 2}, pxy.options());
    rot[0][0] = c.squeeze();
    rot[0][1] = -s.squeeze();
    rot[1][0] = s.squeeze();
    rot[1][1] = c.squeeze();

    // Map points to room frame: p_room = R(phi) * p_robot + t
    const auto points_room_xy = torch::matmul(pxy, rot.transpose(0,1)) + pos;

    if (use_polygon)
    {
        return sdf_polygon(points_room_xy);
    }
    else
    {
        return sdf_box(points_robot, points_room_xy);
    }
}

torch::Tensor Model::sdf_box(const torch::Tensor& points_robot,
                              const torch::Tensor& points_room_xy) const
{
    const auto device = points_room_xy.device();
    const auto pz = points_robot.index({torch::indexing::Slice(), 2}).reshape({-1,1}).to(device);

    // LiDAR z is in floor-referenced coordinates [0, wall_height], while this SDF
    // uses a centered box representation [-half_height, +half_height].
    // Shift z by half-height to align both conventions.
    const auto hz = torch::full({1}, half_height, points_room_xy.options());
    const auto pz_centered = pz - hz;
    const auto points_room = torch::cat({points_room_xy, pz_centered}, 1);

    const auto half_ext = half_extents.to(device);
    const auto half_sizes = torch::cat({half_ext, hz}, 0);

    const auto abs_points = torch::abs(points_room);
    const auto d = abs_points - half_sizes;

    const auto outside = torch::norm(torch::max(d, torch::zeros_like(d)), 2, 1);
    const auto inside = torch::clamp_max(
        torch::max(torch::max(d.select(1,0), d.select(1,1)), d.select(1,2)), 0.0);
    return outside + inside;
}

torch::Tensor Model::sdf_polygon(const torch::Tensor& points_room_xy) const
{
    // Fully vectorized: broadcast [N,1,2] vs [1,S,2] for all segments at once
    const auto verts = current_polygon_vertices().to(points_room_xy.device());
    const auto num_verts = verts.size(0);
    auto indices_a = torch::arange(num_verts,
        torch::TensorOptions().dtype(torch::kLong).device(points_room_xy.device()));
    auto indices_b = (indices_a + 1) % num_verts;
    const auto seg_a = verts.index_select(0, indices_a).contiguous();
    const auto seg_b = verts.index_select(0, indices_b).contiguous();
    const auto seg_ab = (seg_b - seg_a).contiguous();
    const auto seg_ab_sq = torch::sum(seg_ab * seg_ab, /*dim=*/1).contiguous();

    // points: [N, 2] → [N, 1, 2];  segments: [S, 2] → [1, S, 2]
    const auto pts = points_room_xy.unsqueeze(1);  // [N, 1, 2]
    const auto a_  = seg_a.unsqueeze(0);            // [1, S, 2]
    const auto ab_ = seg_ab.unsqueeze(0);           // [1, S, 2]

    // ap = pts - a  →  [N, S, 2]
    const auto ap = pts - a_;

    // t = clamp(dot(ap, ab) / |ab|², 0, 1)  →  [N, S]
    auto t = torch::sum(ap * ab_, /*dim=*/2) / (seg_ab_sq.unsqueeze(0) + 1e-8f);
    t = torch::clamp(t, 0.0f, 1.0f);

    // closest = a + t * ab  →  [N, S, 2]
    const auto closest = a_ + t.unsqueeze(2) * ab_;

    // dist² = |pts - closest|²  →  [N, S]
    const auto diff = pts - closest;
    const auto dist_sq = torch::sum(diff * diff, /*dim=*/2);

    // Min over segments → [N]
    const auto min_dist_sq = std::get<0>(torch::min(dist_sq, /*dim=*/1));

    return torch::sqrt(min_dist_sq + 1e-8f);
}

Eigen::Matrix<float, 5, 1> Model::get_state() const
{
    // Must convert to CPU for accessor
    auto ext_cpu = half_extents.to(torch::kCPU);
    auto pos_cpu = robot_pos.to(torch::kCPU);
    auto th_cpu  = robot_theta.to(torch::kCPU);
    float width = 2.f * ext_cpu.accessor<float,1>()[0];
    float length = 2.f * ext_cpu.accessor<float,1>()[1];

    if (use_polygon && polygon_edge_logits.defined())
    {
        const auto poly_cpu = current_polygon_vertices().to(torch::kCPU);
        auto p = poly_cpu.accessor<float,2>();
        float min_x = p[0][0], max_x = p[0][0];
        float min_y = p[0][1], max_y = p[0][1];
        for (int i = 1; i < poly_cpu.size(0); ++i)
        {
            min_x = std::min(min_x, p[i][0]);
            max_x = std::max(max_x, p[i][0]);
            min_y = std::min(min_y, p[i][1]);
            max_y = std::max(max_y, p[i][1]);
        }
        width = max_x - min_x;
        length = max_y - min_y;
    }

    const auto pos = pos_cpu.accessor<float,1>();
    const auto th  = th_cpu.accessor<float,1>();
    Eigen::Matrix<float,5,1> s;
    s << width, length, pos[0], pos[1], th[0];
    return s;
}

std::vector<torch::Tensor> Model::optim_parameters() const
{
    if (!use_polygon)
        return {half_extents, robot_pos, robot_theta};
    return {polygon_edge_logits, robot_pos, robot_theta};
}

std::pair<torch::Tensor, torch::Tensor> Model::reconstruct_polygon_from_params() const
{
    const auto device = polygon_edge_logits.device();
    const auto dirs = polygon_edge_dirs.to(device);
    const auto anchor = polygon_anchor.to(device);

    const int n = static_cast<int>(dirs.size(0));
    const int n_free = std::max(1, n - 2);

    auto lengths = torch::zeros({n}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
    auto free_lengths = torch::softplus(polygon_edge_logits) + polygon_min_edge;
    lengths.index_put_({torch::indexing::Slice(0, n_free)}, free_lengths);

    std::vector<torch::Tensor> verts;
    verts.reserve(static_cast<size_t>(n));

    auto v = anchor.clone();
    verts.push_back(v);
    for (int i = 0; i < n_free; ++i)
    {
        const auto li = lengths.index({i});
        const auto di = dirs.index({i});
        v = v + li * di;
        verts.push_back(v);
    }

    const auto d_last_a = dirs.index({n - 2});
    const auto d_last_b = dirs.index({n - 1});
    const auto residual = anchor - v;
    const auto l_last_a = torch::sum(residual * d_last_a);
    lengths.index_put_({n - 2}, l_last_a);

    const auto v_last = v + l_last_a * d_last_a;
    const auto l_last_b = torch::sum((anchor - v_last) * d_last_b);
    lengths.index_put_({n - 1}, l_last_b);
    verts.push_back(v_last);

    auto verts_tensor = torch::stack(verts, 0);
    return {verts_tensor, lengths};
}

torch::Tensor Model::current_polygon_vertices() const
{
    if (!use_polygon || !polygon_edge_logits.defined() || !polygon_edge_dirs.defined())
        return torch::Tensor();

    auto [verts, lengths] = reconstruct_polygon_from_params();
    (void)lengths;
    return verts;
}

torch::Tensor Model::polygon_constraint_loss() const
{
    if (!use_polygon || !polygon_edge_logits.defined() || !polygon_edge_dirs.defined())
        return torch::zeros({}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    auto [verts, lengths] = reconstruct_polygon_from_params();

    const auto ref = polygon_ref_lengths.to(lengths.device());
    const auto min_len = torch::full_like(lengths, polygon_min_edge);

    // Penalize edge flips/collapses and large deviations from initialized topology.
    const auto neg_penalty = torch::mean(torch::pow(torch::relu(min_len - lengths), 2));
    const auto shape_reg = torch::mean(torch::pow(lengths - ref, 2));

    // Keep closure numerically tight in optimization.
    const auto v0 = verts.index({0});
    const auto vlast = verts.index({verts.size(0) - 1});
    const auto d_last = polygon_edge_dirs.to(lengths.device()).index({polygon_edge_dirs.size(0) - 1});
    const auto closure_vec = v0 - vlast - lengths.index({lengths.size(0) - 1}) * d_last;
    const auto closure_penalty = torch::sum(closure_vec * closure_vec);

    return 50.0f * neg_penalty + 0.05f * shape_reg + 10.0f * closure_penalty;
}


} // namespace rc

