#include "mesh_sdf_optimizer.h"

#include <algorithm>
#include <random>

// ---- PyTorch vs Qt macros (slots/signals/emit) ----
#ifdef slots
  #define RC_QT_SLOTS_WAS_DEFINED
  #undef slots
#endif
#ifdef signals
  #define RC_QT_SIGNALS_WAS_DEFINED
  #undef signals
#endif
#ifdef emit
  #define RC_QT_EMIT_WAS_DEFINED
  #undef emit
#endif

#include <torch/torch.h>

#ifdef RC_QT_SLOTS_WAS_DEFINED
  #define slots Q_SLOTS
  #undef RC_QT_SLOTS_WAS_DEFINED
#endif
#ifdef RC_QT_SIGNALS_WAS_DEFINED
  #define signals Q_SIGNALS
  #undef RC_QT_SIGNALS_WAS_DEFINED
#endif
#ifdef RC_QT_EMIT_WAS_DEFINED
  #define emit Q_EMIT
  #undef RC_QT_EMIT_WAS_DEFINED
#endif

namespace rc
{
namespace
{
torch::Tensor build_points_xy_tensor(const std::vector<Eigen::Vector3f>& points_xyz)
{
    std::vector<float> flat;
    flat.reserve(points_xyz.size() * 2);
    for (const auto& p : points_xyz)
    {
        flat.emplace_back(p.x());
        flat.emplace_back(p.y());
    }
    return torch::from_blob(flat.data(), {static_cast<long>(points_xyz.size()), 2}, torch::kFloat32).clone();
}

torch::Tensor polygon_unsigned_sdf_softmin(const torch::Tensor& points_xy,
                                           const torch::Tensor& vertices,
                                           float beta)
{
    const auto N = vertices.size(0);
    const auto idx_a = torch::arange(0, N, torch::kLong);
    const auto idx_b = torch::remainder(idx_a + 1, N);
    const auto a = vertices.index_select(0, idx_a);  // [N,2]
    const auto b = vertices.index_select(0, idx_b);  // [N,2]

    const auto ab = b - a;                                   // [N,2]
    const auto ab2 = torch::sum(ab * ab, 1).clamp_min(1e-8f); // [N]

    const auto p = points_xy.unsqueeze(1);    // [P,1,2]
    const auto a_exp = a.unsqueeze(0);        // [1,N,2]
    const auto ab_exp = ab.unsqueeze(0);      // [1,N,2]
    const auto ab2_exp = ab2.unsqueeze(0);    // [1,N]

    auto t = torch::sum((p - a_exp) * ab_exp, 2) / ab2_exp; // [P,N]
    t = torch::clamp(t, 0.0, 1.0);
    const auto proj = a_exp + t.unsqueeze(-1) * ab_exp;      // [P,N,2]
    const auto diff = p - proj;
    const auto d = torch::sqrt(torch::sum(diff * diff, 2).clamp_min(1e-12f)); // [P,N]

    const float bsoft = std::max(1.f, beta);
    return -torch::logsumexp(-bsoft * d, 1) / bsoft; // [P]
}

std::vector<Eigen::Vector3f> sample_pose_deltas(const Eigen::Matrix3f& cov, int n_samples)
{
  std::vector<Eigen::Vector3f> out;
  if (n_samples <= 0)
    n_samples = 1;

  Eigen::Matrix3f cov_sym = 0.5f * (cov + cov.transpose());
  if (!cov_sym.allFinite() || cov_sym.cwiseAbs().maxCoeff() < 1e-12f)
  {
    out.assign(1, Eigen::Vector3f::Zero());
    return out;
  }

  cov_sym += 1e-9f * Eigen::Matrix3f::Identity();
  Eigen::LLT<Eigen::Matrix3f> llt(cov_sym);
  Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  if (llt.info() == Eigen::Success)
  {
    L = llt.matrixL();
  }
  else
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov_sym);
    if (es.info() != Eigen::Success)
    {
      out.assign(1, Eigen::Vector3f::Zero());
      return out;
    }
    Eigen::Vector3f evals = es.eigenvalues().cwiseMax(1e-9f).cwiseSqrt();
    L = es.eigenvectors() * evals.asDiagonal();
  }

  std::mt19937 rng(std::random_device{}());
  std::normal_distribution<float> normal(0.f, 1.f);
  out.reserve(static_cast<std::size_t>(n_samples));
  for (int i = 0; i < n_samples; ++i)
  {
    const Eigen::Vector3f z(normal(rng), normal(rng), normal(rng));
    out.emplace_back(L * z);
  }
  return out;
}

torch::Tensor apply_pose_delta_to_points(const torch::Tensor& points_xy,
                     const Eigen::Vector3f& delta)
{
  const float dx = delta.x();
  const float dy = delta.y();
  const float dtheta = delta.z();

  const float c = std::cos(dtheta);
  const float s = std::sin(dtheta);

  const auto center = torch::mean(points_xy, 0, true);   // [1,2]
  const auto centered = points_xy - center;              // [P,2]

  auto R = torch::tensor({{c, -s}, {s, c}}, torch::TensorOptions().dtype(torch::kFloat32));
  const auto rotated = torch::matmul(centered, R.transpose(0, 1));
  const auto t = torch::tensor({dx, dy}, torch::TensorOptions().dtype(torch::kFloat32)).unsqueeze(0);
  return rotated + center + t;
}

std::pair<torch::Tensor, torch::Tensor> monte_carlo_data_loss(const torch::Tensor& points_xy,
                                 const torch::Tensor& vertices,
                                 float beta,
                                 const Eigen::Matrix3f& pose_cov,
                                 int mc_samples,
                                 bool use_mc_sampling)
{
  if (!use_mc_sampling || mc_samples <= 1)
  {
    const auto sdf_vals = polygon_unsigned_sdf_softmin(points_xy, vertices, beta);
    const auto mean_l = torch::mean(torch::square(sdf_vals));
    const auto std_l = torch::zeros({}, torch::kFloat32);
    return {mean_l, std_l};
  }

  const auto deltas = sample_pose_deltas(pose_cov, std::max(1, mc_samples));
  std::vector<torch::Tensor> losses;
  losses.reserve(deltas.size());
  for (const auto& d : deltas)
  {
    const auto pts = apply_pose_delta_to_points(points_xy, d);
    const auto sdf_vals = polygon_unsigned_sdf_softmin(pts, vertices, beta);
    losses.emplace_back(torch::mean(torch::square(sdf_vals)));
  }
  const auto losses_t = torch::stack(losses);  // [K]
  const auto mean_l = torch::mean(losses_t);
  const auto std_l = torch::sqrt(torch::mean(torch::square(losses_t - mean_l)).clamp_min(1e-12f));
  return {mean_l, std_l};
}

torch::Tensor build_vertices_xy_tensor(const std::vector<Eigen::Vector2f>& vertices_xy)
{
  std::vector<float> flat;
  flat.reserve(vertices_xy.size() * 2);
  for (const auto& v : vertices_xy)
  {
    flat.emplace_back(v.x());
    flat.emplace_back(v.y());
  }
  return torch::from_blob(flat.data(), {static_cast<long>(vertices_xy.size()), 2}, torch::kFloat32).clone();
}

bool point_in_polygon(const Eigen::Vector2f& p, const std::vector<Eigen::Vector2f>& poly)
{
  if (poly.size() < 3)
    return false;

  bool inside = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
  {
    const auto& pi = poly[i];
    const auto& pj = poly[j];
    const bool intersects = ((pi.y() > p.y()) != (pj.y() > p.y())) &&
      (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / ((pj.y() - pi.y()) + 1e-9f) + pi.x());
    if (intersects)
      inside = !inside;
  }
  return inside;
}

Eigen::Vector2f closest_point_on_segment(const Eigen::Vector2f& p,
                     const Eigen::Vector2f& a,
                     const Eigen::Vector2f& b)
{
  const Eigen::Vector2f ab = b - a;
  const float ab2 = std::max(1e-9f, ab.squaredNorm());
  const float t = std::clamp((p - a).dot(ab) / ab2, 0.f, 1.f);
  return a + t * ab;
}

Eigen::Vector2f closest_point_on_polygon(const Eigen::Vector2f& p, const std::vector<Eigen::Vector2f>& poly)
{
  if (poly.size() < 2)
    return p;

  Eigen::Vector2f best = poly.front();
  float best_d2 = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < poly.size(); ++i)
  {
    const Eigen::Vector2f a = poly[i];
    const Eigen::Vector2f b = poly[(i + 1) % poly.size()];
    const Eigen::Vector2f q = closest_point_on_segment(p, a, b);
    const float d2 = (p - q).squaredNorm();
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best = q;
    }
  }
  return best;
}

void push_polygon_inside_room(std::vector<Eigen::Vector2f>& vertices,
                Eigen::Vector2f& translation,
                const std::vector<Eigen::Vector2f>& room_polygon)
{
  if (vertices.empty() || room_polygon.size() < 3)
    return;

  constexpr int max_push_iters = 24;
  constexpr float margin = 0.015f;  // 1.5 cm safety inset

  for (int it = 0; it < max_push_iters; ++it)
  {
    Eigen::Vector2f correction = Eigen::Vector2f::Zero();
    int outside_count = 0;

    for (const auto& v : vertices)
    {
      if (point_in_polygon(v, room_polygon))
        continue;

      const Eigen::Vector2f q = closest_point_on_polygon(v, room_polygon);
      Eigen::Vector2f dir = q - v;
      const float n = dir.norm();
      if (n > 1e-6f)
        dir /= n;
      correction += (q - v) + margin * dir;
      ++outside_count;
    }

    if (outside_count == 0)
      break;

    correction /= static_cast<float>(outside_count);
    for (auto& v : vertices)
      v += correction;
    translation += correction;
  }
}
} // namespace

MeshSDFOptimizer::Result MeshSDFOptimizer::optimize_mesh_with_pose(
    const std::vector<Eigen::Vector3f>& points_xyz,
  const std::vector<Eigen::Vector2f>& mesh_vertices,
  const std::vector<Eigen::Vector2f>& room_polygon,
  const Eigen::Matrix3f& robot_pose_cov) const
{
    Result out;
    if (points_xyz.size() < 8 || mesh_vertices.size() < 3)
        return out;

  const torch::Tensor base_vertices = build_vertices_xy_tensor(mesh_vertices);
  const torch::Tensor points_xy = build_points_xy_tensor(points_xyz);
  const bool has_room = room_polygon.size() >= 3;
  const torch::Tensor room_vertices = has_room ? build_vertices_xy_tensor(room_polygon) : torch::Tensor();

  torch::Tensor translation = torch::zeros({2}, torch::kFloat32);
  translation.set_requires_grad(true);
  torch::Tensor yaw = torch::zeros({}, torch::kFloat32);
  yaw.set_requires_grad(true);
  torch::optim::Adam optimizer({translation, yaw}, torch::optim::AdamOptions(params_.learning_rate));

  auto rigid_transform_vertices = [&](const torch::Tensor& verts,
                                      const torch::Tensor& t,
                                      const torch::Tensor& theta) -> torch::Tensor
  {
    const auto c = torch::cos(theta);
    const auto s = torch::sin(theta);
    const auto row0 = torch::stack({c, -s});
    const auto row1 = torch::stack({s, c});
    const auto R = torch::stack({row0, row1}); // [2,2]
    const auto center = torch::mean(verts, 0, true); // [1,2]
    const auto centered = verts - center;
    const auto rotated = torch::matmul(centered, R.transpose(0, 1));
    return rotated + center + t.unsqueeze(0);
  };

  const auto vertices0 = rigid_transform_vertices(base_vertices, translation, yaw);
  auto [data_loss0, data_std0] = monte_carlo_data_loss(points_xy, vertices0, params_.edge_softmin_beta,
                                                       robot_pose_cov, params_.mc_samples, params_.use_mc_sampling);
  auto wall_loss0 = torch::zeros({}, torch::kFloat32);
  const auto decouple_loss0 = torch::zeros({}, torch::kFloat32);
  const auto vertex_reg0 = torch::zeros({}, torch::kFloat32);
  if (has_room)
  {
    const auto v0_cpu = vertices0.detach().to(torch::kCPU);
    const auto acc0 = v0_cpu.accessor<float, 2>();
    std::vector<float> outside_mask;
    outside_mask.reserve(v0_cpu.size(0));
    for (int i = 0; i < v0_cpu.size(0); ++i)
    {
      const Eigen::Vector2f p(acc0[i][0], acc0[i][1]);
      outside_mask.emplace_back(point_in_polygon(p, room_polygon) ? 0.f : 1.f);
    }
    const auto outside_t = torch::from_blob(outside_mask.data(), {v0_cpu.size(0)}, torch::kFloat32).clone();
    const auto d_room = polygon_unsigned_sdf_softmin(vertices0, room_vertices, params_.edge_softmin_beta);
    wall_loss0 = torch::mean(torch::square(outside_t * d_room));
  }
    auto loss0 = data_loss0
      + params_.mc_risk_beta * data_std0
         + params_.wall_penalty_weight * wall_loss0
         + params_.translation_decouple_weight * decouple_loss0
         + params_.vertex_reg_weight * vertex_reg0;
  out.initial_loss = loss0.item<float>();

    float cur_loss = out.initial_loss;
    for (int it = 0; it < params_.max_iterations; ++it)
    {
        optimizer.zero_grad();

    const auto posed_vertices = rigid_transform_vertices(base_vertices, translation, yaw);
    auto [data_loss, data_std] = monte_carlo_data_loss(points_xy, posed_vertices, params_.edge_softmin_beta,
                               robot_pose_cov, params_.mc_samples, params_.use_mc_sampling);
    const auto decouple_loss = torch::zeros({}, torch::kFloat32);
    const auto vertex_reg = torch::zeros({}, torch::kFloat32);

    auto wall_loss = torch::zeros({}, torch::kFloat32);
    if (has_room)
    {
      const auto v_cpu = posed_vertices.detach().to(torch::kCPU);
      const auto acc = v_cpu.accessor<float, 2>();
      std::vector<float> outside_mask;
      outside_mask.reserve(v_cpu.size(0));
      for (int i = 0; i < v_cpu.size(0); ++i)
      {
        const Eigen::Vector2f p(acc[i][0], acc[i][1]);
        outside_mask.emplace_back(point_in_polygon(p, room_polygon) ? 0.f : 1.f);
      }
      const auto outside_t = torch::from_blob(outside_mask.data(), {v_cpu.size(0)}, torch::kFloat32).clone();
      const auto d_room = polygon_unsigned_sdf_softmin(posed_vertices, room_vertices, params_.edge_softmin_beta);
      wall_loss = torch::mean(torch::square(outside_t * d_room));
    }

        auto loss = data_loss
          + params_.mc_risk_beta * data_std
          + params_.wall_penalty_weight * wall_loss
          + params_.translation_decouple_weight * decouple_loss
          + params_.vertex_reg_weight * vertex_reg;
        loss.backward();
        optimizer.step();

    out.final_data_loss = data_loss.item<float>();
    out.final_data_std = data_std.item<float>();
    out.final_wall_loss = wall_loss.item<float>();
        cur_loss = loss.item<float>();
        out.iterations = it + 1;
        if (!std::isfinite(cur_loss))
            break;
    }

    out.ok = std::isfinite(cur_loss);
    out.final_loss = cur_loss;
  out.translation = Eigen::Vector2f(translation.detach().to(torch::kCPU)[0].item<float>(),
                    translation.detach().to(torch::kCPU)[1].item<float>());
    out.vertices.clear();
    out.vertices.reserve(mesh_vertices.size());

  const auto vertices_final = rigid_transform_vertices(base_vertices, translation, yaw);
  const auto verts_cpu = vertices_final.detach().to(torch::kCPU);
    const auto acc = verts_cpu.accessor<float, 2>();
    for (int i = 0; i < verts_cpu.size(0); ++i)
        out.vertices.emplace_back(acc[i][0], acc[i][1]);

  if (has_room)
  {
    push_polygon_inside_room(out.vertices, out.translation, room_polygon);
  }

    return out;
}
}
