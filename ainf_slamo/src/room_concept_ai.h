#pragma once

#include <memory>
#include <vector>
#include <limits>

// ---- PyTorch vs Qt macros (slots/signals/emit) ----
// Qt uses 'slots' as a macro. PyTorch/libtorch has methods named slots(), which breaks compilation.
// We temporarily undefine Qt macros *only* while including torch headers, then restore them.
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

#include <Eigen/Dense>
#include <Lidar3D.h>

namespace rc
{
/**
 * RoomConceptAI
 * =============
 * Implementación mínima para estimar (o refinar) el estado conjunto robot-habitación usando SDF.
 *
 * Estado (5): [width, length, x, y, phi]
 *  - width/length: dimensiones completas (m)
 *  - x,y,phi: pose del robot respecto al centro de la habitación (habitación centrada en (0,0))
 *
 * Flujo previsto en esta fase:
 *  - set_initial_state(...) con el GT / hipótesis inicial de la habitación y pose aproximada.
 *  - update(...) optimiza esos 5 parámetros minimizando mean(SDF^2) vía Adam.
 */
class RoomConceptAI
{
public:
    struct Params
    {
        int num_iterations = 50;
        float learning_rate = 0.01f;
        float min_loss_threshold = 1e-4f;
        float wall_thickness = 0.1f;
        float wall_height = 2.4f;   // meters
    };

    struct UpdateResult
    {
        bool ok = false;
        float final_loss = 0.f;
        Eigen::Matrix<float,5,1> state = Eigen::Matrix<float,5,1>::Zero();
        Eigen::Affine2f robot_pose = Eigen::Affine2f::Identity();
    };

    RoomConceptAI() = default;

    void set_initial_state(float width, float length, float x, float y, float phi);
    bool is_initialized() const { return model_ != nullptr; }

    UpdateResult update(const RoboCompLidar3D::TPoints &points);

    Params params;

private:
    // ---------------- Minimal Room Model (inline) ----------------
    // Axis-aligned box room at origin, train 5 params: half_extents (2) + robot pose (3)
    struct Model : public torch::nn::Module
    {
        // Room parameters are fixed for now
        torch::Tensor half_extents;  // [half_width, half_length] (no grad)

        // Robot pose wrt room center (optimized)
        torch::Tensor robot_pos;     // [x, y]
        torch::Tensor robot_theta;   // [phi]

        float half_height = 1.2f;    // half of wall height (fixed)

        void init_from_state(float width, float length, float x, float y, float phi, float wall_height)
        {
            const float half_w = width * 0.5f;
            const float half_l = length * 0.5f;
            half_height = wall_height * 0.5f;

            // fixed room size
            half_extents = torch::tensor({half_w, half_l}, torch::kFloat32);

            // optimized robot pose
            robot_pos = torch::tensor({x, y}, torch::requires_grad(true));
            robot_theta = torch::tensor({phi}, torch::requires_grad(true));

            register_parameter("robot_pos", robot_pos);
            register_parameter("robot_theta", robot_theta);
        }

        torch::Tensor sdf(const torch::Tensor &points_robot) const
        {
            // points_robot: [N,3] already in robot frame.
            // We bring the room box into robot frame using the inverse transform:
            // p_room = R(phi)*p_robot + t  =>  p_robot = R(-phi)*(p_room - t)
            // Therefore SDF(p_robot) = SDF_box( R(phi)*p_robot + t ) is equivalent to
            // evaluating points in room frame, but we keep points in robot frame and transform the box.
            // Implementation: compute points in box-local coordinates (room frame) via p_room = R(phi)*p_robot + t.

            const auto pxy = points_robot.index({torch::indexing::Slice(), torch::indexing::Slice(0,2)});
            const auto pz  = points_robot.index({torch::indexing::Slice(), 2}).reshape({-1,1});

            const auto c = torch::cos(robot_theta);
            const auto s = torch::sin(robot_theta);
            const auto rot = torch::stack({
                torch::stack({c.squeeze(), -s.squeeze()}),
                torch::stack({s.squeeze(),  c.squeeze()})
            });

            // Map points to room frame (equivalent to bringing room to robot frame)
            const auto points_room_xy = torch::matmul(pxy, rot.transpose(0,1)) + robot_pos;
            const auto points_room = torch::cat({points_room_xy, pz}, 1);

            const auto hz = torch::full({1}, half_height, points_room.options());
            const auto half_sizes = torch::cat({half_extents.to(points_room.options()), hz}, 0);

            const auto abs_points = torch::abs(points_room);
            const auto d = abs_points - half_sizes;

            const auto outside = torch::norm(torch::max(d, torch::zeros_like(d)), 2, 1);
            const auto inside = torch::clamp_max(
                torch::max(torch::max(d.select(1,0), d.select(1,1)), d.select(1,2)), 0.0);
            return outside + inside;
        }

        Eigen::Matrix<float,5,1> get_state() const
        {
            const auto ext = half_extents.accessor<float,1>();
            const auto pos = robot_pos.accessor<float,1>();
            const auto th  = robot_theta.accessor<float,1>();
            Eigen::Matrix<float,5,1> s;
            s << 2.f*ext[0], 2.f*ext[1], pos[0], pos[1], th[0];
            return s;
        }

        std::vector<torch::Tensor> optim_parameters() const
        {
            return {robot_pos, robot_theta};
        }
    };

    std::shared_ptr<Model> model_;

    // points to tensor [N,3]
    static torch::Tensor points_to_tensor_xyz(const RoboCompLidar3D::TPoints &points)
    {
        std::vector<float> data;
        data.reserve(points.size()*3);
        for(const auto &p : points)
        {
            data.push_back(p.x);
            data.push_back(p.y);
            data.push_back(p.z);
        }
        return torch::from_blob(data.data(), {static_cast<long>(points.size()), 3}, torch::kFloat32).clone();
    }

    static torch::Tensor loss_sdf_mse(const torch::Tensor &points_xyz, const Model &m)
    {
        const auto sdf_vals = m.sdf(points_xyz);
        return torch::mean(torch::square(sdf_vals));
    }
};

} // namespace rc

