#pragma once

#include <Eigen/Dense>
#include <vector>

namespace rc
{
class MeshSDFOptimizer
{
public:
    struct Params
    {
        int max_iterations;
        float learning_rate;
        float edge_softmin_beta;
        float wall_penalty_weight;
        float translation_decouple_weight;
        float vertex_reg_weight;
        bool use_mc_sampling;
        int mc_samples;
        float mc_risk_beta;

        Params()
            : max_iterations(80)
            , learning_rate(0.03f)
            , edge_softmin_beta(20.f)
            , wall_penalty_weight(25.f)
            , translation_decouple_weight(3.f)
            , vertex_reg_weight(0.01f)
            , use_mc_sampling(false)
            , mc_samples(64)
            , mc_risk_beta(0.f)
        {}
    };

    struct Result
    {
        bool ok = false;
        std::vector<Eigen::Vector2f> vertices;
        Eigen::Vector2f translation = Eigen::Vector2f::Zero();
        float initial_loss = 0.f;
        float final_loss = 0.f;
        float final_data_loss = 0.f;
        float final_data_std = 0.f;
        float final_wall_loss = 0.f;
        int iterations = 0;
    };

    MeshSDFOptimizer() = default;
    explicit MeshSDFOptimizer(const Params& p) : params_(p) {}

    Result optimize_mesh_with_pose(const std::vector<Eigen::Vector3f>& points_xyz,
                                   const std::vector<Eigen::Vector2f>& mesh_vertices,
                                   const std::vector<Eigen::Vector2f>& room_polygon,
                                   const Eigen::Matrix3f& robot_pose_cov = Eigen::Matrix3f::Zero()) const;

private:
    Params params_;
};
}
