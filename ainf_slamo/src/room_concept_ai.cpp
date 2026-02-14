#include "room_concept_ai.h"

namespace rc
{

    void RoomConceptAI::set_initial_state(float width, float length, float x, float y, float phi)
    {
        model_ = std::make_shared<Model>();
        model_->init_from_state(width, length, x, y, phi, params.wall_height);
    }

    RoomConceptAI::UpdateResult RoomConceptAI::update(const RoboCompLidar3D::TPoints &points)
    {
        UpdateResult res;
        if(points.empty())
            return res;

        if(model_ == nullptr)
            return res;

        torch::Tensor points_tensor = points_to_tensor_xyz(points);

        std::vector<torch::Tensor> optim_params = model_->optim_parameters();
        torch::optim::Adam optimizer(optim_params, torch::optim::AdamOptions(params.learning_rate));

        float last_loss = std::numeric_limits<float>::infinity();
        for(int i=0; i<params.num_iterations; ++i)
        {
            optimizer.zero_grad();
            torch::Tensor loss = loss_sdf_mse(points_tensor, *model_);
            loss.backward();
            optimizer.step();

            last_loss = loss.item<float>();
            if(last_loss < params.min_loss_threshold)
                break;
        }

        res.ok = true;
        res.final_loss = last_loss;
        res.state = model_->get_state();
        {
            const float x = res.state[2];
            const float y = res.state[3];
            const float phi = res.state[4];
            Eigen::Affine2f pose = Eigen::Affine2f::Identity();
            pose.translation() = Eigen::Vector2f{x, y};
            pose.linear() = Eigen::Rotation2Df(phi).toRotationMatrix();
            res.robot_pose = pose;
        }
        return res;
}

} // namespace rc
