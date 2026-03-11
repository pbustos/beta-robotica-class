#include "object_ownership_em.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rc
{

float ObjectOwnershipEM::robust_charbonnier(float r, float sigma)
{
    const float s = std::max(1e-4f, sigma);
    const float x = r / s;
    return std::sqrt(1.0f + x * x) - 1.0f;
}

void ObjectOwnershipEM::configure(const Config& cfg)
{
    config_ = cfg;
}

void ObjectOwnershipEM::set_models(const std::vector<ClassModel>& models)
{
    models_ = models;
}

void ObjectOwnershipEM::set_observations(const ObservationBatch& observations)
{
    observations_ = observations;

    const int n_points = static_cast<int>(observations_.points_world.size());
    const int n_classes = static_cast<int>(models_.size());
    if (n_points > 0 && n_classes > 0)
        ownership_ = OwnershipMatrix::Constant(n_points, n_classes, 1.0f / static_cast<float>(n_classes));
    else
        ownership_.resize(0, 0);
}

void ObjectOwnershipEM::set_initial_states(const std::vector<ClassState>& states)
{
    states_ = states;
}

bool ObjectOwnershipEM::run_single_iteration()
{
    if (models_.empty() || observations_.points_world.empty())
        return false;

    const int n_points = static_cast<int>(observations_.points_world.size());
    const int n_classes = static_cast<int>(models_.size());
    if (ownership_.rows() != n_points || ownership_.cols() != n_classes)
        ownership_ = OwnershipMatrix::Constant(n_points, n_classes, 1.0f / static_cast<float>(n_classes));

    if (states_.size() != models_.size())
    {
        states_.resize(models_.size());
        for (std::size_t k = 0; k < models_.size(); ++k)
            states_[k].class_id = models_[k].class_id;
    }

    const float alpha = std::clamp(static_cast<float>(em_iteration_) / std::max(1, config_.max_iterations - 1), 0.0f, 1.0f);
    const float temperature = config_.anneal_t_start + (config_.anneal_t_end - config_.anneal_t_start) * alpha;

    std::vector<float> energies(n_classes, 0.0f);
    std::vector<float> expw(n_classes, 0.0f);
    std::vector<float> class_weight_sum(n_classes, 0.0f);

    metrics_.objective = 0.0f;
    metrics_.avg_entropy = 0.0f;
    metrics_.background_fraction = 0.0f;
    metrics_.class_residuals.assign(n_classes, 0.0f);
    metrics_.class_confidences.assign(n_classes, 0.0f);

    for (int i = 0; i < n_points; ++i)
    {
        const Eigen::Vector3f& p = observations_.points_world[static_cast<std::size_t>(i)];
        const bool has_rgbd_hint = observations_.rgbd_residual_hint.size() == static_cast<std::size_t>(n_points);
        const float rgbd_hint = has_rgbd_hint ? observations_.rgbd_residual_hint[static_cast<std::size_t>(i)] : 0.f;

        float min_e = std::numeric_limits<float>::max();
        for (int k = 0; k < n_classes; ++k)
        {
            const auto& model = models_[static_cast<std::size_t>(k)];
            const auto& state = states_[static_cast<std::size_t>(k)];
            const float prior = std::max(1e-4f, model.prior_weight);

            float e = 0.0f;
            float residual = 0.0f;
            if (model.is_background)
            {
                const float r_bg = std::abs(p.z());
                residual = r_bg;
                e = 0.35f * robust_charbonnier(r_bg, config_.robust_sigma);
            }
            else
            {
                const Eigen::Vector2f dxy = p.head<2>() - state.position.head<2>();
                const float r_xy = dxy.norm();
                const float expected_z = (model.expected_height > 1e-4f)
                    ? 0.5f * model.expected_height
                    : state.position.z();
                const float r_z = std::abs(p.z() - expected_z);
                residual = r_xy;

                e = robust_charbonnier(r_xy, config_.robust_sigma)
                  + 0.30f * robust_charbonnier(r_z, config_.robust_sigma);

                if (!state.active)
                    e += 2.0f;
            }

            if (has_rgbd_hint && config_.rgbd_residual_weight > 0.f)
            {
                const float r_rgbd = std::max(0.f, rgbd_hint);
                e += config_.rgbd_residual_weight * robust_charbonnier(r_rgbd, config_.robust_sigma);
            }

            e -= std::log(prior);
            energies[static_cast<std::size_t>(k)] = e;
            metrics_.class_residuals[static_cast<std::size_t>(k)] += residual;
            min_e = std::min(min_e, e);
        }

        float sumw = 0.0f;
        for (int k = 0; k < n_classes; ++k)
        {
            const float w = std::exp(-(energies[static_cast<std::size_t>(k)] - min_e) / std::max(1e-4f, temperature));
            expw[static_cast<std::size_t>(k)] = w;
            sumw += w;
        }
        sumw = std::max(sumw, 1e-8f);

        float entropy_i = 0.0f;
        for (int k = 0; k < n_classes; ++k)
        {
            const float q = expw[static_cast<std::size_t>(k)] / sumw;
            ownership_(i, k) = q;
            class_weight_sum[static_cast<std::size_t>(k)] += q;
            metrics_.class_confidences[static_cast<std::size_t>(k)] += q;
            metrics_.objective += q * energies[static_cast<std::size_t>(k)];
            entropy_i += -q * std::log(std::max(q, 1e-8f));
        }
        metrics_.avg_entropy += entropy_i;
    }

    metrics_.avg_entropy /= static_cast<float>(n_points);
    metrics_.objective /= static_cast<float>(n_points);
    for (int k = 0; k < n_classes; ++k)
    {
        const float wsum = std::max(class_weight_sum[static_cast<std::size_t>(k)], 1e-6f);
        metrics_.class_residuals[static_cast<std::size_t>(k)] /= wsum;
        metrics_.class_confidences[static_cast<std::size_t>(k)] /= static_cast<float>(n_points);
        states_[static_cast<std::size_t>(k)].confidence = metrics_.class_confidences[static_cast<std::size_t>(k)];
    }

    for (int k = 0; k < n_classes; ++k)
    {
        if (models_[static_cast<std::size_t>(k)].is_background)
            metrics_.background_fraction += metrics_.class_confidences[static_cast<std::size_t>(k)];
    }

    // ---------------- M-step: weighted class state update ----------------
    float max_pose_delta = 0.0f;
    constexpr float min_weight_for_update = 1e-3f;

    for (int k = 0; k < n_classes; ++k)
    {
        auto& state = states_[static_cast<std::size_t>(k)];
        const auto& model = models_[static_cast<std::size_t>(k)];
        if (model.is_background || !state.active)
            continue;

        double wsum = 0.0;
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (int i = 0; i < n_points; ++i)
        {
            const float w = ownership_(i, k);
            wsum += static_cast<double>(w);
            mean += static_cast<double>(w) * observations_.points_world[static_cast<std::size_t>(i)].cast<double>();
        }
        if (wsum < min_weight_for_update)
            continue;
        mean /= wsum;

        // Weighted 2D covariance for yaw estimation.
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (int i = 0; i < n_points; ++i)
        {
            const float w = ownership_(i, k);
            const Eigen::Vector2d d = observations_.points_world[static_cast<std::size_t>(i)].head<2>().cast<double>() - mean.head<2>();
            cov += static_cast<double>(w) * (d * d.transpose());
        }
        cov /= std::max(1e-9, wsum);

        float new_yaw = state.yaw_rad;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
        if (eig.info() == Eigen::Success)
        {
            const Eigen::Vector2d axis = eig.eigenvectors().col(1).normalized();
            new_yaw = std::atan2(static_cast<float>(axis.y()), static_cast<float>(axis.x()));
        }

        Eigen::Vector3f new_pos = mean.cast<float>();
        if (model.expected_height > 1e-4f)
            new_pos.z() = 0.5f * model.expected_height;

        // Gate updates by confidence / support and pose jump limits.
        if (state.confidence < 1e-3f)
            continue;

        Eigen::Vector2f dxy = new_pos.head<2>() - state.position.head<2>();
        const float dxy_norm = dxy.norm();
        if (dxy_norm > config_.max_pose_jump_xy)
        {
            dxy *= (config_.max_pose_jump_xy / std::max(1e-6f, dxy_norm));
            new_pos.head<2>() = state.position.head<2>() + dxy;
        }

        float dyaw = new_yaw - state.yaw_rad;
        while (dyaw > static_cast<float>(M_PI)) dyaw -= static_cast<float>(2.0 * M_PI);
        while (dyaw < static_cast<float>(-M_PI)) dyaw += static_cast<float>(2.0 * M_PI);
        dyaw = std::clamp(dyaw, -config_.max_pose_jump_yaw, config_.max_pose_jump_yaw);

        const float pose_delta = (new_pos.head<2>() - state.position.head<2>()).norm() + std::abs(dyaw);
        max_pose_delta = std::max(max_pose_delta, pose_delta);

        state.position = new_pos;
        state.yaw_rad += dyaw;
        while (state.yaw_rad > static_cast<float>(M_PI)) state.yaw_rad -= static_cast<float>(2.0 * M_PI);
        while (state.yaw_rad < static_cast<float>(-M_PI)) state.yaw_rad += static_cast<float>(2.0 * M_PI);
    }

    metrics_.objective += 0.001f * max_pose_delta;

    ++em_iteration_;

    return true;
}

bool ObjectOwnershipEM::run_em(int max_iters)
{
    if (max_iters <= 0)
        max_iters = config_.max_iterations;

    em_iteration_ = 0;

    bool ok = false;
    float prev_objective = std::numeric_limits<float>::infinity();
    for (int iter = 0; iter < max_iters; ++iter)
    {
        ok = run_single_iteration();
        if (!ok)
            return false;

        if (std::isfinite(prev_objective) &&
            std::abs(prev_objective - metrics_.objective) < config_.convergence_eps)
            break;
        prev_objective = metrics_.objective;
    }
    return ok;
}

const std::vector<ObjectOwnershipEM::ClassState>& ObjectOwnershipEM::get_states() const
{
    return states_;
}

const ObjectOwnershipEM::OwnershipMatrix& ObjectOwnershipEM::get_ownership() const
{
    return ownership_;
}

const ObjectOwnershipEM::Metrics& ObjectOwnershipEM::get_metrics() const
{
    return metrics_;
}

} // namespace rc
