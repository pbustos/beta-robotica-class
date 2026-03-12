#include "specificworker.h"

#include <QLabel>
#include <QDebug>

#include <algorithm>
#include <cmath>
#include <numeric>

float SpecificWorker::percentile_copy(std::vector<float> values, float q)
{
    if (values.empty())
        return 0.f;
    q = std::clamp(q, 0.f, 1.f);
    const std::size_t idx = static_cast<std::size_t>(q * static_cast<float>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + static_cast<long>(idx), values.end());
    return values[idx];
}

void SpecificWorker::start_episode(const std::string &mission_type,
                                   const std::optional<Eigen::Vector2f> &target_point,
                                   const std::string &target_object)
{
    if (active_episode_.has_value())
        finish_episode("aborted");

    rc::EpisodicMemory::EpisodeRecord episode;
    episode.episode_id = rc::EpisodicMemory::make_episode_id();
    episode.start_ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    episode.status = "running";

    episode.mission.mission_type = mission_type;
    episode.mission.controller_version = "ainf_slamo_v2";
    episode.target.target_mode = target_point.has_value() ? "point" : (target_object.empty() ? "none" : "object");
    if (target_point.has_value())
    {
        episode.target.target_x = target_point->x();
        episode.target.target_y = target_point->y();
    }
    episode.target.target_object_id = target_object;

    const auto &p = trajectory_controller_.params;
    auto add_param = [&episode](const std::string &key, float value)
    {
        episode.params_snapshot[key] = value;
    };

    add_param("mood", p.mood);
    add_param("max_adv", p.max_adv);
    add_param("max_back_adv", p.max_back_adv);
    add_param("max_rot", p.max_rot);
    add_param("d_safe", p.d_safe);
    add_param("safety_priority_scale", p.safety_priority_scale);
    add_param("carrot_lookahead", p.carrot_lookahead);
    add_param("goal_threshold", p.goal_threshold);
    add_param("num_samples", static_cast<float>(p.num_samples));
    add_param("trajectory_steps", static_cast<float>(p.trajectory_steps));
    add_param("trajectory_dt", p.trajectory_dt);
    add_param("mppi_lambda", p.mppi_lambda);
    add_param("sigma_adv", p.sigma_adv);
    add_param("sigma_rot", p.sigma_rot);
    add_param("noise_alpha", p.noise_alpha);
    add_param("warm_start_adv_weight", p.warm_start_adv_weight);
    add_param("warm_start_rot_weight", p.warm_start_rot_weight);
    add_param("optim_iterations", static_cast<float>(p.optim_iterations));
    add_param("optim_lr", p.optim_lr);
    add_param("lambda_goal", p.lambda_goal);
    add_param("lambda_obstacle", p.lambda_obstacle);
    add_param("lambda_smooth", p.lambda_smooth);
    add_param("lambda_velocity", p.lambda_velocity);
    add_param("lambda_delta_vel", p.lambda_delta_vel);
    add_param("velocity_smoothing", p.velocity_smoothing);
    add_param("gauss_k", p.gauss_k);

    episode.mood_snapshot["mood"] = p.mood;
    episode.mood_snapshot["enable_mood"] = p.enable_mood ? 1.f : 0.f;
    episode.mood_snapshot["mood_speed_gain"] = p.mood_speed_gain;
    episode.mood_snapshot["mood_reactivity_gain"] = p.mood_reactivity_gain;
    episode.mood_snapshot["mood_caution_gain"] = p.mood_caution_gain;

    active_episode_ = std::move(episode);
    episode_accum_ = EpisodeAccum{};
    episode_accum_.last_block_sample = std::chrono::steady_clock::now();
    episode_accum_.last_metric_sample = episode_accum_.last_block_sample;
    episode_accum_.has_last_metric_sample = true;
    if (target_point.has_value())
    {
        const auto state = get_loc_state();
        episode_accum_.start_to_goal_dist_m = std::hypot(target_point->x() - static_cast<float>(state[2]),
                                                         target_point->y() - static_cast<float>(state[3]));
    }

    if (label_episodeStatus != nullptr)
    {
        const QString id = QString::fromStdString(active_episode_->episode_id);
        label_episodeStatus->setText("EP: REC " + id.right(6));
        label_episodeStatus->setStyleSheet("background-color: #B3E5FC; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
    }
}

void SpecificWorker::update_episode_metrics(const rc::RoomConceptAI::UpdateResult &res,
                                            const rc::TrajectoryController::ControlOutput *ctrl,
                                            float current_speed,
                                            float current_rot,
                                            float cpu_usage,
                                            float mppi_ms,
                                            bool blocked_state)
{
    if (!active_episode_.has_value())
        return;

    episode_accum_.n_cycles++;

    const auto now = std::chrono::steady_clock::now();
    float dt_s = 0.05f;
    if (episode_accum_.has_last_metric_sample)
        dt_s = std::max(1e-3f, std::chrono::duration<float>(now - episode_accum_.last_metric_sample).count());
    episode_accum_.last_metric_sample = now;
    episode_accum_.has_last_metric_sample = true;

    const Eigen::Vector2f pos(res.state[2], res.state[3]);
    if (episode_accum_.has_prev_pose)
    {
        const float raw_delta = (pos - episode_accum_.prev_pos).norm();
        const float max_cmd_delta = std::max(trajectory_controller_.params.max_adv, 0.01f) * dt_s;
        const float max_allowed_delta = 1.35f * max_cmd_delta + 0.01f;
        const float jitter_deadband = 0.002f;
        const float bounded_delta = std::clamp(raw_delta - jitter_deadband, 0.f, max_allowed_delta);
        episode_accum_.distance_traveled_m += bounded_delta;
    }
    episode_accum_.prev_pos = pos;
    episode_accum_.has_prev_pose = true;

    if (ctrl != nullptr)
    {
        episode_accum_.min_esdf_m = std::min(episode_accum_.min_esdf_m, ctrl->min_esdf);
        episode_accum_.ess_ratio_samples.push_back((ctrl->ess_K > 0) ? ctrl->ess / static_cast<float>(ctrl->ess_K) : 0.f);
    }

    episode_accum_.speed_samples.push_back(current_speed);
    episode_accum_.rot_samples.push_back(current_rot);
    episode_accum_.cpu_samples.push_back(cpu_usage);
    episode_accum_.mppi_ms_samples.push_back(mppi_ms);

    if (blocked_state)
    {
        if (!episode_accum_.was_blocked)
            episode_accum_.n_blocked_events++;
        const float dt = std::chrono::duration<float>(now - episode_accum_.last_block_sample).count();
        episode_accum_.blocked_time_s += std::max(0.f, dt);
        episode_accum_.was_blocked = true;
    }
    else
    {
        episode_accum_.was_blocked = false;
    }
    episode_accum_.last_block_sample = now;
}

void SpecificWorker::finish_episode(const std::string &status)
{
    if (!active_episode_.has_value())
        return;

    const std::string finished_episode_id = active_episode_->episode_id;

    auto &episode = active_episode_.value();
    episode.end_ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    episode.duration_s = std::max(0.f, static_cast<float>(episode.end_ts_ms - episode.start_ts_ms) / 1000.f);
    episode.status = status;

    episode.trajectory.n_cycles = episode_accum_.n_cycles;
    episode.trajectory.distance_traveled_m = episode_accum_.distance_traveled_m;

    const float target_dist = episode_accum_.start_to_goal_dist_m;
    episode.trajectory.path_efficiency = (episode_accum_.distance_traveled_m > 1e-6f)
        ? (target_dist / episode_accum_.distance_traveled_m)
        : 0.f;

    episode.trajectory.mean_speed = episode_accum_.speed_samples.empty() ? 0.f
        : std::accumulate(episode_accum_.speed_samples.begin(), episode_accum_.speed_samples.end(), 0.f)
          / static_cast<float>(episode_accum_.speed_samples.size());
    episode.trajectory.p95_speed = percentile_copy(episode_accum_.speed_samples, 0.95f);
    episode.trajectory.mean_rot = episode_accum_.rot_samples.empty() ? 0.f
        : std::accumulate(episode_accum_.rot_samples.begin(), episode_accum_.rot_samples.end(), 0.f)
          / static_cast<float>(episode_accum_.rot_samples.size());
    episode.trajectory.p95_rot = percentile_copy(episode_accum_.rot_samples, 0.95f);
    episode.trajectory.mean_ess_ratio = episode_accum_.ess_ratio_samples.empty() ? 0.f
        : std::accumulate(episode_accum_.ess_ratio_samples.begin(), episode_accum_.ess_ratio_samples.end(), 0.f)
          / static_cast<float>(episode_accum_.ess_ratio_samples.size());
    episode.trajectory.p05_ess_ratio = percentile_copy(episode_accum_.ess_ratio_samples, 0.05f);
    episode.trajectory.mean_cpu_pct = episode_accum_.cpu_samples.empty() ? 0.f
        : std::accumulate(episode_accum_.cpu_samples.begin(), episode_accum_.cpu_samples.end(), 0.f)
          / static_cast<float>(episode_accum_.cpu_samples.size());
    episode.trajectory.p95_mppi_ms = percentile_copy(episode_accum_.mppi_ms_samples, 0.95f);

    episode.safety.min_esdf_m = std::isfinite(episode_accum_.min_esdf_m) ? episode_accum_.min_esdf_m : 0.f;
    episode.safety.n_near_collision = 0;
    episode.safety.n_collision = (episode.safety.min_esdf_m < trajectory_controller_.params.robot_radius) ? 1 : 0;
    episode.safety.n_blocked_events = episode_accum_.n_blocked_events;
    episode.safety.blocked_time_s = episode_accum_.blocked_time_s;
    episode.safety.n_replans = 0;

    episode.outcome.time_to_goal_s = episode.duration_s;
    episode.outcome.success_binary = (status == "success") ? 1 : 0;
    episode.outcome.comfort_jerk_score = episode.trajectory.p95_rot;
    episode.outcome.safety_score = std::max(0.f, episode.safety.min_esdf_m);
    episode.outcome.efficiency_score = std::max(0.f, episode.trajectory.path_efficiency);
    episode.outcome.composite_score = 100.f * static_cast<float>(episode.outcome.success_binary)
                                    - episode.outcome.time_to_goal_s
                                    - 50.f * static_cast<float>(episode.safety.n_collision)
                                    - 2.f * episode.safety.blocked_time_s;

    if (!episodic_memory_.save_episode(episode))
        qWarning() << "Failed to save episode" << QString::fromStdString(episode.episode_id);

    if (label_episodeStatus != nullptr)
    {
        const QString qstatus = QString::fromStdString(status).toUpper();
        label_episodeStatus->setText("EP: " + qstatus + " " + QString::fromStdString(finished_episode_id).right(6));
        const bool ok = (status == "success");
        label_episodeStatus->setStyleSheet(ok
            ? "background-color: #C8E6C9; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;"
            : "background-color: #FFE0B2; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
    }

    active_episode_.reset();
    episode_accum_ = EpisodeAccum{};
}
