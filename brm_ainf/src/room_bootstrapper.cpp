#include "room_bootstrapper.h"

#include <algorithm>
#include <cmath>
#include <chrono>

namespace rc
{
RoomBootstrapper::RoomBootstrapper()
    : RoomBootstrapper(Params{})
{
}

RoomBootstrapper::RoomBootstrapper(const Params &params)
    : params_(params), estimator_(PointcloudCenterEstimator::Config{})
{
    rebuild_estimator();
}

void RoomBootstrapper::set_params(const Params &params)
{
    params_ = params;
    rebuild_estimator();
    reset();
}

void RoomBootstrapper::reset()
{
    accumulated_frames_ = 0;
    accumulated_points_.clear();
    last_lidar_timestamp_ms_ = 0;
    accumulated_pose_.setZero();
}

std::optional<RoomBootstrapper::Result> RoomBootstrapper::consume(
    const std::vector<Eigen::Vector3f> &lidar_points,
    std::int64_t lidar_timestamp_ms,
    const std::vector<OdometryReading> &odometry_history)
{
    if (params_.use_odometry_compensation)
    {
        if (last_lidar_timestamp_ms_ != 0 && lidar_timestamp_ms > last_lidar_timestamp_ms_)
        {
            const auto delta = integrate_odometry_between(
                odometry_history,
                last_lidar_timestamp_ms_,
                lidar_timestamp_ms,
                accumulated_pose_[2]);
            accumulated_pose_ += delta;
            while (accumulated_pose_[2] > static_cast<float>(M_PI)) accumulated_pose_[2] -= static_cast<float>(2.0 * M_PI);
            while (accumulated_pose_[2] < static_cast<float>(-M_PI)) accumulated_pose_[2] += static_cast<float>(2.0 * M_PI);
        }
        last_lidar_timestamp_ms_ = lidar_timestamp_ms;
    }

    ++accumulated_frames_;

    const std::size_t stride = std::max<std::size_t>(
        1, lidar_points.size() / std::max(1, params_.accumulation_target_points_per_frame));

    const float x = accumulated_pose_[0];
    const float y = accumulated_pose_[1];
    const float theta = accumulated_pose_[2];
    const float c = std::cos(theta);
    const float s = std::sin(theta);

    for (std::size_t i = 0; i < lidar_points.size(); i += stride)
    {
        const auto &p = lidar_points[i];
        if (params_.use_odometry_compensation)
        {
            accumulated_points_.emplace_back(
                c * p.x() - s * p.y() + x,
                s * p.x() + c * p.y() + y,
                p.z());
        }
        else
        {
            accumulated_points_.push_back(p);
        }
    }

    if (accumulated_frames_ < params_.min_frames)
        return std::nullopt;

    if (static_cast<int>(accumulated_points_.size()) < params_.min_points)
        return std::nullopt;

    const auto obb_opt = estimator_.estimate_obb(accumulated_points_);
    if (!obb_opt.has_value())
        return std::nullopt;

    const auto &obb = obb_opt.value();

    Result result;
    result.width = std::clamp(
        params_.span_scale * static_cast<float>(obb.width) + params_.span_margin,
        params_.min_room_dim,
        params_.max_room_dim);
    result.length = std::clamp(
        params_.span_scale * static_cast<float>(obb.height) + params_.span_margin,
        params_.min_room_dim,
        params_.max_room_dim);
    result.obb_rotation = static_cast<float>(obb.rotation);
    result.points_used = accumulated_points_.size();
    result.frames_used = accumulated_frames_;

    return result;
}

Eigen::Vector3f RoomBootstrapper::integrate_odometry_between(
    const std::vector<OdometryReading> &odometry_history,
    std::int64_t t_start_ms,
    std::int64_t t_end_ms,
    float theta_start) const
{
    if (odometry_history.empty() || t_end_ms <= t_start_ms)
        return Eigen::Vector3f::Zero();

    using clock = std::chrono::high_resolution_clock;
    const auto t_start = clock::time_point(std::chrono::milliseconds(t_start_ms));
    const auto t_end = clock::time_point(std::chrono::milliseconds(t_end_ms));

    Eigen::Vector3f total = Eigen::Vector3f::Zero();
    float theta = theta_start;

    for (std::size_t i = 0; i < odometry_history.size(); ++i)
    {
        const auto &odom = odometry_history[i];
        auto seg_start = odom.timestamp;
        auto seg_end = (i + 1 < odometry_history.size()) ? odometry_history[i + 1].timestamp : t_end;

        if (seg_end < t_start) continue;
        if (seg_start > t_end) break;

        const auto effective_start = std::max(seg_start, t_start);
        const auto effective_end = std::min(seg_end, t_end);
        const float dt = std::chrono::duration<float>(effective_end - effective_start).count();
        if (dt <= 0.f) continue;

        const float dx_local = odom.side * dt;
        const float dy_local = odom.adv * dt;
        const float dtheta = -odom.rot * dt;

        total[0] += dx_local * std::cos(theta) - dy_local * std::sin(theta);
        total[1] += dx_local * std::sin(theta) + dy_local * std::cos(theta);
        total[2] += dtheta;
        theta += dtheta;
    }

    return total;
}

void RoomBootstrapper::rebuild_estimator()
{
    PointcloudCenterEstimator::Config cfg;
    cfg.num_sectors = params_.estimator_num_sectors;
    cfg.max_range = params_.estimator_max_range;
    cfg.min_range = params_.estimator_min_range;
    cfg.outlier_std_threshold = params_.estimator_outlier_std_threshold;
    cfg.min_valid_points = params_.estimator_min_valid_points;
    estimator_ = PointcloudCenterEstimator(cfg);
}
} // namespace rc
