#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

#include "common_types.h"
#include "pointcloud_center_estimator.h"

namespace rc
{
class RoomBootstrapper
{
public:
    struct Params
    {
        int min_frames = 8;
        int min_points = 1200;
        int accumulation_target_points_per_frame = 1500;
        bool use_odometry_compensation = true;

        float min_room_dim = 2.0f;
        float max_room_dim = 30.0f;
        float span_scale = 1.18f;
        float span_margin = 0.35f;

        // Input points are meters in this project.
        float estimator_min_range = 0.2f;
        float estimator_max_range = 30.0f;
        int estimator_num_sectors = 180;
        float estimator_outlier_std_threshold = 2.5f;
        std::size_t estimator_min_valid_points = 50;
    };

    struct Result
    {
        float width = 0.f;
        float length = 0.f;
        float obb_rotation = 0.f;
        std::size_t points_used = 0;
        int frames_used = 0;
    };

    RoomBootstrapper();
    explicit RoomBootstrapper(const Params &params);

    void set_params(const Params &params);
    void reset();
    const std::vector<Eigen::Vector3f>& accumulated_points() const { return accumulated_points_; }

    std::optional<Result> consume(const std::vector<Eigen::Vector3f> &lidar_points,
                                  std::int64_t lidar_timestamp_ms,
                                  const std::vector<OdometryReading> &odometry_history);

private:
    void rebuild_estimator();
    Eigen::Vector3f integrate_odometry_between(
        const std::vector<OdometryReading> &odometry_history,
        std::int64_t t_start_ms,
        std::int64_t t_end_ms,
        float theta_start) const;

    Params params_;
    PointcloudCenterEstimator estimator_;

    int accumulated_frames_ = 0;
    std::vector<Eigen::Vector3f> accumulated_points_;
    std::int64_t last_lidar_timestamp_ms_ = 0;
    Eigen::Vector3f accumulated_pose_ = Eigen::Vector3f::Zero(); // [x, y, theta] in bootstrap reference frame
};
} // namespace rc
