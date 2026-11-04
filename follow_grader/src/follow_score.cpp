#include "follow_score.h"

#include <algorithm>
#include <cmath>

FollowScore::FollowScore(double d_ref) : d_ref_(d_ref) {}

void FollowScore::reset()
{
    samples_ = 0;
    sum_dist_ = sum_angle_ = sum_sq_dist_ = sum_sq_angle_ = max_dist_ = max_angle_ = 0.0;
}

double FollowScore::yaw(const RoboCompWebots2Robocomp::Quaternion& q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

FollowScore::Sample FollowScore::add(const RoboCompWebots2Robocomp::ObjectPose& robot,
                                     const RoboCompWebots2Robocomp::ObjectPose& person)
{
    // person in the robot frame: the Shadow's local +y is forward and +x is to its right (checked against its motion)
    const double dx = person.position.x - robot.position.x, dy = person.position.y - robot.position.y;
    const double th = yaw(robot.orientation);
    const double x = dx * std::cos(th) + dy * std::sin(th);
    const double y = -dx * std::sin(th) + dy * std::cos(th);

    const Sample s{std::hypot(x, y), std::atan2(x, y)};
    const double ed = std::abs(s.distance - d_ref_), ea = std::abs(s.angle);
    sum_dist_ += ed;
    sum_angle_ += ea;
    sum_sq_dist_ += ed * ed;
    sum_sq_angle_ += ea * ea;
    max_dist_ = std::max(max_dist_, ed);
    max_angle_ = std::max(max_angle_, ea);
    ++samples_;
    return s;
}

double FollowScore::variance(double sum, double sum_sq) const
{
    if (samples_ < 1) return 0.0;
    const double mean = sum / samples_;
    return std::max(0.0, sum_sq / samples_ - mean * mean);
}
