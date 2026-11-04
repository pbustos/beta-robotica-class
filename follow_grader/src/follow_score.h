#ifndef FOLLOW_SCORE_H
#define FOLLOW_SCORE_H

#include <cmath>
#include <Webots2Robocomp.h>

// Accumulates how well the robot follows the person, from ground-truth Webots poses (world frame, mm).
// Per sample: distance error |d - d_ref| (mm, centre to centre) and heading error |person_angle| (rad), where
// person_angle = atan2(x, y) of the person in the robot frame (x right, y forward; + = person to the right),
// the same convention as the students' person_angle.
class FollowScore
{
    public:
        struct Sample
        {
            double distance = 0.0;      // mm, robot centre to person centre
            double angle = 0.0;         // rad, person_angle
        };

        explicit FollowScore(double d_ref = 1000.0);

        Sample add(const RoboCompWebots2Robocomp::ObjectPose& robot, const RoboCompWebots2Robocomp::ObjectPose& person);
        void reset();

        int samples() const { return samples_; }
        double sum_distance_error() const { return sum_dist_; }     // mm
        double sum_angle_error() const { return sum_angle_; }       // rad
        double mean_distance_error() const { return samples_ > 0 ? sum_dist_ / samples_ : 0.0; }
        double mean_angle_error() const { return samples_ > 0 ? sum_angle_ / samples_ : 0.0; }
        double max_distance_error() const { return max_dist_; }     // mm
        double max_angle_error() const { return max_angle_; }       // rad
        // population variances of the same absolute errors the means average (mm^2, rad^2)
        double var_distance_error() const { return variance(sum_dist_, sum_sq_dist_); }
        double var_angle_error() const { return variance(sum_angle_, sum_sq_angle_); }
        double std_distance_error() const { return std::sqrt(var_distance_error()); }    // mm
        double std_angle_error() const { return std::sqrt(var_angle_error()); }          // rad
        double d_ref() const { return d_ref_; }

    private:
        // rotation about the world z axis of a quaternion (the robot only yaws)
        static double yaw(const RoboCompWebots2Robocomp::Quaternion& q);
        double variance(double sum, double sum_sq) const;

        double d_ref_;
        int samples_ = 0;
        double sum_dist_ = 0.0;
        double sum_angle_ = 0.0;
        double sum_sq_dist_ = 0.0;
        double sum_sq_angle_ = 0.0;
        double max_dist_ = 0.0;
        double max_angle_ = 0.0;
};

#endif // FOLLOW_SCORE_H
