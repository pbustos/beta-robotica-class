//
// Created by pbustos on 9/12/24.
//

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <vector>
#include <QPointF>
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <QLineF>
#include <Eigen/src/Geometry/ParametrizedLine.h>


// types for the features
struct LineSegment
{
    Eigen::Vector2d start;
    Eigen::Vector2d end;
    Eigen::Vector2d direction;
    std::vector<int> inlier_indices;
    int num_inliers;
    double score;
    [[nodiscard]] QLineF toQLineF() const { return QLineF{QPointF{start.x(), start.y()}, QPointF{end.x(), end.y()}};}
    [[nodiscard]] Eigen::ParametrizedLine<double, 2> toEigenLine() const { return Eigen::ParametrizedLine<double, 2>(start, direction);};
    [[nodiscard]] Eigen::Vector2d midpoint() const { return (start + end) / 2.0; }
    [[nodiscard]] double length() const { return (end - start).norm(); }
    [[nodiscard]] Eigen::Vector3d to_general_form() const
    {
        const auto line = toQLineF();
        // A = y1 - y2
        auto A = line.p1().y() - line.p2().y();
        // B = x2 - x1
        auto B = line.p2().x() - line.p1().x();
        // C = -Ax1 - By1
        // Using p1 to solve for C. You could also use p2.
        auto C = -A * line.p1().x() - B * line.p1().y();
        return{ A, B, C };
    };
};
using Lines = std::vector<LineSegment>;
using Par_lines = std::vector<std::pair<LineSegment, LineSegment>>;
using Corner = std::tuple<QPointF, double, long>; // corner point, angle wrt coordinate axes, timestamp
using Corners =  std::vector<Corner>;
using All_Corners = std::vector<std::tuple<QPointF, QPointF, QPointF, QPointF>>;
using Features = std::tuple<Lines, Par_lines, Corners, All_Corners>;
using Center = std::pair<QPointF, int>;  // center of a polygon and number of votes
using Match = std::vector<std::tuple<Corner, Corner, double>>;  //  measurement - nominal - error Both must be in the same reference system
using Peaks = std::vector<std::tuple<Eigen::Vector2f, float>>; // 2D points representing peaks with angle wrt robot frame
struct Door
{
    Eigen::Vector2f p1;
    float p1_angle;
    Eigen::Vector2f p2;
    float p2_angle;
    [[nodiscard]] float width() const { return (p2 - p1).norm(); }
    [[nodiscard]] Eigen::Vector2f center() const { return 0.5f * (p1 + p2); }
    [[nodiscard]] Eigen::Vector2f center_before(const Eigen::Vector2d &robot_pos, float offset = 500.f) const   // a point 500mm before the center along the door direction
    {
        // computer the normal to the door direction pointing towards the robot
        Eigen::Vector2f dir = p2 - p1;
        const float dir_norm = dir.norm();
        if (dir_norm == 0.f)
            return center(); // degenerate door, return center
        dir /= dir_norm;
        // perpendicular (normal) to door direction
        Eigen::Vector2f normal(-dir.y(), dir.x());
        // choose the normal that points toward the robot
        const Eigen::Vector2f to_robot = robot_pos.cast<float>() - center();
        if (to_robot.dot(normal) < 0.f)
            normal = -normal;
        Eigen::Vector2f before = center() + offset * normal;
        return before;
    }
    [[nodiscard]] float direction() const
    {
        Eigen::Vector2f dir = p2 - p1;
        return std::atan2(dir.y(), dir.x());
    }
    Door(Eigen::Vector2f point1, const float angle1, Eigen::Vector2f point2, const float angle2)
    {
        // Calculate angular difference both ways
        float diff_forward = angle2 - angle1;
        float diff_backward = angle1 - angle2;

        // Normalize differences to [0, 2π)
        if (diff_forward < 0) diff_forward += 2 * M_PI;
        if (diff_backward < 0) diff_backward += 2 * M_PI;

        // Choose ordering that gives smaller angular span
        if (diff_forward <= diff_backward)
        {
            p1 = point1; p1_angle = angle1;
            p2 = point2; p2_angle = angle2;
        } else
        {
            p1 = point2; p1_angle = angle2;
            p2 = point1; p2_angle = angle1;
        }
    }
};
using Doors = std::vector<Door>;
#endif //COMMON_TYPES_H
