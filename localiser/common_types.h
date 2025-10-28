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

enum class STATE
{
    START_SEARCH, MOVE_TO_CENTER, AT_CENTER, MOVE_ODOM_FRAME_TO_TARGET, MOVE_PHANTOMS_TO_TARGET,
    SELECT_NEW_TARGET, INITIAL_ACCUMULATION, MOVE_AROUND_AFFORDANCE, IDLE, INITIALIZE, SAMPLE_ROOMS, PROCESS_ROOMS
};

using RetVal = std::tuple<STATE, double, double>;
using RobotSpeed = std::tuple<double, double>;
using Boost_Circular_Buffer = boost::circular_buffer<Eigen::Matrix<double, 3, Eigen::Dynamic>>;
using LidarPoints = std::vector<Eigen::Vector3d>;   // 3D points with projective coordinates

// nominal_corners, measurement_corners_in_room, norm of measurement_corner_in_robot, angle of measurement_corner_in_robot, error

using Target = Eigen::Vector3d;

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
};
using Lines = std::vector<LineSegment>;
using Par_lines = std::vector<std::pair<LineSegment, LineSegment>>;
using Corner = std::tuple<QPointF, double, long>; // corner point, angle wrt coordinate axes, timestamp
using Corners =  std::vector<Corner>;
using All_Corners = std::vector<std::tuple<QPointF, QPointF, QPointF, QPointF>>;
using Features = std::tuple<Lines, Par_lines, Corners, All_Corners>;
using Center = std::pair<QPointF, int>;  // center of a polygon and number of votes

using Match = std::vector<std::tuple<Corner, Corner, double>>;  //  measurement - nominal - error Both must be in the same reference system

#endif //COMMON_TYPES_H
