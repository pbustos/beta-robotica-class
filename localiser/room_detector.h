//
// Created by pbustos on 2/12/22.
//

#ifndef FORCEFIELD_ROOM_DETECTOR_H
#define FORCEFIELD_ROOM_DETECTOR_H

#include <vector>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <QtCore>
#include <ranges>
#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>
#include "ransac_line_detector.h"

// Room_Detector class is a class that detects rooms in a 2D space. It uses a set of Hough lines to detect the rooms.
// The class has a method detect that receives a set of lines and returns a Room object.
// The class has a method compute_features that receives a set of lines and returns a tuple with the following features:
// 1. A set of lines
// 2. A set of parallel lines
// 3. A set of corners
// 4. A set of rooms

namespace rc   // aka RoboComp
{
    class Room_Detector
    {
        public:
             Corners compute_corners(const std::vector<Eigen::Vector2d> &line, QGraphicsScene *scene= nullptr);
             Corners compute_corners(const std::vector<Eigen::Vector3d> &line, QGraphicsScene *scene= nullptr);
             Corners compute_corners(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene= nullptr);

             Eigen::Vector3d estimate_room_sizes(const Eigen::Vector2d &room_center, std::vector<Eigen::Vector2d> &floor_line_cart);
             Corners get_corners(Lines &elines);
             Lines filter_lines_by_length(const Lines &lines, float threshold );

            // aux
             double euc_distance_between_points(const QPointF &p1, const QPointF &p2);
             QPointF get_most_distant_point(const QPointF &p, const QPointF &p1, const QPointF &p2);
             [[nodiscard]]  std::vector<Center> reorder_points_CCW(const std::vector<Center> &points);

            // draw
             void draw_lines_on_2D_tab(const Lines &lines, QGraphicsScene *scene);
             void draw_corners_on_2D_tab(const Corners &corners, const std::vector<Eigen::Vector2d> &model_corners, QGraphicsScene *scene);

            // local data
             Eigen::Vector2d to_eigen(const QPointF &p);
             Eigen::Vector2d to_eigen(const cv::Point2d &p);
             QPointF to_qpointf(const cv::Point2d &p);
    };


} // rc

#endif //FORCEFIELD_ROOM_DETECTOR_H
