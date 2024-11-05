//
// Created by pbustos on 2/12/22.
//

#ifndef FORCEFIELD_ROOM_DETECTOR_H
#define FORCEFIELD_ROOM_DETECTOR_H

#include <vector>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <QtCore>
#include "room.h"
#include <cmath>
#include <ranges>

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
        // types for the features
        using Lines = std::vector<std::pair<int, QLineF>>;
        using Par_lines = std::vector<std::pair<QLineF, QLineF>>;
        using Corners =  std::vector<std::tuple<int, QPointF>>;
        using All_Corners = std::vector<std::tuple<QPointF, QPointF, QPointF, QPointF>>;
        using Features = std::tuple<Lines, Par_lines, Corners, All_Corners>;

        public:
            Room detect(const std::vector<Eigen::Vector2f> &line, QGraphicsScene *scene=nullptr, bool draw_lines=false);
            Features compute_features(const std::vector<Eigen::Vector2f> &line, QGraphicsScene *scene= nullptr);

        private:
            Eigen::Vector3f estimate_room_sizes(const Eigen::Vector2f &room_center, std::vector<Eigen::Vector2f> &floor_line_cart) const;
            Lines get_hough_lines(std::vector<Eigen::Vector2f> &floor_line_cart) const;
            Par_lines get_parallel_lines(const  Lines &lines, const Eigen::Vector2f &estimated_size);
            Corners get_corners(Lines &elines);
            All_Corners get_rooms(const Eigen::Vector2f &estimated_size, const Corners &corners);
            void filter_lines_by_length(const Lines &lines, std::vector <Eigen::Vector2f> &floor_line_cart);

            // aux
            float euc_distance_between_points(const QPointF &p1, const QPointF &p2) const;
            QPointF get_most_distant_point(const QPointF &p, const QPointF &p1, const QPointF &p2) const;

            // draw
            void draw_par_lines_on_2D_tab(const Par_lines &par_lines, QGraphicsScene *scene);
            void draw_lines_on_2D_tab(const Lines &lines, QGraphicsScene *scene);
            void draw_corners_on_2D_tab(const Corners &corners, const std::vector<Eigen::Vector2f> &model_corners, QGraphicsScene *scene);
            void draw_triple_corners_on_2D_tab(const All_Corners &double_corners, QString color, QGraphicsScene *scene);

            // local data
            Room current_room;

            Eigen::Vector2f to_eigen(const QPointF &p) const;
            Eigen::Vector2f to_eigen(const cv::Point2f &p) const;
            QPointF to_qpointf(const cv::Point2f &p) const;

    };

} // rc

#endif //FORCEFIELD_ROOM_DETECTOR_H
