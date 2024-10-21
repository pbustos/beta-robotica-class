//
// Created by pbustos on 9/10/22.
//

#ifndef ROOM_H
#define ROOM_H

#include <QtCore>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace rc
{
    class Room
    {
        public:
            //        void update(const QSizeF &new_size, const Eigen::Vector2f &center_, float rot_);
            //        void update(const QSizeF &new_size, const Eigen::Vector2f &center_);
            void update(const QPointF &p1, const QPointF &p2, const QPointF &p3, const QPointF &p4);
            float get_width() const;
            float get_depth() const;
            float get_height() const;
            Eigen::Vector2f get_center() const;
            float get_center_x() const;
            float get_center_y() const;
            float get_rotation() const;
            Eigen::Vector2f get_closest_corner(const Eigen::Vector2f &c);
            Eigen::Vector2f get_closest_corner_in_robot_coor(const Eigen::Vector2f &c);
            Eigen::Vector2f get_closest_corner_in_robot_coor2(const Eigen::Vector2f &c) const;
            std::pair<float, QLineF> get_closest_side(const QLineF &line);
            std::vector<QLineF> get_room_lines_qt() const;
            QPolygonF get_qt_polygon() const;
            Eigen::Vector2f to_room_coor(const Eigen::Vector2f &p) const;
            Eigen::Matrix<float, 4, 2> get_corners_mat() const;
            std::vector<Eigen::Vector2f> get_corners() const;
            std::vector<Eigen::Vector3f> get_3d_corners_in_robot_coor();
            Eigen::Vector2f to_local_coor(const Eigen::Vector2f &p);
            void rotate(float delta);  //degrees
            void print();
            void draw_2D(const Room &room, const QString &color, QGraphicsScene *scene);
            bool is_valid() const { return is_initialized; };
            void set_valid(bool v) { is_initialized = v; };
            float get_minX() const;           // returns the minimum x value of the room
            float get_minY() const;
            float get_maxX() const;
            float get_maxY() const;

    private:
            cv::RotatedRect rect;
            Eigen::Vector2f center = {0.f, 0.f};    // mm
            float rot = 0.f; // radians
            QSizeF rsize = {0.f, 0.f}; // mm
            bool is_initialized = false;
            [[nodiscard]] float size_dist(const QSizeF &p1, const QSizeF &p2) const;
            float euc_distance_between_points(const QPointF &p1, const QPointF &p2) const;
            float euc_distance_between_points(const cv::Point2f &p1, const QPointF &p2) const;
            int csign = 1;
            float delta = 0.1;
            QSizeF tmp_size = {0.f, 0.f};
            float size_confidence = -10.f;
            void compute_corners();
            QPointF to_qpoint(const cv::Point2f &p) const;

    };
} //rc
#endif //ROOM_H
