//
// Created by pbustos on 9/10/22.
//

#include "room.h"
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>
#include "cppitertools/combinations.hpp"

namespace rc
{
    void Room::update(const QPointF &p1, const QPointF &p2, const QPointF &p3, const QPointF &p4)
    {
        // Try combination of 3 points until a valid rectangle is found
        std::vector<cv::Point2d> points{cv::Point2d(p1.x(), p1.y()),
                                        cv::Point2d(p2.x(), p2.y()),
                                        cv::Point2d(p3.x(), p3.y()),
                                        cv::Point2d(p4.x(), p4.y())};
        for (auto &&c: iter::combinations(points, 3))
        {
            try
            {
                rect = cv::RotatedRect(c[0], c[1], c[2]);
                return; //found
            }
            catch (const std::exception &e)
            {};
        }
    }
//    void Room::update(const QSizeF &new_size, const Eigen::Vector2f &center_, float rot_)
//    {
//        rect = cv::RotatedRect(cv::Point2f(center_.x(), center.y()), cv::Size2f(new_size.width(), new_size.height()), qRadiansToDegrees(rot_));
//        center = center_;
//        rot = rot_;
//        if (size_dist(new_size, tmp_size) < 500)
//            size_confidence = std::clamp(size_confidence + csign * delta, -10.f, 10.f);
//        else
//        {
//            tmp_size = new_size;
//            csign = -csign;
//            size_confidence = std::clamp(size_confidence + csign * delta, -10.f, 10.f);
//        }
//        double res = 1.0 / (1.0 + exp(-size_confidence));
//        if (res > 0.9 or res < 0.1)
//            rsize = tmp_size;
//
//        this->is_initialized = true;
//    }
//    void Room::update(const QSizeF &new_size, const Eigen::Vector2f &center_)
//    {
//        center = center_;
//        if (size_dist(new_size, tmp_size) < 500)
//            size_confidence = std::clamp(size_confidence + csign * delta, -10.f, 10.f);
//        else
//        {
//            tmp_size = new_size;
//            csign = -csign;
//            size_confidence = std::clamp(size_confidence + csign * delta, -10.f, 10.f);
//        }
//        double res = 1.0 / (1.0 + exp(-size_confidence));
//        if (res > 0.9 or res < 0.1)
//            rsize = tmp_size;
//    }

    float Room::size_dist(const QSizeF &p1, const QSizeF &p2) const
    {
        return sqrt((p1.width() - p2.width()) * (p1.width() - p2.width()) + (p1.height() - p2.height()) * (p1.height() - p2.height()));
    }
    std::vector<Eigen::Vector3f> Room::get_3d_corners_in_robot_coor()
    {
        cv::Point2f pts[4];
        rect.points(pts);
        std::vector<Eigen::Vector3f> rcorners{Eigen::Vector3f(pts[0].x, pts[0].y, 0.f),
                                              Eigen::Vector3f(pts[1].x, pts[1].y, 0.f),
                                              Eigen::Vector3f(pts[2].x, pts[2].y, 0.f),
                                              Eigen::Vector3f(pts[3].x, pts[3].y, 0.f)};
        return rcorners;
    }
    Eigen::Matrix<float, 4, 2> Room::get_corners_mat() const
    {
        cv::Point2f pts[4];
        rect.points(pts);
        Eigen::Matrix<float, 4, 2> rcorners;
        rcorners << Eigen::Vector2f(pts[0].x, pts[0].y), Eigen::Vector2f(pts[1].x, pts[1].y),
                Eigen::Vector2f(pts[2].x, pts[2].y), Eigen::Vector2f(pts[3].x, pts[3].y);
        return rcorners;
    }
    std::vector<Eigen::Vector2f> Room::get_corners() const
    {
        cv::Point2f pts[4];
        rect.points(pts);
        std::vector<Eigen::Vector2f> corners(4);
        for (auto &&i: iter::range(4))
            corners[i] = Eigen::Vector2f(pts[i].x, pts[i].y);
        return corners;
    }
    float Room::get_minX() const
    {
        cv::Point2f corners[4];
        rect.points(corners);
        auto min_x_corner = std::ranges::min_element(corners, [](const cv::Point2f &a, const cv::Point2f &b)
                                            { return a.x < b.x; });
        return min_x_corner->x;
    }
    float Room::get_minY() const
    {
        cv::Point2f corners[4];
        rect.points(corners);
        auto min_x_corner = std::ranges::min_element(corners, [](const cv::Point2f &a, const cv::Point2f &b)
        { return a.y < b.y; });
        return min_x_corner->y;
    }
    float Room::get_maxX() const
    {
        cv::Point2f corners[4];
        rect.points(corners);
        auto max_x_corner = std::ranges::min_element(corners, [](const cv::Point2f &a, const cv::Point2f &b)
        { return a.x > b.x; });
        return max_x_corner->x;
    }
    float Room::get_maxY() const
    {
        cv::Point2f corners[4];
        rect.points(corners);
        auto max_x_corner = std::ranges::min_element(corners, [](const cv::Point2f &a, const cv::Point2f &b)
        { return a.y > b.y; });
        return max_x_corner->y;
    }
    Eigen::Vector2f Room::get_closest_corner_in_robot_coor2(const Eigen::Vector2f &c) const
    {
        cv::Point2f pts[4];
        rect.points(pts);
        std::vector<Eigen::Vector2f> cs;
        for (auto &&i: iter::range(4))
            cs.emplace_back(Eigen::Vector2f(pts[i].x, pts[i].y));
        return *std::ranges::min_element(cs, [c](auto &a, auto &b) { return (a - c).norm() < (b - c).norm(); });
    }
    Eigen::Vector2f Room::get_closest_corner_in_robot_coor(const Eigen::Vector2f &c)
    {
        auto corners = get_corners_mat();
        Eigen::Index index;
        (corners.rowwise() - c.transpose()).rowwise().squaredNorm().minCoeff(&index);
        return corners.row(index);
    }
    Eigen::Vector2f Room::to_room_coor(const Eigen::Vector2f &p) const
    {
        Eigen::Rotation2Df rt_from_robot(rot);
        //Eigen::Matrix<float, 2, 2> rt_from_robot;
        //rt_from_robot << cos(rot), -sin(rot) , sin(rot), cos(rot);
        Eigen::Vector2f res = rt_from_robot.inverse() * (p - center);
        return res;
    }
    Eigen::Vector2f Room::get_closest_corner(const Eigen::Vector2f &c)
    {
        // c should be in rooms's coordinates
        auto corners = get_corners_mat();
        Eigen::Index index;
        (corners.rowwise() - c.transpose()).rowwise().squaredNorm().minCoeff(&index);
        return corners.row(index);
    }
    std::pair<float, QLineF> Room::get_closest_side(const QLineF &l)
    {
        //auto sides = get_sides_in_robot_coor();
        std::vector<std::pair<float, QLineF>> distances;
        cv::Point2f pts[4];
        rect.points(pts);
        float signed_distance;
        for (auto &&p: iter::range(4) | iter::sliding_window(2))
        {
            QLineF line(pts[p[0]].x, pts[p[0]].y, pts[p[1]].x, pts[p[1]].y);
            // if tested line is further than model line, distance is positive
            if (euc_distance_between_points(rect.center, line.center()) < euc_distance_between_points(rect.center, l.center()))
                signed_distance = euc_distance_between_points(line.center(), l.center());
            else
                signed_distance = -euc_distance_between_points(line.center(), l.center());
            distances.emplace_back(std::make_pair(signed_distance, l));
        }
        // last with first
        QLineF line(pts[3].x, pts[3].y, pts[0].x, pts[0].y);
        if (euc_distance_between_points(rect.center, line.center()) < euc_distance_between_points(rect.center, l.center()))
            signed_distance = euc_distance_between_points(line.center(), l.center());
        else
            signed_distance = -euc_distance_between_points(line.center(), l.center());
        distances.emplace_back(std::make_pair(euc_distance_between_points(line.center(), l.center()), l));

        return *std::ranges::min_element(distances, [](auto &a, auto &b) { return fabs(std::get<float>(a)) < fabs(std::get<float>(b)); });
    }
    void Room::print()
    {
        if(is_initialized)
        {
            std::cout << "Room: " << std::endl;
            std::cout << "  center: " << rect.center << std::endl;
            std::cout << "  size: " << rect.size << std::endl;
            std::cout << "  rot: " << rect.angle << "º, " << qDegreesToRadians(rect.angle) << " rads" << std::endl;
            cv::Point2f vertices[4]; rect.points(vertices);
            for(const auto &[i, c]: vertices | iter::enumerate)
                std::cout << "  corner " << i << ": [" << c.x << ", " << c.y << "]" << std::endl;
            std::cout << std::endl;
        }
        else
            std::cout << "Room not initialized" << std::endl;
    }
    Eigen::Vector2f Room::to_local_coor(const Eigen::Vector2f &p)
    {
        //Eigen::Rotation2Df rt_from_robot(rot);
        Eigen::Matrix<float, 2, 2> rt_from_robot;
        rt_from_robot << cos(rot), -sin(rot), sin(rot), cos(rot);
        return rt_from_robot.transpose() * (p - center);
    }
    float Room::euc_distance_between_points(const QPointF &p1, const QPointF &p2) const
    {
        return sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y()));
    }
    float Room::euc_distance_between_points(const cv::Point2f &p1, const QPointF &p2) const
    {
        return sqrt((p1.x - p2.x()) * (p1.x - p2.x()) + (p1.y - p2.y()) * (p1.y - p2.y()));
    }
    QPointF Room::to_qpoint(const cv::Point2f &p) const
    {
        return QPointF{p.x, p.y};
    }
    void Room::rotate(float delta)
    {
        delta = std::clamp(delta, -5.f, 5.f);
        rot += delta;
        //qInfo() << __FUNCTION__ << delta << rot;
    }
    void Room::draw_2D(const Room &room, const QString &color, QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem *> items;
        for (auto i: items)
        {
            scene->removeItem(i);
            delete i;
        }
        items.clear();

        QColor col("yellow");
        col.setAlpha(30);
        //auto size = room.rsize;
        auto size = room.rect.size;
        auto item = scene->addRect(-size.height / 2, -size.width / 2, size.height, size.width, QPen(QColor(col), 60), QBrush(QColor(col)));
        item->setPos(room.rect.center.x, room.rect.center.y);
        item->setRotation(room.rect.angle + 90);
        items.push_back(item);

        //        QColor ccolor("red");
        //        cv::Point2f pts[4];
        //        rect.points(pts);
        //        for (auto &&i: iter::range(4))
        //        {
        //            auto it = scene->addEllipse(-100, -100, 200, 200, QPen(ccolor), QBrush(ccolor));
        //            it->setPos(pts[i].x, pts[i].y);
        //            items.push_back(it);
        //        }
    }
    std::vector<QLineF> Room::get_room_lines_qt() const
    {
        std::vector<QLineF> lines;
        cv::Point2f pts[4];
        rect.points(pts);
        lines.emplace_back(QLineF(pts[0].x, pts[0].y, pts[1].x, pts[1].y));
        lines.emplace_back(QLineF(pts[1].x, pts[1].y, pts[2].x, pts[2].y));
        lines.emplace_back(QLineF(pts[2].x, pts[2].y, pts[3].x, pts[3].y));
        lines.emplace_back(QLineF(pts[3].x, pts[3].y, pts[0].x, pts[0].y));
        return lines;
    }
    QPolygonF Room::get_qt_polygon() const
    {
        QPolygonF poly;
        cv::Point2f pts[4];
        rect.points(pts);
        for(const auto &p: pts)
            poly << QPointF(p.x, p.y);
        return poly;
    }
    float Room::get_width() const
    {
        return rect.size.width;
    }
    float Room::get_depth() const
    {
        return rect.size.height;
    }
    float Room::get_height() const
    {
        return 2000.f;  // mm TODO: to be estimated
    }
    Eigen::Vector2f Room::get_center() const
    {
        return Eigen::Vector2f(rect.center.x, rect.center.y);
    }
    float Room::get_center_x() const
    {
        return rect.center.x;
    }
    float Room::get_center_y() const
    {
        return rect.center.y;
    }
    float Room::get_rotation() const
    {
//        return qDegreesToRadians(rect.angle);
        return rect.angle;

    }
} //rc