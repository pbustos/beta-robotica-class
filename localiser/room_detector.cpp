//
// Created by pbustos on 2/12/22.
//

#include "room_detector.h"
#include <cppitertools/enumerate.hpp>
#include <cppitertools/combinations.hpp>
#include <opencv2/imgproc.hpp>
#include <cppitertools/zip.hpp>
#include <chrono>
#include "specificworker.h"

namespace rc
{
    // par lines and rooms are commented. Lines have to be at least of half-room size
    Corners Room_Detector::compute_corners(const std::vector<Eigen::Vector2d> &line, QGraphicsScene *scene)
    {
        const std::vector<Eigen::Vector2d> floor_line_cart = line;

        // compute mean point
        //Eigen::Vector2d room_center = Eigen::Vector2d::Zero();
        //room_center = accumulate(floor_line_cart.begin(), floor_line_cart.end(), room_center) / (double)floor_line_cart.size();
        // std::cout << "Center " << room_center << std::endl;

        // estimate size
        //Eigen::Vector3d estimated_size = estimate_room_sizes(room_center, floor_line_cart);
        // std::cout << "Size " << estimated_size.x() << " " << estimated_size.y() << std::endl;

        // compute lines
        auto  lines = RansacLineDetector::detect_lines(floor_line_cart);
        if (scene != nullptr) draw_lines_on_2D_tab(lines, scene);

        //lines = filter_lines_by_length(lines, static_cast<float>(estimated_size.head(2).minCoeff()/2.f));

        // compute corners
        const Corners corners = get_corners(lines);
        if (scene != nullptr) draw_corners_on_2D_tab(corners, {Eigen::Vector2d{0,0}}, scene);
        return corners;
    }
    Corners Room_Detector::compute_corners(const std::vector<Eigen::Vector3d> &line, QGraphicsScene *scene)
    {
        std::vector<Eigen::Vector2d> line2d;
        std::ranges::transform(line, std::back_inserter(line2d), [](const auto &p){return p.head(2);});
        return compute_corners(line2d, scene);
    }
    Corners Room_Detector::compute_corners(const RoboCompLidar3D::TPoints &points,  QGraphicsScene *scene)
    {
        std::vector<Eigen::Vector2d> line2d;
        std::ranges::transform(points, std::back_inserter(line2d), [](const auto &p){return Eigen::Vector2d{p.x, p.y};});
        return compute_corners(line2d,  scene);
    }

     ////////////////////////////////////////////////
    Eigen::Vector3d Room_Detector::estimate_room_sizes(const Eigen::Vector2d &room_center, std::vector<Eigen::Vector2d> &floor_line_cart)
    {
        Eigen::MatrixX2d zero_mean_points(floor_line_cart.size(), 2);
        for(const auto &[i, p] : iter::enumerate(floor_line_cart))
            zero_mean_points.row(i) = p - room_center;

        Eigen::Matrix2d cov = (zero_mean_points.adjoint() * zero_mean_points) / double(zero_mean_points.rows() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver;
        eigensolver.compute(cov);
        Eigen::Vector2d values = eigensolver.eigenvalues().real().cwiseSqrt()*2.f;
        Eigen::Index i;
        values.maxCoeff(&i);
        Eigen::Vector2d max_vector = eigensolver.eigenvectors().real().col(i);
        return Eigen::Vector3d(values.x(), values.y(), atan2(max_vector.x(), max_vector.y()));
    }

    Corners Room_Detector::get_corners(Lines &lines)
    {
        Corners corners;
        for(auto &&comb: iter::combinations(lines, 2))
        {
            const auto& line1 = comb[0];
            const auto& line2 = comb[1];
            double angle = fabs(qDegreesToRadians(line1.toQLineF().angleTo(line2.toQLineF())));
            angle = fmod(angle + M_PI, 2 * M_PI);
            if (angle < 0) angle += 2 * M_PI;
            angle -= M_PI;
            //if (angle > M_PI / 2) angle = M_PI - angle;
            //if (angle < -M_PI / 2) angle = -M_PI - angle;
            constexpr double delta = 0.2;
            QPointF intersection;
            const bool angle_condition = (angle < M_PI/2+delta and angle > M_PI/2-delta) or (angle < -M_PI/2+delta and angle > -M_PI/2-delta );
            long now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if(angle_condition and line1.toQLineF().intersects(line2.toQLineF(), &intersection) == QLineF::UnboundedIntersection)
                corners.emplace_back(intersection, 0.0, now );
        }

        // NM suppression
        constexpr double min_distance_among_corners = 200;
        Corners filtered_corners;
        std::ranges::copy_if(corners, std::back_inserter(filtered_corners), [&corners, min_distance_among_corners, this](const Corner &corner1)
                {
                   const auto &p1 = std::get<0>(corner1); // Extract QPointF
                   return std::ranges::none_of(corners, [&corner1, &p1, min_distance_among_corners, this](const Corner &corner2) {
                        const auto &p2 = std::get<0>(corner2); // Extract QPointF
                        return &corner1 != &corner2 and euc_distance_between_points(p1, p2) < min_distance_among_corners;});
                });
        return corners;
    }

     ////////// DRAW  /////////////////////////////////////////////////////////////
    Lines Room_Detector::filter_lines_by_length(const Lines &lines, float threshold )
    {
        Lines filtered_lines;
        std::ranges::copy_if(lines, std::back_inserter(filtered_lines),
                     [threshold](const LineSegment &line) {
                         const double length = (line.end - line.start).norm();
                         return length >= threshold;});
        return filtered_lines;
    }
    void Room_Detector::draw_lines_on_2D_tab(const Lines &lines, QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem*> lines_vec;
        for (const auto l: lines_vec)
        {
            scene->removeItem(l);
            delete l;
        }
        lines_vec.clear();

        const QPen pen(QColor("orange"), 20);
        for(const auto &l : lines)
        {
            auto ql = l.toQLineF();
            const auto p = scene->addLine(ql, pen);
            lines_vec.push_back(p);
        }
    }
    void Room_Detector::draw_corners_on_2D_tab(const Corners &corners, const std::vector<Eigen::Vector2d> &model_corners,
                                               QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem*> items;
        for (const auto &i: items)
        {
            scene->removeItem(i);
            delete i;
        }
        items.clear();

        const QColor color("darkMagenta");
        for(const auto &[p, _, __] : corners)
        {
            const auto i = scene->addEllipse(-100, -100, 200, 200, QPen(color), QBrush(color));
            i->setPos(p.x(), p.y());
            items.push_back(i);
        }
        //model corners
        QColor ccolor("cyan");
        for(const auto &[m, c] : iter::zip(model_corners, corners))
        {
            auto p = scene->addEllipse(-100, -100, 200, 200, QPen(ccolor), QBrush(ccolor));
            p->setPos(m.x(), m.y());
            items.push_back(p);
        }
    }

    ////////// AUX ///////////////////////////////////////////////////////////////////////////////////////
    QPointF Room_Detector::get_most_distant_point(const QPointF &p, const QPointF &p1, const QPointF &p2)
    {
        if( (p-p1).manhattanLength() < (p-p2).manhattanLength()) return p2; else return p1;
    }
    Eigen::Vector2d Room_Detector::to_eigen(const QPointF &p)
    {
        return Eigen::Vector2d{p.x(), p.y()};
    }
    Eigen::Vector2d Room_Detector::to_eigen(const cv::Point2d  &p)
    {
        return Eigen::Vector2d{p.x, p.y};
    }
    QPointF Room_Detector::to_qpointf(const cv::Point2d  &p)
    {
        return QPointF{p.x, p.y};
    }
    double Room_Detector::euc_distance_between_points(const QPointF &p1, const QPointF &p2)
    {
        return sqrt((p1.x()-p2.x())*(p1.x()-p2.x())+(p1.y()-p2.y())*(p1.y()-p2.y()));
    }

} // rc
