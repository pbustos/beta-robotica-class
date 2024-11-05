//
// Created by pbustos on 2/12/22.
//

#include "room_detector.h"
#include <cppitertools/enumerate.hpp>
#include <cppitertools/combinations.hpp>
#include <cppitertools/combinations_with_replacement.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>

namespace rc
{
    Room Room_Detector::detect(const std::vector<Eigen::Vector2f> &line, QGraphicsScene *scene, bool draw_lines)
    {
//        // compute features
//        const auto &[_, par_lines, corners, all_corners] =  compute_features(line, scene);
//
//        // Start with more complex features and go downwards
//        if (not all_corners.empty())    // if there are room candidates, take the first room // TODO:: order by votes
//        {
//            const auto &[c1, c2, c3, c4] = all_corners.front();
//            std::vector<cv::Point2f> poly{cv::Point2f(c1.x(), c1.y()), cv::Point2f(c2.x(), c2.y()),
//                                          cv::Point2f(c3.x(), c3.y()), cv::Point2f(c4.x(), c4.y())};
//            current_room.rect = cv::minAreaRect(poly);
//            current_room.is_initialized = true;
//            //triple_val = 300; corners_val = 0; par_lines_val = 0;
//        }
//        // if there is a room and detected corners that match some of the model, compute a resulting force and torque to rotate the model
//        else if (not corners.empty() and corners.size() > 1 and current_room.is_initialized)
//        {
//            double torque = 0.0;
//            std::vector<Eigen::Vector2f> model_corners;
//            for(const auto &[i, c]: corners | iter::enumerate)
//            {
//                auto &[votes, p] = c;
//                Eigen::Vector2f corner = to_eigen(p);
//                Eigen::Vector2f model = current_room.get_closest_corner_in_robot_coor2(corner);
//                // compute torque as the perp signed distance of corner on Line(robot_center, model);
//                auto line = Eigen::ParametrizedLine<float, 2>::Through(to_eigen(current_room.rect.center), model);
//                Eigen::Hyperplane<float, 2> hline(line);
//                float sdist = hline.signedDistance(corner);
//                model_corners.push_back(model);
//                torque += sdist;
//            }
//            // rotate de rectangle proportional to resulting torque
//            double k = 0.01;
//            double ang = std::clamp(k*torque, -10.0, 10.0);
//            current_room.rect.angle += (float)ang;
//            if(fabs(ang) < 2)   // translate
//            {
//                Eigen::Vector2f tr = to_eigen(std::get<1>(corners[0]))-model_corners[0];
//                current_room.rect.center += cv::Point2f(cv::Point2f(tr.x(), tr.y()));
//            }
//            draw_corners_on_2D_tab(corners, model_corners, scene);
//        }
//
//        current_room.draw_on_2D_tab(current_room, "yellow", scene);
        return  current_room;
    }
    Room_Detector::Features Room_Detector::compute_features(const std::vector<Eigen::Vector2f> &line, QGraphicsScene *scene)
    {
        std::vector<Eigen::Vector2f> floor_line_cart = line;

        // compute mean point
        Eigen::Vector2f room_center = Eigen::Vector2f::Zero();
        room_center = accumulate(floor_line_cart.begin(), floor_line_cart.end(), room_center) / (float)floor_line_cart.size();
        // std::cout << "Center " << room_center << std::endl;

        // estimate size
        Eigen::Vector3f estimated_size = estimate_room_sizes(room_center, floor_line_cart);
        // std::cout << "Size " << estimated_size.x() << " " << estimated_size.y() << std::endl;

        // compute lines
        //std::vector<std::pair<int, QLineF>> elines = hough_transform(floor_line_cart);
        Lines elines = get_hough_lines(floor_line_cart);
        draw_lines_on_2D_tab(elines, scene);

        // compute parallel lines of minimum length and separation
        Par_lines par_lines = get_parallel_lines(elines, estimated_size.head(2));

        // compute corners
        Corners corners = get_corners(elines);
        draw_corners_on_2D_tab(corners, {Eigen::Vector2f{0,0}}, scene);

        // compute room candidates by finding triplets of corners in opposite directions and separation within room_size estimations
        All_Corners all_corners = get_rooms(estimated_size.head(2), corners);

        // draw
        draw_lines_on_2D_tab(elines, scene);
        //draw_triple_corners_on_2D_tab(all_corners, "green", scene);
        //        qInfo() << "Room detector: ";
        //        qInfo() << "    num. points:" << floor_line_cart.size();
        //        qInfo() << "    center: [" << room_center.x() << "," << room_center.y() << "]";
        //        qInfo() << "    size: [" << estimated_size.x() << "," << estimated_size.y() << "]";
        //        qInfo() << "    num. lines:" << elines.size();
        //        qInfo() << "    num. parallel lines:" << par_lines.size();
        //        qInfo() << "    num. corners:" << corners.size();
        //        qInfo() << "    num. triple corners:" << all_corners.size();

        return std::make_tuple(elines, par_lines, corners, all_corners);
    }

     ////////////////////////////////////////////////
    Eigen::Vector3f Room_Detector::estimate_room_sizes(const Eigen::Vector2f &room_center, std::vector<Eigen::Vector2f> &floor_line_cart) const
    {
        Eigen::MatrixX2f zero_mean_points(floor_line_cart.size(), 2);
        for(const auto &[i, p] : iter::enumerate(floor_line_cart))
            zero_mean_points.row(i) = p - room_center;

        Eigen::Matrix2f cov = (zero_mean_points.adjoint() * zero_mean_points) / float(zero_mean_points.rows() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver;
        eigensolver.compute(cov);
        Eigen::Vector2f values = eigensolver.eigenvalues().real().cwiseSqrt()*2.f;
        Eigen::Index i;
        values.maxCoeff(&i);
        Eigen::Vector2f max_vector = eigensolver.eigenvectors().real().col(i);
        return Eigen::Vector3f(values.x(), values.y(), atan2(max_vector.x(), max_vector.y()));
    }
    std::vector<std::pair<int, QLineF>> Room_Detector::get_hough_lines(std::vector<Eigen::Vector2f> &floor_line_cart) const
    {
        const double rhoMin = -7000.0, rhoMax = 7000.0, rhoStep = 20;
        const double thetaMin = 0, thetaMax = CV_PI, thetaStep = CV_PI / 180.0f;
        const double min_votes = 10;  // min number of votes to consider a line
        const float max_norm = 8000.f;  // max norm of a point to be considered
        const int max_lines = 10;   // max number of lines to detect
        const int threshold = 25;
        const double delta = 0.3;  // min value for non-maximum supression degrees
        const double min_dist = 400;  // min value for non_maximun supression mm

        std::vector<cv::Vec2f> floor_line_cv;
        for(const auto &p : floor_line_cart)
            if(not p.isZero() and p.norm() < max_norm)
                floor_line_cv.emplace_back(p.x(), p.y());

        cv::Mat lines;
        HoughLinesPointSet(floor_line_cv, lines, max_lines, threshold, rhoMin,
                           rhoMax, rhoStep, thetaMin, thetaMax, thetaStep);
        if(lines.empty())
            return  {};

        std::vector<cv::Vec3d> lines3d;
        lines.copyTo(lines3d);  // copy-convert to cv::Vec3d  (votes, rho, theta)


            // compute lines from Hough params
        std::vector<std::pair<int, QLineF>> elines;
        // filter lines with more than 10 votes
        //for(auto &l: std::ranges::filter_view(lines3d, [min = min_votes](auto &l){return l[0]>min;}))
        for(const auto &l: lines3d)
        {
            // compute QlineF from rho, theta
            double rho = l[1], theta = l[2];
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            QPointF p1(x0 + max_norm * (-b), y0 + max_norm * (a));
            QPointF p2(x0 - max_norm * (-b), y0 - max_norm* (a));
            elines.emplace_back(l[0], QLineF(p1, p2));    // votes, line
        }

        // Non-Maximum Suppression of close parallel lines
        std::vector<QLineF> to_delete;
        for(auto &&comb: iter::combinations(elines, 2))
        {
            auto &[votes_1, line1] = comb[0];
            auto &[votes_2, line2] = comb[1];
            double angle = qDegreesToRadians(qMin(line1.angleTo(line2), line2.angleTo(line1)));
            QPointF diff = line1.center() - line2.center();
            double dist = std::hypot(diff.x(), diff.y());
            if((fabs(angle)<delta or (fabs(angle)-M_PI)< delta) and dist < min_dist)
            {
                if (votes_1 >= votes_2) to_delete.push_back(line2);
                else to_delete.push_back(line1);
            }
        }
        // erase lines that are parallel and too close
        elines.erase(remove_if(elines.begin(), elines.end(), [to_delete](auto l)
            { return std::ranges::find(to_delete, std::get<1>(l)) != to_delete.end();}), elines.end());
        return elines;
    }
    void Room_Detector::filter_lines_by_length(const Lines &lines, std::vector<Eigen::Vector2f> &floor_line_cart )
    {
        Eigen::ParametrizedLine<float, 2> eline;
        const float max_distance_to_line = 100;
        std::vector<std::vector<std::pair<float, Eigen::Vector2f>>> distances(lines.size());
        for(const auto &[i, line]: lines | iter::enumerate)
        {
            const auto &[vote, l] = line;
            eline = Eigen::ParametrizedLine<float, 2>::Through(to_eigen(l.p1()), to_eigen(l.p2()));
            for (const auto &p: floor_line_cart)
            {
                float d = eline.distance(p);
                if(d < max_distance_to_line)
                    distances[i].emplace_back(std::make_pair((p- to_eigen(l.p1())).norm(), p));
            }
        }
        for(const auto &[i, line]: distances | iter::enumerate)
        {
            auto [min, max] = std::ranges::minmax_element(line, [](auto &a, auto &b){ return a.first < b.first;});
            qInfo() << __FUNCTION__ << "Line" << lines[i].second << "min" << min->second.x() << min->second.x()
                                    << "max" << max->second.x() << max->second.y() << "Dist" << (max->second - min->second).norm();
        }
        qInfo() << __FUNCTION__ << "--------------------";
    }
    std::vector<std::pair<QLineF, QLineF>> Room_Detector::get_parallel_lines(const  std::vector<std::pair<int, QLineF>> &lines,
                                                                             const Eigen::Vector2f &estimated_size)
    {
        std::vector<std::pair<QLineF, QLineF>> par_lines;
        for(auto &&line_pairs : iter::combinations(lines, 2))
        {
            const auto &[v1, line1] = line_pairs[0];
            const auto &[v2, line2] = line_pairs[1];
            double ang = fabs(line1.angleTo(line2));
            const float delta = 10;  //degrees
            if((ang > -delta and ang < delta) or ( ang > 180-delta and ang < 180+delta))
            if( euc_distance_between_points(line1.center(), line2.center()) > estimated_size.minCoeff()/2.0)
            par_lines.emplace_back(line1,  line2);
        }
    return par_lines;
    }
    Room_Detector::Corners Room_Detector::get_corners(Lines &elines)
    {
        Corners corners;
        for(auto &&comb: iter::combinations(elines, 2))
        {
            auto &[votes_1, line1] = comb[0];
            auto &[votes_2, line2] = comb[1];
            double angle = fabs(qDegreesToRadians(line1.angleTo(line2)));
            if(angle> M_PI) angle -= M_PI;
            if(angle< -M_PI) angle += M_PI;
            float delta = 0.2;
            QPointF intersection;
            if(angle < M_PI/2+delta and angle > M_PI/2-delta  and line1.intersects(line2, &intersection) == QLineF::BoundedIntersection)
                corners.emplace_back(std::make_tuple(std::min(votes_1, votes_2), intersection));
        }
        // sort by votes
        std::sort(corners.begin(), corners.end(), [](auto &a, auto &b){ return std::get<0>(a) > std::get<0>(b);});
        // NM suppression
        const float min_distance_among_corners = 200;
        Corners filtered_corners;
        for(auto &&c: corners | iter::combinations(2))
            if(euc_distance_between_points(std::get<1>(c[0]), std::get<1>(c[1])) < min_distance_among_corners and
               std::ranges::find_if_not(filtered_corners, [p=std::get<1>(c[0])](auto &a){ return std::get<1>(a) == p;}) == filtered_corners.end())
                filtered_corners.push_back(c[0]);
        return corners;
    }
    Room_Detector::All_Corners Room_Detector::get_rooms(const Eigen::Vector2f &estimated_size, const Corners &corners)
    {
        // This method has to be improved with:
        // 1, Two corners with opposite angles suffice to define a rectangle and compute the other two corners
        // 2. The distance between corners should be proportional to the room size

        All_Corners all_corners;
        // find pairs of corners in opposite directions and separation within room_size estimations
        float min_dist = estimated_size.minCoeff() / 2; // threshold for distance between corners based on room half size
        for(auto &&comb: iter::combinations(corners, 3))
        {
            const auto &[votes1, p1] = comb[0];
            const auto &[votes2, p2] = comb[1];
            const auto &[votes3, p3] = comb[2];
            std::vector<float> ds{euc_distance_between_points(p1, p2), euc_distance_between_points(p1, p3), euc_distance_between_points(p2, p3)};
            // if all pairs meet the distance criteria of being more than min_dist apart
            if (std::ranges::all_of(ds, [min_dist](auto &a) { return a > min_dist; }))
            {
                auto res = std::ranges::max_element(ds); // find the longest distance
                long pos = std::distance(ds.begin(), res);  //
                QPointF p4;
                // compute the fourth point
                if (pos == 0){ auto l = QLineF(p3, (p1 + p2) / 2).unitVector(); l.setLength(euc_distance_between_points(p1, p2)); p4 = l.pointAt(1);};
                if (pos == 1){ auto l = QLineF(p2, (p1 + p3) / 2).unitVector(); l.setLength(euc_distance_between_points(p1, p3)); p4 = l.pointAt(1);};
                if (pos == 2){ auto l = QLineF(p1, (p2 + p3) / 2).unitVector(); l.setLength(euc_distance_between_points(p2, p3)); p4 = l.pointAt(1);};
                all_corners.emplace_back(p1, p2, p3, p4);
            }
        }
        return all_corners;
    }

     ////////// DRAW
    void Room_Detector::draw_par_lines_on_2D_tab(const Par_lines &par_lines, QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem*> items;
        for(const auto &item: items)
        {
            scene->removeItem(item);
            delete item;
        }
        items.clear();

        for(const auto &[l1, l2]: par_lines)
        {
            auto i1 = scene->addLine(l1, QPen(QColor("brown"), 30));
            auto i2 = scene->addLine(l2, QPen(QColor("brown"), 30));
            items.push_back(i1); items.push_back(i2);
        }
    }
    void Room_Detector::draw_lines_on_2D_tab(const std::vector<std::pair<int, QLineF>> &lines, QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem*> lines_vec;
        for (auto l: lines_vec)
        {
            scene->removeItem(l);
            delete l;
        }
        lines_vec.clear();

        for(const auto &l : lines)
        {
            auto p = scene->addLine(l.second, QPen(QColor("orange"), 20));
            lines_vec.push_back(p);
        }
    }
    void Room_Detector::draw_corners_on_2D_tab(const Corners &corners, const std::vector<Eigen::Vector2f> &model_corners,
                                               QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem*> items;
        for (const auto &i: items)
        {
            scene->removeItem(i);
            delete i;
        }
        items.clear();

        QColor color("lightgreen");
        for(const auto &[votes, p] : corners)
        {
            auto i = scene->addEllipse(-100, -100, 200, 200, QPen(color), QBrush(color));
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
    void Room_Detector::draw_triple_corners_on_2D_tab(const All_Corners &all_corners, QString color,
                                                      QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem*> lines_vec;
        for (auto l: lines_vec)
        {
            scene->removeItem(l);
            delete l;
        }
        lines_vec.clear();

        for(const auto &[first, second, third, forth] : all_corners)
        {
            QPolygonF poly;
            poly << first << second << third;
            QColor col(color); col.setAlpha(30);
            auto p = scene->addPolygon(poly, QPen(col), QBrush(col));
            lines_vec.push_back(p);
            break;  // loops ONLY ONCE
        }
    }

    ////////// AUX
    QPointF Room_Detector::get_most_distant_point(const QPointF &p, const QPointF &p1, const QPointF &p2) const
    {
        if( (p-p1).manhattanLength() < (p-p2).manhattanLength()) return p2; else return p1;
    }
    Eigen::Vector2f Room_Detector::to_eigen(const QPointF &p) const
    {
        return Eigen::Vector2f{p.x(), p.y()};
    }
    Eigen::Vector2f Room_Detector::to_eigen(const cv::Point2f  &p) const
    {
        return Eigen::Vector2f{p.x, p.y};
    }
    QPointF Room_Detector::to_qpointf(const cv::Point2f  &p) const
    {
        return QPointF{p.x, p.y};
    }
    float Room_Detector::euc_distance_between_points(const QPointF &p1, const QPointF &p2) const
    {
        return sqrt((p1.x()-p2.x())*(p1.x()-p2.x())+(p1.y()-p2.y())*(p1.y()-p2.y()));
    }

} // rc
