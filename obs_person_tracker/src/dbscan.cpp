//
// Created by pbustos on 21/10/24.
//

#include "dbscan.h"
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <mlpack/core.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <QVector2D>

namespace rc
{
    QPolygonF enlarge_polygon(const QPolygonF &polygon, qreal amount)
    {
        if (polygon.isEmpty())
            return QPolygonF();

        // Calculate the centroid of the polygon
        QPointF centroid(0, 0);
        for (const QPointF &point : polygon)
            centroid += point;
        centroid /= polygon.size();

        // Move each point away from the centroid
        QPolygonF enlargedPolygon;
        for (const QPointF &point : polygon)
        {
            QPointF direction = point - centroid; // Reverse the direction to move away from the centroid
            qreal length = std::sqrt(direction.x() * direction.x() + direction.y() * direction.y());
            if (length > 0)
            {
                QPointF offset = direction / length * amount;
                enlargedPolygon << (point + offset);
            }
            else
                enlargedPolygon << point; // If the point is at the centroid, leave it unchanged
        }
        return enlargedPolygon;
    }
    std::vector<QPolygonF> dbscan(const std::vector<Eigen::Vector2f> &points,
                                  float eps, int min_points,
                                  float robot_width)
    {
        if(points.empty()) {std::cout << __FUNCTION__ << " No points" << std::endl; return {};}
        arma::mat arma_data(2, points.size());
        for (const auto &[i, p]: points | iter::enumerate)
        {
            arma_data(0, i) = p.x();
            arma_data(1, i) = p.y();
        }
        arma::Row<size_t> assignments;
        mlpack::dbscan::DBSCAN<> dbscan(eps, min_points);
        dbscan.Cluster(arma_data, assignments);
        std::map<size_t, std::vector<cv::Point2f>> clustersMap;
        for (const auto i: iter::range(assignments.n_elem))
        {
            size_t label = assignments[i];
            if (label != (size_t) -1)
                clustersMap[label].emplace_back(points[i].x(), points[i].y());  // -1 indicates noise
        }

        // compute polygons
        std::vector<QPolygonF> list_poly;
        std::vector<cv::Point2f> hull;
        for (const auto &pair: clustersMap)
        {
            // Calculate the convex hull of the cluster
            std::vector<cv::Point2f> hull;
            cv::convexHull(pair.second, hull);

            // Convert the convex hull to a QPolygonF
            QPolygonF poly;
            for (const auto &p: hull)
                poly << QPointF(p.x, p.y);

            // enlarge the polygon to account for the robot size
            QPolygonF exp_poly;
            for(int i = 0; i < poly.size(); ++i)
            {
                const QPointF p1 = poly[i];
                const QPointF p2 = poly[(i + 1) % poly.size()]; // Handle the last point

                // Calculate the angle of the line segment
                const float angle = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());

                // Calculate the new points by translating and rotating the line segment
                QPointF newP1(p1.x() + robot_width * std::cos(angle),p1.y() + robot_width * std::sin(angle));
                QPointF newP2(p2.x() + robot_width * std::cos(angle),p2.y() + robot_width * std::sin(angle));
                exp_poly << newP1 << newP2;
            }
            list_poly.emplace_back(exp_poly);
        }
        return list_poly;
    };
}