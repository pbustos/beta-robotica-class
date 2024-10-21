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
    std::vector<QPolygonF> dbscan(RoboCompLidar3D::TPoints &points, float eps, int min_points)
    {
        arma::mat arma_data(2, points.size());
        for (const auto &[i, p]: points | iter::enumerate)
        {
            arma_data(0, i) = p.x;
            arma_data(1, i) = p.y;
        }
        arma::Row<size_t> assignments;
        mlpack::dbscan::DBSCAN<> dbscan(eps, min_points);
        dbscan.Cluster(arma_data, assignments);
        std::map<size_t, std::vector<cv::Point2f>> clustersMap;
        for (const auto i: iter::range(assignments.n_elem))
        {
            size_t label = assignments[i];
            if (label != (size_t) -1)
                clustersMap[label].emplace_back(points[i].x, points[i].y);  // -1 indicates noise
        }
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
            poly = enlarge_polygon(poly, 300);
            for (auto &p: poly)
                p *= 1.1;
            list_poly.emplace_back(poly);
        }
        return list_poly;
    };

}