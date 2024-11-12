//
// Created by pbustos on 21/10/24.
//

#ifndef PERSON_TRACKER_DBSCAN_H
#define PERSON_TRACKER_DBSCAN_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <QPolygonF>
#include <Eigen/Dense>

namespace rc
{
        /**
       * @brief Performs DBSCAN clustering on a set of 2D points.
       *
       * @param points A vector of 2D points to be clustered.
       * @param eps The maximum distance between two points to be considered as neighbors.
       * @param min_points The minimum number of points required to form a dense region.
       * @param robot_width The width of the robot, used to adjust the clustering.
       * @return A vector of polygons representing the clusters.
       */
        std::vector<QPolygonF> dbscan(const std::vector<Eigen::Vector2f> &points,
                                      float eps, int min_points);
}
#endif //PERSON_TRACKER_DBSCAN_H
