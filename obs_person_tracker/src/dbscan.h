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
        std::vector<QPolygonF> dbscan(const std::vector<Eigen::Vector2f> &points,
                                      float eps, int min_points,
                                      float robot_width);
}
#endif //PERSON_TRACKER_DBSCAN_H
