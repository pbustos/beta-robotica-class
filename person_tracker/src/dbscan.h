//
// Created by pbustos on 21/10/24.
//

#ifndef PERSON_TRACKER_DBSCAN_H
#define PERSON_TRACKER_DBSCAN_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <Lidar3D.h>
#include <map>
#include <QPolygonF>

namespace rc
{
        std::vector<QPolygonF> dbscan(RoboCompLidar3D::TPoints &points, float eps, int min_points);
}
#endif //PERSON_TRACKER_DBSCAN_H
