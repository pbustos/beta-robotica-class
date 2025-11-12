//
// Created by pbustos on 11/11/25.
//

#ifndef DOORDETECTOR_H
#define DOORDETECTOR_H

#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>

class DoorDetector
{
    public:
        DoorDetector() = default;
        ~DoorDetector() = default;

        Doors detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene = nullptr);
        RoboCompLidar3D::TPoints filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
        [[nodiscard]] Doors doors() const { return doors_cache; };

    private:
        Doors doors_cache;
};

#endif //DOORDETECTOR_H
