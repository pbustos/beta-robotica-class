//
// Created by pbustos on 11/11/25.
//

#ifndef DOORDETECTOR_H
#define DOORDETECTOR_H

#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>
#include <expected>

class DoorDetector
{
    public:
        DoorDetector() = default;
        ~DoorDetector() = default;

    Doors detect(const RoboCompLidar3D::TPoints &points,
                       const Eigen::Affine2d &robot_pose = Eigen::Affine2d::Identity(),
                       bool localised = false,
                       QGraphicsScene *robot_scene = nullptr,
                       QGraphicsScene *scene_room = nullptr);
        RoboCompLidar3D::TPoints filter_points(const RoboCompLidar3D::TPoints &points);
        [[nodiscard]] Doors doors() const { return doors_cache; };
        [[nodiscard]] std::expected<Door, std::string> get_current_door() const;

    private:
        Doors doors_cache;
        float min_door_width = 600.f;  // mm
        float max_door_width = 1200.f;
        float min_peak_distance = 500.f;
};

#endif //DOORDETECTOR_H
