//
// Created by robolab on 12/5/24.
//

#ifndef BETA_ROBOTICA_CLASS_PRIVATE_HUNGARIAN_H
#define BETA_ROBOTICA_CLASS_PRIVATE_HUNGARIAN_H

#include <Eigen/Geometry>
#include "munkres.hpp"
#include "common_types.h"

namespace rc
{
    class Hungarian
    {
        public:
            /**
          * @brief Hungarian function to work with corners.
          * @param measurement_corners
          * @param nominal_corners_in_robot
          * @param robot_current_pose
          * @param max_corner_diff
          * @return Match
          */
            Match match(const Corners &measurement_corners, const Corners &nominal_corners, double max_corner_diff = std::numeric_limits<double>::max());

            // aux methods
            double euclidean_distance(const QPointF &p1, const QPointF &p2);
            double euclidean_distance(const Corner &c1, const Corner &c2);
    };
} // rc

#endif //BETA_ROBOTICA_CLASS_PRIVATE_HUNGARIAN_H
