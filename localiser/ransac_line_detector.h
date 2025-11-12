//
// Created by pbustos on 27/10/25.
//

#ifndef RANSACLINEDETECTOR_H
#define RANSACLINEDETECTOR_H

#include <vector>
#include <Eigen/Geometry>
#include "common_types.h"

namespace rc {

class RansacLineDetector
{
    public:
        struct Params
        {
            double distance_threshold = 50.0;  // mm - points within this are inliers
            int min_points_per_line = 30;      // minimum inliers to accept line
            int max_iterations = 100;          // RANSAC iterations per line
            int max_lines = 10;                // maximum lines to detect
            double min_line_length = 300.0;    // mm - minimum line length
            double max_line_separation = 200.0; // mm - maximum separation between lines for non-maximum suppression
            // Constructor to initialize default values
            Params()
                : distance_threshold(50.0),
                  min_points_per_line(30),
                  max_iterations(100),
                  max_lines(10),
                  min_line_length(300.0)
            {}
        };

        static std::vector<LineSegment> detect_lines(const std::vector<Eigen::Vector2d>& points, const Params& params = Params());
};

} // rc

#endif //RANSACLINEDETECTOR_H
