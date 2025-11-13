//
// Created by pbustos on 2/12/22.
//

#ifndef FORCEFIELD_ROOM_DETECTOR_H
#define FORCEFIELD_ROOM_DETECTOR_H

#include <iosfwd>
#include <vector>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <QtCore>
#include <ranges>
#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>
#include <tuple>
#include <vector>

#include "ransac_line_detector.h"

// Room_Detector class is a class that detects rooms in a 2D space. It uses a set of Hough lines to detect the rooms.
// The class has a method detect that receives a set of lines and returns a Room object.
// The class has a method compute_features that receives a set of lines and returns a tuple with the following features:
// 1. A set of lines
// 2. A set of parallel lines
// 3. A set of corners
// 4. A set of rooms

namespace rc   // aka RoboComp
{
    class Room_Detector
    {
        public:
             std::tuple<Corners, Lines> compute_corners(const std::vector<Eigen::Vector2d> &line, QGraphicsScene *scene= nullptr);
             std::tuple<Corners, Lines> compute_corners(const std::vector<Eigen::Vector3d> &line, QGraphicsScene *scene= nullptr);
             std::tuple<Corners, Lines> compute_corners(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene= nullptr);
             Eigen::Vector3d estimate_room_sizes(const Eigen::Vector2d &room_center, std::vector<Eigen::Vector2d> &floor_line_cart);
             Corners get_corners(Lines &elines);
             Lines filter_lines_by_length(const Lines &lines, float threshold );

            // aux
             double euc_distance_between_points(const QPointF &p1, const QPointF &p2);
             QPointF get_most_distant_point(const QPointF &p, const QPointF &p1, const QPointF &p2);
             [[nodiscard]]  std::vector<Center> reorder_points_CCW(const std::vector<Center> &points);

            // draw
             void draw_lines_on_2D_tab(const Lines &lines, QGraphicsScene *scene);
             void draw_corners_on_2D_tab(const Corners &corners, const std::vector<Eigen::Vector2d> &model_corners, QGraphicsScene *scene);

            // local data
             Eigen::Vector2d to_eigen(const QPointF &p);
             Eigen::Vector2d to_eigen(const cv::Point2d &p);
             QPointF to_qpointf(const cv::Point2d &p);

             /// Estimate the room center (robot frame) from a set of wall lines expressed as
             /// ax + by + c = 0 in Eigen::Vector3d (a,b,c). Lines can be unnormalized.
             /// Returns std::nullopt if not enough info (e.g., missing one direction).
             std::optional<Eigen::Vector2d>
             estimate_center_from_walls(const Lines &lines) const;

            [[nodiscard]] std::optional<Eigen::Vector2d> estimate_center_from_walls() const;
            [[nodiscard]] std::optional<Eigen::Vector2d> estimate_center_from_walls_simple(const Lines &lines) const;
            [[nodiscard]] std::optional<Eigen::Vector2d> estimate_center_from_walls_opencv(const Lines &lines) const;
             Corners select_minimal_rectangle(const Corners &corners);

             /// Normalize a line so that sqrt(a^2 + b^2) = 1 and choose a consistent sign.
             static Eigen::Vector3d normalizeLineABC_(const Eigen::Vector3d &abc);

             /// Pick two outermost parallel lines in a group by sorting their c (after normalization).
             static std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
             pickOuterParallelPairByC_(std::vector<Eigen::Vector3d> &grp);

             /// Midline between two parallel lines (same a,b).
             static Eigen::Vector3d midline_(const Eigen::Vector3d &L1, const Eigen::Vector3d &L2);

             /// Intersection of two lines ax + by + c = 0.
             static Eigen::Vector2d intersect_(const Eigen::Vector3d &L1, const Eigen::Vector3d &L2);

        private:
              Lines current_walls;
    };
} // rc

#endif //FORCEFIELD_ROOM_DETECTOR_H
