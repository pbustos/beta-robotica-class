//
// Created by pbustos on 1/12/22.
//

#ifndef FORCEFIELD_DOOR_DETECTOR_H
#define FORCEFIELD_DOOR_DETECTOR_H

#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include <QGraphicsItem>
#include <QColor>
#include <QtCore>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <opencv2/core.hpp>
#include <Lidar3D.h>

class DoorDetector
{
    public:
        DoorDetector();
        struct Door
        {
            Eigen::Vector2f p0, p1, middle;
            int idx_in_peaks_0, idx_in_peaks_1;
            const float THRESHOLD = 500; //door equality

            Door(){ p0 = p1 = middle = Eigen::Vector2f::Zero();};
            Door(const Eigen::Vector2f &p0_,
                 const Eigen::Vector2f &p1_,
                 int idx_0, int idx_1) : p0(p0_), p1(p1_), idx_in_peaks_0(idx_0), idx_in_peaks_1(idx_1)
            {
                middle = (p0 + p1)/2.f;
            };
            bool operator==(const Door &d) const
            {
                return (d.middle - middle).norm() < THRESHOLD;
            };
            Door& operator=(const Door &d)
            {
                p0 = d.p0;
                p1 = d.p1;
                middle = d.middle;
                return *this;
            };
            void print()
            {
                qInfo() << "Door:";
                qInfo() << "    left:" << p0.x() << p0.y();
                qInfo() << "    right:" << p1.x() << p1.y();
            };
            float dist_to_robot() const
            { return middle.norm();}
            float angle_to_robot() const
            { return (float)atan2(middle.x(), middle.y());}
            std::pair<Eigen::Vector2f, Eigen::Vector2f> point_perpendicular_to_door_at(float dist=1000.f) const // mm
            {
                // Calculate the direction vector from p1 to p2 and rotate it by 90 degrees
                Eigen::Vector2f d_perp{-(p0.y() - p1.y()), p0.x() - p1.x()};
                // Normalize the perpendicular vector to get the unit vector
                Eigen::Vector2f u_perp = d_perp.normalized();
                // Calculate the points P1 and P2 at a distance of 1 meter from M along the perpendicular
                Eigen::Vector2f a, b;
                a = middle + (u_perp * dist); // 1 meter in the direction of u_perp
                b = middle - (u_perp * dist); // 1 meter in the opposite direction of u_perp
                return a.norm() < b.norm() ? std::make_pair(a,b) : std::make_pair(b,a);
            }
            float perp_dist_to_robot() const
            {
                auto p = point_perpendicular_to_door_at();
                return p.first.norm();
            }
            float perp_angle_to_robot() const
            {
                auto p = point_perpendicular_to_door_at();
                return atan2(p.first.x(), p.first.y());
            }
         };

        using Doors = std::vector<Door>;
        using Doors_list = std::vector<Doors>;
        using Line = std::vector<Eigen::Vector2f>;
        using Lines = std::vector<Line>;
        using Peaks_list = std::vector<std::vector<int>>;
        struct Constants
        {
            const float SAME_DOOR = 400;   // same door threshold (mm)
            const float MAX_DOOR_WIDTH = 1400;  // mm
            const float MIN_DOOR_WIDTH = 700;   // mm
        };
        Constants consts;
        Doors detect(const Lines &lines, QGraphicsScene *scene);
        Line current_line;

private:
        std::vector<std::vector<Eigen::Vector2f>> extract_lines(const RoboCompLidar3D::TPoints &points, const std::vector<std::pair<float, float>> &ranges);
        Peaks_list extract_peaks(const Lines &lines);
        Doors_list get_doors(const Peaks_list &peaks, const Lines &lines);
        Doors filter_doors(const Doors_list &doors);
        void draw_doors(const Doors &doors, const Door &current_door, QGraphicsScene *scene, QColor=QColor("green"));
        Line filter_out_points_beyond_doors(const Line&floor_line_cart, const Doors &doors);

        std::vector<std::pair<float, float>> height_ranges;
};


#endif //FORCEFIELD_DOOR_DETECTOR_H
