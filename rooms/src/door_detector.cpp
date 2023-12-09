//
// Created by pbustos on 1/12/22.
//

#include "door_detector.h"
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/slice.hpp>

DoorDetector::DoorDetector()
{
}
DoorDetector::Doors
DoorDetector::detect(const Lines &lines, QGraphicsScene *scene)
{
    auto peaks = extract_peaks(lines);
    auto doors = get_doors(peaks, lines);
    auto final_doors = filter_doors(doors);

    draw_doors(final_doors, Door(), scene);
    return final_doors;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////7
DoorDetector::Peaks_list DoorDetector::extract_peaks(const DoorDetector::Lines &lines)
{
    Peaks_list peaks_list(lines.size());
    const float THRES_PEAK = 1000;
    // for each line in lines (each level of lidar) and for each pair of points in the line check
    // if the distance between them is greater than THRES_PEAK
    for(const auto &[i, line] : lines | iter::enumerate)
        for (const auto &[j, both]: iter::sliding_window(line, 2) | iter::enumerate)
            if (fabs(both[1].norm() - both[0].norm()) > THRES_PEAK)
            {
                // select the point with the smallest distance to the robot
                if (both[0].norm() < both[1].norm()) peaks_list[i].push_back(j);
                else peaks_list[i].push_back(j+1);
            }
    return peaks_list;
}
DoorDetector::Doors_list DoorDetector::get_doors(const DoorDetector::Peaks_list &peaks_list, const DoorDetector::Lines &lines)
{
    std::vector<Doors> doors_list(peaks_list.size());
    // lambda to check if a door is near another door in the
    auto near_door = [thres = consts.SAME_DOOR](auto &doors_list, auto d)
    {
        return std::ranges::any_of(doors_list, [d, thres](auto &old)
            {return (old.p0-d.p0).norm() < thres or
                    (old.p1-d.p1).norm() < thres or
                    (old.p1-d.p0).norm() < thres or
                    (old.p0-d.p1).norm() < thres;});
    };
    // for each line in lines (each level of lidar) and for each pair of peaks in the line check
    // if the distance between them is greater than MIN_DOOR_WIDTH and less than MAX_DOOR_WIDTH
    for(const auto &[i, peaks] : peaks_list | iter::enumerate)
        for(auto &par : peaks | iter::combinations(2))
            if((lines[i][par[0]]-lines[i][par[1]]).norm() < consts.MAX_DOOR_WIDTH and (lines[i][par[0]]-lines[i][par[1]]).norm() > consts.MIN_DOOR_WIDTH)
            {
                auto door = Door{lines[i][par[0]], lines[i][par[1]], (int)par[0], (int)par[1]};
                if(not near_door(doors_list[i], door))
                    doors_list[i].emplace_back(std::move(door));
            }
    return doors_list;
}
DoorDetector::Doors DoorDetector::filter_doors(const DoorDetector::Doors_list &doors_list)
{
    Doors final_doors;
    auto lowest_doors = doors_list[0];
    //
    for(const auto &dl: lowest_doors)
    {
        bool match = true;
        for(const auto &doors: iter::slice(doors_list, 1, (int)doors_list.size(), 1))  // start from second element
            match = match and std::ranges::find(doors, dl) != doors.end();

        if (match)
            final_doors.push_back(dl);
    }
    return final_doors;
}

void DoorDetector::draw_doors(const Doors &doors, const Door &door_target, QGraphicsScene *scene, QColor color)
{
    static std::vector<QGraphicsItem *> borrar;
    for (auto &b: borrar)
    {
        scene->removeItem(b);
        delete b;
    }
    borrar.clear();

    QColor target_color;
    for (const auto &d: doors)
    {
        if(d == door_target)
        {
            target_color = QColor("magenta");
            auto middle = scene->addRect(-100, -100, 200, 200, QColor("blue"), QBrush(QColor("blue")));
            auto perp = door_target.point_perpendicular_to_door_at();
            middle->setPos(perp.first.x(), perp.first.y());
            borrar.push_back(middle);
            auto middle_line = scene->addLine(perp.first.x(), perp.first.y(), d.middle.x(), d.middle.y(), QPen(QColor("blue"), 20));
            borrar.push_back(middle_line);
        }
        else
            target_color = color;
        auto point = scene->addRect(-50, -50, 100, 100, QPen(target_color), QBrush(target_color));
        point->setPos(d.p0.x(), d.p0.y());
        borrar.push_back(point);
        point = scene->addRect(-50, -50, 100, 100, QPen(target_color), QBrush(target_color));
        point->setPos(d.p1.x(), d.p1.y());
        borrar.push_back(point);
        auto line = scene->addLine(d.p0.x(), d.p0.y(), d.p1.x(), d.p1.y(), QPen(target_color, 50));
        borrar.push_back(line);
    }
}
DoorDetector::Line DoorDetector::filter_out_points_beyond_doors(const Line &floor_line, const Doors &doors)
{
    std::vector<Eigen::Vector2f> inside_points(floor_line);
    std::vector<Eigen::Vector2f> remove_points;
    // for each door in doors check if the line going from the robot to each point in the segment of the floor_line
    // that corresponds to the door, intersects with the door line
    for (const auto &door: doors)       // all in robot's reference system
    {
        QLineF door_line(door.p0.x(), door.p0.y(), door.p1.x(), door.p1.y());
        //
        for (auto &&i: iter::range(std::min(door.idx_in_peaks_0, door.idx_in_peaks_1), std::max(door.idx_in_peaks_0, door.idx_in_peaks_1)))
        {
            // line from robot to point in floor_line
            QLineF robot_to_point(0.f, 0.f, floor_line[i].x(), floor_line[i].y());
            QPointF point;
            if (auto res = robot_to_point.intersects(door_line, &point); res == QLineF::BoundedIntersection)
                remove_points.emplace_back(point.x(), point.y());
        }
    }
    qInfo() << __FUNCTION__ << "Before" << floor_line.size() << "After" << inside_points.size() << "Removed" << remove_points.size();
    // remove from inside_points all points in remove_points
    for (const auto &p: remove_points)
        inside_points.erase(std::remove_if(inside_points.begin(), inside_points.end(), [&p](auto &i){return (i-p).norm() < 100;}), inside_points.end());
    return inside_points;
}