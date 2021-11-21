/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include "/home/robocomp/robocomp/classes/grid2d/grid.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include "IoU/src/iou.h"
#include <cppitertools/zip_longest.hpp>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
    void new_target_slot(QPointF);

private:
	bool startup_check_flag;
    AbstractGraphicViewer *viewer;

    struct Constants
    {
        const float max_advance_speed = 800;
        const float tile_size = 100;
        const float max_laser_range = 4000;
        float current_rot_speed = 0;
        float current_adv_speed = 0;
        float robot_length = 450;
        const float robot_semi_length = robot_length/2.0;
    };
    Constants constants;

    //robot
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    float gaussian(float x);

    // state machine
    enum class State {IDLE, INIT_TURN, TURN, ESTIMATE, GOTO_DOOR, GOTO_ROOM_CENTER};
    State state = State::IDLE;

    // laser
    const int MAX_LASER_RANGE = 4000;

    // grid
    int TILE_SIZE = 50;
    QRectF dimensions;
    Grid grid;

    void update_map(const RoboCompLaser::TLaserData &ldata);
    Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p);
    void fit_rectangle();

    // target
    struct Target
    {
        bool active = false;
        QPointF pos;
        Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(pos.x(), pos.y());}
        QGraphicsEllipseItem *draw = nullptr;
    };
    Target target;

    void check_free_path_to_target(const RoboCompLaser::TLaserData &ldata,
                                   const Eigen::Vector2f &goal);

    Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p);

    // doors
    struct Door
    {
        Eigen::Vector2f p1,p2;
        std::set<int> to_rooms;
        const float diff = 400;
        float width() const {return (p1-p2).norm();}
        bool operator ==(const Door &d) { return ((d.p1-p1).norm() < diff and (d.p2-p2).norm() < diff) or
                                                 ((d.p1-p2).norm() < diff and (d.p2-p1).norm() < diff);};
        Eigen::Vector2f get_midpoint() const {return p1 + ((p2-p1)/2.0);};
        Eigen::Vector2f get_external_midpoint() const
        {
            Eigen::ParametrizedLine<float, 2> r =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p1-p2).unitOrthogonal());
            qInfo() << __FUNCTION__ << r.pointAt(800.0).x() << r.pointAt(800.0).y();
            return r.pointAt(1000.0);
        };
        std::optional<int> connecting_room(int inside_room) const
        {
            if( to_rooms.size() == 2 and to_rooms.find(inside_room) != to_rooms.end())
            {
                auto r = std::ranges::find_if_not(to_rooms, [inside_room](auto a) { return a == inside_room; });
                return *r;
            }
            else
                return {};
        };

    };
    std::vector<Door> doors, selected_doors;

    // thanks to https://github.com/CheckBoxStudio/IoU
    struct Room
    {
        IOU::Quad quad;
        int id;
        IOU::Vertexes points;
        const float diff = 300;
        bool operator == (const Room &d)
        {
            double iou = IOU::iou(quad, d.quad);
            return iou > 0.9;
        }
        Room(const IOU::Quad &quad_, int id_) : quad(quad_), id(id_)
        {
            quad.beInClockWise();
            quad.getVertList(points);
//            for(auto p:points)
//                qInfo() << p.x << p.y;
        };
    };
    std::vector<Room> rooms;

};

#endif
