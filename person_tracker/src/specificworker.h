/*
 *    Copyright (C) 2024 by YOUR NAME HERE
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

#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "Lidar3D.h"
#include <expected>
#include <random>
#include <doublebuffer_sync/doublebuffer_sync.h>
//#include "room.h"
//#include "room_detector.h"
//#include "dbscan.h"
//#include "visibility_graph.h"
#include <locale>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);
        void VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data);

    public slots:
        void initialize();
        void compute();
        void emergency();
        void restore();
        int startup_check();

    private:
        bool startup_check_flag;
        struct Params
        {
            float ROBOT_WIDTH = 460;  // mm
            float ROBOT_LENGTH = 480;  // mm
            float MAX_ADV_SPEED = 1500; // mm/s
            float MAX_ROT_SPEED = 2; // rad/s
            float STOP_THRESHOLD = 700; // mm
            float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
            float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
            // person
            float PERSON_MIN_DIST = 800; // mm

            std::string LIDAR_NAME_LOW = "bpearl";
            std::string LIDAR_NAME_HIGH = "helios";
            QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

        };
        Params params;

        // state machine
        enum class STATE
        {
            TRACK, STOP, WAIT
        };
        STATE state = STATE::TRACK;
        using RetVal = std::tuple<STATE, float, float>;
        using RobotSpeed = std::tuple<float, float>;
        RetVal track(const RoboCompVisualElementsPub::TObject &person);
        RetVal wait(const RoboCompVisualElementsPub::TObject &person);
        RetVal stop();
        RobotSpeed state_machine(const RoboCompVisualElementsPub::TObject &person);

        // lidar
        RoboCompLidar3D::TData read_lidar_bpearl();
        RoboCompLidar3D::TData read_lidar_helios();

        // draw
        AbstractGraphicViewer *viewer;
        void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
        QGraphicsPolygonItem *robot_draw;
        void draw_person(RoboCompVisualElementsPub::TObject &person, QGraphicsScene *scene) const;
        void draw_path_to_person(const auto &points, QGraphicsScene *scene);
        // person
        std::expected<RoboCompVisualElementsPub::TObject, std::string> find_person_in_data(const std::vector<RoboCompVisualElementsPub::TObject> &objects);

        // aux
        std::expected<int, string> closest_lidar_index_to_given_angle(const auto &points, float angle);

        // random number generator
        std::random_device rd;

        // DoubleBufferSync to syncronize the subscription thread with compute
        BufferSync<InOut<RoboCompVisualElementsPub::TData, RoboCompVisualElementsPub::TData>> buffer;

};
#endif
