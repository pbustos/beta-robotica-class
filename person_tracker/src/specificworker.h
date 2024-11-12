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
#include <locale>
#include <qcustomplot/qcustomplot.h>
#include "room_detector.h"
#include "dbscan.h"
#include "visibility_graph.h"

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
            float MAX_ADV_SPEED = 1900; // mm/s
            float MAX_ROT_SPEED = 2; // rad/s
            float SEARCH_ROT_SPEED = 0.9; // rad/s
            float STOP_THRESHOLD = 700; // mm
            float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
            float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
            // person
            float PERSON_MIN_DIST = 800; // mm
            int MAX_DIST_POINTS_TO_SHOW = 300; // points to show in plot
            // lidar
            std::string LIDAR_NAME_LOW = "bpearl";
            std::string LIDAR_NAME_HIGH = "helios";
            QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
            // control track
            float acc_distance_factor = 2;
            float k1 = 1.1;  // proportional gain for the angle error;
            float k2 = 0.5; // proportional gain for derivative of the angle error;
        };
        Params params;

        // state machine
        enum class STATE
        {
            TRACK, STOP, WAIT, SEARCH
        };
        STATE state = STATE::TRACK;
        using RetVal = std::tuple<STATE, float, float>;
        using RobotSpeed = std::tuple<float, float>;
        using TPerson = std::expected<RoboCompVisualElementsPub::TObject, std::string>;
        RetVal track(const TPerson &person);
        RetVal wait(const TPerson &person);
        RetVal search(const TPerson &person);
        RetVal stop();
        RobotSpeed state_machine(const TPerson &person);

        // lidar
        std::vector<Eigen::Vector2f> read_lidar_bpearl();
        std::vector<Eigen::Vector2f> read_lidar_helios();

        // draw
        AbstractGraphicViewer *viewer;
        void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
        QGraphicsPolygonItem *robot_draw;
        void draw_person(RoboCompVisualElementsPub::TObject &person, QGraphicsScene *scene) const;
        void draw_path_to_person(const auto &points, QGraphicsScene *scene);
        void draw_obstacles(const vector<QPolygonF> &list_poly, QGraphicsScene *scene, const QColor &color) const;

        // room
        rc::Room_Detector room_detector;
        std::vector<QLineF> detect_wall_lines(const vector<Eigen::Vector2f> &points, QGraphicsScene *scene);
        std::vector<Eigen::Vector2f> remove_wall_points(const std::vector<QLineF> &lines, const auto &bpearl);
        std::vector<QPolygonF> get_walls_as_polygons(const std::vector<QLineF> &lines, float robot_width);
        std::vector<QPolygonF> enlarge_polygons(const std::vector<QPolygonF> &polygons, float amount);

        // person
        std::expected<RoboCompVisualElementsPub::TObject, std::string> find_person_in_data(const std::vector<RoboCompVisualElementsPub::TObject> &objects);
        std::vector<QPolygonF> find_person_polygon_and_remove(const RoboCompVisualElementsPub::TObject &person, const std::vector<QPolygonF> &obstacles);

        // aux
        std::expected<int, string> closest_lidar_index_to_given_angle(const auto &points, float angle);

        // random number generator
        std::random_device rd;

        // DoubleBufferSync to syncronize the subscription thread with compute
        BufferSync<InOut<RoboCompVisualElementsPub::TData, RoboCompVisualElementsPub::TData>> buffer;

        // QCustomPlot object
        QCustomPlot *plot;
        void plot_distance(double distance);

    float running_average(float dist);
};
#endif
