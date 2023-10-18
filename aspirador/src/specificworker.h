/*
 *    Copyright (C)2019 by YOUR NAME HERE
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
#include <grid2d/grid.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

    public slots:
        void compute();
        void initialize(int period);
        void reset_time();

        //--------------------
    private:
        Grid fm;
        QElapsedTimer time;
        int COUNT_DOWN=180; //secs
        AbstractGraphicViewer *viewer;

        //robot
        const int ROBOT_LENGTH = 400;
        QGraphicsPolygonItem *robot_polygon;
        int robot_id;
        std::vector<QPointF> trail;
        std::vector<QGraphicsItem *> items;
        void draw_trail(const QPointF &new_point);
};

#endif
