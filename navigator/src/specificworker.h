/*
 *    Copyright (C) 2020 by jvallero & mtorocom
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
	@author jvallero & mtorocom
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <Eigen/Dense>
#include <ranges>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include "grid.h"
#include <cppitertools/range.hpp>

using namespace Eigen;

class MyScene : public QGraphicsScene
{
    Q_OBJECT
    signals:
        void new_target(QGraphicsSceneMouseEvent *);
    protected:
        void mousePressEvent(QGraphicsSceneMouseEvent *event)
        { emit new_target(event); }
};

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
        template<typename T>
        struct Target
        {
            T data;
            std::mutex mutex;
            bool activate = false;
            bool empty = true;

            void put(const T &Data)
            {
                std::lock_guard<std::mutex> guard(mutex);
                data = Data;
                activate = true;
                empty = false;
            }

            std::optional<T> get()
            {
                std::lock_guard<std::mutex> guard(mutex);
                if (not empty)
                {
                    empty = true;
                    return data;
                }
                else
                    return {};
            }

            void set_task_finished()
            {
                std::lock_guard<std::mutex> guard(mutex);
                activate = false;
            }

            bool is_active()
            {
                std::lock_guard<std::mutex> guard(mutex);
                return activate;
            }
        };

    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);
        void RCISMousePicker_setPick(RoboCompRCISMousePicker::Pick myPick);


    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    protected:
        void resizeEvent(QResizeEvent * event)
        {
            graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio);
        }

    private:
        std::shared_ptr<InnerModel> innerModel;
        bool startup_check_flag;

        //tupla de 3 variables float para las coordenadas x,y,z.
        using Tpose = std::tuple<float, float, float>;

        //grid
        using MyGrid = Grid<int, -2500, int, 5000, int, 100>;
        MyGrid grid;

        //variable tipo Target con la tupla Tpose
        Target<Tpose> target_buffer;
        Tpose target;
        MyGrid::Value target_cell;
        QGraphicsEllipseItem *target_draw = nullptr;
        QGraphicsRectItem *subtarget_draw = nullptr;

        using Tupla = std::tuple<float, float, float, float, float>;
        Eigen::Vector2f transformar_targetRW(RoboCompGenericBase::TBaseState bState,  const MyGrid::Value &target);

        //DWA
        std::vector<std::vector<Tupla>> calcularPuntos(float vOrigen, float wOrigen);
        std::optional<Tupla> ordenar(std::vector<Tupla> vector, float x, float y, float rx, float ry, float previous_turn);
        std::vector<Tupla> obstaculos(std::vector<std::vector<Tupla>> vector, float aph, const RoboCompLaser::TLaserData &ldata);
        std::vector<Tupla> dynamicWindowApproach(const RoboCompGenericBase::TBaseState &bState,
                                                              const RoboCompLaser::TLaserData &ldata,
                                                              const MyGrid::Value &target_cell);
        void fill_grid_with_obstacles();

        // NF
        void navigation_function(const MyGrid::Value &target);

        //draw
        MyScene scene;
        QGraphicsView *graphicsView;
        QGraphicsPolygonItem *robot_polygon_draw = nullptr;
        QPolygonF robot_polygon;
        QGraphicsItem *laser_polygon_draw = nullptr;
        const float ROBOT_LENGTH = 400;

        void
        draw_things(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata, const std::vector<Tupla> &puntos);
        std::vector<QGraphicsEllipseItem *> arcs_vector;


};

#endif
