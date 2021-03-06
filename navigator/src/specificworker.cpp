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
#include "specificworker.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "grid.h"
#include <QGraphicsSceneMouseEvent>


using namespace std;

const float landa = -0.5 / log(0.1);

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx) {
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    try {
        RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
        std::string innermodel_path = par.value;
        innerModel = std::make_shared<InnerModel>(innermodel_path);
    }
    catch (const std::exception &e) { qFatal("Error reading config params"); }
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;

    // graphics
    graphicsView = new QGraphicsView(this);
    graphicsView->resize(this->size());
    graphicsView->setScene(&scene);
    graphicsView->setMinimumSize(400, 400);
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    struct Dimensions {
        int TILE_SIZE = 100;
        float HMIN = -2500, VMIN = -2500, WIDTH = 5000, HEIGHT = 5000;
    };
    Dimensions dim;
    scene.setSceneRect(dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT);
    graphicsView->scale(1, -1);
    connect(&scene, &MyScene::new_target, this, [this](QGraphicsSceneMouseEvent *e)
    {
        qInfo() << "Lambda SLOT: " << e->scenePos();
        target_buffer.put(std::make_tuple(e->scenePos().x(), 0, e->scenePos().y()));
    });
    graphicsView->show();

    //robot
    QPolygonF poly2;
    float size = ROBOT_LENGTH / 2.f;
    poly2 << QPointF(-size, -size)<<QPointF(-size, size)<<QPointF(size, size) << QPoint(size, -size);
    QColor rcolor("DarkGreen"); rcolor.setAlpha(90);
    QBrush brush(rcolor);
    brush.setStyle(Qt::SolidPattern);
    robot_polygon_draw = scene.addPolygon(poly2, QPen(QColor("DarkGreen")), brush);
    robot_polygon_draw->setZValue(15);
    auto front = scene.addEllipse(0,0,50,50, QPen(QColor("White")), QBrush(QColor("White")));
    front->setParentItem(robot_polygon_draw);
    front->setPos(0,150);
    RoboCompGenericBase::TBaseState bState;
    try
    {
        omnirobot_proxy->getBaseState(bState);
        robot_polygon_draw->setRotation(qRadiansToDegrees(-bState.alpha));
        robot_polygon_draw->setPos(bState.x, bState.z);
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio);

    // grid
    grid.create_graphic_items(scene, graphicsView);
    fill_grid_with_obstacles();
    // recorrer las cajas y poner a ocupado todos las celdas que caigan
    // recorrer las pared y poner las celdas a rojo

    this->Period = 100;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    //Coordenadas del robot
    RoboCompGenericBase::TBaseState bState;
    try { omnirobot_proxy->getBaseState(bState);  }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

    RoboCompLaser::TLaserData ldata;
    try { ldata = laser_proxy->getLaserData(); }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

    if (auto data = target_buffer.get(); data.has_value())
    {
        target = data.value();
        auto [x,y,z] = target;
        if(target_draw) scene.removeItem(target_draw);
        target_draw  = scene.addEllipse(x-50, z-50, 100, 100, QPen(QColor("red")), QBrush(QColor("red")));
        // calcular la función de navegación
        if( auto tg = grid.get_value(x,z) ; tg.has_value())
        {
            target_cell = tg.value();
            navigation_function(target_cell);
        }
        else
            qWarning() << __FUNCTION__ << "Target out of bounds";
    }
    std::vector<Tupla> points;
    if( target_buffer.is_active())
    {
        Eigen::Vector2f tr = transformar_targetRW(bState, target_cell);
        auto dist = tr.norm();
        //qInfo() << __FUNCTION__ << dist;
        if (dist < 70)
        {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            target_buffer.set_task_finished();
            std::cout << __FUNCTION__ << " At target" << std::endl;
            omnirobot_proxy->setSpeedBase(0, 0, 0);
        }
        //buscar el vecino más bajo en el grid

        if(auto robot_cell = grid.get_value(bState.x, bState.z); robot_cell.has_value())
        {
            //auto sub_target = grid.get_steepest_direction(robot_cell.value(), target_cell);
            //if(subtarget_draw) scene.removeItem(subtarget_draw);
            //subtarget_draw = scene.addRect(sub_target.cx-50, sub_target.cy-50, 100, 100, QPen(QColor("magenta")), QBrush(QColor("magenta")));
            //llamar a DWA con ese punto
            points = dynamicWindowApproach(bState, ldata, target_cell);
        }
    }
    draw_things(bState, ldata, points);
}

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::fill_grid_with_obstacles()
{
    for(int i=1; i<10; i++)
    {
        auto caja = "caja" + QString::number(i);
        auto node = innerModel->getNode(caja);
        auto mesh = innerModel->getNode("cajaMesh" + QString::number(i));
        if(node and mesh)
        {
            auto pose = innerModel->transform("world", caja);
            auto plane = dynamic_cast<InnerModelPlane*>(mesh);
            std::cout << __FUNCTION__ << " " << pose.x() << " " << pose.z() << " " << plane->depth << " " << plane->width << std::endl;
            int x = pose.x();
            int z = pose.z();
            int width = plane->depth;
            int height = plane->width;
            for(int k = x - width/2; k<= x + width/2; k++)
                for(int l = z - height/2; l<= z + height/2; l++)
                    grid.set_occupied(k,l);
        }
    }
}

void SpecificWorker::navigation_function(const MyGrid::Value &target)
{
    grid.reset_cell_distances();
    float dist = 0;
    auto L1 = grid.neighboors(target, dist++);
    std::vector<MyGrid::Value> L2;
    bool end = false;
    while( not end )
    {
        for(const auto &current_cell : L1)
        {
            auto selected = grid.neighboors( current_cell, dist);
            L2.insert(std::end(L2), std::begin(selected), std::end(selected));
        }
        dist++;
        end = L2.empty();
        L1.swap(L2);
        L2.clear();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<SpecificWorker::Tupla> SpecificWorker::dynamicWindowApproach(const RoboCompGenericBase::TBaseState &bState,
                                                                          const RoboCompLaser::TLaserData &ldata,
                                                                          const MyGrid::Value &target )
{

//posiciones originales del robot
    float current_adv = bState.advVz; // Advance V
    float current_rot = bState.rotV; // Rotation W
    static float previous_turn = 0;

    // calculamos las posiciones futuras del robot y se insertan en un vector.
    auto vector_arcos = calcularPuntos(current_adv, current_rot);

    // quitamos los puntos futuros que nos llevan a obstaculos
    auto vector_sin_obs = obstaculos(vector_arcos, bState.alpha, ldata);

    // ordenamos el vector de puntos segun la distancia
    Eigen::Vector2f tr = transformar_targetRW(bState, target);
    auto best_choice = ordenar(vector_sin_obs, tr.x(), tr.y(), bState.x, bState.z, previous_turn);

    if (best_choice.has_value())
    {
        auto[x, y, v, w, alpha] = best_choice.value();
        //std::cout << __FUNCTION__ << " " << x << " " << y << " " << v << " " << w << " " << alpha << std::endl;
        auto va = std::min(v / 5, 1000.f);
        try{  omnirobot_proxy->setSpeedBase(0, va, -w); previous_turn = -w;}  // w should come positive
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
        vector_sin_obs.insert(vector_sin_obs.begin(), best_choice.value());
        return vector_sin_obs;
    }
    else
    {
        //std::cout << "Vector vacio" << std::endl;
        return std::vector<Tupla>{};
    }
}


std::vector<std::vector<SpecificWorker::Tupla>> SpecificWorker::calcularPuntos(float current_adv, float current_rot)
    {
        std::vector <Tupla> vectorT;
        std::vector<std::vector<Tupla>> list_arcs;
        const float semiwidth = 50;
        //Calculamos las posiciones futuras del robot y se insertan en un vector.
        float dt = 2; // 1 second ahead
        for (float v = -100; v <= 700; v += 100) //advance
        {
            for (float w = -2; w <= 2; w += 0.1) //rotacion
            {
                std::vector<Tupla> list_points;
                float new_adv = current_adv + v;
                float new_rot = current_rot + w;
                if (fabs(w) > 0.01)
                {
                    // Nuevo punto posible
                    float r = new_adv / new_rot; // radio de giro ubicado en el eje x del robot
                    float x = r - r * cos(new_rot * dt); //coordenada futura X
                    float y = r * sin(new_rot * dt); //coordenada futura Z
                    float alp = new_rot * dt; //angulo nuevo del robot
                    float arc_length = new_rot * dt * r;
                    for (float t = semiwidth; t < arc_length; t += semiwidth)
                        list_points.emplace_back(std::make_tuple(r - r * cos(t / r), r * sin(t / r), new_adv, new_rot, t / r));
                }
                else // para evitar la división por cero en el cálculo de r
                {
                    for(float t = semiwidth; t < v*dt; t+=semiwidth)
                        list_points.emplace_back(std::make_tuple(0.f, t, new_adv, new_rot, new_rot*dt));
                }
                list_arcs.push_back(list_points);
            }
        }
        return list_arcs;
    }

    std::vector <SpecificWorker::Tupla>
    SpecificWorker::obstaculos(std::vector<std::vector<Tupla>> vector_arcs, float aph, const RoboCompLaser::TLaserData &ldata)
    {
        QPolygonF polygonF_Laser;
        const float semiancho = 250; // el semiancho del robot
        std::vector<Tupla> vector_obs;

        // poligono creado con los puntos del laser
        for (auto &l: ldata)
            polygonF_Laser << QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle));
        // extend laser to include the robot's body
        //        float size = ROBOT_LENGTH / 1.4;
        //        polygonF_Laser << QPointF(-size,size) << QPointF(-size,-size) << QPointF(size,-size) << QPointF(size,size);

        for(auto &arc_points : vector_arcs)
        {
            for (auto &point : arc_points)
            {
                auto [x, y, adv, giro, ang] = point;
                QPolygonF temp_robot;
                temp_robot << QPointF(x - semiancho, y + semiancho) << QPointF(x + semiancho, y + semiancho) <<
                           QPointF(x + semiancho, y - semiancho) << QPointF(x - semiancho, y - semiancho);
                temp_robot = QTransform().rotate(ang).map(temp_robot);

                //si el poligono del laser no contiene un punto del robot, no contiene alguna esquina por tanto pasamos a otro.

                auto res = std::find_if_not(std::begin(temp_robot), std::end(temp_robot), [polygonF_Laser](const auto &p){return polygonF_Laser.containsPoint(p,Qt::OddEvenFill);});
                if(res == std::end(temp_robot))  //all inside
                     vector_obs.emplace_back(point);
                else
                    break;
            }
        }
        return vector_obs;
    }

/**
 * Ordenamos el vector segun distancia a las coordenadas x y z
 * @param vector
 * @param x
 * @param z
 * @return vector ordenado
 */
    std::optional<SpecificWorker::Tupla> SpecificWorker::ordenar(std::vector<Tupla> vector_points, float tx, float ty, float rx, float ry, float previous_turn)
    {
        const float A=1, B=0.1, C=0.1, D=0;
        int k=0;
        std::vector<std::tuple<float, Tupla>> values;
        values.resize(vector_points.size());
        for(auto &point : vector_points)
            {
                auto [x, y, adv, giro, ang] = point;
                auto va = this->grid.get_value(x,y); auto vb = this->grid.get_value(x,y);
                if(va.has_value() and vb.has_value())
                {
                    float nav_function = va.value().dist;
                    float dist_to_target = sqrt(pow(tx-x,2)+pow(ty-y,2));
                    float dist_to_previous_turn =  fabs(ang - previous_turn);
                    //float dist_from_robot = 1/sqrt(pow(rx-x,2)+pow(ry-y,2));
                    //float clearance_to_obstacle = 1/grid.dist_to_nearest_obstacle(x, y);
                    values[k++] = std::make_tuple(A * nav_function + B* dist_to_target + C*dist_to_previous_turn, point);
                }
            }
        auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
        if(min != values.end())
            return std::get<Tupla>(*min);
        else
            return {};

//        std::vector<tupla> vdist = vector;
//        std::sort(vdist.begin(), vdist.end(), [x, z, this](const auto &a, const auto &b)
//        {
//            const auto &[ax, ay, ca, cw, aa] = a;
//            const auto &[bx, by, ba, bw, bb] = b;
//            //return ((ax - x) * (ax - x) + (ay - z) * (ay - z)) < ((bx - x) * (bx - x) + (by - z) * (by - z));
//            auto va = this->grid.get_value(ax,ay); auto vb = this->grid.get_value(bx,by);
//            if(va.has_value() and vb.has_value())
//                return va.value().dist < vb.value().dist;
//            else
//                return false;
//        });
//
//        return vector;
    }

void SpecificWorker::draw_things(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata, const std::vector <Tupla> &puntos)
{
    //draw robot
    //innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
    robot_polygon_draw->setRotation(qRadiansToDegrees(bState.alpha));
    robot_polygon_draw->setPos(bState.x, bState.z);
    graphicsView->resize(this->size());

    //draw laser
    if (laser_polygon_draw != nullptr)
        scene.removeItem(laser_polygon_draw);
    QPolygonF poly;
    float size = ROBOT_LENGTH / 1.4;
    for (auto &l : ldata)
        poly << robot_polygon_draw->mapToScene(QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle)));
    poly += robot_polygon_draw->mapToScene(QPointF(-size,size));
    poly += robot_polygon_draw->mapToScene(QPointF(-size,-size));
    poly += robot_polygon_draw->mapToScene(QPointF(size,-size));
    poly += robot_polygon_draw->mapToScene(QPointF(size,size));
    QColor color("LightPink");
    color.setAlpha(60);
    laser_polygon_draw = scene.addPolygon(poly, QPen(color), QBrush(color));
    laser_polygon_draw->setZValue(13);

    // draw future. Draw and arch going out from the robot
    // remove existing arcspwd
    for (auto arc: arcs_vector)
        scene.removeItem(arc);
    arcs_vector.clear();
    QColor col("Blue");
    for (auto &[x, y, vx, wx, a] : puntos)
    {
        QPointF centro = robot_polygon_draw->mapToScene(x, y);
        arcs_vector.push_back(scene.addEllipse(centro.x(), centro.y(), 20, 20, QPen(col), QBrush(col)));
    }
    if(not puntos.empty())
    {
        auto front = puntos.front();
        QPointF selected = robot_polygon_draw->mapToScene(std::get<0>(front), std::get<1>(front));
        arcs_vector.push_back(scene.addEllipse(selected.x(), selected.y(), 80, 80, QPen(Qt::black), QBrush(Qt::black)));
    }
}

/**
* A traves de un punto calculamos las coordenadas del punto en el mundo real y las transformamos al mundo del robot
* @param bState
* @return
*/

Eigen::Vector2f SpecificWorker::transformar_targetRW(RoboCompGenericBase::TBaseState bState, const MyGrid::Value &target)
{
    //Target mundo real
    Eigen::Vector2f tw(target.cx, target.cy);

    // Robot mundo robot
    Eigen::Vector2f rw(bState.x, bState.z);

    // Inicializamos una matriz de 2x2 en sentido horario.
    Eigen::Matrix2f rot;
    rot << cos(bState.alpha), sin(bState.alpha),
            -sin(bState.alpha), cos(bState.alpha);

    // Guardamos en un vector el resultado de la transpuesta de rot por la resta de tw - rw
    Eigen::Vector2f tr = rot * (tw - rw);  //no trasnpose for Coppelia
    return tr;
}

////////////////////////////////////////////////////////////////
    int SpecificWorker::startup_check() {
        std::cout << "Startup check" << std::endl;
        QTimer::singleShot(200, qApp, SLOT(quit()));
        return 0;
    }

/**
 * SUBSCRIPTION to setPick method from RCISMousePicker interface
 * @param myPick
 */
    void SpecificWorker::RCISMousePicker_setPick(RoboCompRCISMousePicker::Pick myPick) {

        target_buffer.put(std::make_tuple(myPick.x, myPick.y, myPick.z)); //metemos las coordenadas con el mutex iniciado
        //Coordenadas del target
        std::cout << "x: " << myPick.x;
        std::cout << "..y: " << myPick.y;
        std::cout << "..z: " << myPick.z << std::endl;
    }

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompRCISMousePicker you can use this types:
// RoboCompRCISMousePicker::Pick

//  //posicion del robot ocupada
//        grid.set_Value(bState.x, bState.z, true);
//        //posiciones de las cajas ocupadas
//        auto caja1 = innerModel->getTransform("caja1");
//        if (caja1)
//            grid.set_Value(caja1->backtX, caja1->backtZ, true);
//
//        auto caja2 = innerModel->getTransform("caja2");
//        if (caja2)
//            grid.set_Value(caja2->backtX, caja2->backtZ, true);
//
//        auto caja3 = innerModel->getTransform("caja3");
//        if (caja3)
//            grid.set_Value(caja3->backtX, caja3->backtZ, true);
//
//        auto caja4 = innerModel->getTransform("caja4");
//        if (caja4)
//            grid.set_Value(caja4->backtX, caja4->backtZ, true);
//
//        auto caja5 = innerModel->getTransform("caja5");
//        if (caja5)
//            grid.set_Value(caja5->backtX, caja5->backtZ, true);
//
//        auto caja6 = innerModel->getTransform("caja6");
//        if (caja6)
//            grid.set_Value(caja6->backtX, caja6->backtZ, true);
