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
#include "specificworker.h"
#include <set>
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    try
    {
        COUNT_DOWN = stoi(params.at("max_time").value);
        robot_id = stoi(params.at("robot_id").value);
    }
    catch(const std::exception &e) {std::cout << e.what() << " Error reading params. Aborting" << std::endl; std::terminate();}
    return true;
}


void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    // visor
    viewer = new AbstractGraphicViewer(this->frame, QRectF(-5000, 2500, 10000, -5000));
    auto [rp, rl] = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH);
    robot_polygon = rp;

    // grid
    fm.initialize(QRectF(-5000, -2500, 10000, 5000), 500, &viewer->scene, false);
    connect(pushButton, &QPushButton::clicked, this, &SpecificWorker::reset_time );

	this->Period = period;
	timer.start(Period);

    std::cout << "Worker Initialized" << std::endl;
}

void SpecificWorker::compute()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        omnirobot_proxy->getBaseState(bState);
        bState.x *= 1000;
        bState.z *= 1000;
        bState.alpha += M_PI/2;
        //qInfo() << bState.x << bState.z << bState.alpha;
    }
    catch(const Ice::Exception &ex){std::cout << ex << std::endl; return;}

    static QPointF last_point(bState.x, bState.z);

    robot_polygon->setRotation(qRadiansToDegrees(-bState.alpha));
    robot_polygon->setPos(bState.x, bState.z);
    fm.setVisited(fm.pointToKey((long int)bState.x, (long int)bState.z), true);
    lcdNumber_percent->display(100.0 * fm.count_total_visited() / fm.count_total());
    lcdNumber_time->display((int)(COUNT_DOWN - time.elapsed()/1000));
    items.push_back(viewer->scene.addLine(last_point.x(), last_point.y(), bState.x, bState.z, QPen(QColor("blue"), 20)));
    last_point = QPointF(bState.x, bState.z);
}

void SpecificWorker::reset_time()
{
    time.restart();
    fm.set_all_to_not_visited();
    trail.clear();
    for(const auto &i: items)
        viewer->scene.removeItem(i);
    items.clear();

}




