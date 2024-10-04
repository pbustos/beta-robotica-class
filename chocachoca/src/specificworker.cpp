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
#include <ranges>
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
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
	return true;
}

void SpecificWorker::initialize()
{
	std::cout << "Initialize worker" << std::endl;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        ///////////// Your code ////////
        // Viewer
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        viewer->show();

        ///////////////////////////////
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		this->setPeriod(STATES::Compute, 100);
	}
}

void SpecificWorker::compute()
{
    RoboCompLidar3D::TData ldata;
    try{ ldata =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);}
    catch(const Ice::Exception &e){std::cout << e << std::endl;}

    auto p_filter = std::ranges::views::filter(ldata.points,
                                               [](auto  &a){ return a.z > 0.1 and a.z < 0.3 and a.distance2d > 0.2;});

    draw_lidar(p_filter, &viewer->scene);

    /// Add State machine with your sweeping logic
    float advance_speed = 0.f;  // initial robot speeds for this iteration
    float rot_speed = 0.f;
    RetVal ret_val;

    switch(state)
    {
        case STATE::FORWARD:
        {
            ret_val = forward(p_filter);
            break;
        }
        case STATE::TURN:
        {
            ret_val = turn(p_filter);
            break;
        }
    }
    /// unpack  the tuple
    auto [st, adv, rot] = ret_val;
    state = st;

    /// Send movements commands to the robot
    //try{ ldata =  omnirobot_proxy->setSpeedBase(0, adv, rot);
    //catch(const Ice::Exception &e){std::cout << e << std::endl;}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
SpecificWorker::RetVal SpecificWorker::forward(auto &filtered_points)
{

    return SpecificWorker::RetVal();
}

SpecificWorker::RetVal SpecificWorker::turn(auto &filtered_points)
{

    return SpecificWorker::RetVal();
}

/**
 * Draws LIDAR points onto a QGraphicsScene.
 *
 * This method clears any existing graphical items from the scene, then iterates over the filtered
 * LIDAR points to add new items. Each LIDAR point is represented as a colored rectangle. The point
 * with the minimum distance is highlighted in red, while the other points are drawn in green.
 *
 * @param filtered_points A collection of filtered points to be drawn, each containing the coordinates
 *                        and distance.
 * @param scene A pointer to the QGraphicsScene where the points will be drawn.
 */
void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene) const
{
    static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    auto color = QColor(Qt::green);
    auto brush = QBrush(QColor(Qt::green));
    for(const auto &p : filtered_points)
    {
        auto item = scene->addRect(-100, -100, 200, 200, color, brush);
        item->setPos(p.y*1000, p.x*1000);
        items.push_back(item);
    }
    // compute and draw minimum distance point
    auto p_min = std::ranges::min_element(filtered_points, [](auto &a, auto &b){return a.distance2d < b.distance2d;});
    auto item = scene->addRect(-150, -150, 300, 300, QColor(Qt::red), QBrush(QColor(Qt::red)));
    item->setPos(p_min->y*1000, p_min->x*1000);
    items.push_back(item);
    lcdNumber_minangle->display(atan2(p_min->y, p_min->x));
    lcdNumber_mindist->display(p_min->distance2d);
    //qDebug()  << atan2(p_min->y, p_min->x) << p_min->phi;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}
//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)
// this->lidar3d_proxy->getLidarDataArrayProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataWithThreshold2d(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// this->omnirobot_proxy->correctOdometer(...)
// this->omnirobot_proxy->getBasePose(...)
// this->omnirobot_proxy->getBaseState(...)
// this->omnirobot_proxy->resetOdometer(...)
// this->omnirobot_proxy->setOdometer(...)
// this->omnirobot_proxy->setOdometerPose(...)
// this->omnirobot_proxy->setSpeedBase(...)
// this->omnirobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

