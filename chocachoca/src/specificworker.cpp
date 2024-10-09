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
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;
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

    RoboCompLidar3D::TPoints  p_filter;
    std::ranges::copy_if(ldata.points, std::back_inserter(p_filter),
                                               [](auto  &a){ return a.z < 500 and a.distance2d > 200;});

    draw_lidar(p_filter, &viewer->scene);

    /// Add State machine with your sweeping logic
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
    try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
    catch(const Ice::Exception &e){std::cout << e << std::endl;}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Analyzes the filtered points to determine whether to continue moving forward or to stop and turn.
 *
 * This method examines the central part of the `filtered_points` vector to find the minimum distance
 * point within that range. If the minimum distance is less than the width of the robot, it indicates
 * an obstacle is too close, prompting a state change to `TURN` and stopping motion. Otherwise,
 * the robot continues to move forward.
 *
 * @param filtered_points A vector of filtered points representing the robot's perception of obstacles.
 * @return A `RetVal` tuple consisting of the state (`FORWARD` or `TURN`), speed, and rotation.
 */
SpecificWorker::RetVal SpecificWorker::forward(auto &points)
{
    // check if the central part of the filtered_points vector has a minimum lower than the size of the robot
    int offset = params.LIDAR_OFFSET * (points.size() / 2);
    auto min_point = std::min_element(std::begin(points) + offset, std::end(points) - offset, [](auto &a, auto &b)
        {  return a.distance2d < b.distance2d; });
    if (min_point != points.end() and min_point->distance2d < params.STOP_THRESHOLD)
            return RetVal(STATE::TURN, 0.f, 0.f);  // stop and change state if obstacle detected
    else
        return RetVal(STATE::FORWARD, params.MAX_ADV_SPEED, 0.f);
}

/**
 * @brief Checks if the central part of the provided filtered points is free to proceed and determines the next state.
 *
 * This function inspects the central third of the filtered points vector to find the point with the minimum distance.
 * If the minimum distance in this central region is greater than twice the robot's width, the robot will switch to
 * the FORWARD state. Otherwise, it will continue to TURN.
 *
 * @param filtered_points A vector containing points with distance information used for making navigation decisions.
 * @returns A tuple containing the next state (FORWARD or TURN), and speed values.
 */
SpecificWorker::RetVal SpecificWorker::turn(auto &points)
{
    // Instantiate the random number generator and distribution
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<int> dist(0, 1);
    static bool first_time = true;

    // check if the central part of the filtered_points vector is free to go. If so stop turning and change state to FORWARD
    int offset = params.LIDAR_OFFSET * (points.size() / 2);
    auto min_point = std::min_element(std::begin(points) + offset, std::end(points) - offset, [](auto &a, auto &b)
        { return a.distance2d < b.distance2d; });
    if(min_point != std::end(points) and min_point->distance2d > params.ADVANCE_THRESHOLD)
    {
        first_time = true;
        return RetVal(STATE::FORWARD, 0.f, 0.f);
    }
    else    // Keep doing my business
    {
        // Generate a random sign (-1 or 1) if first_time = true;
        int sign = 1;
        if(first_time)
        {
            sign = dist(gen);
            if (sign == 0) sign = -1; else sign = 1;
            first_time = false;
        }
        return RetVal(STATE::TURN, 0.f, sign * params.MAX_ROT_SPEED);
    }
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
void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene)
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
        auto item = scene->addRect(-50, -50, 100, 100, color, brush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }
    // compute and draw minimum distance point
    auto p_min = std::ranges::min_element(filtered_points, [](auto &a, auto &b){return a.distance2d < b.distance2d;});
    auto item = scene->addRect(-150, -150, 300, 300, QColor(Qt::red), QBrush(QColor(Qt::red)));
    item->setPos(p_min->x, p_min->y);
    items.push_back(item);


    // Draw two lines coming out from the robot at angles given by params.LIDAR_OFFSET
    // Calculate the end points of the lines
    //float angle1 = params.LIDAR_FRONT_SECTION / 2.f;
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION/2.f);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION/2.f);
    if(res_right and res_left)
    {
        float right_line_length = filtered_points[res_right.value()].distance2d;
        float left_line_length = filtered_points[res_left.value()].distance2d;
        float angle1 = params.LIDAR_FRONT_SECTION/2.f;
        float angle2 = -angle1;
        int x1_end = right_line_length * sin(angle1);
        int y1_end = right_line_length * cos(angle1);
        int x2_end = left_line_length * sin(angle2);
        int y2_end = left_line_length * cos(angle2);

        QPen left_pen(Qt::blue, 10); // Blue color pen with thickness 3
        QPen right_pen(Qt::red, 10); // Blue color pen with thickness 3
        auto line1 = scene->addLine(QLineF(robot_draw->mapToScene(0, 0), robot_draw->mapToScene(x1_end, y1_end)), left_pen);
        auto line2 = scene->addLine(QLineF(robot_draw->mapToScene(0, 0), robot_draw->mapToScene(x2_end, y2_end)), right_pen);
        items.push_back(line1);
        items.push_back(line2);
    }
    else
        std::cout << res_right.error() << " " << res_left.error() << std::endl;

    // update UI
    lcdNumber_minangle->display(atan2(p_min->x,p_min->y));
    lcdNumber_mindist->display(p_min->distance2d);
}

/**
 * @brief Calculates the index of the closest lidar point to the given angle.
 *
 * This method searches through the provided list of lidar points and finds the point
 * whose angle (phi value) is closest to the specified angle. If a matching point is found,
 * the index of the point in the list is returned. If no point is found that matches the condition,
 * an error message is returned.
 *
 * @param points The collection of lidar points to search through.
 * @param angle The target angle to find the closest matching point.
 * @return std::expected<int, string> containing the index of the closest lidar point if found, 
 * or an error message if no such point exists.
 */
std::expected<int, string> SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle)
{
    // search for the point in points whose phi value is closest to angle
    auto res = std::ranges::find_if(points, [angle](auto &a){ return a.phi > angle;});
    if(res != std::end(points))
        return std::distance(std::begin(points), res);
    else
        return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
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

