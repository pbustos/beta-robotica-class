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

        // connect stop button un UI with a lambda function
        connect(pushButton_stop, &QPushButton::clicked, [this]()
            {
                try
                { omnirobot_proxy->setSpeedBase(0, 0, 0); }
                catch (const Ice::Exception &e)
                { std::cout << e << std::endl; }
                std::terminate();
            });

        horizontalSlider_wall_distance->setValue(params.WALL_MIN_DISTANCE);
        lcdNumber_wall_distance->display(params.WALL_MIN_DISTANCE);
        // connect the slider with the wall distance using a lambda
        connect(horizontalSlider_wall_distance, &QSlider::valueChanged, [this](int value)
            {
                params.WALL_MIN_DISTANCE = value;
                lcdNumber_wall_distance->display(value);
            });
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
            label_state->setText("FORWARD");
            break;
        }
        case STATE::TURN:
        {
            ret_val = turn(p_filter);
            label_state->setText("TURN");
            break;
        }
        case STATE::WALL:
        {
            ret_val = wall(p_filter);
            label_state->setText("FOLLOW WALL");
            break;
        }
    }
    /// unpack  the tuple
    auto [st, adv, rot] = ret_val;
    state = st;
    lcdNumber_adv->display(adv);
    lcdNumber_rot->display(rot);

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
    auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);
    if(not offset_begin or not offset_end)
    {
        qWarning() << "No valid readings. Stopping";
        return RetVal(STATE::FORWARD, 0.f, 0.f);
    }

    // FORWARD
    auto min_point = std::min_element(std::begin(points) + offset_begin.value(), std::begin(points) + offset_end.value(), [](auto &a, auto &b)
        { return a.distance2d < b.distance2d; });
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
    static int sign = 1;

    /// check if the narrow central part of the filtered_points vector is free to go. If so stop turning and change state to FORWARD
    auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);

    // exit if no valid readings
    if (not offset_begin or not offset_end)
    {
        qWarning() << "No valid readings. Stopping";
        return RetVal(STATE::TURN, 0.f, 0.f);
    }

    // TURN
    auto min_point = std::min_element(std::begin(points) + offset_begin.value(), std::begin(points) + offset_end.value(), [](auto &a, auto &b)
    { return a.distance2d < b.distance2d; });
    if (min_point != std::end(points) and min_point->distance2d > params.ADVANCE_THRESHOLD)
    {
        first_time = true;
        //return RetVal(STATE::FORWARD, 0.f, 0.f);
        return RetVal(STATE::WALL, 0.f, 0.f);
    }

    /// Keep doing my business
    // compute the accumulated sum of all point in the left side of the robot
    auto half = closest_lidar_index_to_given_angle(points, 0.f);
    auto left_sum = std::accumulate(std::begin(points), std::begin(points) + half.value(), 0.f, [](auto a, auto b) { return a + 1.f/b.distance2d; });
    auto right_sum = std::accumulate(std::begin(points) + half.value(), std::end(points), 0.f, [](auto a, auto b) { return a + 1.f/b.distance2d; });
    // if there is more obstacles on the left, turn right, otherwise turn left. If it is close to zero, turn randomly
    if (first_time)
    {
        if (fabs(left_sum-right_sum) < 1)  // if the difference is too small, turn randomly
        {
            sign = dist(gen);
            if (sign == 0) sign = -1; else sign = 1;
        }
        else
            sign = left_sum > right_sum ? 1 : -1;
        first_time = false;
    }
    return RetVal(STATE::TURN, 0.f, sign * params.MAX_ROT_SPEED);
}
/**
 * @brief Determines the robot's behavior when following a wall.
 *
 * This method analyzes the filtered points to determine the robot's behavior when following a wall.
 * It first checks if the robot is about to crash into an obstacle, in which case it stops and changes
 * the state to TURN. If no obstacle is detected, it then calculates the distance to the wall on the
 * robot's side and computes the necessary speed and rotation to maintain a safe distance from the wall.
 *
 * @param filtered_points A vector containing points with distance information used for making navigation decisions.
 * @returns A tuple containing the next state (WALL), and speed values.
 */
SpecificWorker::RetVal SpecificWorker::wall(auto &filtered_points)
{
    static bool first_time = true;

    // check if about to crash
    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION);
    auto min_point = std::min_element(std::begin(filtered_points) + offset_begin.value(), std::begin(filtered_points) + offset_end.value(), [](auto &a, auto &b)
    { return a.distance2d < b.distance2d; });
    if(min_point->distance2d < params.STOP_THRESHOLD)
    {
        first_time = true;
        return RetVal(STATE::TURN, 0.f, 0.f);  // stop and change state if obstacle detected
    }

    // get lidar readings in the sides of the robot
    RoboCompLidar3D::TPoint min_obj;
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_RIGHT_SIDE_SECTION);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_LEFT_SIDE_SECTION);
    if (not res_right or not res_left)   // abandon the ship
    {
        qWarning() << "No valid lateral readings" << QString::fromStdString(res_right.error()) << QString::fromStdString(res_left.error());
        return RetVal(STATE::WALL, 0.f, 0.f);
    }
    auto right_point = filtered_points[res_right.value()];
    auto left_point = filtered_points[res_left.value()];
    if(first_time)    // compare both to get the one with minimum distance and keep it until next TURN
    {
        handness = (right_point.distance2d < left_point.distance2d) ? HANDNESS::RIGHT : HANDNESS::LEFT;
        label_handness->setText((handness == HANDNESS::RIGHT ? "RIGHT" : "LEFT"));
        first_time = false;
    }
    min_obj = handness == HANDNESS::RIGHT ? right_point : left_point;

    // compute the distance to the virtual line that has to be followed. Positive if the robot is too far from the wall, negative otherwise
    auto error = min_obj.distance2d - params.WALL_MIN_DISTANCE;
    lcdNumber_error->display(error);

    // compute breaks
    auto adv_brake = std::clamp(-1.f/(params.ROBOT_WIDTH/2.f) * std::fabs(error) + 1.f, 0.f, 1.f);
    auto rot_brake = std::clamp(1.f/(params.ROBOT_WIDTH/3.f) * std::fabs(error), 0.f, 1.f);

    // check the left/right hand side and the distance to the wall conditions
    if(min_obj.phi >= 0 and error >= 0)   // right hand side and too far from the wall: turn left
        return RetVal(STATE::WALL, params.MAX_ADV_SPEED * adv_brake, params.MAX_ROT_SPEED * rot_brake);
    if(min_obj.phi >= 0 and error < 0)   // right hand side and too close to the wall: turn right
        return RetVal(STATE::WALL, params.MAX_ADV_SPEED * adv_brake, -params.MAX_ROT_SPEED * rot_brake);
    if(min_obj.phi < 0 and error >= 0)   // left hand side and too far from the wall: turn left
        return RetVal(STATE::WALL, params.MAX_ADV_SPEED * adv_brake, -params.MAX_ROT_SPEED * rot_brake);
    if(min_obj.phi < 0 and error < 0)   // left hand side and too close to the wall: turn right
        return RetVal(STATE::WALL, params.MAX_ADV_SPEED * adv_brake, params.MAX_ROT_SPEED * rot_brake);

    qWarning() << "We should not reach this point. Stopping";
    return RetVal (STATE::WALL, 0.f, 0.f);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Your code here
//////////////////////////////////////////////////////////////////////////////////////////////////
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

    // compute and draw minimum distance point in frontal range
    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION);
    if(not offset_begin or not offset_end)
    { std::cout << offset_begin.error() << " " << offset_end.error() << std::endl; return ;}    // abandon the ship
    auto min_point = std::min_element(std::begin(filtered_points) + offset_begin.value(), std::begin(filtered_points) + offset_end.value(), [](auto &a, auto &b)
    { return a.distance2d < b.distance2d; });
    QColor dcolor;
    if(min_point->distance2d < params.STOP_THRESHOLD)
        dcolor = QColor(Qt::red);
    else
        dcolor = QColor(Qt::magenta);
    auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    ditem->setPos(min_point->x, min_point->y);
    items.push_back(ditem);

    // compute and draw minimum distance point to wall
    auto wall_res_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_RIGHT_SIDE_SECTION);
    auto wall_res_left = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_LEFT_SIDE_SECTION);
    if(not wall_res_right or not wall_res_left)   // abandon the ship
    {
        qWarning() << "No valid lateral readings" << QString::fromStdString(wall_res_right.error()) << QString::fromStdString(wall_res_left.error());
        return;
    }
    auto right_point = filtered_points[wall_res_right.value()];
    auto left_point = filtered_points[wall_res_left.value()];
    // compare both to get the one with minimum distance
    auto min_obj = (right_point.distance2d < left_point.distance2d) ? right_point : left_point;
    auto item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::orange), QBrush(QColor(QColorConstants::Svg::orange)));
    item->setPos(min_obj.x, min_obj.y);
    items.push_back(item);

    // draw a line from the robot to the minimum distance point
    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)), QPen(QColorConstants::Svg::orange, 10));
    items.push_back(item_line);

    // update UI
    lcdNumber_minangle->display(atan2(min_obj.x,min_obj.y));
    lcdNumber_mindist->display(min_obj.distance2d);

    // Draw two lines coming out from the robot at angles given by params.LIDAR_OFFSET
    // Calculate the end points of the lines
    //float angle1 = params.LIDAR_FRONT_SECTION / 2.f;
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION);
    if(not res_right or not res_left)
    { std::cout << res_right.error() << " " << res_left.error() << std::endl; return ;}

    float right_line_length = filtered_points[res_right.value()].distance2d;
    float left_line_length = filtered_points[res_left.value()].distance2d;
    float angle1 = filtered_points[res_left.value()].phi;
    float angle2 = filtered_points[res_right.value()].phi;
    QLineF line_left{QPointF(0.f, 0.f),
                     robot_draw->mapToScene(left_line_length * sin(angle1), left_line_length * cos(angle1))};
    QLineF line_right{QPointF(0.f, 0.f),
                      robot_draw->mapToScene(right_line_length * sin(angle2), right_line_length * cos(angle2))};
    QPen left_pen(Qt::blue, 10); // Blue color pen with thickness 3
    QPen right_pen(Qt::red, 10); // Blue color pen with thickness 3
    auto line1 = scene->addLine(line_left, left_pen);
    auto line2 = scene->addLine(line_right, right_pen);
    items.push_back(line1);
    items.push_back(line2);
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
/// Auxiliary methods
//////////////////////////////////////////////////////////////////////////////////////////////////
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

