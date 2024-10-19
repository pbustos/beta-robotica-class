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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
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
        // Viewer
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;
        viewer->setStyleSheet("background-color: lightGray;");


        // connect stop button un UI with a lambda function
        connect(pushButton_stop, &QPushButton::clicked, [this]()
        {
            try
            { omnirobot_proxy->setSpeedBase(0, 0, 0); }
            catch (const Ice::Exception &e)
            { std::cout << e << std::endl; }
            pushButton_stop->setText(pushButton_stop->isChecked() ? "Track" : "Stop");
        });
        viewer->show();

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);

	}
}
void SpecificWorker::compute()
{
    // read lidar and draw
    auto ldata = read_lidar("bpearl");
    if(ldata.points.empty()) { qWarning() << "Empty lidar data"; return; };
    draw_lidar(ldata.points, &viewer->scene);

    // check if there is YOLO data in buffer
    RoboCompVisualElementsPub::TData data;
    auto [data_] = buffer.read_first();
    if(not data_.has_value())
        { qWarning() << __FUNCTION__ << "Empty buffer"; return; }
    else data = data_.value();

    // get person and draw on viewer
    RoboCompVisualElementsPub::TObject person;
    if(auto p_ = std::ranges::find_if(data.objects, [](auto &a){ return a.id == 0;}); p_ == std::end(data.objects))
    { qWarning() << __FUNCTION__ << "No person detected"; return;}
    else person = *p_;

    // print person attributes
    //for(const auto &[key, val]: person->attributes)
        //std::cout << key << " " << val << std::endl;
    draw_person(person, &viewer->scene);

    // call state machine to track person
    const auto &[adv, rot] = state_machine(person, ldata.points);

    // move the robot
    try{ omnirobot_proxy->setSpeedBase(0.f, adv, rot); }
    catch(const Ice::Exception &e){std::cout << e << std::endl;}
    lcdNumber_adv->display(adv);
    lcdNumber_rot ->display(rot);
}

//////////////////////////////////////////////////////////////////
/// YOUR CODE HERE
//////////////////////////////////////////////////////////////////
RoboCompLidar3D::TData SpecificWorker::read_lidar(const string &lidar_name)
{
    try
    {
        auto ldata =  lidar3d_proxy->getLidarData(lidar_name, 0, 2*M_PI, 1);
        // filter points according to height and distance
        RoboCompLidar3D::TPoints  p_filter;
        std::ranges::copy_if(ldata.points, std::back_inserter(p_filter),
                             [](auto  &a){ return a.z < 500 and a.distance2d > 200;});
        return RoboCompLidar3D::TData{.points = p_filter, .period = ldata.period, .timestamp = ldata.timestamp};
    }
    catch(const Ice::Exception &e){std::cout << e << std::endl;}
    return {};
}

//////////////////////////////////////////////////////////////////
/// STATE  MACHINE
//////////////////////////////////////////////////////////////////
SpecificWorker::RobotSpeed SpecificWorker::state_machine(const RoboCompVisualElementsPub::TObject &person, const RoboCompLidar3D::TPoints &points)
{
    // call the appropriate state function
    RetVal res;
    if(pushButton_stop->isChecked())
        state = STATE::STOP;

    switch(state)
    {
        case STATE::TRACK:
            res = track(person, points);
            label_state->setText("TRACK");
            break;
        case STATE::WAIT:
            res = wait(person);
            label_state->setText("WAIT");
            break;
        case STATE::STOP:
            res = stop();
            label_state->setText("STOP");
            break;
    }
    auto &[st, speed, rot] = res;
    state = st;
    return {speed, rot};
}
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
SpecificWorker::RetVal SpecificWorker::track(const RoboCompVisualElementsPub::TObject &person, auto &points)
{
    //qDebug() << __FUNCTION__ ;

    auto gaussian_break = [](float x) -> float
    {
        // gaussian function where x is the rotation speed -1 to 1. Returns 1 for x = 0 and 0.4 for x = 0.5
        const double xset = 0.5;
        const double yset = 0.4;
        const double s = -xset*xset/log(yset);
        return (float)exp(-x*x/s);
    };

    auto distance = std::hypot(std::stof(person.attributes.at("x_pos")), std::stof(person.attributes.at("y_pos")));
    lcdNumber_dist_to_person->display(distance);

    // check if the distance to the person is lower than a threshold
    if(distance < params.PERSON_MIN_DIST)
        return RetVal(STATE::WAIT, 0.f, 0.f);

    // TRACK
    float rot_angle = 1.2 * atan2(std::stof(person.attributes.at("x_pos")), std::stof(person.attributes.at("y_pos")));
    float adv = std::clamp(params.MAX_ADV_SPEED*gaussian_break(rot_angle), 0.f, params.MAX_ADV_SPEED);
    float rot = std::clamp(rot_angle, -params.MAX_ROT_SPEED, params.MAX_ROT_SPEED);
    return RetVal(STATE::TRACK, adv, rot);
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
SpecificWorker::RetVal SpecificWorker::wait(const RoboCompVisualElementsPub::TObject &person)
{
    qDebug() << __FUNCTION__ ;

    // check if the person is further than a threshold
    if(std::hypot(std::stof(person.attributes.at("x_pos")), std::stof(person.attributes.at("y_pos"))) > params.PERSON_MIN_DIST + 100)
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal(STATE::WAIT, 0.f, 0.f);

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
SpecificWorker::RetVal SpecificWorker::stop()
{
    qDebug() << __FUNCTION__ ;
    // Check the status of the pushButton_stop
    if(not pushButton_stop->isChecked())
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal (STATE::STOP, 0.f, 0.f);
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

    auto color = QColor(Qt::darkGreen);
    auto brush = QBrush(QColor(Qt::darkGreen));
    for(const auto &p : filtered_points)
    {
        auto item = scene->addRect(-50, -50, 100, 100, color, brush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

    // compute and draw minimum distance point
    //    auto min_point = std::ranges::min_element(filtered_points, [](auto &a, auto &b)
    //    { return a.distance2d < b.distance2d; });
    //    QColor dcolor;
    //    if(min_point->distance2d < params.STOP_THRESHOLD)
    //        dcolor = QColor(Qt::red);
    //    else
    //        dcolor = QColor(Qt::darkGreen);
    //    auto ditem = scene->addRect(-50, -50, 100, 100, dcolor, QBrush(dcolor));
    //    ditem->setPos(min_point->x, min_point->y);
    //    items.push_back(ditem);

    // draw a line from the robot to the minimum distance point
    //auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_point->x, min_point->y)), QPen(Qt::darkGreen, 10));
    //items.push_back(item_line);

    // update UI
    //lcdNumber_minangle->display(atan2(min_point->x,min_point->y));
    //lcdNumber_mindist->display(min_point->distance2d);
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
void SpecificWorker::draw_person(RoboCompVisualElementsPub::TObject &person, QGraphicsScene *scene) const
{
    static std::vector<QGraphicsItem*> items;
    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    // draw a circle around the person
    float radius = 300;
    auto person_draw = scene->addEllipse(-radius, -radius, radius*2, radius*2, QPen(Qt::magenta, 30));
    person_draw->setPos(std::stof(person.attributes["x_pos"]), std::stof(person.attributes["y_pos"]));
    items.push_back(person_draw);

    // draw a radius inside the ellipse to indicate the person's orientation
    auto x = std::stof(person.attributes.at("x_pos"));
    auto y = std::stof(person.attributes.at("y_pos"));
    auto angle = std::stof(person.attributes.at("orientation")) + M_PI;
    auto item_radius = scene->addLine(QLineF(QPointF(x, y),
                                                                    QPointF( x - radius * sin(angle),y + radius * cos(angle))),
                                                         QPen(Qt::magenta, 20));
    items.push_back(item_radius);

    // draw a line from the robot to the person circle but ending on the circunference. The end point is the exterior of the circle
    // I need a line from the robot to the person x,y but it has to be 300mm shorter
    auto len = std::hypot(x, y);
    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f),
                                                                   QPointF((len -radius) *x/len, (len - radius)*y/len )),
                                                           QPen(Qt::magenta, 20));
    items.push_back(item_line);
}
std::expected<int, string> SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle)
{
    // search for the point in points whose phi value is closest to angle
    auto res = std::ranges::find_if(points, [angle](auto &a){ return a.phi > angle;});
    if(res != std::end(points))
        return std::distance(std::begin(points), res);
    else
        return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}

//////////////////////////////////////////////////////////////////
/// SUBSCRIPTIONS (called in a different thread)
//////////////////////////////////////////////////////////////////
//SUBSCRIPTION to setVisualObjects method from VisualElementsPub interface. This is called in a different thread.
void SpecificWorker::VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data)
{
    // std::cout << "VisualElements_setVisualObjects" << std::endl;
    //    for(auto object : data.objects)
    //        std::cout << "Object type: " << object.id << std::endl;
    //    qDebug() << "Size: " << data.objects.size();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    buffer.put<0>(std::move(data), timestamp); // inserts the laser data value to the queue 0.
}

//////////////////////////////////////////////////////////////////
/// AUXILIARY FUNCTIONS
//////////////////////////////////////////////////////////////////
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

/**************************************/
// From the RoboCompVisualElements you can call this methods:
// this->visualelements_proxy->getVisualObjects(...)
// this->visualelements_proxy->setVisualObjects(...)

/**************************************/
// From the RoboCompVisualElements you can use this types:
// RoboCompVisualElements::TRoi
// RoboCompVisualElements::TObject
// RoboCompVisualElements::TObjects

// Instantiate the random number generator and distribution
//    static std::mt19937 gen(rd());
//    static std::uniform_int_distribution<int> dist(0, 1);
//    static bool first_time = true;
//    static int sign = 1;