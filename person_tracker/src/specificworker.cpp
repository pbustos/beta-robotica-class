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
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    std::locale::global(std::locale("C"));
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
        this->resize(800, 700);

        // Initialize the plot
        plot = new QCustomPlot(frame_dist);
        plot->resize(frame_dist->size());
        plot->addGraph();
        plot->graph(0)->setPen(QPen(QColor(0, 0, 255)));
        plot->xAxis->setLabel("Time");
        plot->yAxis->setLabel("Dist. to person");
        plot->xAxis->setRange(0, 50);
        plot->yAxis->setRange(-1000, 1000);
        plot->replot();
        plot->show();

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
     //read bpearl (lower) lidar and draw
    auto ldata_bpearl = read_lidar_bpearl();
    if(ldata_bpearl.empty()) { qWarning() << __FUNCTION__ << "Empty bpearl lidar data"; return; };
    //draw_lidar(ldata.points, &viewer->scene);

    auto ldata_helios = read_lidar_helios();
    if(ldata_helios.empty()) { qWarning() << __FUNCTION__ << "Empty helios lidar data"; return; };
    //draw_lidar(ldata.points, &viewer->scene);

    /// wall lines
    auto lines = detect_wall_lines(ldata_helios, &viewer->scene);

    /// remove wall points
    auto bpearl = remove_wall_points(lines, ldata_bpearl);
    draw_lidar(bpearl, &viewer->scene);

    /// find obstacles
    auto obstacles = rc::dbscan(bpearl, params.ROBOT_WIDTH, 2);

    /// enlarge obstacles
    obstacles = enlarge_polygons(obstacles, params.ROBOT_WIDTH);

    /// check if there is new YOLO data in buffer
    std::expected<RoboCompVisualElementsPub::TObject, std::string> tp_person = std::unexpected("No person found");
    auto [data_] = buffer.read_first();
    if(data_.has_value())
        tp_person = find_person_in_data(data_.value().objects);

    /// remove person from obstacles
    if(tp_person)
        obstacles = find_person_polygon_and_remove(tp_person.value(), obstacles);

    draw_obstacles(obstacles, &viewer->scene, Qt::darkMagenta);

    /// get walls as polygons
    std::vector<QPolygonF> wall_obs = get_walls_as_polygons(lines, params.ROBOT_WIDTH/4);
    obstacles.insert(obstacles.end(), wall_obs.begin(), wall_obs.end());

    /// compute an obstacle free path
    std::vector<Eigen::Vector2f> path;
    if(tp_person)
    {
        Eigen::Vector2f goal{std::stof(tp_person.value().attributes.at("x_pos")), std::stof(tp_person.value().attributes.at("y_pos"))};
        path = rc::VisibilityGraph().generate_path(Eigen::Vector2f::Zero(),
                                                        goal,
                                                        obstacles,
                                                       params.ROBOT_WIDTH / 2,
                                                       nullptr);
        draw_path_to_person(path, &viewer->scene);
    }

    // call state machine to track person
    const auto &[adv, rot] = state_machine(path);

    // plot on UI
    if(tp_person)
    {
        float d = std::hypot(std::stof(tp_person.value().attributes.at("x_pos")),
                                 std::stof(tp_person.value().attributes.at("y_pos")));
        plot_distance(running_average(d) - params.PERSON_MIN_DIST);
        lcdNumber_dist_to_person->display(d);
        lcdNumber_angle_to_person->display(atan2(std::stof(tp_person.value().attributes.at("x_pos")),
                                                 std::stof(tp_person.value().attributes.at("y_pos"))));
    }
    lcdNumber_adv->display(adv);
    lcdNumber_rot ->display(rot);

    // move the robot
    try{ omnirobot_proxy->setSpeedBase(0.f, adv, rot); }
    catch(const Ice::Exception &e){std::cout << e << std::endl;}

}

//////////////////////////////////////////////////////////////////
/// YOUR CODE HERE
//////////////////////////////////////////////////////////////////
// Read the BPEARL lidar data and filter the points
std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_bpearl()
{
    try
    {
        auto ldata =  lidar3d1_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
        // filter points according to height and distance
        std::vector<Eigen::Vector2f>  p_filter;
        for(const auto &a: ldata.points)
        {
            if(a.z < 500 and a.distance2d > 200)
                p_filter.emplace_back(a.x, a.y);
        }
        return p_filter;
    }
    catch(const Ice::Exception &e){std::cout << e << std::endl;}
    return {};
}
std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_helios()
{
    try
    {

        auto ldata =  lidar3d_proxy->getLidarData("helios", 0, 2*M_PI, 2);
        // filter points according to height and distance
        std::vector<Eigen::Vector2f> p_filter;
        for(const auto &a: ldata.points)
        {
            if(a.z > 1300 and a.distance2d > 200)
                p_filter.emplace_back(a.x, a.y);
        }



        return p_filter;
    }
    catch(const Ice::Exception &e){std::cout << e << std::endl;}
    return {};
}
std::vector<QLineF> SpecificWorker::detect_wall_lines(const vector<Eigen::Vector2f> &points, QGraphicsScene *scene)
{
    std::vector<QLineF> lines;
    const auto &[ls, _, __, ___] = room_detector.compute_features(points, &viewer->scene);
    for(const auto &l: ls)
        lines.emplace_back(l.second);
    return lines;
}
std::vector<Eigen::Vector2f> SpecificWorker::remove_wall_points(const std::vector<QLineF> &lines, const auto &bpearl)
{
    std::vector<Eigen::Vector2f> points_inside;
    for(const auto &p: bpearl)
    {
        bool outside = true;
        for(const auto &line: lines)
        {
            Eigen::Vector2f p1{line.x1(), line.y1()};
            Eigen::Vector2f p2{line.x2(), line.y2()};
            auto pline = Eigen::ParametrizedLine<float, 2>::Through(p1, p2);
            if (pline.distance(p) < params.ROBOT_WIDTH)
            {
                outside = false;
                break;
            }
        }
        if(outside) points_inside.emplace_back(p);
    }
    return points_inside;
}
std::vector<QPolygonF> SpecificWorker::get_walls_as_polygons(const std::vector<QLineF> &lines, float robot_width)
{
    std::vector<QPolygonF> obstacles;
    for(const auto &l: lines)
    {
        // create line
        QLineF line = l;
        // Calculate the direction vector of the line
        QPointF direction = line.p2() - line.p1();
        // Calculate the normal vector of the line
        QPointF normal = QPointF(-direction.y(), direction.x());
        // Normalize the normal vector
        normal /= sqrt(normal.x()*normal.x() + normal.y()*normal.y());
        // Create the polygon
        QPolygonF poly;
        poly << line.p1() + normal * robot_width/2 << line.p2() + normal * robot_width/2
             << line.p2() - normal * robot_width/2 << line.p1() - normal * robot_width/2;
        obstacles.push_back(poly);
    }
    return obstacles;
}
std::vector<QPolygonF> SpecificWorker::enlarge_polygons(const std::vector<QPolygonF> &polygons, float amount)
{
    std::vector<QPolygonF> enlargedPolygons;
    for(const auto &poly: polygons)
    {
        QPolygonF exp_poly; // expanded polygon
        if (poly.size() < 3) continue;  // skip polygons with less than 3 points
        QPolygonF copy_poly(poly); // copy of the polygon to insert the first point at the end
        copy_poly << poly[0] << poly[1];
        for (const auto &p: iter::sliding_window(copy_poly, 3))
        {
            const auto p1 = Eigen::Vector2f{p[0].x(), p[0].y()};
            const auto p2 = Eigen::Vector2f{p[1].x(), p[1].y()};
            const auto p3 = Eigen::Vector2f{p[2].x(), p[2].y()};
            const auto bisectrix = ((p1 - p2).normalized() + (p3 - p2).normalized()).normalized();
            const Eigen::Vector2f np2 = p2 - amount * bisectrix;
            exp_poly << QPointF{np2.x(), np2.y()};
        }
        enlargedPolygons.emplace_back(exp_poly);
    }
    return enlargedPolygons;
}
std::expected<RoboCompVisualElementsPub::TObject, std::string> SpecificWorker::find_person_in_data(const std::vector<RoboCompVisualElementsPub::TObject> &objects)
{
    if(objects.empty())
        return std::unexpected("Empty objects in method <find_person_in_data>");
    if(auto p_ = std::ranges::find_if(objects, [](auto &a)
            { return a.id == 0 and std::stof(a.attributes.at("score")) > 0.6;}); p_ == std::end(objects))
        return std::unexpected("No person found in method <find_person_in_data>");
    else
    {
        draw_person(const_cast<RoboCompVisualElementsPub::TObject &>(*p_), &viewer->scene);
        return *p_;
    }
}
std::vector<QPolygonF> SpecificWorker::find_person_polygon_and_remove(const RoboCompVisualElementsPub::TObject &person, const std::vector<QPolygonF> &obstacles)
{
    std::vector<QPolygonF> new_obs;
    QPointF pp = QPointF(std::stof(person.attributes.at("x_pos")), std::stof(person.attributes.at("y_pos")));
    // compute 8 point around pp in circular configuration
    std::vector<QPointF> ppoly;
    for (auto i: iter::range(0.0, 2 * M_PI, M_PI / 2))
        ppoly.push_back(pp + QPointF(params.PERSON_RADIUS * cos(i), params.PERSON_RADIUS * sin(i)));
    // check if any polygon contains the person and remove it
    for(const auto &poly: obstacles)
    {
        bool contains = false;
        for(const auto &p: ppoly)
            if(poly.containsPoint(p, Qt::OddEvenFill))
            {
                contains = true;
                break;
            }
        if(not contains)
            new_obs.push_back(poly);
    }
    return new_obs;
}
//////////////////////////////////////////////////////////////////
/// STATE  MACHINE
//////////////////////////////////////////////////////////////////
// State machine to track a person
SpecificWorker::RobotSpeed SpecificWorker::state_machine(const Path &path)
{
    // call the appropriate state function
    RetVal res;
    if(pushButton_stop->isChecked())    // stop if buttom is pressed
        state = STATE::STOP;

    switch(state)
    {
        case STATE::TRACK:
            res = track(path);
            label_state->setText("TRACK");
            break;
        case STATE::WAIT:
            res = wait(path);
            label_state->setText("WAIT");
            break;
        case STATE::SEARCH:
            res = search(path);
            label_state->setText("SEARCH");
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
 // State function to track a person
SpecificWorker::RetVal SpecificWorker::track(const Path &path)
{
    static float ant_angle_error = 0.0;
    //qDebug() << __FUNCTION__;
    // variance of the gaussian function is set by the user giving a point xset where the function must be yset, and solving for s
    auto gaussian_break = [](float x) -> float
    {
        // gaussian function where x is the rotation speed -1 to 1. Returns 1 for x = 0 and 0.4 for x = 0.5
        const double xset = 0.5;
        const double yset = 0.73;
        //compute the variance s so the function is yset for x = xset
        float s = -xset*xset/(log(yset));
        return (float)exp(-x*x/s);
    };

    if(path.empty())
    { /* qWarning() << __FUNCTION__ << "No person found"; */ return RetVal(STATE::SEARCH, 0.f, 0.f); }

    auto distance = distance_to_person_through_path(path);
    lcdNumber_dist_to_person->display(distance);

    // check if the distance to the person is lower than a threshold
    if(distance < params.PERSON_MIN_DIST)
    { qWarning() << __FUNCTION__ << "Distance to person lower than threshold"; return RetVal(STATE::WAIT, 0.f, 0.f);}

    // angle error is the angle between the robot and the person. It has to be brought to zero
    float angle_error = atan2(path[1].x(), path[1].y());
    float rot_speed = params.k1 * angle_error + params.k2 * (angle_error-ant_angle_error);
    ant_angle_error = angle_error;
    // rot_brake is a value between 0 and 1 that decreases the speed when the robot is not facing the person
    float rot_brake = gaussian_break(rot_speed);
    // acc_distance is the distance given to the robot to reach again the maximum speed
    float acc_distance = params.acc_distance_factor * params.ROBOT_WIDTH;
    // advance brake is a value between 0 and 1 that decreases the speed when the robot is too close to the person
    float adv_brake = std::clamp(distance * 1.f/acc_distance - (params.PERSON_MIN_DIST / acc_distance), 0.f, 1.f);
    return RetVal(STATE::TRACK, params.MAX_ADV_SPEED * rot_brake * adv_brake, rot_speed);
}
//
SpecificWorker::RetVal SpecificWorker::wait(const Path &path)
{
    if(path.empty())
    {  qWarning() << __FUNCTION__ << "No person found"; return RetVal(STATE::TRACK, 0.f, 0.f); }

    // check if the person is further than a threshold
    auto distance = std::accumulate(std::begin(path), std::end(path), 0.f, [](auto acc, auto &p)
    {   static Eigen::Vector2f ant_p = Eigen::Vector2f::Zero();
        return acc + (p-ant_p).norm();
        ant_p = p;
    });
    if(distance > params.PERSON_MIN_DIST + 100)
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal(STATE::WAIT, 0.f, 0.f);

}
// Search when no person is found
SpecificWorker::RetVal SpecificWorker::search(const Path &path)
{
    if(not path.empty())
    {  qWarning() << __FUNCTION__ << "Person found, moving to TRACK"; return RetVal(STATE::TRACK, 0.f, 0.f); }

    return RetVal(STATE::SEARCH, 0.f, params.SEARCH_ROT_SPEED);
}
// Stops the robot
SpecificWorker::RetVal SpecificWorker::stop()
{
    //qDebug() << __FUNCTION__ ;
    // Check the status of the pushButton_stop
    if(not pushButton_stop->isChecked())
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal (STATE::STOP, 0.f, 0.f);
}
float SpecificWorker::distance_to_person_through_path(const Path &path)
{
    return std::accumulate(std::begin(path), std::end(path), 0.f, [](auto acc, auto &p)
                {   static Eigen::Vector2f ant_p = Eigen::Vector2f::Zero();
                    return acc + (p-ant_p).norm();
                    ant_p = p;
                });
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
        item->setPos(p.x(), p.y());
        items.push_back(item);
    }
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
void SpecificWorker::draw_path_to_person(const auto &points, QGraphicsScene *scene)
{
    if(points.empty())
        return;

    // remove all items drawn in the previous iteration
    static std::vector<QGraphicsItem*> items;
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    // draw the path as a series of lines with dots in between
    for (auto i : iter::range(0UL, points.size() - 1))
    {
        auto line = scene->addLine(QLineF(QPointF(points[i].x(), points[i].y()), QPointF(points[i+1].x(), points[i+1].y())),
                                   QPen(Qt::blue, 40));
        items.push_back(line);
        auto dot = scene->addEllipse(-30, -30, 60, 60, QPen(Qt::darkBlue, 40));
        dot->setPos(points[i].x(), points[i].y());
       items.push_back(dot);
    }
}
void SpecificWorker::plot_distance(double distance)
{
    // add value to plot
    static int key = 0;
    plot->graph(0)->addData(key++, distance);
    // Remove data points if there are more than X
    if (plot->graph(0)->dataCount() > params.MAX_DIST_POINTS_TO_SHOW)
        plot->graph(0)->data()->removeBefore(key - params.MAX_DIST_POINTS_TO_SHOW);
    // plot
    plot->rescaleAxes();  plot->replot();
}
float SpecificWorker::running_average(float dist)
{
    static float avg = 0;
    static int count = 0;
    avg = (avg * count + dist) / (count + 1);
    count++;
    return avg;
}
void SpecificWorker::draw_obstacles(const vector<QPolygonF> &list_poly, QGraphicsScene *scene, const QColor &color) const
{
    static std::vector<QGraphicsItem*> items;
    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    for(const auto &poly : list_poly)
    {
        auto item = scene->addPolygon(poly, QPen(color, 50));
        items.push_back(item);
    }
}
//////////////////////////////////////////////////////////////////
/// SUBSCRIPTIONS (runs in a different thread)
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