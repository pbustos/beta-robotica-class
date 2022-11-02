/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/filter.hpp>

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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

	return true;
}
void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        // graphics
        QRectF viewer_dimensions = QRectF(-2500, -2500, 5000, 5000);  //robot view
        viewer = new AbstractGraphicViewer(this->beta_frame, viewer_dimensions);
        this->resize(900,650);
        viewer->add_robot(robot.width, robot.length);

        // sets servo to zero position
        try
        { jointmotorsimple_proxy->setPosition("camera_pan_joint", RoboCompJointMotorSimple::MotorGoalPosition(0, 1));}
        catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; return;}

        // get object names
        try
        { yolo_object_names = yoloobjects_proxy->getYoloObjectNames(); }
        catch(const Ice::Exception &e) {std::cout << e.what() << " Error connecting with YoloObjects interface to retrieve names" << std::endl;}
        {
            COLORS.resize(80, 3);
            COLORS << 0.000, 0.447, 0.741,
                    0.850, 0.325, 0.098,
                    0.929, 0.694, 0.125,
                    0.494, 0.184, 0.556,
                    0.466, 0.674, 0.188,
                    0.301, 0.745, 0.933,
                    0.635, 0.078, 0.184,
                    0.300, 0.300, 0.300,
                    0.600, 0.600, 0.600,
                    1.000, 0.000, 0.000,
                    1.000, 0.500, 0.000,
                    0.749, 0.749, 0.000,
                    0.000, 1.000, 0.000,
                    0.000, 0.000, 1.000,
                    0.667, 0.000, 1.000,
                    0.333, 0.333, 0.000,
                    0.333, 0.667, 0.000,
                    0.333, 1.000, 0.000,
                    0.667, 0.333, 0.000,
                    0.667, 0.667, 0.000,
                    0.667, 1.000, 0.000,
                    1.000, 0.333, 0.000,
                    1.000, 0.667, 0.000,
                    1.000, 1.000, 0.000,
                    0.000, 0.333, 0.500,
                    0.000, 0.667, 0.500,
                    0.000, 1.000, 0.500,
                    0.333, 0.000, 0.500,
                    0.333, 0.333, 0.500,
                    0.333, 0.667, 0.500,
                    0.333, 1.000, 0.500,
                    0.667, 0.000, 0.500,
                    0.667, 0.333, 0.500,
                    0.667, 0.667, 0.500,
                    0.667, 1.000, 0.500,
                    1.000, 0.000, 0.500,
                    1.000, 0.333, 0.500,
                    1.000, 0.667, 0.500,
                    1.000, 1.000, 0.500,
                    0.000, 0.333, 1.000,
                    0.000, 0.667, 1.000,
                    0.000, 1.000, 1.000,
                    0.333, 0.000, 1.000,
                    0.333, 0.333, 1.000,
                    0.333, 0.667, 1.000,
                    0.333, 1.000, 1.000,
                    0.667, 0.000, 1.000,
                    0.667, 0.333, 1.000,
                    0.667, 0.667, 1.000,
                    0.667, 1.000, 1.000,
                    1.000, 0.000, 1.000,
                    1.000, 0.333, 1.000,
                    1.000, 0.667, 1.000,
                    0.333, 0.000, 0.000,
                    0.500, 0.000, 0.000,
                    0.667, 0.000, 0.000,
                    0.833, 0.000, 0.000,
                    1.000, 0.000, 0.000,
                    0.000, 0.167, 0.000,
                    0.000, 0.333, 0.000,
                    0.000, 0.500, 0.000,
                    0.000, 0.667, 0.000,
                    0.000, 0.833, 0.000,
                    0.000, 1.000, 0.000,
                    0.000, 0.000, 0.167,
                    0.000, 0.000, 0.333,
                    0.000, 0.000, 0.500,
                    0.000, 0.000, 0.667,
                    0.000, 0.000, 0.833,
                    0.000, 0.000, 1.000,
                    0.000, 0.000, 0.000,
                    0.143, 0.143, 0.143,
                    0.286, 0.286, 0.286,
                    0.429, 0.429, 0.429,
                    0.571, 0.571, 0.571,
                    0.714, 0.714, 0.714,
                    0.857, 0.857, 0.857,
                    0.000, 0.447, 0.741,
                    0.314, 0.717, 0.741,
                    0.50, 0.5, 0;
        }
        COLORS *= 255;

        Period = 50;
        timer.start(Period);
	}
}
void SpecificWorker::compute()
{
    cv::Mat omni_rgb_frame;
    cv::Mat omni_depth_frame;

    /// omni
    omni_depth_frame = read_depth_coppelia();
    if(omni_depth_frame.empty()) { qWarning() << "omni_depth_frame empty"; return;}

    /// top camera
    auto top_rgb_frame = read_rgb("/Shadow/camera_top");
    if(top_rgb_frame.empty()) { qWarning() << "rgb_top_frame empty"; return;}
    auto [top_depth_frame, focalx, focaly] = read_depth_top("/Shadow/camera_top");
    if(top_depth_frame.empty()) { qWarning() << "depth_top_frame empty"; return;}

    /// read base current speed
    RoboCompGenericBase::TBaseState bState;
    try{ omnirobot_proxy->getBaseState(bState); robot.current_adv_speed = bState.advVz; robot.current_rot_speed = bState.rotV;}
    catch (const Ice::Exception &e)
    { std::cout << e.what() << " Error reading omnibase_proxy::getBaseState" << std::endl;}

    /// compute level_lines  (aka read laser)
    auto omni_lines = get_multi_level_3d_points_omni(omni_depth_frame);
    draw_floor_line(omni_lines, 1);
    auto current_line = omni_lines[1];  // second line of the list of laser lines at different heights

    /// YOLO
    RoboCompYoloObjects::TObjects objects = yolo_detect_objects(top_rgb_frame);

    /// draw top image
    cv::imshow("top", top_rgb_frame); cv::waitKey(1);

    /// draw yolo_objects on 2D view
    draw_objects_on_2dview(objects, RoboCompYoloObjects::TBox());

    // state machine to activate basic behaviours
    // states: IDLE, SEARCHING, APPROACHING
    // return a  target_force vector
    //target_force = state_machine(objects, current_line);

    /// eye tracking: tracks  current selected object or  selects a new one
    eye_track(objects);
    draw_top_camera_optic_ray();

    /// Dynamic Window algorithm to compute obstacle-free path to target
    if(robot.has_target)
    {
        auto [adv, rot, side] = dwa.update(robot.target_coordinates, current_line, robot.current_adv_speed, robot.current_rot_speed, viewer);
        //qInfo() << __FUNCTION__ << adv <<  side << rot;
    }
}

//////////////////// BASIC REPULSION LOOP. EVERYTHING GOES ON TOP OF THIS/////////////////////////////////////////////////
// perception
cv::Mat SpecificWorker::read_rgb(const std::string &camera_name)
{
    RoboCompCameraRGBDSimple::TImage omni_rgb;
    cv::Mat omni_rgb_frame;
    try
    {
        omni_rgb = camerargbdsimple_proxy->getImage(camera_name);
        if (not omni_rgb.image.empty())
            omni_rgb_frame= cv::Mat(cv::Size(omni_rgb.width, omni_rgb.height), CV_8UC3, &omni_rgb.image[0], cv::Mat::AUTO_STEP);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << " Error reading camerargbdsimple_proxy::getImage" << std::endl;}
    return omni_rgb_frame.clone();
}
std::tuple<cv::Mat, float, float> SpecificWorker::read_depth_top(const std::string &camera_name)
{
    RoboCompCameraRGBDSimple::TDepth top_depth;
    cv::Mat top_depth_frame;
    try
    {
        top_depth = camerargbdsimple_proxy->getDepth(camera_name);
        if (not top_depth.depth.empty())
            top_depth_frame = cv::Mat(cv::Size(top_depth.width, top_depth.height), CV_32FC1, &top_depth.depth[0], cv::Mat::AUTO_STEP);

    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << " Error reading camerargbdsimple_proxy::getDepth" << std::endl;}
    return std::make_tuple(top_depth_frame.clone(), top_depth.focalx, top_depth.focaly);
}
cv::Mat SpecificWorker::read_depth_coppelia()
{
    RoboCompCameraRGBDSimple::TImage omni_depth;       //omni_camera depth comes as RGB
    cv::Mat omni_depth_float;
    try
    {
        omni_depth = camerargbdsimple_proxy->getImage("/Shadow/omnicamera/sensorDepth");
        if(not omni_depth.image.empty())
        {
            cv::Mat omni_depth_frame(cv::Size(omni_depth.width, omni_depth.height), CV_8UC3, &omni_depth.image[0], cv::Mat::AUTO_STEP);
            cv::cvtColor(omni_depth_frame, omni_depth_frame, cv::COLOR_RGB2GRAY);
            omni_depth_frame.convertTo(omni_depth_float, CV_32FC1);
        }
    }
    catch(const Ice::Exception &e)
    { std::cout << e.what() << " Error reading camerargbdsimple_proxy::getImage for depth" << std::endl;}
    return omni_depth_float.clone();
}
RoboCompYoloObjects::TObjects SpecificWorker::yolo_detect_objects(cv::Mat rgb)
{
    RoboCompYoloObjects::TObjects objects;
    RoboCompYoloObjects::TData yolo_objects;
    try
    { yolo_objects = yoloobjects_proxy->getYoloObjects(); }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; return objects;}

    // remove unwanted types
    yolo_objects.objects.erase(std::remove_if(yolo_objects.objects.begin(), yolo_objects.objects.end(), [names = yolo_object_names](auto p)
    { return names[p.type] == "toilet"
             or names[p.type] == "cup"
             or names[p.type] == "trafficlight"
             or names[p.type] == "bench"; }), yolo_objects.objects.end());

    for(auto &&o: yolo_objects.objects | iter::filter([th = consts.yolo_threshold](auto &o){return o.score > th;}))
    {
        objects.push_back(o);
        auto tl = round(0.002 * (rgb.cols + rgb.rows) / 2) + 1; // line / fontthickness
        const auto &c = COLORS.row(o.type);
        cv::Scalar color(c.x(), c.y(), c.z()); // box color
        cv::Point c1(o.left, o.top);
        cv::Point c2(o.right, o.bot);
        cv::rectangle(rgb, c1, c2, color, tl, cv::LINE_AA);
        int tf = (int) std::max(tl - 1, 1.0);  // font thickness
        int baseline = 0;
        std::string label = yolo_object_names.at(o.type) + " " + std::to_string((int) (o.score * 100)) + "%";
        auto t_size = cv::getTextSize(label, 0, tl / 3.f, tf, &baseline);
        c2 = {c1.x + t_size.width, c1.y - t_size.height - 3};
        cv::rectangle(rgb, c1, c2, color, -1, cv::LINE_AA);  // filled
        cv::putText(rgb, label, cv::Size(c1.x, c1.y - 2), 0, tl / 3, cv::Scalar(225, 255, 255), tf, cv::LINE_AA);
    }
    return objects;
}
std::vector<std::vector<Eigen::Vector2f>> SpecificWorker::get_multi_level_3d_points_omni(const cv::Mat &depth_frame)
{
    std::vector<std::vector<Eigen::Vector2f>> points(int((consts.depth_lines_max_height-consts.depth_lines_min_height)/consts.depth_lines_step));  //height steps
    for(auto &p: points)
        p.resize(360, Eigen::Vector2f(consts.max_camera_depth_range, consts.max_camera_depth_range));   // angular resolution

    int semi_height = depth_frame.rows/2;
    float hor_ang, dist, x, y, z, proy;
    float ang_slope = 2*M_PI/depth_frame.cols;
    const float ang_bin = 2.0*M_PI/consts.num_angular_bins;

    for(int u=0; u<depth_frame.rows; u++)
        for(int v=0; v<depth_frame.cols; v++)
        {
            hor_ang = ang_slope * v - M_PI; // cols to radians
            if(consts.IS_COPPELIA)
                dist = depth_frame.ptr<float>(u)[v] * consts.coppelia_depth_scaling_factor;  // pixel to dist scaling factor  -> to mm
            else
                dist = depth_frame.ptr<float>(u)[v] * consts.dreamvu_depth_scaling_factor;  // pixel to dist scaling factor  -> to mm
            if(dist > consts.max_camera_depth_range) continue;
            if(dist < consts.min_camera_depth_range) continue;
            x = -dist * sin(hor_ang);
            y = dist * cos(hor_ang);
            float fov;
            if(consts.IS_COPPELIA)
                fov = 128;
            else
                fov = (depth_frame.rows / 2.f) / tan(qDegreesToRadians(111.f / 2.f));   // 111º vertical angle of dreamvu

            proy = dist * cos(atan2((semi_height - u), fov));
            z = (semi_height - u) / fov * proy;

            // add Z and Y axis displacement
            z += consts.omni_camera_height;
            y += consts.omni_camera_y_offset;

            if(z < 0) continue; // filter out floor

            // add only to its bin if less than current value
            for(auto &&[level, step] : iter::range(consts.depth_lines_min_height, consts.depth_lines_max_height, consts.depth_lines_step) | iter::enumerate)
                if(z > step and z < step+consts.depth_lines_step )
                {
                    int ang_index = floor((M_PI + atan2(x, y)) / ang_bin);
                    Eigen::Vector2f new_point(x, y);
                    if(new_point.norm() <  points[level][ang_index].norm() and new_point.norm() > consts.min_dist_from_robot_center)
                        points[level][ang_index] = new_point;
                }
        };
    return points;
}

// action
void SpecificWorker::eye_track(const RoboCompYoloObjects::TObjects &objects)
{
    static float error_ant = 0.f;
    if(robot.has_target)  // track the target. The robot's base will turn to cancel the camera's angle
    {
        // match current target with new objects
        for(const auto &o: objects)
            if(o.type == robot.target.type)
            {
                robot.target = o;
                qInfo() << __FUNCTION__ << "Match to previous target";
                // transform target x,y coordinates in camera CS to robot CS
                Eigen::Transform<float, 3, Eigen::Affine> t = Eigen::Translation3f(Eigen::Vector3f{0.f, 0.f, consts.top_camera_height}) *
                                                              Eigen::AngleAxisf(robot.camera_tilt_angle, Eigen::Vector3f::UnitX()) *
                                                              Eigen::AngleAxisf(robot.camera_pan_angle, Eigen::Vector3f::UnitZ());
                robot.target_coordinates = (t * Eigen::Vector3f{robot.target.x, robot.target.y, robot.target.z}).head(2);
            }

        float hor_angle = atan2(robot.target.x, robot.target.y);  // angle wrt camera origin
        qInfo() << __FUNCTION__ << " eye tracking to " << QString::fromStdString(yolo_object_names[robot.target.type]) << "Angle:" << hor_angle;
        auto angle = jointmotorsimple_proxy->getMotorState("camera_pan_joint").pos;
        if (std::isnan(angle) or std::isnan(hor_angle))
        { qWarning() << "NAN value in servo position";  return; }
        robot.camera_pan_angle = angle;
        if( fabs(hor_angle) > consts.max_hor_angle_error)  // saccadic: fast movement to the target when it is far from the center of the image
        {
            qInfo() << __FUNCTION__ << "saccade" << hor_angle;
            try
            {
                float error = 0.5 * (current_servo_angle - hor_angle);
                float new_angle = std::clamp(error, -1.f, 1.f);  // dumping
                jointmotorsimple_proxy->setPosition("camera_pan_joint", RoboCompJointMotorSimple::MotorGoalPosition(new_angle, 1));
            }
            catch (const Ice::Exception &e)
            {  std::cout << e.what() << " Error connecting to MotorGoalPosition" << std::endl; return; }
        }
        else    // smooth pursuit: velocity control (no jumps) when it is close to the center of the image
        {
            try
            {
                float new_vel = -0.7 * hor_angle + 0.3 * (hor_angle - error_ant);  // proportional - derivative control
                new_vel -= 0.5*robot.current_rot_speed;  // compensate with current base rotation speed
                new_vel = std::clamp(new_vel, -1.f, 1.f);  // dumping
                jointmotorsimple_proxy->setVelocity("camera_pan_joint", RoboCompJointMotorSimple::MotorGoalVelocity(new_vel, 1));
                qInfo() << __FUNCTION__ << "error" << hor_angle << "deriv" << hor_angle-error_ant << "base" << robot.current_rot_speed;
                error_ant = hor_angle;
            }
            catch (const Ice::Exception &e)
            {  std::cout << e.what() << " Error connecting to MotorGoalPosition" << std::endl; return; }
        }
    }
    else  // choose a new target different from the current one.
        for (const auto &o: objects)
            if(robot.target.type != o.type )
            {
                robot.target = o;
                robot.has_target = true;
                qInfo() << __FUNCTION__ << "New target:" << QString::fromStdString(yolo_object_names[o.type]);
            }
        //try
        //{ jointmotorsimple_proxy->setPosition("camera_pan_joint", RoboCompJointMotorSimple::MotorGoalPosition(0.f, 1.f)); }
        //catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; return;}
}

///////////////////  State machine ////////////////////////////////////////////
Eigen::Vector2f SpecificWorker::state_machine(const RoboCompYoloObjects::TObjects &objects, const std::vector<Eigen::Vector2f> &line)
{
    Eigen::Vector2f direction;
    switch (state)
    {
        case State::IDLE:
            state = State::SEARCHING;
            break;
        case State::SEARCHING:
            direction = search_state(objects); // turns around until it finds a valid target
            break;
        case State::APPROACHING:
            direction = approach_state(objects, line);
            break;
        case State::WAITING:
            wait_state();
            break;
    };

    return direction;
}
Eigen::Vector2f SpecificWorker::search_state(const RoboCompYoloObjects::TObjects &objects)
{
    qInfo() << __FUNCTION__;
    //  get first object different from current robot.target and stick with it (be persistent)
    for (const auto &o: objects)
        if(robot.target.type != o.type )
        {
            robot.target = o;
            robot.has_target = true;
            state = State::APPROACHING;
            //qInfo() << __FUNCTION__ << "Target selected" << QString::fromStdString(yolo_object_names[o.type]);
            return robot.target_coordinates;
        }
    // keep rotating
    return Eigen::Vector2f{100.f, 0.f};
}
Eigen::Vector2f SpecificWorker::approach_state(const RoboCompYoloObjects::TObjects &objects, const std::vector<Eigen::Vector2f> &line)
{
    // keep and refresh visual target while approaching
    qInfo() << __FUNCTION__ << " target selected" << QString::fromStdString(yolo_object_names[robot.target.type]);
    if(objects.empty())
    {
        qWarning() << "Empty list of yolo objects";
        return robot.target_coordinates;
    }
    //auto res = std::ranges::min_element(objects, [&, t = robot.target](auto a, auto b){ return iou(a,t) < iou(b,t);});
    //auto index = std::distance(objects.begin(), res);   // position of minimum in array
    //if( iou(objects[index], robot.target) > consts.min_similarity_iou_threshold )
    for(const auto &o: objects)
        if(o.type == robot.target.type)
        {
            robot.target = o;
            //qInfo() << __FUNCTION__ << "Match to previous target";
            // transform target x,y coordinates in camera CS to robot CS
            Eigen::Transform<float, 3, Eigen::Affine> t = Eigen::Translation3f(Eigen::Vector3f{0.f, 0.f, consts.top_camera_height}) *
                                                          Eigen::AngleAxisf(robot.camera_tilt_angle, Eigen::Vector3f::UnitX()) *
                                                          Eigen::AngleAxisf(robot.camera_pan_angle, Eigen::Vector3f::UnitZ());
            robot.target_coordinates = (t * Eigen::Vector3f{robot.target.x, robot.target.y, robot.target.z}).head(2);
        }
    // else keep going for a while and eventually forget the target and move to SEARCHING

    //qInfo() << __FUNCTION__ << "distance to target" << robot.target_coordinates.norm();
    if(robot.target_coordinates.norm() < consts.min_dist_to_target or closest_distance_ahead(line) < consts.min_dist_to_target/2) // arrival condition
    {
        state = State::WAITING;
        robot.target_coordinates = Eigen::Vector2f{0.f, 0.f};
        robot.has_target = false;
        std::cout << __FUNCTION__  << " Arrived " << robot.target_coordinates << std::endl;
        return robot.target_coordinates;
    }
    else
    {
        //std::cout << "Keep going " << robot.target_coordinates << std::endl;
        return robot.target_coordinates;
    }
}
Eigen::Vector2f  SpecificWorker::wait_state()
{
    static rc::Timer clock;
    static const uint  TIME_INTERVAL = 2000; //ms
    static bool first_time = true;
    if (first_time)
    {
        clock.tick();
        first_time = false;
        qInfo() << __FUNCTION__ << "Waiting";
    }
    else
    {
        clock.tock();
        if(clock.duration() >= TIME_INTERVAL)
        {
            state = State::SEARCHING;
            first_time = true;
        }
    }
    return Eigen::Vector2f{0.f, 0.f};
}

///////////////////////////////////////////////////////////////////////////////////////////////7
float SpecificWorker::closest_distance_ahead(const std::vector<Eigen::Vector2f> &line)
{
    // minimum distance in central sector
    if(line.empty()) { qWarning() << __FUNCTION__ << "Empty line vector"; return 0.f;}
    size_t offset = 4*line.size()/9;
    auto res = std::min_element(line.begin()+offset, line.end()-offset, [](auto &a, auto &b){ return a.norm() < b.norm();});
    return res->norm();

}
//// IOU auxiliary function
float SpecificWorker::iou(const RoboCompYoloObjects::TBox &a, const RoboCompYoloObjects::TBox &b)
{
    // coordinates of the area of intersection.
    float ix1 = std::max(a.left, b.left);
    float iy1 = std::max(a.top, b.top);
    float ix2 = std::min(a.right, b.right);
    float iy2 = std::min(a.bot, b.bot);

    // Intersection height and width.
    float i_height = std::max(iy2 - iy1 + 1, 0.f);
    float i_width = std::max(ix2 - ix1 + 1, 0.f);

    float area_of_intersection = i_height * i_width;

    // Ground Truth dimensions.
    float a_height = a.bot - a.top + 1;
    float a_width = a.right - a.left + 1;

    // Prediction dimensions.
    float b_height = b.bot - b.top + 1;
    float b_width = b.right - b.left + 1;

    float area_of_union = a_height * a_width + b_height * b_width - area_of_intersection;
    return area_of_intersection / area_of_union;
}
float SpecificWorker::gaussian(float x)
{
    const double xset = consts.xset_gaussian;
    const double yset = consts.yset_gaussian;
    const double s = -xset*xset/log(yset);
    return exp(-x*x/s);
}
////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::draw_floor_line(const vector<vector<Eigen::Vector2f>> &lines, int i)    //one vector for each height level
{
    static std::vector<QGraphicsItem *> items;
    for(const auto &item: items)
        viewer->scene.removeItem(item);
    items.clear();

    static QStringList my_color = {"blue", "red", "orange", "magenta", "black", "yellow", "brown", "cyan"};
    std::vector<vector<Eigen::Vector2f>> copy;
    if(i == -1)
        copy.assign(lines.begin(), lines.end());
    else
        copy.assign(lines.begin()+i, lines.begin()+i+1);

    for(auto &&[k, line]: copy | iter::enumerate)
        for(const auto &p: line)
        {
            auto item = viewer->scene.addEllipse(-30, -30, 60, 60, QPen(QColor(my_color.at(k))), QBrush(QColor(my_color.at(k))));
            item->setPos(viewer->robot_poly()->mapToScene(p.x(), p.y()));
            items.push_back(item);
        }
}
void SpecificWorker::draw_top_camera_optic_ray()
{
    // draws a line from the robot to the intersection point with the floor

    static std::vector<QGraphicsItem *> items;
    for(const auto &i: items)
        viewer->scene.removeItem(i);
    items.clear();

    Eigen::Vector3d x1{0.0, 0.0, 0.0};
    Eigen::Vector3d x2{1000.f, 0.0, 0.0};
    Eigen::Vector3d x3{0.0, 1000.f, 0.0};
    Eigen::Hyperplane<double, 3> floor = Eigen::Hyperplane<double, 3>::Through(x1, x2, x3);
    Eigen::Transform<float, 3, Eigen::Affine> t = Eigen::Translation3f(Eigen::Vector3f{0.f, 0.f, consts.top_camera_height}) *
                                                  Eigen::AngleAxisf(robot.camera_tilt_angle, Eigen::Vector3f::UnitX()) *
                                                  Eigen::AngleAxisf(robot.camera_pan_angle, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f x4{consts.top_camera_y_offset, consts.top_camera_y_offset, consts.top_camera_height};
    Eigen::Vector3f x5 = t * Eigen::Vector3f(0.f, 1000.f, 0.f);  // vector pointing to a point of the optic ray
    auto ray = Eigen::Hyperplane<double, 3>::Through(Eigen::Vector3d(0.0,0.0, consts.top_camera_height), Eigen::Vector3d());
    // compute intersection according to https://mathworld.wolfram.com/Line-PlaneIntersection.html
    Eigen::Matrix4f numerator;
    numerator << 1.f, 1.f, 1.f, 1.f,
                 x1.x(), x2.x(), x3.x(), x4.x(),
                 x1.y(), x2.y(), x3.y(), x4.y(),
                 x1.z(), x2.z(), x3.z(), x4.z();
    Eigen::Matrix4f denominator;
    denominator << 1.f, 1.f, 1.f, 0.f,
                   x1.x(), x2.x(), x3.x(), x5.x()-x4.x(),
                   x1.y(), x2.y(), x3.y(), x5.y()-x4.y(),
                   x1.z(), x2.z(), x3.z(), x5.z()-x4.z();
    float k = numerator.determinant()/denominator.determinant();
    float x = x4.x() + (x5.x()-x4.x())*k;
    float y = x4.y() + (x5.y()-x4.y())*k;
    //float z = x4.z() + (x5.z()-x4.z())*k;
    items.push_back(viewer->scene.addLine(0, 0, x, y, QPen(QColor("magenta"), 20)));
}
void SpecificWorker::draw_objects_on_2dview(RoboCompYoloObjects::TObjects objects, const RoboCompYoloObjects::TBox &selected)
{
    static std::vector<QGraphicsItem *> items;
    for(const auto &i: items)
        viewer->scene.removeItem(i);
    items.clear();

    Eigen::Transform<float, 3, Eigen::Affine> t = Eigen::Translation3f(Eigen::Vector3f{0.f, 0.f, consts.top_camera_height}) *
                                                  Eigen::AngleAxisf(robot.camera_tilt_angle, Eigen::Vector3f::UnitX());

    // draw selected
//    auto item = viewer->scene.addRect(-200, -200, 400, 400, QPen(QColor("green"), 20));
//    Eigen::Vector2f corrected = (m * Eigen::Vector3f(selected.x, selected.y, selected.z)).head(2);
//    item->setPos(corrected.x(), corrected.y());
//    items.push_back(item);

    // draw rest
    for(const auto &o: objects)
    {
        const auto &c = COLORS.row(o.type);
        QColor color(c.z(), c.y(), c.x());  //BGR
        auto item = viewer->scene.addRect(-200, -200, 400, 400, QPen(color, 20));
        Eigen::Vector2f corrected = (t * Eigen::Vector4f(o.x, o.y, o.z, 1.f)).head(2);
        //std::cout << t * Eigen::Vector4f(o.x, o.y, o.z, 1.f) << " D: " << o.depth << " Type: " << yolo_object_names.at(o.type) << std::endl;
        item->setPos(corrected.x(), corrected.y());
        items.push_back(item);
    }
}

///
// SUBSCRIPTION to sendData method from JoystickAdapter interface
///
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    Eigen::Vector2f target_force{0.f, 0.f};
    for(const auto &axe : data.axes)
    {
        if (axe.name == "advance") target_force += Eigen::Vector2f{0.f, axe.value/1000.f};
        if (axe.name == "rotate") target_force += Eigen::Vector2f{axe.value, 0.f};
    }
    //set_target_force(target_force);
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)
// this->camerargbdsimple_proxy->getPoints(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

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
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

//////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
