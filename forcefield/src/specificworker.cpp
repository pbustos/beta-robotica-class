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
#include <cppitertools/chunked.hpp>
#include <cppitertools/sliding_window.hpp>

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
    jointmotorsimple_proxy->setVelocity("camera_pan_joint", RoboCompJointMotorSimple::MotorGoalVelocity{0.f, 1.f});
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
	std::cout << "Initializing worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        // graphics
        viewer = new AbstractGraphicViewer(this->beta_frame,  QRectF(-2500, -2500, 5000, 5000));
        this->resize(900,650);
        viewer->add_robot(robot.width, robot.length);

        // initialize top_camera
        std::string top_camera_name = "/Shadow/camera_top";
        try{ top_camera.initialize(top_camera_name, camerargbdsimple_proxy); }
        catch(...){ std::cout << "Error initializing camera " << top_camera_name << ". Aborting" << std::endl; std::terminate();}

        // sets servo to zero position
        // TODO: pasar a Robot
        RoboCompJointMotorSimple::MotorState servo_state;
        while(true)
            try
            {
                servo_state = jointmotorsimple_proxy->getMotorState("camera_pan_joint");
                if( fabs(servo_state.pos)  < 0.03)  break;
                jointmotorsimple_proxy->setPosition("camera_pan_joint", RoboCompJointMotorSimple::MotorGoalPosition{0.f, 1.f});
                usleep(100000);
            }
            catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; return;}

        // camera position wrt to robot
        Eigen::Transform<float, 3, Eigen::Affine> tf(Eigen::Translation3f(Eigen::Vector3f{0.f, 0.f, consts.top_camera_height}) *
                                                     Eigen::AngleAxisf(consts.camera_tilt_angle, Eigen::Vector3f::UnitX()) *
                                                     Eigen::AngleAxisf(servo_state.pos, Eigen::Vector3f::UnitZ()));

        // initialize robot
        robot.initialize(omnirobot_proxy /*viewer*/);
        robot.add_camera(tf, {"z"}, jointmotorsimple_proxy);

        // get list of object's names from YOLO
        try
        {yolo_object_names = yoloobjects_proxy->getYoloObjectNames(); }
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

        // create bumper
        float security_threshold = 100;
        robot.create_bumper(security_threshold, viewer);

        Period = 50;
        timer.start(Period);
        std::cout << "Worker initialized OK" << std::endl;
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
    auto top_rgb_frame = top_camera.capture_rgb();
    if(top_rgb_frame.empty()) { qWarning() << "rgb_top_frame empty"; return;}

    /// get current robot speed and servo positon;
    robot.update_speed();
    robot.update_joints();

    /// compute level_lines
    auto omni_lines = get_multi_level_3d_points_omni(omni_depth_frame);
    //draw_floor_line(omni_lines, {1});
    auto current_line = omni_lines[1];  // second line of the list of laser lines at different heights
    //auto top_lines = get_multi_level_3d_points_top(top_depth_frame, top_camera.get_depth_focalx(), top_camera.get_depth_focaly());
    auto top_lines  = top_camera.get_depth_lines_in_robot(0, 1600, 50, robot.get_tf_cam_to_base());
    draw_floor_line(top_lines, {1});

    /// YOLO
    RoboCompYoloObjects::TObjects objects = yolo_detect_objects(top_rgb_frame);

    /// draw top image
    //cv::imshow("top", top_rgb_frame); cv::waitKey(5);

    /// draw yolo_objects on 2D view
    draw_objects_on_2dview(objects, RoboCompYoloObjects::TBox());

    // TODO:: STATE MACHINE
    // state machine to activate basic behaviours. Returns a  target_coordinates vector
    //  state_machine(objects, current_line);

    /// eye tracking: tracks  current selected object or  IOR if none
    eye_track(robot);
    draw_top_camera_optic_ray();

    // DWA algorithm
    auto [adv, rot, side] =  dwa.update(robot.get_robot_target_coordinates(), current_line, robot.get_current_advance_speed(), robot.get_current_rot_speed(), viewer);

    //qInfo() << __FUNCTION__ << adv <<  side << rot;
    //    try{ omnirobot_proxy->setSpeedBase(side, adv, rot); }
    //    catch(const Ice::Exception &e){ std::cout << e.what() << "Error connecting to omnirobot" << std::endl;}
    // execute move commands
    //move_robot(force);

    //robot.print();
}

//////////////////// ELEMENTS OF CONTROL/////////////////////////////////////////////////
// perception
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
    { return names[p.type] != "person" and names[p.type] != "chair"; }), yolo_objects.objects.end());

    // draw boxes
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

// control
Eigen::Vector2f SpecificWorker::compute_repulsion_forces(vector<Eigen::Vector2f> &floor_line)
{
    // update threshold with speed
    //    if( fabs(robot.current_adv_speed) > 10.f)
    //        consts.dynamic_threshold = consts.quadratic_dynamic_threshold_coefficient * (robot.current_adv_speed * robot.current_adv_speed);
    //    else
    //        consts.dynamic_threshold = robot.width;
    //qInfo() << __FUNCTION__ << consts.dynamic_threshold << robot.current_adv_speed  << "[" << target_coordinates.x() << target_coordinates.y()  << "]";

    //  computation in meters to reduce the size of the numbers
    Eigen::Vector2f res = {0.f, 0.f};
    float threshold = consts.dynamic_threshold/1000.f;   // to meters
    float max_dist = consts.max_distance_for_repulsion/1000.f;
    for(const auto &ray: floor_line)
    {
        const float &dist = (ray/1000.f).norm();
        if (dist <= threshold)
            res += consts.nu * (1.0 / dist - 1.0 / max_dist) * (1.0 / (dist * dist)) * (-(ray/1000.f) / dist);  // as in original paper
    }
    return res*1000.f; //mm
}
std::vector<std::vector<Eigen::Vector2f>> SpecificWorker::get_multi_level_3d_points_omni(const cv::Mat &depth_frame)
{
    std::vector<std::vector<Eigen::Vector2f>> points(int((consts.depth_lines_max_height-consts.depth_lines_min_height)/consts.depth_lines_step));  //height steps
    for(auto &p: points)
        p.resize(consts.num_angular_bins, Eigen::Vector2f(consts.max_camera_depth_range, consts.max_camera_depth_range));   // angular resolution

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
                    // maps -pi:pi to 0:360, with 0 in the middle of the array
                    int ang_index = floor((M_PI + atan2(x, y)) / ang_bin);
                    Eigen::Vector2f new_point(x, y);
                    if(new_point.norm() <  points[level][ang_index].norm() and new_point.norm() > consts.min_dist_from_robot_center)
                        points[level][ang_index] = new_point;
                }
        };

    // filter initialized points not filled with real measures, with the value of its closest valid neighboor
    auto nearest_initialized_neighboor = [c=consts](const std::vector<Eigen::Vector2f> &line, std::vector<Eigen::Vector2f>::const_iterator it)
            {
                // go from index alternating left and right until condition is met
                auto it_l = it+1; auto it_r = it-1;
                bool end_r = false;
                bool end_l = false;
                while(not end_r and not end_l)
                    if(it_r->x() != c.max_camera_depth_range and it_r->y() != c.max_camera_depth_range)
                        return  *it_r;
                    else if (it_l->x() != c.max_camera_depth_range and it_l->y() != c.max_camera_depth_range)
                            return *it_l;
                    else
                    {
                        if (it_r != line.end()) it_r++; else end_r = true;
                        if (it_l != line.begin()) it_l--; else end_l = true;
                    }
                return Eigen::Vector2f{0.f, 0.f};  // should not go through here
            };
    for(auto &level : points)
        for(auto it=level.begin(); it!=level.end(); it++)
            if(it->x() == consts.max_camera_depth_range and it->y() == consts.max_camera_depth_range)
                *it = nearest_initialized_neighboor(level, it);

    return points;
}
void SpecificWorker::set_target_force(const Eigen::Vector3f &vec)
{
    target_coordinates = vec * 1000;  //to mm/sg
}

// action
void SpecificWorker::eye_track(rc::Robot &robot)
{
    static float error_ant = 0.f;
    if(robot.has_target())
    {
        float hor_angle = robot.get_target_angle_in_frame();  // angle wrt camera origin
        if( fabs(hor_angle) > consts.max_hor_angle_error)  // saccade
        {
            try
            {
                float error = 0.4 * (robot.get_current_pan_angle() - hor_angle);
                float new_angle = std::clamp(error, -1.f, 1.f);  // dumping
                //qInfo() << __FUNCTION__ << "image error" << hor_angle << "error" << error << "saccade to: " << new_angle;
                jointmotorsimple_proxy->setPosition("camera_pan_joint", RoboCompJointMotorSimple::MotorGoalPosition{new_angle, 1});
            }
            catch (const Ice::Exception &e)
            {  std::cout << e.what() << " Error connecting to MotorGoalPosition" << std::endl; return; }
        }
        else    // smooth pursuit
        {
            try
            {
                float new_vel = -hor_angle + 0.3 * (hor_angle - error_ant);
                //new_vel = std::clamp(new_vel, -1.f, 1.f);  // dumping
                new_vel -= 0.5 * robot.get_current_rot_speed();  // compensate with current base rotation speed
                //qInfo() << __FUNCTION__ << "image error" << hor_angle << "smooth vel: " << new_vel;
                jointmotorsimple_proxy->setVelocity("camera_pan_joint", RoboCompJointMotorSimple::MotorGoalVelocity{new_vel, 1});
                //qInfo() << __FUNCTION__ << "smooth" << hor_angle << current_servo_angle << new_vel;
            }
            catch (const Ice::Exception &e)
            {  std::cout << e.what() << " Error connecting to MotorGoalPosition" << std::endl; return; }
        }
    }
    else  // inhibition of return
        try
        { jointmotorsimple_proxy->setPosition("camera_pan_joint", RoboCompJointMotorSimple::MotorGoalPosition{0.f, 1.f}); }
        catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; return;}

}
void SpecificWorker::move_robot(Eigen::Vector2f force)
{
    //auto sigmoid = [](auto x){ return std::clamp(x / 1000.f, 0.f, 1.f);};
    try
    {
        Eigen::Vector2f gains{0.8, 0.8};
        force = force.cwiseProduct(gains);
        float rot = atan2(force.x(), force.y())  - 0.9*current_servo_angle;  // dumps rotation for small resultant force
        float adv = force.y() ;
        float side = force.x() ;
        qInfo() << __FUNCTION__ << side << adv << rot;
        omnirobot_proxy->setSpeedBase(side, adv, rot);
    }
    catch (const Ice::Exception &e){ std::cout << e.what() << std::endl;}
}

///////////////////  State machine ////////////////////////////////////////////


///////////////////// Aux //////////////////////////////////////////////////////////////////
float SpecificWorker::closest_distance_ahead(const std::vector<Eigen::Vector2f> &line)
{
    // minimum distance in central sector
    if(line.empty()) { qWarning() << __FUNCTION__ << "Empty line vector"; return 0.f;}
    size_t offset = 3*line.size()/7;
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

/////////////////// Draw  /////////////////////////////////////////////////////////////
void SpecificWorker::draw_floor_line(const vector<vector<Eigen::Vector2f>> &lines, std::initializer_list<int> list)    //one vector for each height level
{
    static std::vector<QGraphicsItem *> items;
    for(const auto &item: items)
        viewer->scene.removeItem(item);
    items.clear();

    if(list.size() > lines.size()) {qWarning()<< "Requested list bigger than data. Returning"; return;}
    std::vector<vector<Eigen::Vector2f>> copy_of_line(list.size());
    for(auto &&[i, e]: list|iter::enumerate)
        copy_of_line[i] =  lines[e];

    for(auto &&[k, line]: copy_of_line | iter::enumerate)
    {
        //qInfo() << __FUNCTION__ << k << (int)COLORS.row(k).x();
        QColor color((int)COLORS.row(k).x(), (int)COLORS.row(k).y(), (int)COLORS.row(k).z());
        QBrush brush(color);
        for(const auto &p: line)
        {
            auto item = viewer->scene.addEllipse(-30, -30, 60, 60, color, brush);
            item->setPos(p.x(), p.y());
            items.push_back(item);
        }
    }
}
void SpecificWorker::draw_forces(const Eigen::Vector2f &force, const Eigen::Vector2f &target, const Eigen::Vector2f &res)
{
      static std::vector<QGraphicsItem *> items;
    for(const auto &i: items)
        viewer->scene.removeItem(i);
    items.clear();

    auto large_force = force * 3.f;
    QPointF tip1 = viewer->robot_poly()->mapToScene(large_force.x(), large_force.y());
    QPointF tip2 = viewer->robot_poly()->mapToScene(target.x(), target.y());
    QPointF tip3 = viewer->robot_poly()->mapToScene(res.x(), res.y());
    items.push_back(viewer->scene.addLine(viewer->robot_poly()->x(), viewer->robot_poly()->pos().y(), tip1.x(), tip1.y(), QPen(QColor("red"), 50)));
    auto item = viewer->scene.addEllipse(-50, -50, 100, 100, QPen(QColor("red")), QBrush(QColor("red")));
    item->setPos(tip1.x(), tip1.y()); items.push_back(item);
    items.push_back(viewer->scene.addLine(viewer->robot_poly()->pos().x(), viewer->robot_poly()->pos().y(), tip2.x(), tip2.y(), QPen(QColor("blue"), 50)));
    item = viewer->scene.addEllipse(-50, -50, 100, 100, QPen(QColor("blue")), QBrush(QColor("blue")));
    item->setPos(tip1.x(), tip1.y()); items.push_back(item);
    items.push_back(viewer->scene.addLine(viewer->robot_poly()->pos().x(), viewer->robot_poly()->pos().y(), tip3.x(), tip3.y(), QPen(QColor("green"), 50)));
    item = viewer->scene.addEllipse(-50, -50, 100, 100, QPen(QColor("green")), QBrush(QColor("green")));
    item->setPos(tip1.x(), tip1.y()); items.push_back(item);
}
void SpecificWorker::draw_top_camera_optic_ray()
{
    // draws a line from the robot to the intersection point with the floor
    static std::vector<QGraphicsItem *> items;
    for(const auto &i: items)
        viewer->scene.removeItem(i);
    items.clear();

    // plane
    Eigen::Vector3d x1{0.0, 0.0, 0.0};
    Eigen::Vector3d x2{1000.f, 0.0, 0.0};
    Eigen::Vector3d x3{0.0, 1000.f, 0.0};
    Eigen::Hyperplane<double, 3> floor = Eigen::Hyperplane<double, 3>::Through(x1, x2, x3);
    // line
    Eigen::Transform<float, 3, Eigen::Affine> tf = robot.get_tf_cam_to_base();
    Eigen::Vector3f x4{consts.top_camera_x_offset, consts.top_camera_y_offset, consts.top_camera_height};
    Eigen::Vector3f x5 = tf * Eigen::Vector3f(0.f, 1000.f, 0.f);  // vector pointing to a point of the optic ray
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
    items.push_back(viewer->scene.addLine(0, 0, -x, -y, QPen(QColor("darkgrey"), 20)));
}
void SpecificWorker::draw_objects_on_2dview(RoboCompYoloObjects::TObjects objects, const RoboCompYoloObjects::TBox &selected)
{
    static std::vector<QGraphicsItem *> items;
    for(const auto &i: items)
        viewer->scene.removeItem(i);
    items.clear();

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

        Eigen::Vector2f corrected = (robot.get_tf_cam_to_base() * Eigen::Vector3f(o.x, o.y, o.z)).head(2);
        item->setPos(corrected.x(), corrected.y());
        items.push_back(item);
        Eigen::Vector3f yolo = robot.get_tf_cam_to_base() * Eigen::Vector3f(o.x, o.y, o.z);
        //qInfo() << __FUNCTION__ << corrected.x() << corrected.y() << yolo.x() << yolo.y();
    }
}
void SpecificWorker::draw_dynamic_threshold(float threshold)
{
    static std::vector<QGraphicsItem *> items;
    for(const auto &i: items)
        viewer->scene.removeItem(i);
    items.clear();

    items.push_back(viewer->scene.addEllipse(-threshold, -threshold, 2*threshold, 2* threshold, QPen(QColor("yellow"), 20)));
}

///
// SUBSCRIPTION to sendData method from JoystickAdapter interface
///
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    Eigen::Vector2f target_force{0.f, 0.f};
//    for(const auto &axe : data.axes)
//    {
//        if (axe.name == "advance") target_coordinates += Eigen::Vector2f{0.f, axe.value/1000.f};
//        if (axe.name == "rotate") target_coordinates += Eigen::Vector2f{axe.value, 0.f};
//    }
    //set_target_force(target_coordinates);
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


//        int row = (o.top + o.bot)/2;
//        int col = (o.left + o.right)/2;
//        float dist = depth_frame.ptr<float>(row)[col]*1000.f;  //  -> to mm   WARN:  assuming depth and rgb are  the same size
//        // TODO: do not assume depth and rgb are the same size
//        if(std::isnan(dist)) {qWarning() << " Distance value un depth frame coors " << o.x << o.y << "is nan:";};
//        if(dist > consts.max_camera_depth_range or dist < consts.min_camera_depth_range) continue;
//        // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
//        float y = dist;
//        float x = (col-(depth_frame.cols/2.f)) * y / focalx;
//        float z = -(row-(depth_frame.rows/2.f)) * y / focaly;