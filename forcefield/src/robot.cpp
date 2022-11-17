//
// Created by pbustos on 11/11/22.
//

#include "robot.h"
#include <cppitertools/range.hpp>
#include <ranges>

namespace rc
{
    void Robot::initialize(RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy_)
    {
        this->omnirobot_proxy = omnirobot_proxy_;
    }
    Eigen::Vector3f Robot::get_robot_target_coordinates()
    {
        if(not has_target_flag)
            return Eigen::Vector3f{0.f, 0.f, 0.f};
        if(get_pure_rotation() != 0)
            return Eigen::Vector3f{0.f, 0.f, get_pure_rotation()};

        Eigen::Transform<float, 3, Eigen::Affine> tf = get_tf_cam_to_base();
        Eigen::Vector3f target = tf * get_camera_target_coordinates();
        target[2] = 0.f;  // dismiss pure rotation here
        target = target.normalized() * (target.norm() - min_distance_to_target);  // set target coordinates before the real target
        return target;
    }
    Eigen::Vector3f Robot::get_camera_target_coordinates() const
    {
        return Eigen::Vector3f{current_target.x, current_target.y, current_target.z};
    }
    void Robot::set_current_speed(float adv, float rot)
    {
        current_adv_speed = std::clamp(adv, -max_advance_speed, max_advance_speed);
        current_rot_speed = std::clamp(rot, -max_rot_speed, max_rot_speed);
    }
    void Robot::set_current_advance_speed(float adv)
    {
        current_adv_speed = std::clamp(adv, -max_advance_speed, max_advance_speed);
    }
    void Robot::set_current_rot_speed(float rot)
    {
        current_rot_speed = std::clamp(rot, -max_rot_speed, max_rot_speed);
    }
    void Robot::set_current_pan_angle(float pan)
    {
        camera_pan_angle = std::clamp(pan, min_pan_angle, max_pan_angle);
    }
    void Robot::set_current_target(const RoboCompYoloObjects::TBox &target)
    {
        current_target = target;
        has_target_flag = true;
    }
    void Robot::set_has_target(bool val)
    {
        has_target_flag = val;
        if(val == false)
            current_target.type = -1;
    }
    bool Robot::has_target() const
    {
        return has_target_flag;
    }

    float Robot::get_current_advance_speed() const
    {
        return current_adv_speed;
    }
    float Robot::get_current_rot_speed() const
    {
        return current_rot_speed;
    }
    float Robot::get_target_angle_in_frame() const
    {
        return atan2(current_target.x, current_target.y);
    }
    float Robot::get_current_pan_angle() const
    {
        return camera_pan_angle;
    }
    RoboCompYoloObjects::TBox Robot::get_current_target() const
    {
        return current_target;
    }
    void Robot::set_desired_distance_to_target(float dist)
    {
        min_distance_to_target = dist;
    }
    float Robot::get_distance_to_target()
    {
        return get_robot_target_coordinates().norm();
    }
    Eigen::Transform<float, 3, Eigen::Affine> Robot::get_tf_cam_to_base()
    {
//        Eigen::Transform<float, 3, Eigen::Affine> tf = Eigen::Translation3f(Eigen::Vector3f{0.f, 0.f, top_camera_height}) *
//                                                       Eigen::AngleAxisf(camera_tilt_angle, Eigen::Vector3f::UnitX()) *
//                                                       Eigen::AngleAxisf(camera_pan_angle, Eigen::Vector3f::UnitZ());
        for(const auto &a: axes)
            if(a == "z")
                tf.linear() = (Eigen::AngleAxisf(camera_tilt_angle, Eigen::Vector3f::UnitX()) *
                               Eigen::AngleAxisf(camera_pan_angle, Eigen::Vector3f::UnitZ())).toRotationMatrix();

        return tf;
    }

    void Robot::print()
    {
        std::cout << "---- Robot Current State ----" << std::endl;
        std::cout << "  Advance speed: " << current_adv_speed << std::endl;
        std::cout << "  Rotation speed: " << current_rot_speed << std::endl;
        std::cout << "  Pan angle: " << camera_pan_angle << std::endl;

        if(has_target_flag)
        {
            std::cout << "  Target:" << std::endl;
            std::cout << "      Type: " << current_target.type << std::endl;
            std::cout << "      Cam coor: [" << get_camera_target_coordinates().x() << ", " << get_camera_target_coordinates().y() << "]" << std::endl;
            std::cout << "      Robot coor: [" << get_robot_target_coordinates().x() << ", " << get_camera_target_coordinates().y() << "]" << std::endl;
            std::cout << "      Distance: " << get_distance_to_target() << std::endl;
        }
    }
    void Robot::create_bumper(float offset,  AbstractGraphicViewer *viewer)
    {
        // create bumper
        // sample the robot's contour at equally separated 360 angular bins
        //  get rectangle internal angles
        float s1  = atan2(semi_height+offset, semi_width+offset);
        float s2 = atan2(semi_width+offset, semi_height+offset);
        int dsample = 6;

        // compute angle samples grouped by the rectangle's internal angles, starting from -PI and all the way to PI
        sector1 = Eigen::ArrayXf::LinSpaced(Eigen::Sequential, (int)qRadiansToDegrees(s1)/dsample, -M_PI, -M_PI+s1);
        sector2 = Eigen::ArrayXf::LinSpaced(Eigen::Sequential, (int)qRadiansToDegrees(2*s2)/dsample, -M_PI+s1, -s1);
        sector3 = Eigen::ArrayXf::LinSpaced(Eigen::Sequential, (int)qRadiansToDegrees(2*s1)/dsample, -s1, s1);
        sector4 = Eigen::ArrayXf::LinSpaced(Eigen::Sequential, (int)qRadiansToDegrees(2*s2)/dsample, s1, s1+2*s2);
        sector5 = Eigen::ArrayXf::LinSpaced(Eigen::Sequential, (int)qRadiansToDegrees(s1)/dsample, s1+2*s2, M_PI);

        // compute the distances to the border of the rectangle
        float semi_offset = offset/2;
        std::map<float, float> bumper_points_rotated;
        for(auto &&i : iter::range(sector1.rows()))
            bumper_points_rotated.insert(std::pair(sector1(i), fabs((semi_width+semi_offset)/cos(sector1(i)))));
        for(auto &&i : iter::range(sector2.rows()))
            bumper_points_rotated.insert(std::pair(sector2(i), fabs((semi_height+semi_offset)/sin(sector2(i)))));
        for(auto &&i : iter::range(sector3.rows()))
            bumper_points_rotated.insert(std::pair(sector3(i), fabs((semi_width+semi_offset)/cos(sector3(i)))));
        for(auto &&i : iter::range(sector4.rows()))
            bumper_points_rotated.insert(std::pair(sector4(i), fabs((semi_height+semi_offset)/sin(sector4(i)))));
        for(auto &&i : iter::range(sector5.rows()))
            bumper_points_rotated.insert(std::pair(sector5(i), fabs((semi_height+semi_offset)/cos(sector5(i)))));

        // the vector runs from -pi to p1 in 360 steps, but 0 points towards the X axis.
        // we need to add pi/2 to the first part and 3pi/2 to the rest, so it is "rotated"  ccw 90 degrees and zero is the robot's nose
        for(const auto &[k, v]:  bumper_points_rotated)
            if(k <= M_PI/2.f)
                bumper.insert(std::pair(k+M_PI/2.f, v));
            else
                bumper.insert(std::pair(k-3.f*M_PI_2, v));

        // draw bumper in viewer
        if(viewer != nullptr)
            for(const auto &[ang, dist] : bumper)
            {
                auto item = viewer->scene.addEllipse(-offset/2, -offset/2, offset, offset, QPen(QColor("orange"), 3));
                item->setPos(dist*sin(ang), dist*cos(ang));
            }
    }
    void Robot::recompute_bumper(float dynamic_offset)
    {
        for(auto &&i : iter::range(sector1.rows()))
            bumper[sector1(i)] = fabs((semi_width+dynamic_offset)/cos(sector1(i)));
        for(auto &&i : iter::range(sector2.rows()))
            bumper[sector2(i)] =  fabs((semi_height+dynamic_offset)/sin(sector2(i)));
        for(auto &&i : iter::range(sector3.rows()))
            bumper[sector3(i)] = fabs((semi_width+dynamic_offset)/cos(sector3(i)));
        for(auto &&i : iter::range(sector4.rows()))
            bumper[sector4(i)] = fabs((semi_height+dynamic_offset)/sin(sector4(i)));
        for(auto &&i : iter::range(sector5.rows()))
            bumper[sector5(i)] = fabs((semi_height+dynamic_offset)/cos(sector5(i)));
    }
//    void Robot::add_camera(const std::string &name, const Eigen::Transform<float, 3, Eigen::Affine> &tf,
//                           std::map<std::string, RoboCompJointMotorSimple::JointMotorSimplePrxPtr> axes)
    void Robot::add_camera(const Eigen::Transform<float, 3, Eigen::Affine> &tf_,
                           const std::vector<std::string> &axes_,
                           RoboCompJointMotorSimple::JointMotorSimplePrxPtr joint_proxy_)
    {
        this->tf = tf_;
        this->axes = axes_;
        this->joint_motor_proxy = joint_proxy_;
        std::ranges::sort(this->axes);  // to get x, y z
        update_joints();
    }

    void Robot::update_joints()
    {
        try
        {
            if(float angle = joint_motor_proxy->getMotorState("camera_pan_joint").pos; not std::isnan(angle))
                set_current_pan_angle(angle);
            else
            {
                qWarning() << "NAN value in servo position";
                return;
            }
        }
        catch (const Ice::Exception &e)
        {
            std::cout << e.what() << " Warning: error connecting with jointmotorsimple" << std::endl;
            return;
        }
    }
    void Robot::update_speed()
    {
        RoboCompGenericBase::TBaseState bState;
        try
        {
            omnirobot_proxy->getBaseState(bState);
            set_current_speed(bState.advVz, bState.rotV);
        }
        catch (const Ice::Exception &e)
        { std::cout << e.what() << " Error reading omnirobot_proxy::getBaseSpeed" << std::endl; }
    }


} // rc

//
//try
//{
//if(float angle = joint_proxy->getMotorState("camera_pan_joint").pos; not std::isnan(angle))
//set_current_pan_angle(angle);
//else
//{
//qWarning() << "NAN value in servo position";
//return;
//}
//}
//catch (const Ice::Exception &e)
//{
//std::cout << e.what() << " Warning: error connecting with jointmotorsimple" << std::endl;
//return;
//}