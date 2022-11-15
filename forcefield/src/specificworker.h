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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <timer/timer.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <dynamic_window.h>
#include <timer/timer.h>
#include "robot.h"
#include "camera.h"

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    using v2f = Eigen::Vector2f;
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

        void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    private:

    rc::Robot robot;
    rc::Camera top_camera;

    struct Constants
    {
        bool IS_COPPELIA = true;
        const float max_camera_depth_range = 5000;
        const float min_camera_depth_range = 300;
        const float omni_camera_height = 580; //mm
        const float omni_camera_y_offset = 120; //mm
        const float omni_camera_x_offset = 0.f; //mm
        const float top_camera_height = 1555; //mm
        const float top_camera_y_offset = -40; //mm
        const float top_camera_x_offset = 0.f; //mm
        const float camera_tilt_angle = -0.35;  //  20º

        int num_angular_bins = 360;
        float coppelia_depth_scaling_factor = 19.f;
        float dreamvu_depth_scaling_factor = 10.f;
        const float max_hor_angle_error = 0.6; // rads
        const float yolo_threshold = 0.6;
        const float depth_lines_max_height = 1550;
        const float depth_lines_min_height = 350;
        const float depth_lines_step = 100;
        const float min_dist_from_robot_center = 300; //mm
        const float max_distance_for_repulsion = 1000; // mm. Distance beyond which repulsion vanishes. It follows an inverse law with current robot speed
        const float speed_for_max_repulsion = 1000;
        //float dynamic_threshold = max_distance_for_repulsion/(speed_for_max_repulsion*speed_for_max_repulsion);
        float dynamic_threshold = 500;
        float nu = 0.1f;   // nervousness
        float quadratic_dynamic_threshold_coefficient = max_distance_for_repulsion / (speed_for_max_repulsion * speed_for_max_repulsion);
        const float min_similarity_iou_threshold = 0.5;
        const float min_dist_to_target = 500; //mm  target is defined 1000 mm before detection
        const float forces_similarity_threshold = 200;
        double xset_gaussian = 0.4;             // gaussian break x set value
        double yset_gaussian = 0.3;             // gaussian break y set value
    };
    Constants consts;
    float current_servo_angle = 0.f;
    bool startup_check_flag;

    AbstractGraphicViewer *viewer;

    std::vector<std::vector<Eigen::Vector2f>> get_multi_level_3d_points_omni(const cv::Mat &depth_frame);
    Eigen::Vector2f compute_repulsion_forces(vector<Eigen::Vector2f> &floor_line);
    cv::Mat read_depth_coppelia();
    void eye_track(rc::Robot &robot); // modifies pan angle
    void move_robot(Eigen::Vector2f force);
    RoboCompYoloObjects::TObjects yolo_detect_objects(cv::Mat rgb);

    // draw
    void draw_floor_line(const vector<vector<Eigen::Vector2f>> &lines, std::initializer_list<int> list);
    void draw_forces(const Eigen::Vector2f &force, const Eigen::Vector2f &target, const Eigen::Vector2f &res);
    void draw_objects_on_2dview(RoboCompYoloObjects::TObjects objects, const RoboCompYoloObjects::TBox &selected);
    void draw_dynamic_threshold(float threshold);
    void draw_top_camera_optic_ray();

    // objects
    RoboCompYoloObjects::TObjectNames yolo_object_names;
    Eigen::MatrixX3f COLORS;

    // joy
    void set_target_force(const Eigen::Vector3f &vec);
    Eigen::Vector3f target_coordinates{0.f, 0.f, 0.f};  //third component for pure  rotations

    // state machine
    Eigen::Vector3f state_machine(const RoboCompYoloObjects::TObjects &objects, const std::vector<Eigen::Vector2f> &line);
    enum class State {IDLE, SEARCHING, APPROACHING, WAITING};
    State state = State::SEARCHING;
    Eigen::Vector3f search_state(const RoboCompYoloObjects::TObjects &objects);
    Eigen::Vector3f approach_state(const RoboCompYoloObjects::TObjects &objects, const std::vector<Eigen::Vector2f> &line);
    Eigen::Vector3f wait_state();

    float iou(const RoboCompYoloObjects::TBox &a, const RoboCompYoloObjects::TBox &b);
    float closest_distance_ahead(const vector<Eigen::Vector2f> &line);

    // DWA
    Dynamic_Window dwa;
    float gaussian(float x);

    std::map<float, float> bumper_points;

    // Clock
    rc::Timer<> clock;


};

#endif
