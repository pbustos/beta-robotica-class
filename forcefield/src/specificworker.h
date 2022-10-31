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

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
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
    struct Robot
    {
        float current_adv_speed = 0;
        float current_rot_speed = 0;
        float width = 450;
        float length = 450;
        float semi_width =  width/2;
        const float camera_tilt_angle = 0.35;  //  20º
        float camera_pan_angle = 0.f;
        RoboCompYoloObjects::TBox target;
        bool no_target = true;
    };
    Robot robot;
    struct Constants
    {
        bool IS_COPPELIA = true;
        const float max_camera_depth_range = 5000;
        const float min_camera_depth_range = 300;
        const float omni_camera_height = 580; //mm
        const float omni_camera_y_offset = 120; //mm
        const float omni_camera_x_offset = 0; //mm
        const float top_camera_height = 1555; //mm
        const float top_camera_y_offset = -40; //mm
        float num_angular_bins = 360;
        float coppelia_depth_scaling_factor = 19.f;
        float dreamvu_depth_scaling_factor = 10.f;
        const float max_hor_angle_error = 0.6; // rads
        const float yolo_threshold = 0.5;
        const float depth_lines_max_height = 1550;
        const float depth_lines_min_height = 350;
        const float depth_lines_step = 100;
        const float min_dist_from_robot_center = 300; //mm
        const float max_distance_for_repulsion = 1500; // mm. Distance beyond which repulsion vanishes. It follows an inverse law with current robot speed
        const float speed_for_max_repulsion = 1000;
        float dynamic_threshold = max_distance_for_repulsion/(speed_for_max_repulsion*speed_for_max_repulsion);
        float nu = 0.05f;
        float quadratic_dynamic_threshold_coefficient = max_distance_for_repulsion / (speed_for_max_repulsion * speed_for_max_repulsion);
        const float min_similarity_iou_threshold = 0.5;
        const float min_dist_to_target = 500; //mm
    };
    Constants consts;
    float current_servo_angle = 0.f;
    bool startup_check_flag;

    AbstractGraphicViewer *viewer;

    std::vector<std::vector<Eigen::Vector2f>> get_multi_level_3d_points_omni(const cv::Mat &depth_frame);
    vector<vector<Eigen::Vector2f>> get_multi_level_3d_points_top(const cv::Mat &depth_frame, float focalx, float focaly);
    Eigen::Vector2f compute_repulsion_forces(vector<Eigen::Vector2f> &floor_line);
    cv::Mat read_depth_coppelia();
    cv::Mat read_rgb(const std::string &camera_name);
    std::tuple<cv::Mat, float, float> read_depth_top(const std::string &camera_name);
    void eye_track(bool active_person, const RoboCompYoloObjects::TBox &person_box);
    void move_robot(Eigen::Vector2f force);
    RoboCompYoloObjects::TObjects yolo_detect_objects(cv::Mat rgb);

    // draw
    void draw_floor_line(const vector<vector<Eigen::Vector2f>> &lines, int i=1);
    void draw_forces(const Eigen::Vector2f &force, const Eigen::Vector2f &target, const Eigen::Vector2f &res);
    void draw_objects_on_2dview(RoboCompYoloObjects::TObjects objects, const RoboCompYoloObjects::TBox &selected);
    void draw_dynamic_threshold(float threshold);
    void draw_top_camera_optic_ray();

    // objects
    RoboCompYoloObjects::TObjectNames yolo_object_names;
    Eigen::MatrixX3f COLORS;

    // joy
    void set_target_force(const Eigen::Vector2f &vec);
    Eigen::Vector2f target_force{0.f, 0.f};

    // state machine
    Eigen::Vector2f state_machine(const RoboCompYoloObjects::TObjects &objects);
    enum class State {IDLE, SEARCHING, APPROACHING};
    State state = State::IDLE;
    Eigen::Vector2f search_state(const RoboCompYoloObjects::TObjects &objects);
    Eigen::Vector2f approach_state(const RoboCompYoloObjects::TObjects &objects);

    float iou(const RoboCompYoloObjects::TBox &a, const RoboCompYoloObjects::TBox &b);

};

#endif
