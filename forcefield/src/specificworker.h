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
    struct Constants
    {
        bool IS_COPPELIA = true;
        const float max_camera_depth_range = 5000;
        const float min_camera_depth_range = 300;
        const float omni_camera_height = 580; //mm
        const float top_camera_height = 1555; //mm
        float robot_length = 500;
        float num_angular_bins = 360;
        float coppelia_depth_scaling_factor = 19.f;
        float dreamvu_depth_scaling_factor = 10.f;
        const float max_hor_angle_error = 0.6; // rads
        const float yolo_threshold = 0.5;
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

    // objects
    RoboCompYoloObjects::TObjectNames yolo_object_names;
    Eigen::MatrixX3f COLORS;

    void draw_objects_on_2dview(RoboCompYoloObjects::TObjects objects, const RoboCompYoloObjects::TBox &selected);

    cv::Mat read_depth(const string &camera_name);

};

#endif
