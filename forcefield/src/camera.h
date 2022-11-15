//
// Created by pbustos on 14/11/22.
//

#ifndef FORCEFIELD_CAMERA_H
#define FORCEFIELD_CAMERA_H

#include <Eigen/Dense>
#include <QtCore>
#include <CameraRGBDSimple.h>
#include <opencv2/core.hpp>

namespace rc
{
    class Camera
    {
        public:
            Camera() = default;
            void initialize(const std::string &name_, RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr proxy_);
            cv::Mat capture_rgb();
            cv::Mat capture_rgbd();
            cv::Mat capture_depth();
            std::vector<std::vector<Eigen::Vector2f>> capture_depth_lines();
            std::vector<std::vector<Eigen::Vector2f>> capture_depth_line(int i);
            float get_depth_focalx() const;
            float get_depth_focaly() const;
            std::vector<std::vector<Eigen::Vector2f>> get_depth_lines_in_robot(float min_height, float max_height, float step_size,
                                                                      const Eigen::Transform<float, 3, Eigen::Affine> &tf);

    private:
            std::string name;
            RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr proxy;

            int rgb_width;
            int rgb_height;
            int rgb_depth;
            int rgb_cameraID;
            float rgb_focalx;
            float rgb_focaly;
            bool rgb_compressed;
            int depth_width;
            int depth_height;
            int depth_depth;
            int depth_cameraID;
            float depth_focalx;
            float depth_focaly;
            bool depth_compressed;

            int num_angular_bins =  360;
            float max_camera_depth_range = 5000; // mm
            float min_camera_depth_range = 300; // mm
    };

} // rc

#endif //FORCEFIELD_CAMERA_H
