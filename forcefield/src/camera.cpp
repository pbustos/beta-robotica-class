//
// Created by pbustos on 14/11/22.
//

#include "camera.h"
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>

namespace rc
{
    void Camera::initialize(const std::string &name_, RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr proxy_)
    {
        std::cout << __FUNCTION__ << "Testing camera connection..." << std::endl;
        name = name_;
        proxy = proxy_;
        RoboCompCameraRGBDSimple::TImage rgb;
        RoboCompCameraRGBDSimple::TDepth depth;
        RoboCompCameraRGBDSimple::TRGBD rgbd;
        try
        {
            rgb = proxy->getImage(name);
            if (not rgb.image.empty() and rgb.image.size() == (size_t)(rgb.width*rgb.height*rgb.depth))
            {
                rgb_width = rgb.width;
                rgb_height = rgb.height;
                rgb_depth = rgb.depth;
                rgb_cameraID = rgb.cameraID;
                rgb_focalx = rgb.focalx;
                rgb_focaly = rgb.focaly;
                rgb_compressed = rgb.compressed;
            }
            else
            {
                std::cout << __FUNCTION__ << "Warning: Image read is empty -" << rgb.image.size() << "- or dimensions don't agree: rows = "
                          << rgb.height << " cols = " << rgb.width << "  depth = " << rgb.depth << std::endl;
                throw std::runtime_error(std::string("Warning: Image read is empty -" + std::to_string(rgb.image.size())
                                                     + "- or dimensions don't agree: rows = "
                                                     + std::to_string(rgb.height) + " cols = " + std::to_string(rgb.width)
                                                     + "  depth = " + std::to_string(rgb.depth)));
            }
        }
        catch (const Ice::Exception &e)
        {
            std::cout << e.what() << std::endl;
            throw std::runtime_error("Warning: Error reading camerargbdsimple_proxy::getImage");
        }

        try
        {
            depth = proxy->getDepth(name);
            if (not depth.depth.empty() and depth.depth.size() == depth.width*depth.height*sizeof(float))
            {
                depth_width = depth.width;
                depth_height = depth.height;
                depth_cameraID = depth.cameraID;
                depth_focalx = depth.focalx;
                depth_focaly = depth.focaly;
                depth_compressed = depth.compressed;
            }
            else
                throw std::runtime_error(std::string("Warning: Depth read is empty -" + std::to_string(rgb.image.size())
                                                     + "- or dimensions don't agree: rows = "
                                                     + std::to_string(rgb.height) + " cols = " + std::to_string(rgb.width)));
        }
        catch (const Ice::Exception &e)
        {
            std::cout << e.what() << std::endl;
            throw std::runtime_error("Warning: Error reading camerargbdsimple_proxy::depthImage");
        }

        std::cout << __FUNCTION__ << "Camera " << name << " tested and operational" << std::endl;
    };
    cv::Mat Camera::capture_rgb()
    {
        RoboCompCameraRGBDSimple::TImage rgb;
        cv::Mat rgb_frame;
        try
        {
            rgb = proxy->getImage(name);
            if (not rgb.image.empty())
                rgb_frame= cv::Mat(cv::Size(rgb.width, rgb.height), CV_8UC3, &rgb.image[0], cv::Mat::AUTO_STEP);
        }
        catch (const Ice::Exception &e)
        { std::cout << e.what() << " Error reading camerargbdsimple_proxy::getImage" << std::endl;}
        return rgb_frame.clone();
    }
    cv::Mat Camera::capture_depth()
    {
        RoboCompCameraRGBDSimple::TDepth depth;
        cv::Mat depth_frame;
        try
        {
            depth = proxy->getDepth(name);
            if (not depth.depth.empty())
                depth_frame = cv::Mat(cv::Size(depth.width, depth.height), CV_32FC1, &depth.depth[0], cv::Mat::AUTO_STEP);

        }
        catch (const Ice::Exception &e)
        { std::cout << e.what() << " Error reading camerargbdsimple_proxy::getDepth" << std::endl;}
        return depth_frame.clone();
    }
    float Camera::get_depth_focalx() const
    {
        return depth_focalx;
    }
    float Camera::get_depth_focaly() const
    {
        return depth_focaly;
    }
    std::vector<std::vector<Eigen::Vector2f>> Camera::get_depth_lines_in_robot(float min_height, float max_height, float step_size,
                                                                               const Eigen::Transform<float, 3, Eigen::Affine> &tf)
    {
        //const float min_height = 0, max_height = 1600, step_size = 50;
        cv::Mat depth_frame = this->capture_depth();
        std::vector<std::vector<Eigen::Vector2f>> points(int((max_height-min_height)/step_size));  //height steps
        for(auto &p: points)
            p.resize(num_angular_bins, Eigen::Vector2f(max_camera_depth_range*2, max_camera_depth_range*2));   // angular resolution

        float  dist, x, y, z;
        const float ang_bin = 2.f*M_PI/num_angular_bins;
        for(int u=0; u<depth_frame.rows; u++)
            for(int v=0; v<depth_frame.cols; v++)
            {
                dist = depth_frame.ptr<float>(u)[v]*1000.f;  //  -> to mm
                if(std::isnan(dist)) {qWarning() << " Distance value un depth frame coors " << u << v << "is nan:";};
                if(dist > max_camera_depth_range or dist < min_camera_depth_range) continue;
                // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up). Dist 0 Y
                y = dist;
                x = (v-(depth_frame.cols/2.f)) * y / depth_focalx;
                z = -(u-(depth_frame.rows/2.f)) * y / depth_focaly;

                // convert to robot's CS
                Eigen::Vector3f robot_p  =  tf * Eigen::Vector3f(x, y, z);

                // add only to its bin if less than current value
                for(auto &&[level, step] : iter::range(min_height, max_height, step_size) | iter::enumerate)
                    if (robot_p.z() > step and  robot_p.z() < step + step_size and level < points.size())
                    {
                        int ang_index = floor((M_PI + atan2(robot_p.x(), robot_p.y())) / ang_bin);  //all positive starting at zero
                        Eigen::Vector2f &p = points[level][ang_index];
                        if (robot_p.head(2).norm() < p.norm() /*and new_point.norm() > 400*/)
                            p = robot_p.head(2);
                    }
            };
        // remove all bins not hit by a points
        for(auto &level : points)
            level.erase(std::remove_if(level.begin(), level.end(), [d=max_camera_depth_range](auto p){ return p.x()==d*2 and p.y()==d*2;}), level.end());
        return points;
    }

} // rc