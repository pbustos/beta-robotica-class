//
// Created by pbustos on 12/11/25.
//

// C++
#pragma once

#include <tuple>
#include <opencv2/opencv.hpp>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <cmath>
#include <Camera360RGB.h>

namespace rc
{
    struct ImageProcessor
    {
        // Detect a large red patch in the given BGR image.
        // - img: input image in BGR color order (OpenCV default)
        // - label_img: optional QLabel to update for visualization (can be nullptr)
        // - min_nonzero: minimum number of red pixels required to consider detection valid
        // Returns: (detected, left_right) where left_right = -1 (left) or 1 (right)

        static std::tuple<bool, int> check_red_patch_in_image(RoboCompCamera360RGB::Camera360RGBPrxPtr proxy, QLabel *label_img = nullptr, int min_nonzero = 1000)
        {
            RoboCompCamera360RGB::TImage img;
            try{ img = proxy->getROI(-1, -1, -1, -1, -1, -1);}
            catch (const Ice::Exception &e){ std::cout << e.what() << " Error reading 360 camera " << std::endl; return {false, 1}; }

            // convert to cv::Mat
            cv::Mat cv_img(img.height, img.width, CV_8UC3, img.image.data());

            // extract a ROI leaving out borders (same as original logic)
            const int left_offset = cv_img.cols / 8;
            const int vert_offset = cv_img.rows / 4;
            const cv::Rect roi(left_offset, vert_offset, cv_img.cols - 2 * left_offset, cv_img.rows - 2 * vert_offset);
            if (roi.width <= 0 || roi.height <= 0) return {false, 1};
            cv_img = cv_img(roi);

            // Convert BGR -> RGB for display
            cv::Mat display_img;
            cv::cvtColor(cv_img, display_img, cv::COLOR_BGR2RGB);

            // Convert RGB -> HSV for color thresholding
            cv::Mat hsv_img;
            cv::cvtColor(display_img, hsv_img, cv::COLOR_RGB2HSV);

            // Red ranges (two intervals)
            cv::Mat mask1, mask2;
            cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
            cv::inRange(hsv_img, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask2);
            cv::Mat mask = mask1 | mask2;

            const int nonZeroCount = cv::countNonZero(mask);
            if (nonZeroCount < min_nonzero)
            {
                // optionally update the label with the ROI preview even when not detected
                if (label_img)
                {
                    QImage qimg(display_img.data, display_img.cols, display_img.rows, static_cast<int>(display_img.step), QImage::Format_RGB888);
                    label_img->setPixmap(QPixmap::fromImage(qimg).scaled(label_img->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
                }
                return {false, 1};
            }

            // compute moments and center of red patch
            const cv::Moments mu = cv::moments(mask, true);
            if (mu.m00 < 1.0) return {false, 1};

            cv::Point2f bestCenter(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));

            // decide turning direction: default right (1), left (-1)
            int left_right = 1;
            if (bestCenter.x < (display_img.cols / 2) && bestCenter.x > 0)
                left_right = -1;

            // check center is near middle of image (tolerance)
            const int tolerance = display_img.cols / 10;
            const int left_bound = display_img.cols / 2 - tolerance;
            const int right_bound = display_img.cols / 2 + tolerance;
            if ((bestCenter.x < left_bound) || (bestCenter.x > right_bound))
            {
                // update display before returning
                if (label_img)
                {
                    QImage qimg(display_img.data, display_img.cols, display_img.rows, static_cast<int>(display_img.step), QImage::Format_RGB888);
                    label_img->setPixmap(QPixmap::fromImage(qimg).scaled(label_img->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
                }
                return {false, left_right};
            }

            // draw marker on detected center and update label if provided
            cv::circle(display_img, bestCenter, 40, cv::Scalar(0, 255, 0), -1);
            if (label_img)
            {
                QImage qimg(display_img.data, display_img.cols, display_img.rows, static_cast<int>(display_img.step), QImage::Format_RGB888);
                label_img->setPixmap(QPixmap::fromImage(qimg).scaled(label_img->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
            }

            return {true, left_right};
        }
    };
}