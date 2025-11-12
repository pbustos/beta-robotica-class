#pragma once
#include <QPointF>
#include <QRectF>
#include <Eigen/Dense>
#include <vector>
#include "src/common_types.h"

  struct NominalRoom
        {
            float width; //  mm
            float length;
            explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) :
                width(width_), length(length_)
            {};
            [[nodiscard]] Corners corners() const
            {
                // compute corners from width and length
                return {
                    {QPointF{-width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, length/2.f}, 0.f, 0.f},
                    {QPointF{-width/2.f, length/2.f}, 0.f, 0.f}
                };
            }
            [[nodiscard]] QRectF rect() const
            {
                return QRectF{-width/2.f, -length/2.f, width, length};
            }
            [[nodiscard]] Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
            {
                Corners transformed_corners;
                for(const auto &[p, _, __] : corners())
                {
                    auto ep = Eigen::Vector2d{p.x(), p.y()};
                    Eigen::Vector2d tp = transform * ep;
                    transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
                }
                return transformed_corners;
            }
        };