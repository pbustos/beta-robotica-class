#pragma once
#include <QPointF>
#include <QRectF>
#include <Eigen/Dense>
#include <vector>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>

#include "src/common_types.h"

  struct NominalRoom
  {
      float width; //  mm
      float length;
      std::string name = "unknown";
      Doors doors;
      bool visited = false;

      explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) :
          width(width_), length(length_)
      {};
      [[nodiscard]] Corners corners() const
      {
          // compute corners from width and length
          return {
                        {QPointF{-width/2.f, length/2.f}, 0.f, 0.f},
                        {QPointF{width/2.f, length/2.f}, 0.f, 0.f},
                        {QPointF{width/2.f, -length/2.f}, 0.f, 0.f},
                        {QPointF{-width/2.f, -length/2.f}, 0.f, 0.f}
          };
      }
      [[nodiscard]] QRectF rect() const
      {
          return QRectF{-width/2.f, -length/2.f, width, length};
      }
      [[nodiscard]] Corners transform_corners_to(const Eigen::Affine2f &transform) const  // for room to robot pass the inverse of robot_pose
      {
          Corners transformed_corners;
          for(const auto &[p, _, __] : corners())
          {
              auto ep = Eigen::Vector2f{p.x(), p.y()};
              Eigen::Vector2f tp = transform * ep;
              transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
          }
          return transformed_corners;
      }
      [[nodiscard]] Walls get_walls() const
      {
          auto c = corners();
          c.push_back(c.front()); // to create a closed loop
          Walls walls;
          for (const auto &[i, cs] : corners() | iter::sliding_window(2) | iter::enumerate)
          {
              const auto &[c1, _, __] = cs[0];
              const auto &[c2, ___, ____] = cs[1];
              walls.emplace_back(
              Eigen::ParametrizedLine<float, 2>::Through(Eigen::Vector2f{c1.x(), c1.y()},Eigen::Vector2f{c2.x(), c2.y()}), i, cs[0], cs[1]);
          }
          return walls;
      }
      [[nodiscard]] Wall get_closest_wall_to_point(const Eigen::Vector2f &p)
      {
          const auto walls = get_walls();
          const auto closest_wall = std::ranges::min_element(walls, [&](const auto &w1, const auto &w2) {
                                return std::get<0>(w1).distance(p) < std::get<0>(w2).distance(p);});
          return *closest_wall;
      }
      [[nodiscard]] Eigen::Vector2f get_projection_of_point_on_closest_wall(const Eigen::Vector2f &p)
      {
          const auto wall = get_closest_wall_to_point(p);
          return std::get<0>(wall).projection(p);
      };
      void print() const
      {
          qInfo() << "NominalRoom: name= "<< QString::fromStdString(name) << "width=" << width << " length=" << length;
          qInfo() << "    Corners:";
          for (const auto &[p, _, __] : corners())
          {
              qInfo() << "      (" << p.x() << ", " << p.y() << ")";
          }
          qInfo() << "    Doors:";
          for (const auto &[i, d] : doors | iter::enumerate)
          {
              qInfo() << "        Door index " << i << ": p1_global=(" << d.p1_global.x() << ", " <<
                  d.p1_global.y() << "), p2=(" << d.p2_global.x() << ", " << d.p2_global.y() << ")";
              qInfo() << "        Visited: " << d.visited << ", connects_to_room: " << d.connects_to_room << ", connects_to_door: " << d.connects_to_door;
          }
      }
  };