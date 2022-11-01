//
// Created by pbustos on 11/11/21.
//

#ifndef ATTENTION_CONTROL_DYNAMIC_WINDOW_H
#define ATTENTION_CONTROL_DYNAMIC_WINDOW_H

#include <iostream>
#include <tuple>
#include <vector>
#include <optional>
#include <Eigen/Dense>
#include <QPolygonF>
#include <QPointF>
#include <QTransform>
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

class Dynamic_Window
{
    public:
        Dynamic_Window();
        using Result = std::tuple<float, float, float, float, float>;
        // returns adv, rot, side velocities
        std::tuple<float, float, float> update(const Eigen::Vector2f &target_r, const std::vector<Eigen::Vector2f> &ldata, // x,y
                                               float current_adv = 0.f,
                                               float current_rot = 0.f,
                                               AbstractGraphicViewer *viewer = nullptr);

    private:
        std::vector<Result> compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly);
        bool point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly);
        std::optional<Result> compute_optimus(const std::vector<Result> &points, const Eigen::Vector2f &target, float previous_turn);
        Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot);
        Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p, const Eigen::Vector3f &robot);
        inline QPointF to_qpointf(const Eigen::Vector2f &p) const {return QPointF(p.x(), p.y());}
        void draw(const Eigen::Vector3f &robot, const std::vector <Result> &puntos, const std::optional<Result> &best, QGraphicsScene *scene);
        float gaussian(float x);
        void draw_target(const Eigen::Vector2f &target_r, QGraphicsPolygonItem *robot_polygon, QGraphicsScene *scene);

        QPolygonF polygon_robot; // to check if the point is reachable
        struct Constants
        {
            const float robot_semi_width = 400;   // robot semi size
            const float step_along_arc = 200;      // advance step along arc
            const float time_ahead = 1.4;         // time ahead ahead
            const float max_advance_velocity = 1000;
            const float max_rotation_velociy = 2;
        };
        Constants constants;
};

#endif //ATTENTION_CONTROL_DYNAMIC_WINDOW_H
