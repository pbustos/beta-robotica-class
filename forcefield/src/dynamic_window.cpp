//
// Created by pbustos on 11/11/21.
//

#include "dynamic_window.h"
#include <QtCore>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>

Dynamic_Window::Dynamic_Window()
{
    polygon_robot <<  QPointF(-constants.robot_semi_width, constants.robot_semi_width) <<
                      QPointF(constants.robot_semi_width, constants.robot_semi_width) <<
                      QPointF(constants.robot_semi_width, -constants.robot_semi_width) <<
                      QPointF(-constants.robot_semi_width, -constants.robot_semi_width);
}

std::tuple<float, float, float> Dynamic_Window::update(const Eigen::Vector2f &target_r,
                                                       const std::vector<Eigen::Vector2f> &ldata, // x,y
                                                       float current_adv, float current_rot,
                                                       AbstractGraphicViewer *viewer)
{
    static float previous_turn = 0;
    //float robot_angle = robot_pos[2];
    // advance velocity should come from robot. It is computed here from world referenced velocities
    //float current_adv = -sin(robot_angle)*robot_vel[0] + cos(robot_angle)*robot_vel[1];
    //float current_rot = robot_vel[2];  // Rotation W
    if(fabs(current_adv) > constants.max_advance_velocity or fabs(current_rot)>constants.max_rotation_velociy)
    {
        qWarning() << __FILE__ << __FUNCTION__ << "Advance or rotation speed are too high. Assuming 0 for both.";
        current_rot = 0.f; current_adv = 0.f;
    }

    // compute future positions of the robot
    QPolygonF laser_poly;
    for(const auto &l : ldata)
        laser_poly << QPointF(l.x(), l.y());
    auto point_list = compute_predictions(current_adv, current_rot, laser_poly);

    // compute best value
    auto best_choice = compute_optimus(point_list, target_r, previous_turn);

    // draw target
    if(viewer != nullptr)
        draw_target(target_r, viewer->robot_poly(), &viewer->scene);

    draw(Eigen::Vector3f(0.f,0.f,0.f), point_list, best_choice, &viewer->scene);

    if (best_choice.has_value())
    {
        auto &[x, y, v, w, alpha]  = best_choice.value();  // x,y coordinates of best point, v,w velocities to reach that point, alpha robot's angle at that point
        // add control constraints
        w = std::clamp(w, -constants.max_rotation_velociy, constants.max_rotation_velociy);
        float k2 = 0.8;
        w = k2*w;
        float dist = target_r.norm();
        float dist_break = std::clamp(dist / 1000, 0.f, 1.f);
        v = constants.max_advance_velocity * dist_break * gaussian(w);
        previous_turn = w;
        return std::make_tuple(v, w, 0.f);
    }
    else
        return {};
}
void Dynamic_Window::draw_target(const Eigen::Vector2f &target_r, QGraphicsPolygonItem *robot_polygon, QGraphicsScene *scene)
{
    static QGraphicsRectItem *target_draw;
    if(target_draw != nullptr)
        scene->removeItem(target_draw);

    auto target_world = robot_polygon->mapToScene(QPointF(target_r.x(), target_r.y()));
    target_draw = scene->addRect(target_world.x()-100, target_world.y()-100, 200 , 200, QPen(QColor("Orange")), QBrush(QColor("Orange")));
}
float Dynamic_Window::gaussian(float x)
{
    const double xset = 0.4;
    const double yset = 0.4;
    const double s = -xset*xset/log(yset);
    return exp(-x*x/s);
}

std::vector<Dynamic_Window::Result> Dynamic_Window::compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly)
{
    std::vector<Result> list_points;

    for (float v = -100; v <= 800; v += 100) //advance
        for (float w = -2; w <= 2; w += 0.2) //rotation
        {
            float new_adv = current_adv + v;
            float new_rot = -current_rot + w;
            if (fabs(w) > 0.001)  // avoid division by zero to compute the radius
            {
                float r = new_adv / new_rot; // radio de giro ubicado en el eje x del robot
                float arc_length = new_rot * constants.time_ahead * r;
                for (float t = constants.step_along_arc; t < arc_length; t += constants.step_along_arc)
                {
                    float x = r - r * cos(t / r); float y= r * sin(t / r);  // circle parametric coordinates
                    auto point = std::make_tuple(x, y, new_adv, new_rot, t / r);
                    if(sqrt(x*x + y*y)> constants.robot_semi_width and point_reachable_by_robot(point, laser_poly)) // skip points in the robot
                        list_points.emplace_back(std::move(point));
                }
            }
            else // para evitar la división por cero en el cálculo de r
            {
                for(float t = constants.step_along_arc; t < new_adv * constants.time_ahead; t+=constants.step_along_arc)
                {
                    auto point = std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead);
                    if (t > constants.robot_semi_width and point_reachable_by_robot(point, laser_poly))
                        list_points.emplace_back(std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead));
                }
            }
        }
    return list_points;
}
//bool Dynamic_Window::point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly)
//{
//    auto [x, y, adv, giro, ang] = point;
//    float parts = Eigen::Vector2f(x,y).norm()/(constants.robot_semi_width/2);
//    for(const auto &l: iter::range(0.0, 1.0, 1.0/parts))
//    {
//        auto temp_robot = QTransform().rotate(ang).translate(x, y).map(polygon_robot);  // compute incremental rotation
//        if (auto res = std::find_if_not(std::begin(temp_robot), std::end(temp_robot),
//                                        [laser_poly](const auto &p) { return laser_poly.containsPoint(p, Qt::OddEvenFill); }); res != std::end(temp_robot))
//            return false;
//    }
//    return true;
//}

bool Dynamic_Window::point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly)
{
    auto [x, y, adv, giro, ang] = point;
    Eigen::Vector2f robot_r(0.0,0.0);
    Eigen::Vector2f goal_r(x, y);
    float parts = Eigen::Vector2f(x,y).norm()/(constants.robot_semi_width/6.0);
    Eigen::Vector2f rside(260, 100);
    Eigen::Vector2f lside(-260, 100);
    QPointF p,q,r;
    for(const auto &l: iter::range(0.0, 1.0, 1.0/parts))
    {
        p = to_qpointf(robot_r*(1-l) + goal_r*l);
        q = to_qpointf((robot_r+rside)*(1-l) + (goal_r+rside)*l);
        r = to_qpointf((robot_r+lside)*(1-l) + (goal_r+lside)*l);
        if( not laser_poly.containsPoint(p, Qt::OddEvenFill) or
            not laser_poly.containsPoint(q, Qt::OddEvenFill) or
            not laser_poly.containsPoint(r, Qt::OddEvenFill))
        return false;
    }
    return true;
}

std::optional<Dynamic_Window::Result> Dynamic_Window::compute_optimus(const std::vector<Result> &points, const Eigen::Vector2f &tr,
                                                                      float previous_turn)
{
    const float A=1, B=5;  // CHANGE
    std::vector<std::tuple<float, Result>> values(points.size());
    for(auto &&[k, point] : iter::enumerate(points))
    {
        auto [x, y, adv, giro, ang] = point;
        float dist_to_target = (Eigen::Vector2f(x, y) - tr).norm();
        float dist_to_previous_turn =  fabs(giro - previous_turn);
        values[k] = std::make_tuple(A*dist_to_target + B*dist_to_previous_turn, point);
    }
    auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
    if(min != values.end())
        return std::get<Result>(*min);
    else
        return {};
}
Eigen::Vector2f Dynamic_Window::from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot)
{
    Eigen::Matrix2f matrix;
    const float &angle = robot.z();
    matrix << cos(angle) , -sin(angle) , sin(angle) , cos(angle);
    return (matrix * p) + Eigen::Vector2f(robot.x(), robot.y());
}

Eigen::Vector2f Dynamic_Window::from_world_to_robot(const Eigen::Vector2f &p, const Eigen::Vector3f &robot)
{
    Eigen::Matrix2f matrix;
    const float &angle = robot.z();
    matrix << cos(angle) , -sin(angle) , sin(angle) , cos(angle);
    return (matrix.transpose() * (p - Eigen::Vector2f(robot.x(), robot.y())));
}

void Dynamic_Window::draw(const Eigen::Vector3f &robot, const std::vector <Result> &puntos,  const std::optional<Result> &best, QGraphicsScene *scene)
{
    static std::vector<QGraphicsEllipseItem *> arcs_vector;
    // remove current arcs
    for (auto arc: arcs_vector)
        scene->removeItem(arc);
    arcs_vector.clear();

    int radius = 30;
    QColor col("green");
    QPen pen(col, 10);
    QBrush brush(col);
    for (auto &[x, y, vx, wx, a] : puntos)
    {
        //QPointF centro = robot_polygon_draw->mapToScene(x, y);
        QPointF centro = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
        auto arc = scene->addEllipse(-radius, -radius, 2*radius, 2*radius, pen, brush);
        arc->setPos(centro.x(), centro.y());
        arcs_vector.push_back(arc);
    }

    if(best.has_value())
    {
        auto &[x, y, _, __, ___] = best.value();
        QPointF selected = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
        auto arc = scene->addEllipse(-50, -50, 100, 100, QPen(Qt::black), QBrush(Qt::black));
        arc->setPos(selected.x(), selected.y());
        arcs_vector.push_back(arc);
    }
}
