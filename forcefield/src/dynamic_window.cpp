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

std::tuple<float, float, float> Dynamic_Window::update(const Eigen::Vector3f &target_r,
                                                       const std::vector<Eigen::Vector2f> &ldata, // x,y
                                                       float current_adv, float current_rot,
                                                       AbstractGraphicViewer *viewer)
{
    // check limits
    current_adv  = std::clamp(current_adv,  constants.min_advance_velocity, constants.max_advance_velocity);
    current_rot  = std::clamp(current_rot,  -constants.max_rotation_velociy, constants.max_advance_velocity);

    // if zerov target vector, stop de robot
    if(target_r.isApprox(Eigen::Vector3f{0.f, 0.f, 0.f}))
        return  std::make_tuple(0.f, 0.f, 0.f);

    if(target_r.z() !=0 ) // only rotation
        return std::make_tuple(0.f, target_r.z(), 0.f);

    // compute future positions of the robot
    QPolygonF laser_poly;
    for(const auto &l : ldata)
        laser_poly << QPointF(l.x(), l.y());

    auto t1 = clock.now();
    auto point_list = compute_predictions(current_adv, current_rot, laser_poly);

    auto t2 = clock.now();

    // compute best value
    auto best_choice = compute_optimus(point_list, target_r, ldata);

    auto t3 = clock.now();
    //qInfo() << __FUNCTION__ << t2-t1 << t3-t2;

    // draw target
    if(viewer != nullptr)
    {
        draw_target(target_r, viewer->robot_poly(), &viewer->scene);
        //draw_polygon(laser_poly, &viewer->scene);
    }

    draw(Eigen::Vector3f(0.f,0.f,0.f), point_list, best_choice, &viewer->scene);

    if (best_choice.has_value())
    {
        auto &[x, y, v, w, alpha]  = best_choice.value();  // x,y coordinates of best point, v,w velocities to reach that point, alpha robot's angle at that point
        // add control constraints
        //w = std::clamp(w, -constants.max_rotation_velociy, constants.max_rotation_velociy);
        //float dist = target_r.norm();
        //float dist_break = std::clamp(dist / 1000, 0.f, 1.f);
        //v = constants.max_advance_velocity * dist_break * gaussian(w);
        return std::make_tuple(v, w, 0.f);
    }
    else
        return {};
}
std::vector<Dynamic_Window::Result> Dynamic_Window::compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly)
{
    std::vector<Result> list_points;
    // given advance acceleration, adv_a, in one second the robot will pass from current_speed to current_speed + adv_a
    float max_reachable_adv_speed =  constants.adv_accel * constants.time_ahead;
    float max_reachable_rot_speed =  constants.rot_accel * constants.time_ahead;
    for(float v=constants.min_advance_velocity; v<=max_reachable_adv_speed; v+=constants.adv_step)
        for(float w=-max_reachable_rot_speed; w<=max_reachable_rot_speed; w+=constants.rot_step)
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
            else
            {
                for(float t = constants.step_along_arc; t < new_adv * constants.time_ahead; t+=constants.step_along_arc)
                {
                    auto point = std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead);
                    if (t > constants.robot_semi_width and point_reachable_by_robot(point, laser_poly))
                        list_points.emplace_back(std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead));
                }
            }
        }
    if(list_points.empty())
        qWarning() << __FUNCTION__ << "Empty list of points";
    return list_points;
}
bool Dynamic_Window::point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly)
{
    // checks if three points in the center, left and right of the robot, moved between the robot  and the target, are all contained inside laser_poly
    auto [x, y, adv, giro, ang] = point;
    //Eigen::Vector2f goal_r(x, y);
    //float parts = Eigen::Vector2f(x,y).norm()/(constants.robot_semi_width/3.0);
    // 2 points on the sides and three points on the front
    Eigen::Vector3f center(0.0, constants.robot_semi_width * 1.5, 1.0);
    Eigen::Vector3f rs(constants.robot_semi_width, 0.f, 1.f);
    Eigen::Vector3f ls(-constants.robot_semi_width, 0.f, 1.f);
    Eigen::Vector3f rside(constants.robot_semi_width, constants.robot_semi_width * 1.5, 1.f);
    Eigen::Vector3f lside(-constants.robot_semi_width, constants.robot_semi_width * 1.5, 1.f);

    // check if it fits at the final position
    Eigen::Matrix3f rt;
    rt << cos(giro) , -sin(giro) , x, sin(giro) , cos(giro), y, 0.f, 0.f, 1.f;
    QPointF c = to_qpointf((rt * center).head(2));
    QPointF l = to_qpointf((rt * lside).head(2));
    QPointF r = to_qpointf((rt * rside).head(2));
    QPointF l1 = to_qpointf((rt * rs).head(2));
    QPointF r1 = to_qpointf((rt * ls).head(2));

    if( not laser_poly.containsPoint(c, Qt::OddEvenFill) or
        not laser_poly.containsPoint(r, Qt::OddEvenFill) or
        not laser_poly.containsPoint(l, Qt::OddEvenFill) or
        not laser_poly.containsPoint(l1, Qt::OddEvenFill) or
        not laser_poly.containsPoint(r1, Qt::OddEvenFill)
    )
          return false;

    // check if it fits at the middle position of the chord, not the arc as it should be
//    Eigen::Matrix3f rtm;
//    rtm << cos(giro/2) , -sin(giro/2) , x/2, sin(giro/2) , cos(giro/2), y/2, 0.f, 0.f, 1.f;
//    QPointF cm = to_qpointf((rt * center).head(2));
//    QPointF lm = to_qpointf((rt * lside).head(2));
//    QPointF rm = to_qpointf((rt * rside).head(2));
//
//    if( not laser_poly.containsPoint(cm, Qt::OddEvenFill) or
//        not laser_poly.containsPoint(rm, Qt::OddEvenFill) or
//        not laser_poly.containsPoint(lm, Qt::OddEvenFill))
//        return false;

//    for(const auto &l: iter::range(0.0, 1.0, 1.0/parts))

    return true;
}
std::optional<Dynamic_Window::Result> Dynamic_Window::compute_optimus(const std::vector<Result> &points, const Eigen::Vector3f &tr,
                                                                      const std::vector<Eigen::Vector2f> &dist_line)
{
    static float previous_turn = 0;
    const float A=5, B=0.1, C=0.01;  // CHANGE
    std::vector<std::tuple<float, Result>> values(points.size());
    for(auto &&[k, point] : iter::enumerate(points))
    {
        auto [x, y, adv, giro, ang] = point;
        float dist_to_target = (Eigen::Vector2f(x, y) - tr.head(2)).norm() / tr.head(2).norm();  // normalized to 1
        float dist_to_previous_turn =  fabs(giro - previous_turn) / M_2_PI;                      // normalized to 1
        Eigen::Vector2f dist_to_obstacle = std::ranges::min(dist_line, [x, y](auto &a, auto &b){ Eigen::Vector2f p{x,y}; return (p-a).norm() < (p-b).norm();});
        float dist_o = dist_to_obstacle.norm();
        if(dist_o > 800)
            dist_o = 0.f;
        else
            dist_o = 1.f/ (dist_o/1000.f);
        values[k] = std::make_tuple(A*dist_to_target + B*dist_to_previous_turn + C*dist_to_obstacle.norm(), point);
    }
    auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
    if(min != values.end())
    {
        previous_turn = std::get<3>(std::get<Result>(*min));
        return std::get<Result>(*min);
    }
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

    QColor col("green");
    for (auto &[x, y, vx, wx, a] : puntos)
    {
        //QPointF centro = robot_polygon_draw->mapToScene(x, y);
        QPointF centro = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
        auto arc = scene->addEllipse(centro.x(), centro.y(), 50, 50, QPen(col, 10));
        arc->setZValue(30);
        arcs_vector.push_back(arc);
    }

    if(best.has_value())
    {
        auto &[x, y, _, __, ___] = best.value();
        QPointF selected = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
        auto arc = scene->addEllipse(selected.x(), selected.y(), 180, 180, QPen(Qt::black), QBrush(Qt::black));
        arc->setZValue(30);
        arcs_vector.push_back(arc);
    }
}
void Dynamic_Window::draw_target(const Eigen::Vector3f &target_r, QGraphicsPolygonItem *robot_polygon, QGraphicsScene *scene)
{
    static QGraphicsRectItem *target_draw;
    if(target_draw != nullptr)
    {
        scene->removeItem(target_draw);
        delete target_draw;
    }

    auto target_world = robot_polygon->mapToScene(QPointF(target_r.x(), target_r.y()));
    target_draw = scene->addRect(target_world.x()-100, target_world.y()-100, 200 , 200, QPen(QColor("Orange")), QBrush(QColor("Orange")));
}
void Dynamic_Window::draw_polygon(const QPolygonF &poly, QGraphicsScene *scene)
{
    static QGraphicsItem *item;
    if(item != nullptr)
    {
        scene->removeItem(item);
        delete item;
    }
    item = scene->addPolygon(poly, QPen(QColor("orange"), 25));
}
float Dynamic_Window::gaussian(float x)
{
    const double xset = 0.5;
    const double yset = 0.3;
    const double s = -xset*xset/log(yset);
    return exp(-x*x/s);
}
