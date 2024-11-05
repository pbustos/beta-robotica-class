//
// Created by pbustos on 3/04/22.
//

#include "mpc.h"
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/chunked.hpp>
#include <QGraphicsScene>
#include <QPen>
#include <QBrush>

namespace rc
{
    casadi::Opti MPC::initialize_differential(const int N)
    {
        consts.num_steps = N;
        casadi::Slice all;
        this->opti = casadi::Opti();
        auto specific_options = casadi::Dict();
        auto generic_options = casadi::Dict();
        //specific_options["accept_after_max_steps"] = 100;
        specific_options["fixed_variable_treatment"] = "relax_bounds";
        //specific_options["print_time"] = 0;
        //specific_options["max_iter"] = 10000;
        specific_options["print_level"] = 0;
        specific_options["acceptable_tol"] = 1e-8;
        specific_options["acceptable_obj_change_tol"] = 1e-6;
        opti.solver("ipopt", generic_options, specific_options);

        // ---- state variables ---------
        state = opti.variable(3, N+1);
        pos = state(casadi::Slice(0,2), all);
        phi = state(2, all);

        // ---- inputs variables 2 adv and rot---------
        control = opti.variable(2, N);
        adv = control(0, all);
        rot = control(1, all);

        // Gap closing: dynamic constraints for differential robot: dx/dt = f(x, u)   3 x 2 * 2 x 1 -> 3 x 1
        auto integrate = [](casadi::MX x, casadi::MX u) { return casadi::MX::mtimes(
                casadi::MX::vertcat(std::vector<casadi::MX>{
                        casadi::MX::horzcat(std::vector<casadi::MX>{casadi::MX::sin(x(2)), 0.0}),
                        casadi::MX::horzcat(std::vector<casadi::MX>{casadi::MX::cos(x(2)), 0.0}),
                        casadi::MX::horzcat(std::vector<casadi::MX>{0.0,      1.0})}
                ), u);};
        double dt = 0.1;   // timer interval in secs
        for(const auto k : iter::range(N))  // loop over control intervals
        {
            auto k1 = integrate(state(all, k), control(all,k));
            auto k2 = integrate(state(all,k) + (dt/2)* k1 , control(all, k));
            auto k3 = integrate(state(all,k) + (dt/2)*k2, control(all, k));
            auto k4 = integrate(state(all,k) + k3, control(all, k));
            auto x_next = state(all, k) + dt / 6 * (k1 + 2*k2 + 2*k3 + k4);
            //auto x_next = state(all, k) + dt * integrate(state(all,k), control(all,k));
            opti.subject_to( state(all, k + 1) == x_next);  // close  the gaps
        }
        for(const auto k : iter::range(N-1))  // acceleration constraints
        {
            auto v1 = control(0,k);
            auto v2 = control(0,k+1);
            auto acc = (v2-v1)/dt;
            //auto x_next = state(all, k) + dt * integrate(state(all,k), control(all,k));
            opti.subject_to(opti.bounded(-3.19, acc, 3.19)); 
            auto w1 = control(1,k);
            auto w2 = control(1,k+1);
            auto ang_acc = (w2-w1)/dt;
            opti.subject_to(opti.bounded(-0.5, ang_acc, 0.5)); 
        }


        // control constraints -----------
        opti.subject_to(opti.bounded(consts.min_advance_value, adv, consts.max_advance_value));  // control is limited meters
        opti.subject_to(opti.bounded(-consts.max_rotation_value, rot, consts.max_rotation_value));         // control is limited

        // forward velocity constraints -----------
        //opti.subject_to(adv >= 0);

        // initial point constraints ------
        //    auto initial_oparam = opti.parameter(3);
        //    opti.set_value(initial_oparam, std::vector<double>{0.0, 0.0, 0.0});
        opti.subject_to(state(all, 0) == std::vector<double>{0.0, 0.0, 0.0});

        // slack vector declaration
        slack_vector = opti.variable(consts.num_steps);

        return opti;
    };
    casadi::Opti MPC::initialize_omni(const int N)
    {
        consts.num_steps = N;
        casadi::Slice all;
        this->opti = casadi::Opti();
        auto specific_options = casadi::Dict();
        auto generic_options = casadi::Dict();
        //specific_options["accept_after_max_steps"] = 100;
        specific_options["fixed_variable_treatment"] = "relax_bounds";
        //specific_options["print_time"] = 0;
        //specific_options["max_iter"] = 10000;
        specific_options["print_level"] = 0;
        specific_options["acceptable_tol"] = 1e-8;
        specific_options["acceptable_obj_change_tol"] = 1e-6;
        generic_options["verbose"] = false;  // Minimize verbosity
        //specific_options["ipopt.print_level"] = 0;  // Specific to Ipopt solver
        opti.solver("ipopt", generic_options, specific_options);

        // ---- state variables ---------
        state = opti.variable(3, N+1);
        pos = state(casadi::Slice(0,2), all);
        phi = state(2, all);

        // ---- inputs variables 3 adv, side and rot---------
        control = opti.variable(3, N);
        adv = control(0, all);
        side = control(1, all);
        rot = control(2, all);

        // Gap closing: dynamic constraints for omni robot: dx/dt = f(x, u)   3 x 3 * 3 x 1 -> 3 x 1
        auto integrate = [](const casadi::MX &x, const casadi::MX &u)
                { return casadi::MX::mtimes(
                          casadi::MX::vertcat(std::vector<casadi::MX>
                             {
                                casadi::MX::horzcat(std::vector<casadi::MX>{0.0, 1.0, 0.0}),
                                casadi::MX::horzcat(std::vector<casadi::MX>{1.0, 0.0, 0.0}),
                                casadi::MX::horzcat(std::vector<casadi::MX>{0.0, 0.0, 1.0})
                             }), u);};

        double dt = consts.time_interval;   // integration timer interval in secs
        // Runge-Kutta integration over control intervals
        for(const auto k : iter::range(N))
        {
            auto k1 = integrate(state(all, k), control(all,k));
            auto k2 = integrate(state(all,k) + (dt/2)* k1 , control(all, k));
            auto k3 = integrate(state(all,k) + (dt/2)*k2, control(all, k));
            auto k4 = integrate(state(all,k) + k3, control(all, k));
            auto x_next = state(all, k) + dt / 6 * (k1 + 2*k2 + 2*k3 + k4);
            //auto x_next = state(all, k) + dt * integrate(state(all,k), control(all,k)); // Euler
            opti.subject_to( state(all, k + 1) == x_next);  // close  the gaps
        }

        // acceleration constraints
        for(const auto k : iter::range(N-1))
        {
            // adv
            auto v1 = control(0,k);
            auto v2 = control(0,k+1);
            auto acc = (v2-v1)/dt;
            //auto x_next = state(all, k) + dt * integrate(state(all,k), control(all,k));
            //opti.subject_to(opti.bounded(-3.19, acc, 3.19));
            opti.subject_to(opti.bounded(-0.5, acc, 0.5));

            // side
            auto s1 = control(1,k);
            auto s2 = control(1,k+1);
            auto sacc = (s2-s1)/dt;
            //auto x_next = state(all, k) + dt * integrate(state(all,k), control(all,k));
            opti.subject_to(opti.bounded(-0.2, sacc, 0.2));

            // rot
            auto w1 = control(2,k);
            auto w2 = control(2,k+1);
            auto ang_acc = (w2-w1)/dt;
            opti.subject_to(opti.bounded(-0.5, ang_acc, 0.5));
        }

        // control constraints -----------
        opti.subject_to(opti.bounded(consts.min_advance_value, adv, consts.max_advance_value));     // control is limited meters
        opti.subject_to(opti.bounded(consts.min_side_value, side, consts.max_side_value));     // control is limited meters
        opti.subject_to(opti.bounded(consts.min_rotation_value, rot, consts.max_rotation_value));  // control is limited

        // forward velocity constraints -----------
        //opti.subject_to(adv >= 0);

        opti.subject_to(state(all, 0) == std::vector<double>{0.0, 0.0, 0.0}); //TODO: check if this is needed
        return opti;
    };
    std::optional<std::pair<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector2f>>>  // control and state vectors
    MPC::update(const std::vector<Eigen::Vector2f> &path_robot, const std::vector<Eigen::Vector2f> &obstacles)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // transform path to meters
        std::vector<Eigen::Vector2f> path_robot_meters(path_robot.size());
        for(const auto &[i, p] : path_robot | iter::enumerate)
            path_robot_meters[i] = p / 1000.f;
        
        // target in robot RS in meters
        auto target_robot = path_robot_meters.back();

        auto opti_local = this->opti.copy();
        casadi::Slice all;

        // Warm start
//        if (previous_values_of_solution.empty())
//        {
//            previous_values_of_solution.resize(consts.num_steps+1);
//            double landa = 1.0 / (target_robot.norm() / consts.num_steps);
//            for (auto &&[i, step]: iter::range(0.0, 1.0, landa) | iter::enumerate)
//            {
//                auto paso = target_robot * step;
//                previous_values_of_solution[3 * i] = paso.x();
//                previous_values_of_solution[3 * i + 1] = paso.y();
//                previous_values_of_solution[3 * i + 2] = 0.0;
//            }
//        }

        /// cost function as sum of several terms weighted by a factor

        // minimze distance to each element of path
        auto sum_dist_path = opti_local.parameter();
        opti_local.set_value(sum_dist_path, 0.0);
        for (auto k: iter::range(consts.num_steps))
            sum_dist_path += casadi::MX::sumsqr(pos(all, k) - e2v(path_robot_meters[k].cast<double>()));

        // maximize distance from pos to the closest obstacle, if any

        auto sum_dist_obstacle = opti_local.parameter();
        opti_local.set_value(sum_dist_obstacle, 0.0);
        for (auto k: iter::range(consts.num_steps))
        {
            auto dist = opti_local.parameter();
            opti_local.set_value(dist, std::numeric_limits<float>::max());
            for (auto &&o: obstacles)
                dist = casadi::MX::fmin(dist, casadi::MX::sumsqr(pos(all, k) - e2v(o.cast<double>())));
            sum_dist_obstacle += dist;
            // to minimize the distance to the closest obstacle, we maximize the inverse
        }

        // minimze sum of rotations
                auto sum_rot = opti_local.parameter();
                opti_local.set_value(sum_rot, 0.0);
                for (auto k: iter::range(consts.num_steps))
                    sum_rot += casadi::MX::sumsqr(rot(k));

        // minimze angle of robot's nose wrt to next point in path
        auto sum_angle = opti_local.parameter();
        opti_local.set_value(sum_angle, 0.0);
        for (auto k: iter::range(consts.num_steps-1))
            sum_angle += casadi::MX::sumsqr(phi(k) - atan2(path_robot_meters[k+1].x(), path_robot_meters[k+1].y()));

        // minimize sum of distances to target
        auto sum_dist_target = opti_local.parameter();
        opti_local.set_value(sum_dist_target, 0.0);
        for (auto k: iter::range(consts.num_steps))
            sum_dist_target += casadi::MX::sumsqr(pos(all, k) - e2v(target_robot.cast<double>()));

        opti_local.minimize( sum_dist_path +
                             sum_angle * 8 +
                             sum_dist_target * 0.05 +
                             2.0/(sum_dist_obstacle+0.0001)
                             + sum_rot);

        // solve NLP ------
        try
        {
            auto solution = opti_local.solve();
            std::string retstat = solution.stats().at("return_status");
            if (retstat != "Solve_Succeeded")
            {
                std::cout << "NOT succeeded" << std::endl;
                return {};
            }
            //previous_values_of_solution = std::vector<double>(solution.value(state));
            //previous_control_of_solution = std::vector<double>(solution.value(control));

            // print output -----
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
            qInfo() << __FUNCTION__ << "Iterations: " << (int) solution.stats()["iter_count"];
            qInfo() << __FUNCTION__ << "Status: " << QString::fromStdString(solution.stats().at("return_status"));
            qInfo() << __FUNCTION__ << "Steps: " << consts.num_steps << path_robot.size();

            // extract control vector   TODO: CHECK IF omni or differential
            std::vector<Eigen::Vector3f> control_vector(consts.num_steps);
            const auto &advs = std::vector<double>(solution.value(adv));
            //const auto &sides = std::vector<double>(solution.value(side));
            const auto &rots = std::vector<double>(solution.value(rot));
            //qDebug() << __FUNCTION__ << "Control vector size: " << advs.size() << rots.size();
            for(auto &&i: iter::range(consts.num_steps))
                control_vector[i] = Eigen::Vector3f{static_cast<float>(advs.at(i)*1000.f),  // move back to mm
                                                    /*static_cast<float>(sides.at(i)*1000.f),*/
                                                    0.f,
                                                    static_cast<float>(rots.at(i))};
            // extract path vector
            std::vector<Eigen::Vector2f> path_vector; path_vector.reserve(consts.num_steps);
            const auto &state_sol = std::vector<double>(solution.value(pos));
            for(auto &&p: iter::chunked(state_sol, 2))
                path_vector.emplace_back(p[0]*1000.f,p[1]*1000.f);

            double acum = 0.0;
            for(const auto &[i, p]: path_vector | iter::enumerate)
                acum += (p - path_robot[i]).norm();
            qInfo() << __FUNCTION__ << " Error " << acum;

            return std::make_pair(control_vector, path_vector);
        }
        catch (const casadi::CasadiException& e)
        {
            std::cout << "CasADi exception caught: " << e.what() << std::endl;
            //previous_values_of_solution.clear();
            //previous_control_of_solution.clear();
            return {};
        }
    }
    ////////////////////// AUX /////////////////////////////////////////////////
    float MPC::gaussian(float x)
    {
        const double xset = consts.xset_gaussian;
        const double yset = consts.yset_gaussian;
        const double s = -xset*xset/log(yset);
        return exp(-x*x/s);
    }
    std::vector<double> MPC::e2v(const Eigen::Vector2d &v)
    {
        return std::vector<double>{v.x(), v.y()};
    }
    void MPC::draw_path(const std::vector<double> &path_robot_meters, QGraphicsPolygonItem *robot_polygon, QGraphicsScene *scene)
    {
        // draw optimum N points solution
        static std::vector<QGraphicsItem *> path_paint;
        static QString path_color = "orange";

        for(auto p : path_paint)
            scene->removeItem(p);
        path_paint.clear();

        uint s = 100;
        for(auto &&p : path_robot_meters | iter::chunked(3))
        {
            auto pw = robot_polygon->mapToScene(QPointF(p[0]*1000.f, p[1]*1000.f));
            path_paint.push_back(scene->addEllipse(pw.x()-s/2, pw.y()-s/2, s , s, QPen(path_color), QBrush(QColor(path_color))));
            path_paint.back()->setZValue(30);
        }
    }

} // mpc