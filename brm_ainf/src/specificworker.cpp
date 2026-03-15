/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
#include "specificworker.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <QVBoxLayout>


const char *SpecificWorker::phase_to_cstr(RuntimePhase phase)
{
    switch (phase)
    {
        case RuntimePhase::BOOTSTRAPPING: return "BOOTSTRAPPING";
        case RuntimePhase::LOCALIZING: return "LOCALIZING";
        default: return "UNKNOWN";
    }
}

void SpecificWorker::set_phase(RuntimePhase new_phase)
{
    if (phase_ != new_phase)
        qInfo() << "Phase transition:" << phase_to_cstr(phase_) << "->" << phase_to_cstr(new_phase);

    phase_ = new_phase;

    if (phase_label_ != nullptr)
    {
        const bool boot = (phase_ == RuntimePhase::BOOTSTRAPPING);
        phase_label_->setText(QString("Phase: %1").arg(phase_to_cstr(phase_)));
        phase_label_->setStyleSheet(boot
            ? "QLabel { color: rgb(255,170,0); font-weight: 700; padding: 4px; }"
            : "QLabel { color: rgb(0,170,255); font-weight: 700; padding: 4px; }");
    }
}

void SpecificWorker::set_compound_score_label(const rc::RoomConceptAI::UpdateResult &res)
{
    if (score_label_ == nullptr)
        return;

    const float score = std::clamp(res.belief_score, 0.f, 1.f);
    const int pct = static_cast<int>(std::round(100.f * score));
    score_label_->setText(QString("Compound Score: %1%  |  p90=%2  reverse=%3")
        .arg(pct)
        .arg(res.sdf_p90, 0, 'f', 3)
        .arg(res.reverse_sdf, 0, 'f', 3));

    if (score >= params.INIT_BELIEF_MIN)
    {
        score_label_->setStyleSheet("QLabel { color: rgb(0,160,70); font-weight: 700; padding: 2px 4px; }");
    }
    else if (score >= 0.35f)
    {
        score_label_->setStyleSheet("QLabel { color: rgb(210,140,0); font-weight: 700; padding: 2px 4px; }");
    }
    else
    {
        score_label_->setStyleSheet("QLabel { color: rgb(210,40,40); font-weight: 700; padding: 2px 4px; }");
    }
}


SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
    stop_efe_hotzone_motion();
    set_bootstrap_rotation(false);
     // Stop background threads first
    room_ai.stop();
    stop_lidar_thread = true;
    if (read_lidar_th.joinable())
    {
        read_lidar_th.join();
    }
}

void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();

  // Lidar thread is created
    read_lidar_th = std::thread(&SpecificWorker::read_lidar,this);
    qInfo() << __FUNCTION__ << "Started lidar reader";


    // Configure localization context. The thread itself starts only after room bootstrap.
    {
        rc::RoomConceptAI::RunContext loc_ctx;
        loc_ctx.sensor_buffer = &lidar_buffer;
        loc_ctx.velocity_buffer = &velocity_buffer_;
        loc_ctx.odometry_buffer = &odometry_buffer_;
        room_ai.set_run_context(loc_ctx);
    }
    qInfo() << __FUNCTION__ << "RoomConceptAI context configured. Waiting for rectangular initialization";

    rc::RoomBootstrapper::Params bootstrap_params;
    bootstrap_params.min_frames = params.INIT_MIN_FRAMES;
    bootstrap_params.min_points = params.INIT_MIN_POINTS;
    bootstrap_params.span_scale = params.INIT_SPAN_SCALE;
    bootstrap_params.span_margin = params.INIT_SPAN_MARGIN;
    room_bootstrapper_.set_params(bootstrap_params);

    bootstrap_seeded_ = false;
    room_bootstrapped_ = false;
    bootstrap_accept_streak_ = 0;
    set_phase(RuntimePhase::BOOTSTRAPPING);

    if (viewer_2d_ == nullptr)
    {
        viewer_2d_ = std::make_unique<rc::Viewer2D>(this, params.GRID_MAX_DIM, true);
        viewer_2d_->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0.f, 0.f, QColor("blue"));

        QVBoxLayout *root_layout = qobject_cast<QVBoxLayout*>(this->layout());
        if (root_layout == nullptr)
        {
            root_layout = new QVBoxLayout(this);
            this->setLayout(root_layout);
        }

        if (phase_label_ == nullptr)
        {
            phase_label_ = new QLabel(this);
            root_layout->addWidget(phase_label_);
        }

        if (score_label_ == nullptr)
        {
            score_label_ = new QLabel(this);
            score_label_->setText("Compound Score: n/a");
            score_label_->setStyleSheet("QLabel { color: rgb(120,120,120); font-weight: 600; padding: 2px 4px; }");
            root_layout->addWidget(score_label_);
        }

        root_layout->addWidget(viewer_2d_->get_widget());
        viewer_2d_->show();

        // Force visible initial state even if phase did not transition.
        set_phase(phase_);
    }
}

void SpecificWorker::compute()
{
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // Read lidar
    const auto &[lidar_data_] = lidar_buffer.read(timestamp);
    if (!lidar_data_.has_value())
    { qWarning() << "No lidar data from lidar_buffer"; return; };

    // ===== BOOTSTRAP PHASE: seed model and wait for stable fit-quality acceptance =====
    if (!room_bootstrapped_)
    {
        const bool should_rotate = params.BOOTSTRAP_ENABLE_ROTATION &&
                                   (!params.BOOTSTRAP_ROTATE_ONLY_PRESEED || !bootstrap_seeded_);
        set_bootstrap_rotation(should_rotate);

        if (!bootstrap_seeded_)
        {
            const auto odom_snap = odometry_buffer_.get_snapshot<0>();
            if (!try_initialize_room_from_lidar(lidar_data_.value(), odom_snap))
                return;

            bootstrap_seeded_ = true;
            bootstrap_accept_streak_ = 0;

            if (!loc_thread_started_)
            {
                room_ai.start();
                loc_thread_started_ = true;
                qInfo() << "Localization thread started for bootstrap acceptance validation";
            }
            return;
        }

        auto res_opt = room_ai.get_last_result();
        const auto &bootstrap_points = room_bootstrapper_.accumulated_points();
        const auto &points_to_draw = bootstrap_points.empty() ? lidar_data_->first : bootstrap_points;
        if (!res_opt.has_value() || !res_opt->ok)
        {
            // Show seeded room hypothesis even before first localization result arrives.
            if (room_ai.is_initialized())
            {
                rc::RoomConceptAI::UpdateResult seeded_res;
                seeded_res.ok = true;
                seeded_res.state = room_ai.get_current_state();
                update_viewer(points_to_draw, seeded_res);
            }
            if (!room_ai.is_loc_initialized())
                qWarning() << "Waiting for localization thread...";
            return;
        }

        const auto &res = res_opt.value();
        update_viewer(points_to_draw, res);
        set_compound_score_label(res);

        const bool accepted_now =
            (res.belief_score >= params.INIT_BELIEF_MIN) &&
            (res.sdf_p90 <= params.INIT_SDF_P90_MAX) &&
            (res.reverse_sdf <= params.INIT_REVERSE_SDF_MAX);

        if (accepted_now)
            ++bootstrap_accept_streak_;
        else
            bootstrap_accept_streak_ = 0;

        static std::int64_t last_bootstrap_log_ms = 0;
        if (timestamp - last_bootstrap_log_ms >= 1000)
        {
            qInfo() << "Bootstrap quality: belief=" << res.belief_score
                    << " p90=" << res.sdf_p90
                    << " reverse=" << res.reverse_sdf
                    << " streak=" << bootstrap_accept_streak_ << "/" << params.INIT_ACCEPT_CONSECUTIVE;
            last_bootstrap_log_ms = timestamp;
        }

        if (bootstrap_accept_streak_ >= params.INIT_ACCEPT_CONSECUTIVE)
        {
            room_bootstrapped_ = true;
            set_bootstrap_rotation(false);
            set_phase(RuntimePhase::LOCALIZING);
            qInfo() << "Bootstrap accepted after" << bootstrap_accept_streak_ << "consecutive good-fit frames";
        }

        return;
    }

    if (!loc_thread_started_)
    {
        room_ai.start();
        loc_thread_started_ = true;
        set_phase(RuntimePhase::LOCALIZING);
    }

    set_bootstrap_rotation(false);

    // Read latest localization =====
    auto res_opt = room_ai.get_last_result();
    if (!res_opt.has_value() || !res_opt->ok)
    {
        if (!room_ai.is_loc_initialized())
            qWarning() << "Waiting for localization thread...";
        return;
    }
    const auto &res = res_opt.value();
    update_viewer(lidar_data_.value(), res);
    set_compound_score_label(res);

    static std::int64_t last_pose_log_ms = 0;
    static float last_sdf_mse = -1.f;
    if (timestamp - last_pose_log_ms >= 1000)
    {
        const float sdf_delta = (last_sdf_mse >= 0.f) ? (res.sdf_mse - last_sdf_mse) : 0.f;
        qInfo() << "Pose [x y theta]:"
                << res.state[2] << res.state[3] << res.state[4]
            << "| Room [w l]:" << res.state[0] << res.state[1]
                << "| SDF(median_abs,m):" << res.sdf_mse
                << "| SDF_p90(m):" << res.sdf_p90
                << "| reverseSDF(m):" << res.reverse_sdf
                << "| belief:" << res.belief_score
                << "| dSDF:" << sdf_delta;
        last_sdf_mse = res.sdf_mse;
        last_pose_log_ms = timestamp;
    }

    if (const auto bmr_opt = bmr_engine_.maybe_evaluate_from_lidar(
            lidar_data_->first,
            res.state,
            room_ai.params.enable_bmr,
            room_ai.params.bmr_check_period,
            room_ai.params.bmr_min_frames_before_check);
        bmr_opt.has_value())
    {
        const auto &b = *bmr_opt;
        qInfo() << "[BMR] check"
                << "logBF=" << b.log_bf_01
                << "z2=" << b.posterior_z_sq
                << "proposal=" << static_cast<int>(b.proposal)
                << "expand=" << b.expand
                << "side=" << b.indent_side
                << "segment=[" << b.indent_a << "," << b.indent_b << "]"
                << "depth=" << b.indent_depth
                << "latent p4..p9="
                << b.vertex_precision[4] << b.vertex_precision[5] << b.vertex_precision[6]
                << b.vertex_precision[7] << b.vertex_precision[8] << b.vertex_precision[9]
                << "active=" << b.activated_vertex_a << b.activated_vertex_b;

        if (params.ENABLE_EFE_HOTZONE_POLICY
            && !b.expand
            && b.proposal == rc::BmrResult::ProposalType::ADD_TWO_POINT_INDENT)
        {
            drive_toward_hotzone_efe(res, b);
        }
        else
        {
            stop_efe_hotzone_motion();
        }

        if (bmr_opt->expand)
        {
            stop_efe_hotzone_motion();
            room_ai.apply_bmr_result(*bmr_opt);
        }
    }
    else
    {
        stop_efe_hotzone_motion();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::update_viewer(const rc::LidarData &lidar_data, const rc::RoomConceptAI::UpdateResult &res)
{
    update_viewer(lidar_data.first, res);
}

void SpecificWorker::update_viewer(const std::vector<Eigen::Vector3f> &points, const rc::RoomConceptAI::UpdateResult &res)
{
    if (!viewer_2d_)
        return;

    // Update robot pose and estimated rectangular room.
    viewer_2d_->update_robot(res.state[2], res.state[3], res.state[4]);
    viewer_2d_->update_estimated_room_rect(res.state[0], res.state[1], false);

    // Also draw current rectangular estimate as explicit polygon overlay.
    {
        const float w = res.state[0];
        const float l = res.state[1];
        const std::vector<Eigen::Vector2f> room_poly = {
            {-w * 0.5f, -l * 0.5f},
            { w * 0.5f, -l * 0.5f},
            { w * 0.5f,  l * 0.5f},
            {-w * 0.5f,  l * 0.5f}
        };
        viewer_2d_->draw_room_polygon(room_poly, false);
    }

    auto &scene = viewer_2d_->scene();

    // Clear previous lidar items.
    for (auto *item : lidar_draw_items_)
    {
        scene.removeItem(item);
        delete item;
    }
    lidar_draw_items_.clear();

    // Draw decimated lidar points transformed to room/world frame.
    const auto &pts = points;
    const size_t stride = std::max<size_t>(1, pts.size() / std::max(1, params.MAX_LIDAR_DRAW_POINTS));

    const float x = res.state[2];
    const float y = res.state[3];
    const float phi = res.state[4];
    const float c = std::cos(phi);
    const float s = std::sin(phi);

    constexpr float r = 0.03f;
    for (size_t i = 0; i < pts.size(); i += stride)
    {
        const auto &p = pts[i];
        const float qx = c * p.x() - s * p.y() + x;
        const float qy = s * p.x() + c * p.y() + y;
        auto *dot = scene.addEllipse(-r, -r, 2.f * r, 2.f * r,
                                     QPen(QColor(30, 200, 30), 0.01),
                                     QBrush(QColor(30, 200, 30, 140)));
        dot->setPos(qx, qy);
        dot->setZValue(6);
        lidar_draw_items_.push_back(dot);
    }

    viewer_2d_->invalidate();
}


bool SpecificWorker::try_initialize_room_from_lidar(
    const rc::LidarData &lidar_data,
    const std::vector<rc::OdometryReading> &odometry_history)
{
    const auto bootstrap_opt = room_bootstrapper_.consume(
        lidar_data.first,
        lidar_data.second,
        odometry_history);
    if (!bootstrap_opt.has_value())
        return false;

    const auto bootstrap = bootstrap_opt.value();
    float init_phi = -bootstrap.obb_rotation;
    while (init_phi > static_cast<float>(M_PI)) init_phi -= static_cast<float>(2.0 * M_PI);
    while (init_phi < static_cast<float>(-M_PI)) init_phi += static_cast<float>(2.0 * M_PI);

    room_ai.set_initial_state(bootstrap.width, bootstrap.length, 0.f, 0.f, init_phi);
    const bool grid_ok = room_ai.grid_search_initial_pose(
        lidar_data.first,
        params.INIT_GRID_RESOLUTION,
        params.INIT_ANGLE_RESOLUTION);

    if (!grid_ok)
    {
        qWarning() << "Bootstrap grid-search did not find a confident initial pose. Re-trying with fresh LiDAR accumulation";
        room_bootstrapper_.reset();
        return false;
    }

    room_bootstrapped_ = false;
    bmr_engine_.reset_cycle_counter();

    qInfo() << "Room bootstrap ready. Rectangle (W,L)=" << bootstrap.width << bootstrap.length
            << " obb rotation=" << bootstrap.obb_rotation
            << " using" << bootstrap.points_used << "points in" << bootstrap.frames_used << "frames";

    room_bootstrapper_.reset();
    return true;
}

void SpecificWorker::set_bootstrap_rotation(bool enable)
{
    if (!params.BOOTSTRAP_ENABLE_ROTATION)
        enable = false;

    if (enable == bootstrap_rotating_)
        return;

    try
    {
        if (enable)
        {
            omnirobot_proxy->setSpeedBase(0.f, 0.f, params.BOOTSTRAP_ROT_SPEED);
            bootstrap_rotating_ = true;
        }
        else
        {
            omnirobot_proxy->stopBase();
            bootstrap_rotating_ = false;
        }
    }
    catch(const Ice::Exception &e)
    {
        qWarning() << "Bootstrap rotation command failed:" << e.what();
    }
}

void SpecificWorker::drive_toward_hotzone_efe(const rc::RoomConceptAI::UpdateResult &res, const rc::BmrResult &bmr)
{
    if (!params.ENABLE_EFE_HOTZONE_POLICY)
        return;

    const float w = std::max(0.5f, res.state[0]);
    const float l = std::max(0.5f, res.state[1]);
    const float hx = 0.5f * w;
    const float hy = 0.5f * l;
    const float tmid = 0.5f * (bmr.indent_a + bmr.indent_b);
    const float depth = std::max(0.05f, bmr.indent_depth);

    Eigen::Vector2f hot = Eigen::Vector2f::Zero();
    switch (bmr.indent_side)
    {
        case 0: hot = Eigen::Vector2f(tmid * hx, -hy + 0.5f * depth); break;
        case 1: hot = Eigen::Vector2f(hx - 0.5f * depth, tmid * hy); break;
        case 2: hot = Eigen::Vector2f(tmid * hx, hy - 0.5f * depth); break;
        case 3: hot = Eigen::Vector2f(-hx + 0.5f * depth, tmid * hy); break;
        default: return;
    }

    struct Action
    {
        float advx;
        float advz;
        float rot;
        const char *name;
    };

    const float vf = params.EFE_FORWARD_SPEED;
    const float wr = params.EFE_ROT_SPEED;
    const std::array<Action, 6> actions = {{
        {0.f,  vf,  0.f, "fwd"},
        {0.f,  0.f,  wr, "turn_left"},
        {0.f,  0.f, -wr, "turn_right"},
        {0.f,  0.8f * vf,  0.7f * wr, "arc_left"},
        {0.f,  0.8f * vf, -0.7f * wr, "arc_right"},
        {0.f,  0.f,  0.f, "hold"}
    }};

    const float x0 = res.state[2];
    const float y0 = res.state[3];
    const float phi0 = res.state[4];
    const float dt = std::max(0.1f, params.EFE_DT);
    const float sigma2 = std::max(1e-3f, params.EFE_PRIOR_SIGMA * params.EFE_PRIOR_SIGMA);

    auto wrap_pi = [](float a) -> float
    {
        while (a > static_cast<float>(M_PI)) a -= static_cast<float>(2.0 * M_PI);
        while (a < static_cast<float>(-M_PI)) a += static_cast<float>(2.0 * M_PI);
        return a;
    };

    float best_g = std::numeric_limits<float>::infinity();
    Action best = actions.back();

    for (const auto &a : actions)
    {
        const float c = std::cos(phi0);
        const float s = std::sin(phi0);
        const float x1 = x0 + (a.advx * c - a.advz * s) * dt;
        const float y1 = y0 + (a.advx * s + a.advz * c) * dt;
        const float phi1 = wrap_pi(phi0 + a.rot * dt);

        const float dx = x1 - hot.x();
        const float dy = y1 - hot.y();
        const float risk = 0.5f * (dx * dx + dy * dy) / sigma2;

        const float heading = std::atan2(hot.y() - y1, hot.x() - x1);
        const float h_err = wrap_pi(heading - phi1);
        const float ambiguity = params.EFE_ANGLE_WEIGHT * h_err * h_err;

        const float effort = params.EFE_CONTROL_WEIGHT *
                             (a.advx * a.advx + a.advz * a.advz + 0.5f * a.rot * a.rot);

        const float g = risk + ambiguity + effort;
        if (g < best_g)
        {
            best_g = g;
            best = a;
        }
    }

    try
    {
        omnirobot_proxy->setSpeedBase(best.advx, best.advz, best.rot);
        efe_hotzone_active_ = true;

        static std::int64_t last_efe_log_ms = 0;
        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        if (now_ms - last_efe_log_ms >= 1000)
        {
            qInfo() << "[EFE] hot-zone target=" << hot.x() << hot.y()
                    << " action=" << best.name
                    << " cmd=" << best.advx << best.advz << best.rot
                    << " G=" << best_g;
            last_efe_log_ms = now_ms;
        }
    }
    catch(const Ice::Exception &e)
    {
        qWarning() << "EFE hot-zone command failed:" << e.what();
    }
}

void SpecificWorker::stop_efe_hotzone_motion()
{
    if (!efe_hotzone_active_)
        return;

    try
    {
        omnirobot_proxy->stopBase();
    }
    catch(const Ice::Exception &e)
    {
        qWarning() << "Stop EFE motion failed:" << e.what();
    }
    efe_hotzone_active_ = false;
}

void SpecificWorker::read_lidar()
{
    auto wait_period = std::chrono::milliseconds (getPeriod("Compute"));
    while(!stop_lidar_thread)
    {
        const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        try
        {
            const float body_offset_sq = params.ROBOT_SEMI_WIDTH * params.ROBOT_SEMI_WIDTH;

            // ---- Launch both lidar requests in parallel (async Ice) — HELIOS ----
            std::future<RoboCompLidar3D::TData> future_high;
            try
            {
                future_high = lidar3d_proxy->getLidarDataWithThreshold2dAsync(
                        params.LIDAR_NAME_HIGH,
                        params.MAX_LIDAR_HIGH_RANGE * 1000.f,
                        params.LIDAR_LOW_DECIMATION_FACTOR);
            }
            catch (const Ice::Exception &e)
            { qWarning() << "[read_lidar] HELIOS async launch failed:" << e.what(); }

            // ---- Wait and process HELIOS ----
            const auto data_high = future_high.get();
            std::vector<Eigen::Vector3f> points_high;
            points_high.reserve(data_high.points.size());
            for (const auto &p : data_high.points)
            {
                const float pmx = p.x / 1000.f;
                const float pmy = p.y / 1000.f;
                const float pmz = p.z / 1000.f;
                if (pmx*pmx + pmy*pmy > body_offset_sq and pmz < params.LIDAR_HIGH_MAX_HEIGHT and pmz > params.LIDAR_HIGH_MIN_HEIGHT)
                        points_high.emplace_back(pmx, pmy, pmz);
            }
            lidar_buffer.put<0>(std::make_pair(std::move(points_high), static_cast<std::int64_t>(data_high.timestamp)), static_cast<size_t>(timestamp));

            // Adjust period with hysteresis
            if (wait_period > std::chrono::milliseconds((long) data_high.period + 2)) --wait_period;
            else if (wait_period < std::chrono::milliseconds((long) data_high.period - 2)) ++wait_period;
            std::this_thread::sleep_for(wait_period);
        }
        catch (const Ice::Exception &e)
        { qWarning() << "Error reading from Lidar3D or robot pose:" << e.what(); }
    }
} // Thread to read the lidar

///////////////////////////////////////////////////////////////////////////////////////
//SUBSCRIPTION to newFullPose method from FullPoseEstimationPub interface
///////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)
{
      // Add configurable Gaussian noise to simulate realistic odometry uncertainty
    static std::mt19937 gen{std::random_device{}()};
    const float nf = params.ODOMETRY_NOISE_FACTOR;

    auto add_noise = [&](float value) -> float
    {
        if (nf <= 0.f || value == 0.f) return value;
        std::normal_distribution<float> dist(0.f, std::abs(value) * nf);
        return value + dist(gen);
    };

    rc::OdometryReading odom;
    odom.adv  = add_noise(pose.adv);    // forward velocity, m/s
    odom.side = add_noise(pose.side);   // lateral velocity, m/s
    odom.rot  = add_noise(pose.rot);    // angular velocity, rad/s
    odom.timestamp = std::chrono::high_resolution_clock::now();
    const auto ts = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    odometry_buffer_.put<0>(std::move(odom), ts);
}

////////////////////////////////////////////////////////////////////////////////////////7
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    // Stop the robot
    try {
        omnirobot_proxy->stopBase();
    } catch(const Ice::Exception &e) {
        std::cout << "Could not stop robot in emergency: " << e << std::endl;
    }
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams
