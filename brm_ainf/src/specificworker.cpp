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

// ---------------------------------------------------------------------------
// Helper: build the preview polygon for a BMR indent candidate.
// Coordinates are in the room-centred frame (same as draw_room_polygon).
// ---------------------------------------------------------------------------
static std::vector<Eigen::Vector2f>
indent_candidate_polygon(const rc::BmrResult::IndentCandidateInfo& c, float hx, float hy)
{
    using PT = rc::BmrResult::ProposalType;
    const float depth = std::clamp(c.depth, 0.1f, 0.8f * std::min(hx, hy));

    if (c.proposal == PT::ADD_TWO_POINT_INDENT && c.side >= 0 && c.side <= 3)
    {
        const float a = std::clamp(std::min(c.a, c.b), -0.95f, 0.95f);
        const float b = std::clamp(std::max(c.a, c.b), -0.95f, 0.95f);
        switch (c.side)
        {
            case 0: { float x1=a*hx, x2=b*hx; return {{-hx,-hy},{x1,-hy},{x1,-hy+depth},{x2,-hy+depth},{x2,-hy},{hx,-hy},{hx,hy},{-hx,hy}}; }
            case 1: { float y1=a*hy, y2=b*hy; return {{-hx,-hy},{hx,-hy},{hx,y1},{hx-depth,y1},{hx-depth,y2},{hx,y2},{hx,hy},{-hx,hy}}; }
            case 2: { float x1=a*hx, x2=b*hx; return {{-hx,-hy},{hx,-hy},{hx,hy},{x2,hy},{x2,hy-depth},{x1,hy-depth},{x1,hy},{-hx,hy}}; }
            case 3: default: { float y1=a*hy, y2=b*hy; return {{-hx,-hy},{hx,-hy},{hx,hy},{-hx,hy},{-hx,y2},{-hx+depth,y2},{-hx+depth,y1},{-hx,y1}}; }
        }
    }
    else if (c.proposal == PT::ADD_CORNER_INDENT && c.corner >= 0 && c.corner <= 3)
    {
        // 2-DOF corner: independent dx (horizontal) and dy (vertical).
        const float dx = (c.depth_x > 0.01f || c.depth_y > 0.01f)
            ? std::clamp(c.depth_x, 0.1f, 0.8f * hx)
            : depth;
        const float dy = (c.depth_x > 0.01f || c.depth_y > 0.01f)
            ? std::clamp(c.depth_y, 0.1f, 0.8f * hy)
            : depth;
        switch (c.corner)
        {
            case 0: return {{-hx+dx,-hy},{hx,-hy},{hx,hy},{-hx,hy},{-hx,-hy+dy},{-hx+dx,-hy+dy}}; // BL
            case 1: return {{-hx,-hy},{hx-dx,-hy},{hx-dx,-hy+dy},{hx,-hy+dy},{hx,hy},{-hx,hy}};    // BR
            case 2: return {{-hx,-hy},{hx,-hy},{hx,hy-dy},{hx-dx,hy-dy},{hx-dx,hy},{-hx,hy}};      // TR
            case 3: default: return {{-hx,-hy},{hx,-hy},{hx,hy},{-hx+dx,hy},{-hx+dx,hy-dy},{-hx,hy-dy}}; // TL
        }
    }
    return {};
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

        // Named widgets are provided by the .ui file via Ui_guiDlg.
        phase_label_ = phaseLabel;
        score_label_ = scoreLabel;
        score_label_->setText("Compound Score: n/a");
        score_label_->setStyleSheet("QLabel { color: rgb(120,120,120); font-weight: 600; padding: 2px 4px; }");

        // Add the 2D viewer into the left frame of the splitter.
        auto *vf_layout = new QVBoxLayout(viewerFrame);
        vf_layout->setContentsMargins(0, 0, 0, 0);
        vf_layout->addWidget(viewer_2d_->get_widget());

        // Style the bottom candidates panel.
        candidatesTitleLabel->setStyleSheet(
            "QLabel { font-weight: bold; font-size: 11pt; padding: 4px; border-bottom: 1px solid #555; }");
        winnerLabel->setStyleSheet(
            "QLabel { font-weight: bold; color: rgb(50, 200, 50); padding: 4px; border-top: 1px solid #555; }");
        candidateListWidget->setStyleSheet(
            "QListWidget { background: #1a1a1a; color: #dddddd; font-family: monospace; font-size: 9pt; }"
            "QListWidget::item { padding: 3px 6px; border-bottom: 1px solid #2a2a2a; }");

        // Set initial splitter sizes (viewer ~75%, candidates panel ~25%).
        mainSplitter->setSizes({600, 200});

        // Wire member pointers for later updates.
        candidate_list_ = candidateListWidget;
        winner_label_   = winnerLabel;

        // Control bar: stop button + velocity readout.
        stop_btn_ = stopRobotBtn;
        vel_advx_label_ = velAdvxLabel;
        vel_advz_label_ = velAdvzLabel;
        vel_rot_label_  = velRotLabel;
        stop_btn_->setStyleSheet(
            "QPushButton { background: #8b0000; color: white; font-weight: bold; padding: 4px 12px; border-radius: 3px; }"
            "QPushButton:hover { background: #cc0000; }"
            "QPushButton:pressed { background: #550000; }");
        const QString vel_style = "QLabel { color: #00ccff; font-family: monospace; font-size: 9pt; padding: 2px 8px; }";
        vel_advx_label_->setStyleSheet(vel_style);
        vel_advz_label_->setStyleSheet(vel_style);
        vel_rot_label_->setStyleSheet(vel_style);
        connect(stop_btn_, &QPushButton::clicked, this, [this]() {
            efe_motion_paused_ = !efe_motion_paused_;
            if (efe_motion_paused_)
            {
                stop_efe_hotzone_motion();
                try { omnirobot_proxy->stopBase(); } catch (...) {}
                if (vel_advx_label_) vel_advx_label_->setText("vx: 0.000 m/s");
                if (vel_advz_label_) vel_advz_label_->setText("vz: 0.000 m/s");
                if (vel_rot_label_)  vel_rot_label_->setText("rot: 0.000 rad/s");
                stop_btn_->setText("Resume Robot");
                stop_btn_->setStyleSheet(
                    "QPushButton { background: #006400; color: white; font-weight: bold; padding: 4px 12px; border-radius: 3px; }"
                    "QPushButton:hover { background: #009900; }"
                    "QPushButton:pressed { background: #004400; }");
            }
            else
            {
                stop_btn_->setText("Stop Robot");
                stop_btn_->setStyleSheet(
                    "QPushButton { background: #8b0000; color: white; font-weight: bold; padding: 4px 12px; border-radius: 3px; }"
                    "QPushButton:hover { background: #cc0000; }"
                    "QPushButton:pressed { background: #550000; }");
            }
        });

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
    static std::optional<rc::BmrResult> last_indent_bmr;
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
                << "| dSDF:" << sdf_delta
                << "| VFE(acc/cplx/tot):" << res.vfe.accuracy << "/" << res.vfe.complexity << "/" << res.vfe.total;
        last_sdf_mse = res.sdf_mse;
        last_pose_log_ms = timestamp;
    }

    // Build a VFE scorer: evaluates each candidate polygon's SDF accuracy at the current pose.
    // Thread-safe: uses only captured values and a static RoomConceptAI method.
    const float _vfe_x = res.state[2], _vfe_y = res.state[3], _vfe_phi = res.state[4];
    const float _vfe_wh = room_ai.params.wall_height;
    const auto& _vfe_pts = lidar_data_->first;
    rc::CandidateScorer vfe_scorer =
        [_vfe_x, _vfe_y, _vfe_phi, _vfe_wh, &_vfe_pts](const std::vector<Eigen::Vector2f>& poly) -> float
        {
            return rc::RoomConceptAI::eval_polygon_sdf_only(
                poly, _vfe_pts, _vfe_x, _vfe_y, _vfe_phi, _vfe_wh);
        };

    if (const auto bmr_opt = bmr_engine_.maybe_evaluate_from_lidar(
            lidar_data_->first,
            res.state,
            room_ai.params.enable_bmr,
            room_ai.params.bmr_check_period,
            room_ai.params.bmr_min_frames_before_check,
            vfe_scorer);
        bmr_opt.has_value())
    {
        const auto &b = *bmr_opt;
        qInfo() << "[BMR] check"
                << "logBF=" << b.log_bf_01
                << "z2=" << b.posterior_z_sq
                << "proposal=" << static_cast<int>(b.proposal)
                << "expand=" << b.expand
                << "side=" << b.indent_side
                << "corner=" << b.indent_corner
                << "segment=[" << b.indent_a << "," << b.indent_b << "]"
                << "depth=" << b.indent_depth
                << "dx=" << b.indent_depth_x << "dy=" << b.indent_depth_y
                << "score(chal/current)=" << b.indent_score << "/" << b.current_indent_score
                << "switchBF=" << b.switch_log_bf_01
                << "latent p4..p9="
                << b.vertex_precision[4] << b.vertex_precision[5] << b.vertex_precision[6]
                << b.vertex_precision[7] << b.vertex_precision[8] << b.vertex_precision[9]
                << "active=" << b.activated_vertex_a << b.activated_vertex_b;

        const bool indent_candidate =
            (!b.expand && (b.proposal == rc::BmrResult::ProposalType::ADD_TWO_POINT_INDENT ||
                           b.proposal == rc::BmrResult::ProposalType::ADD_CORNER_INDENT));

        if (indent_candidate)
            last_indent_bmr = b;
        else
            last_indent_bmr.reset();

        if (!efe_motion_paused_ && params.ENABLE_EFE_HOTZONE_POLICY && indent_candidate)
        {
            drive_toward_hotzone_efe(res, b);
        }
        else if (!indent_candidate || efe_motion_paused_)
        {
            stop_efe_hotzone_motion();
        }

        // ── Hot-zone only debug mode ──
        // Disable candidate polygons; show only hot-zone segments on the viewer.
        if (viewer_2d_)
        {
            const float L1 = std::max(0.5f, res.state[0]);
            const float W1 = std::max(0.5f, res.state[1]);
            const float hx = 0.5f * L1;
            const float hy = 0.5f * W1;

            // Clear old candidate ghosts.
            viewer_2d_->clear_candidate_polygons();

            // Draw top-3 hot-zone segments and print debug values.
            if (!b.hot_zones.empty())
            {
                static const char* side_names_hz[] = {"Bottom", "Right", "Top", "Left"};
                qInfo() << "──── HOT-ZONE SEGMENTS (top" << b.hot_zones.size() << ") ────";
                std::vector<std::array<float,4>> hz_draw;
                for (std::size_t k = 0; k < b.hot_zones.size(); ++k)
                {
                    const auto& hz = b.hot_zones[k];
                    const char* sn = (hz.side >= 0 && hz.side < 4) ? side_names_hz[hz.side] : "?";
                    qInfo().nospace()
                        << "  #" << (k+1)
                        << "  side=" << hz.side << " (" << sn << ")"
                        << "  bins=[" << hz.i0 << "," << hz.i1 << "]"
                        << "  seg=[" << QString::number(hz.a, 'f', 2).toStdString().c_str()
                        << "," << QString::number(hz.b, 'f', 2).toStdString().c_str() << "]"
                        << "  mean_d=" << QString::number(hz.mean_dist, 'f', 3).toStdString().c_str()
                        << "  raw=" << QString::number(hz.raw_score, 'f', 4).toStdString().c_str()
                        << "  prox=" << QString::number(hz.prox, 'f', 3).toStdString().c_str()
                        << "  score=" << QString::number(hz.score, 'f', 4).toStdString().c_str();
                    hz_draw.push_back({static_cast<float>(hz.side), hz.a, hz.b, static_cast<float>(k)});
                }
                viewer_2d_->draw_hot_zones(hz_draw, hx, hy);
            }
            else
            {
                viewer_2d_->clear_hot_zones();
                qInfo() << "──── HOT-ZONE SEGMENTS: none found ────";
            }

            // Dump per-side distance profiles (compact: show bins > 0.05 only).
            {
                static const char* side_names_prof[] = {"Bot", "Rgt", "Top", "Lft"};
                for (int s = 0; s < 4; ++s)
                {
                    QString bins;
                    int active_count = 0;
                    for (int i = 0; i < 32; ++i)
                    {
                        float d = b.dist_profiles[s][i];
                        if (d > 0.05f)
                        {
                            bins += QString(" [%1]=%2").arg(i).arg(double(d), 0, 'f', 2);
                            ++active_count;
                        }
                    }
                    if (active_count > 0)
                        qInfo().nospace() << "  profile " << side_names_prof[s] << " (" << active_count << " active):" << bins;
                    else
                        qInfo().nospace() << "  profile " << side_names_prof[s] << ": all zero";
                }
            }

            update_candidate_list(b);
        }

        if (bmr_opt->expand)
        {
            last_indent_bmr.reset();
            stop_efe_hotzone_motion();
            room_ai.apply_bmr_result(*bmr_opt);
            if (viewer_2d_) viewer_2d_->clear_candidate_polygons();
            update_candidate_list(rc::BmrResult{});
        }
    }
    else
    {
        if (!efe_motion_paused_ && params.ENABLE_EFE_HOTZONE_POLICY && last_indent_bmr.has_value())
        {
            // Continue EFE control at compute rate using the latest valid BMR indent hypothesis.
            drive_toward_hotzone_efe(res, *last_indent_bmr);
        }
        else
        {
            stop_efe_hotzone_motion();
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    else if (score >= 0.20f)
    {
        score_label_->setStyleSheet("QLabel { color: rgb(210,140,0); font-weight: 700; padding: 2px 4px; }");
    }
    else
    {
        score_label_->setStyleSheet("QLabel { color: rgb(210,40,40); font-weight: 700; padding: 2px 4px; }");
    }
}

void SpecificWorker::update_viewer(const rc::LidarData &lidar_data, const rc::RoomConceptAI::UpdateResult &res)
{
    update_viewer(lidar_data.first, res);
}

void SpecificWorker::update_candidate_list(const rc::BmrResult& bmr)
{
    if (!candidate_list_ || !winner_label_)
        return;

    const auto& candidates = bmr.all_candidates;
    candidate_list_->clear();

    // Corner/side name tables.
    static const char* corner_names[]   = {"BL (bottom-left)", "BR (bottom-right)",
                                            "TR (top-right)",   "TL (top-left)"};
    static const char* side_names[]     = {"Bottom", "Right", "Top", "Left"};



    int chosen_idx = -1;
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i)
    {
        const auto& c = candidates[i];
        const bool is_wall   = (c.proposal == rc::BmrResult::ProposalType::ADD_TWO_POINT_INDENT);
        const bool is_corner = (c.proposal == rc::BmrResult::ProposalType::ADD_CORNER_INDENT);
        if (!is_wall && !is_corner) continue;

        // ── Type header ──
        QString type_label, geometry_detail;
        if (is_corner)
        {
            const char* cn  = (c.corner >= 0 && c.corner < 4) ? corner_names[c.corner]  : "?";
            type_label    = QString("[Corner %1]  %2").arg(c.corner).arg(cn);
            geometry_detail = QString("dx = %1 m  dy = %2 m  |  2 free params (dx, dy)")
                .arg(double(c.depth_x), 0, 'f', 3)
                .arg(double(c.depth_y), 0, 'f', 3);
        }
        else
        {
            const char* sn  = (c.side >= 0 && c.side < 4) ? side_names[c.side]  : "?";
            type_label    = QString("[Wall  side %1]  %2 wall").arg(c.side).arg(sn);
            geometry_detail = QString("seg [%1, %2]  depth = %3 m  |  3 free params (d, a, b)")
                .arg(double(c.a), 0, 'f', 2)
                .arg(double(c.b), 0, 'f', 2)
                .arg(double(c.depth), 0, 'f', 3);
        }

        const QString score_str = QString("score = %1  (raw = %2)").arg(double(c.score), 0, 'f', 4).arg(double(c.raw_score), 0, 'f', 4);
        const QString prefix    = c.is_chosen ? QString::fromUtf8("\u2605 ") : "  ";

        // Build two-line entry: "★ [type]   score" / geometry.
        const QString line1 = QString("%1%2    %3").arg(prefix).arg(type_label).arg(score_str);
        const QString line2 = QString("    %1").arg(geometry_detail);

        // Item 1 — main line (bold, white).
        {
            auto* item = new QListWidgetItem(line1, candidate_list_);
            QFont f = item->font();
            f.setBold(c.is_chosen);
            item->setFont(f);
            item->setForeground(Qt::white);
            if (c.is_chosen) chosen_idx = i;
        }
        // Item 2 — geometry detail.
        {
            auto* item = new QListWidgetItem(line2, candidate_list_);
            item->setForeground(Qt::white);
        }
    }

    // ── Winner banner ──
    // Show both the current accepted hypothesis (if any) and the current challenger.
    QString banner;

    // Line 1: current accepted hypothesis.
    if (bmr.current_proposal != rc::BmrResult::ProposalType::NONE)
    {
        const bool cur_corner = (bmr.current_proposal == rc::BmrResult::ProposalType::ADD_CORNER_INDENT);
        if (cur_corner && bmr.current_corner >= 0 && bmr.current_corner < 4)
        {
            const char* cn = corner_names[bmr.current_corner];
            banner = QString("Accepted: Corner %1 (%2)  dx=%3 dy=%4  |  raw=%5")
                .arg(bmr.current_corner).arg(cn)
                .arg(double(bmr.current_depth_x), 0, 'f', 3)
                .arg(double(bmr.current_depth_y), 0, 'f', 3)
                .arg(double(bmr.current_indent_score), 0, 'f', 4);
        }
        else if (bmr.current_side >= 0 && bmr.current_side < 4)
        {
            const char* sn = side_names[bmr.current_side];
            banner = QString("Accepted: Wall %1 (%2)  depth=%3  |  raw=%4")
                .arg(bmr.current_side).arg(sn)
                .arg(double(bmr.current_depth), 0, 'f', 3)
                .arg(double(bmr.current_indent_score), 0, 'f', 4);
        }
    }

    // Line 2: current frame's challenger (winner of this round).
    if (chosen_idx >= 0)
    {
        const auto& w = candidates[chosen_idx];
        const bool is_corner = (w.proposal == rc::BmrResult::ProposalType::ADD_CORNER_INDENT);
        const bool is_wall   = (w.proposal == rc::BmrResult::ProposalType::ADD_TWO_POINT_INDENT);

        QString kind, geom;
        if (is_corner)
        {
            const char* cn = (w.corner >= 0 && w.corner < 4) ? corner_names[w.corner] : "?";
            kind = QString("Corner %1 (%2)").arg(w.corner).arg(cn);
            geom = QString("dx=%1 dy=%2 | score=%3 raw=%4")
                .arg(double(w.depth_x), 0, 'f', 3).arg(double(w.depth_y), 0, 'f', 3)
                .arg(double(w.score), 0, 'f', 4).arg(double(w.raw_score), 0, 'f', 4);
        }
        else if (is_wall)
        {
            const char* sn = (w.side >= 0 && w.side < 4) ? side_names[w.side] : "?";
            kind = QString("Wall %1 (%2)").arg(w.side).arg(sn);
            geom = QString("seg[%1,%2] d=%3 | score=%4 raw=%5")
                .arg(double(w.a), 0, 'f', 2).arg(double(w.b), 0, 'f', 2)
                .arg(double(w.depth), 0, 'f', 3)
                .arg(double(w.score), 0, 'f', 4).arg(double(w.raw_score), 0, 'f', 4);
        }
        if (!banner.isEmpty()) banner += '\n';
        banner += QString("Challenger: %1  %2").arg(kind).arg(geom);

        // BF status: show switch BF if there's an accepted hypothesis,
        // or accumulation BF toward first commitment otherwise.
        if (bmr.current_proposal != rc::BmrResult::ProposalType::NONE)
            banner += QString("\nSwitch BF: %1").arg(double(bmr.switch_log_bf_01), 0, 'f', 3);
        else
            banner += QString("\nBF: %1 / %2 (threshold)")
                .arg(double(bmr.log_bf_01), 0, 'f', 3)
                .arg(double(-2.0), 0, 'f', 1);
    }
    else if (!candidates.empty())
    {
        // Candidates were evaluated but none beats the rectangle (all scores ≤ 0).
        if (!banner.isEmpty()) banner += '\n';
        banner += "Challenger: none (rectangle wins)";
    }

    if (!banner.isEmpty())
    {
        winner_label_->setText(banner);
        winner_label_->setStyleSheet(
            "QLabel { font-weight: bold; color: white; font-size: 9pt;"
            " padding: 6px; border-top: 2px solid white; background: #111; }");
    }
    else
    {
        winner_label_->setText("Winner: \xe2\x80\x94");
        winner_label_->setStyleSheet(
            "QLabel { font-weight: bold; color: rgb(120,120,120); padding: 4px; border-top: 1px solid #555; }");
    }
}

void SpecificWorker::update_viewer(const std::vector<Eigen::Vector3f> &points, const rc::RoomConceptAI::UpdateResult &res)
{
    if (!viewer_2d_)
        return;

    const auto polygon_verts = room_ai.get_room_polygon_vertices();
    const bool has_polygon_layout = polygon_verts.size() >= 3;

    // Update robot pose and estimated rectangular room.
    viewer_2d_->update_robot(res.state[2], res.state[3], res.state[4]);
    viewer_2d_->update_estimated_room_rect(res.state[0], res.state[1], has_polygon_layout);

    // Draw current estimated layout: polygon if available, otherwise fallback rectangle.
    if (has_polygon_layout)
    {
        viewer_2d_->draw_room_polygon(polygon_verts, false);
    }
    else
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

    // One-shot: fit the view to the estimated room layout on first render.
    if (!viewer_fitted_)
    {
        const float w = res.state[0];
        const float l = res.state[1];
        const float margin = 0.15f * std::max(w, l);
        viewer_2d_->fit_to_scene(QRectF(-w * 0.5f - margin, -l * 0.5f - margin,
                                         w + 2.f * margin, l + 2.f * margin));
        viewer_fitted_ = true;
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
    constexpr float GT_ROOM_WIDTH = 7.88136f;
    constexpr float GT_ROOM_LENGTH = 14.3441f;

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

    room_ai.set_initial_state(GT_ROOM_WIDTH, GT_ROOM_LENGTH, 0.f, 0.f, init_phi);
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
            << "| using GT init (W,L)=" << GT_ROOM_WIDTH << GT_ROOM_LENGTH
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
            send_base_command(0.f, 0.f, params.BOOTSTRAP_ROT_SPEED);
            bootstrap_rotating_ = true;
        }
        else
        {
            omnirobot_proxy->stopBase();
            push_velocity_command(0.f, 0.f, 0.f);
            bootstrap_rotating_ = false;
        }
    }
    catch(const Ice::Exception &e)
    {
        qWarning() << "Bootstrap rotation command failed:" << e.what();
    }
}

void SpecificWorker::push_velocity_command(float advx, float advz, float rot)
{
    rc::VelocityCommand cmd{advx, advz, rot};
    const auto ts = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    velocity_buffer_.put<0>(std::move(cmd), ts);
}

void SpecificWorker::send_base_command(float advx, float advz, float rot)
{
    // Internal policy uses m/s with (advx=lateral, advz=forward).
    // OmniRobot expects translational components in mm/s and this call is set as (advance, side).
    constexpr float M_TO_MM = 1000.f;
    omnirobot_proxy->setSpeedBase(advx * M_TO_MM, advz * M_TO_MM, rot);
    push_velocity_command(advx, advz, rot);
    // Update velocity display.
    if (vel_advx_label_) vel_advx_label_->setText(QString("vx: %1 m/s").arg(double(advx), 0, 'f', 3));
    if (vel_advz_label_) vel_advz_label_->setText(QString("vz: %1 m/s").arg(double(advz), 0, 'f', 3));
    if (vel_rot_label_)  vel_rot_label_->setText(QString("rot: %1 rad/s").arg(double(rot), 0, 'f', 3));
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
    const float wm = std::max(0.1f, params.EFE_WALL_MARGIN); // minimum inward distance from wall
    if (bmr.indent_side >= 0)
    {
        // Wall indent: hot-zone inward from wall by at least wall_margin.
        const float off = std::max(wm, 0.5f * depth);
        switch (bmr.indent_side)
        {
            case 0: hot = Eigen::Vector2f(tmid * hx, -hy + off); break;
            case 1: hot = Eigen::Vector2f(hx - off, tmid * hy); break;
            case 2: hot = Eigen::Vector2f(tmid * hx, hy - off); break;
            case 3: hot = Eigen::Vector2f(-hx + off, tmid * hy); break;
            default: return;
        }
    }
    else if (bmr.indent_corner >= 0)
    {
        // Corner indent: hot-zone offset by half of each independent depth, clamped to wall_margin.
        const float ox = std::max(wm, 0.5f * std::max(0.05f, bmr.indent_depth_x));
        const float oy = std::max(wm, 0.5f * std::max(0.05f, bmr.indent_depth_y));
        switch (bmr.indent_corner)
        {
            case 0: hot = Eigen::Vector2f(-hx + ox,  -hy + oy); break;  // BL
            case 1: hot = Eigen::Vector2f( hx - ox,  -hy + oy); break;  // BR
            case 2: hot = Eigen::Vector2f( hx - ox,   hy - oy); break;  // TR
            case 3: hot = Eigen::Vector2f(-hx + ox,   hy - oy); break;  // TL
            default: return;
        }
    }
    else
        return;

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
        {0.f,  0.f, -wr, "turn_left"},
        {0.f,  0.f,  wr, "turn_right"},
        {0.f,  0.8f * vf, -0.7f * wr, "arc_left"},
        {0.f,  0.8f * vf,  0.7f * wr, "arc_right"},
        {0.f,  0.f,  0.f, "hold"}
    }};

    const float x0 = res.state[2];
    const float y0 = res.state[3];
    const float phi0 = res.state[4];

    Eigen::Vector2f hot_eff = hot;
    {
        const Eigen::Vector2f cur{x0, y0};
        const Eigen::Vector2f d = hot - cur;
        const float n = d.norm();
        const float max_d = std::max(0.2f, params.EFE_LOCAL_GOAL_MAX_DIST);
        if (n > max_d)
            hot_eff = cur + d * (max_d / n);
    }

    // Smooth target across BMR checks to reduce command direction flip-flop.
    {
        const float alpha = std::clamp(params.EFE_TARGET_SMOOTH_ALPHA, 0.f, 1.f);
        if (!has_smoothed_efe_target_)
        {
            smoothed_efe_target_ = hot_eff;
            has_smoothed_efe_target_ = true;
        }
        else
        {
            smoothed_efe_target_ = (1.0f - alpha) * smoothed_efe_target_ + alpha * hot_eff;
        }
        hot_eff = smoothed_efe_target_;
    }

    const float dt = std::max(0.05f, params.EFE_DT);
    const int tree_depth = std::max(1, params.EFE_TREE_DEPTH);
    const float discount = std::clamp(params.EFE_DISCOUNT, 0.0f, 1.0f);
    const float sigma2 = std::max(1e-3f, params.EFE_PRIOR_SIGMA * params.EFE_PRIOR_SIGMA);
    const float process_noise_trans2 = params.EFE_PROCESS_NOISE_TRANS * params.EFE_PROCESS_NOISE_TRANS;
    const float process_noise_rot2   = params.EFE_PROCESS_NOISE_ROT   * params.EFE_PROCESS_NOISE_ROT;
    const float obs_noise_var = std::max(1e-4f, params.EFE_OBS_NOISE_VAR);

    auto wrap_pi = [](float a) -> float
    {
        while (a > static_cast<float>(M_PI)) a -= static_cast<float>(2.0 * M_PI);
        while (a < static_cast<float>(-M_PI)) a += static_cast<float>(2.0 * M_PI);
        return a;
    };

    // Indent half-width in world metres.
    float indent_half_w;
    if (bmr.indent_side >= 0 && bmr.indent_side < 4)
        indent_half_w = 0.5f * std::abs(bmr.indent_b - bmr.indent_a)
                        * ((bmr.indent_side == 0 || bmr.indent_side == 2) ? hx : hy);
    else
        indent_half_w = 0.5f * depth;  // corner: use depth as proxy for visible extent

    // Wall-inward normal in world (room) frame for the active indent side.
    // Used both for D-optimality projection and rank-1 belief update.
    Eigen::Vector2f wall_normal = Eigen::Vector2f::Zero();
    if (bmr.indent_side >= 0)
    {
        switch (bmr.indent_side)
        {
            case 0: wall_normal = { 0.f,  1.f}; break;  // bottom wall, normal +y
            case 1: wall_normal = {-1.f,  0.f}; break;  // right  wall, normal -x
            case 2: wall_normal = { 0.f, -1.f}; break;  // top    wall, normal -y
            case 3: wall_normal = { 1.f,  0.f}; break;  // left   wall, normal +x
            default: break;
        }
    }
    else if (bmr.indent_corner >= 0)
    {
        // Corner: use diagonal inward normal (pointing toward room centre).
        constexpr float inv_sqrt2 = 0.70710678f;
        switch (bmr.indent_corner)
        {
            case 0: wall_normal = { inv_sqrt2,  inv_sqrt2}; break;  // BL → toward centre
            case 1: wall_normal = {-inv_sqrt2,  inv_sqrt2}; break;  // BR
            case 2: wall_normal = {-inv_sqrt2, -inv_sqrt2}; break;  // TR
            case 3: wall_normal = { inv_sqrt2, -inv_sqrt2}; break;  // TL
            default: break;
        }
    }
    if (wall_normal.squaredNorm() < 0.5f)
        return;  // no valid proposal

    // Initial belief from current Laplace posterior.
    const Eigen::Vector3f mu0{x0, y0, phi0};
    // res.covariance is (x,y,phi) 3x3 Laplace posterior covariance.
    // Guard against near-singular matrices from early uninitialised frames.
    Eigen::Matrix3f Sigma0 = res.covariance;
    Sigma0(0,0) = std::max(Sigma0(0,0), 1e-4f);
    Sigma0(1,1) = std::max(Sigma0(1,1), 1e-4f);
    Sigma0(2,2) = std::max(Sigma0(2,2), 1e-4f);

    // --- D-optimality expected information gain ----------------------------
    // EIG = 0.5 * log( 1 + lambda_obs * v_n^T Sigma_xy v_n )
    // where lambda_obs = vis_width^2 / (range^2 * obs_noise_var).
    // This is the Bayesian D-optimality criterion: it measures how much the
    // observation from this pose will reduce pose uncertainty in the wall-normal
    // direction, given the *current* belief covariance Sigma.
    auto d_opt_eig = [&](const Eigen::Vector3f& mu, const Eigen::Matrix3f& Sigma) -> float
    {
        const Eigen::Vector2f d = hot_eff - Eigen::Vector2f(mu.x(), mu.y());
        const float range   = d.norm() + 0.1f;
        const float bearing = std::atan2(d.y(), d.x());
        const float sensor_fwd = wrap_pi(mu.z() + static_cast<float>(M_PI_2));
        const float ang_err    = wrap_pi(bearing - sensor_fwd);
        const float cos_a  = std::max(0.f, std::cos(ang_err));
        const float vis_w  = indent_half_w * cos_a;

        const float lambda_obs = (vis_w * vis_w) / (range * range * obs_noise_var);
        // Marginal variance of belief along wall-normal direction (2x2 xy sub-block).
        const float var_n = wall_normal.dot(Sigma.topLeftCorner<2,2>() * wall_normal);
        return 0.5f * std::log(1.f + lambda_obs * var_n);
    };

    // --- Rank-1 belief update after expected observation ------------------
    // Woodbury identity: Sigma_post = Sigma - (lambda/(1+lambda*var_n)) * Su * Su^T
    // where u = [wall_normal; 0] (observation only affects x,y, not phi).
    auto belief_update = [&](const Eigen::Matrix3f& Sigma, float lambda_obs) -> Eigen::Matrix3f
    {
        Eigen::Vector3f u = Eigen::Vector3f::Zero();
        u.head<2>() = wall_normal;
        const float var_n = wall_normal.dot(Sigma.topLeftCorner<2,2>() * wall_normal);
        const float alpha = lambda_obs / (1.f + lambda_obs * var_n);
        const Eigen::Vector3f Su = Sigma * u;
        return Sigma - alpha * Su * Su.transpose();
    };

    // --- Unscented Transform belief propagation ---------------------------
    // Propagates Gaussian belief (mu, Sigma) through the nonlinear motion model.
    // Uses 6 symmetric sigma points (kappa=0, equal weights 1/6).
    // Returns (mu', Sigma') including process noise Q.
    auto ut_propagate = [&](const Eigen::Vector3f& mu, const Eigen::Matrix3f& Sigma,
                             float advx, float advz, float rot_rate)
        -> std::pair<Eigen::Vector3f, Eigen::Matrix3f>
    {
        // Cholesky decomposition; fallback to scaled identity on failure.
        Eigen::LLT<Eigen::Matrix3f> llt(Sigma);
        Eigen::Matrix3f L;
        if (llt.info() == Eigen::Success)
            L = llt.matrixL();
        else
            L = Eigen::Matrix3f::Identity() * 0.01f;

        constexpr float kSqrt3 = 1.7320508f;  // sqrt(n=3) with kappa=0

        // 6 symmetric sigma points (center gets weight 0 with kappa=0, omitted).
        Eigen::Matrix<float, 3, 6> sp_prop;
        for (int i = 0; i < 3; ++i)
        {
            for (int sign = 0; sign < 2; ++sign)
            {
                const Eigen::Vector3f sp = mu + (sign == 0 ? 1.f : -1.f) * kSqrt3 * L.col(i);
                const float c = std::cos(sp.z()), s = std::sin(sp.z());
                const int col = i * 2 + sign;
                sp_prop(0, col) = sp.x() + (advx * c - advz * s) * dt;
                sp_prop(1, col) = sp.y() + (advx * s + advz * c) * dt;
                sp_prop(2, col) = wrap_pi(sp.z() - rot_rate * dt);
            }
        }

        // Reconstruct mean (equal weight 1/6).
        Eigen::Vector3f mu_new = sp_prop.rowwise().mean();
        mu_new.z() = wrap_pi(mu_new.z());

        // Reconstruct covariance.
        Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
        for (int j = 0; j < 6; ++j)
        {
            Eigen::Vector3f dv = sp_prop.col(j) - mu_new;
            dv.z() = wrap_pi(dv.z());
            S += dv * dv.transpose();
        }
        S /= 6.f;
        // Add process noise.
        S(0,0) += process_noise_trans2;
        S(1,1) += process_noise_trans2;
        S(2,2) += process_noise_rot2;

        return {mu_new, S};
    };

    // --- Recursive EFE tree evaluation ------------------------------------
    // State: (mu, Sigma) — full Gaussian belief.
    // Returns minimum discounted G over all continuations at this level.
    // At each node: propagate belief via UT, compute D-opt EIG, update belief
    // via rank-1 Woodbury (accumulating information across the rollout).
    std::function<float(const Eigen::Vector3f&, const Eigen::Matrix3f&, int, float)> eval_tree;
    eval_tree = [&](const Eigen::Vector3f& mu, const Eigen::Matrix3f& Sigma,
                    int depth, float disc) -> float
    {
        if (depth >= tree_depth) return 0.f;
        float min_g = std::numeric_limits<float>::infinity();
        for (const auto &a : actions)
        {
            // 1. UT prediction: propagate belief through motion model.
            auto [mu_pred, Sigma_pred] = ut_propagate(mu, Sigma, a.advx, a.advz, a.rot);

            // 2. D-optimality EIG from predicted belief.
            const float eig = d_opt_eig(mu_pred, Sigma_pred);

            // 3. Rank-1 belief update from hypothetical observation (info accumulates).
            const Eigen::Vector2f d = hot_eff - Eigen::Vector2f(mu_pred.x(), mu_pred.y());
            const float range = d.norm() + 0.1f;
            const float bearing = std::atan2(d.y(), d.x());
            const float sensor_fwd = wrap_pi(mu_pred.z() + static_cast<float>(M_PI_2));
            const float ang_err = wrap_pi(bearing - sensor_fwd);
            const float cos_a = std::max(0.f, std::cos(ang_err));
            const float vis_w = indent_half_w * cos_a;
            const float lambda_obs = (vis_w * vis_w) / (range * range * obs_noise_var);
            const Eigen::Matrix3f Sigma_post = belief_update(Sigma_pred, lambda_obs);

            // 4. G: pragmatic (goal-reaching) + epistemic (negative EIG) + effort.
            const float ddx = mu_pred.x() - hot_eff.x(), ddy = mu_pred.y() - hot_eff.y();
            const float risk      = 0.5f * (ddx*ddx + ddy*ddy) / sigma2;
            const float epistemic = -params.EFE_INFO_GAIN_WEIGHT * eig;
            const float effort    = params.EFE_CONTROL_WEIGHT *
                                    (a.advx*a.advx + a.advz*a.advz + 0.5f*a.rot*a.rot);
            const float g_step    = disc * (risk + epistemic + effort);

            // 5. Recurse with posterior belief.
            min_g = std::min(min_g, g_step + eval_tree(mu_pred, Sigma_post, depth + 1, disc * discount));
        }
        return min_g;
    };

    // --- Root action evaluation -------------------------------------------
    constexpr int N_ACTIONS = 6;
    std::array<float, N_ACTIONS> G_vals;

    float best_g = std::numeric_limits<float>::infinity();
    int best_idx = static_cast<int>(actions.size()) - 1;

    rc::Viewer2D::TrajDrawData traj_debug;
    traj_debug.trajectories.resize(actions.size());
    traj_debug.carrot = hot_eff;
    traj_debug.robot_x = x0;
    traj_debug.robot_y = y0;

    for (int ai = 0; ai < N_ACTIONS; ++ai)
    {
        const auto &a = actions[ai];

        // Root step: UT-propagate from initial belief.
        auto [mu_pred, Sigma_pred] = ut_propagate(mu0, Sigma0, a.advx, a.advz, a.rot);

        const float eig = d_opt_eig(mu_pred, Sigma_pred);

        // Belief update at root step.
        const Eigen::Vector2f d0 = hot_eff - Eigen::Vector2f(mu_pred.x(), mu_pred.y());
        const float range0 = d0.norm() + 0.1f;
        const float bearing0 = std::atan2(d0.y(), d0.x());
        const float sfwd0 = wrap_pi(mu_pred.z() + static_cast<float>(M_PI_2));
        const float ae0   = wrap_pi(bearing0 - sfwd0);
        const float lobs0 = (indent_half_w * indent_half_w * std::max(0.f, std::cos(ae0)) * std::max(0.f, std::cos(ae0)))
                            / (range0 * range0 * obs_noise_var);
        const Eigen::Matrix3f Sigma_post0 = belief_update(Sigma_pred, lobs0);

        const float ddx = mu_pred.x() - hot_eff.x(), ddy = mu_pred.y() - hot_eff.y();
        const float risk      = 0.5f * (ddx*ddx + ddy*ddy) / sigma2;
        const float epistemic = -params.EFE_INFO_GAIN_WEIGHT * eig;
        const float effort    = params.EFE_CONTROL_WEIGHT *
                                (a.advx*a.advx + a.advz*a.advz + 0.5f*a.rot*a.rot);
        const float g_root = risk + epistemic + effort
                           + eval_tree(mu_pred, Sigma_post0, 1, discount);

        G_vals[ai] = g_root;
        if (g_root < best_g) { best_g = g_root; best_idx = ai; }

        // Visualization: greedy single-step lookahead continuation.
        auto &traj = traj_debug.trajectories[ai];
        traj.clear();
        traj.emplace_back(x0, y0);
        traj.emplace_back(mu_pred.x(), mu_pred.y());
        Eigen::Vector3f vm = mu_pred;
        Eigen::Matrix3f vS = Sigma_post0;
        for (int k = 2; k <= tree_depth; ++k)
        {
            float best_local = std::numeric_limits<float>::infinity();
            Eigen::Vector3f bm = vm;
            for (const auto &va : actions)
            {
                auto [tm, tS] = ut_propagate(vm, vS, va.advx, va.advz, va.rot);
                const float tdx = tm.x() - hot_eff.x(), tdy = tm.y() - hot_eff.y();
                const float tg  = 0.5f * (tdx*tdx + tdy*tdy) / sigma2
                                 - params.EFE_INFO_GAIN_WEIGHT * d_opt_eig(tm, tS);
                if (tg < best_local) { best_local = tg; bm = tm; vS = tS; }
            }
            traj.emplace_back(bm.x(), bm.y());
            vm = bm;
        }
    }

    // --- Deterministic argmin policy ----------------------------------------
    // Use the action with the lowest Expected Free Energy.
    // Previous Boltzmann sampling introduced stochastic selection of zero-speed
    // actions (hold, turn-only) that killed forward motion.
    const float cmd_advx = actions[best_idx].advx;
    float       cmd_advz = actions[best_idx].advz;
    const float cmd_rot  = actions[best_idx].rot;

    const Eigen::Vector2f cur{x0, y0};
    const float target_dist = (hot_eff - cur).norm();
    // Turn-only guard: if the blended command has negligible forward speed and the
    // target is far, inject a minimum forward component to avoid turn-in-place dithering.
    if (target_dist > params.EFE_TURN_ONLY_MIN_DIST && cmd_advz < 0.3f * params.EFE_FORWARD_SPEED)
        cmd_advz = 0.3f * params.EFE_FORWARD_SPEED;

    traj_debug.best_idx = best_idx;
    if (viewer_2d_ != nullptr)
    {
        viewer_2d_->draw_trajectory_debug(traj_debug);
        viewer_2d_->invalidate();
    }

    try
    {
        send_base_command(cmd_advx, cmd_advz, cmd_rot);
        efe_hotzone_active_ = true;

        static std::int64_t last_efe_log_ms = 0;
        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        if (now_ms - last_efe_log_ms >= 1000)
        {
            qInfo() << "[EFE] hot-zone global=" << hot.x() << hot.y()
                << " local=" << hot_eff.x() << hot_eff.y()
                        << " depth=" << tree_depth
                        << " best_action=" << actions[best_idx].name
                        << " cmd=" << cmd_advx << cmd_advz << cmd_rot
                        << " G_best=" << best_g;
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
    if (viewer_2d_ != nullptr)
    {
        viewer_2d_->hide_trajectory_debug();
        viewer_2d_->invalidate();
    }

    if (!efe_hotzone_active_)
        return;

    try
    {
        omnirobot_proxy->stopBase();
        push_velocity_command(0.f, 0.f, 0.f);
    }
    catch(const Ice::Exception &e)
    {
        qWarning() << "Stop EFE motion failed:" << e.what();
    }
    efe_hotzone_active_ = false;
    has_smoothed_efe_target_ = false;
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
        push_velocity_command(0.f, 0.f, 0.f);
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
