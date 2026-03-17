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
        switch (c.corner)
        {
            case 0: return {{-hx+depth,-hy},{hx,-hy},{hx,hy},{-hx,hy},{-hx,-hy+depth},{-hx+depth,-hy+depth}}; // BL
            case 1: return {{-hx,-hy},{hx-depth,-hy},{hx-depth,-hy+depth},{hx,-hy+depth},{hx,hy},{-hx,hy}};    // BR
            case 2: return {{-hx,-hy},{hx,-hy},{hx,hy-depth},{hx-depth,hy-depth},{hx-depth,hy},{-hx,hy}};      // TR
            case 3: default: return {{-hx,-hy},{hx,-hy},{hx,hy},{-hx+depth,hy},{-hx+depth,hy-depth},{-hx,hy-depth}}; // TL
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

        // Style the right candidates panel.
        candidatesTitleLabel->setStyleSheet(
            "QLabel { font-weight: bold; font-size: 11pt; padding: 4px; border-bottom: 1px solid #555; }");
        winnerLabel->setStyleSheet(
            "QLabel { font-weight: bold; color: rgb(50, 200, 50); padding: 4px; border-top: 1px solid #555; }");
        candidateListWidget->setStyleSheet(
            "QListWidget { background: #1e1e1e; color: #dddddd; font-family: monospace; font-size: 9pt; }");

        // Set initial splitter sizes (viewer ~80 %, candidates panel ~20 %).
        mainSplitter->setSizes({900, 280});

        // Wire member pointers for later updates.
        candidate_list_ = candidateListWidget;
        winner_label_   = winnerLabel;

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
        const bool polygon_mode = room_ai.is_using_polygon_room();
        qInfo() << "[BMR] check"
                << "logBF=" << b.log_bf_01
                << "z2=" << b.posterior_z_sq
                << "proposal=" << static_cast<int>(b.proposal)
                << "expand=" << b.expand
                << "side=" << b.indent_side
                << "corner=" << b.indent_corner
                << "segment=[" << b.indent_a << "," << b.indent_b << "]"
                << "depth=" << b.indent_depth
                << "score(chal/current)=" << b.indent_score << "/" << b.current_indent_score
                << "switchBF=" << b.switch_log_bf_01
                << "latent p4..p9="
                << b.vertex_precision[4] << b.vertex_precision[5] << b.vertex_precision[6]
                << b.vertex_precision[7] << b.vertex_precision[8] << b.vertex_precision[9]
                << "active=" << b.activated_vertex_a << b.activated_vertex_b;

        const bool indent_candidate =
            (!polygon_mode && !b.expand && b.proposal == rc::BmrResult::ProposalType::ADD_TWO_POINT_INDENT);

        if (indent_candidate)
            last_indent_bmr = b;
        else
            last_indent_bmr.reset();

        if (params.ENABLE_EFE_HOTZONE_POLICY && indent_candidate)
        {
            drive_toward_hotzone_efe(res, b);
        }
        else
        {
            stop_efe_hotzone_motion();
        }

        // Draw ghost candidate polygons in the viewer for visual debugging.
        if (viewer_2d_ && !b.all_candidates.empty())
        {
            const float L1 = std::max(0.5f, res.state[0]);
            const float W1 = std::max(0.5f, res.state[1]);
            const float hx = 0.5f * L1;
            const float hy = 0.5f * W1;

            std::vector<rc::Viewer2D::CandidateDrawData> draw_candidates;
            for (const auto& cand : b.all_candidates)
            {
                auto verts = indent_candidate_polygon(cand, hx, hy);
                if (!verts.empty())
                {
                    rc::Viewer2D::CandidateDrawData d;
                    d.verts     = std::move(verts);
                    d.score     = cand.score;
                    d.is_corner = (cand.proposal == rc::BmrResult::ProposalType::ADD_CORNER_INDENT);
                    d.is_chosen = cand.is_chosen;
                    d.side_id   = cand.side;
                    d.corner_id = cand.corner;
                    d.seg_a     = cand.a;
                    d.seg_b     = cand.b;
                    draw_candidates.push_back(std::move(d));
                }
            }
            viewer_2d_->draw_candidate_polygons(draw_candidates);
            update_candidate_list(b.all_candidates);
        }

        if (bmr_opt->expand)
        {
            last_indent_bmr.reset();
            stop_efe_hotzone_motion();
            room_ai.apply_bmr_result(*bmr_opt);
            if (viewer_2d_) viewer_2d_->clear_candidate_polygons();
            update_candidate_list({});
        }
    }
    else
    {
        if (params.ENABLE_EFE_HOTZONE_POLICY && last_indent_bmr.has_value())
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
    else if (score >= 0.35f)
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

void SpecificWorker::update_candidate_list(const std::vector<rc::BmrResult::IndentCandidateInfo>& candidates)
{
    if (!candidate_list_ || !winner_label_)
        return;

    candidate_list_->clear();

    int chosen_idx = -1;
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i)
    {
        const auto& c = candidates[i];
        const bool is_wall   = (c.proposal == rc::BmrResult::ProposalType::ADD_TWO_POINT_INDENT);
        const bool is_corner = (c.proposal == rc::BmrResult::ProposalType::ADD_CORNER_INDENT);

        QString type_str = is_wall ? "Wall" : (is_corner ? "Corner" : "???");
        QString detail;
        if (is_wall)
            detail = QString("side=%1  [%2, %3]  d=%4")
                .arg(c.side)
                .arg(double(c.a), 0, 'f', 2)
                .arg(double(c.b), 0, 'f', 2)
                .arg(double(c.depth), 0, 'f', 3);
        else
        {
            static const char* corner_names[] = {"BL", "BR", "TR", "TL"};
            const char* cname = (c.corner >= 0 && c.corner < 4) ? corner_names[c.corner] : "?";
            detail = QString("corner=%1(%2)  d=%3")
                .arg(c.corner)
                .arg(cname)
                .arg(double(c.depth), 0, 'f', 3);
        }

        const QString prefix = c.is_chosen ? QString::fromUtf8("\u2605 ") : "  ";
        const QString text = QString("%1[%2]  score=%3  %4")
            .arg(prefix)
            .arg(type_str)
            .arg(double(c.score), 0, 'f', 4)
            .arg(detail);

        auto* item = new QListWidgetItem(text, candidate_list_);
        if (c.is_chosen)
        {
            item->setForeground(is_corner ? QColor(60, 220, 100) : QColor(60, 180, 255));
            QFont f = item->font();
            f.setBold(true);
            item->setFont(f);
            chosen_idx = i;
        }
        else
        {
            item->setForeground(QColor(140, 140, 140));
        }
    }

    // Winner summary label
    if (chosen_idx >= 0)
    {
        const auto& w = candidates[chosen_idx];
        const bool is_corner = (w.proposal == rc::BmrResult::ProposalType::ADD_CORNER_INDENT);
        const QString kind = is_corner
            ? QString("Corner %1").arg(w.corner)
            : QString("Wall side %1  [%2,%3]").arg(w.side).arg(double(w.a), 0, 'f', 2).arg(double(w.b), 0, 'f', 2);
        winner_label_->setText(QString("Winner: %1\n  depth=%2  score=%3")
            .arg(kind)
            .arg(double(w.depth), 0, 'f', 3)
            .arg(double(w.score), 0, 'f', 4));
        winner_label_->setStyleSheet(
            is_corner
                ? "QLabel { font-weight: bold; color: rgb(60,220,100); padding: 4px; border-top: 1px solid #555; }"
                : "QLabel { font-weight: bold; color: rgb(60,180,255); padding: 4px; border-top: 1px solid #555; }");
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

    auto wrap_pi = [](float a) -> float
    {
        while (a > static_cast<float>(M_PI)) a -= static_cast<float>(2.0 * M_PI);
        while (a < static_cast<float>(-M_PI)) a += static_cast<float>(2.0 * M_PI);
        return a;
    };

    // Indent half-width in world metres (used as Fisher-info scale for the SDF observation).
    // The indent runs over fraction |b-a| of the relevant half-extent.
    const float indent_half_w = 0.5f * std::abs(bmr.indent_b - bmr.indent_a)
                                * ((bmr.indent_side == 0 || bmr.indent_side == 2) ? hx : hy);

    // Predict expected Fisher information of the SDF lidar observation about indent geometry
    // from predicted pose (px, py, pphi).
    // Proxy: (visible_angular_width)^2 = (indent_half_w * cos(off-axis angle) / range)^2.
    // Higher when the robot faces the indent squarely and is close to it.
    auto predict_info_gain = [&](float px, float py, float pphi) -> float
    {
        const Eigen::Vector2f d = hot_eff - Eigen::Vector2f(px, py);
        const float range   = d.norm() + 0.1f;
        const float bearing = std::atan2(d.y(), d.x());
        // Robot sensor forward axis is local +Y -> world direction phi+pi/2.
        const float sensor_fwd = wrap_pi(pphi + static_cast<float>(M_PI_2));
        const float ang_err    = wrap_pi(bearing - sensor_fwd);
        const float cos_a  = std::max(0.f, std::cos(ang_err));
        const float vis    = indent_half_w * cos_a;
        return (vis * vis) / (range * range);  // Fisher info \propto (visible_width / range)^2
    };

    // Recursive EFE tree evaluation.
    // Returns the MINIMUM discounted cumulative G over all (tree_depth - depth) deep
    // continuations from state (px, py, pphi).
    // disc = discount factor accumulated up to this call level.
    std::function<float(float, float, float, int, float)> eval_tree;
    eval_tree = [&](float px, float py, float pphi, int depth, float disc) -> float
    {
        if (depth >= tree_depth) return 0.f;
        float min_g = std::numeric_limits<float>::infinity();
        for (const auto &a : actions)
        {
            const float c  = std::cos(pphi);
            const float sp = std::sin(pphi);
            const float nx   = px + (a.advx * c  - a.advz * sp) * dt;
            const float ny   = py + (a.advx * sp + a.advz * c ) * dt;
            const float nphi = wrap_pi(pphi - a.rot * dt);

            const float ddx = nx - hot_eff.x(), ddy = ny - hot_eff.y();
            const float risk      = 0.5f * (ddx*ddx + ddy*ddy) / sigma2;
            // Epistemic: negative info-gain (lower G = more informative pose)
            const float epistemic = -params.EFE_INFO_GAIN_WEIGHT * predict_info_gain(nx, ny, nphi);
            const float effort    = params.EFE_CONTROL_WEIGHT *
                                    (a.advx*a.advx + a.advz*a.advz + 0.5f*a.rot*a.rot);
            const float g_step    = disc * (risk + epistemic + effort);
            min_g = std::min(min_g, g_step + eval_tree(nx, ny, nphi, depth + 1, disc * discount));
        }
        return min_g;
    };

    float best_g = std::numeric_limits<float>::infinity();
    int best_idx = static_cast<int>(actions.size()) - 1;

    rc::Viewer2D::TrajDrawData traj_debug;
    traj_debug.trajectories.resize(actions.size());
    traj_debug.carrot = hot_eff;
    traj_debug.robot_x = x0;
    traj_debug.robot_y = y0;

    for (int ai = 0; ai < static_cast<int>(actions.size()); ++ai)
    {
        const auto &a = actions[ai];

        // Root step (depth 0).
        const float c  = std::cos(phi0);
        const float s  = std::sin(phi0);
        const float nx   = x0 + (a.advx * c  - a.advz * s) * dt;
        const float ny   = y0 + (a.advx * s  + a.advz * c) * dt;
        const float nphi = wrap_pi(phi0 - a.rot * dt);

        const float ddx = nx - hot_eff.x(), ddy = ny - hot_eff.y();
        const float risk      = 0.5f * (ddx*ddx + ddy*ddy) / sigma2;
        const float epistemic = -params.EFE_INFO_GAIN_WEIGHT * predict_info_gain(nx, ny, nphi);
        const float effort    = params.EFE_CONTROL_WEIGHT *
                                (a.advx*a.advx + a.advz*a.advz + 0.5f*a.rot*a.rot);
        // G(root action ai) = root step cost + minimum G over depth-1 subtree.
        const float g_root = risk + epistemic + effort
                           + eval_tree(nx, ny, nphi, 1, discount);

        if (g_root < best_g)
        {
            best_g = g_root;
            best_idx = ai;
        }

        // Build visualization trajectory: greedy single-step lookahead from root step.
        auto &traj = traj_debug.trajectories[ai];
        traj.clear();
        traj.emplace_back(x0, y0);
        traj.emplace_back(nx, ny);
        float vx = nx, vy = ny, vphi = nphi;
        for (int k = 2; k <= tree_depth; ++k)
        {
            float best_local = std::numeric_limits<float>::infinity();
            float bx = vx, by = vy, bphi = vphi;
            for (const auto &va : actions)
            {
                const float vc = std::cos(vphi), vs = std::sin(vphi);
                const float tx = vx + (va.advx * vc - va.advz * vs) * dt;
                const float ty = vy + (va.advx * vs + va.advz * vc) * dt;
                const float tp = wrap_pi(vphi - va.rot * dt);
                const float tdx = tx - hot_eff.x(), tdy = ty - hot_eff.y();
                const float tg  = 0.5f * (tdx*tdx + tdy*tdy) / sigma2
                                 - params.EFE_INFO_GAIN_WEIGHT * predict_info_gain(tx, ty, tp);
                if (tg < best_local) { best_local = tg; bx = tx; by = ty; bphi = tp; }
            }
            traj.emplace_back(bx, by);
            vx = bx; vy = by; vphi = bphi;
        }
    }

    const Eigen::Vector2f cur{x0, y0};
    const float target_dist = (hot_eff - cur).norm();
    if (target_dist > params.EFE_TURN_ONLY_MIN_DIST && (best_idx == 1 || best_idx == 2))
    {
        best_idx = (best_idx == 1) ? 3 : 4;
    }

    traj_debug.best_idx = best_idx;
    if (viewer_2d_ != nullptr)
    {
        viewer_2d_->draw_trajectory_debug(traj_debug);
        viewer_2d_->invalidate();
    }

    const Action &best = actions[best_idx];

    try
    {
        send_base_command(best.advx, best.advz, best.rot);
        efe_hotzone_active_ = true;

        static std::int64_t last_efe_log_ms = 0;
        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        if (now_ms - last_efe_log_ms >= 1000)
        {
            qInfo() << "[EFE] hot-zone global=" << hot.x() << hot.y()
                << " local=" << hot_eff.x() << hot_eff.y()
                        << " action=" << best.name
                        << " depth=" << tree_depth
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
