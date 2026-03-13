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
#include "object_models/table_analytic_model.h"
#include <QSplitter>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QHash>
#include <QFileDialog>
#include <QTextStream>
#include <QRegularExpression>
#include <QDateTime>
#include <QDomDocument>
#include <QSettings>
#include <QCoreApplication>
#include <QEventLoop>
#include <QThread>
#include <QDataStream>
#include <QShortcut>
#include <unistd.h>  // For sysconf
#include <cmath>     // For std::fabs
#include <limits>    // For std::numeric_limits
#include <print>     // C++23 std::println
#include <random>    // For odometry noise
#include <numeric>
#include <algorithm>
#include <cstring>
#include <unordered_map>
#include <set>
#include <sstream>
#include <optional>
#include <array>
#include <functional>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
        this->startup_check_flag = startup_check;
        if (this->startup_check_flag)
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
            if (error.length() > 0)
            {
                qWarning() << error;
                throw error;
            }
        }
}

SpecificWorker::~SpecificWorker()
{
	qInfo() << "Destroying SpecificWorker";

    // Stop state machine timers to prevent compute/emergency callbacks during teardown
    statemachine.stop();

    // Stop background threads first
    stop_localization_thread_ = true;
    stop_lidar_thread = true;
    if (localization_th_.joinable())
    {
        localization_th_.join();
    }
    if (read_lidar_th.joinable())
    {
        read_lidar_th.join();
    }

	// Save window geometry for next launch
	QSettings settings("robocomp", "ainf_slamo");
	settings.setValue("window/geometry", this->saveGeometry());

	// Save 2D viewer zoom
	if (viewer)
	{
		QByteArray ba;
		QDataStream ds(&ba, QIODevice::WriteOnly);
		ds << viewer->transform();
		settings.setValue("viewer2d/transform", ba);
	}
	// Save splitter proportions
    if (splitter_)
    {
        settings.setValue("splitter/state", splitter_->saveState());
    }
    if (right_splitter_)
    {
        settings.setValue("right_splitter/state", right_splitter_->saveState());
    }
	// Save 3D camera pose
    save_camera_state_to_settings();

	// Stop trajectory controller
	if (trajectory_controller_.is_active())
	{
		trajectory_controller_.stop();
		try { omnirobot_proxy->setSpeedBase(0, 0, 0); } catch (...) {}
	}

	// Save the last known pose for fast restart
	save_last_pose();
}

void SpecificWorker::initialize()
{
    qInfo() << "Initializing worker";

    // Initialize CPU usage tracking
    num_processors_ = sysconf(_SC_NPROCESSORS_ONLN);
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    last_user_cpu_time_ = usage.ru_utime.tv_sec * 1000000 + usage.ru_utime.tv_usec;
    last_sys_cpu_time_ = usage.ru_stime.tv_sec * 1000000 + usage.ru_stime.tv_usec;

    // Viewer
    viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM, true);
    viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0.0, 0.2, QColor("Blue"));
    viewer->show();

    // Add 3D viewer alongside the 2D viewer in a horizontal splitter.
    // AbstractGraphicViewer already created a QVBoxLayout on this->frame;
    // we grab 'viewer' back from it, insert a QSplitter, and add Viewer3D.
    {
        QLayout* frame_layout = this->frame->layout();  // QVBoxLayout from AGV ctor
        frame_layout->removeWidget(viewer);
        auto* splitter = new QSplitter(Qt::Horizontal, this->frame);
        splitter->addWidget(viewer);

        viewer_3d_ = std::make_unique<rc::Viewer3D>(splitter, 2.5f);
        splitter->addWidget(viewer_3d_->container_widget());

        auto* right_splitter = new QSplitter(Qt::Vertical, splitter);
        scene_tree_ = std::make_unique<SceneTreePanel>(right_splitter);
        right_splitter->addWidget(scene_tree_.get());

        grounding_panel_ = new QWidget(right_splitter);
        auto* grounding_layout = new QVBoxLayout(grounding_panel_);
        grounding_layout->setContentsMargins(8, 6, 8, 6);
        grounding_layout->setSpacing(4);
        auto* grounding_title = new QLabel("Grounding", grounding_panel_);
        grounding_title->setStyleSheet("font-weight: bold; font-size: 10pt;");
        grounding_status_label_ = new QLabel("Status: waiting camera detections", grounding_panel_);
        grounding_cam_label_ = new QLabel("Camera object: -", grounding_panel_);
        grounding_world_label_ = new QLabel("World object: -", grounding_panel_);
        grounding_score_label_ = new QLabel("Score: -", grounding_panel_);
        grounding_sdf_label_ = new QLabel("SDF fit: -", grounding_panel_);
        grounding_fit_mesh_button_ = new QPushButton("Fit Grounded Mesh", grounding_panel_);
        grounding_reload_svg_button_ = new QPushButton("Reload Meshes from SVG", grounding_panel_);

        grounding_panel_->setMinimumWidth(0);
        grounding_panel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        grounding_status_label_->setMinimumWidth(0);
        grounding_cam_label_->setMinimumWidth(0);
        grounding_world_label_->setMinimumWidth(0);
        grounding_score_label_->setMinimumWidth(0);
        grounding_sdf_label_->setMinimumWidth(0);
        grounding_status_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        grounding_cam_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        grounding_world_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        grounding_score_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        grounding_sdf_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        grounding_status_label_->setWordWrap(false);
        grounding_cam_label_->setWordWrap(false);
        grounding_world_label_->setWordWrap(false);
        grounding_score_label_->setWordWrap(false);
        grounding_sdf_label_->setWordWrap(false);

        grounding_title->setVisible(false);
        grounding_status_label_->setVisible(false);
        grounding_cam_label_->setVisible(false);
        grounding_world_label_->setVisible(false);
        grounding_score_label_->setVisible(false);
        grounding_sdf_label_->setVisible(false);
        grounding_fit_mesh_button_->setVisible(false);

        grounding_layout->addWidget(grounding_reload_svg_button_);
        grounding_layout->addStretch();
        right_splitter->addWidget(grounding_panel_);
        right_splitter->setSizes({300, 140});
        right_splitter_ = right_splitter;

        splitter->addWidget(right_splitter);
        splitter->setSizes({450, 450, 220});
        splitter_ = splitter;   // store for state save/restore
        frame_layout->addWidget(splitter);
        
        // Connect object picking to navigation
        connect(viewer_3d_.get(), &rc::Viewer3D::objectPicked, this, [this](const QString& n) {
            QString picked = n.trimmed();
            const int idx = picked.indexOf(" (");
            if (idx > 0)
                picked = picked.left(idx);

            qInfo() << "[Viewer3D] Selected object:" << picked;

            if (scene_tree_)
                scene_tree_->select_item_by_name(picked);

            focused_model_index_ = find_furniture_index_by_name(picked);

            this->Navigator_gotoObject(picked.toStdString());
        });

        connect(viewer_3d_.get(), &rc::Viewer3D::objectLeftClicked, this, [this](const QString& n) {
            QString picked = n.trimmed();
            const int idx = picked.indexOf(" (");
            if (idx > 0)
                picked = picked.left(idx);

            const int picked_idx = find_furniture_index_by_name(picked);
            if (picked_idx >= 0)
                focused_model_index_ = picked_idx;

            if (scene_tree_)
                scene_tree_->select_item_by_name(picked);
        });

        connect(viewer_3d_.get(), &rc::Viewer3D::objectTranslateRequested,
                this, [this](const QString& n, float dx_room, float dy_room)
        {
            translate_furniture_by_name(n, dx_room, dy_room);
        });

        connect(viewer_3d_.get(), &rc::Viewer3D::objectRotateRequested,
                this, [this](const QString& n, float angle_rad, const QVector3D& axis)
        {
            rotate_furniture_by_name(n, angle_rad, axis);
        });

        // Connect floor picking to navigation
        connect(viewer_3d_.get(), &rc::Viewer3D::floorPicked, this, [this](float x, float y) {
            RoboCompNavigator::TPoint p;
            p.x = x;
            p.y = y;
            this->Navigator_gotoPoint(p);
        });
        
        // Connect mission cancellation
        connect(viewer_3d_.get(), &rc::Viewer3D::cancelMissionRequested, this, [this]() {
            if (!trajectory_controller_.is_active() && current_path_.empty()) return;
            qInfo() << "Navigation cancelled by Ctrl+Right click (3D View)";
            active_episode_.reset();
            episode_accum_ = EpisodeAccum{};
            if (label_episodeStatus != nullptr)
            {
                label_episodeStatus->setText("EP: CANCELLED");
                label_episodeStatus->setStyleSheet("background-color: #FFE0B2; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
            }
            trajectory_controller_.stop();
            current_path_.clear();
            clear_path();
            try { omnirobot_proxy->setSpeedBase(0, 0, 0); } catch (...) {}
        });

        connect(grounding_fit_mesh_button_, &QPushButton::clicked, this, [this]()
        {
            run_camera_em_validator();
        });

        connect(grounding_reload_svg_button_, &QPushButton::clicked, this, [this]()
        {
            if (current_layout_file_.empty())
            {
                if (grounding_status_label_)
                    grounding_status_label_->setText("Status: no layout file to reload from");
                return;
            }

            if (QFile::exists(PERSISTED_SCENE_GRAPH_FILE))
            {
                if (!QFile::remove(PERSISTED_SCENE_GRAPH_FILE))
                {
                    if (grounding_status_label_)
                        grounding_status_label_->setText("Status: cannot remove scene_graph.usda");
                    return;
                }
            }

            load_layout_from_file(current_layout_file_);
            if (grounding_status_label_)
                grounding_status_label_->setText("Status: meshes reloaded from SVG (fitted overrides cleared)");
            if (grounding_sdf_label_)
                grounding_sdf_label_->setText("SDF fit: -");
        });
    }

    // Restore saved window geometry/size
    QSettings settings("robocomp", "ainf_slamo");
    if (settings.contains("window/geometry"))
        this->restoreGeometry(settings.value("window/geometry").toByteArray());

    // Restore 2D viewer zoom; on first run apply a zoomed-out default
    if (settings.contains("viewer2d/transform"))
    {
        QByteArray ba = settings.value("viewer2d/transform").toByteArray();
        QDataStream ds(&ba, QIODevice::ReadOnly);
        QTransform t;
        ds >> t;
        viewer->setTransform(t);
    }
    else
    {
        // First run: ~8 px/m so the full room fits; Y stays flipped (negative scale)
        viewer->setTransform(QTransform().scale(8.0, -8.0));
    }
    // Restore splitter proportions; fall back to defaults if any pane is collapsed
    if (splitter_ && settings.contains("splitter/state"))
    {
        splitter_->restoreState(settings.value("splitter/state").toByteArray());
        const auto sizes = splitter_->sizes();
        const bool any_zero = sizes.size() < 3 || std::any_of(sizes.begin(), sizes.end(), [](int s){ return s == 0; });
        if (any_zero)
            splitter_->setSizes({450, 450, 220});
    }
    if (right_splitter_ && settings.contains("right_splitter/state"))
    {
        right_splitter_->restoreState(settings.value("right_splitter/state").toByteArray());
        const auto rsizes = right_splitter_->sizes();
        const bool any_zero = rsizes.size() < 2 || std::any_of(rsizes.begin(), rsizes.end(), [](int s){ return s == 0; });
        if (any_zero)
            right_splitter_->setSizes({380, 140});
    }
    // Restore 3D camera pose
    if (viewer_3d_ && settings.contains("camera/position"))
    {
        rc::Viewer3D::CameraState cs;
        cs.position   = settings.value("camera/position",   QVector3D(0.f, 15.f, 15.f)).value<QVector3D>();
        cs.viewCenter = settings.value("camera/viewCenter", QVector3D(0.f,  0.f,  0.f)).value<QVector3D>();
        cs.upVector   = settings.value("camera/upVector",   QVector3D(0.f,  1.f,  0.f)).value<QVector3D>();
        viewer_3d_->set_camera_state(cs);
    }

    // Connect capture room button
    connect(pushButton_captureRoom, &QPushButton::toggled, this, &SpecificWorker::slot_capture_room_toggled);

    // Connect Ctrl+Left click on the scene for polygon capture (uses robot_rotate signal)
    connect(viewer, &AbstractGraphicViewer::robot_rotate, this, &SpecificWorker::on_scene_clicked);

    // Connect robot drag and rotate signals
    connect(viewer, &AbstractGraphicViewer::robot_dragging, this, &SpecificWorker::slot_robot_dragging);
    connect(viewer, &AbstractGraphicViewer::robot_drag_end, this, &SpecificWorker::slot_robot_drag_end);
    connect(viewer, &AbstractGraphicViewer::robot_rotate, this, &SpecificWorker::slot_robot_rotate);

    // Connect save/load layout buttons
    connect(pushButton_saveLayout, &QPushButton::clicked, this, &SpecificWorker::slot_save_layout);
    connect(pushButton_loadLayout, &QPushButton::clicked, this, &SpecificWorker::slot_load_layout);
    connect(pushButton_flipX, &QPushButton::clicked, this, &SpecificWorker::slot_flip_x);
    connect(pushButton_flipY, &QPushButton::clicked, this, &SpecificWorker::slot_flip_y);
    connect(pushButton_calibrateGT, &QPushButton::clicked, this, &SpecificWorker::slot_calibrate_gt);

    // AutoCenter toggle
    connect(pushButton_autoCenter, &QPushButton::toggled, this, [this](bool checked) { auto_center_ = checked; });

    // Lidar / Trajectory drawing toggles
    connect(pushButton_showLidar, &QPushButton::toggled, this, [this](bool checked) { draw_lidar_ = checked; });
    connect(pushButton_showTrajs, &QPushButton::toggled, this, [this](bool checked) { draw_trajectories_ = checked; });

    // Camera viewer popup
    camera_viewer_ = std::make_unique<CameraViewer>(imagesegmentation_proxy, this);
    camera_viewer_->set_period_ms(50);
    if (pushButton_camera != nullptr)
    {
        connect(pushButton_camera, &QPushButton::toggled, this, [this](bool checked) {
            if (checked) camera_viewer_->show();
            else         camera_viewer_->hide();
        });
        connect(camera_viewer_.get(), &CameraViewer::emRequested, this, [this]() {
            if (em_overlay_persistent_active_)
            {
                em_overlay_persistent_active_ = false;
                em_validator_overlay_lock_ = false;
                em_decision_pending_ = false;
                pending_em_adjustments_.clear();
                camera_viewer_->set_em_decision_buttons_visible(false);
                camera_viewer_->clear_wireframe_overlay();
                camera_viewer_->clear_em_points_overlay();
                if (grounding_status_label_)
                    grounding_status_label_->setText("Status: EM overlay cleared");
                if (grounding_sdf_label_)
                    grounding_sdf_label_->setText("SDF fit: -");
            }
            else
                run_camera_em_validator();
        });
        connect(camera_viewer_.get(), &CameraViewer::emAcceptRequested, this, [this]() {
            apply_pending_em_adjustments(true);
        });
        connect(camera_viewer_.get(), &CameraViewer::emRejectRequested, this, [this]() {
            apply_pending_em_adjustments(false);
        });
        connect(camera_viewer_.get(), &QDialog::finished, this, [this](int) {
            if (pushButton_camera) pushButton_camera->setChecked(false);
        });
    }

    if (scene_tree_)
    {
        connect(scene_tree_.get(), &SceneTreePanel::furnitureClicked, this, [this](const QString& name, bool selected)
        {
            if (selected)
            {
                focused_model_index_ = find_furniture_index_by_name(name);
                if (viewer_3d_)
                    viewer_3d_->set_selected_object_for_gizmo(name);
            }
            else if (focused_model_index_ == find_furniture_index_by_name(name))
            {
                focused_model_index_ = -1;
                if (viewer_3d_)
                    viewer_3d_->clear_selected_object_for_gizmo();
            }
        });
    }

    // PD / MPPI mode toggle
    connect(pushButton_pdMode, &QPushButton::toggled, this, [this](bool checked)
    {
        if (checked)
        {
            trajectory_controller_.set_control_mode(rc::TrajectoryController::ControlMode::PD);
            pushButton_pdMode->setStyleSheet("background-color: #FFF176; font-weight: bold;");
            qInfo() << "Controller mode: PD carrot-follower";
        }
        else
        {
            trajectory_controller_.set_control_mode(rc::TrajectoryController::ControlMode::MPPI);
            pushButton_pdMode->setStyleSheet("");
            qInfo() << "Controller mode: MPPI";
        }
    });

    // Mood slider (0..100 mapped to 0.0..1.0) for trajectory controller behavior
    if (horizontalSlider_mood != nullptr)
    {
        horizontalSlider_mood->setRange(0, 100);
        horizontalSlider_mood->setSingleStep(1);
        horizontalSlider_mood->setPageStep(5);

        connect(horizontalSlider_mood, &QSlider::valueChanged, this, [this](int value)
        {
            const float mood = static_cast<float>(value) / 100.f;
            trajectory_controller_.params.mood = mood;
            if (label_moodValue != nullptr)
                label_moodValue->setText(QString::number(mood, 'f', 2));
        });

        horizontalSlider_mood->setValue(50);  // init value = 0.50
    }
    trajectory_controller_.params.mood = 0.5f;

    if (label_episodeStatus != nullptr)
    {
        label_episodeStatus->setText("EP: IDLE");
        label_episodeStatus->setStyleSheet("background-color: #CFD8DC; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
    }
    if (label_safetyGuard != nullptr)
    {
        label_safetyGuard->setText("SG: OFF");
        label_safetyGuard->setStyleSheet("background-color: #CFD8DC; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
    }

    // Connect Shift+Right click on the scene for navigation target
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::slot_new_target);

    // Ctrl+Right click on scene: cancel current navigation mission
    connect(viewer, &AbstractGraphicViewer::right_click, this, [this](QPointF)
    {
        if (!trajectory_controller_.is_active() && current_path_.empty()) return;
        qInfo() << "Navigation cancelled by Ctrl+Right click";
        active_episode_.reset();
        episode_accum_ = EpisodeAccum{};
        if (label_episodeStatus != nullptr)
        {
            label_episodeStatus->setText("EP: CANCELLED");
            label_episodeStatus->setStyleSheet("background-color: #FFE0B2; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
        }
        trajectory_controller_.stop();
        current_path_.clear();
        clear_path();
        try { omnirobot_proxy->setSpeedBase(0, 0, 0); } catch (...) {}
    });

    // Ctrl+Shift+Right keyboard: cancel current navigation mission without saving episode
    auto *cancel_shortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_Right), this);
    connect(cancel_shortcut, &QShortcut::activated, this, [this]()
    {
        if (!trajectory_controller_.is_active() && current_path_.empty()) return;
        qInfo() << "Navigation cancelled by Ctrl+Shift+Right";
        // Discard episode without saving
        active_episode_.reset();
        episode_accum_ = EpisodeAccum{};
        if (label_episodeStatus != nullptr)
        {
            label_episodeStatus->setText("EP: CANCELLED");
            label_episodeStatus->setStyleSheet("background-color: #FFE0B2; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
        }
        // Stop controller and clear path
        trajectory_controller_.stop();
        current_path_.clear();
        try { omnirobot_proxy->setSpeedBase(0, 0, 0); } catch (...) {}
    });

    // Try to load default layout on startup (only loads vertices, doesn't init room_ai yet)
    // Config keys: layout_file (SVG path)
    std::string layout_file = "";
    try { layout_file  = configLoader.get<std::string>("layout_file");  } catch(...) {}
    try { params.USE_WEBOTS = configLoader.get<bool>("use_webots"); } catch(...) {}
    try { params.CAMERA_TX = static_cast<float>(configLoader.get<double>("camera_tx")); } catch(...) {}
    try { params.CAMERA_TY = static_cast<float>(configLoader.get<double>("camera_ty")); } catch(...) {}
    try { params.CAMERA_TZ = static_cast<float>(configLoader.get<double>("camera_tz")); } catch(...) {}
    qInfo() << "Segmented camera extrinsics (tx, ty, tz):"
        << params.CAMERA_TX << params.CAMERA_TY << params.CAMERA_TZ;
    load_polygon_from_file(layout_file);

    // Show mode indicator in UI
    if (params.USE_WEBOTS)
    {
        label_mode->setText("WEBOTS");
        label_mode->setStyleSheet("background-color: #87CEEB; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
    }
    else
    {
        label_mode->setText("ROBOT");
        label_mode->setStyleSheet("background-color: #FF8C00; color: white; padding: 2px; border-radius: 2px; font-size: 8pt;");
        pushButton_calibrateGT->setVisible(false);
        lcdNumber_gt_xy_err->setVisible(false);
        lcdNumber_gt_ang_err->setVisible(false);
        label_gt_xy_err->setVisible(false);
        label_gt_ang_err->setVisible(false);
        qInfo() << "Webots disabled (use_webots=false). Running in real robot mode.";
    }


    // Lidar thread is created
    read_lidar_th = std::thread(&SpecificWorker::read_lidar,this);
    qInfo() << __FUNCTION__ << "Started lidar reader";

    // Wait for first lidar so we can initialize the concept with a data-driven guess
    // GT pose from webots (robot) is optional — used only for debug/statistics
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    int startup_check_counter = 0;
    std::optional<Eigen::Affine2f> robot;
    std::optional<std::pair<std::vector<Eigen::Vector3f>, std::int64_t>> lidar_local;
    do
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const auto &[r, ll, ll_low] = buffer_sync.read(timestamp);
        robot = r; lidar_local = ll;
    }while (++startup_check_counter < 20 && !lidar_local.has_value());

    if (!lidar_local.has_value())
    {
        qWarning() << "initialize(): No lidar data from buffer_sync. Exiting.";
        std::terminate();
    }
    if (!robot.has_value())
        qWarning() << "initialize(): GT pose from webots not available — debug/error stats will be disabled.";

    // Default room dimensions for fallback rectangular model
    constexpr float room_width = 14.f;   // m
    constexpr float room_length = 8.f;   // m

    // Estimate room center from lidar for initial pose
    const auto &pts = lidar_local.value().first;

    rc::PointcloudCenterEstimator estimator;
    Eigen::Vector2d room_center_in_robot = Eigen::Vector2d::Zero();
    float init_phi = 0.f;
    if(const auto obb = estimator.estimate_obb(pts); obb.has_value())
    {
        room_center_in_robot = obb->center;
        init_phi = static_cast<float>(obb->rotation);
    }
    else if(const auto c = estimator.estimate(pts); c.has_value())
    {
        room_center_in_robot = c.value();
        init_phi = estimate_orientation_from_points(pts);
        qWarning() << "initialize(): estimate_obb failed. Using center + PCA orientation";
    }
    else
    {
        qWarning() << "initialize(): PointcloudCenterEstimator failed. Using (0,0,0)";
    }

    // If room center in robot frame is c_r, then robot position in room frame is p = -R(phi)*c_r.
    Eigen::Rotation2Df R(init_phi);
    const Eigen::Vector2f init_xy = -(R * room_center_in_robot.cast<float>());

    // Initialize room_ai: use loaded polygon if available, otherwise use rectangle
    if (!room_polygon_.empty())
    {
        // Use pre-loaded polygon from file
        room_ai.set_polygon_room(room_polygon_);
        path_planner_.set_polygon(room_polygon_);
        trajectory_controller_.set_room_boundary(room_polygon_);
        if (!furniture_polygons_.empty())
        {
            std::vector<std::vector<Eigen::Vector2f>> obs;
            for (const auto& fp : furniture_polygons_) obs.push_back(fp.vertices);
            path_planner_.set_obstacles(obs);
            trajectory_controller_.set_static_obstacles(obs);
        }
        draw_room_polygon();
        draw_furniture();
        qInfo() << "RoomConceptAI initialized with loaded polygon:" << room_polygon_.size() << "vertices"
                << "furniture:" << furniture_polygons_.size();

        // Perform grid search or load saved pose to solve kidnapping problem
        perform_grid_search(pts);
    }
    else
    {
        // Use default rectangle
        room_ai.set_initial_state(room_width, room_length, init_xy.x(), init_xy.y(), init_phi);
        qInfo() << "RoomConceptAI init state [w,l,x,y,phi]="
            << room_width << room_length << init_xy.x() << init_xy.y() << init_phi
            << "  (room_center_in_robot=" << room_center_in_robot.x() << room_center_in_robot.y() << ")";
    }

   // Center view around the robot initial position
    const float view_side_m = 6.f;
    const float robot_cx = init_xy.x();
    const float robot_cy = init_xy.y();
    viewer->fitToScene(QRectF(robot_cx - view_side_m/2.f, robot_cy - view_side_m/2.f, view_side_m, view_side_m));
    viewer->centerOn(robot_cx, robot_cy);

    // Start localization thread (room_ai runs independently from compute loop)
    localization_th_ = std::thread(&SpecificWorker::run_localization, this);
    qInfo() << __FUNCTION__ << "Started localization thread";

	setPeriod("Compute", 50);
}

void SpecificWorker::compute()
{
    auto t_cycle_start = std::chrono::steady_clock::now();

    const int fps_val = fps.print("Compute", 2000);
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const auto &[robot_pose_gt_, lidar_high_, lidar_low_high_] = buffer_sync.read(timestamp);

    // Check lidar availability
    if (!lidar_high_.has_value() or !lidar_low_high_.has_value())
    { qWarning() << "No lidar data from buffer_sync"; return; };

    // ===== READ LATEST LOCALIZATION RESULT FROM THREAD =====
    std::optional<rc::RoomConceptAI::UpdateResult> res_opt;
    {
        std::lock_guard lock(loc_result_mutex_);
        res_opt = loc_result_;
    }

    if (!res_opt.has_value() || !res_opt->ok)
    {
        if (!loc_initialized_.load())
            qWarning() << "Waiting for localization thread...";
        return;
    }
    const auto &res = res_opt.value();

    if (ownership_em_enabled_ && lidar_low_high_.has_value())
        run_ownership_em_step(lidar_low_high_->first, res.robot_pose);

    if (camera_viewer_)
    {
        camera_viewer_->set_infrastructure_context(res.robot_pose,
                                                   room_polygon_,
                                                   params.CAMERA_TX,
                                                   params.CAMERA_TY,
                                                   params.CAMERA_TZ,
                                                   2.5f);
        if (!em_validator_overlay_lock_)
            update_camera_wireframe_overlay(res.robot_pose);
    }

    auto t0 = std::chrono::steady_clock::now();

    // Visualization at 10 Hz (every other frame), compute stays at 20 Hz
    static int viz_frame = 0;
    const bool do_draw = (++viz_frame % 2 == 0);

    if (do_draw)
    {
        // Get latest velocity for UI display
        const auto vel_tuple = velocity_buffer_.read_last<0>();
        const auto &vel_opt = std::get<0>(vel_tuple);
        const rc::VelocityCommand current_vel = vel_opt.has_value() ? vel_opt.value() : rc::VelocityCommand();

        update_ui(res, current_vel, fps_val);
        display_robot(res.robot_pose, res.covariance);
        if (draw_lidar_)
            draw_lidar_points(lidar_high_->first, lidar_low_high_->first, res.robot_pose);
        else
            draw_lidar_points({}, {}, res.robot_pose);  // clear drawn points

        if (viewer_3d_)
        {
            if (draw_lidar_)
            {
                const float rpx = res.robot_pose.translation().x();
                const float rpy = res.robot_pose.translation().y();
                const float rpt = std::atan2(res.robot_pose.linear()(1,0),
                                             res.robot_pose.linear()(0,0));
                viewer_3d_->update_lidar_points(lidar_high_->first, lidar_low_high_->first,
                                                rpx, rpy, rpt);
            }
            else
                viewer_3d_->update_lidar_points({}, {}, 0.f, 0.f, 0.f);

            viewer_3d_->update_segmented_points({});
            viewer_3d_->update_segmented_boxes({});
        }

        if (params.USE_WEBOTS)
        {
            if (!gt_calibrated_ && robot_pose_gt_.has_value() && res.sdf_mse < 0.10f)
                calibrate_gt_offset(res.robot_pose, robot_pose_gt_.value());
            display_gt_error(res.robot_pose, robot_pose_gt_);
        }

        draw_estimated_room(res.state);
    }

    auto t_viz1 = std::chrono::steady_clock::now();

    // Save pose periodically (every ~30 seconds at 20Hz = 600 frames)
    static int pose_save_counter = 0;
    if (++pose_save_counter >= 600)
    {
        save_last_pose();
        save_camera_state_to_settings();
        pose_save_counter = 0;
    }

    // Clean up expired temporary obstacles (every ~5 seconds at 20Hz = 100 frames)
    static int temp_obs_cleanup_counter = 0;
    if (++temp_obs_cleanup_counter >= 100)
    {
        temp_obs_cleanup_counter = 0;
        if (!temp_obstacles_.empty())
            cleanup_temp_obstacles();
    }

    // ===== LOCAL TRAJECTORY CONTROLLER =====
    // Run if a path is active and joystick is not overriding
    float mppi_ms = 0.f, viz2_ms = 0.f;
    if (trajectory_controller_.is_active())
    {
        auto t2 = std::chrono::steady_clock::now();
        const auto ctrl = trajectory_controller_.compute(lidar_low_high_->first, res.robot_pose);
        auto t3 = std::chrono::steady_clock::now();
        mppi_ms = std::chrono::duration<float, std::milli>(t3 - t2).count();

        // Store ESS for UI
        last_ess_ = ctrl.ess;
        last_ess_K_ = ctrl.ess_K;
        if (ctrl.safety_guard_triggered)
            last_safety_guard_trigger_time_ = std::chrono::steady_clock::now();

        if (ctrl.goal_reached)
        {
            const bool goto_object = active_episode_.has_value()
                                  && active_episode_->mission.mission_type == "goto_object"
                                  && nav_target_object_center_.has_value();

            if (goto_object)
            {
                // Switch to dedicated final heading alignment mode.
                trajectory_controller_.stop();
                current_path_.clear();
                object_final_align_active_ = true;
                object_align_cycles_ = 0;
                send_velocity_command(0.f, 0.f, 0.f);
                auto cmd = rc::VelocityCommand(0.f, 0.f, 0.f);
                velocity_buffer_.put<0>(std::move(cmd), timestamp);
                if (label_episodeStatus != nullptr)
                {
                    label_episodeStatus->setText("EP: ALIGNING OBJECT");
                    label_episodeStatus->setStyleSheet("background-color: #BBDEFB; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
                }
                return;
            }

            object_align_cycles_ = 0;
            nav_target_object_center_.reset();
            nav_target_object_name_.clear();
            finish_episode("success");
            send_velocity_command(0.f, 0.f, 0.f);
            auto cmd = rc::VelocityCommand(0.f, 0.f, 0.f);
            velocity_buffer_.put<0>(std::move(cmd), timestamp);
            clear_path();
            qInfo() << "[TrajectoryCtrl] Navigation complete.";
        }
        else
        {
            const float speed = std::hypot(ctrl.adv, ctrl.side);
            const float rot = std::fabs(ctrl.rot);
            const float cpu_usage = get_cpu_usage();
            const bool blocked_like = (ctrl.dist_to_goal > trajectory_controller_.params.goal_threshold)
                                   && (speed < BLOCKED_SPEED_THRESHOLD);

            if (ctrl.path_blocked)
            {
                safeguard_blockage_center_ = ctrl.blockage_center_room;
                safeguard_blockage_radius_ = std::max(0.20f, ctrl.blockage_radius);
            }

            if (!safeguard_recovery_active_ && ctrl.safety_guard_triggered && blocked_like)
            {
                safeguard_recovery_active_ = true;
                safeguard_replan_pending_ = true;
                safeguard_recovery_cycles_ = 0;
                safeguard_clear_counter_ = 0;
                qWarning() << "[SafeguardRecovery] Triggered: starting backward recovery";
            }

            bool recovering_now = false;
            if (safeguard_recovery_active_)
            {
                recovering_now = true;
                safeguard_recovery_cycles_++;
                if (ctrl.safety_guard_triggered)
                    safeguard_clear_counter_ = 0;
                else
                    safeguard_clear_counter_++;

                const bool recovered = (safeguard_clear_counter_ >= SAFEGUARD_CLEAR_CONFIRM_CYCLES);
                const bool timeout = (safeguard_recovery_cycles_ >= SAFEGUARD_RECOVERY_MAX_CYCLES);

                if (!recovered && !timeout)
                {
                    send_velocity_command(SAFEGUARD_BACKWARD_SPEED, 0.f, 0.f);
                    auto cmd = rc::VelocityCommand(0.f, SAFEGUARD_BACKWARD_SPEED, 0.f);
                    velocity_buffer_.put<0>(std::move(cmd), timestamp);
                }
                else
                {
                    safeguard_recovery_active_ = false;
                    safeguard_recovery_cycles_ = 0;
                    safeguard_clear_counter_ = 0;

                    send_velocity_command(0.f, 0.f, 0.f);
                    auto stop_cmd = rc::VelocityCommand(0.f, 0.f, 0.f);
                    velocity_buffer_.put<0>(std::move(stop_cmd), timestamp);

                    if (safeguard_replan_pending_ && lidar_low_high_.has_value())
                    {
                        float obs_height = 0.f;
                        const auto polygon = cluster_lidar_to_polygon(
                            lidar_low_high_->first,
                            safeguard_blockage_center_,
                            std::min(safeguard_blockage_radius_ + 0.15f, 1.1f),
                            res.robot_pose,
                            obs_height);

                        if (!polygon.empty())
                            replan_around_obstacle(polygon, obs_height, safeguard_blockage_center_, res.robot_pose);
                        else
                            qWarning() << "[SafeguardRecovery] Recovery ended but obstacle cluster too sparse for replan";
                    }
                    safeguard_replan_pending_ = false;
                    recovering_now = false;
                }
            }

            if (!recovering_now)
            {
                send_velocity_command(ctrl.adv, ctrl.side, ctrl.rot);
                auto cmd = rc::VelocityCommand(ctrl.side, ctrl.adv, ctrl.rot);
                velocity_buffer_.put<0>(std::move(cmd), timestamp);

                // ── Obstacle avoidance: detect blockage → cluster → replan ──
                if (ctrl.path_blocked && lidar_low_high_.has_value())
                {
                    float obs_height = 0.f;
                    const auto polygon = cluster_lidar_to_polygon(
                        lidar_low_high_->first,
                        ctrl.blockage_center_room,
                        std::min(ctrl.blockage_radius + 0.1f, 1.0f),  // tight search, capped at 1m
                        res.robot_pose,
                        obs_height);

                    if (!polygon.empty())
                    {
                        replan_around_obstacle(polygon, obs_height, ctrl.blockage_center_room, res.robot_pose);
                    }
                    else
                    {
                        qWarning() << "[ObstacleAvoid] Blockage detected but LiDAR cluster too sparse to model";
                    }
                }
            }

            update_episode_metrics(res, &ctrl, speed, rot, cpu_usage, mppi_ms, blocked_like || recovering_now);

            if (do_draw && draw_trajectories_) draw_trajectory_debug(ctrl, res.robot_pose);
        }
        viz2_ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t3).count();
    }

    // Dedicated final object-facing alignment phase (decoupled from trajectory controller).
    if (object_final_align_active_ && nav_target_object_center_.has_value())
    {
        // Recompute object center in world each cycle when available.
        Eigen::Vector2f object_world = nav_target_object_center_.value();
        if (!nav_target_object_name_.empty())
        {
            const QString target_name = QString::fromStdString(nav_target_object_name_).trimmed();
            for (const auto& fp : furniture_polygons_)
            {
                const QString flabel = QString::fromStdString(fp.label).trimmed();
                const QString fid = QString::fromStdString(fp.id).trimmed();
                if ((flabel.compare(target_name, Qt::CaseInsensitive) == 0 ||
                        fid.compare(target_name, Qt::CaseInsensitive) == 0) &&
                    !fp.vertices.empty())
                {
                    Eigen::Vector2f c = Eigen::Vector2f::Zero();
                    for (const auto& v : fp.vertices) c += v;
                    c /= static_cast<float>(fp.vertices.size());
                    object_world = c;
                    nav_target_object_center_ = c;
                    break;
                }
            }
        }

        const auto loc_state = get_loc_state();
        const float rx = loc_state[2];
        const float ry = loc_state[3];
        const float rth = loc_state[4];
        const float cr = std::cos(rth);
        const float sr = std::sin(rth);
        const Eigen::Vector2f d_world = object_world - Eigen::Vector2f(rx, ry);
        const Eigen::Vector2f object_robot(cr * d_world.x() + sr * d_world.y(),
                                           -sr * d_world.x() + cr * d_world.y());

        constexpr float kYawTol = 0.08f;
        constexpr int kMaxAlignCycles = 70;
        static float prev_angle_err = 0.f;
        const float angle_err = std::atan2(object_robot.x(), object_robot.y());

        const bool done = (std::abs(angle_err) <= kYawTol)
                       || (object_robot.norm() <= 0.05f)
                       || (object_align_cycles_ >= kMaxAlignCycles)
                       || (object_align_cycles_ > 3 && std::signbit(angle_err) != std::signbit(prev_angle_err)
                           && std::abs(angle_err) < 0.18f);

        if (done)
        {
            object_final_align_active_ = false;
            object_align_cycles_ = 0;
            nav_target_object_center_.reset();
            nav_target_object_name_.clear();
            send_velocity_command(0.f, 0.f, 0.f);
            auto stop_cmd = rc::VelocityCommand(0.f, 0.f, 0.f);
            velocity_buffer_.put<0>(std::move(stop_cmd), timestamp);
            finish_episode("success");
            clear_path();
            qInfo() << "[TrajectoryCtrl] Navigation complete after final object alignment.";
        }
        else
        {
            const float kP = 1.2f;
            float rot_cmd = std::clamp(kP * angle_err, -0.45f, 0.45f);
            if (std::abs(rot_cmd) < 0.06f)
                rot_cmd = (angle_err > 0.f) ? 0.06f : -0.06f;

            send_velocity_command(0.f, 0.f, rot_cmd);
            auto cmd = rc::VelocityCommand(0.f, 0.f, rot_cmd);
            velocity_buffer_.put<0>(std::move(cmd), timestamp);
            prev_angle_err = angle_err;
            object_align_cycles_++;
            return;
        }
    }

    // Per-subsystem timing (every ~2 seconds at 20Hz)
    static int timing_counter = 0;
    if (++timing_counter >= 40)
    {
        timing_counter = 0;
        auto t_cycle_end = std::chrono::steady_clock::now();
        const float viz_ms = std::chrono::duration<float, std::milli>(t_viz1 - t0).count() + viz2_ms;
        const float cycle_ms = std::chrono::duration<float, std::milli>(t_cycle_end - t_cycle_start).count();
        const float cpu_total = get_cpu_usage();
        const float cycle_pct = cycle_ms / 50.f * 100.f;
        std::println("[TIMING] Viz: {:.1f} ms | MPPI: {:.1f} ms | Cycle: {:.1f} ms ({}%) | Lidar+Loc+other: ~{}%",
                     viz_ms, mppi_ms, cycle_ms,
                     static_cast<int>(cycle_pct),
                     static_cast<int>(cpu_total - cycle_pct));
    }
}

void SpecificWorker::save_camera_state_to_settings() const
{
    if (!viewer_3d_)
        return;

    QSettings settings("robocomp", "ainf_slamo");
    const auto cs = viewer_3d_->camera_state();
    settings.setValue("camera/position", cs.position);
    settings.setValue("camera/viewCenter", cs.viewCenter);
    settings.setValue("camera/upVector", cs.upVector);
}

void SpecificWorker::rebuild_ownership_em_models()
{
    if (!ownership_em_enabled_)
        return;

    rc::ObjectOwnershipEM::Config cfg;
    cfg.max_iterations = 3;
    cfg.convergence_eps = 1e-3f;
    cfg.anneal_t_start = 2.0f;
    cfg.anneal_t_end = 0.8f;
    cfg.robust_sigma = 0.30f;
    cfg.max_pose_jump_xy = 0.30f;
    cfg.max_pose_jump_yaw = 0.20f;
    ownership_em_.configure(cfg);

    std::vector<rc::ObjectOwnershipEM::ClassModel> models;
    models.reserve(furniture_polygons_.size() + 1);
    std::vector<rc::ObjectOwnershipEM::ClassState> states;
    states.reserve(furniture_polygons_.size() + 1);

    auto expected_height = [](const std::string& label) -> float
    {
        const QString ql = QString::fromStdString(label).toLower();
        if (ql.contains("silla") || ql.contains("chair")) return 0.95f;
        if (ql.contains("mesa") || ql.contains("table")) return 0.75f;
        if (ql.contains("banco") || ql.contains("bench")) return 0.52f;
        if (ql.contains("monitor") || ql.contains("pantalla") || ql.contains("screen")) return 1.10f;
        return 0.85f;
    };

    for (const auto& fp : furniture_polygons_)
    {
        if (fp.vertices.size() < 3)
            continue;

        Eigen::Vector2f cen = Eigen::Vector2f::Zero();
        for (const auto& v : fp.vertices) cen += v;
        cen /= static_cast<float>(fp.vertices.size());

        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (const auto& v : fp.vertices)
        {
            const Eigen::Vector2f d = v - cen;
            cov += d * d.transpose();
        }
        cov /= static_cast<float>(fp.vertices.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
        Eigen::Vector2f axis_x(1.f, 0.f);
        if (eig.info() == Eigen::Success)
            axis_x = eig.eigenvectors().col(1).normalized();

        rc::ObjectOwnershipEM::ClassModel m;
        m.class_id = fp.label.empty() ? fp.id : fp.label;
        m.prior_weight = 1.0f;
        m.expected_height = expected_height(fp.label.empty() ? fp.id : fp.label);
        models.emplace_back(std::move(m));

        rc::ObjectOwnershipEM::ClassState s;
        s.class_id = fp.label.empty() ? fp.id : fp.label;
        s.position = Eigen::Vector3f(cen.x(), cen.y(), 0.5f * expected_height(fp.label.empty() ? fp.id : fp.label));
        s.yaw_rad = std::atan2(axis_x.y(), axis_x.x());
        s.scale = 1.0f;
        s.confidence = 0.5f;
        s.active = true;
        states.emplace_back(std::move(s));
    }

    rc::ObjectOwnershipEM::ClassModel bg;
    bg.class_id = "background";
    bg.is_background = true;
    bg.prior_weight = 0.8f;
    bg.expected_height = 0.0f;
    models.emplace_back(std::move(bg));

    rc::ObjectOwnershipEM::ClassState bg_state;
    bg_state.class_id = "background";
    bg_state.active = true;
    bg_state.confidence = 0.5f;
    states.emplace_back(std::move(bg_state));

    ownership_em_.set_models(models);
    ownership_em_.set_initial_states(states);
}

void SpecificWorker::run_ownership_em_step(const std::vector<Eigen::Vector3f>& points_robot,
                                           const Eigen::Affine2f& robot_pose)
{
    if (!ownership_em_enabled_ || points_robot.empty())
        return;

    rc::ObjectOwnershipEM::ObservationBatch batch;
    batch.robot_pose = robot_pose;
    batch.timestamp_ms = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    const float c = std::cos(std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0)));
    const float s = std::sin(std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0)));
    const float rx = robot_pose.translation().x();
    const float ry = robot_pose.translation().y();

    constexpr std::size_t max_points = 1800;
    const std::size_t stride = std::max<std::size_t>(1, points_robot.size() / max_points);
    batch.points_world.reserve(points_robot.size() / stride + 1);

    for (std::size_t i = 0; i < points_robot.size(); i += stride)
    {
        const auto& p = points_robot[i];
        const float wx = c * p.x() - s * p.y() + rx;
        const float wy = s * p.x() + c * p.y() + ry;
        batch.points_world.emplace_back(wx, wy, p.z());
    }

    ownership_em_.set_observations(batch);
    if (!ownership_em_.run_single_iteration())
        return;

    apply_ownership_em_visuals();

    if (++ownership_em_log_counter_ >= 40)
    {
        ownership_em_log_counter_ = 0;
        const auto& m = ownership_em_.get_metrics();
        qInfo() << "[OwnershipEM] objective=" << m.objective
                << "entropy=" << m.avg_entropy
                << "bg=" << m.background_fraction;
    }
}

void SpecificWorker::run_camera_em_validator()
{
    if (!camera_viewer_)
        return;

    em_decision_pending_ = false;
    pending_em_adjustments_.clear();
    camera_viewer_->set_em_decision_buttons_visible(false);

    em_validator_overlay_lock_ = true;
    bool keep_overlay_lock = false;
    struct OverlayUnlock
    {
        bool &flag;
        bool &keep;
        ~OverlayUnlock() { if (!keep) flag = false; }
    } overlay_unlock{em_validator_overlay_lock_, keep_overlay_lock};
    em_overlay_persistent_active_ = false;
    camera_viewer_->clear_em_points_overlay();

    if (grounding_status_label_)
        grounding_status_label_->setText("Status: running EM validator...");

    std::optional<rc::RoomConceptAI::UpdateResult> res_opt;
    {
        std::lock_guard lock(loc_result_mutex_);
        res_opt = loc_result_;
    }
    if (!res_opt.has_value() || !res_opt->ok)
    {
        if (grounding_status_label_)
            grounding_status_label_->setText("Status: EM validator needs localization");
        return;
    }
    const Eigen::Affine2f robot_pose = res_opt->robot_pose;

    auto centroid_of = [](const rc::FurniturePolygonData& fp) -> Eigen::Vector2f
    {
        Eigen::Vector2f c = Eigen::Vector2f::Zero();
        if (fp.vertices.empty())
            return c;
        for (const auto& v : fp.vertices) c += v;
        c /= static_cast<float>(fp.vertices.size());
        return c;
    };

    try
    {
        const auto tdata = imagesegmentation_proxy->getAll(false);

        std::vector<Eigen::Vector3f> points_cam_all;
        std::vector<Eigen::Vector3f> points_world_all;
        points_cam_all.reserve(12000);
        points_world_all.reserve(12000);

        int depth_w = 0;
        int depth_h = 0;
        std::vector<float> depth_m;
        const bool have_depth = extract_depth_from_tdata(tdata, depth_w, depth_h, depth_m);

        const auto &timg = tdata.image;
        const int img_w = std::max(1, timg.width);
        const int img_h = std::max(1, timg.height);
        float fx = static_cast<float>(timg.focalx);
        float fy = static_cast<float>(timg.focaly);
        const float max_reasonable_fx = static_cast<float>(img_w) * 5.f;
        const float max_reasonable_fy = static_cast<float>(img_h) * 5.f;
        if (!std::isfinite(fx) || fx < 10.f || fx > max_reasonable_fx) fx = static_cast<float>(img_w) * 0.9f;
        if (!std::isfinite(fy) || fy < 10.f || fy > max_reasonable_fy) fy = static_cast<float>(img_h) * 0.9f;

        if (have_depth && depth_w > 0 && depth_h > 0)
        {
            const float cx_d = static_cast<float>(depth_w) * 0.5f;
            const float cy_d = static_cast<float>(depth_h) * 0.5f;
            const float fx_d = (depth_w == img_w) ? fx : (fx * static_cast<float>(depth_w) / static_cast<float>(img_w));
            const float fy_d = (depth_h == img_h) ? fy : (fy * static_cast<float>(depth_h) / static_cast<float>(img_h));

            constexpr float z_min = 0.08f;
            constexpr float z_max = 8.0f;
            const int img_channels = std::max(1, timg.depth);
            const std::size_t img_expected = static_cast<std::size_t>(img_w) * static_cast<std::size_t>(img_h) * static_cast<std::size_t>(img_channels);
            const bool has_rgb = timg.image.size() >= img_expected;

            for (int v = 0; v < depth_h; ++v)
            {
                for (int u = 0; u < depth_w; ++u)
                {
                    const float d = depth_m[static_cast<std::size_t>(v) * static_cast<std::size_t>(depth_w) + static_cast<std::size_t>(u)];
                    if (!std::isfinite(d) || d < z_min || d > z_max)
                        continue;

                    if (has_rgb)
                    {
                        const int u_img = std::clamp((u * img_w) / std::max(1, depth_w), 0, img_w - 1);
                        const int v_img = std::clamp((v * img_h) / std::max(1, depth_h), 0, img_h - 1);
                        const std::size_t pix = (static_cast<std::size_t>(v_img) * static_cast<std::size_t>(img_w)
                                               + static_cast<std::size_t>(u_img)) * static_cast<std::size_t>(img_channels);
                        if (pix + 2 < timg.image.size())
                        {
                            const unsigned char c0 = static_cast<unsigned char>(timg.image[pix + 0]);
                            const unsigned char c1 = static_cast<unsigned char>(timg.image[pix + 1]);
                            const unsigned char c2 = static_cast<unsigned char>(timg.image[pix + 2]);
                            if (c0 == 0 && c1 == 0 && c2 == 0)
                                continue;
                        }
                    }

                    const float x_cam = (static_cast<float>(u) - cx_d) * d / std::max(1e-5f, fx_d);
                    const float y_cam = d;
                    const float z_cam = (cy_d - static_cast<float>(v)) * d / std::max(1e-5f, fy_d);

                    const Eigen::Vector3f p_cam(x_cam, y_cam, z_cam);
                    const float x_robot = x_cam + params.CAMERA_TX;
                    const float y_robot = y_cam + params.CAMERA_TY;
                    const float z_robot = z_cam + params.CAMERA_TZ;
                    const Eigen::Vector2f p_world_xy = robot_pose * Eigen::Vector2f(x_robot, y_robot);
                    const Eigen::Vector3f p_world(p_world_xy.x(), p_world_xy.y(), z_robot);
                    points_cam_all.emplace_back(p_cam);
                    points_world_all.emplace_back(p_world);
                }
            }
        }

        const bool enough_points = points_world_all.size() >= 30;

        std::vector<Eigen::Vector3f> points_cam = points_cam_all;
        std::vector<Eigen::Vector3f> points_world = points_world_all;

        if (furniture_polygons_.empty())
        {
            if (grounding_status_label_)
                grounding_status_label_->setText("Status: no map furniture for EM classes");
            return;
        }

        const Eigen::Affine2f world_to_robot = robot_pose.inverse();
        const float cx = static_cast<float>(img_w) * 0.5f;
        const float cy = static_cast<float>(img_h) * 0.5f;
        constexpr float near_depth = 0.05f;

        auto to_camera_sel = [this](const Eigen::Vector2f& p_robot, float z_world) -> Eigen::Vector3f
        {
            return Eigen::Vector3f(p_robot.x() - params.CAMERA_TX,
                                   p_robot.y() - params.CAMERA_TY,
                                   z_world - params.CAMERA_TZ);
        };

        auto projects_inside = [&](const Eigen::Vector3f& p_cam) -> bool
        {
            // Camera frame convention used in this component:
            // x = lateral, y = forward(depth), z = up.
            const float depth = p_cam.y();
            if (depth <= near_depth) return false;
            const float lateral = p_cam.x();
            const float u = cx + fx * (lateral / depth);
            const float v = cy - fy * (p_cam.z() / depth);
            return std::isfinite(u) && std::isfinite(v) && u >= 0.f && u < static_cast<float>(img_w) && v >= 0.f && v < static_cast<float>(img_h);
        };

        std::vector<int> candidate_indices;
        candidate_indices.reserve(furniture_polygons_.size());
        for (std::size_t i = 0; i < furniture_polygons_.size(); ++i)
        {
            const auto& fp = furniture_polygons_[i];
            if (fp.vertices.size() < 3)
                continue;

            const std::string base_name = fp.label.empty() ? fp.id : fp.label;
            const float h = model_height_from_label(base_name);

            int projected_in_frustum = 0;
            for (const auto& v : fp.vertices)
            {
                const Eigen::Vector2f vr = world_to_robot * v;
                const Eigen::Vector3f pc0 = to_camera_sel(vr, 0.f);
                const Eigen::Vector3f pc1 = to_camera_sel(vr, h);
                if (projects_inside(pc0)) ++projected_in_frustum;
                if (projects_inside(pc1)) ++projected_in_frustum;
            }

            if (projected_in_frustum == 0)
            {
                const Eigen::Vector2f c_world = centroid_of(fp);
                const Eigen::Vector2f c_robot = world_to_robot * c_world;
                const Eigen::Vector3f c_cam = to_camera_sel(c_robot, 0.5f * h);
                if (projects_inside(c_cam))
                    projected_in_frustum = 1;
            }

            // Require at least one valid projection in the true frustum test.
            if (projected_in_frustum > 0)
                candidate_indices.push_back(static_cast<int>(i));
        }

        if (candidate_indices.empty())
        {
            if (grounding_status_label_)
                grounding_status_label_->setText("Status: no model objects inside camera frustum");
            return;
        }

        struct EMClass
        {
            int furniture_index = -1;
            std::string label;
            float height = 0.8f;
            std::vector<Eigen::Vector2f> vertices;
            float last_sdf = 0.f;
            std::optional<float> prev_sdf;
        };

        std::vector<EMClass> classes;
        classes.reserve(candidate_indices.size());
        for (int idx : candidate_indices)
        {
            const auto& fp = furniture_polygons_[idx];
            EMClass c;
            c.furniture_index = idx;
            c.label = fp.label.empty() ? fp.id : fp.label;
            c.height = model_height_from_label(c.label);
            c.vertices = fp.vertices;
            c.prev_sdf = 1.0f;  // temporary debug baseline requested by user
            classes.emplace_back(std::move(c));
        }

        const int K = static_cast<int>(classes.size());
        const int N = static_cast<int>(points_world.size());
        if (K <= 0)
            return;

        std::vector<QString> class_labels;
        std::vector<QColor> class_colors;
        class_labels.reserve(K + 1);
        class_colors.reserve(K + 1);
        QStringList visible_model_names;
        visible_model_names.reserve(K);
        for (int k = 0; k < K; ++k)
        {
            class_labels.emplace_back(QString::fromStdString(classes[static_cast<std::size_t>(k)].label));
            class_colors.emplace_back(QColor::fromHsv((47 * k) % 360, 220, 255, 255));
            visible_model_names << class_labels.back();
        }
        class_labels.emplace_back("background");
        class_colors.emplace_back(QColor(96, 96, 96, 255));

        const QString visible_names_text = visible_model_names.isEmpty()
            ? QString("Visible models: -")
            : QString("Visible models: %1").arg(visible_model_names.join(", "));
        const QColor em_wireframe_color(255, 255, 255, 255);

        auto point_seg_distance = [](const Eigen::Vector2f& p,
                                     const Eigen::Vector2f& a,
                                     const Eigen::Vector2f& b) -> float
        {
            const Eigen::Vector2f ab = b - a;
            const float ab2 = std::max(1e-9f, ab.squaredNorm());
            const float t = std::clamp((p - a).dot(ab) / ab2, 0.f, 1.f);
            const Eigen::Vector2f q = a + t * ab;
            return (p - q).norm();
        };

        auto point_polygon_distance = [&](const Eigen::Vector2f& p,
                                          const std::vector<Eigen::Vector2f>& poly) -> float
        {
            if (poly.size() < 2)
                return 1e3f;
            float best = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < poly.size(); ++i)
            {
                const auto& a = poly[i];
                const auto& b = poly[(i + 1) % poly.size()];
                best = std::min(best, point_seg_distance(p, a, b));
            }
            return best;
        };

        // If no previous fit exists, estimate it on-the-fly from current class-selected points using table SDF.
        for (auto& c : classes)
        {
            if (c.prev_sdf.has_value() && std::isfinite(c.prev_sdf.value()) && c.prev_sdf.value() > 0.f)
                continue;

            const QString label_lc = QString::fromStdString(c.label).toLower();
            if (!(label_lc.contains("mesa") || label_lc.contains("table")))
                continue;
            if (c.vertices.size() < 3)
                continue;

            Eigen::Vector2f cen = Eigen::Vector2f::Zero();
            for (const auto& v : c.vertices) cen += v;
            cen /= static_cast<float>(c.vertices.size());

            Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
            for (const auto& v : c.vertices)
            {
                const Eigen::Vector2f d = v - cen;
                cov += d * d.transpose();
            }
            cov /= static_cast<float>(std::max<std::size_t>(1, c.vertices.size()));
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
            Eigen::Vector2f axis_x(1.f, 0.f);
            if (eig.info() == Eigen::Success)
                axis_x = eig.eigenvectors().col(1).normalized();
            const float yaw = std::atan2(axis_x.y(), axis_x.x());

            Eigen::Vector2f axis_z(-axis_x.y(), axis_x.x());
            float min_u = std::numeric_limits<float>::max();
            float max_u = -std::numeric_limits<float>::max();
            float min_v = std::numeric_limits<float>::max();
            float max_v = -std::numeric_limits<float>::max();
            for (const auto& p : c.vertices)
            {
                const Eigen::Vector2f d = p - cen;
                const float u = d.dot(axis_x);
                const float v = d.dot(axis_z);
                min_u = std::min(min_u, u); max_u = std::max(max_u, u);
                min_v = std::min(min_v, v); max_v = std::max(max_v, v);
            }
            const float w = std::max(0.08f, max_u - min_u);
            const float d = std::max(0.08f, max_v - min_v);
            const float h = std::max(0.20f, c.height);

            std::vector<float> xyz;
            xyz.reserve(points_world.size() * 3);
            // Conservative baseline set: intentionally broader than the EM update set.
            // This avoids over-optimistic "prev" scores when the initial pose is poor.
            const float select_dist = std::max(0.35f, 0.60f * std::max(w, d));
            const float radial_gate = std::max(0.80f, 1.40f * std::max(w, d));
            for (const auto& pw : points_world)
            {
                const float dist = point_polygon_distance(pw.head<2>(), c.vertices);
                const float radial = (pw.head<2>() - cen).norm();
                if (dist > select_dist && radial > radial_gate)
                    continue;
                if (pw.z() < -0.10f || pw.z() > h + 0.60f)
                    continue;
                xyz.push_back(pw.x());
                xyz.push_back(pw.y());
                xyz.push_back(pw.z());
            }

            if (xyz.size() < 15 * 3)
                continue;

            auto pts = torch::from_blob(xyz.data(),
                                        {static_cast<long>(xyz.size() / 3), 3},
                                        torch::TensorOptions().dtype(torch::kFloat32)).clone();
            auto params = torch::tensor({cen.x(), cen.y(), yaw, w, d, h},
                                        torch::TensorOptions().dtype(torch::kFloat32));
            auto sdf = rc::object_models::TableAnalyticModel::forward_sdf(pts, params);
            const float mse = torch::mean(torch::square(sdf)).item<float>();
            const float mae = torch::mean(torch::abs(sdf)).item<float>();
            const float prev_est = 0.5f * mae + 0.5f * mse;
            if (std::isfinite(prev_est) && prev_est > 0.f)
                c.prev_sdf = std::max(0.01f, prev_est);
        }

        float prev_sdf_sum = 0.f;
        int prev_sdf_count = 0;
        for (const auto& c : classes)
        {
            if (c.prev_sdf.has_value() && std::isfinite(c.prev_sdf.value()) && c.prev_sdf.value() > 0.f)
            {
                prev_sdf_sum += c.prev_sdf.value();
                ++prev_sdf_count;
            }
        }
        const float prev_mean_sdf = (prev_sdf_count > 0) ? (prev_sdf_sum / static_cast<float>(prev_sdf_count)) : 0.f;

        // Draw initial visible wireframes immediately (even before EM iterations)
        {
            auto to_camera = [this](const Eigen::Vector2f& p_robot, float z_world) -> Eigen::Vector3f
            {
                return Eigen::Vector3f(p_robot.x() - params.CAMERA_TX,
                                       p_robot.y() - params.CAMERA_TY,
                                       z_world - params.CAMERA_TZ);
            };

            auto oriented_box_corners = [](const std::vector<Eigen::Vector2f>& poly) -> std::array<Eigen::Vector2f, 4>
            {
                std::array<Eigen::Vector2f, 4> out{Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()};
                if (poly.empty())
                    return out;

                Eigen::Vector2f c = Eigen::Vector2f::Zero();
                for (const auto& p : poly) c += p;
                c /= static_cast<float>(poly.size());

                Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
                for (const auto& p : poly)
                {
                    const Eigen::Vector2f d = p - c;
                    cov += d * d.transpose();
                }
                cov /= static_cast<float>(std::max<std::size_t>(1, poly.size()));

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
                Eigen::Vector2f u(1.f, 0.f);
                if (eig.info() == Eigen::Success)
                    u = eig.eigenvectors().col(1).normalized();
                Eigen::Vector2f v(-u.y(), u.x());

                float umin = std::numeric_limits<float>::max();
                float umax = -std::numeric_limits<float>::max();
                float vmin = std::numeric_limits<float>::max();
                float vmax = -std::numeric_limits<float>::max();
                for (const auto& p : poly)
                {
                    const Eigen::Vector2f d = p - c;
                    const float pu = d.dot(u);
                    const float pv = d.dot(v);
                    umin = std::min(umin, pu);
                    umax = std::max(umax, pu);
                    vmin = std::min(vmin, pv);
                    vmax = std::max(vmax, pv);
                }

                out[0] = c + u * umin + v * vmin;
                out[1] = c + u * umax + v * vmin;
                out[2] = c + u * umax + v * vmax;
                out[3] = c + u * umin + v * vmax;
                return out;
            };

            auto append_mesh_like_wireframe = [&](const std::vector<Eigen::Vector2f>& poly,
                                                  const std::string& label,
                                                  float h,
                                                  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& out_segments,
                                                  std::vector<QColor>& out_colors,
                                                  const QColor& color)
            {
                if (poly.size() < 3)
                    return;

                const auto corners = oriented_box_corners(poly);
                const auto& c0 = corners[0];
                const auto& c1 = corners[1];
                const auto& c2 = corners[2];
                const auto& c3 = corners[3];

                auto add = [&](const Eigen::Vector2f& a, float za, const Eigen::Vector2f& b, float zb)
                {
                    const Eigen::Vector2f ar = world_to_robot * a;
                    const Eigen::Vector2f br = world_to_robot * b;
                    out_segments.emplace_back(to_camera(ar, za), to_camera(br, zb));
                    out_colors.emplace_back(color);
                };

                const Eigen::Vector2f center = 0.25f * (c0 + c1 + c2 + c3);
                const Eigen::Vector2f front_mid = 0.5f * (c0 + c1);
                const Eigen::Vector2f back_mid  = 0.5f * (c2 + c3);
                const Eigen::Vector2f left_mid  = 0.5f * (c0 + c3);
                const Eigen::Vector2f right_mid = 0.5f * (c1 + c2);
                const QString ql = QString::fromStdString(label).toLower();

                if (ql.contains("mesa") || ql.contains("table"))
                {
                    const Eigen::Vector2f center = 0.25f * (c0 + c1 + c2 + c3);
                    Eigen::Vector2f ux = c1 - c0;
                    Eigen::Vector2f uy = c3 - c0;
                    const float w = std::max(0.08f, ux.norm());
                    const float d = std::max(0.08f, uy.norm());
                    if (ux.norm() < 1e-6f) ux = Eigen::Vector2f(1.f, 0.f); else ux.normalize();
                    if (uy.norm() < 1e-6f) uy = Eigen::Vector2f(-ux.y(), ux.x()); else uy.normalize();

                    const float h_table = std::max(0.20f, h);
                    const float top_t = std::max(0.03f, 0.08f * h_table);
                    const float leg_t = std::max(0.03f, 0.12f * std::min(w, d));
                    const float leg_h = std::max(0.05f, h_table - top_t);
                    const float leg_dx = std::max(0.f, 0.5f * w - 0.5f * leg_t);
                    const float leg_dy = std::max(0.f, 0.5f * d - 0.5f * leg_t);

                    auto local_to_world = [&](float lx, float ly) -> Eigen::Vector2f
                    {
                        return center + ux * lx + uy * ly;
                    };

                    auto add_cuboid_wire = [&](float lx, float ly, float sx, float sy, float sz, float cz)
                    {
                        const float hx = 0.5f * sx;
                        const float hy = 0.5f * sy;
                        const float hz = 0.5f * sz;
                        const float z0 = cz - hz;
                        const float z1 = cz + hz;

                        const Eigen::Vector2f p00 = local_to_world(lx - hx, ly - hy);
                        const Eigen::Vector2f p10 = local_to_world(lx + hx, ly - hy);
                        const Eigen::Vector2f p11 = local_to_world(lx + hx, ly + hy);
                        const Eigen::Vector2f p01 = local_to_world(lx - hx, ly + hy);

                        add(p00, z0, p10, z0); add(p10, z0, p11, z0); add(p11, z0, p01, z0); add(p01, z0, p00, z0);
                        add(p00, z1, p10, z1); add(p10, z1, p11, z1); add(p11, z1, p01, z1); add(p01, z1, p00, z1);
                        add(p00, z0, p00, z1); add(p10, z0, p10, z1); add(p11, z0, p11, z1); add(p01, z0, p01, z1);
                    };

                    add_cuboid_wire(0.f, 0.f, w, d, top_t, leg_h + 0.5f * top_t);
                    add_cuboid_wire( leg_dx,  leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                    add_cuboid_wire( leg_dx, -leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                    add_cuboid_wire(-leg_dx,  leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                    add_cuboid_wire(-leg_dx, -leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                    return;
                }

                if (ql.contains("silla") || ql.contains("chair"))
                {
                    const float z_seat = 0.50f * h;
                    const float z_back = 0.98f * h;
                    const float t = 0.18f;

                    const Eigen::Vector2f s0 = c0 + t * (c1 - c0) + t * (c3 - c0);
                    const Eigen::Vector2f s1 = c1 + t * (c0 - c1) + t * (c2 - c1);
                    const Eigen::Vector2f s2 = c2 + t * (c1 - c2) + t * (c3 - c2);
                    const Eigen::Vector2f s3 = c3 + t * (c2 - c3) + t * (c0 - c3);

                    // Seat contour
                    add(s0, z_seat, s1, z_seat);
                    add(s1, z_seat, s2, z_seat);
                    add(s2, z_seat, s3, z_seat);
                    add(s3, z_seat, s0, z_seat);
                    // Legs
                    add(s0, 0.f, s0, z_seat);
                    add(s1, 0.f, s1, z_seat);
                    add(s2, 0.f, s2, z_seat);
                    add(s3, 0.f, s3, z_seat);
                    // Backrest at rear side (c2-c3)
                    const Eigen::Vector2f b0 = 0.5f * (s2 + c2);
                    const Eigen::Vector2f b1 = 0.5f * (s3 + c3);
                    add(b0, z_seat, b0, z_back);
                    add(b1, z_seat, b1, z_back);
                    add(b0, z_back, b1, z_back);
                    add(0.5f * (b0 + b1), z_seat, 0.5f * (b0 + b1), z_back);
                    return;
                }

                // Default mesh-like wireframe: top contour + main supports.
                const float z_top = h;
                const float z_support = 0.92f * h;
                add(c0, z_top, c1, z_top);
                add(c1, z_top, c2, z_top);
                add(c2, z_top, c3, z_top);
                add(c3, z_top, c0, z_top);
                add(c0, 0.f, c0, z_support);
                add(c1, 0.f, c1, z_support);
                add(c2, 0.f, c2, z_support);
                add(c3, 0.f, c3, z_support);
                add(front_mid, 0.70f * h, back_mid, 0.70f * h);
                add(left_mid, 0.70f * h, right_mid, 0.70f * h);
                add(center, 0.68f * h, center, z_top);
            };

            std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> init_segments;
            std::vector<QColor> init_segment_colors;
            std::vector<Eigen::Vector3f> init_ann_points;
            std::vector<QString> init_ann_texts;
            std::vector<QColor> init_ann_colors;
            for (int k = 0; k < K; ++k)
            {
                const auto& c = classes[static_cast<std::size_t>(k)];
                Eigen::Vector2f center = Eigen::Vector2f::Zero();
                for (const auto& v : c.vertices) center += v;
                center /= static_cast<float>(std::max<std::size_t>(1, c.vertices.size()));
                init_ann_points.emplace_back(to_camera(world_to_robot * center, c.height + 0.05f));
                const QString prev_txt = c.prev_sdf.has_value()
                    ? QString::number(c.prev_sdf.value(), 'f', 4)
                    : QString("--");
                init_ann_texts.emplace_back(QString::fromStdString(c.label) + QString(" | prev=%1 cur=--").arg(prev_txt));
                init_ann_colors.emplace_back(QColor(255, 255, 255, 255));
                append_mesh_like_wireframe(c.vertices, c.label, c.height, init_segments, init_segment_colors, em_wireframe_color);
            }
            const QString start_banner = (prev_sdf_count > 0)
                ? QString("EM start | prev=%1 cur=--").arg(prev_mean_sdf, 0, 'f', 4)
                : QString("EM start | prev=-- cur=--");
            camera_viewer_->set_wireframe_segments_camera_colored(init_segments, init_segment_colors, start_banner);
            camera_viewer_->set_wireframe_annotations_camera(init_ann_points, init_ann_texts, init_ann_colors);
            QCoreApplication::processEvents(QEventLoop::AllEvents, 15);
        }

        if (!enough_points || N <= 0)
        {
            if (grounding_status_label_)
                grounding_status_label_->setText("Status: visible models shown (insufficient 3D points for EM)");
            keep_overlay_lock = true;
            em_overlay_persistent_active_ = true;
            return;
        }

        std::vector<float> q(static_cast<std::size_t>(N * (K + 1)), 1.f / static_cast<float>(K + 1));
        std::vector<int> point_classes(static_cast<std::size_t>(N), K);

        constexpr int max_iters = 8;
        constexpr float sigma_xy = 0.25f;
        constexpr float sigma_z = 0.40f;

        auto to_camera = [this](const Eigen::Vector2f& p_robot, float z_world) -> Eigen::Vector3f
        {
            return Eigen::Vector3f(p_robot.x() - params.CAMERA_TX,
                                   p_robot.y() - params.CAMERA_TY,
                                   z_world - params.CAMERA_TZ);
        };

        struct PixelDepth
        {
            int u = -1;
            int v = -1;
            float depth = 0.f;
        };
        std::vector<PixelDepth> point_pixels(static_cast<std::size_t>(N));
        for (int i = 0; i < N; ++i)
        {
            const auto& pc = points_cam[static_cast<std::size_t>(i)];
            const float d = pc.y();
            if (d <= near_depth)
                continue;
            const float u = cx + fx * (pc.x() / d);
            const float v = cy - fy * (pc.z() / d);
            if (std::isfinite(u) && std::isfinite(v) && u >= 0.f && u < static_cast<float>(img_w) && v >= 0.f && v < static_cast<float>(img_h))
            {
                point_pixels[static_cast<std::size_t>(i)] = PixelDepth{static_cast<int>(u), static_cast<int>(v), d};
            }
        }

        const int rw = std::max(1, img_w / 2);
        const int rh = std::max(1, img_h / 2);
        constexpr float inf_depth = std::numeric_limits<float>::infinity();

        std::vector<Eigen::Vector2f> room_robot;
        room_robot.reserve(room_polygon_.size());
        for (const auto& p : room_polygon_)
            room_robot.emplace_back(world_to_robot * p);

        auto point_in_polygon = [](const Eigen::Vector2f& p, const std::vector<Eigen::Vector2f>& poly) -> bool
        {
            if (poly.size() < 3)
                return false;
            bool inside = false;
            for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
            {
                const auto& pi = poly[i];
                const auto& pj = poly[j];
                const bool intersect = ((pi.y() > p.y()) != (pj.y() > p.y())) &&
                                       (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) /
                                                    std::max(1e-9f, (pj.y() - pi.y())) + pi.x());
                if (intersect)
                    inside = !inside;
            }
            return inside;
        };

        float last_mean_sdf = 0.f;
        for (int iter = 0; iter < max_iters; ++iter)
        {
            // Build per-class depth raster from current wireframe (safe, lightweight z-buffer).
            std::vector<std::vector<float>> class_depth_raster(static_cast<std::size_t>(K), std::vector<float>(static_cast<std::size_t>(rw * rh), inf_depth));

            auto project_uvd = [&](const Eigen::Vector3f& p_cam, float& u, float& v, float& d) -> bool
            {
                d = p_cam.y();
                if (d <= near_depth)
                    return false;
                u = cx + fx * (p_cam.x() / d);
                v = cy - fy * (p_cam.z() / d);
                return std::isfinite(u) && std::isfinite(v);
            };

            auto raster_write = [&](std::vector<float>& ras, int u, int v, float d)
            {
                if (u < 0 || v < 0 || u >= rw || v >= rh || d <= near_depth)
                    return;
                const std::size_t idx = static_cast<std::size_t>(v) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(u);
                ras[idx] = std::min(ras[idx], d);
            };

            auto raster_segment = [&](std::vector<float>& ras, const Eigen::Vector3f& a_cam, const Eigen::Vector3f& b_cam)
            {
                float ua = 0.f, va = 0.f, da = 0.f;
                float ub = 0.f, vb = 0.f, db = 0.f;
                if (!project_uvd(a_cam, ua, va, da) || !project_uvd(b_cam, ub, vb, db))
                    return;

                const float du = ub - ua;
                const float dv = vb - va;
                const int steps = std::max(1, static_cast<int>(std::ceil(std::max(std::abs(du), std::abs(dv)) / 2.f)));
                for (int s = 0; s <= steps; ++s)
                {
                    const float t = static_cast<float>(s) / static_cast<float>(steps);
                    const float u = ua + t * du;
                    const float v = va + t * dv;
                    const float d = da + t * (db - da);
                    raster_write(ras,
                                 static_cast<int>(u * static_cast<float>(rw) / static_cast<float>(img_w)),
                                 static_cast<int>(v * static_cast<float>(rh) / static_cast<float>(img_h)),
                                 d);
                }
            };

            for (int k = 0; k < K; ++k)
            {
                const auto& c = classes[static_cast<std::size_t>(k)];
                if (c.vertices.size() < 3)
                    continue;
                auto& ras = class_depth_raster[static_cast<std::size_t>(k)];
                for (std::size_t e = 0; e < c.vertices.size(); ++e)
                {
                    const auto& a = c.vertices[e];
                    const auto& b = c.vertices[(e + 1) % c.vertices.size()];
                    const Eigen::Vector2f ar = world_to_robot * a;
                    const Eigen::Vector2f br = world_to_robot * b;
                    raster_segment(ras, to_camera(ar, 0.f), to_camera(br, 0.f));
                    raster_segment(ras, to_camera(ar, c.height), to_camera(br, c.height));
                    raster_segment(ras, to_camera(ar, 0.f), to_camera(ar, c.height));
                }
            }

            // Build infrastructure depth raster (walls/floor/ceiling) in camera depth units.
            std::vector<float> infra_depth_raster(static_cast<std::size_t>(rw * rh), inf_depth);
            if (room_robot.size() >= 3)
            {
                const Eigen::Vector3f cam_o(params.CAMERA_TX, params.CAMERA_TY, params.CAMERA_TZ);
                constexpr float wall_h = 2.5f;

                auto raster_write_depth = [&](int u, int v, float d)
                {
                    if (u < 0 || v < 0 || u >= rw || v >= rh || d <= near_depth)
                        return;
                    const std::size_t idx = static_cast<std::size_t>(v) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(u);
                    infra_depth_raster[idx] = std::min(infra_depth_raster[idx], d);
                };

                for (int vr = 0; vr < rh; ++vr)
                {
                    for (int ur = 0; ur < rw; ++ur)
                    {
                        const float u_full = (static_cast<float>(ur) + 0.5f) * static_cast<float>(img_w) / static_cast<float>(rw);
                        const float v_full = (static_cast<float>(vr) + 0.5f) * static_cast<float>(img_h) / static_cast<float>(rh);
                        const Eigen::Vector3f d_cam((u_full - cx) / std::max(1e-5f, fx),
                                                    1.f,
                                                    (cy - v_full) / std::max(1e-5f, fy));

                        float best_t = inf_depth;

                        // Floor / ceiling intersections.
                        if (std::abs(d_cam.z()) > 1e-6f)
                        {
                            const float t_floor = (0.f - cam_o.z()) / d_cam.z();
                            if (t_floor > near_depth)
                            {
                                const Eigen::Vector2f pxy = cam_o.head<2>() + t_floor * d_cam.head<2>();
                                if (point_in_polygon(pxy, room_robot))
                                    best_t = std::min(best_t, t_floor);
                            }

                            const float t_ceil = (wall_h - cam_o.z()) / d_cam.z();
                            if (t_ceil > near_depth)
                            {
                                const Eigen::Vector2f pxy = cam_o.head<2>() + t_ceil * d_cam.head<2>();
                                if (point_in_polygon(pxy, room_robot))
                                    best_t = std::min(best_t, t_ceil);
                            }
                        }

                        // Wall intersections in XY, then validate z in [0, wall_h].
                        const Eigen::Vector2f o_xy = cam_o.head<2>();
                        const Eigen::Vector2f r_xy = d_cam.head<2>();
                        const auto cross2 = [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) -> float
                        {
                            return a.x() * b.y() - a.y() * b.x();
                        };
                        for (std::size_t e = 0; e < room_robot.size(); ++e)
                        {
                            const Eigen::Vector2f a = room_robot[e];
                            const Eigen::Vector2f b = room_robot[(e + 1) % room_robot.size()];
                            const Eigen::Vector2f s = b - a;
                            const float den = cross2(r_xy, s);
                            if (std::abs(den) < 1e-8f)
                                continue;

                            const Eigen::Vector2f qmp = a - o_xy;
                            const float t = cross2(qmp, s) / den;
                            const float seg_u = cross2(qmp, r_xy) / den;
                            if (t <= near_depth || seg_u < 0.f || seg_u > 1.f)
                                continue;

                            const float z_hit = cam_o.z() + t * d_cam.z();
                            if (z_hit >= 0.f && z_hit <= wall_h)
                                best_t = std::min(best_t, t);
                        }

                        if (std::isfinite(best_t))
                            raster_write_depth(ur, vr, best_t);
                    }
                }
            }

            // Recompute active points with current object pose hypothesis.
            std::vector<unsigned char> active_points(static_cast<std::size_t>(N), 0);
            int active_count = 0;
            for (int i = 0; i < N; ++i)
            {
                const auto pix = point_pixels[static_cast<std::size_t>(i)];
                if (pix.u < 0 || pix.v < 0)
                    continue;

                const int ur = pix.u * rw / std::max(1, img_w);
                const int vr = pix.v * rh / std::max(1, img_h);
                float best_obj_pred = inf_depth;
                for (int k = 0; k < K; ++k)
                {
                    const auto& ras = class_depth_raster[static_cast<std::size_t>(k)];
                    for (int dv = -1; dv <= 1; ++dv)
                    {
                        for (int du = -1; du <= 1; ++du)
                        {
                            const int uu = ur + du;
                            const int vv = vr + dv;
                            if (uu < 0 || vv < 0 || uu >= rw || vv >= rh)
                                continue;
                            const float d_pred = ras[static_cast<std::size_t>(vv) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(uu)];
                            best_obj_pred = std::min(best_obj_pred, d_pred);
                        }
                    }
                }

                float best_infra_pred = inf_depth;
                for (int dv = -1; dv <= 1; ++dv)
                {
                    for (int du = -1; du <= 1; ++du)
                    {
                        const int uu = ur + du;
                        const int vv = vr + dv;
                        if (uu < 0 || vv < 0 || uu >= rw || vv >= rh)
                            continue;
                        const float d_inf = infra_depth_raster[static_cast<std::size_t>(vv) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(uu)];
                        best_infra_pred = std::min(best_infra_pred, d_inf);
                    }
                }

                const bool obj_supported = std::isfinite(best_obj_pred)
                    && (std::abs(pix.depth - best_obj_pred) < 0.60f || pix.depth + 0.08f < best_obj_pred);
                const bool infra_close = std::isfinite(best_infra_pred)
                    && (pix.depth >= best_infra_pred - 0.05f);
                const bool keep = obj_supported || !infra_close;

                if (keep)
                {
                    active_points[static_cast<std::size_t>(i)] = 1;
                    ++active_count;
                }
            }

            // E-step: class likelihoods from current class geometry (SDF-like distance to polygon contour)
            float objective = 0.f;
            for (int i = 0; i < N; ++i)
            {
                if (active_points[static_cast<std::size_t>(i)] == 0)
                {
                    for (int k = 0; k < K; ++k)
                        q[static_cast<std::size_t>(i * (K + 1) + k)] = 0.f;
                    q[static_cast<std::size_t>(i * (K + 1) + K)] = 1.f;
                    point_classes[static_cast<std::size_t>(i)] = K;
                    continue;
                }

                const auto& pw = points_world[static_cast<std::size_t>(i)];
                const Eigen::Vector2f pxy = pw.head<2>();

                std::vector<float> logp(static_cast<std::size_t>(K + 1), 0.f);
                float max_logp = -std::numeric_limits<float>::max();

                for (int k = 0; k < K; ++k)
                {
                    const auto& c = classes[static_cast<std::size_t>(k)];
                    const float dxy = point_polygon_distance(pxy, c.vertices);
                    const float dz = std::abs(pw.z() - 0.5f * c.height);
                    float depth_pen = 0.f;
                    const auto pix = point_pixels[static_cast<std::size_t>(i)];
                    if (pix.u >= 0 && pix.v >= 0)
                    {
                        const int ur = pix.u * rw / std::max(1, img_w);
                        const int vr = pix.v * rh / std::max(1, img_h);
                        const auto& ras = class_depth_raster[static_cast<std::size_t>(k)];
                        float best_pred = inf_depth;
                        for (int dv = -1; dv <= 1; ++dv)
                        {
                            for (int du = -1; du <= 1; ++du)
                            {
                                const int uu = ur + du;
                                const int vv = vr + dv;
                                if (uu < 0 || vv < 0 || uu >= rw || vv >= rh)
                                    continue;
                                const float d_pred = ras[static_cast<std::size_t>(vv) * static_cast<std::size_t>(rw) + static_cast<std::size_t>(uu)];
                                best_pred = std::min(best_pred, d_pred);
                            }
                        }
                        if (std::isfinite(best_pred))
                        {
                            const float dr = std::abs(pix.depth - best_pred);
                            depth_pen = std::min(dr, 2.0f);
                            // If observed point is in front of model wireframe, class likely occluded by something else.
                            if (pix.depth + 0.10f < best_pred)
                                depth_pen += 0.60f;
                        }
                    }
                    const float e = (dxy * dxy) / (2.f * sigma_xy * sigma_xy)
                                  + 0.35f * (dz * dz) / (2.f * sigma_z * sigma_z);
                    const float l = -(e + 0.45f * depth_pen);
                    logp[static_cast<std::size_t>(k)] = l;
                    max_logp = std::max(max_logp, l);
                }
                const float l_bg = -0.7f * std::abs(pw.z());
                logp[static_cast<std::size_t>(K)] = l_bg;
                max_logp = std::max(max_logp, l_bg);

                float sumw = 0.f;
                for (int k = 0; k < K + 1; ++k)
                {
                    const float w = std::exp(logp[static_cast<std::size_t>(k)] - max_logp);
                    q[static_cast<std::size_t>(i * (K + 1) + k)] = w;
                    sumw += w;
                }
                sumw = std::max(sumw, 1e-8f);

                int best_k = K;
                float best_q = -1.f;
                for (int k = 0; k < K + 1; ++k)
                {
                    const float qq = q[static_cast<std::size_t>(i * (K + 1) + k)] / sumw;
                    q[static_cast<std::size_t>(i * (K + 1) + k)] = qq;
                    if (qq > best_q)
                    {
                        best_q = qq;
                        best_k = k;
                    }
                }
                point_classes[static_cast<std::size_t>(i)] = best_k;
                objective += -std::log(std::max(best_q, 1e-8f));
            }
            objective /= static_cast<float>(std::max(1, active_count));

            std::vector<float> class_likelihood(static_cast<std::size_t>(K), 0.f);
            for (int k = 0; k < K; ++k)
            {
                float sum_q = 0.f;
                for (int i = 0; i < N; ++i)
                    if (active_points[static_cast<std::size_t>(i)] != 0)
                        sum_q += q[static_cast<std::size_t>(i * (K + 1) + k)];
                class_likelihood[static_cast<std::size_t>(k)] = sum_q / static_cast<float>(std::max(1, active_count));
            }

            // Wireframe overlay for visible map classes (updated per iteration).
            std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> segments;
            std::vector<QColor> segment_colors;
            std::vector<Eigen::Vector3f> annotation_points;
            std::vector<QString> annotation_texts;
            std::vector<QColor> annotation_colors;

            auto oriented_box_corners_iter = [](const std::vector<Eigen::Vector2f>& poly) -> std::array<Eigen::Vector2f, 4>
            {
                std::array<Eigen::Vector2f, 4> out{Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()};
                if (poly.empty())
                    return out;

                Eigen::Vector2f c = Eigen::Vector2f::Zero();
                for (const auto& p : poly) c += p;
                c /= static_cast<float>(poly.size());

                Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
                for (const auto& p : poly)
                {
                    const Eigen::Vector2f d = p - c;
                    cov += d * d.transpose();
                }
                cov /= static_cast<float>(std::max<std::size_t>(1, poly.size()));

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
                Eigen::Vector2f u(1.f, 0.f);
                if (eig.info() == Eigen::Success)
                    u = eig.eigenvectors().col(1).normalized();
                Eigen::Vector2f v(-u.y(), u.x());

                float umin = std::numeric_limits<float>::max();
                float umax = -std::numeric_limits<float>::max();
                float vmin = std::numeric_limits<float>::max();
                float vmax = -std::numeric_limits<float>::max();
                for (const auto& p : poly)
                {
                    const Eigen::Vector2f d = p - c;
                    const float pu = d.dot(u);
                    const float pv = d.dot(v);
                    umin = std::min(umin, pu);
                    umax = std::max(umax, pu);
                    vmin = std::min(vmin, pv);
                    vmax = std::max(vmax, pv);
                }

                out[0] = c + u * umin + v * vmin;
                out[1] = c + u * umax + v * vmin;
                out[2] = c + u * umax + v * vmax;
                out[3] = c + u * umin + v * vmax;
                return out;
            };

            auto append_mesh_like_wireframe_iter = [&](const std::vector<Eigen::Vector2f>& poly,
                                                       const std::string& label,
                                                       float h,
                                                       std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& out_segments,
                                                       std::vector<QColor>& out_colors,
                                                       const QColor& color)
            {
                if (poly.size() < 3)
                    return;

                const auto corners = oriented_box_corners_iter(poly);
                const auto& c0 = corners[0];
                const auto& c1 = corners[1];
                const auto& c2 = corners[2];
                const auto& c3 = corners[3];

                auto add = [&](const Eigen::Vector2f& a, float za, const Eigen::Vector2f& b, float zb)
                {
                    const Eigen::Vector2f ar = world_to_robot * a;
                    const Eigen::Vector2f br = world_to_robot * b;
                    out_segments.emplace_back(to_camera(ar, za), to_camera(br, zb));
                    out_colors.emplace_back(color);
                };

                const Eigen::Vector2f center = 0.25f * (c0 + c1 + c2 + c3);
                const Eigen::Vector2f front_mid = 0.5f * (c0 + c1);
                const Eigen::Vector2f back_mid  = 0.5f * (c2 + c3);
                const Eigen::Vector2f left_mid  = 0.5f * (c0 + c3);
                const Eigen::Vector2f right_mid = 0.5f * (c1 + c2);
                const QString ql = QString::fromStdString(label).toLower();

                if (ql.contains("mesa") || ql.contains("table"))
                {
                    const Eigen::Vector2f center = 0.25f * (c0 + c1 + c2 + c3);
                    Eigen::Vector2f ux = c1 - c0;
                    Eigen::Vector2f uy = c3 - c0;
                    const float w = std::max(0.08f, ux.norm());
                    const float d = std::max(0.08f, uy.norm());
                    if (ux.norm() < 1e-6f) ux = Eigen::Vector2f(1.f, 0.f); else ux.normalize();
                    if (uy.norm() < 1e-6f) uy = Eigen::Vector2f(-ux.y(), ux.x()); else uy.normalize();

                    const float h_table = std::max(0.20f, h);
                    const float top_t = std::max(0.03f, 0.08f * h_table);
                    const float leg_t = std::max(0.03f, 0.12f * std::min(w, d));
                    const float leg_h = std::max(0.05f, h_table - top_t);
                    const float leg_dx = std::max(0.f, 0.5f * w - 0.5f * leg_t);
                    const float leg_dy = std::max(0.f, 0.5f * d - 0.5f * leg_t);

                    auto local_to_world = [&](float lx, float ly) -> Eigen::Vector2f
                    {
                        return center + ux * lx + uy * ly;
                    };

                    auto add_cuboid_wire = [&](float lx, float ly, float sx, float sy, float sz, float cz)
                    {
                        const float hx = 0.5f * sx;
                        const float hy = 0.5f * sy;
                        const float hz = 0.5f * sz;
                        const float z0 = cz - hz;
                        const float z1 = cz + hz;

                        const Eigen::Vector2f p00 = local_to_world(lx - hx, ly - hy);
                        const Eigen::Vector2f p10 = local_to_world(lx + hx, ly - hy);
                        const Eigen::Vector2f p11 = local_to_world(lx + hx, ly + hy);
                        const Eigen::Vector2f p01 = local_to_world(lx - hx, ly + hy);

                        add(p00, z0, p10, z0); add(p10, z0, p11, z0); add(p11, z0, p01, z0); add(p01, z0, p00, z0);
                        add(p00, z1, p10, z1); add(p10, z1, p11, z1); add(p11, z1, p01, z1); add(p01, z1, p00, z1);
                        add(p00, z0, p00, z1); add(p10, z0, p10, z1); add(p11, z0, p11, z1); add(p01, z0, p01, z1);
                    };

                    add_cuboid_wire(0.f, 0.f, w, d, top_t, leg_h + 0.5f * top_t);
                    add_cuboid_wire( leg_dx,  leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                    add_cuboid_wire( leg_dx, -leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                    add_cuboid_wire(-leg_dx,  leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                    add_cuboid_wire(-leg_dx, -leg_dy, leg_t, leg_t, leg_h, 0.5f * leg_h);
                    return;
                }

                if (ql.contains("silla") || ql.contains("chair"))
                {
                    const float z_seat = 0.50f * h;
                    const float z_back = 0.98f * h;
                    const float t = 0.18f;
                    const Eigen::Vector2f s0 = c0 + t * (c1 - c0) + t * (c3 - c0);
                    const Eigen::Vector2f s1 = c1 + t * (c0 - c1) + t * (c2 - c1);
                    const Eigen::Vector2f s2 = c2 + t * (c1 - c2) + t * (c3 - c2);
                    const Eigen::Vector2f s3 = c3 + t * (c2 - c3) + t * (c0 - c3);
                    add(s0, z_seat, s1, z_seat);
                    add(s1, z_seat, s2, z_seat);
                    add(s2, z_seat, s3, z_seat);
                    add(s3, z_seat, s0, z_seat);
                    add(s0, 0.f, s0, z_seat);
                    add(s1, 0.f, s1, z_seat);
                    add(s2, 0.f, s2, z_seat);
                    add(s3, 0.f, s3, z_seat);
                    const Eigen::Vector2f b0 = 0.5f * (s2 + c2);
                    const Eigen::Vector2f b1 = 0.5f * (s3 + c3);
                    add(b0, z_seat, b0, z_back);
                    add(b1, z_seat, b1, z_back);
                    add(b0, z_back, b1, z_back);
                    add(0.5f * (b0 + b1), z_seat, 0.5f * (b0 + b1), z_back);
                    return;
                }

                const float z_top = h;
                const float z_support = 0.92f * h;
                add(c0, z_top, c1, z_top);
                add(c1, z_top, c2, z_top);
                add(c2, z_top, c3, z_top);
                add(c3, z_top, c0, z_top);
                add(c0, 0.f, c0, z_support);
                add(c1, 0.f, c1, z_support);
                add(c2, 0.f, c2, z_support);
                add(c3, 0.f, c3, z_support);
                add(front_mid, 0.70f * h, back_mid, 0.70f * h);
                add(left_mid, 0.70f * h, right_mid, 0.70f * h);
                add(center, 0.68f * h, center, z_top);
            };

            for (const auto& c : classes)
            {
                if (c.vertices.size() < 3)
                    continue;
                const int class_idx = static_cast<int>(&c - classes.data());

                Eigen::Vector2f center = Eigen::Vector2f::Zero();
                for (const auto& v : c.vertices) center += v;
                center /= static_cast<float>(std::max<std::size_t>(1, c.vertices.size()));
                const Eigen::Vector2f center_robot = world_to_robot * center;
                annotation_points.emplace_back(to_camera(center_robot, c.height + 0.05f));
                const float lk = (class_idx >= 0 && class_idx < static_cast<int>(class_likelihood.size()))
                    ? class_likelihood[static_cast<std::size_t>(class_idx)] : 0.f;
                const QString prev_txt = c.prev_sdf.has_value()
                    ? QString::number(c.prev_sdf.value(), 'f', 4)
                    : QString("--");
                const QString cur_txt = (std::isfinite(c.last_sdf) && c.last_sdf > 0.f)
                    ? QString::number(c.last_sdf, 'f', 4)
                    : QString("--");
                annotation_texts.emplace_back(QString::fromStdString(c.label)
                                              + QString(" | prev=%1 cur=%2 L=%3")
                                                    .arg(prev_txt)
                                                    .arg(cur_txt)
                                                    .arg(lk, 0, 'f', 2));
                annotation_colors.emplace_back(QColor(255, 255, 255, 255));
                append_mesh_like_wireframe_iter(c.vertices, c.label, c.height, segments, segment_colors, em_wireframe_color);
            }
            const QString em_banner = (prev_sdf_count > 0)
                ? QString("EM iter %1/%2 | prev=%3 cur=%4")
                    .arg(iter + 1)
                    .arg(max_iters)
                    .arg(prev_mean_sdf, 0, 'f', 4)
                    .arg(last_mean_sdf, 0, 'f', 4)
                : QString("EM iter %1/%2 | prev=-- cur=%3")
                    .arg(iter + 1)
                    .arg(max_iters)
                    .arg(last_mean_sdf, 0, 'f', 4);
            camera_viewer_->set_wireframe_segments_camera_colored(segments, segment_colors, em_banner);
            camera_viewer_->set_wireframe_annotations_camera(annotation_points, annotation_texts, annotation_colors);

            const QString title = (prev_sdf_count > 0)
                ? QString("EM iter %1/%2 | prev=%3 cur=%4")
                    .arg(iter + 1)
                    .arg(max_iters)
                    .arg(prev_mean_sdf, 0, 'f', 4)
                    .arg(last_mean_sdf, 0, 'f', 4)
                : QString("EM iter %1/%2 | prev=-- cur=%3")
                    .arg(iter + 1)
                    .arg(max_iters)
                    .arg(last_mean_sdf, 0, 'f', 4);
            camera_viewer_->set_em_points_overlay(points_cam, point_classes, class_colors, class_labels, title);

            if (grounding_status_label_)
                grounding_status_label_->setText(QString("Status: EM iter %1/%2").arg(iter + 1).arg(max_iters));
            if (grounding_sdf_label_)
                grounding_sdf_label_->setText(QString("SDF fit: mean=%1").arg(last_mean_sdf, 0, 'f', 4));

            QCoreApplication::processEvents(QEventLoop::AllEvents, 15);

            // M-step: optimize each class pose/shape with high-probability points via autograd SDF optimizer.
            float sdf_acc = 0.f;
            int sdf_count = 0;
            for (int k = 0; k < K; ++k)
            {
                std::vector<Eigen::Vector3f> class_points;
                class_points.reserve(static_cast<std::size_t>(N / std::max(1, K)) + 1);
                for (int i = 0; i < N; ++i)
                {
                    if (active_points[static_cast<std::size_t>(i)] == 0)
                        continue;
                    const float qik = q[static_cast<std::size_t>(i * (K + 1) + k)];
                    if (qik > 0.45f || point_classes[static_cast<std::size_t>(i)] == k)
                        class_points.emplace_back(points_world[static_cast<std::size_t>(i)]);
                }

                if (class_points.size() < 15)
                    continue;

                const QString class_label_lc = QString::fromStdString(classes[static_cast<std::size_t>(k)].label).toLower();
                if (class_label_lc.contains("mesa") || class_label_lc.contains("table"))
                {
                    auto estimate_obb = [](const std::vector<Eigen::Vector2f>& poly,
                                           Eigen::Vector2f& center,
                                           float& yaw,
                                           float& width,
                                           float& depth) -> bool
                    {
                        if (poly.size() < 3)
                            return false;

                        center = Eigen::Vector2f::Zero();
                        for (const auto& p : poly) center += p;
                        center /= static_cast<float>(poly.size());

                        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
                        for (const auto& p : poly)
                        {
                            const Eigen::Vector2f d = p - center;
                            cov += d * d.transpose();
                        }
                        cov /= static_cast<float>(std::max<std::size_t>(1, poly.size()));

                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
                        Eigen::Vector2f ux(1.f, 0.f);
                        if (eig.info() == Eigen::Success)
                            ux = eig.eigenvectors().col(1).normalized();
                        Eigen::Vector2f uy(-ux.y(), ux.x());

                        float min_u = std::numeric_limits<float>::max();
                        float max_u = -std::numeric_limits<float>::max();
                        float min_v = std::numeric_limits<float>::max();
                        float max_v = -std::numeric_limits<float>::max();
                        for (const auto& p : poly)
                        {
                            const Eigen::Vector2f d = p - center;
                            const float u = d.dot(ux);
                            const float v = d.dot(uy);
                            min_u = std::min(min_u, u);
                            max_u = std::max(max_u, u);
                            min_v = std::min(min_v, v);
                            max_v = std::max(max_v, v);
                        }

                        yaw = std::atan2(ux.y(), ux.x());
                        width = std::max(0.08f, max_u - min_u);
                        depth = std::max(0.08f, max_v - min_v);
                        return true;
                    };

                    auto make_rect = [](float tx, float ty, float yaw, float w, float d) -> std::vector<Eigen::Vector2f>
                    {
                        const float hw = 0.5f * std::max(0.08f, w);
                        const float hd = 0.5f * std::max(0.08f, d);
                        const Eigen::Vector2f cx(tx, ty);
                        const Eigen::Vector2f ux(std::cos(yaw), std::sin(yaw));
                        const Eigen::Vector2f uy(-ux.y(), ux.x());

                        std::vector<Eigen::Vector2f> out;
                        out.reserve(4);
                        out.emplace_back(cx + ux * (-hw) + uy * (-hd));
                        out.emplace_back(cx + ux * ( hw) + uy * (-hd));
                        out.emplace_back(cx + ux * ( hw) + uy * ( hd));
                        out.emplace_back(cx + ux * (-hw) + uy * ( hd));
                        return out;
                    };

                    try
                    {
                        Eigen::Vector2f cxy = Eigen::Vector2f::Zero();
                        float yaw0 = 0.f;
                        float w0 = 1.f;
                        float d0 = 0.8f;
                        const bool obb_ok = estimate_obb(classes[static_cast<std::size_t>(k)].vertices, cxy, yaw0, w0, d0);
                        if (!obb_ok)
                            throw std::runtime_error("Cannot estimate initial OBB for table class");

                        std::vector<float> xyz;
                        xyz.reserve(class_points.size() * 3);
                        for (const auto& p : class_points)
                        {
                            xyz.push_back(p.x());
                            xyz.push_back(p.y());
                            xyz.push_back(p.z());
                        }

                        auto pts = torch::from_blob(xyz.data(),
                                                    {static_cast<long>(class_points.size()), 3},
                                                    torch::TensorOptions().dtype(torch::kFloat32)).clone();

                        const float h0 = std::max(0.20f, classes[static_cast<std::size_t>(k)].height);
                        auto init_params = torch::tensor({cxy.x(), cxy.y(), yaw0, w0, d0, h0},
                                                         torch::TensorOptions().dtype(torch::kFloat32));

                        rc::object_models::TableAnalyticModel table_model(init_params);
                        auto fit = table_model.fit_autograd(pts, 70, 0.025f);
                        if (fit.ok)
                        {
                            const auto p = fit.params.to(torch::kCPU);
                            const float tx = p[0].item<float>();
                            const float ty = p[1].item<float>();
                            const float yaw = p[2].item<float>();
                            const float fw = std::max(0.08f, p[3].item<float>());
                            const float fd = std::max(0.08f, p[4].item<float>());
                            const float fh = std::max(0.20f, p[5].item<float>());

                            classes[static_cast<std::size_t>(k)].vertices = make_rect(tx, ty, yaw, fw, fd);
                            classes[static_cast<std::size_t>(k)].height = fh;
                            classes[static_cast<std::size_t>(k)].last_sdf = fit.final_loss;
                            sdf_acc += fit.final_loss;
                            ++sdf_count;
                            continue;
                        }
                    }
                    catch (const std::exception&)
                    {
                        // Fallback to generic mesh optimizer below.
                    }
                }

                auto result = mesh_sdf_optimizer_.optimize_mesh_with_pose(class_points,
                                                                           classes[static_cast<std::size_t>(k)].vertices,
                                                                           room_polygon_);
                if (!result.ok)
                    continue;

                classes[static_cast<std::size_t>(k)].vertices = result.vertices;
                classes[static_cast<std::size_t>(k)].last_sdf = result.final_data_loss;
                sdf_acc += result.final_data_loss;
                ++sdf_count;
            }

            last_mean_sdf = (sdf_count > 0) ? (sdf_acc / static_cast<float>(sdf_count)) : last_mean_sdf;
            QThread::msleep(70);
        }

        if (grounding_status_label_)
            grounding_status_label_->setText("Status: EM validator finished");

        int valid_candidates = 0;
        int improved_candidates = 0;
        int non_improved = 0;
        pending_em_adjustments_.clear();
        for (const auto& c : classes)
        {
            if (c.furniture_index < 0 || c.furniture_index >= static_cast<int>(furniture_polygons_.size()))
                continue;
            if (c.vertices.size() < 3)
                continue;
            if (!std::isfinite(c.last_sdf) || c.last_sdf <= 0.f)
                continue;

            const auto prev = c.prev_sdf;
            const bool improved = !prev.has_value() || (c.last_sdf + 1e-4f < prev.value());
            if (improved) ++improved_candidates;
            else ++non_improved;

            PendingEmAdjustment cand;
            cand.furniture_index = c.furniture_index;
            cand.vertices = c.vertices;
            cand.new_sdf = c.last_sdf;
            cand.prev_sdf = prev;
            cand.label = c.label;
            pending_em_adjustments_.push_back(std::move(cand));
            ++valid_candidates;
        }

        if (valid_candidates > 0)
        {
            em_decision_pending_ = true;
            camera_viewer_->set_em_decision_buttons_visible(true);
            if (grounding_status_label_)
                grounding_status_label_->setText(QString("Status: EM finished (%1 candidates, %2 improved). Accept/Reject pending")
                                                     .arg(valid_candidates)
                                                     .arg(improved_candidates));
        }
        else
        {
            em_decision_pending_ = false;
            camera_viewer_->set_em_decision_buttons_visible(false);
            if (grounding_status_label_)
                grounding_status_label_->setText("Status: EM validator finished (no valid fitted candidates)");
        }

        if (!points_cam.empty() && points_cam.size() == point_classes.size())
        {
            const QString final_title = (prev_sdf_count > 0)
                ? QString("EM done | prev=%1 cur=%2")
                      .arg(prev_mean_sdf, 0, 'f', 4)
                      .arg(last_mean_sdf, 0, 'f', 4)
                : QString("EM done | prev=-- cur=%1").arg(last_mean_sdf, 0, 'f', 4);
            camera_viewer_->set_em_points_overlay(points_cam, point_classes, class_colors, class_labels, final_title);
        }

        keep_overlay_lock = true;
        em_overlay_persistent_active_ = true;
    }
    catch (const Ice::Exception&)
    {
        if (grounding_status_label_)
            grounding_status_label_->setText("Status: EM validator failed (segmentation unavailable)");
        em_overlay_persistent_active_ = false;
    }
}

void SpecificWorker::apply_pending_em_adjustments(bool accept)
{
    if (!camera_viewer_)
        return;

    if (!em_decision_pending_)
    {
        camera_viewer_->set_em_decision_buttons_visible(false);
        return;
    }

    int applied = 0;
    if (accept)
    {
        for (const auto& cand : pending_em_adjustments_)
        {
            if (cand.furniture_index < 0 || cand.furniture_index >= static_cast<int>(furniture_polygons_.size()))
                continue;

            if (rc::SceneGraphAdapter::accept_fit_if_improved(scene_graph_,
                                                               furniture_polygons_[static_cast<std::size_t>(cand.furniture_index)],
                                                               cand.vertices,
                                                               cand.new_sdf))
            {
                ++applied;
            }
            else
            {
                // Temporary fallback path for manual validation: force apply on explicit user Accept.
                auto& fp = furniture_polygons_[static_cast<std::size_t>(cand.furniture_index)];
                fp.vertices = cand.vertices;
                fp.last_fit_sdf = cand.new_sdf;
                fp.height = std::max(0.2f, model_height_from_label(fp.label.empty() ? fp.id : fp.label));

                rc::SceneGraphObject obj;
                obj.id = fp.id;
                obj.label = fp.label;
                obj.vertices = fp.vertices;
                obj.last_fit_sdf = fp.last_fit_sdf;
                obj.frame_yaw_inward_rad = fp.frame_yaw_inward_rad;
                obj.height = fp.height;
                if (scene_graph_.upsert_object(obj))
                    ++applied;
            }
        }

        if (applied > 0)
        {
            draw_furniture();
            save_scene_graph_to_usd();
            if (grounding_status_label_)
                grounding_status_label_->setText(QString("Status: EM adjustments accepted (%1 applied)").arg(applied));
            if (grounding_sdf_label_)
                grounding_sdf_label_->setText(QString("SDF fit: accepted %1").arg(applied));
        }
        else
        {
            if (grounding_status_label_)
                grounding_status_label_->setText("Status: EM adjustments accepted (no applicable updates)");
        }
    }
    else
    {
        if (grounding_status_label_)
            grounding_status_label_->setText("Status: EM adjustments rejected");
    }

    em_decision_pending_ = false;
    pending_em_adjustments_.clear();
    camera_viewer_->set_em_decision_buttons_visible(false);
    em_overlay_persistent_active_ = false;
    em_validator_overlay_lock_ = false;
    camera_viewer_->clear_em_points_overlay();
    camera_viewer_->clear_wireframe_overlay();
}

float SpecificWorker::get_cpu_usage()
{
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);

    clock_t user_time = usage.ru_utime.tv_sec * 1000000 + usage.ru_utime.tv_usec;
    clock_t sys_time = usage.ru_stime.tv_sec * 1000000 + usage.ru_stime.tv_usec;

    clock_t user_diff = user_time - last_user_cpu_time_;
    clock_t sys_diff = sys_time - last_sys_cpu_time_;

    last_user_cpu_time_ = user_time;
    last_sys_cpu_time_ = sys_time;

    // CPU usage as percentage (normalized by number of processors)
    static auto last_call = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - last_call).count();
    last_call = now;

    if (elapsed_us <= 0) return 0.0f;

    float cpu_percent = 100.0f * (user_diff + sys_diff) / static_cast<float>(elapsed_us);
    return cpu_percent;
}

void SpecificWorker::clear_path(bool stop_controller, bool clear_stored_path)
{
    object_final_align_active_ = false;
    object_align_cycles_ = 0;
    nav_target_object_center_.reset();
    nav_target_object_name_.clear();

    if (stop_controller && active_episode_.has_value())
        finish_episode("aborted");

    // Stop the trajectory controller if active
    if (stop_controller && trajectory_controller_.is_active())
    {
        trajectory_controller_.stop();
        try { omnirobot_proxy->setSpeedBase(0, 0, 0); } catch (...) {}
    }

    for (auto* item : path_draw_items_)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    path_draw_items_.clear();
    if (clear_stored_path)
        current_path_.clear();

    // Clear expanded obstacle boundaries
    for (auto* item : obstacle_expanded_items_)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    obstacle_expanded_items_.clear();

    if (target_marker_)
        target_marker_->setVisible(false);

    // Hide trajectory controller debug items
    for (auto& segs : traj_draw_items_)
        for (auto* item : segs)
            item->setVisible(false);
    if (traj_carrot_marker_) traj_carrot_marker_->setVisible(false);
    if (traj_robot_to_carrot_) traj_robot_to_carrot_->setVisible(false);

    if (viewer_3d_ && clear_stored_path)
        viewer_3d_->update_path({});
}

////////////////////////////////////////////////////////////////////////////////////////////////
/// Room Polygon Capture
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
/// Layout Save/Load
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::slot_robot_dragging(QPointF pos)
{
    // Don't move robot while capturing room polygon
    if (capturing_room_polygon)
        return;

    if (!loc_initialized_.load())
        return;

    // Get current robot orientation to preserve it during drag
    const auto current_state = get_loc_state();
    const float current_theta = current_state[4];  // [half_w, half_h, x, y, theta]

    // Move robot to new position, keeping orientation
    push_loc_command(LocCmdSetPose{static_cast<float>(pos.x()), static_cast<float>(pos.y()), current_theta});
}

void SpecificWorker::slot_robot_drag_end(QPointF pos)
{
    // Don't move robot while capturing room polygon
    if (capturing_room_polygon)
        return;

    if (!loc_initialized_.load())
        return;

    // Final position after drag
    const auto current_state = get_loc_state();
    const float current_theta = current_state[4];

    push_loc_command(LocCmdSetPose{static_cast<float>(pos.x()), static_cast<float>(pos.y()), current_theta});
    qInfo() << "Robot dragged to (" << pos.x() << "," << pos.y() << ")";
}

void SpecificWorker::slot_robot_rotate(QPointF pos)
{
    // Don't rotate robot while capturing room polygon
    if (capturing_room_polygon)
        return;

    if (!loc_initialized_.load())
        return;

    // Get current robot position
    const auto current_state = get_loc_state();
    const float robot_x = current_state[2];
    const float robot_y = current_state[3];

    // Calculate angle from robot to cursor position
    const float dx = pos.x() - robot_x;
    const float dy = pos.y() - robot_y;
    const float new_theta = std::atan2(dy, dx);

    // Update robot orientation only, keep position
    push_loc_command(LocCmdSetPose{robot_x, robot_y, new_theta});
}

////////////////////////////////////////////////////////////////////////////////////////////////
/// Pose Persistence
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::save_last_pose()
{
    if (!loc_initialized_.load())
        return;

    const auto state = get_loc_state();
    const float x = state[2];
    const float y = state[3];
    const float theta = state[4];

    QJsonObject root;
    root["x"] = static_cast<double>(x);
    root["y"] = static_cast<double>(y);
    root["theta"] = static_cast<double>(theta);
    root["flip_x"] = flip_x_applied_;
    root["flip_y"] = flip_y_applied_;
    root["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);

    QJsonDocument doc(root);
    QFile file(LAST_POSE_FILE);
    if (file.open(QIODevice::WriteOnly))
    {
        file.write(doc.toJson());
        file.close();
        qDebug() << "Last pose saved: (" << x << "," << y << "," << qRadiansToDegrees(theta) << "°)"
                 << "flip_x=" << flip_x_applied_ << "flip_y=" << flip_y_applied_;
    }
}

bool SpecificWorker::load_last_pose()
{
    QFile file(LAST_POSE_FILE);
    if (!file.open(QIODevice::ReadOnly))
    {
        qInfo() << "No saved pose file found at" << LAST_POSE_FILE;
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        qWarning() << "Failed to parse pose file:" << parseError.errorString();
        return false;
    }

    QJsonObject root = doc.object();
    const float x = static_cast<float>(root["x"].toDouble());
    const float y = static_cast<float>(root["y"].toDouble());
    const float theta = static_cast<float>(root["theta"].toDouble());
    const bool saved_flip_x = root["flip_x"].toBool(false);
    const bool saved_flip_y = root["flip_y"].toBool(false);

    qInfo() << "Loaded last pose: (" << x << "," << y << "," << qRadiansToDegrees(theta) << "°)";
    qInfo() << "  Flip state: flip_x=" << saved_flip_x << "flip_y=" << saved_flip_y;
    qInfo() << "  Saved at:" << root["timestamp"].toString();

    // Apply saved flip state - flip the polygon if needed
    // The polygon is loaded fresh from file, so we need to apply flips if they were saved
    // NOTE: This is called from initialize() BEFORE localization thread starts, so direct access is safe.
    if (saved_flip_x)
    {
        for (auto& vertex : room_polygon_)
            vertex.x() = -vertex.x();
        flip_x_applied_ = true;

        room_ai.set_polygon_room(room_polygon_);
        path_planner_.set_polygon(room_polygon_);
        draw_room_polygon();
        qInfo() << "Applied saved flip_x";
    }
    if (saved_flip_y)
    {
        for (auto& vertex : room_polygon_)
            vertex.y() = -vertex.y();
        flip_y_applied_ = true;

        room_ai.set_polygon_room(room_polygon_);
        path_planner_.set_polygon(room_polygon_);
        draw_room_polygon();
        qInfo() << "Applied saved flip_y";
    }

    // Set the pose in room_ai (direct access, before thread starts)
    room_ai.set_robot_pose(x, y, theta);

    return true;
}

void SpecificWorker::perform_grid_search(const std::vector<Eigen::Vector3f>& lidar_points)
{
    if (!room_ai.is_initialized())
    {
        qWarning() << "Cannot perform grid search: room_ai not initialized";
        return;
    }

    qInfo() << "Performing grid search for initial pose...";

    // Try to load last pose first
    if (load_last_pose())
    {
        // For now, we trust the saved pose - user can use Flip buttons if wrong
        qInfo() << "Using saved pose. If incorrect, use Flip X/Y buttons or drag the robot.";
        return;
    }

    // No saved pose or invalid - do full grid search
    const bool found = room_ai.grid_search_initial_pose(lidar_points, 0.5f, M_PI_4);

    if (found)
    {
        qInfo() << "Grid search found a valid initial pose";
    }
    else
    {
        qWarning() << "Grid search did not find a good pose. Manual adjustment may be needed.";
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////
/// Localization Thread
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::push_loc_command(LocCommand cmd)
{
    std::lock_guard lock(loc_cmd_mutex_);
    loc_pending_commands_.push_back(std::move(cmd));
}

void SpecificWorker::run_localization()
{
    qInfo() << "[LocThread] Localization thread started";

    auto wait_period = std::chrono::milliseconds(40);
    std::int64_t last_lidar_timestamp = -1;
    constexpr auto kMinWait = std::chrono::milliseconds(2);
    constexpr auto kMaxWait = std::chrono::milliseconds(100);

    while (!stop_localization_thread_.load())
    {
        // ===== 1. DRAIN PENDING UI COMMANDS =====
        {
            std::vector<LocCommand> cmds;
            {
                std::lock_guard lock(loc_cmd_mutex_);
                cmds.swap(loc_pending_commands_);
            }
            for (auto& cmd : cmds)
            {
                std::visit([this](auto&& arg)
                {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, LocCmdSetPolygon>)
                        room_ai.set_polygon_room(arg.vertices);
                    else if constexpr (std::is_same_v<T, LocCmdSetPose>)
                        room_ai.set_robot_pose(arg.x, arg.y, arg.theta);
                    else if constexpr (std::is_same_v<T, LocCmdGridSearch>)
                        room_ai.grid_search_initial_pose(arg.lidar_points, arg.grid_res, arg.angle_res);
                }, cmd);
            }
        }

        // ===== 2. CHECK INITIALIZATION =====
        if (!room_ai.is_initialized())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // ===== 3. READ LIDAR DATA =====
        const auto &[gt_, lidar_high_, lidar_low_] = buffer_sync.read_last();

        if (!lidar_high_.has_value())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // ===== 4. SNAPSHOT VELOCITY & ODOMETRY HISTORY =====
        auto vel_snap = velocity_buffer_.get_snapshot<0>();
        auto odom_snap = odometry_buffer_.get_snapshot<0>();

        // ===== 5. RUN LOCALIZATION UPDATE =====
        const auto res = room_ai.update(lidar_high_.value(), vel_snap, odom_snap);

        // ===== 6. PUBLISH RESULT =====
        {
            std::lock_guard lock(loc_result_mutex_);
            loc_result_ = res;
        }
        if (res.ok && !loc_initialized_.load())
            loc_initialized_ = true;

        // Pace with timestamp feedback from buffer_sync (similar to read_lidar hysteresis):
        // if lidar timestamp is repeated, slow down; if it advances, speed up.
        const auto current_lidar_timestamp = lidar_high_->second;
        const bool repeated_timestamp = (current_lidar_timestamp == last_lidar_timestamp);
        if (repeated_timestamp)
            wait_period = std::min(wait_period + std::chrono::milliseconds(1), kMaxWait);
        else
            wait_period = std::max(wait_period - std::chrono::milliseconds(1), kMinWait);

        last_lidar_timestamp = current_lidar_timestamp;
        std::this_thread::sleep_for(wait_period);
    }

    qInfo() << "[LocThread] Localization thread stopped";
}

///////////////////////////////////////////////////////////////////////////////
/////SUBSCRIPTION to sendData method from JoystickAdapter interface
//////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
	rc::VelocityCommand cmd;
	for (const auto &axis: data.axes)
	{
		if (axis.name == "rotate")
			cmd.rot = axis.value;
		else if (axis.name == "advance") // forward is positive Z. Right-hand rule
			cmd.adv_y = axis.value/1000.0f; // from mm/s to m/s
		else if (axis.name == "side")
			cmd.adv_x = 0.0f; // not lateral motion allowed
	}
    cmd.timestamp = std::chrono::high_resolution_clock::now();
    const auto ts = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
	velocity_buffer_.put<0>(std::move(cmd), ts);

	// Track joystick activity to override trajectory controller
	if (std::abs(cmd.adv_y) > 0.01f || std::abs(cmd.rot) > 0.01f)
	{
		last_joystick_time_ = std::chrono::steady_clock::now();
		if (trajectory_controller_.is_active())
		{
			trajectory_controller_.stop();
			clear_path();
			try { omnirobot_proxy->setSpeedBase(0, 0, 0); } catch (...) {}
			qInfo() << "Trajectory controller stopped: joystick override";
		}
	}
}

//SUBSCRIPTION to newFullPose method from FullPoseEstimationPub interface
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

////////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION of Navigator interface methods (called by external clients, e.g. GUI)
///////////////////////////////////////////////////////////////////////////////////
RoboCompNavigator::LayoutData SpecificWorker::Navigator_getLayout()
{
    RoboCompNavigator::LayoutData ret;
    ret.layout.reserve(room_polygon_.size());

    for (const auto &p : room_polygon_)
    {
        RoboCompNavigator::TPoint pt;
        pt.x = p.x();
        pt.y = p.y();
        ret.layout.push_back(pt);
    }

    ret.objects.reserve(furniture_polygons_.size());
    for (const auto &fp : furniture_polygons_)
    {
        RoboCompNavigator::TObject obj;
        obj.name = fp.label.empty() ? fp.id : fp.label;
        obj.layout.reserve(fp.vertices.size());
        for (const auto &v : fp.vertices)
        {
            RoboCompNavigator::TPoint pt;
            pt.x = v.x();
            pt.y = v.y();
            obj.layout.push_back(pt);
        }
        ret.objects.push_back(std::move(obj));
    }

    return ret;
}

RoboCompNavigator::Result SpecificWorker::Navigator_getPath(RoboCompNavigator::TPoint source, RoboCompNavigator::TPoint target, float safety)
{
    RoboCompNavigator::Result ret;
    ret.timestamp = static_cast<long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    ret.valid = false;

    if (!path_planner_.is_ready())
    {
        ret.errorMsg = "Planner not ready (layout not initialized)";
        return ret;
    }

    // IDSL points are in meters; internal planner works in meters
    const Eigen::Vector2f start_m(source.x, source.y);
    const Eigen::Vector2f goal_m(target.x, target.y);

    if (!path_planner_.is_inside(start_m) || !path_planner_.is_inside(goal_m))
    {
        ret.errorMsg = "Source or target is outside navigable area";
        return ret;
    }

    // Optional safety override (meters). Rebuild visibility graph if changed.
    if (safety > 0.f && std::fabs(path_planner_.params.robot_radius - safety) > 1e-4f)
    {
        path_planner_.params.robot_radius = safety;
        if (!room_polygon_.empty())
            path_planner_.set_polygon(room_polygon_);
        if (!furniture_polygons_.empty())
        {
            std::vector<std::vector<Eigen::Vector2f>> obstacles;
            obstacles.reserve(furniture_polygons_.size());
            for (const auto &fp : furniture_polygons_)
                obstacles.push_back(fp.vertices);
            path_planner_.set_obstacles(obstacles);
        }
    }

    const auto path = path_planner_.plan(start_m, goal_m);
    if (path.empty())
    {
        ret.errorMsg = "No path found";
        return ret;
    }

    ret.path.reserve(path.size());
    for (const auto &p : path)
    {
        RoboCompNavigator::TPoint pt;
        pt.x = p.x();
        pt.y = p.y();
        ret.path.push_back(pt);
    }

    // Draw requested path in local UI (without changing active controller state)
    draw_path_threadsafe(path);

    ret.valid = true;
    ret.errorMsg = "ok";
    return ret;
}

RoboCompNavigator::TPose SpecificWorker::Navigator_getRobotPose()
{
    const auto state = get_loc_state();

    RoboCompNavigator::TPose pose;
    pose.x = state[2];
    pose.y = state[3];
    pose.r = state[4];
    return pose;
}

RoboCompNavigator::NavigationStatus SpecificWorker::Navigator_getStatus()
{
    RoboCompNavigator::NavigationStatus status;
    const auto now = std::chrono::steady_clock::now();

    const auto state = get_loc_state();
    status.currentPosition.x = state[2];
    status.currentPosition.y = state[3];
    status.currentOrientation = state[4];

    status.currentTarget.x = 0.f;
    status.currentTarget.y = 0.f;
    status.distanceToTarget = 0.f;
    status.estimatedTime = 0.f;
    status.pathWaypointsRemaining = 0;

    const auto vel_tuple = velocity_buffer_.read_last<0>();
    const auto &vel_opt = std::get<0>(vel_tuple);
    float speed = 0.f;
    if (vel_opt.has_value())
        speed = std::hypot(vel_opt->adv_x, vel_opt->adv_y);
    status.currentSpeed = speed;

    if (!loc_initialized_.load())
    {
        low_speed_block_timer_active_ = false;
        status.state = RoboCompNavigator::NavigationState::ERROR;
        status.statusMessage = "Localization not initialized";
        return status;
    }

    if (!current_path_.empty())
    {
        const auto &goal = current_path_.back();
        status.currentTarget.x = goal.x();
        status.currentTarget.y = goal.y();
        status.distanceToTarget = (goal - Eigen::Vector2f(status.currentPosition.x, status.currentPosition.y)).norm();
        status.pathWaypointsRemaining = static_cast<int>(current_path_.size());
        if (speed > 1e-3f)
            status.estimatedTime = status.distanceToTarget / speed;
        else
            status.estimatedTime = -1.f;
    }

    if (trajectory_controller_.is_active())
    {
        const bool far_from_goal = status.distanceToTarget > trajectory_controller_.params.goal_threshold;
        const bool low_speed = speed < BLOCKED_SPEED_THRESHOLD;

        if (far_from_goal && low_speed)
        {
            if (!low_speed_block_timer_active_)
            {
                low_speed_block_timer_active_ = true;
                low_speed_block_start_ = now;
            }

            const float blocked_elapsed_s = std::chrono::duration<float>(now - low_speed_block_start_).count();
            if (blocked_elapsed_s >= BLOCKED_TIME_THRESHOLD_SEC)
            {
                status.state = RoboCompNavigator::NavigationState::BLOCKED;
                status.statusMessage = "Blocked: low speed while far from target";
                return status;
            }
        }
        else
        {
            low_speed_block_timer_active_ = false;
        }

        status.state = RoboCompNavigator::NavigationState::NAVIGATING;
        status.statusMessage = "Navigating";
    }
    else if (!current_path_.empty())
    {
        low_speed_block_timer_active_ = false;
        status.state = RoboCompNavigator::NavigationState::PAUSED;
        status.statusMessage = "Path available but controller paused";
    }
    else
    {
        low_speed_block_timer_active_ = false;
        status.state = RoboCompNavigator::NavigationState::IDLE;
        status.statusMessage = "Idle";
    }

    return status;
}

RoboCompNavigator::TPoint SpecificWorker::Navigator_gotoObject(std::string object)
{
    if (object.empty() || furniture_polygons_.empty())
        return {};

    const QString query = QString::fromStdString(object).trimmed();
    if (query.isEmpty())
        return {};

    const rc::FurniturePolygonData *selected = nullptr;

    // 1) exact match against label or id
    for (const auto &fp : furniture_polygons_)
    {
        const QString label = QString::fromStdString(fp.label);
        const QString id = QString::fromStdString(fp.id);
        if (label.compare(query, Qt::CaseInsensitive) == 0 || id.compare(query, Qt::CaseInsensitive) == 0)
        {
            selected = &fp;
            break;
        }
    }

    // 2) substring match if exact not found
    if (selected == nullptr)
    {
        for (const auto &fp : furniture_polygons_)
        {
            const QString label = QString::fromStdString(fp.label);
            const QString id = QString::fromStdString(fp.id);
            if (label.contains(query, Qt::CaseInsensitive) || id.contains(query, Qt::CaseInsensitive))
            {
                selected = &fp;
                break;
            }
        }
    }

    if (selected == nullptr || selected->vertices.empty())
        return {};

    Eigen::Vector2f object_center = Eigen::Vector2f::Zero();
    for (const auto &v : selected->vertices)
        object_center += v;
    object_center /= static_cast<float>(selected->vertices.size());

    const auto state = get_loc_state();
    const Eigen::Vector2f robot_pos_m(state[2], state[3]);

    // Compute closest point on selected object's boundary to current robot position.
    Eigen::Vector2f closest = selected->vertices.front();
    float best_d2 = std::numeric_limits<float>::max();
    const size_t n = selected->vertices.size();
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector2f a = selected->vertices[i];
        const Eigen::Vector2f b = selected->vertices[(i + 1) % n];
        const Eigen::Vector2f ab = b - a;
        const float denom = ab.squaredNorm();

        float t = 0.f;
        if (denom > 1e-8f)
            t = (robot_pos_m - a).dot(ab) / denom;
        if (t < 0.f) t = 0.f;
        if (t > 1.f) t = 1.f;

        const Eigen::Vector2f proj = a + t * ab;
        const float d2 = (robot_pos_m - proj).squaredNorm();
        if (d2 < best_d2)
        {
            best_d2 = d2;
            closest = proj;
        }
    }

    // Outward direction: from object boundary toward robot. If degenerate, use centroid heuristic.
    Eigen::Vector2f outward = robot_pos_m - closest;
    if (outward.squaredNorm() < 1e-8f)
    {
        Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
        for (const auto &v : selected->vertices)
            centroid += v;
        centroid /= static_cast<float>(selected->vertices.size());
        outward = closest - centroid;
    }
    if (outward.squaredNorm() < 1e-8f)
        outward = Eigen::Vector2f(1.f, 0.f);
    outward.normalize();

    const float base_offset = std::max(0.25f, path_planner_.params.robot_radius + 0.15f);

    RoboCompNavigator::TPoint source;
    source.x = state[2];
    source.y = state[3];

    // Try several stand-off distances from object boundary until path becomes feasible.
    for (const float scale : {1.f, 1.5f, 2.f, 2.5f})
    {
        const Eigen::Vector2f target_m = closest + outward * (base_offset * scale);
        if (!path_planner_.is_inside(target_m))
            continue;

        RoboCompNavigator::TPoint target;
        target.x = target_m.x();
        target.y = target_m.y();

        const auto res = Navigator_getPath(source, target, path_planner_.params.robot_radius);
        if (res.valid && res.path.size() >= 2)
        {
            std::vector<Eigen::Vector2f> path_m;
            path_m.reserve(res.path.size());
            for (const auto &p : res.path)
                path_m.emplace_back(p.x, p.y);

            current_path_ = path_m;
            trajectory_controller_.set_path(current_path_);
            draw_path_threadsafe(trajectory_controller_.get_path());

            nav_target_object_center_ = object_center;
            nav_target_object_name_ = selected->label.empty() ? selected->id : selected->label;
            object_align_cycles_ = 0;
            start_episode("goto_object", Eigen::Vector2f(target.x, target.y), nav_target_object_name_);

            qInfo() << "Navigator_gotoObject: path with" << current_path_.size() << "waypoints activated for"
                    << QString::fromStdString(nav_target_object_name_);
            return target;
        }
    }

    qWarning() << "Navigator_gotoObject: no reachable stand-off point for object"
               << QString::fromStdString(selected->label);
    return {};
}

void SpecificWorker::Navigator_resume()
{
    if (!current_path_.empty())
        trajectory_controller_.set_path(current_path_);

}

RoboCompNavigator::TPoint SpecificWorker::Navigator_gotoPoint(RoboCompNavigator::TPoint target)
{
    nav_target_object_center_.reset();
    nav_target_object_name_.clear();
    object_align_cycles_ = 0;

    auto state = get_loc_state();
    RoboCompNavigator::TPoint source;
    source.x = state[2];
    source.y = state[3];

    const auto res = Navigator_getPath(source, target, path_planner_.params.robot_radius);
    if (!res.valid || res.path.size() < 2)
    {
        qWarning() << "Navigator_setTarget failed:" << QString::fromStdString(res.errorMsg);
        return {};
    }

    std::vector<Eigen::Vector2f> path_m;
    path_m.reserve(res.path.size());
    for (const auto &p : res.path)
        path_m.emplace_back(p.x, p.y);

    current_path_ = path_m;
    trajectory_controller_.set_path(current_path_);

    // Redraw with the relaxed path so the viewer shows what the controller actually follows
    draw_path_threadsafe(trajectory_controller_.get_path());

    start_episode("goto_point", Eigen::Vector2f(target.x, target.y));

    qInfo() << "Navigator_setTarget: path with" << current_path_.size() << "waypoints activated";
    return target;

}

void SpecificWorker::Navigator_stop()
{
    nav_target_object_center_.reset();
    nav_target_object_name_.clear();
    object_align_cycles_ = 0;

    finish_episode("stopped");
    trajectory_controller_.stop();
    current_path_.clear();
    try { omnirobot_proxy->setSpeedBase(0, 0, 0); } catch (...) {}

}

////////////////////////////////////////////////////////////////////////////////////////////////
/// Subscription to emergencyState signal from Hibernation interface
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
    qInfo() << "Emergency worker";
}

void SpecificWorker::restore()
{
    qInfo() << "Restore worker";

}

int SpecificWorker::startup_check()
{
	qInfo() << "Startup check";
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}
