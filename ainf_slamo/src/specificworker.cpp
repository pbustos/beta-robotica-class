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
#include <QDataStream>
#include <QShortcut>
#include <unistd.h>  // For sysconf
#include <cmath>     // For std::fabs
#include <limits>    // For std::numeric_limits
#include <print>     // C++23 std::println
#include <random>    // For odometry noise
#include <numeric>
#include <algorithm>

float SpecificWorker::percentile_copy(std::vector<float> values, float q)
{
    if (values.empty())
        return 0.f;
    q = std::clamp(q, 0.f, 1.f);
    const std::size_t idx = static_cast<std::size_t>(q * static_cast<float>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + static_cast<long>(idx), values.end());
    return values[idx];
}

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

        grounding_layout->addWidget(grounding_title);
        grounding_layout->addWidget(grounding_status_label_);
        grounding_layout->addWidget(grounding_cam_label_);
        grounding_layout->addWidget(grounding_world_label_);
        grounding_layout->addWidget(grounding_score_label_);
        grounding_layout->addWidget(grounding_sdf_label_);
        grounding_layout->addWidget(grounding_fit_mesh_button_);
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
            {
                if (focused_model_index_ == picked_idx)
                    focused_model_index_ = -1;
                else
                    focused_model_index_ = picked_idx;
            }

            if (scene_tree_)
                scene_tree_->toggle_item_by_name(picked);
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
            grounding_focus_points_.clear();
            grounding_focus_label_.clear();
            grounding_world_index_ = -1;
            if (grounding_status_label_)
                grounding_status_label_->setText("Status: SDF fitting disabled");
            if (grounding_cam_label_)
                grounding_cam_label_->setText("Camera object: -");
            if (grounding_world_label_)
                grounding_world_label_->setText("World object: -");
            if (grounding_score_label_)
                grounding_score_label_->setText("Score: -");
            if (grounding_sdf_label_)
                grounding_sdf_label_->setText("SDF fit: disabled");
        });

        connect(grounding_reload_svg_button_, &QPushButton::clicked, this, [this]()
        {
            if (current_layout_file_.empty())
            {
                if (grounding_status_label_)
                    grounding_status_label_->setText("Status: no layout file to reload from");
                return;
            }

            if (QFile::exists(FITTED_MESHES_FILE))
            {
                if (!QFile::remove(FITTED_MESHES_FILE))
                {
                    if (grounding_status_label_)
                        grounding_status_label_->setText("Status: cannot remove fitted_meshes.json");
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
        connect(camera_viewer_.get(), &QDialog::finished, this, [this](int) {
            if (pushButton_camera) pushButton_camera->setChecked(false);
        });
    }

    if (scene_tree_)
    {
        connect(scene_tree_.get(), &SceneTreePanel::furnitureClicked, this, [this](const QString& name, bool selected)
        {
            if (selected)
                focused_model_index_ = find_furniture_index_by_name(name);
            else if (focused_model_index_ == find_furniture_index_by_name(name))
                focused_model_index_ = -1;
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

    if (camera_viewer_)
        update_camera_wireframe_overlay(res.robot_pose);

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
            finish_episode("success");
            send_velocity_command(0.f, 0.f, 0.f);
            auto cmd = rc::VelocityCommand(0.f, 0.f, 0.f);
            velocity_buffer_.put<0>(std::move(cmd), timestamp);
            clear_path();
            qInfo() << "[TrajectoryCtrl] Navigation complete.";
        }
        else
        {
            send_velocity_command(ctrl.adv, ctrl.side, ctrl.rot);
            auto cmd = rc::VelocityCommand(ctrl.side, ctrl.adv, ctrl.rot);
            velocity_buffer_.put<0>(std::move(cmd), timestamp);

            const float speed = std::hypot(ctrl.adv, ctrl.side);
            const float rot = std::fabs(ctrl.rot);
            const float cpu_usage = get_cpu_usage();
            const bool blocked_like = (ctrl.dist_to_goal > trajectory_controller_.params.goal_threshold)
                                   && (speed < BLOCKED_SPEED_THRESHOLD);
            update_episode_metrics(res, &ctrl, speed, rot, cpu_usage, mppi_ms, blocked_like);

            if (do_draw && draw_trajectories_) draw_trajectory_debug(ctrl, res.robot_pose);

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
        viz2_ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t3).count();
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

int SpecificWorker::find_furniture_index_by_name(const QString& name) const
{
    auto normalize = [](QString s)
    {
        s = s.trimmed();
        const int idx = s.indexOf(" (");
        if (idx > 0)
            s = s.left(idx);
        return s.toLower();
    };

    const QString target = normalize(name);
    if (target.isEmpty())
        return -1;

    int contains_match = -1;
    for (std::size_t i = 0; i < furniture_polygons_.size(); ++i)
    {
        const auto& fp = furniture_polygons_[i];
        const QString label = normalize(QString::fromStdString(fp.label));
        const QString id = normalize(QString::fromStdString(fp.id));
        if ((!label.isEmpty() && label == target) || (!id.isEmpty() && id == target))
            return static_cast<int>(i);

        if ((contains_match < 0) &&
            ((!label.isEmpty() && (label.contains(target) || target.contains(label))) ||
             (!id.isEmpty() && (id.contains(target) || target.contains(id)))))
            contains_match = static_cast<int>(i);
    }
    return contains_match;
}

float SpecificWorker::model_height_from_label(const std::string& label) const
{
    const QString ql = QString::fromStdString(label).toLower();
    if (ql.contains("silla") || ql.contains("chair")) return 0.95f;
    if (ql.contains("mesa") || ql.contains("table")) return 0.75f;
    if (ql.contains("banco") || ql.contains("bench")) return 0.48f;
    if (ql.contains("monitor") || ql.contains("pantalla") || ql.contains("screen")) return 1.10f;
    if (ql.contains("vitrina") || ql.contains("cabinet") || ql.contains("shelf")) return 1.60f;
    if (ql.contains("maceta") || ql.contains("plant") || ql.contains("pot")) return 0.60f;
    return 0.85f;
}

void SpecificWorker::update_camera_wireframe_overlay(const Eigen::Affine2f &robot_pose)
{
    if (!camera_viewer_)
        return;

    if (focused_model_index_ < 0 || focused_model_index_ >= static_cast<int>(furniture_polygons_.size()))
    {
        camera_viewer_->clear_wireframe_overlay();
        return;
    }

    const auto& fp = furniture_polygons_[focused_model_index_];
    if (fp.vertices.size() < 3)
    {
        camera_viewer_->clear_wireframe_overlay();
        return;
    }

    const Eigen::Affine2f world_to_robot = robot_pose.inverse();
    const float height = model_height_from_label(fp.label);

    auto to_camera = [this](const Eigen::Vector2f& p_world, float z_world) -> Eigen::Vector3f
    {
        // Camera frame in this component: x=lateral, y=forward, z=up.
        return Eigen::Vector3f(p_world.x() - params.CAMERA_TX,
                               p_world.y() - params.CAMERA_TY,
                               z_world - params.CAMERA_TZ);
    };

    std::vector<Eigen::Vector2f> poly_robot;
    poly_robot.reserve(fp.vertices.size());
    for (const auto& v : fp.vertices)
        poly_robot.emplace_back(world_to_robot * v);

    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> segments;
    segments.reserve(poly_robot.size() * 3);
    for (std::size_t i = 0; i < poly_robot.size(); ++i)
    {
        const std::size_t j = (i + 1) % poly_robot.size();
        const Eigen::Vector3f b0 = to_camera(poly_robot[i], 0.f);
        const Eigen::Vector3f b1 = to_camera(poly_robot[j], 0.f);
        const Eigen::Vector3f t0 = to_camera(poly_robot[i], height);
        const Eigen::Vector3f t1 = to_camera(poly_robot[j], height);

        segments.emplace_back(b0, b1); // base
        segments.emplace_back(t0, t1); // top
        segments.emplace_back(b0, t0); // vertical
    }

    const QString wire_label = QString::fromStdString(fp.label.empty() ? fp.id : fp.label);
    camera_viewer_->set_wireframe_segments_camera(segments, wire_label);
}

void SpecificWorker::update_segmented_points_3d(const Eigen::Affine2f &robot_pose)
{
    (void)robot_pose;
    if (!viewer_3d_)
        return;

    // Segmented 3D object processing/projection is intentionally disabled.
    viewer_3d_->update_segmented_points({});
    viewer_3d_->update_segmented_boxes({});
    grounding_focus_points_.clear();
    grounding_focus_label_.clear();
    grounding_world_index_ = -1;
    if (grounding_status_label_ && grounding_cam_label_ && grounding_world_label_ && grounding_score_label_)
    {
        grounding_status_label_->setText("Status: segmented 3D grounding disabled");
        grounding_cam_label_->setText("Camera object: -");
        grounding_world_label_->setText("World object: -");
        grounding_score_label_->setText("Score: -");
    }
    return;

    try
    {
        const auto tdata = imagesegmentation_proxy->getAll();
        std::vector<Eigen::Vector3f> points_layout;
        std::vector<rc::Viewer3D::SegmentedBoxItem> boxes_layout;
        boxes_layout.reserve(1);

        struct CamObjInfo
        {
            bool valid = false;
            std::string label;
            Eigen::Vector2f center = Eigen::Vector2f::Zero();
            Eigen::Vector3f size = Eigen::Vector3f::Zero();
            float yaw_rad = 0.f;
            float img_dist = std::numeric_limits<float>::max();
            float img_area = 0.f;
        };
        std::vector<CamObjInfo> cam_infos(tdata.objects.size());
        std::vector<std::vector<Eigen::Vector3f>> obj_points_by_idx(tdata.objects.size());
        std::vector<rc::Viewer3D::SegmentedBoxItem> obj_box_by_idx(tdata.objects.size());
        std::vector<bool> obj_box_valid(tdata.objects.size(), false);

        std::size_t fallback_idx = tdata.objects.size();
        std::size_t fallback_points = 0;

        for (std::size_t obj_idx = 0; obj_idx < tdata.objects.size(); ++obj_idx)
        {
            const auto &obj = tdata.objects[obj_idx];
            std::vector<Eigen::Vector3f> obj_points_layout_all;
            obj_points_layout_all.reserve(obj.points3D.size());

            if (obj.points3D.size() > fallback_points)
            {
                fallback_points = obj.points3D.size();
                fallback_idx = obj_idx;
            }

            for (const auto &p : obj.points3D)
            {
                // points3D now come in RoboComp local frame.
                // Convert camera frame -> robot frame via translation extrinsics,
                // then robot frame -> local layout frame, then local layout -> global layout.
                const float x_robot = p.x + params.CAMERA_TX;
                const float y_robot = p.y + params.CAMERA_TY;
                const float z_robot = p.z + params.CAMERA_TZ;

                const Eigen::Vector2f p_layout_local(x_robot, y_robot);
                const Eigen::Vector2f p_layout = robot_pose * p_layout_local;
                constexpr float z_lift = 0.05f;  // keep points slightly above floor
                const Eigen::Vector3f p3(p_layout.x(), p_layout.y(), z_robot + z_lift);
                obj_points_layout_all.emplace_back(p3);
            }

            std::vector<Eigen::Vector3f> obj_points_layout;
            obj_points_layout.reserve(obj_points_layout_all.size());
            if (!obj_points_layout_all.empty())
            {
                float min_obj_z = std::numeric_limits<float>::max();
                for (const auto &p : obj_points_layout_all)
                    min_obj_z = std::min(min_obj_z, p.z());

                constexpr float floor_clip_margin = 0.06f;  // remove points near per-object floor level
                const float floor_z = min_obj_z + floor_clip_margin;
                for (const auto &p : obj_points_layout_all)
                {
                    if (p.z() > floor_z)
                        obj_points_layout.emplace_back(p);
                }

                // If filtering removed too much, keep original to avoid losing tiny objects
                if (obj_points_layout.size() < 8)
                    obj_points_layout = obj_points_layout_all;
            }

            if (obj_points_layout.size() >= 4)
            {
                Eigen::Vector2f mean_xy = Eigen::Vector2f::Zero();
                float min_z = std::numeric_limits<float>::max();
                float max_z = -std::numeric_limits<float>::max();
                for (const auto &p : obj_points_layout)
                {
                    mean_xy += Eigen::Vector2f(p.x(), p.y());
                    min_z = std::min(min_z, p.z());
                    max_z = std::max(max_z, p.z());
                }
                mean_xy /= static_cast<float>(obj_points_layout.size());

                Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
                for (const auto &p : obj_points_layout)
                {
                    const Eigen::Vector2f d = Eigen::Vector2f(p.x(), p.y()) - mean_xy;
                    cov += d * d.transpose();
                }
                cov /= static_cast<float>(obj_points_layout.size());

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
                Eigen::Vector2f major_axis(1.f, 0.f);
                if (eig.info() == Eigen::Success)
                    major_axis = eig.eigenvectors().col(1).normalized();
                const Eigen::Vector2f minor_axis(-major_axis.y(), major_axis.x());

                float min_u = std::numeric_limits<float>::max();
                float max_u = -std::numeric_limits<float>::max();
                float min_v = std::numeric_limits<float>::max();
                float max_v = -std::numeric_limits<float>::max();
                for (const auto &p : obj_points_layout)
                {
                    const Eigen::Vector2f d = Eigen::Vector2f(p.x(), p.y()) - mean_xy;
                    const float u = d.dot(major_axis);
                    const float v = d.dot(minor_axis);
                    min_u = std::min(min_u, u); max_u = std::max(max_u, u);
                    min_v = std::min(min_v, v); max_v = std::max(max_v, v);
                }

                const float c_u = 0.5f * (min_u + max_u);
                const float c_v = 0.5f * (min_v + max_v);
                const Eigen::Vector2f center_xy = mean_xy + major_axis * c_u + minor_axis * c_v;

                rc::Viewer3D::SegmentedBoxItem box;
                box.label = obj.label.empty() ? ("object_" + std::to_string(obj_idx + 1)) : obj.label;
                box.center = center_xy;
                box.center_height = 0.5f * (min_z + max_z);
                box.size = Eigen::Vector3f(
                    std::max(0.08f, max_u - min_u),
                    std::max(0.08f, max_z - min_z),
                    std::max(0.08f, max_v - min_v));
                box.yaw_rad = std::atan2(major_axis.y(), major_axis.x());

                CamObjInfo &ci = cam_infos[obj_idx];
                ci.valid = true;
                ci.label = obj.label.empty() ? ("object_" + std::to_string(obj_idx + 1)) : obj.label;
                ci.center = center_xy;
                ci.size = box.size;
                ci.yaw_rad = box.yaw_rad;

                obj_points_by_idx[obj_idx] = std::move(obj_points_layout);
                obj_box_by_idx[obj_idx] = box;
                obj_box_valid[obj_idx] = true;
            }
        }

        const Eigen::Affine2f world_to_robot = robot_pose.inverse();
        std::size_t focus_idx = tdata.objects.size();
        constexpr float min_forward_dist = 0.08f;
        constexpr float strict_center_cone = 12.f * static_cast<float>(M_PI) / 180.f;
        constexpr float relaxed_center_cone = 22.f * static_cast<float>(M_PI) / 180.f;

        auto choose_focus_in_cone = [&](float max_abs_bearing) -> std::size_t
        {
            std::size_t best_idx = tdata.objects.size();
            float best_forward = std::numeric_limits<float>::max();
            float best_abs_bearing = std::numeric_limits<float>::max();

            for (std::size_t i = 0; i < cam_infos.size(); ++i)
            {
                if (!cam_infos[i].valid) continue;

                const Eigen::Vector2f c_robot = world_to_robot * cam_infos[i].center;
                const float forward = c_robot.y();
                if (forward <= min_forward_dist) continue;  // must be in front of robot

                const float abs_bearing = std::abs(std::atan2(c_robot.x(), c_robot.y()));
                if (abs_bearing > max_abs_bearing) continue; // must be centered enough

                // First object in line of sight (nearest forward) with center tie-break.
                const bool better_forward = forward < best_forward - 1e-3f;
                const bool same_forward = std::abs(forward - best_forward) <= 1e-3f;
                const bool better_bearing = abs_bearing < best_abs_bearing;
                if (better_forward || (same_forward && better_bearing))
                {
                    best_forward = forward;
                    best_abs_bearing = abs_bearing;
                    best_idx = i;
                }
            }
            return best_idx;
        };

        focus_idx = choose_focus_in_cone(strict_center_cone);
        if (focus_idx >= tdata.objects.size())
            focus_idx = choose_focus_in_cone(relaxed_center_cone);

        if (focus_idx >= tdata.objects.size())
        {
            // Last fallback: nearest bearing among forward objects.
            float best_abs_bearing = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < cam_infos.size(); ++i)
            {
                if (!cam_infos[i].valid) continue;
                const Eigen::Vector2f c_robot = world_to_robot * cam_infos[i].center;
                if (c_robot.y() <= min_forward_dist) continue;
                const float abs_bearing = std::abs(std::atan2(c_robot.x(), c_robot.y()));
                if (abs_bearing < best_abs_bearing)
                {
                    best_abs_bearing = abs_bearing;
                    focus_idx = i;
                }
            }
        }

        if (focus_idx >= tdata.objects.size())
            focus_idx = fallback_idx;

        if (focus_idx < obj_points_by_idx.size() && obj_box_valid[focus_idx])
        {
            points_layout = obj_points_by_idx[focus_idx];
            boxes_layout.clear();
            boxes_layout.emplace_back(obj_box_by_idx[focus_idx]);
            grounding_focus_points_ = points_layout;
            grounding_focus_center_ = cam_infos[focus_idx].center;
            grounding_focus_label_ = cam_infos[focus_idx].label;
        }

        if (focus_idx < cam_infos.size() && cam_infos[focus_idx].valid)
        {
            grounding_focus_center_ = cam_infos[focus_idx].center;
            grounding_focus_label_ = cam_infos[focus_idx].label;
        }

        viewer_3d_->update_segmented_points(points_layout);
        viewer_3d_->update_segmented_boxes(boxes_layout);

        if (grounding_status_label_ && grounding_cam_label_ && grounding_world_label_ && grounding_score_label_)
        {
            if (focus_idx >= cam_infos.size() || !cam_infos[focus_idx].valid)
            {
                grounding_focus_points_.clear();
                grounding_focus_label_.clear();
                grounding_world_index_ = -1;
                grounding_status_label_->setText("Status: no valid camera box to ground");
                grounding_cam_label_->setText("Camera object: -");
                grounding_world_label_->setText("World object: -");
                grounding_score_label_->setText("Score: -");
            }
            else
            {
                const auto &cam = cam_infos[focus_idx];
                grounding_cam_label_->setText(QString("Camera object: %1").arg(QString::fromStdString(cam.label)));

                const Eigen::Vector2f robot_pos = robot_pose.translation();
                Eigen::Vector2f forward = robot_pose.linear() * Eigen::Vector2f(0.f, 1.f);
                if (forward.norm() < 1e-6f) forward = Eigen::Vector2f(0.f, 1.f);
                forward.normalize();
                const Eigen::Vector2f left(-forward.y(), forward.x());

                const Eigen::Vector2f cam_center_robot = world_to_robot * cam.center;
                const float cam_bearing = std::atan2(cam_center_robot.x(), cam_center_robot.y());
                auto wrap_angle = [](float a)
                {
                    while (a > static_cast<float>(M_PI)) a -= static_cast<float>(2.0 * M_PI);
                    while (a < static_cast<float>(-M_PI)) a += static_cast<float>(2.0 * M_PI);
                    return a;
                };
                constexpr float max_bearing_diff = 35.f * static_cast<float>(M_PI) / 180.f;

                float best_score = -1.f;
                QString best_world = "-";
                float best_dist = 0.f, best_lat = 0.f;
                float best_size_ratio = 1.f;
                float best_center_dist = 0.f;
                float best_bearing_deg = 0.f;
                int best_world_index = -1;
                bool matched_with_relaxed_gate = false;

                auto evaluate_candidates = [&](bool strict_bearing_gate)
                {
                    float local_best_score = -1.f;
                    QString local_best_world = "-";
                    float local_best_dist = 0.f, local_best_lat = 0.f;
                    float local_best_size_ratio = 1.f;
                    float local_best_center_dist = 0.f;
                    float local_best_bearing_deg = 0.f;
                    int local_best_world_index = -1;

                    for (std::size_t world_idx = 0; world_idx < furniture_polygons_.size(); ++world_idx)
                    {
                        const auto &fp = furniture_polygons_[world_idx];
                        if (fp.vertices.size() < 3) continue;

                        Eigen::Vector2f cen = Eigen::Vector2f::Zero();
                        float min_x = fp.vertices[0].x(), max_x = fp.vertices[0].x();
                        float min_y = fp.vertices[0].y(), max_y = fp.vertices[0].y();
                        for (const auto &v : fp.vertices)
                        {
                            cen += v;
                            min_x = std::min(min_x, v.x()); max_x = std::max(max_x, v.x());
                            min_y = std::min(min_y, v.y()); max_y = std::max(max_y, v.y());
                        }
                        cen /= static_cast<float>(fp.vertices.size());

                        const Eigen::Vector2f diff = cen - robot_pos;
                        const float along = diff.dot(forward);
                        if (along <= 0.05f) continue;
                        const float lateral = std::abs(diff.dot(left));
                        const float dist = diff.norm();

                        const Eigen::Vector2f w_robot = world_to_robot * cen;
                        const float world_bearing = std::atan2(w_robot.x(), w_robot.y());
                        const float bearing_diff = std::abs(wrap_angle(world_bearing - cam_bearing));
                        const bool bearing_gate_ok = bearing_diff <= max_bearing_diff;

                        const float center_dist = (cen - cam.center).norm();

                        const float world_sx = std::max(0.08f, max_x - min_x);
                        const float world_sz = std::max(0.08f, max_y - min_y);
                        const float cam_area = std::max(0.01f, cam.size.x() * cam.size.z());
                        const float world_area = std::max(0.01f, world_sx * world_sz);
                        const float size_ratio = cam_area / world_area;

                        const float score_pose = 1.f / (1.f + dist);
                        const float score_lat = 1.f / (1.f + lateral);
                        const float score_center = 1.f / (1.f + center_dist);
                        const float score_bearing = 1.f - std::min(1.f, bearing_diff / (2.f * max_bearing_diff));
                        const float score_size = 1.f / (1.f + std::abs(std::log(size_ratio)));

                        const QString qcam = QString::fromStdString(cam.label).toLower();
                        const QString qworld = QString::fromStdString(fp.label).toLower();
                        const float score_label = (qcam.contains(qworld) || qworld.contains(qcam)) ? 1.f : 0.f;

                        const float total = strict_bearing_gate
                            ? (0.15f * score_pose + 0.10f * score_lat + 0.45f * score_center + 0.20f * score_bearing + 0.08f * score_size + 0.02f * score_label)
                            : (0.18f * score_pose + 0.12f * score_lat + 0.52f * score_center + 0.10f * score_bearing + 0.06f * score_size + 0.02f * score_label);

                        if (strict_bearing_gate && !bearing_gate_ok) continue;

                        if (total > local_best_score)
                        {
                            local_best_score = total;
                            local_best_world = QString::fromStdString(fp.label.empty() ? fp.id : fp.label);
                            local_best_dist = dist;
                            local_best_lat = lateral;
                            local_best_size_ratio = size_ratio;
                            local_best_center_dist = center_dist;
                            local_best_bearing_deg = qRadiansToDegrees(bearing_diff);
                            local_best_world_index = static_cast<int>(world_idx);
                        }
                    }

                    if (local_best_score > 0.f)
                    {
                        best_score = local_best_score;
                        best_world = local_best_world;
                        best_dist = local_best_dist;
                        best_lat = local_best_lat;
                        best_size_ratio = local_best_size_ratio;
                        best_center_dist = local_best_center_dist;
                        best_bearing_deg = local_best_bearing_deg;
                        best_world_index = local_best_world_index;
                        return true;
                    }
                    return false;
                };

                const bool strict_match = evaluate_candidates(true);
                if (!strict_match)
                {
                    const bool relaxed_match = evaluate_candidates(false);
                    matched_with_relaxed_gate = relaxed_match;
                }

                if (best_score > 0.f)
                {
                    grounding_world_index_ = best_world_index;
                    grounding_status_label_->setText(matched_with_relaxed_gate
                        ? "Status: grounded to front world object (relaxed alignment)"
                        : "Status: grounded to front world object");
                    grounding_world_label_->setText(QString("World object: %1").arg(best_world));
                    grounding_score_label_->setText(
                        QString("Score: %1 | center=%2m d=%3m lat=%4m ang=%5° areaRatio=%6")
                            .arg(best_score, 0, 'f', 2)
                            .arg(best_center_dist, 0, 'f', 2)
                            .arg(best_dist, 0, 'f', 2)
                            .arg(best_lat, 0, 'f', 2)
                            .arg(best_bearing_deg, 0, 'f', 1)
                            .arg(best_size_ratio, 0, 'f', 2));
                }
                else
                {
                    grounding_world_index_ = -1;
                    grounding_status_label_->setText("Status: no front candidate aligned with camera object");
                    grounding_world_label_->setText("World object: -");
                    grounding_score_label_->setText("Score: -");
                }
            }
        }
    }
    catch (const Ice::Exception &)
    {
        viewer_3d_->update_segmented_points({});
        viewer_3d_->update_segmented_boxes({});
        grounding_focus_points_.clear();
        grounding_focus_label_.clear();
        grounding_world_index_ = -1;
        if (grounding_status_label_ && grounding_cam_label_ && grounding_world_label_ && grounding_score_label_)
        {
            grounding_status_label_->setText("Status: segmentation unavailable");
            grounding_cam_label_->setText("Camera object: -");
            grounding_world_label_->setText("World object: -");
            grounding_score_label_->setText("Score: -");
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////
/// Methods
/////////////////////////////////////////////////////////////////////////////////

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

void SpecificWorker::start_episode(const std::string &mission_type,
                                   const std::optional<Eigen::Vector2f> &target_point,
                                   const std::string &target_object)
{
    if (active_episode_.has_value())
        finish_episode("aborted");

    rc::EpisodicMemory::EpisodeRecord episode;
    episode.episode_id = rc::EpisodicMemory::make_episode_id();
    episode.start_ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    episode.status = "running";

    episode.mission.mission_type = mission_type;
    episode.mission.controller_version = "ainf_slamo_v2";
    episode.target.target_mode = target_point.has_value() ? "point" : (target_object.empty() ? "none" : "object");
    if (target_point.has_value())
    {
        episode.target.target_x = target_point->x();
        episode.target.target_y = target_point->y();
    }
    episode.target.target_object_id = target_object;

    const auto &p = trajectory_controller_.params;
    auto add_param = [&episode](const std::string &key, float value)
    {
        episode.params_snapshot[key] = value;
    };

    add_param("mood", p.mood);
    add_param("max_adv", p.max_adv);
    add_param("max_back_adv", p.max_back_adv);
    add_param("max_rot", p.max_rot);
    add_param("d_safe", p.d_safe);
    add_param("safety_priority_scale", p.safety_priority_scale);
    add_param("carrot_lookahead", p.carrot_lookahead);
    add_param("goal_threshold", p.goal_threshold);
    add_param("num_samples", static_cast<float>(p.num_samples));
    add_param("trajectory_steps", static_cast<float>(p.trajectory_steps));
    add_param("trajectory_dt", p.trajectory_dt);
    add_param("mppi_lambda", p.mppi_lambda);
    add_param("sigma_adv", p.sigma_adv);
    add_param("sigma_rot", p.sigma_rot);
    add_param("noise_alpha", p.noise_alpha);
    add_param("warm_start_adv_weight", p.warm_start_adv_weight);
    add_param("warm_start_rot_weight", p.warm_start_rot_weight);
    add_param("optim_iterations", static_cast<float>(p.optim_iterations));
    add_param("optim_lr", p.optim_lr);
    add_param("lambda_goal", p.lambda_goal);
    add_param("lambda_obstacle", p.lambda_obstacle);
    add_param("lambda_smooth", p.lambda_smooth);
    add_param("lambda_velocity", p.lambda_velocity);
    add_param("lambda_delta_vel", p.lambda_delta_vel);
    add_param("velocity_smoothing", p.velocity_smoothing);
    add_param("gauss_k", p.gauss_k);

    episode.mood_snapshot["mood"] = p.mood;
    episode.mood_snapshot["enable_mood"] = p.enable_mood ? 1.f : 0.f;
    episode.mood_snapshot["mood_speed_gain"] = p.mood_speed_gain;
    episode.mood_snapshot["mood_reactivity_gain"] = p.mood_reactivity_gain;
    episode.mood_snapshot["mood_caution_gain"] = p.mood_caution_gain;

    active_episode_ = std::move(episode);
    episode_accum_ = EpisodeAccum{};
    episode_accum_.last_block_sample = std::chrono::steady_clock::now();
    episode_accum_.last_metric_sample = episode_accum_.last_block_sample;
    episode_accum_.has_last_metric_sample = true;
    if (target_point.has_value())
    {
        const auto state = get_loc_state();
        episode_accum_.start_to_goal_dist_m = std::hypot(target_point->x() - static_cast<float>(state[2]),
                                                         target_point->y() - static_cast<float>(state[3]));
    }

    if (label_episodeStatus != nullptr)
    {
        const QString id = QString::fromStdString(active_episode_->episode_id);
        label_episodeStatus->setText("EP: REC " + id.right(6));
        label_episodeStatus->setStyleSheet("background-color: #B3E5FC; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
    }
}

void SpecificWorker::update_episode_metrics(const rc::RoomConceptAI::UpdateResult &res,
                                            const rc::TrajectoryController::ControlOutput *ctrl,
                                            float current_speed,
                                            float current_rot,
                                            float cpu_usage,
                                            float mppi_ms,
                                            bool blocked_state)
{
    if (!active_episode_.has_value())
        return;

    episode_accum_.n_cycles++;

    const auto now = std::chrono::steady_clock::now();
    float dt_s = 0.05f;
    if (episode_accum_.has_last_metric_sample)
        dt_s = std::max(1e-3f, std::chrono::duration<float>(now - episode_accum_.last_metric_sample).count());
    episode_accum_.last_metric_sample = now;
    episode_accum_.has_last_metric_sample = true;

    const Eigen::Vector2f pos(res.state[2], res.state[3]);
    if (episode_accum_.has_prev_pose)
    {
        const float raw_delta = (pos - episode_accum_.prev_pos).norm();
        const float max_cmd_delta = std::max(trajectory_controller_.params.max_adv, 0.01f) * dt_s;
        const float max_allowed_delta = 1.35f * max_cmd_delta + 0.01f;
        const float jitter_deadband = 0.002f;
        const float bounded_delta = std::clamp(raw_delta - jitter_deadband, 0.f, max_allowed_delta);
        episode_accum_.distance_traveled_m += bounded_delta;
    }
    episode_accum_.prev_pos = pos;
    episode_accum_.has_prev_pose = true;

    if (ctrl != nullptr)
    {
        episode_accum_.min_esdf_m = std::min(episode_accum_.min_esdf_m, ctrl->min_esdf);
        episode_accum_.ess_ratio_samples.push_back((ctrl->ess_K > 0) ? ctrl->ess / static_cast<float>(ctrl->ess_K) : 0.f);
    }

    episode_accum_.speed_samples.push_back(current_speed);
    episode_accum_.rot_samples.push_back(current_rot);
    episode_accum_.cpu_samples.push_back(cpu_usage);
    episode_accum_.mppi_ms_samples.push_back(mppi_ms);

    if (blocked_state)
    {
        if (!episode_accum_.was_blocked)
            episode_accum_.n_blocked_events++;
        const float dt = std::chrono::duration<float>(now - episode_accum_.last_block_sample).count();
        episode_accum_.blocked_time_s += std::max(0.f, dt);
        episode_accum_.was_blocked = true;
    }
    else
    {
        episode_accum_.was_blocked = false;
    }
    episode_accum_.last_block_sample = now;
}

void SpecificWorker::finish_episode(const std::string &status)
{
    if (!active_episode_.has_value())
        return;

    const std::string finished_episode_id = active_episode_->episode_id;

    auto &episode = active_episode_.value();
    episode.end_ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    episode.duration_s = std::max(0.f, static_cast<float>(episode.end_ts_ms - episode.start_ts_ms) / 1000.f);
    episode.status = status;

    episode.trajectory.n_cycles = episode_accum_.n_cycles;
    episode.trajectory.distance_traveled_m = episode_accum_.distance_traveled_m;

    const float target_dist = episode_accum_.start_to_goal_dist_m;
    episode.trajectory.path_efficiency = (episode_accum_.distance_traveled_m > 1e-6f)
        ? (target_dist / episode_accum_.distance_traveled_m)
        : 0.f;

    episode.trajectory.mean_speed = episode_accum_.speed_samples.empty() ? 0.f
        : std::accumulate(episode_accum_.speed_samples.begin(), episode_accum_.speed_samples.end(), 0.f)
          / static_cast<float>(episode_accum_.speed_samples.size());
    episode.trajectory.p95_speed = percentile_copy(episode_accum_.speed_samples, 0.95f);
    episode.trajectory.mean_rot = episode_accum_.rot_samples.empty() ? 0.f
        : std::accumulate(episode_accum_.rot_samples.begin(), episode_accum_.rot_samples.end(), 0.f)
          / static_cast<float>(episode_accum_.rot_samples.size());
    episode.trajectory.p95_rot = percentile_copy(episode_accum_.rot_samples, 0.95f);
    episode.trajectory.mean_ess_ratio = episode_accum_.ess_ratio_samples.empty() ? 0.f
        : std::accumulate(episode_accum_.ess_ratio_samples.begin(), episode_accum_.ess_ratio_samples.end(), 0.f)
          / static_cast<float>(episode_accum_.ess_ratio_samples.size());
    episode.trajectory.p05_ess_ratio = percentile_copy(episode_accum_.ess_ratio_samples, 0.05f);
    episode.trajectory.mean_cpu_pct = episode_accum_.cpu_samples.empty() ? 0.f
        : std::accumulate(episode_accum_.cpu_samples.begin(), episode_accum_.cpu_samples.end(), 0.f)
          / static_cast<float>(episode_accum_.cpu_samples.size());
    episode.trajectory.p95_mppi_ms = percentile_copy(episode_accum_.mppi_ms_samples, 0.95f);

    episode.safety.min_esdf_m = std::isfinite(episode_accum_.min_esdf_m) ? episode_accum_.min_esdf_m : 0.f;
    episode.safety.n_near_collision = 0;
    episode.safety.n_collision = (episode.safety.min_esdf_m < trajectory_controller_.params.robot_radius) ? 1 : 0;
    episode.safety.n_blocked_events = episode_accum_.n_blocked_events;
    episode.safety.blocked_time_s = episode_accum_.blocked_time_s;
    episode.safety.n_replans = 0;

    episode.outcome.time_to_goal_s = episode.duration_s;
    episode.outcome.success_binary = (status == "success") ? 1 : 0;
    episode.outcome.comfort_jerk_score = episode.trajectory.p95_rot;
    episode.outcome.safety_score = std::max(0.f, episode.safety.min_esdf_m);
    episode.outcome.efficiency_score = std::max(0.f, episode.trajectory.path_efficiency);
    episode.outcome.composite_score = 100.f * static_cast<float>(episode.outcome.success_binary)
                                    - episode.outcome.time_to_goal_s
                                    - 50.f * static_cast<float>(episode.safety.n_collision)
                                    - 2.f * episode.safety.blocked_time_s;

    if (!episodic_memory_.save_episode(episode))
        qWarning() << "Failed to save episode" << QString::fromStdString(episode.episode_id);

    if (label_episodeStatus != nullptr)
    {
        const QString qstatus = QString::fromStdString(status).toUpper();
        label_episodeStatus->setText("EP: " + qstatus + " " + QString::fromStdString(finished_episode_id).right(6));
        const bool ok = (status == "success");
        label_episodeStatus->setStyleSheet(ok
            ? "background-color: #C8E6C9; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;"
            : "background-color: #FFE0B2; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
    }

    active_episode_.reset();
    episode_accum_ = EpisodeAccum{};
}

void SpecificWorker::update_ui(const rc::RoomConceptAI::UpdateResult &res,
							   const rc::VelocityCommand &current_velocity,
							   int fps_val)
{
    const auto nav_status = Navigator_getStatus();
    QString nav_text = "NAV: UNKNOWN";
    QString nav_style = "background-color: #B0B0B0; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
    switch (nav_status.state)
    {
        case RoboCompNavigator::NavigationState::IDLE:
            nav_text = "NAV: IDLE";
            nav_style = "background-color: #B0B0B0; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::NAVIGATING:
            nav_text = "NAV: NAVIGATING";
            nav_style = "background-color: #90EE90; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::PAUSED:
            nav_text = "NAV: PAUSED";
            nav_style = "background-color: #FFF59D; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::REACHED:
            nav_text = "NAV: REACHED";
            nav_style = "background-color: #80CBC4; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::BLOCKED:
            nav_text = "NAV: BLOCKED";
            nav_style = "background-color: #FFB74D; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
        case RoboCompNavigator::NavigationState::ERROR:
            nav_text = "NAV: ERROR";
            nav_style = "background-color: #EF9A9A; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;";
            break;
    }
    static QString last_nav_text, last_nav_style;
    if (QLabel *nav_label = this->findChild<QLabel*>("label_navStatus"); nav_label != nullptr)
    {
        if (nav_text != last_nav_text)
        {
            nav_label->setText(nav_text);
            last_nav_text = nav_text;
        }
        if (nav_style != last_nav_style)
        {
            nav_label->setStyleSheet(nav_style);
            last_nav_style = nav_style;
        }
    }
    else
    {
        const bool webots_mode = params.USE_WEBOTS;
        const QString mode_prefix = webots_mode ? "WEBOTS" : "ROBOT";
        const QString fallback_text = mode_prefix + " | " + nav_text;
        if (label_mode->text() != fallback_text)
            label_mode->setText(fallback_text);
    }

    if (label_safetyGuard != nullptr)
    {
        const auto now = std::chrono::steady_clock::now();
        const bool guard_recent = (last_safety_guard_trigger_time_.time_since_epoch().count() > 0)
                               && (std::chrono::duration<float>(now - last_safety_guard_trigger_time_).count() <= SAFETY_GUARD_UI_HOLD_SEC);
        label_safetyGuard->setText(guard_recent ? "SG: ON" : "SG: OFF");
        label_safetyGuard->setStyleSheet(guard_recent
            ? "background-color: #FF8A80; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;"
            : "background-color: #CFD8DC; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
    }

	// Update QLCDNumber displays
	const float sigma_xy_cm = std::sqrt(res.covariance(0,0) + res.covariance(1,1)) * 100.0f; // m to cm
	const float sdf_median_cm = res.sdf_mse * 100.0f;  // Median SDF error in cm (already in meters)
	const float innovation_cm = res.innovation_norm * 100.0f;  // Innovation norm in cm

	lcdNumber_fps->display(fps_val);
	lcdNumber_loss->display(sdf_median_cm);
	lcdNumber_sigma->display(sigma_xy_cm);
	lcdNumber_velocity->display(std::abs(current_velocity.adv_y));
	lcdNumber_innov->display(innovation_cm);

	// ESS display: show ratio as percentage (ESS/K * 100)
	const float ess_ratio_pct = (last_ess_K_ > 0) ? (last_ess_ / static_cast<float>(last_ess_K_)) * 100.f : 0.f;
	lcdNumber_ess->display(static_cast<int>(ess_ratio_pct));

	// Only update CPU and color stylesheets every 10 frames to save overhead
	static int ui_slow_counter = 0;
	if (++ui_slow_counter >= 10)
	{
		ui_slow_counter = 0;
		const float cpu_usage = get_cpu_usage();
		lcdNumber_cpu->display(static_cast<int>(cpu_usage));

		// Helper lambda: only call setStyleSheet when the color key changes
		static QString last_sigma_color, last_innov_color, last_cpu_color, last_ess_color;

		auto set_style_if_changed = [](QLCDNumber *w, const QString &style, QString &last) {
			if (style != last) { w->setStyleSheet(style); last = style; }
		};

		// Color sigma
		QString sigma_color;
		if (sigma_xy_cm < 5.0f)       sigma_color = "background-color: #90EE90;";
		else if (sigma_xy_cm < 10.0f)  sigma_color = "background-color: #FFFF00;";
		else if (sigma_xy_cm < 20.0f)  sigma_color = "background-color: #FFA500;";
		else                            sigma_color = "background-color: #FF6B6B;";
		set_style_if_changed(lcdNumber_sigma, sigma_color, last_sigma_color);

		// Color innovation
		QString innov_color;
		if (innovation_cm < 5.0f)       innov_color = "background-color: #90EE90;";
		else if (innovation_cm < 15.0f)  innov_color = "background-color: #FFFF00;";
		else                              innov_color = "background-color: #FF6B6B;";
		set_style_if_changed(lcdNumber_innov, innov_color, last_innov_color);

		// Color ESS: green >50%, yellow 25-50%, orange 15-25%, red <15%
		QString ess_color;
		if (ess_ratio_pct > 50.f)       ess_color = "background-color: #90EE90;";  // healthy
		else if (ess_ratio_pct > 25.f)  ess_color = "background-color: #FFFF00;";  // moderate
		else if (ess_ratio_pct > 15.f)  ess_color = "background-color: #FFA500;";  // stressed
		else                             ess_color = "background-color: #FF6B6B;";  // collapsed
		set_style_if_changed(lcdNumber_ess, ess_color, last_ess_color);

		// Color CPU
		QString cpu_color;
		if (cpu_usage < 30.0f)       cpu_color = "background-color: #90EE90;";
		else if (cpu_usage < 60.0f)  cpu_color = "background-color: #FFFF00;";
		else                          cpu_color = "background-color: #FF6B6B;";
		set_style_if_changed(lcdNumber_cpu, cpu_color, last_cpu_color);
	}
}

void SpecificWorker::display_robot(const Eigen::Affine2f &robot_pose, const Eigen::Matrix3f &covariance)
{
	// ============ Update robot visualization in the viewer
	const float display_x = robot_pose.translation().x();
	const float display_y = robot_pose.translation().y();
	const float display_angle = std::atan2(robot_pose.linear()(1,0), robot_pose.linear()(0,0));
	viewer->robot_poly()->setPos(display_x, display_y);
	viewer->robot_poly()->setRotation(qRadiansToDegrees(display_angle));

	if (viewer_3d_)
		viewer_3d_->update_robot_pose(display_x, display_y, display_angle);

	// Keep view centered on the robot (if enabled)
        if (auto_center_ || !initial_center_done_)
        {
                viewer->centerOn(display_x, display_y);
                initial_center_done_ = true;
        }
	// ============ Update robot coordinates in UI
	lcdNumber_robotX->display(static_cast<double>(display_x));
	lcdNumber_robotY->display(static_cast<double>(display_y));
	lcdNumber_robotTheta->display(static_cast<double>(qRadiansToDegrees(display_angle)));

	// ============ Draw covariance ellipse (2D position uncertainty)
	// Extract 2x2 position covariance submatrix
	Eigen::Matrix2f pos_cov = covariance.block<2,2>(0,0);

	// Compute eigenvalues and eigenvectors for ellipse axes
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(pos_cov);
	Eigen::Vector2f eigenvalues = solver.eigenvalues();
	Eigen::Matrix2f eigenvectors = solver.eigenvectors();

	// Ellipse radii are sqrt(eigenvalues) * scale_factor (2.4477 for 95% confidence)
	// Use larger scale to make it more visible
	constexpr float confidence_scale = 3.0f;  // ~99% confidence, more visible
	const float radius_x = std::sqrt(std::max(0.001f, eigenvalues(0))) * confidence_scale;
	const float radius_y = std::sqrt(std::max(0.001f, eigenvalues(1))) * confidence_scale;

	// Ellipse rotation angle (from eigenvectors)
	const float ellipse_angle = std::atan2(eigenvectors(1,0), eigenvectors(0,0));

	// Create or update the ellipse item
	if (cov_ellipse_item_ == nullptr)
	{
		cov_ellipse_item_ = viewer->scene.addEllipse(
			-radius_x, -radius_y, 2*radius_x, 2*radius_y,
			QPen(QColor(255, 50, 50), 0.03),       // Bright red outline, thicker
			QBrush(QColor(255, 100, 100, 80))      // Semi-transparent red fill
		);
		cov_ellipse_item_->setZValue(100);  // Well above robot and lidar points
	}
	else
	{
		cov_ellipse_item_->setRect(-radius_x, -radius_y, 2*radius_x, 2*radius_y);
	}

	// Position and rotate the ellipse to match robot pose
	cov_ellipse_item_->setPos(display_x, display_y);
	cov_ellipse_item_->setRotation(qRadiansToDegrees(ellipse_angle));
}

void SpecificWorker::display_gt_error(const Eigen::Affine2f &estimated_pose,
                                      const std::optional<Eigen::Affine2f> &gt_pose_opt)
{
    // If GT pose is not available (webots not connected), blank the displays
    if (!gt_pose_opt.has_value())
    {
        lcdNumber_gt_xy_err->display(0.0);
        lcdNumber_gt_ang_err->display(0.0);
        lcdNumber_gt_xy_err->setStyleSheet("background-color: #888888;");  // Grey = no data
        lcdNumber_gt_ang_err->setStyleSheet("background-color: #888888;");
        return;
    }

    // Transform raw Webots pose to map frame using calibration offset
    const Eigen::Affine2f gt = gt_offset_ * gt_pose_opt.value();

    // Position error in cm
    const float dx = estimated_pose.translation().x() - gt.translation().x();
    const float dy = estimated_pose.translation().y() - gt.translation().y();
    const float xy_err_cm = std::sqrt(dx*dx + dy*dy) * 100.f;

    // Angular error in degrees (wrap to [-180, 180])
    const float est_ang = std::atan2(estimated_pose.linear()(1,0), estimated_pose.linear()(0,0));
    const float gt_ang  = std::atan2(gt.linear()(1,0), gt.linear()(0,0));
    float ang_err_deg = qRadiansToDegrees(est_ang - gt_ang);
    // Wrap
    while (ang_err_deg >  180.f) ang_err_deg -= 360.f;
    while (ang_err_deg < -180.f) ang_err_deg += 360.f;
    ang_err_deg = std::abs(ang_err_deg);

    lcdNumber_gt_xy_err->display(static_cast<double>(xy_err_cm));
    lcdNumber_gt_ang_err->display(static_cast<double>(ang_err_deg));

    // Color coding — only update stylesheet when color changes
    static QString last_xy_color, last_ang_color;
    auto set_if_changed = [](QLCDNumber *w, const QString &style, QString &last) {
        if (style != last) { w->setStyleSheet(style); last = style; }
    };

    // Position error: green <5cm, yellow <15cm, red >=15cm
    QString xy_color;
    if      (xy_err_cm < 5.f)  xy_color = "background-color: #90EE90;";
    else if (xy_err_cm < 15.f) xy_color = "background-color: #FFFF00;";
    else                        xy_color = "background-color: #FF6B6B;";
    set_if_changed(lcdNumber_gt_xy_err, xy_color, last_xy_color);

    // Angular error: green <3°, yellow <10°, red >=10°
    QString ang_color;
    if      (ang_err_deg < 3.f)  ang_color = "background-color: #90EE90;";
    else if (ang_err_deg < 10.f) ang_color = "background-color: #FFFF00;";
    else                          ang_color = "background-color: #FF6B6B;";
    set_if_changed(lcdNumber_gt_ang_err, ang_color, last_ang_color);
}

void SpecificWorker::calibrate_gt_offset(const Eigen::Affine2f &estimated_pose, const Eigen::Affine2f &webots_pose)
{
    // Compute offset T such that:  T * webots_pose ≈ estimated_pose
    // => T = estimated_pose * webots_pose.inverse()
    gt_offset_ = estimated_pose * webots_pose.inverse();
    gt_calibrated_ = true;

    const float tx = gt_offset_.translation().x();
    const float ty = gt_offset_.translation().y();
    const float ang = std::atan2(gt_offset_.linear()(1,0), gt_offset_.linear()(0,0));
    qInfo() << "GT calibrated: offset = (" << tx << "," << ty << ") rot =" << qRadiansToDegrees(ang) << "°";
}

void SpecificWorker::slot_calibrate_gt()
{
    if (!loc_initialized_.load())
    {
        qWarning() << "Cannot calibrate GT: room_ai not initialized";
        return;
    }

    // Read current GT pose from buffer
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const auto &[robot_pose_gt_, lidar_local_, lidar_low_unused_] = buffer_sync.read(timestamp);
    if (!robot_pose_gt_.has_value())
    {
        qWarning() << "Cannot calibrate GT: no Webots pose available";
        return;
    }

    // Get the current estimated pose (thread-safe)
    const auto state = get_loc_state();
    Eigen::Affine2f estimated;
    estimated.setIdentity();
    estimated.translation() = Eigen::Vector2f(state[2], state[3]);
    estimated.linear() = Eigen::Rotation2Df(state[4]).toRotationMatrix();

    calibrate_gt_offset(estimated, robot_pose_gt_.value());
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
            // Get robot GT pose from Webots (only for debug/stats, not used by algorithm)
            if (params.USE_WEBOTS)
            {
                const auto &[position, orientation] = webots2robocomp_proxy->getObjectPose("shadow");
                Eigen::Affine2f eig_pose;
                eig_pose.translation() = Eigen::Vector2f(-position.y/1000.f, position.x/1000.f);
                eig_pose.linear() = Eigen::Rotation2Df(yawFromQuaternion(orientation)).toRotationMatrix();
                buffer_sync.put<0>(std::move(eig_pose), timestamp);
            }

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

            // BPEARL
            std::future<RoboCompLidar3D::TData> future_low;
            try
            {
                future_low = lidar3d1_proxy->getLidarDataWithThreshold2dAsync(
                    params.LIDAR_NAME_LOW,
                    params.MAX_LIDAR_LOW_RANGE * 1000.f,
                    params.LIDAR_LOW_DECIMATION_FACTOR_LOW);
            }
            catch (const Ice::Exception &e)
            { qWarning() << "[read_lidar] BPEARL async launch failed:" << e.what(); }

            // ---- Wait and process HELIOS ----
            const auto data_high = future_high.get();
            std::vector<Eigen::Vector3f> points_high, points_low_high;
            points_low_high.reserve(data_high.points.size());
            points_high.reserve(data_high.points.size());
            for (const auto &p : data_high.points)
            {
                const float pmx = p.x / 1000.f;
                const float pmy = p.y / 1000.f;
                const float pmz = p.z / 1000.f;
                if (pmx*pmx + pmy*pmy > body_offset_sq && pmz < params.LIDAR_HIGH_MAX_HEIGHT)
                {
                    points_low_high.emplace_back(pmx, pmy, pmz);
                    if (pmz > params.LIDAR_HIGH_MIN_HEIGHT)
                        points_high.emplace_back(pmx, pmy, pmz);
                }
            }
            buffer_sync.put<1>(std::make_pair(std::move(points_high), data_high.timestamp), timestamp);

            // ---- Wait and process BPEARL ----
            const auto data_low = future_low.get();
            points_low_high.reserve(points_low_high.size() + data_low.points.size());
            for (const auto &p : data_low.points)
            {
                const float pmx = p.x / 1000.f;
                const float pmy = p.y / 1000.f;
                const float pmz = p.z / 1000.f;
                if (pmx*pmx + pmy*pmy > body_offset_sq)
                    points_low_high.emplace_back(pmx, pmy, pmz);
            }
            buffer_sync.put<2>(std::make_pair(std::move(points_low_high), data_low.timestamp), timestamp);

            // Adjust period with hysteresis
            if (wait_period > std::chrono::milliseconds((long) data_high.period + 2)) --wait_period;
            else if (wait_period < std::chrono::milliseconds((long) data_high.period - 2)) ++wait_period;
            std::this_thread::sleep_for(wait_period);
        }
        catch (const Ice::Exception &e)
        { qWarning() << "Error reading from Lidar3D or robot pose:" << e.what(); }
    }
} // Thread to read the lidar

float SpecificWorker::estimate_orientation_from_points(const std::vector<Eigen::Vector3f> &pts) const
{
	if(pts.size() < 10)
		return 0.f;

	// PCA on 2D points to get dominant direction
	Eigen::Vector2f mean = Eigen::Vector2f::Zero();
	for(const auto &p : pts) mean += p.head(2);
	mean /= static_cast<float>(pts.size());

	Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
	for(const auto &p : pts)
	{
		const Eigen::Vector2f d = p.head(2) - mean;
		cov += d * d.transpose();
	}
	cov /= static_cast<float>(pts.size());

	const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
	const Eigen::Vector2f v = es.eigenvectors().col(1);  // largest eigenvalue
	return std::atan2(v.y(), v.x());
}

void SpecificWorker::draw_estimated_room(const Eigen::Matrix<float, 5, 1> &state)
{
    static QGraphicsRectItem* estimated_room_item = nullptr;

    // If using polygon mode, remove rectangle and return
    if (!room_polygon_.empty())
    {
        if (estimated_room_item != nullptr)
        {
            viewer->scene.removeItem(estimated_room_item);
            delete estimated_room_item;
            estimated_room_item = nullptr;
        }
        return;
    }

    // state = [width, length, x, y, phi]
    // Room is always at origin in room frame

    const float width = state[0];
    const float length = state[1];

    QRectF room_rect(-width/2, -length/2, width, length);

    if (estimated_room_item == nullptr)
    {
        estimated_room_item = viewer->scene.addRect(room_rect, QPen(Qt::magenta, 0.05), QBrush(Qt::NoBrush));
        estimated_room_item->setZValue(2);
    }
    else
    {
        estimated_room_item->setRect(room_rect);
    }
}

auto SpecificWorker::draw_lidar_points(const std::vector<Eigen::Vector3f> &points_high,
                                       const std::vector<Eigen::Vector3f> &points_low,
                                       const Eigen::Affine2f &robot_pose) -> void
{
    // ---- Helper lambda to draw a point set with a given color pool ----
    auto draw_layer = [&](const std::vector<Eigen::Vector3f> &points,
                          std::vector<QGraphicsEllipseItem*> &pool,
                          const QColor &color, int max_points)
    {
        static const qreal radius_px = 1.5;
        const QRectF ellipse_rect(-radius_px, -radius_px, 2*radius_px, 2*radius_px);
        QPen pen(color); pen.setWidthF(0.0); pen.setCosmetic(true);
        QBrush brush(color);

        const int stride = std::max(1, static_cast<int>(points.size() / max_points));
        const size_t num_draw = (points.size() + stride - 1) / stride;

        // Shrink pool if needed
        while (pool.size() > num_draw)
        {
            auto *p = pool.back();
            viewer->scene.removeItem(p);
            delete p;
            pool.pop_back();
        }

        size_t idx = 0;
        for (size_t i = 0; i < points.size() && idx < num_draw; i += stride, ++idx)
        {
            const Eigen::Vector2f pr = points[i].head<2>();
            const Eigen::Vector2f pw = robot_pose.linear() * pr + robot_pose.translation();

            if (idx < pool.size())
            {
                pool[idx]->setPos(pw.x(), pw.y());
            }
            else
            {
                auto *item = viewer->scene.addEllipse(ellipse_rect, pen, brush);
                item->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
                item->setPos(pw.x(), pw.y());
                item->setZValue(5);
                pool.push_back(item);
            }
        }
    };

    // HELIOS (high) in green
    static std::vector<QGraphicsEllipseItem*> pool_high;
    draw_layer(points_high, pool_high, QColor("Green"), params.MAX_LIDAR_DRAW_POINTS);

    // BPEARL (low) in cyan — fewer points to keep it lightweight
    static std::vector<QGraphicsEllipseItem*> pool_low;
    draw_layer(points_low, pool_low, QColor("Cyan"), params.MAX_LIDAR_DRAW_POINTS / 2);
}
///////////////////////////////////////////////////////////////////////////////////////////////
// Utility: Convert quaternion to yaw angle
///////////////////////////////////////////////////////////////////////////////////////////////
float SpecificWorker::yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat)
{
	double w = quat.w;
	double x = quat.x;
	double y = quat.y;
	double z = quat.z;
	const auto norm = std::sqrt(w*w + x*x + y*y + z*z);
	w /= norm; x /= norm; y /= norm; z /= norm;
	return static_cast<float>(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
}


////////////////////////////////////////////////////////////////////////////////////////////////
/// Navigation target (Shift+Right click)
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::slot_new_target(QPointF pos)
{
    if (!loc_initialized_.load() || !path_planner_.is_ready())
    {
        qWarning() << "Cannot set target: room not initialized or planner not ready";
        return;
    }

    const Eigen::Vector2f target(pos.x(), pos.y());

    // Get current robot pose (thread-safe)
    const auto state = get_loc_state();
    const Eigen::Vector2f robot_pos(state[2], state[3]);

    qInfo() << "[PathPlanner] Robot at (" << robot_pos.x() << "," << robot_pos.y()
            << ") Target at (" << target.x() << "," << target.y() << ")"
            << " Navigable polygon:" << path_planner_.get_navigable_polygon().size() << "vertices"
            << " Inside(robot):" << path_planner_.is_inside(robot_pos)
            << " Inside(target):" << path_planner_.is_inside(target);

    // Plan path
    const auto path = path_planner_.plan(robot_pos, target);
    if (path.empty())
    {
        qWarning() << "No path found to target (" << pos.x() << "," << pos.y() << ")";
        clear_path();
        return;
    }

    current_path_ = path;
    draw_path(path);

    // Compute total path length
    float total = 0.f;
    for (size_t i = 1; i < path.size(); ++i)
        total += (path[i] - path[i - 1]).norm();

    // Activate the trajectory controller to follow the path (applies elastic relaxation)
    trajectory_controller_.set_path(path);

    // Redraw with the relaxed path so the viewer shows what the controller actually follows
    draw_path(trajectory_controller_.get_path());

    start_episode("goto_point", Eigen::Vector2f(pos.x(), pos.y()));

    qInfo() << "Path planned:" << path.size() << "waypoints," << total << "m to ("
            << pos.x() << "," << pos.y() << ")";
}

void SpecificWorker::send_velocity_command(float adv, float side, float rot)
{
    try
    {
        omnirobot_proxy->setSpeedBase(side*1000.f, adv*1000.f, rot);
    }
    catch (const Ice::Exception &e)
    {
        static int err_count = 0;
        if (++err_count % 100 == 1)
            qWarning() << "OmniRobot proxy error:" << e.what();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Temporary obstacle avoidance: cluster → polygon → replan
// ═══════════════════════════════════════════════════════════════════════════

std::vector<Eigen::Vector2f> SpecificWorker::cluster_lidar_to_polygon(
    const std::vector<Eigen::Vector3f>& lidar_points,
    const Eigen::Vector2f& blockage_center_room,
    float search_radius,
    const Eigen::Affine2f& robot_pose,
    float& height_out) const
{
    height_out = 0.f;

    // Transform blockage center to robot frame
    const Eigen::Rotation2Df rot(robot_pose.rotation());
    const Eigen::Vector2f t = robot_pose.translation();
    const Eigen::Vector2f center_robot = rot.inverse() * (blockage_center_room - t);

    // Collect 2D lidar points within search_radius of blockage center (robot frame)
    const float r2 = search_radius * search_radius;
    std::vector<Eigen::Vector2f> cluster;
    float z_min =  std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    for (const auto& p : lidar_points)
    {
        const Eigen::Vector2f p2d(p.x(), p.y());
        if ((p2d - center_robot).squaredNorm() < r2)
        {
            cluster.push_back(p2d);
            z_min = std::min(z_min, p.z());
            z_max = std::max(z_max, p.z());
        }
    }

    if (cluster.size() < 3)
        return {};  // too few points to form a polygon

    // Estimate height from Z range of clustered points
    if (z_max > z_min)
        height_out = z_max - z_min;

    // Compute centroid
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (const auto& p : cluster) centroid += p;
    centroid /= static_cast<float>(cluster.size());

    // PCA to find principal axes
    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& p : cluster)
    {
        const Eigen::Vector2f d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(cluster.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
    // Eigenvectors sorted ascending — col(1) is major axis
    const Eigen::Vector2f axis_major = eig.eigenvectors().col(1);
    const Eigen::Vector2f axis_minor = eig.eigenvectors().col(0);

    // Project points onto axes, find extents
    float min_major = std::numeric_limits<float>::max(), max_major = -std::numeric_limits<float>::max();
    float min_minor = std::numeric_limits<float>::max(), max_minor = -std::numeric_limits<float>::max();
    for (const auto& p : cluster)
    {
        const Eigen::Vector2f d = p - centroid;
        const float proj_maj = d.dot(axis_major);
        const float proj_min = d.dot(axis_minor);
        min_major = std::min(min_major, proj_maj);
        max_major = std::max(max_major, proj_maj);
        min_minor = std::min(min_minor, proj_min);
        max_minor = std::max(max_minor, proj_min);
    }

    // Add small margin (planner already does Minkowski expansion by robot_radius)
    const float margin = TEMP_OBSTACLE_MARGIN;
    min_major -= margin;
    max_major += margin;
    min_minor -= margin;
    max_minor += margin;

    // Build OBB corners in robot frame
    std::vector<Eigen::Vector2f> corners_robot = {
        centroid + min_major * axis_major + min_minor * axis_minor,
        centroid + max_major * axis_major + min_minor * axis_minor,
        centroid + max_major * axis_major + max_minor * axis_minor,
        centroid + min_major * axis_major + max_minor * axis_minor
    };

    // Transform to room frame
    std::vector<Eigen::Vector2f> corners_room;
    corners_room.reserve(4);
    for (const auto& c : corners_robot)
        corners_room.push_back(rot * c + t);

    return corners_room;
}

bool SpecificWorker::replan_around_obstacle(const std::vector<Eigen::Vector2f>& obstacle_polygon,
                                             float obstacle_height,
                                             const Eigen::Vector2f& center,
                                             const Eigen::Affine2f& robot_pose)
{
    if (obstacle_polygon.size() < 3 || current_path_.empty())
        return false;

    // Check if this obstacle is near an existing temp obstacle — merge if so
    bool merged = false;
    for (auto& existing : temp_obstacles_)
    {
        if ((existing.center - center).norm() < TEMP_OBSTACLE_MERGE_DIST)
        {
            // Replace with new, larger polygon
            existing.vertices = obstacle_polygon;
            existing.center = center;
            existing.created = std::chrono::steady_clock::now();
            existing.replan_count++;
            if (obstacle_height > 0.05f) existing.height = obstacle_height;
            merged = true;
            break;
        }
    }

    if (!merged)
    {
        TempObstacle obs;
        obs.vertices = obstacle_polygon;
        obs.center = center;
        obs.created = std::chrono::steady_clock::now();
        obs.replan_count = 1;
        obs.height = obstacle_height;
        temp_obstacles_.push_back(std::move(obs));
    }

    // Rebuild obstacle list: static furniture + all temp obstacles
    std::vector<std::vector<Eigen::Vector2f>> all_obstacles;
    for (const auto& fp : furniture_polygons_)
        all_obstacles.push_back(fp.vertices);
    for (const auto& to : temp_obstacles_)
        all_obstacles.push_back(to.vertices);

    path_planner_.set_obstacles(all_obstacles);
    trajectory_controller_.set_static_obstacles(all_obstacles);

    // Replan from current robot position to original target
    const Eigen::Vector2f robot_pos = robot_pose.translation();
    const Eigen::Vector2f target = current_path_.back();

    const auto new_path = path_planner_.plan(robot_pos, target);
    if (new_path.empty())
    {
        qWarning() << "[ObstacleAvoid] Replan failed — no path found around obstacle";
        return false;
    }

    current_path_ = new_path;
    trajectory_controller_.set_path(new_path);

    // Redraw with relaxed path
    draw_path(trajectory_controller_.get_path());
    draw_temp_obstacles();

    qInfo() << "[ObstacleAvoid] Replanned:" << new_path.size() << "waypoints around obstacle at ("
            << center.x() << "," << center.y() << ") temp_obstacles:" << temp_obstacles_.size();
    return true;
}

void SpecificWorker::cleanup_temp_obstacles()
{
    const auto now = std::chrono::steady_clock::now();
    bool removed = false;

    temp_obstacles_.erase(
        std::remove_if(temp_obstacles_.begin(), temp_obstacles_.end(),
            [&](const TempObstacle& o)
            {
                const float age = std::chrono::duration<float>(now - o.created).count();
                if (age > TEMP_OBSTACLE_TIMEOUT_SEC)
                {
                    removed = true;
                    qInfo() << "[ObstacleAvoid] Removing expired temp obstacle at ("
                            << o.center.x() << "," << o.center.y() << ") age:" << age << "s";
                    return true;
                }
                return false;
            }),
        temp_obstacles_.end());

    if (removed)
    {
        // Rebuild obstacle list with only furniture + remaining temp obstacles
        std::vector<std::vector<Eigen::Vector2f>> all_obstacles;
        for (const auto& fp : furniture_polygons_)
            all_obstacles.push_back(fp.vertices);
        for (const auto& to : temp_obstacles_)
            all_obstacles.push_back(to.vertices);

        path_planner_.set_obstacles(all_obstacles);
        trajectory_controller_.set_static_obstacles(all_obstacles);
        draw_temp_obstacles();
    }
}

void SpecificWorker::draw_temp_obstacles()
{
    // Remove old items
    for (auto* item : temp_obstacle_draw_items_)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    temp_obstacle_draw_items_.clear();

    // Draw current temp obstacles
    for (const auto& obs : temp_obstacles_)
    {
        QPolygonF poly;
        for (const auto& v : obs.vertices)
            poly << QPointF(v.x(), v.y());
        poly << QPointF(obs.vertices.front().x(), obs.vertices.front().y());

        auto* item = viewer->scene.addPolygon(poly,
            QPen(QColor(255, 100, 0), 0.06),          // orange outline (scene units = meters)
            QBrush(QColor(255, 100, 0, 60)));          // semi-transparent fill
        item->setZValue(5);
        temp_obstacle_draw_items_.push_back(item);
    }

    if (viewer_3d_)
    {
        std::vector<rc::Viewer3D::ObstacleItem> obs_items;
        obs_items.reserve(temp_obstacles_.size());
        for (const auto& obs : temp_obstacles_)
            obs_items.push_back({obs.vertices, obs.height});
        viewer_3d_->update_obstacles(obs_items);
    }
}

void SpecificWorker::draw_trajectory_debug(const rc::TrajectoryController::ControlOutput &ctrl,
                                            const Eigen::Affine2f &robot_pose)
{
    const float rx = robot_pose.translation().x();
    const float ry = robot_pose.translation().y();

    // ---- 1. Draw all sampled trajectories ----
    const int num_traj = static_cast<int>(ctrl.trajectories_room.size());

    // Grow item pool if needed
    while (static_cast<int>(traj_draw_items_.size()) < num_traj)
        traj_draw_items_.emplace_back();

    for (int t = 0; t < static_cast<int>(traj_draw_items_.size()); ++t)
    {
        if (t >= num_traj)
        {
            for (auto* seg : traj_draw_items_[t])
                seg->setVisible(false);
            continue;
        }

        const auto& traj = ctrl.trajectories_room[t];
        const bool is_best = (t == ctrl.best_trajectory_idx);
        const QColor color = is_best ? QColor(0, 220, 0) : QColor(150, 150, 150, 100);
        const float width = is_best ? 0.06f : 0.02f;
        const int z = is_best ? 32 : 28;

        auto& segments = traj_draw_items_[t];

        while (segments.size() < traj.size())
        {
            auto* item = viewer->scene.addLine(0, 0, 0, 0, QPen(color, width));
            item->setZValue(z);
            segments.push_back(item);
        }

        for (size_t s = 0; s + 1 < traj.size(); ++s)
        {
            segments[s]->setLine(traj[s].x(), traj[s].y(), traj[s+1].x(), traj[s+1].y());
            segments[s]->setPen(QPen(color, width));
            segments[s]->setZValue(z);
            segments[s]->setVisible(true);
        }
        for (size_t s = (traj.empty() ? 0 : traj.size() - 1); s < segments.size(); ++s)
            segments[s]->setVisible(false);
    }

    // ---- 2. Carrot marker (bright orange filled circle) ----
    {
        constexpr float cr = 0.15f;
        if (!traj_carrot_marker_)
        {
            traj_carrot_marker_ = viewer->scene.addEllipse(
                -cr, -cr, 2*cr, 2*cr,
                QPen(QColor(255, 140, 0), 0.03f),
                QBrush(QColor(255, 140, 0, 180)));
            traj_carrot_marker_->setZValue(33);
        }
        traj_carrot_marker_->setPos(ctrl.carrot_room.x(), ctrl.carrot_room.y());
        traj_carrot_marker_->setVisible(true);
    }

    // ---- 3. Dashed line from robot to carrot (orange) ----
    {
        if (!traj_robot_to_carrot_)
        {
            traj_robot_to_carrot_ = viewer->scene.addLine(
                rx, ry, ctrl.carrot_room.x(), ctrl.carrot_room.y(),
                QPen(QColor(255, 165, 0, 200), 0.02f, Qt::DashLine));
            traj_robot_to_carrot_->setZValue(29);
        }
        else
        {
            traj_robot_to_carrot_->setLine(rx, ry, ctrl.carrot_room.x(), ctrl.carrot_room.y());
            traj_robot_to_carrot_->setVisible(true);
        }
    }
}

void SpecificWorker::draw_path(const std::vector<Eigen::Vector2f>& path)
{
    // Remove previous path drawing
    clear_path(false, false);

    if (path.size() < 2) return;

    // Draw original polygon vertices as green circles (debug)
    const auto& orig_poly = path_planner_.get_original_polygon();
    for (const auto& v : orig_poly)
    {
        constexpr float r = 0.1f;
        auto* dot = viewer->scene.addEllipse(-r, -r, 2*r, 2*r,
            QPen(QColor(0, 200, 0), 0.02), QBrush(QColor(0, 200, 0, 80)));
        dot->setPos(v.x(), v.y());
        dot->setZValue(17);
        path_draw_items_.push_back(dot);
    }

    // Draw inner polygon boundary as orange dashed line (the barrier paths cannot cross)
    if (navigable_poly_item_)
    {
        viewer->scene.removeItem(navigable_poly_item_);
        delete navigable_poly_item_;
        navigable_poly_item_ = nullptr;
    }
    const auto& inner_poly = path_planner_.get_inner_polygon();
    if (inner_poly.size() >= 3)
    {
        QPolygonF qpoly;
        for (const auto& v : inner_poly)
            qpoly << QPointF(v.x(), v.y());
        qpoly << QPointF(inner_poly.front().x(), inner_poly.front().y());
        QPen inner_pen(QColor(255, 140, 0, 200), 0.03, Qt::DashLine);  // orange dashed
        navigable_poly_item_ = viewer->scene.addPolygon(qpoly, inner_pen, Qt::NoBrush);
        navigable_poly_item_->setZValue(19);
    }

    // Draw expanded obstacle boundaries as orange dashed polygons
    for (auto* item : obstacle_expanded_items_)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    obstacle_expanded_items_.clear();
    const auto& exp_obstacles = path_planner_.get_expanded_obstacles();
    for (const auto& obs : exp_obstacles)
    {
        if (obs.size() < 3) continue;
        QPolygonF qpoly_obs;
        for (const auto& v : obs)
            qpoly_obs << QPointF(v.x(), v.y());
        qpoly_obs << QPointF(obs.front().x(), obs.front().y());
        QPen obs_pen(QColor(255, 140, 0, 200), 0.03, Qt::DashLine);  // orange dashed, same as inner
        auto* obs_item = viewer->scene.addPolygon(qpoly_obs, obs_pen, Qt::NoBrush);
        obs_item->setZValue(19);
        obstacle_expanded_items_.push_back(obs_item);
    }

    // Draw navigation nodes as yellow dots
    const auto& nav_poly = path_planner_.get_navigable_polygon();
    for (const auto& v : nav_poly)
    {
        constexpr float r = 0.08f;
        auto* dot = viewer->scene.addEllipse(-r, -r, 2*r, 2*r,
            QPen(QColor(255, 255, 0, 200), 0.01), QBrush(QColor(255, 255, 0, 120)));
        dot->setPos(v.x(), v.y());
        dot->setZValue(18);
        path_draw_items_.push_back(dot);
    }

    // Draw path segments as light green lines
    const QPen path_pen(QColor(100, 255, 100), 0.04);  // Light green
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        auto* line = viewer->scene.addLine(
            path[i].x(), path[i].y(),
            path[i + 1].x(), path[i + 1].y(),
            path_pen);
        line->setZValue(20);
        path_draw_items_.push_back(line);
    }

    // Draw waypoint dots
    const QPen wp_pen(Qt::NoPen);
    const QBrush wp_brush(QColor(0, 220, 220));
    for (size_t i = 1; i + 1 < path.size(); ++i)  // skip start and goal
    {
        constexpr float r = 0.06f;
        auto* dot = viewer->scene.addEllipse(-r, -r, 2*r, 2*r, wp_pen, wp_brush);
        dot->setPos(path[i].x(), path[i].y());
        dot->setZValue(21);
        path_draw_items_.push_back(dot);
    }

    // Draw target marker (larger, red circle)
    const auto& goal = path.back();
    if (target_marker_ == nullptr)
    {
        constexpr float tr = 0.12f;
        target_marker_ = viewer->scene.addEllipse(-tr, -tr, 2*tr, 2*tr,
            QPen(QColor(255, 50, 50), 0.03), QBrush(QColor(255, 50, 50, 120)));
        target_marker_->setZValue(22);
    }
    target_marker_->setPos(goal.x(), goal.y());
    target_marker_->setVisible(true);

    if (viewer_3d_)
        viewer_3d_->update_path(path);
}

void SpecificWorker::draw_path_threadsafe(const std::vector<Eigen::Vector2f>& path)
{
    if (QThread::currentThread() == this->thread())
    {
        draw_path(path);
    }
    else
    {
        QMetaObject::invokeMethod(this, [this, path]() { draw_path(path); }, Qt::QueuedConnection);
    }
}

void SpecificWorker::clear_path(bool stop_controller, bool clear_stored_path)
{
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
void SpecificWorker::slot_capture_room_toggled(bool checked)
{
    capturing_room_polygon = checked;

    if (checked)
    {
        // Start capturing - backup existing polygon (don't remove it yet)
        room_polygon_backup_ = room_polygon_;
        room_polygon_.clear();

        // Clear previous vertex markers (these are only for capture mode)
        for (auto* item : polygon_vertex_items)
        {
            viewer->scene.removeItem(item);
            delete item;
        }
        polygon_vertex_items.clear();

        // Keep the existing polygon visible but store reference for later removal
        // Move current polygon_item to backup (will be removed when new capture succeeds)
        polygon_item_backup_ = polygon_item;
        polygon_item = nullptr;

        pushButton_captureRoom->setText("Ctrl+Click vertices...");
        qInfo() << "Room capture started. Use Ctrl+Left click on the scene to add vertices. Ctrl+Click near the first point to close.";
        qInfo() << "Existing polygon preserved until new one is completed.";
    }
    else
    {
        pushButton_captureRoom->setText("Capture Room");

        if (room_polygon_.size() >= 3)
        {
            qInfo() << "Room polygon captured with" << room_polygon_.size() << "vertices";

            // Keep old polygon visible until user saves the new one

            // Clear vertex markers (yellow circles)
            for (auto* item : polygon_vertex_items)
            {
                viewer->scene.removeItem(item);
                delete item;
            }
            polygon_vertex_items.clear();

            // Vertices are already in room frame (where user clicked)
            // Pass them to room_ai via thread-safe command queue
            push_loc_command(LocCmdSetPolygon{room_polygon_});
        path_planner_.set_polygon(room_polygon_);

            // Draw final polygon (fixed in room frame)
            draw_room_polygon();
        }
        else
        {
            qWarning() << "Need at least 3 vertices for a valid polygon. Restoring previous polygon.";

            // Restore the backup polygon
            room_polygon_ = room_polygon_backup_;
            room_polygon_backup_.clear();

            // Remove any partial new polygon drawing
            if (polygon_item)
            {
                viewer->scene.removeItem(polygon_item);
                delete polygon_item;
                polygon_item = nullptr;
            }

            // Restore backup polygon graphic as the active one
            polygon_item = polygon_item_backup_;
            polygon_item_backup_ = nullptr;

            // Clear vertex markers from failed capture
            for (auto* item : polygon_vertex_items)
            {
                viewer->scene.removeItem(item);
                delete item;
            }
            polygon_vertex_items.clear();
        }
    }
}

void SpecificWorker::on_scene_clicked(QPointF pos)
{
    // This function is now only used for polygon capture mode
    // Robot movement is handled by slot_robot_dragging and slot_robot_rotate
    if (!capturing_room_polygon)
        return;

    // Capture mode: add polygon vertices
    // The scene shows points in room frame (transformed by robot_pose)
    // We need to store vertices in room frame (where they appear on screen)
    const Eigen::Vector2f click_pos(pos.x(), pos.y());

    // Check if clicking near the first point to close the polygon
    constexpr float close_threshold = 0.3f;  // 30cm
    if (room_polygon_.size() >= 3)
    {
        const float dist_to_first = (click_pos - room_polygon_.front()).norm();
        if (dist_to_first < close_threshold)
        {
            // Close the polygon
            pushButton_captureRoom->setChecked(false);  // This triggers slot_capture_room_toggled(false)
            return;
        }
    }

    // Add new vertex in room frame coordinates
    room_polygon_.push_back(click_pos);

    // Draw vertex marker
    const float radius = 0.15f;
    auto* vertex_item = viewer->scene.addEllipse(
        -radius, -radius, 2*radius, 2*radius,
        QPen(Qt::yellow, 0.05), QBrush(Qt::yellow));
    vertex_item->setPos(pos.x(), pos.y());
    vertex_item->setZValue(10);
    polygon_vertex_items.push_back(vertex_item);

    // Draw temporary polygon outline
    draw_room_polygon();

    qInfo() << "Added vertex" << room_polygon_.size() << "at (" << pos.x() << "," << pos.y() << ")";
}

void SpecificWorker::draw_room_polygon()
{
    if (room_polygon_.size() < 2)
        return;

    // Remove old main polygon item
    if (polygon_item)
    {
        viewer->scene.removeItem(polygon_item);
        delete polygon_item;
        polygon_item = nullptr;
    }

    // --- Draw main room_polygon_ ---
    QPolygonF poly;
    for (const auto& v : room_polygon_)
        poly << QPointF(v.x(), v.y());

    if (!capturing_room_polygon && room_polygon_.size() >= 3)
        poly << QPointF(room_polygon_.front().x(), room_polygon_.front().y());

    QPen pen(capturing_room_polygon ? Qt::yellow : Qt::magenta, capturing_room_polygon ? 0.08 : 0.15);
    polygon_item = viewer->scene.addPolygon(poly, pen, QBrush(Qt::NoBrush));
    polygon_item->setZValue(8);

    // Sync 3D walls whenever the room polygon is finalised
    if (viewer_3d_ && !capturing_room_polygon && room_polygon_.size() >= 3)
        viewer_3d_->rebuild_walls(room_polygon_);
}

void SpecificWorker::draw_furniture()
{
    // Remove old furniture items
    for (auto* item : furniture_draw_items_)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    furniture_draw_items_.clear();

    const QPen furniture_pen(QColor(50, 100, 255), 0.06);          // Blue outline
    const QBrush furniture_brush(QColor(50, 100, 255, 40));        // Semi-transparent blue fill

    for (const auto& fp : furniture_polygons_)
    {
        if (fp.vertices.size() < 3) continue;

        QPolygonF qpoly;
        for (const auto& v : fp.vertices)
            qpoly << QPointF(v.x(), v.y());
        qpoly << QPointF(fp.vertices.front().x(), fp.vertices.front().y());

        auto* item = viewer->scene.addPolygon(qpoly, furniture_pen, furniture_brush);
        item->setZValue(7);  // below room polygon (8)
        furniture_draw_items_.push_back(item);
    }

    if (!furniture_polygons_.empty())
        qInfo() << "[draw_furniture] Drew" << furniture_polygons_.size() << "furniture polygons";

    // Refresh scene tree panel
    if (scene_tree_)
    {
        std::vector<SceneTreePanel::FurnitureEntry> entries;
        entries.reserve(furniture_polygons_.size());
        for (const auto& fp : furniture_polygons_)
        {
            SceneTreePanel::FurnitureEntry e;
            e.id       = fp.id;
            e.label    = fp.label;
            e.vertices = fp.vertices;
            entries.push_back(std::move(e));
        }
        scene_tree_->refresh(room_polygon_, entries);
    }

    if (viewer_3d_)
    {
        std::vector<rc::Viewer3D::FurnitureItem> items;
        items.reserve(furniture_polygons_.size());
        for (const auto& fp : furniture_polygons_)
        {
            if (fp.vertices.empty()) continue;
            Eigen::Vector2f cen = Eigen::Vector2f::Zero();
            for (const auto& v : fp.vertices)
            {
                cen += v;
            }
            cen /= static_cast<float>(fp.vertices.size());

            Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
            for (const auto& v : fp.vertices)
            {
                const Eigen::Vector2f d = v - cen;
                cov += d * d.transpose();
            }
            cov /= static_cast<float>(std::max<std::size_t>(1, fp.vertices.size()));
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(cov);
            Eigen::Vector2f axis_x(1.f, 0.f);
            if (eig.info() == Eigen::Success)
                axis_x = eig.eigenvectors().col(1).normalized();
            Eigen::Vector2f axis_z(-axis_x.y(), axis_x.x());

            float min_u = std::numeric_limits<float>::max();
            float max_u = -std::numeric_limits<float>::max();
            float min_v = std::numeric_limits<float>::max();
            float max_v = -std::numeric_limits<float>::max();
            for (const auto& p : fp.vertices)
            {
                const Eigen::Vector2f d = p - cen;
                const float u = d.dot(axis_x);
                const float v = d.dot(axis_z);
                min_u = std::min(min_u, u); max_u = std::max(max_u, u);
                min_v = std::min(min_v, v); max_v = std::max(max_v, v);
            }

            const float c_u = 0.5f * (min_u + max_u);
            const float c_v = 0.5f * (min_v + max_v);
            const Eigen::Vector2f obb_center = cen + axis_x * c_u + axis_z * c_v;

            rc::Viewer3D::FurnitureItem fi;
            fi.label    = fp.label;
            fi.centroid = obb_center;
            fi.size     = Eigen::Vector2f(std::max(0.08f, max_u - min_u), std::max(0.08f, max_v - min_v));
            fi.yaw_rad  = std::atan2(axis_x.y(), axis_x.x());
            items.push_back(fi);
        }
        viewer_3d_->update_furniture(items);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////
/// Layout Save/Load
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::slot_save_layout()
{
    if (room_polygon_.empty())
    {
        qWarning() << "No polygon to save - capture a room first";
        return;
    }

    // Use native dialog with explicit options to avoid freezing
    QString filename = QFileDialog::getSaveFileName(this,
        "Save Room Layout",
        "./room_layout",
        "SVG Files (*.svg)",
        nullptr,
        QFileDialog::DontUseNativeDialog);

    if (filename.isEmpty())
        return;

    if (!filename.endsWith(".svg", Qt::CaseInsensitive))
        filename += ".svg";

    save_layout_to_svg(filename.toStdString());

    // Remove the old polygon from the UI now that the new one is saved
    if (polygon_item_backup_)
    {
        viewer->scene.removeItem(polygon_item_backup_);
        delete polygon_item_backup_;
        polygon_item_backup_ = nullptr;
    }
    room_polygon_backup_.clear();
}

void SpecificWorker::slot_load_layout()
{
    QString filename = QFileDialog::getOpenFileName(this,
        "Load Room Layout",
        "./",
        "SVG Files (*.svg)",
        nullptr,
        QFileDialog::DontUseNativeDialog);

    if (filename.isEmpty())
        return;

    load_layout_from_file(filename.toStdString());
}

void SpecificWorker::slot_flip_x()
{
    if (room_polygon_.empty())
    {
        qWarning() << "Cannot flip: no room polygon defined";
        return;
    }

    // Flip all polygon vertices on X axis
    for (auto& vertex : room_polygon_)
    {
        vertex.x() = -vertex.x();
    }

    // Toggle flip state
    flip_x_applied_ = !flip_x_applied_;

    // Update the room model with flipped polygon (thread-safe)
    push_loc_command(LocCmdSetPolygon{room_polygon_});
    path_planner_.set_polygon(room_polygon_);

    // Redraw the polygon
    draw_room_polygon();

    qInfo() << "Room polygon flipped on X axis (flip_x=" << flip_x_applied_ << ")";
}

void SpecificWorker::slot_flip_y()
{
    if (room_polygon_.empty())
    {
        qWarning() << "Cannot flip: no room polygon defined";
        return;
    }

    // Flip all polygon vertices on Y axis
    for (auto& vertex : room_polygon_)
    {
        vertex.y() = -vertex.y();
    }

    // Toggle flip state
    flip_y_applied_ = !flip_y_applied_;

    // Update the room model with flipped polygon (thread-safe)
    push_loc_command(LocCmdSetPolygon{room_polygon_});
    path_planner_.set_polygon(room_polygon_);

    // Redraw the polygon
    draw_room_polygon();

    qInfo() << "Room polygon flipped on Y axis (flip_y=" << flip_y_applied_ << ")";
}

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

void SpecificWorker::save_layout_to_svg(const std::string& filename)
{
    if (room_polygon_.empty())
    {
        qWarning() << "No polygon to save";
        return;
    }

    // Calculate bounding box of the polygon
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& v : room_polygon_)
    {
        min_x = std::min(min_x, v.x());
        min_y = std::min(min_y, v.y());
        max_x = std::max(max_x, v.x());
        max_y = std::max(max_y, v.y());
    }

    // Add margin around the polygon (10% of size)
    const float margin_x = (max_x - min_x) * 0.1f;
    const float margin_y = (max_y - min_y) * 0.1f;
    min_x -= margin_x;
    min_y -= margin_y;
    max_x += margin_x;
    max_y += margin_y;

    const float width = max_x - min_x;
    const float height = max_y - min_y;

    // Scale factor: 100 pixels per meter for good resolution in Inkscape
    constexpr float px_per_meter = 100.0f;
    const float svg_width = width * px_per_meter;
    const float svg_height = height * px_per_meter;

    QFile file(QString::fromStdString(filename));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qWarning() << "Failed to save SVG to" << QString::fromStdString(filename);
        return;
    }

    QTextStream out(&file);
    out.setRealNumberPrecision(4);

    // SVG header with viewBox for proper scaling
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\"\n";
    out << "     xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\"\n";
    out << "     width=\"" << svg_width << "\" height=\"" << svg_height << "\"\n";
    out << "     viewBox=\"" << min_x << " " << -max_y << " " << width << " " << height << "\">\n";
    out << "  <!-- Room layout polygon - editable in Inkscape -->\n";
    out << "  <!-- Coordinates are in meters. Scale: " << px_per_meter << " pixels/meter -->\n";
    out << "  <!-- Note: Y-axis is flipped (SVG Y increases downward) -->\n";
    out << "\n";

    // Add a grid layer for reference (optional, helps with editing)
    out << "  <g inkscape:groupmode=\"layer\" inkscape:label=\"Grid\" style=\"opacity:0.3\">\n";
    const int grid_start_x = static_cast<int>(std::floor(min_x));
    const int grid_end_x = static_cast<int>(std::ceil(max_x));
    const int grid_start_y = static_cast<int>(std::floor(min_y));
    const int grid_end_y = static_cast<int>(std::ceil(max_y));
    for (int x = grid_start_x; x <= grid_end_x; ++x)
    {
        out << "    <line x1=\"" << x << "\" y1=\"" << -max_y << "\" x2=\"" << x << "\" y2=\"" << -min_y
            << "\" stroke=\"#cccccc\" stroke-width=\"0.01\"/>\n";
    }
    for (int y = grid_start_y; y <= grid_end_y; ++y)
    {
        out << "    <line x1=\"" << min_x << "\" y1=\"" << -y << "\" x2=\"" << max_x << "\" y2=\"" << -y
            << "\" stroke=\"#cccccc\" stroke-width=\"0.01\"/>\n";
    }
    out << "  </g>\n\n";

    // Origin marker
    out << "  <g inkscape:groupmode=\"layer\" inkscape:label=\"Origin\">\n";
    out << "    <circle cx=\"0\" cy=\"0\" r=\"0.1\" fill=\"red\" opacity=\"0.7\"/>\n";
    out << "    <line x1=\"-0.3\" y1=\"0\" x2=\"0.3\" y2=\"0\" stroke=\"red\" stroke-width=\"0.02\"/>\n";
    out << "    <line x1=\"0\" y1=\"-0.3\" x2=\"0\" y2=\"0.3\" stroke=\"red\" stroke-width=\"0.02\"/>\n";
    out << "  </g>\n\n";

    // Room polygon layer
    out << "  <g inkscape:groupmode=\"layer\" inkscape:label=\"Room Polygon\">\n";
    out << "    <polygon\n";
    out << "      id=\"room_contour\"\n";
    out << "      inkscape:label=\"Room Contour\"\n";
    out << "      points=\"";

    // Write polygon points (flip Y for SVG coordinate system)
    for (size_t i = 0; i < room_polygon_.size(); ++i)
    {
        if (i > 0) out << " ";
        out << room_polygon_[i].x() << "," << -room_polygon_[i].y();
    }

    out << "\"\n";
    out << "      style=\"fill:none;stroke:#ff00ff;stroke-width:0.05;stroke-linejoin:round\"/>\n";

    // Add vertex circles for easier editing
    out << "    <!-- Vertex markers -->\n";
    for (size_t i = 0; i < room_polygon_.size(); ++i)
    {
        out << "    <circle cx=\"" << room_polygon_[i].x() << "\" cy=\"" << -room_polygon_[i].y()
            << "\" r=\"0.08\" fill=\"#ffff00\" stroke=\"#000000\" stroke-width=\"0.01\""
            << " inkscape:label=\"Vertex " << i << "\"/>\n";
    }
    out << "  </g>\n";

    out << "</svg>\n";

    file.close();
    qInfo() << "SVG layout saved to" << QString::fromStdString(filename)
            << "(" << room_polygon_.size() << " vertices)";
}

void SpecificWorker::load_layout_from_file(const std::string& filename)
{
    load_polygon_from_file(filename);

    // If polygon was loaded, initialize room_ai
    if (room_polygon_.size() >= 3)
    {
        push_loc_command(LocCmdSetPolygon{room_polygon_});
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
        qInfo() << "Layout loaded and room_ai initialized with" << room_polygon_.size() << "vertices,"
                << furniture_polygons_.size() << "furniture polygons";
    }
}

void SpecificWorker::save_fitted_meshes_to_json()
{
    QJsonArray objects;
    for (const auto& fp : furniture_polygons_)
    {
        if (fp.vertices.size() < 3)
            continue;

        QJsonObject obj;
        obj["id"] = QString::fromStdString(fp.id);
        obj["label"] = QString::fromStdString(fp.label);

        QJsonArray verts;
        for (const auto& v : fp.vertices)
        {
            QJsonArray p;
            p.append(static_cast<double>(v.x()));
            p.append(static_cast<double>(v.y()));
            verts.append(p);
        }
        obj["vertices"] = verts;
        objects.append(obj);
    }

    QJsonObject root;
    root["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    root["objects"] = objects;

    QFile file(FITTED_MESHES_FILE);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        qWarning() << "Cannot write fitted meshes to" << FITTED_MESHES_FILE;
        return;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    qInfo() << "Saved fitted meshes to" << FITTED_MESHES_FILE << "objects=" << objects.size();
}

void SpecificWorker::load_fitted_meshes_from_json()
{
    QFile file(FITTED_MESHES_FILE);
    if (!file.open(QIODevice::ReadOnly))
    {
        qInfo() << "No fitted meshes file found at" << FITTED_MESHES_FILE;
        return;
    }

    QJsonParseError parseError;
    const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    file.close();
    if (parseError.error != QJsonParseError::NoError || !doc.isObject())
    {
        qWarning() << "Failed parsing fitted meshes file:" << parseError.errorString();
        return;
    }

    const auto arr = doc.object().value("objects").toArray();
    QHash<QString, std::vector<Eigen::Vector2f>> by_id;
    QHash<QString, std::vector<Eigen::Vector2f>> by_label;
    for (const auto& item : arr)
    {
        const auto obj = item.toObject();
        std::vector<Eigen::Vector2f> verts;
        const auto jverts = obj.value("vertices").toArray();
        verts.reserve(jverts.size());
        for (const auto& vv : jverts)
        {
            const auto p = vv.toArray();
            if (p.size() < 2) continue;
            verts.emplace_back(static_cast<float>(p[0].toDouble()), static_cast<float>(p[1].toDouble()));
        }
        if (verts.size() < 3)
            continue;

        const QString id = obj.value("id").toString().trimmed();
        const QString label = obj.value("label").toString().trimmed();
        if (!id.isEmpty()) by_id.insert(id, verts);
        if (!label.isEmpty()) by_label.insert(label, verts);
    }

    int loaded_count = 0;
    for (auto& fp : furniture_polygons_)
    {
        const QString qid = QString::fromStdString(fp.id);
        const QString qlabel = QString::fromStdString(fp.label);
        if (by_id.contains(qid))
        {
            fp.vertices = by_id.value(qid);
            ++loaded_count;
        }
        else if (by_label.contains(qlabel))
        {
            fp.vertices = by_label.value(qlabel);
            ++loaded_count;
        }
    }

    qInfo() << "Loaded fitted meshes from" << FITTED_MESHES_FILE << "matched objects=" << loaded_count;
}

void SpecificWorker::load_polygon_from_file(const std::string& filename)
{
    if (!filename.empty())
        current_layout_file_ = filename;

    QFile file(QString::fromStdString(filename));
    if (!file.open(QIODevice::ReadOnly))
    {
        qInfo() << "No default layout file found at" << QString::fromStdString(filename);
        return;
    }
    qInfo() << "Loading layout from" << QString::fromStdString(filename);

    QByteArray data = file.readAll();
    file.close();

    // Clear previous polygon
    room_polygon_.clear();
    for (auto* item : polygon_vertex_items)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    polygon_vertex_items.clear();

    // Parse SVG file
    QString content = QString::fromUtf8(data);
    load_polygon_from_svg(content);
    load_fitted_meshes_from_json();

    qInfo() << "Polygon loaded from" << QString::fromStdString(filename)
            << "with" << room_polygon_.size() << "vertices";
}

void SpecificWorker::load_polygon_from_svg(const QString& svg_content)
{
    // -----------------------------------------------------------------------
    // Helper: parse an SVG path 'd' attribute into a list of absolute points.
    // Returns empty vector if the path is NOT closed (no Z/z command).
    // -----------------------------------------------------------------------
    auto parsePath = [](const QString& pathData, bool requireClosed) -> std::vector<Eigen::Vector2f>
    {
        std::vector<Eigen::Vector2f> pts;
        bool isClosed = false;

        QRegularExpression cmdRegex(R"(([MmLlHhVvCcSsQqTtAaZz])\s*([-\d\.\s,eE+]*))");
        QRegularExpressionMatchIterator it = cmdRegex.globalMatch(pathData);

        float cx = 0, cy = 0, sx = 0, sy = 0;
        bool firstPoint = true;

        auto parseNums = [](const QString& s) -> std::vector<float> {
            std::vector<float> nums;
            QRegularExpression numRe(R"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)");
            auto nit = numRe.globalMatch(s);
            while (nit.hasNext()) nums.push_back(nit.next().captured(0).toFloat());
            return nums;
        };

        while (it.hasNext())
        {
            auto m = it.next();
            QString cmd = m.captured(1);
            auto nums = parseNums(m.captured(2).trimmed());

            if (cmd == "M")
            {
                if (nums.size() >= 2) { cx = nums[0]; cy = nums[1]; sx = cx; sy = cy;
                    pts.emplace_back(cx, -cy); firstPoint = false;
                    for (size_t i = 2; i + 1 < nums.size(); i += 2)
                    { cx = nums[i]; cy = nums[i+1]; pts.emplace_back(cx, -cy); } }
            }
            else if (cmd == "m")
            {
                if (nums.size() >= 2) {
                    if (firstPoint) { cx = nums[0]; cy = nums[1]; }
                    else            { cx += nums[0]; cy += nums[1]; }
                    sx = cx; sy = cy; pts.emplace_back(cx, -cy); firstPoint = false;
                    for (size_t i = 2; i + 1 < nums.size(); i += 2)
                    { cx += nums[i]; cy += nums[i+1]; pts.emplace_back(cx, -cy); } }
            }
            else if (cmd == "L")
            { for (size_t i = 0; i + 1 < nums.size(); i += 2)
                { cx = nums[i]; cy = nums[i+1]; pts.emplace_back(cx, -cy); } }
            else if (cmd == "l")
            { for (size_t i = 0; i + 1 < nums.size(); i += 2)
                { cx += nums[i]; cy += nums[i+1]; pts.emplace_back(cx, -cy); } }
            else if (cmd == "H") { for (auto v : nums) { cx = v;  pts.emplace_back(cx, -cy); } }
            else if (cmd == "h") { for (auto v : nums) { cx += v; pts.emplace_back(cx, -cy); } }
            else if (cmd == "V") { for (auto v : nums) { cy = v;  pts.emplace_back(cx, -cy); } }
            else if (cmd == "v") { for (auto v : nums) { cy += v; pts.emplace_back(cx, -cy); } }
            else if (cmd == "C")
            { for (size_t i = 0; i + 5 < nums.size(); i += 6)
                { cx = nums[i+4]; cy = nums[i+5]; pts.emplace_back(cx, -cy); } }
            else if (cmd == "c")
            { for (size_t i = 0; i + 5 < nums.size(); i += 6)
                { cx += nums[i+4]; cy += nums[i+5]; pts.emplace_back(cx, -cy); } }
            else if (cmd == "Z" || cmd == "z")
            { cx = sx; cy = sy; isClosed = true; }
        }

        if (requireClosed && !isClosed) return {};

        // Remove duplicate closing point
        if (pts.size() > 2)
            if (std::fabs(pts.front().x() - pts.back().x()) < 0.01f &&
                std::fabs(pts.front().y() - pts.back().y()) < 0.01f)
                pts.pop_back();

        return pts;
    };

    // -----------------------------------------------------------------------
    // Helper: apply SVG matrix(a,b,c,d,e,f) transform to a list of points.
    // parsePath already stores Y-flipped (-cy), so we must undo+redo the flip.
    // -----------------------------------------------------------------------
    auto applyMatrix = [](std::vector<Eigen::Vector2f>& pts, const std::array<float,6>& mat)
    {
        float a=mat[0], b=mat[1], c=mat[2], d=mat[3], e=mat[4], f=mat[5];
        for (auto& p : pts)
        {
            float px = p.x(), py = -p.y();   // undo Y-flip
            float tx = a*px + c*py + e;
            float ty = b*px + d*py + f;
            p = Eigen::Vector2f(tx, -ty);    // re-apply Y-flip
        }
    };

    // -----------------------------------------------------------------------
    // Helper: extract matrix from a transform="matrix(...)" attribute string.
    // -----------------------------------------------------------------------
    auto extractMatrix = [](const QString& attr, std::array<float,6>& mat) -> bool
    {
        QRegularExpression re(R"(matrix\(\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*\))");
        auto m = re.match(attr);
        if (!m.hasMatch()) return false;
        for (int i = 0; i < 6; ++i) mat[i] = m.captured(i+1).toFloat();
        return true;
    };

    // -----------------------------------------------------------------------
    // Struct to hold a parsed path with its metadata
    // -----------------------------------------------------------------------
    struct ParsedPath {
        std::vector<Eigen::Vector2f> pts;
        QString id;
        QString label;         // inkscape:label of the path itself
        QString layer_label;   // inkscape:label of the parent <g> layer
    };
    std::vector<ParsedPath> allPaths;

    // -----------------------------------------------------------------------
    // Parse <path> elements using Qt's XML parser for robustness.
    // For paths inside a <g transform="matrix(...)"> apply the group transform.
    // -----------------------------------------------------------------------
    QDomDocument doc;
    QString parseErr;
    int errLine, errCol;
    if (!doc.setContent(svg_content, false, &parseErr, &errLine, &errCol))
    {
        qWarning() << "[SVG] XML parse error at line" << errLine << "col" << errCol << ":" << parseErr;
    }
    else
    {
        // Recursive lambda to walk the DOM tree, propagating layer label
        std::function<void(const QDomElement&, std::array<float,6>, QString)> walkElement;
        walkElement = [&](const QDomElement& elem, std::array<float,6> parentMat, QString layerLabel)
        {
            QString tag = elem.tagName();

            // Accumulate transform from this element
            std::array<float,6> currentMat = parentMat;
            if (elem.hasAttribute("transform"))
            {
                std::array<float,6> localMat = {1,0,0,1,0,0};
                if (extractMatrix(elem.attribute("transform"), localMat))
                {
                    float a1=parentMat[0],b1=parentMat[1],c1=parentMat[2],
                          d1=parentMat[3],e1=parentMat[4],f1=parentMat[5];
                    float a2=localMat[0], b2=localMat[1], c2=localMat[2],
                          d2=localMat[3], e2=localMat[4], f2=localMat[5];
                    currentMat[0] = a1*a2 + c1*b2;
                    currentMat[1] = b1*a2 + d1*b2;
                    currentMat[2] = a1*c2 + c1*d2;
                    currentMat[3] = b1*c2 + d1*d2;
                    currentMat[4] = a1*e2 + c1*f2 + e1;
                    currentMat[5] = b1*e2 + d1*f2 + f1;
                }
            }

            // Track Inkscape layer label: <g inkscape:groupmode="layer" inkscape:label="XXX">
            QString currentLayerLabel = layerLabel;
            if (tag == "g" && elem.attribute("inkscape:groupmode") == "layer")
            {
                currentLayerLabel = elem.attribute("inkscape:label");
            }

            if (tag == "path")
            {
                QString d = elem.attribute("d");
                if (!d.isEmpty())
                {
                    auto pts = parsePath(d, /*requireClosed=*/true);
                    if (pts.size() >= 3)
                    {
                        // Apply accumulated transform
                        bool isIdentity = (currentMat[0]==1 && currentMat[1]==0 && currentMat[2]==0 &&
                                           currentMat[3]==1 && currentMat[4]==0 && currentMat[5]==0);
                        if (!isIdentity) applyMatrix(pts, currentMat);

                        ParsedPath pp;
                        pp.pts = std::move(pts);
                        pp.id  = elem.attribute("id");
                        pp.label = elem.attribute("inkscape:label");
                        pp.layer_label = currentLayerLabel;
                        allPaths.push_back(std::move(pp));

                        qInfo() << "[SVG] Found closed path id=" << allPaths.back().id
                                << "label=" << allPaths.back().label
                                << "layer=" << allPaths.back().layer_label
                                << "vertices=" << allPaths.back().pts.size();
                    }
                }
            }
            else if (tag == "rect")
            {
                // Inkscape draws rectangles as <rect> elements; convert to 4-point polygon.
                const float rx = elem.attribute("x", "0").toFloat();
                const float ry = elem.attribute("y", "0").toFloat();
                const float rw = elem.attribute("width",  "0").toFloat();
                const float rh = elem.attribute("height", "0").toFloat();
                if (rw > 0.f && rh > 0.f)
                {
                    // Points stored Y-flipped to match parsePath convention (-cy)
                    std::vector<Eigen::Vector2f> pts = {
                        { rx,      -ry       },
                        { rx + rw, -ry       },
                        { rx + rw, -(ry + rh)},
                        { rx,      -(ry + rh)}
                    };
                    bool isIdentity = (currentMat[0]==1 && currentMat[1]==0 && currentMat[2]==0 &&
                                       currentMat[3]==1 && currentMat[4]==0 && currentMat[5]==0);
                    if (!isIdentity) applyMatrix(pts, currentMat);

                    ParsedPath pp;
                    pp.pts = std::move(pts);
                    pp.id  = elem.attribute("id");
                    pp.label = elem.attribute("inkscape:label");
                    if (pp.label.isEmpty()) pp.label = pp.id;
                    pp.layer_label = currentLayerLabel;
                    allPaths.push_back(std::move(pp));

                    qInfo() << "[SVG] Found rect id=" << allPaths.back().id
                            << "label=" << allPaths.back().label
                            << "layer=" << allPaths.back().layer_label;
                }
            }
            else
            {
                QDomNode child = elem.firstChild();
                while (!child.isNull())
                {
                    if (child.isElement())
                        walkElement(child.toElement(), currentMat, currentLayerLabel);
                    child = child.nextSibling();
                }
            }
        };

        std::array<float,6> identity = {1,0,0,1,0,0};
        walkElement(doc.documentElement(), identity, QString());
    }

    if (allPaths.empty())
    {
        qWarning() << "[SVG] No closed paths found.";
        return;
    }

    qInfo() << "[SVG] Total closed paths found:" << allPaths.size();

    // Classify paths by layer:
    //   - Paths in a layer whose label contains "Furniture" (case-insensitive) → furniture obstacles
    //   - First path NOT in a "Furniture" layer → room contour
    //   - Remaining non-furniture paths are ignored (or could be alternate room contours)
    furniture_polygons_.clear();
    bool room_found = false;

    for (const auto& pp : allPaths)
    {
        const bool is_furniture = pp.layer_label.contains("furniture", Qt::CaseInsensitive) ||
                                  pp.layer_label.contains("obstacle", Qt::CaseInsensitive);
        if (is_furniture)
        {
            FurniturePolygon fp;
            fp.id = pp.id.toStdString();
            fp.label = pp.label.isEmpty() ? pp.id.toStdString() : pp.label.toStdString();
            fp.vertices = pp.pts;
            furniture_polygons_.push_back(std::move(fp));
            qInfo() << "[SVG] Furniture:" << QString::fromStdString(furniture_polygons_.back().label)
                    << "vertices=" << furniture_polygons_.back().vertices.size();
        }
        else if (!room_found)
        {
            room_polygon_ = pp.pts;
            room_found = true;
            qInfo() << "[SVG] room_polygon_ set to path id='" << pp.id
                    << "' (layer=" << pp.layer_label << ") with" << room_polygon_.size() << "vertices";
        }
        else
        {
            qInfo() << "[SVG] Skipping extra non-furniture path id='" << pp.id
                    << "' (layer=" << pp.layer_label << ")";
        }
    }

    if (!room_found && !allPaths.empty())
    {
        // Fallback: no layer classification, use first path as room contour
        room_polygon_ = allPaths[0].pts;
        qInfo() << "[SVG] No layer classification found. Using first path as room_polygon_: id='" << allPaths[0].id
                << "' with" << room_polygon_.size() << "vertices";
    }

    qInfo() << "[SVG] Room polygon:" << room_polygon_.size() << "vertices,"
            << furniture_polygons_.size() << "furniture polygons";
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

    const FurniturePolygon *selected = nullptr;

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
            Navigator_gotoPoint(target);
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
