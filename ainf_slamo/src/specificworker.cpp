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
	if (viewer_2d_)
	{
		QByteArray ba;
		QDataStream ds(&ba, QIODevice::WriteOnly);
		ds << viewer_2d_->transform();
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
	if (nav_manager_.is_active())
	{
		nav_manager_.stop();
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

    // 2D Viewer
    viewer_2d_ = std::make_unique<rc::Viewer2D>(this->frame, params.GRID_MAX_DIM, true);
    viewer_2d_->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0.0, 0.2, QColor("Blue"));
    viewer_2d_->show();

    // Add 3D viewer alongside the 2D viewer in a horizontal splitter.
    // AbstractGraphicViewer already created a QVBoxLayout on this->frame;
    // we grab viewer_2d_ back from it, insert a QSplitter, and add Viewer3D.
    {
        QLayout* frame_layout = this->frame->layout();  // QVBoxLayout from AGV ctor
        frame_layout->removeWidget(viewer_2d_->get_widget());
        auto* splitter = new QSplitter(Qt::Horizontal, this->frame);
        splitter->addWidget(viewer_2d_->get_widget());

        viewer_3d_ = std::make_unique<rc::Viewer3D>(splitter, 2.5f);
        splitter->addWidget(viewer_3d_->container_widget());

        auto* right_splitter = new QSplitter(Qt::Vertical, splitter);
        scene_tree_ = std::make_unique<SceneTreePanel>(right_splitter);
        right_splitter->addWidget(scene_tree_.get());

        // Hidden grounding-status container: labels still receive setText() calls
        // from grounding algorithms but are never shown in the layout.
        grounding_panel_ = new QWidget(this);
        grounding_status_label_    = new QLabel("Status: waiting camera detections", grounding_panel_);
        grounding_cam_label_       = new QLabel("Camera object: -",  grounding_panel_);
        grounding_world_label_     = new QLabel("World object: -",   grounding_panel_);
        grounding_score_label_     = new QLabel("Score: -",          grounding_panel_);
        grounding_sdf_label_       = new QLabel("SDF fit: -",        grounding_panel_);
        grounding_fit_mesh_button_ = new QPushButton("Fit Grounded Mesh", grounding_panel_);
        grounding_panel_->setVisible(false);

        // Object-palette panel (lower half of right column)
        object_palette_ = new ObjectPalettePanel(right_splitter);
        right_splitter->addWidget(object_palette_);
        right_splitter->setSizes({300, 160});
        right_splitter_ = right_splitter;

        splitter->addWidget(right_splitter);
        splitter->setSizes({450, 450, 220});
        splitter->setCollapsible(2, true);  // right panel can be collapsed
        splitter_ = splitter;   // store for state save/restore
        frame_layout->addWidget(splitter);

        // Toggle tree panel button
        connect(pushButton_toggleTree, &QPushButton::toggled, this, [this](bool checked) {
            if (checked) {
                // Show: restore saved width
                auto sizes = splitter_->sizes();
                sizes[2] = tree_panel_saved_width_ > 0 ? tree_panel_saved_width_ : 220;
                splitter_->setSizes(sizes);
                pushButton_toggleTree->setText("◀ Tree");
            } else {
                // Hide: save current width then collapse
                const auto sizes = splitter_->sizes();
                if (sizes.size() > 2 && sizes[2] > 0)
                    tree_panel_saved_width_ = sizes[2];
                auto new_sizes = sizes;
                new_sizes[2] = 0;
                splitter_->setSizes(new_sizes);
                pushButton_toggleTree->setText("▶ Tree");
            }
        });

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

        // Save scene to disk once the user releases the gizmo after dragging.
        connect(viewer_3d_.get(), &rc::Viewer3D::objectDragFinished,
                this, [this](const QString&)
        {
            save_scene_graph_to_usd();
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
            if (!nav_manager_.is_active() && !nav_manager_.has_path()) return;
            qInfo() << "Navigation cancelled by Ctrl+Right click (3D View)";
            active_episode_.reset();
            episode_accum_ = EpisodeAccum{};
            if (label_episodeStatus != nullptr)
            {
                label_episodeStatus->setText("EP: CANCELLED");
                label_episodeStatus->setStyleSheet("background-color: #FFE0B2; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
            }
            nav_manager_.stop();
            clear_path();
        });

        connect(grounding_fit_mesh_button_, &QPushButton::clicked, this, [this]()
        {
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
            try
            {
                const auto tdata = imagesegmentation_proxy->getAll(false);
                em_manager_.run_camera_validator(res_opt->robot_pose, tdata);
            }
            catch (const Ice::Exception&)
            {
                if (grounding_status_label_)
                    grounding_status_label_->setText("Status: EM validator failed (segmentation unavailable)");
            }
        });

        // Object palette: add a new furniture object to all views on request
        connect(object_palette_, &ObjectPalettePanel::addObjectRequested,
                this, [this](const QString& type_label,
                             float tx, float ty, float yaw_deg,
                             float width_m, float depth_m, float height_m)
        {
            // Build a unique numbered label (e.g. "chair_2")
            int count = 1;
            for (const auto& fp : layout_manager_.furniture())
            {
                const QString ql = QString::fromStdString(fp.label).toLower();
                if (ql.startsWith(type_label.toLower()))
                    ++count;
            }
            const std::string label_std = (type_label + "_" + QString::number(count)).toStdString();

            const float yaw_rad = yaw_deg * static_cast<float>(M_PI) / 180.f;

            rc::FurniturePolygonData fp;
            fp.id                   = label_std;
            fp.label                = label_std;
            fp.height               = height_m;
            fp.frame_yaw_inward_rad = yaw_rad;
            fp.vertices             = rc::SceneGraphModel::make_world_rect_polygon(
                                          tx, ty, yaw_rad, width_m, depth_m);
            layout_manager_.add_furniture(std::move(fp));

            // Sync planner obstacles
            {
                const auto obs = layout_manager_.obstacle_polygons();
                nav_manager_.path_planner().set_obstacles(obs);
                nav_manager_.trajectory_controller().set_static_obstacles(obs);
            }

            // Rebuild the scene graph model to include the new object, then draw and save.
            rc::SceneGraphAdapter::rebuild_graph(
                scene_graph_, layout_manager_.room_polygon(), layout_manager_.furniture(),
                [](const std::string& l) { return EMManager::model_height_from_label(l); }, 2.6f);
            draw_furniture();
            save_scene_graph_to_usd();

            qInfo() << "[Palette] Added" << QString::fromStdString(label_std)
                    << "at (" << tx << "," << ty << ") yaw=" << yaw_deg << "°";
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
        viewer_2d_->set_transform(t);
    }
    else
    {
        // First run: ~8 px/m so the full room fits; Y stays flipped (negative scale)
        viewer_2d_->set_transform(QTransform().scale(8.0, -8.0));
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
            right_splitter_->setSizes({380, 160});
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
    connect(viewer_2d_.get(), &rc::Viewer2D::robot_rotate, this, &SpecificWorker::on_scene_clicked);

    // Connect robot drag and rotate signals
    connect(viewer_2d_.get(), &rc::Viewer2D::robot_dragging, this, &SpecificWorker::slot_robot_dragging);
    connect(viewer_2d_.get(), &rc::Viewer2D::robot_drag_end, this, &SpecificWorker::slot_robot_drag_end);
    connect(viewer_2d_.get(), &rc::Viewer2D::robot_rotate, this, &SpecificWorker::slot_robot_rotate);

    // Connect save/load layout buttons
    connect(pushButton_saveLayout, &QPushButton::clicked, this, &SpecificWorker::slot_save_layout);
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
            if (em_manager_.is_overlay_persistent())
            {
                em_manager_.clear_overlay();
                if (grounding_status_label_)
                    grounding_status_label_->setText("Status: EM overlay cleared");
                if (grounding_sdf_label_)
                    grounding_sdf_label_->setText("SDF fit: -");
            }
            else
            {
                // Fetch localization and camera data then delegate to EM manager
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
                try
                {
                    const auto tdata = imagesegmentation_proxy->getAll(false);
                    em_manager_.run_camera_validator(res_opt->robot_pose, tdata);
                }
                catch (const Ice::Exception&)
                {
                    if (grounding_status_label_)
                        grounding_status_label_->setText("Status: EM validator failed (segmentation unavailable)");
                }
            }
        });
        connect(camera_viewer_.get(), &CameraViewer::emAcceptRequested, this, [this]() {
            em_manager_.apply_pending_adjustments(true);
        });
        connect(camera_viewer_.get(), &CameraViewer::emRejectRequested, this, [this]() {
            em_manager_.apply_pending_adjustments(false);
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

        // ---- New MVC wiring ----
        // Route tree property edits through set_object_property.
        connect(scene_tree_.get(), &SceneTreePanel::objectPropertyEdited,
                this, [this](const QString& label, const QString& prop, float val)
        {
            set_object_property(label, prop, val);
        });

        // Remove a furniture object via right-click context menu in the tree.
        connect(scene_tree_.get(), &SceneTreePanel::removeObjectRequested,
                this, [this](const QString& label)
        {
            const std::string lbl = label.toStdString();
            layout_manager_.remove_furniture(lbl);
            // Update planner obstacles
            {
                const auto obs = layout_manager_.obstacle_polygons();
                nav_manager_.path_planner().set_obstacles(obs);
                nav_manager_.trajectory_controller().set_static_obstacles(obs);
            }
            // Remove from model, then rebuild graph from updated furniture list.
            scene_graph_.remove_object("", lbl);
            draw_furniture();
            save_scene_graph_to_usd();
            qInfo() << "[Tree] Removed" << label;
        });

        // Attach model — the panel will now auto-rebuild when the model emits modelRebuilt.
        scene_tree_->set_model(&scene_graph_);
    }

    // When the model changes a single object (via set_object_pose / set_object_extents),
    // update ONLY that object's visuals — never rebuild the entire scene graph.
    connect(&scene_graph_, &rc::SceneGraphModel::objectChanged,
            this, [this](const QString& label)
    {
        auto node_opt = scene_graph_.get_object_node(label.toStdString());
        if (!node_opt) return;
        const auto& node = *node_opt;

        // Update the single affected entry in layout_manager_ furniture.
        const int idx = find_furniture_index_by_name(label);
        if (idx < 0 || idx >= static_cast<int>(layout_manager_.furniture().size()))
            return;

        const auto uidx = static_cast<std::size_t>(idx);
        auto& fp = layout_manager_.furniture_mutable()[uidx];
        fp.height               = std::max(0.2f, node.extents.z());
        fp.frame_yaw_inward_rad = node.yaw_rad;
        fp.vertices             = rc::footprints::make(
            node.object_type,
            node.translation.x(), node.translation.y(), node.yaw_rad,
            std::max(0.08f, node.extents.x()),
            std::max(0.08f, node.extents.y()));

        // Redraw only this object in 2D.
        update_furniture_draw_item(uidx);

        // Rebuild the 3D mesh for this object (needs full list but only this item changed).
        if (viewer_3d_)
        {
            std::vector<rc::Viewer3D::FurnitureItem> items;
            items.reserve(layout_manager_.furniture().size());
            for (const auto& f : layout_manager_.furniture())
            {
                if (f.vertices.empty()) continue;
                auto nd = scene_graph_.get_object_node(f.label);
                if (!nd) continue;
                rc::Viewer3D::FurnitureItem fi;
                fi.label    = f.label;
                fi.centroid = Eigen::Vector2f(nd->translation.x(), nd->translation.y());
                fi.size     = Eigen::Vector2f(std::max(0.08f, nd->extents.x()),
                                              std::max(0.08f, nd->extents.y()));
                fi.yaw_rad  = nd->yaw_rad;
                fi.height   = std::max(0.2f, nd->extents.z());
                items.push_back(fi);
            }
            viewer_3d_->update_furniture(items);
        }

        // Update tree display for this object.
        if (scene_tree_)
            scene_tree_->update_object_display(label);

        // Persist to disk (this is a model-only serialization, no rebuild).
        save_scene_graph_to_usd();
    });

    // PD / MPPI mode toggle
    connect(pushButton_pdMode, &QPushButton::toggled, this, [this](bool checked)
    {
        if (checked)
        {
            nav_manager_.trajectory_controller().set_control_mode(rc::TrajectoryController::ControlMode::PD);
            pushButton_pdMode->setStyleSheet("background-color: #FFF176; font-weight: bold;");
            qInfo() << "Controller mode: PD carrot-follower";
        }
        else
        {
            nav_manager_.trajectory_controller().set_control_mode(rc::TrajectoryController::ControlMode::MPPI);
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
            nav_manager_.trajectory_controller().params.mood = mood;
            if (label_moodValue != nullptr)
                label_moodValue->setText(QString::number(mood, 'f', 2));
        });

        horizontalSlider_mood->setValue(50);  // init value = 0.50
    }
    nav_manager_.trajectory_controller().params.mood = 0.5f;

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
    connect(viewer_2d_.get(), &rc::Viewer2D::new_mouse_coordinates, this, &SpecificWorker::slot_new_target);

    // Ctrl+Right click on scene: cancel current navigation mission
    connect(viewer_2d_.get(), &rc::Viewer2D::right_click, this, [this](QPointF)
    {
        if (!nav_manager_.is_active() && !nav_manager_.has_path()) return;
        qInfo() << "Navigation cancelled by Ctrl+Right click";
        active_episode_.reset();
        episode_accum_ = EpisodeAccum{};
        if (label_episodeStatus != nullptr)
        {
            label_episodeStatus->setText("EP: CANCELLED");
            label_episodeStatus->setStyleSheet("background-color: #FFE0B2; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
        }
        nav_manager_.stop();
        clear_path();
    });

    // Ctrl+Shift+Right keyboard: cancel current navigation mission without saving episode
    auto *cancel_shortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_Right), this);
    connect(cancel_shortcut, &QShortcut::activated, this, [this]()
    {
        if (!nav_manager_.is_active() && !nav_manager_.has_path()) return;
        qInfo() << "Navigation cancelled by Ctrl+Shift+Right";
        // Discard episode without saving
        active_episode_.reset();
        episode_accum_ = EpisodeAccum{};
        if (label_episodeStatus != nullptr)
        {
            label_episodeStatus->setText("EP: CANCELLED");
            label_episodeStatus->setStyleSheet("background-color: #FFE0B2; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
        }
        nav_manager_.stop();
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
    if (!layout_manager_.room_polygon().empty())
    {
        // Use pre-loaded polygon from file
        room_ai.set_polygon_room(layout_manager_.room_polygon());
        nav_manager_.path_planner().set_polygon(layout_manager_.room_polygon());
        nav_manager_.trajectory_controller().set_room_boundary(layout_manager_.room_polygon());
        if (!layout_manager_.furniture().empty())
        {
            const auto obs = layout_manager_.obstacle_polygons();
            nav_manager_.path_planner().set_obstacles(obs);
            nav_manager_.trajectory_controller().set_static_obstacles(obs);
        }
        draw_room_polygon();
        draw_furniture();
        qInfo() << "RoomConceptAI initialized with loaded polygon:" << layout_manager_.room_polygon().size() << "vertices"
                << "furniture:" << layout_manager_.furniture().size();

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
    viewer_2d_->fit_to_scene(QRectF(robot_cx - view_side_m/2.f, robot_cy - view_side_m/2.f, view_side_m, view_side_m));
    viewer_2d_->center_on(robot_cx, robot_cy);

    // Start localization thread (room_ai runs independently from compute loop)
    localization_th_ = std::thread(&SpecificWorker::run_localization, this);
    qInfo() << __FUNCTION__ << "Started localization thread";

    // EM manager context setup (after camera_viewer, scene_graph, furniture are ready)
    {
        EMManager::Context em_ctx;
        em_ctx.camera_viewer = camera_viewer_.get();
        em_ctx.viewer_2d = viewer_2d_.get();
        em_ctx.furniture_polygons = &layout_manager_.furniture_mutable();
        em_ctx.room_polygon = &layout_manager_.room_polygon();
        em_ctx.scene_graph = &scene_graph_;
        em_ctx.status_label = grounding_status_label_;
        em_ctx.sdf_label = grounding_sdf_label_;
        em_ctx.camera_tx = params.CAMERA_TX;
        em_ctx.camera_ty = params.CAMERA_TY;
        em_ctx.camera_tz = params.CAMERA_TZ;
        em_ctx.on_accepted = [this]() { draw_furniture(); save_scene_graph_to_usd(); };
        em_manager_.set_context(em_ctx);
    }

    // Navigation manager context setup
    {
        NavigationManager::Context nav_ctx;
        nav_ctx.layout_manager = &layout_manager_;
        nav_ctx.get_loc_state = [this]() { return get_loc_state(); };
        nav_ctx.send_velocity = [this](float adv, float side, float rot) { send_velocity_command(adv, side, rot); };
        nav_ctx.draw_path = [this](const std::vector<Eigen::Vector2f>& path) { draw_path_threadsafe(path); };
        nav_ctx.draw_temp_obstacles = [this]() { draw_temp_obstacles(); };
        nav_ctx.clear_path_visual = [this]() {
            viewer_2d_->clear_path_items();
            viewer_2d_->hide_trajectory_debug();
            if (viewer_3d_) viewer_3d_->update_path({});
        };
        nav_manager_.set_context(nav_ctx);
    }

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

    if (em_manager_.is_enabled() && lidar_low_high_.has_value())
        em_manager_.run_ownership_step(lidar_low_high_->first, res.robot_pose);

    if (camera_viewer_ && !camera_viewer_->is_raw_mode())
    {
        camera_viewer_->set_infrastructure_context(res.robot_pose,
                                                   layout_manager_.room_polygon(),
                                                   params.CAMERA_TX,
                                                   params.CAMERA_TY,
                                                   params.CAMERA_TZ,
                                                   2.5f);
        if (!em_manager_.is_overlay_locked())
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
        if (!nav_manager_.temp_obstacles().empty())
            nav_manager_.cleanup_temp_obstacles();
    }

    // ===== LOCAL TRAJECTORY CONTROLLER (delegated to NavigationManager) =====
    float mppi_ms = 0.f, viz2_ms = 0.f;
    if (nav_manager_.is_active() || nav_manager_.is_aligning())
    {
        const auto step = nav_manager_.compute_step(lidar_low_high_->first, res.robot_pose);
        mppi_ms = step.mppi_ms;

        // Put velocity into buffer for UI/status display
        auto cmd = rc::VelocityCommand(step.side, step.adv, step.rot);
        velocity_buffer_.put<0>(std::move(cmd), timestamp);

        if (step.object_align_started && label_episodeStatus != nullptr)
        {
            label_episodeStatus->setText("EP: ALIGNING OBJECT");
            label_episodeStatus->setStyleSheet("background-color: #BBDEFB; color: black; padding: 2px; border-radius: 2px; font-size: 8pt;");
            return; // alignment just started, skip rest of cycle
        }

        if (step.goal_reached || step.object_align_done)
        {
            finish_episode("success");
            clear_path();
            qInfo() << "[TrajectoryCtrl] Navigation complete.";
        }
        else if (step.has_ctrl_output && step.navigating)
        {
            const float cpu_usage = get_cpu_usage();
            update_episode_metrics(res, &step.ctrl, step.speed, step.rot_speed, cpu_usage, step.mppi_ms, step.blocked_like);

            auto t_viz2_start = std::chrono::steady_clock::now();
            if (do_draw && draw_trajectories_)
                draw_trajectory_debug(step.ctrl, res.robot_pose);
            viz2_ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t_viz2_start).count();
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

/////////////////////////////////////////////////////////////////////
///  INTERNAL METHODS
////////////////////////////////////////////////////////////////////

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
    if (stop_controller && active_episode_.has_value())
        finish_episode("aborted");

    nav_manager_.clear(stop_controller, clear_stored_path);

    viewer_2d_->clear_path_items();

    // Hide trajectory controller debug items
    viewer_2d_->hide_trajectory_debug();

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
        auto poly = layout_manager_.room_polygon();
        for (auto& vertex : poly)
            vertex.x() = -vertex.x();
        layout_manager_.set_room_polygon(std::move(poly));
        flip_x_applied_ = true;

        room_ai.set_polygon_room(layout_manager_.room_polygon());
        nav_manager_.path_planner().set_polygon(layout_manager_.room_polygon());
        draw_room_polygon();
        qInfo() << "Applied saved flip_x";
    }
    if (saved_flip_y)
    {
        auto poly = layout_manager_.room_polygon();
        for (auto& vertex : poly)
            vertex.y() = -vertex.y();
        layout_manager_.set_room_polygon(std::move(poly));
        flip_y_applied_ = true;

        room_ai.set_polygon_room(layout_manager_.room_polygon());
        nav_manager_.path_planner().set_polygon(layout_manager_.room_polygon());
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
		if (nav_manager_.is_active())
		{
			nav_manager_.stop();
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
    ret.layout.reserve(layout_manager_.room_polygon().size());

    for (const auto &p : layout_manager_.room_polygon())
    {
        RoboCompNavigator::TPoint pt;
        pt.x = p.x();
        pt.y = p.y();
        ret.layout.push_back(pt);
    }

    ret.objects.reserve(layout_manager_.furniture().size());
    for (const auto &fp : layout_manager_.furniture())
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
    return nav_manager_.compute_path(Eigen::Vector2f(source.x, source.y),
                                     Eigen::Vector2f(target.x, target.y), safety);
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
    const auto vel_tuple = velocity_buffer_.read_last<0>();
    const auto &vel_opt = std::get<0>(vel_tuple);
    float speed = 0.f;
    if (vel_opt.has_value())
        speed = std::hypot(vel_opt->adv_x, vel_opt->adv_y);

    return nav_manager_.get_status(speed);
}

RoboCompNavigator::TPoint SpecificWorker::Navigator_gotoObject(std::string object)
{
    if (!nav_manager_.goto_object(object))
        return {};

    // Start episode with the computed target (last waypoint)
    const auto& path = nav_manager_.current_path();
    const Eigen::Vector2f target_m = path.empty() ? Eigen::Vector2f::Zero() : path.back();
    start_episode("goto_object", target_m, nav_manager_.nav_target_object_name());

    RoboCompNavigator::TPoint tp;
    tp.x = target_m.x();
    tp.y = target_m.y();
    return tp;
}

void SpecificWorker::Navigator_resume()
{
    nav_manager_.resume();
}

RoboCompNavigator::TPoint SpecificWorker::Navigator_gotoPoint(RoboCompNavigator::TPoint target)
{
    const Eigen::Vector2f tgt(target.x, target.y);
    if (!nav_manager_.goto_point(tgt))
        return {};

    start_episode("goto_point", tgt);
    return target;
}

void SpecificWorker::Navigator_stop()
{
    finish_episode("stopped");
    nav_manager_.stop();
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
