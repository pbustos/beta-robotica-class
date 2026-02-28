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
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFileDialog>
#include <QTextStream>
#include <QRegularExpression>
#include <QDateTime>
#include <QDomDocument>
#include <unistd.h>  // For sysconf
#include <cmath>     // For std::fabs
#include <limits>    // For std::numeric_limits
#include <random>    // For odometry noise

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

	// Save the last known pose for fast restart
	save_last_pose();

	// Stop lidar thread
	stop_lidar_thread = true;
	if (read_lidar_th.joinable())
	    read_lidar_th.join();
}

void SpecificWorker::initialize()
{
    std::cout << "Initializing worker" << std::endl;

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

    // Try to load default layout on startup (only loads vertices, doesn't init room_ai yet)
    // Config keys: layout_file (SVG path)
    std::string layout_file = "";
    try { layout_file  = configLoader.get<std::string>("layout_file");  } catch(...) {}
    try { params.USE_WEBOTS = configLoader.get<bool>("use_webots"); } catch(...) {}
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
    std::cout << __FUNCTION__ << " Started lidar reader" << std::endl;

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
        const auto &[r, ll] = buffer_sync.read(timestamp);
        robot = r; lidar_local = ll;
    }while (++startup_check_counter < 20 && !lidar_local.has_value());

    if (!lidar_local.has_value())
    {
        qWarning() << "initialize(): No lidar data from buffer_sync. Exiting.";
        std::terminate();
    }
    if (!robot.has_value())
        qWarning() << "initialize(): GT pose from webots not available — debug/error stats will be disabled.";

    // Center view in (0,0) since GT room is centered there
    const float view_side_m = 6.f;
    viewer->fitToScene(QRectF(- view_side_m/2.f, - view_side_m/2.f, view_side_m, view_side_m));
    viewer->centerOn(0, 0);

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
        draw_room_polygon();
        qInfo() << "RoomConceptAI initialized with loaded polygon:" << room_polygon_.size() << "vertices";

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
	setPeriod("Compute", 50);
}

void SpecificWorker::compute()
{
    const int fps_val = fps.print("Compute", 2000);
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const auto &[robot_pose_gt_, lidar_local_] = buffer_sync.read(timestamp);

    // Only lidar is required; GT pose is optional (debug/stats only)
    if (!lidar_local_.has_value())
    { qWarning() << "No lidar data from buffer_sync"; return; };
    const auto &lidar_local = lidar_local_.value().first;

    if(room_ai.is_initialized())
    {
        // Pass velocity, odometry and lidar to update
        if(const auto res = room_ai.update(lidar_local_.value(),
                                           velocity_history_,
                                           odometry_history_); res.ok)
        {
            update_ui(res, velocity_history_.back(), fps_val);
            display_robot(res.robot_pose, res.covariance);
            draw_lidar_points(lidar_local, res.robot_pose);

            // GT debug/statistics: compute and display pose error vs ground truth if available
            if (params.USE_WEBOTS)
            {
                // Auto-calibrate GT offset on first low-SDF reading
                if (!gt_calibrated_ && robot_pose_gt_.has_value() && res.sdf_mse < 0.10f)
                    calibrate_gt_offset(res.robot_pose, robot_pose_gt_.value());
                display_gt_error(res.robot_pose, robot_pose_gt_);
            }

            // Draw room rectangle (only if not using polygon)
            draw_estimated_room(res.state);

            // Save pose periodically (every ~30 seconds at 20Hz = 600 frames)
            static int pose_save_counter = 0;
            if (++pose_save_counter >= 600)
            {
                save_last_pose();
                pose_save_counter = 0;
            }
        }
    }
    else qWarning() << "room not initialized";
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

void SpecificWorker::update_ui(const rc::RoomConceptAI::UpdateResult &res,
							   const rc::VelocityCommand &current_velocity,
							   int fps_val)
{
	// Update QLCDNumber displays
	const float sigma_xy_cm = std::sqrt(res.covariance(0,0) + res.covariance(1,1)) * 100.0f; // m to cm
	const float sdf_median_cm = res.sdf_mse * 100.0f;  // Median SDF error in cm (already in meters)
	const float innovation_cm = res.innovation_norm * 100.0f;  // Innovation norm in cm

	lcdNumber_fps->display(fps_val);
	lcdNumber_loss->display(sdf_median_cm);
	lcdNumber_sigma->display(sigma_xy_cm);
	lcdNumber_velocity->display(std::abs(current_velocity.adv_y));
	lcdNumber_innov->display(innovation_cm);

	// Only update CPU and color stylesheets every 10 frames to save overhead
	static int ui_slow_counter = 0;
	if (++ui_slow_counter >= 10)
	{
		ui_slow_counter = 0;
		const float cpu_usage = get_cpu_usage();
		lcdNumber_cpu->display(static_cast<int>(cpu_usage));

		// Helper lambda: only call setStyleSheet when the color key changes
		static QString last_sigma_color, last_innov_color, last_cpu_color;

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
    static QGraphicsEllipseItem *gt_marker = nullptr;
    static QGraphicsLineItem   *gt_heading = nullptr;

    // If GT pose is not available (webots not connected), blank the displays and hide marker
    if (!gt_pose_opt.has_value())
    {
        lcdNumber_gt_xy_err->display(0.0);
        lcdNumber_gt_ang_err->display(0.0);
        lcdNumber_gt_xy_err->setStyleSheet("background-color: #888888;");  // Grey = no data
        lcdNumber_gt_ang_err->setStyleSheet("background-color: #888888;");
        if (gt_marker) gt_marker->setVisible(false);
        if (gt_heading) gt_heading->setVisible(false);
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

    // Draw GT robot ghost on the viewer for visual comparison
    constexpr float R = 0.25f;  // visual radius in meters (scene units)
    if (!gt_marker)
    {
        gt_marker = viewer->scene.addEllipse(-R, -R, 2*R, 2*R,
                                             QPen(QColor("magenta"), 0.02, Qt::DashLine));
        gt_marker->setZValue(50);
    }
    if (!gt_heading)
    {
        // Heading line along local X axis (same convention as robot_polygon via add_robot)
        gt_heading = viewer->scene.addLine(0, 0, 0, R,
                                           QPen(QColor("magenta"), 0.02));
        gt_heading->setZValue(50);
    }
    gt_marker->setVisible(true);
    gt_heading->setVisible(true);

    const float gx  = gt.translation().x();
    const float gy  = gt.translation().y();
    // Use the same angle convention as display_robot: qRadiansToDegrees(atan2(sin,cos))
    const float gan = qRadiansToDegrees(std::atan2(gt.linear()(1,0), gt.linear()(0,0)));
    gt_marker->setPos(gx, gy);
    gt_heading->setPos(gx, gy);
    gt_heading->setRotation(gan);
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
    if (!room_ai.is_initialized())
    {
        qWarning() << "Cannot calibrate GT: room_ai not initialized";
        return;
    }

    // Read current GT pose from buffer
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const auto &[robot_pose_gt_, lidar_local_] = buffer_sync.read(timestamp);
    if (!robot_pose_gt_.has_value())
    {
        qWarning() << "Cannot calibrate GT: no Webots pose available";
        return;
    }

    // Get the current estimated pose from room_ai
    const auto state = room_ai.get_current_state();
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

            // Get LiDAR data
            const auto data = lidar3d_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_HIGH,
                                                                   params.MAX_LIDAR_HIGH_RANGE*1000.f,  // Convert to mm
                                                                   params.LIDAR_LOW_DECIMATION_FACTOR);

            // Store local points in robot frame, filtering by height and body proximity
            std::vector<Eigen::Vector3f> points_local;
            points_local.reserve(data.points.size());
            const float body_offset_sq = params.ROBOT_SEMI_WIDTH * params.ROBOT_SEMI_WIDTH;
            for (const auto &p : data.points)
            {
                const float pmx = p.x / 1000.f;
                const float pmy = p.y / 1000.f;
                const float pmz = p.z / 1000.f;
                if (pmx*pmx + pmy*pmy > body_offset_sq &&
                    pmz > params.LIDAR_HIGH_MIN_HEIGHT &&
                    pmz < params.LIDAR_HIGH_MAX_HEIGHT)
                    points_local.emplace_back(pmx, pmy, pmz);
            }

            // Put lidar data in sync buffer
            buffer_sync.put<1>(std::move(std::make_pair(points_local, data.timestamp)), timestamp);

            // Adjust period with hysteresis
            if (wait_period > std::chrono::milliseconds((long) data.period + 2)) --wait_period;
            else if (wait_period < std::chrono::milliseconds((long) data.period - 2)) ++wait_period;
        }
        catch (const Ice::Exception &e)
        { std::cout << "Error reading from Lidar3D or robot pose: " << e.what() << std::endl; }
        std::this_thread::sleep_for(wait_period);
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

auto SpecificWorker::draw_lidar_points(const std::vector<Eigen::Vector3f> &points,
                                       const Eigen::Affine2f &robot_pose) -> void
{
    static std::vector<QGraphicsEllipseItem*> lidar_points;

    static QPen pen(QColor("Green"));
    pen.setWidthF(0.0);
    pen.setCosmetic(true);
    static const QBrush brush(QColor("Green"));

    // Size in *item* coordinates. With ItemIgnoresTransformations the size is in pixels.
    static constexpr qreal radius_px = 1.5;
    const QRectF ellipse_rect(-radius_px, -radius_px, 2*radius_px, 2*radius_px);

    const int stride = std::max(1, static_cast<int>(points.size() / params.MAX_LIDAR_DRAW_POINTS));
    const size_t num_points_to_draw = (points.size() + stride - 1) / stride;

    while (lidar_points.size() > num_points_to_draw)
    {
        auto *p = lidar_points.back();
        viewer->scene.removeItem(p);
        delete p;
        lidar_points.pop_back();
    }

    size_t idx = 0;
    for (size_t i = 0; i < points.size() && idx < num_points_to_draw; i += stride, ++idx)
    {
        // Draw in room frame. Points come in robot frame.
        // Use only XY for 2D viewer: p_room_xy = R(phi)*p_robot_xy + t
        const Eigen::Vector2f pr = points[i].head<2>();
        const Eigen::Vector2f pw = robot_pose.linear() * pr + robot_pose.translation();

        if (idx < lidar_points.size())
        {
            lidar_points[idx]->setPos(pw.x(), pw.y());
        }
        else
        {
            auto *item = viewer->scene.addEllipse(ellipse_rect, pen, brush);
            item->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
            item->setPos(pw.x(), pw.y());
            item->setZValue(5);
            lidar_points.push_back(item);
        }
    }
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
	velocity_history_.push_back(cmd);
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
    odometry_history_.push_back(odom);
}

////////////////////////////////////////////////////////////////////////////////////////////////
/// Subscription to emergencyState signal from Hibernation interface
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
}

void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
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
            // Pass them directly to room_ai
            room_ai.set_polygon_room(room_polygon_);

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

    // Update the room model with flipped polygon
    if (room_ai.is_initialized())
    {
        room_ai.set_polygon_room(room_polygon_);
    }

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

    // Update the room model with flipped polygon
    if (room_ai.is_initialized())
    {
        room_ai.set_polygon_room(room_polygon_);
    }

    // Redraw the polygon
    draw_room_polygon();

    qInfo() << "Room polygon flipped on Y axis (flip_y=" << flip_y_applied_ << ")";
}

void SpecificWorker::slot_robot_dragging(QPointF pos)
{
    // Don't move robot while capturing room polygon
    if (capturing_room_polygon)
        return;

    if (!room_ai.is_initialized())
        return;

    // Get current robot orientation to preserve it during drag
    const auto current_state = room_ai.get_current_state();
    const float current_theta = current_state[4];  // [half_w, half_h, x, y, theta]

    // Move robot to new position, keeping orientation
    room_ai.set_robot_pose(pos.x(), pos.y(), current_theta);
}

void SpecificWorker::slot_robot_drag_end(QPointF pos)
{
    // Don't move robot while capturing room polygon
    if (capturing_room_polygon)
        return;

    if (!room_ai.is_initialized())
        return;

    // Final position after drag
    const auto current_state = room_ai.get_current_state();
    const float current_theta = current_state[4];

    room_ai.set_robot_pose(pos.x(), pos.y(), current_theta);
    qInfo() << "Robot dragged to (" << pos.x() << "," << pos.y() << ")";
}

void SpecificWorker::slot_robot_rotate(QPointF pos)
{
    // Don't rotate robot while capturing room polygon
    if (capturing_room_polygon)
        return;

    if (!room_ai.is_initialized())
        return;

    // Get current robot position
    const auto current_state = room_ai.get_current_state();
    const float robot_x = current_state[2];
    const float robot_y = current_state[3];

    // Calculate angle from robot to cursor position
    const float dx = pos.x() - robot_x;
    const float dy = pos.y() - robot_y;
    const float new_theta = std::atan2(dy, dx);

    // Update robot orientation only, keep position
    room_ai.set_robot_pose(robot_x, robot_y, new_theta);
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
        room_ai.set_polygon_room(room_polygon_);
        draw_room_polygon();
        qInfo() << "Layout loaded and room_ai initialized with" << room_polygon_.size() << "vertices";
    }
}

void SpecificWorker::load_polygon_from_file(const std::string& filename)
{
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
        // Recursive lambda to walk the DOM tree
        std::function<void(const QDomElement&, std::array<float,6>)> walkElement;
        walkElement = [&](const QDomElement& elem, std::array<float,6> parentMat)
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
                        allPaths.push_back(std::move(pp));

                        qInfo() << "[SVG] Found closed path id=" << allPaths.back().id
                                << "vertices=" << allPaths.back().pts.size();
                    }
                }
            }
            else
            {
                QDomNode child = elem.firstChild();
                while (!child.isNull())
                {
                    if (child.isElement())
                        walkElement(child.toElement(), currentMat);
                    child = child.nextSibling();
                }
            }
        };

        std::array<float,6> identity = {1,0,0,1,0,0};
        walkElement(doc.documentElement(), identity);
    }

    if (allPaths.empty())
    {
        qWarning() << "[SVG] No closed paths found.";
        return;
    }

    qInfo() << "[SVG] Total closed paths found:" << allPaths.size();

    // Use the first closed path as the room polygon
    room_polygon_ = allPaths[0].pts;
    qInfo() << "[SVG] room_polygon_ set to path id='" << allPaths[0].id
            << "' with" << room_polygon_.size() << "vertices";
}

////////////////////////////////////////////////////////////////////////////////////////////////
/// Pose Persistence
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::save_last_pose()
{
    if (!room_ai.is_initialized())
        return;

    const auto state = room_ai.get_current_state();
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
    if (saved_flip_x)
    {
        for (auto& vertex : room_polygon_)
            vertex.x() = -vertex.x();
        flip_x_applied_ = true;

        if (room_ai.is_initialized())
            room_ai.set_polygon_room(room_polygon_);
        draw_room_polygon();
        qInfo() << "Applied saved flip_x";
    }
    if (saved_flip_y)
    {
        for (auto& vertex : room_polygon_)
            vertex.y() = -vertex.y();
        flip_y_applied_ = true;

        if (room_ai.is_initialized())
            room_ai.set_polygon_room(room_polygon_);
        draw_room_polygon();
        qInfo() << "Applied saved flip_y";
    }

    // Set the pose in room_ai
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

