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
#include <unistd.h>  // For sysconf

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

    // Connect mouse clicks on the scene for polygon capture
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::on_scene_clicked);

    // Connect save/load layout buttons
    connect(pushButton_saveLayout, &QPushButton::clicked, this, &SpecificWorker::slot_save_layout);
    connect(pushButton_loadLayout, &QPushButton::clicked, this, &SpecificWorker::slot_load_layout);

    // Try to load default layout on startup (only loads vertices, doesn't init room_ai yet)
    load_polygon_from_file("etc/room_layout.json");

    // Lidar thread is created
    read_lidar_th = std::thread(&SpecificWorker::read_lidar,this);
    std::cout << __FUNCTION__ << " Started lidar reader" << std::endl;

    // Wait for first robot pose + lidar so we can initialize the concept with a data-driven guess
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
    }while (++startup_check_counter < 20 and (not robot.has_value() or not lidar_local.has_value()));

    if (not robot.has_value() or not lidar_local.has_value())
    {
        qWarning() << "initialize(): No data from buffer_sync. Exiting.";
        std::terminate();
    }

    // Center view in (0,0) since GT room is centered there
    const float view_side_m = 6.f;
    viewer->fitToScene(QRectF(- view_side_m/2.f, - view_side_m/2.f, view_side_m, view_side_m));
    viewer->centerOn(0, 0);

    // Room dimensions for initial rectangular model (will be replaced if polygon is captured)
    room_rect_gt = QRectF(-7, -4, 14, 8);  // 14m x 8m area centered on origin
    // Don't draw rectangle here - will be drawn by draw_estimated_room or draw_room_polygon

    const float room_width = room_rect_gt.width();
    const float room_length = room_rect_gt.height();

    // Estimate room center from lidar (robot frame) and derive robot pose wrt room center
    const auto &pts = lidar_local.value().first;
    RoboCompLidar3D::TPoints lidar_points;
    lidar_points.reserve(pts.size());
    for(const auto &p : pts)
    {
        RoboCompLidar3D::TPoint tp;	// TODO: change RoomConceptAI to use Eigen::Vector3f directly and avoid this copy
        tp.x = p.x();
        tp.y = p.y();
        tp.z = p.z();
        lidar_points.push_back(tp);
    }

    rc::PointcloudCenterEstimator estimator;
    Eigen::Vector2d room_center_in_robot = Eigen::Vector2d::Zero();
    float init_phi = 0.f;
    if(const auto obb = estimator.estimate_obb(lidar_points); obb.has_value())
    {
        room_center_in_robot = obb->center;
        init_phi = static_cast<float>(obb->rotation);
    }
    else if(const auto c = estimator.estimate(lidar_points); c.has_value())
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
    if (!room_polygon_gt.empty())
    {
        // Use pre-loaded polygon from file
        room_ai.set_polygon_room(room_polygon_gt);
        draw_room_polygon();
        qInfo() << "RoomConceptAI initialized with loaded polygon:" << room_polygon_gt.size() << "vertices";
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
    if (not lidar_local_.has_value() or not robot_pose_gt_.has_value())
    { qWarning() << "No data from buffer_sync: robot has value?"; return; };
    const auto &lidar_local = lidar_local_.value().first;

    if(room_ai.is_initialized())
    {
        // Pass velocity and dt to update
        if(const auto res = room_ai.update(lidar_local_.value(), velocity_history_); res.ok)
        {
			update_ui(res, velocity_history_.back(), fps_val);
            display_robot(res.robot_pose, res.covariance);
            draw_lidar_points(lidar_local, res.robot_pose);

            // Draw room rectangle (only if not using polygon)
            draw_estimated_room(res.state);
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

void SpecificWorker::update_ui(const rc::RoomConceptAI::UpdateResult &res,
							   const rc::VelocityCommand &current_velocity,
							   int fps_val)
{
	// Update QLCDNumber displays
	const float sigma_xy_cm = std::sqrt(res.covariance(0,0) + res.covariance(1,1)) * 100.0f; // m to cm
	const float sdf_median_cm = res.sdf_mse * 100.0f;  // Median SDF error in cm (already in meters)
	const float innovation_cm = res.innovation_norm * 100.0f;  // Innovation norm in cm
	const float cpu_usage = get_cpu_usage();

	lcdNumber_fps->display(fps_val);
	lcdNumber_loss->display(sdf_median_cm);  // Median SDF error in cm
	lcdNumber_sigma->display(sigma_xy_cm);
	lcdNumber_velocity->display(std::abs(current_velocity.adv_y));
	lcdNumber_innov->display(innovation_cm);  // Innovation as health indicator
	lcdNumber_cpu->display(static_cast<int>(cpu_usage));

	// Color sigma based on absolute uncertainty values
	// Green: < 5cm, Yellow: 5-10cm, Orange: 10-20cm, Red: > 20cm
	QString sigma_color;
	if (sigma_xy_cm < 5.0f)
		sigma_color = "background-color: #90EE90;";  // Light green - very low uncertainty
	else if (sigma_xy_cm < 10.0f)
		sigma_color = "background-color: #FFFF00;";  // Yellow - moderate uncertainty
	else if (sigma_xy_cm < 20.0f)
		sigma_color = "background-color: #FFA500;";  // Orange - high uncertainty
	else
		sigma_color = "background-color: #FF6B6B;";  // Red - very high uncertainty
	lcdNumber_sigma->setStyleSheet(sigma_color);

	// Color the innovation display based on health
	// Green: < 5cm, Yellow: 5-15cm, Red: > 15cm
	QString innov_color;
	if (innovation_cm < 5.0f)
		innov_color = "background-color: #90EE90;";  // Light green
	else if (innovation_cm < 15.0f)
		innov_color = "background-color: #FFFF00;";  // Yellow
	else
		innov_color = "background-color: #FF6B6B;";  // Light red
	lcdNumber_innov->setStyleSheet(innov_color);

	// Color CPU based on usage
	QString cpu_color;
	if (cpu_usage < 30.0f)
		cpu_color = "background-color: #90EE90;";  // Light green
	else if (cpu_usage < 60.0f)
		cpu_color = "background-color: #FFFF00;";  // Yellow
	else
		cpu_color = "background-color: #FF6B6B;";  // Red
	lcdNumber_cpu->setStyleSheet(cpu_color);
}

void SpecificWorker::display_robot(const Eigen::Affine2f &robot_pose, const Eigen::Matrix3f &covariance)
{
	// ============ Update robot visualization in the viewer
	const float display_x = robot_pose.translation().x();
	const float display_y = robot_pose.translation().y();
	const float display_angle = std::atan2(robot_pose.linear()(1,0), robot_pose.linear()(0,0));
	viewer->robot_poly()->setPos(display_x, display_y);
	viewer->robot_poly()->setRotation(qRadiansToDegrees(display_angle));

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

void SpecificWorker::read_lidar()
{
    auto wait_period = std::chrono::milliseconds (getPeriod("Compute"));
    while(!stop_lidar_thread)
    {
        const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        try
        {
            // Get robot pose
            const auto &[position, orientation] = webots2robocomp_proxy->getObjectPose("shadow");
            Eigen::Affine2f eig_pose;
            eig_pose.translation() = Eigen::Vector2f(-position.y/1000.f, position.x/1000.f);
            eig_pose.linear() = Eigen::Rotation2Df(yawFromQuaternion(orientation)).toRotationMatrix();

            // Get LiDAR data
            const auto data = lidar3d_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_HIGH,
                                                                   params.MAX_LIDAR_HIGH_RANGE*1000.f,  // Convert to mm
                                                                   params.LIDAR_LOW_DECIMATION_FACTOR);

            // // Store local points (original LiDAR frame) and transform to world frame
            std::vector<Eigen::Vector3f> points_local;
            points_local.reserve(data.points.size());
            const float low_height_offset = params.LIDAR_HIGH_MIN_HEIGHT*params.LIDAR_HIGH_MIN_HEIGHT;
            const float high_height_offset = (params.LIDAR_HIGH_MAX_HEIGHT * params.LIDAR_HIGH_MAX_HEIGHT);
            const float body_offset = params.ROBOT_SEMI_WIDTH * params.ROBOT_SEMI_WIDTH; // Filter points that are too close to the robot's center (e.g. points on the robot itself)
            for (const auto &p : data.points)
            {
                const float pmx = p.x / 1000.f;
                const float pmy = p.y / 1000.f;
                const float pmz = p.z / 1000.f;
                if (pmx*pmx + pmy*pmy > body_offset and
                    pmz > low_height_offset and
                    pmz < high_height_offset) // Filter points below min height (e.g. to ignore tables)
                {
                    points_local.emplace_back(pmx, pmy, pmz);
                }
            }

            // Put all in sync buffer with same timestamp
        	buffer_sync.put<0>(std::move(eig_pose), timestamp);
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
    if (!room_polygon_gt.empty())
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
	// Parse velocity command (assuming axes[0]=adv_x, axes[1]=adv_z, axes[2]=rot)
	rc::VelocityCommand cmd;
	// Parse velocity command (assuming axes[0]=adv_x, axes[1]=adv_z, axes[2]=rot)
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

////////////////////////////////////////////////////////////////////////////////////////////////
/// Subscription to emergencyState signal from Hibernation interface
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}

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

////////////////////////////////////////////////////////////////////////////////////////////////
/// Room Polygon Capture
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::slot_capture_room_toggled(bool checked)
{
    capturing_room_polygon = checked;

    if (checked)
    {
        // Start capturing - clear previous polygon
        room_polygon_gt.clear();

        // Clear previous vertex markers
        for (auto* item : polygon_vertex_items)
        {
            viewer->scene.removeItem(item);
            delete item;
        }
        polygon_vertex_items.clear();

        // Clear previous polygon item
        if (polygon_item)
        {
            viewer->scene.removeItem(polygon_item);
            delete polygon_item;
            polygon_item = nullptr;
        }

        pushButton_captureRoom->setText("Click vertices...");
        qInfo() << "Room capture started. Click on the scene to add vertices. Click near the first point to close.";
    }
    else
    {
        pushButton_captureRoom->setText("Capture Room");

        if (room_polygon_gt.size() >= 3)
        {
            qInfo() << "Room polygon captured with" << room_polygon_gt.size() << "vertices";

            // Clear vertex markers (yellow circles)
            for (auto* item : polygon_vertex_items)
            {
                viewer->scene.removeItem(item);
                delete item;
            }
            polygon_vertex_items.clear();

            // Vertices are already in room frame (where user clicked)
            // Pass them directly to room_ai
            room_ai.set_polygon_room(room_polygon_gt);

            // Draw final polygon (fixed in room frame)
            draw_room_polygon();
        }
        else
        {
            qWarning() << "Need at least 3 vertices for a valid polygon";
        }
    }
}

void SpecificWorker::on_scene_clicked(QPointF pos)
{
    if (!capturing_room_polygon)
        return;

    // The scene shows points in room frame (transformed by robot_pose)
    // We need to store vertices in room frame (where they appear on screen)
    const Eigen::Vector2f click_pos(pos.x(), pos.y());

    // Check if clicking near the first point to close the polygon
    constexpr float close_threshold = 0.3f;  // 30cm
    if (room_polygon_gt.size() >= 3)
    {
        const float dist_to_first = (click_pos - room_polygon_gt.front()).norm();
        if (dist_to_first < close_threshold)
        {
            // Close the polygon
            pushButton_captureRoom->setChecked(false);  // This triggers slot_capture_room_toggled(false)
            return;
        }
    }

    // Add new vertex in room frame coordinates
    room_polygon_gt.push_back(click_pos);

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

    qInfo() << "Added vertex" << room_polygon_gt.size() << "at (" << pos.x() << "," << pos.y() << ")";
}

void SpecificWorker::draw_room_polygon()
{
    if (room_polygon_gt.size() < 2)
        return;

    // Remove old polygon item
    if (polygon_item)
    {
        viewer->scene.removeItem(polygon_item);
        delete polygon_item;
        polygon_item = nullptr;
    }

    // Create QPolygonF from vertices - polygon is FIXED in room frame (where user clicked)
    QPolygonF poly;
    for (const auto& v : room_polygon_gt)
    {
        poly << QPointF(v.x(), v.y());
    }

    // Close the polygon if capture is complete
    if (!capturing_room_polygon && room_polygon_gt.size() >= 3)
    {
        poly << QPointF(room_polygon_gt.front().x(), room_polygon_gt.front().y());
    }

    // Draw polygon - stays fixed in room frame
    QPen pen(capturing_room_polygon ? Qt::yellow : Qt::magenta, 0.08);
    polygon_item = viewer->scene.addPolygon(poly, pen, QBrush(Qt::NoBrush));
    polygon_item->setZValue(8);
}

////////////////////////////////////////////////////////////////////////////////////////////////
/// Layout Save/Load
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::slot_save_layout()
{
    if (room_polygon_gt.empty())
    {
        qWarning() << "No polygon to save - capture a room first";
        return;
    }

    // Use native dialog with explicit options to avoid freezing
    QString filename = QFileDialog::getSaveFileName(this,
        "Save Room Layout",
        "etc/room_layout.json",
        "JSON Files (*.json)",
        nullptr,
        QFileDialog::DontUseNativeDialog);

    if (filename.isEmpty())
        return;

    save_layout_to_file(filename.toStdString());
}

void SpecificWorker::slot_load_layout()
{
    QString filename = QFileDialog::getOpenFileName(this,
        "Load Room Layout",
        "etc/",
        "JSON Files (*.json)",
        nullptr,
        QFileDialog::DontUseNativeDialog);

    if (filename.isEmpty())
        return;

    load_layout_from_file(filename.toStdString());
}

void SpecificWorker::save_layout_to_file(const std::string& filename)
{
    if (room_polygon_gt.empty())
    {
        qWarning() << "No polygon to save";
        return;
    }

    QJsonObject root;
    root["type"] = "polygon";
    root["version"] = 1;

    QJsonArray vertices;
    for (const auto& v : room_polygon_gt)
    {
        QJsonObject vertex;
        vertex["x"] = static_cast<double>(v.x());
        vertex["y"] = static_cast<double>(v.y());
        vertices.append(vertex);
    }
    root["vertices"] = vertices;

    // Save room dimensions for reference
    root["room_width"] = static_cast<double>(room_rect_gt.width());
    root["room_height"] = static_cast<double>(room_rect_gt.height());

    QJsonDocument doc(root);
    QFile file(QString::fromStdString(filename));
    if (file.open(QIODevice::WriteOnly))
    {
        file.write(doc.toJson());
        file.close();
        qInfo() << "Layout saved to" << QString::fromStdString(filename);
    }
    else
    {
        qWarning() << "Failed to save layout to" << QString::fromStdString(filename);
    }
}

void SpecificWorker::load_layout_from_file(const std::string& filename)
{
    load_polygon_from_file(filename);

    // If polygon was loaded, initialize room_ai
    if (room_polygon_gt.size() >= 3)
    {
        room_ai.set_polygon_room(room_polygon_gt);
        draw_room_polygon();
        qInfo() << "Layout loaded and room_ai initialized with" << room_polygon_gt.size() << "vertices";
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

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        qWarning() << "Failed to parse layout file:" << parseError.errorString();
        return;
    }

    QJsonObject root = doc.object();
    if (root["type"].toString() != "polygon")
    {
        qWarning() << "Unknown layout type";
        return;
    }

    // Clear previous polygon
    room_polygon_gt.clear();
    for (auto* item : polygon_vertex_items)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    polygon_vertex_items.clear();

    // Load vertices
    QJsonArray vertices = root["vertices"].toArray();
    for (const auto& v : vertices)
    {
        QJsonObject vertex = v.toObject();
        float x = static_cast<float>(vertex["x"].toDouble());
        float y = static_cast<float>(vertex["y"].toDouble());
        room_polygon_gt.emplace_back(x, y);
    }

    qInfo() << "Polygon loaded from" << QString::fromStdString(filename)
            << "with" << room_polygon_gt.size() << "vertices";
}


