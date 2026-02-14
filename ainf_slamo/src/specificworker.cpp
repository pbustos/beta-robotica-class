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

    // Viewer
    viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM, true);
    viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0.0, 0.2, QColor("Blue"));
    viewer->show();

    // Grab status label from UI if present
    status_label = this->findChild<QLabel*>("status_label");

    // Lidar thread is created
    read_lidar_th = std::thread(&SpecificWorker::read_lidar,this);
    std::cout << __FUNCTION__ << " Started lidar reader" << std::endl;

    // Wait for first robot pose + lidar so we can initialize the concept with a data-driven guess
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    int startup_check_counter = 0;
    std::optional<Eigen::Affine2f> robot;
    std::optional<std::vector<Eigen::Vector3f>> lidar_local;
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

    // set GT room dimensions
    room_rect_gt = QRectF(-7, -4, 14, 8);  // 20m x 10m area centered on origin
    viewer->scene.addRect(room_rect_gt, QPen(Qt::red, 0.1), QBrush(Qt::NoBrush));

    const float room_width = room_rect_gt.width();
    const float room_length = room_rect_gt.height();

    // Estimate room center from lidar (robot frame) and derive robot pose wrt room center
    const auto &pts = lidar_local.value();
    RoboCompLidar3D::TPoints lidar_points;
    lidar_points.reserve(pts.size());
    for(const auto &p : pts)
    {
        RoboCompLidar3D::TPoint tp;
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

    room_ai.set_initial_state(room_width, room_length, init_xy.x(), init_xy.y(), init_phi);

    qInfo() << "RoomConceptAI init state [w,l,x,y,phi]="
            << room_width << room_length << init_xy.x() << init_xy.y() << init_phi
            << "  (room_center_in_robot=" << room_center_in_robot.x() << room_center_in_robot.y() << ")";
}


void SpecificWorker::compute()
{
    const int fps_val = fps.print("Compute", 2000);
    const auto timestamp = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const auto &[robot_pose_gt_, lidar_local_] = buffer_sync.read(timestamp);
    if (not lidar_local_.has_value() or not robot_pose_gt_.has_value())
    { qWarning() << "No data from buffer_sync: robot has value?"; return; };
    const auto &lidar_local = lidar_local_.value();

    if(room_ai.is_initialized())
    {
        RoboCompLidar3D::TPoints lidar_points;
        lidar_points.reserve(lidar_local.size());
        for(const auto &p : lidar_local)
        {
            RoboCompLidar3D::TPoint tp;
            tp.x = p.x();
            tp.y = p.y();
            tp.z = p.z();
            lidar_points.push_back(tp);
        }
        const auto res = room_ai.update(lidar_points);
        if(res.ok)
        {
            if(status_label != nullptr)
            {
                status_label->setText(QString("FPS: %1   SDF: %2")
                                          .arg(fps_val)
                                          .arg(res.final_loss, 0, 'g', 4));
            }

            display_robot(res.robot_pose);
            draw_lidar_points(lidar_local, res.robot_pose);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::display_robot(const Eigen::Affine2f &robot_pose_gt)
{
	// ============ Update robot visualization in the viewer using GT pose
	const float display_x = robot_pose_gt.translation().x();
	const float display_y = robot_pose_gt.translation().y();
	const float display_angle = std::atan2(robot_pose_gt.linear()(1,0), robot_pose_gt.linear()(0,0));
	viewer->robot_poly()->setPos(display_x, display_y);
	viewer->robot_poly()->setRotation(qRadiansToDegrees(display_angle));
}

void SpecificWorker::read_lidar()
{
    auto wait_period = std::chrono::milliseconds (getPeriod("Compute"));
    while(!stop_lidar_thread)
    {
        const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        try
        {
            // Get robot pose
            const auto &[position, orientation] = webots2robocomp_proxy->getObjectPose("shadow");
            Eigen::Affine2f eig_pose;
            eig_pose.translation() = Eigen::Vector2f(-position.y/1000.f, position.x/1000.f);
            eig_pose.linear() = Eigen::Rotation2Df(yawFromQuaternion(orientation)).toRotationMatrix();

            // Get LiDAR data
            auto data = lidar3d_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_HIGH,
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
	            if (pmx*pmx + pmy*pmy > body_offset and pmz > low_height_offset and pmz < high_height_offset) // Filter points below min height (e.g. to ignore tables)
	            {
	            	points_local.emplace_back(pmx, pmy, pmz);
	            }
            }

            // Put all in sync buffer with same timestamp
        	buffer_sync.put<0>(std::move(eig_pose), timestamp);
            buffer_sync.put<1>(std::move(points_local), timestamp);

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

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
	const Eigen::Vector2f v = es.eigenvectors().col(1);  // largest eigenvalue
	return std::atan2(v.y(), v.x());
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
////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
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

/**************************************/
// From the RoboCompWebots2Robocomp you can call this methods:
// RoboCompWebots2Robocomp::ObjectPose this->webots2robocomp_proxy->getObjectPose(string DEF)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->resetWebots()
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setDoorAngle(float angle)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setPathToHuman(int humanId, RoboCompGridder::TPath path)

/**************************************/
// From the RoboCompWebots2Robocomp you can use this types:
// RoboCompWebots2Robocomp::Vector3
// RoboCompWebots2Robocomp::Quaternion
// RoboCompWebots2Robocomp::ObjectPose

