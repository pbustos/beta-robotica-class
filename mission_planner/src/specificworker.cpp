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
    std::cout << "initialize worker" << std::endl;

	// Viewer
	viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM, true);
	auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
	robot_draw = r;
	show ();
}

void SpecificWorker::compute()
{
	// Get robot pose from Webots and return transform
	robot_pose = get_robot_pose();

	// Update visual representation of the robot
	robot_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	robot_draw->setRotation(qRadiansToDegrees((Eigen::Rotation2Df(robot_pose.linear()).angle())));

	// Draw lidar points
	const auto data = get_lidar();
	draw_lidar(data, robot_pose, &viewer->scene);

	// =========================================================================
	// MPPI Controller - descomentar para activar
	// =========================================================================
	// if (has_target_)
	// {
	//     // 1. Request path from Gridder (if needed)
	//     // try {
	//     //     auto gridder_path = gridder_proxy->getPath(
	//     //         robot_pose.translation().x(), robot_pose.translation().y(),
	//     //         current_target_.x(), current_target_.y());
	//     //     current_path_.clear();
	//     //     for (const auto& p : gridder_path)
	//     //         current_path_.emplace_back(p.x, p.y);
	//     // } catch (const Ice::Exception& e) {
	//     //     std::cerr << "Gridder error: " << e.what() << std::endl;
	//     // }
	//
	//     // 2. Convert lidar points to obstacle positions (world frame)
	//     // auto obstacles = lidar_to_obstacles(data);
	//
	//     // 3. Prepare current state for MPPI
	//     // MPPIController::State current_state;
	//     // current_state.x = robot_pose.translation().x();
	//     // current_state.y = robot_pose.translation().y();
	//     // current_state.theta = Eigen::Rotation2Df(robot_pose.linear()).angle();
	//
	//     // 4. Compute optimal control using MPPI
	//     // auto control = mppi_controller_.compute(current_state, current_path_, obstacles);
	//
	//     // 5. Send velocities to the robot
	//     // try {
	//     //     omnirobot_proxy->setSpeedBase(control.vx, control.vy, control.omega);
	//     // } catch (const Ice::Exception& e) {
	//     //     std::cerr << "OmniRobot error: " << e.what() << std::endl;
	//     // }
	//
	//     // 6. Draw MPPI optimal trajectory for visualization
	//     // draw_mppi_trajectory(mppi_controller_.getOptimalTrajectory(), &viewer->scene);
	//
	//     // 7. Check if goal reached
	//     // if (mppi_controller_.goalReached(current_state, current_path_.back()))
	//     // {
	//     //     has_target_ = false;
	//     //     omnirobot_proxy->stopBase();
	//     //     std::cout << "Goal reached!" << std::endl;
	//     // }
	// }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine2f SpecificWorker::get_robot_pose()
{
	RoboCompWebots2Robocomp::ObjectPose pose;
	Eigen::Affine2f robot_pose;
	try
	{
		pose = webots2robocomp_proxy->getObjectPose("shadow");
		//qInfo() << "Robot pose from Webots: x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z;
		robot_pose.translation() = Eigen::Vector2f(-pose.position.y, pose.position.x);
		const auto yaw = yawFromQuaternion(pose.orientation);
		robot_pose.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();
	}
	catch (const Ice::Exception &e){ std::cout<<e.what()<<std::endl; return {};}
	return robot_pose;
}

// Devuelve el yaw (rotación alrededor del eje Z) en radianes
double SpecificWorker::yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat)
{
	double w = quat.w;
	double x = quat.x;
	double y = quat.y;
	double z = quat.z;
	const auto norm = std::sqrt(w*w + x*x + y*y + z*z);
	w /= norm; x /= norm; y /= norm; z /= norm;
	return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points,
								const Eigen::Affine2f &robot_pose,
								QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	const QColor color("LightGreen");
	const QPen pen(color, 10);
	//const QBrush brush(color, Qt::SolidPattern);
	for (const auto &p : points)
	{
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
		Eigen::Vector2f worldP = robot_pose * Eigen::Vector2f(p.x, p.y);
		dp->setPos(worldP.x(), worldP.y());
		draw_points.push_back(dp);   // add to the list of points to be deleted next time
	}
}

Eigen::Affine2f SpecificWorker::update_robot_transform(const RoboCompWebots2Robocomp::ObjectPose &pose, Eigen::Affine2f &robot_pose)
{
	robot_pose.translation() = Eigen::Vector2f(pose.position.x, pose.position.z);
	auto yaw = yawFromQuaternion(pose.orientation);
	robot_pose.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();
	return robot_pose;
}



RoboCompLidar3D::TPoints SpecificWorker::get_lidar()
{
	RoboCompLidar3D::TData  data;
	try
	{
		data =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1); //para mayor precision (puedo comparar ejemplos de ejecucion entre este y 0.1f round en la docu)
		//qInfo() << "Size: "<<data.points.size();
	}
	catch (const Ice::Exception &e){ std::cout<<e.what()<<std::endl; return {};}
	return data.points;
}

////////////////////////////////////////////////////////////
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
// From the RoboCompCamera360RGB you can call this methods:
// RoboCompCamera360RGB::TImage this->camera360rgb_proxy->getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

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

// =========================================================================
// MPPI Helper Functions - descomentar para activar
// =========================================================================

// /**
//  * @brief Convert lidar points to obstacle positions in world frame
//  * @param points Lidar points in robot frame
//  * @return Vector of obstacle positions in world frame
//  */
// std::vector<Eigen::Vector2f> SpecificWorker::lidar_to_obstacles(const RoboCompLidar3D::TPoints& points)
// {
//     std::vector<Eigen::Vector2f> obstacles;
//     obstacles.reserve(points.size());
//     for (const auto& p : points)
//     {
//         // Transform from robot frame to world frame
//         Eigen::Vector2f local_point(p.x, p.y);
//         Eigen::Vector2f world_point = robot_pose * local_point;
//         obstacles.push_back(world_point);
//     }
//     return obstacles;
// }

// /**
//  * @brief Draw the MPPI optimal trajectory for visualization
//  * @param trajectory Optimal trajectory from MPPI
//  * @param scene Graphics scene to draw on
//  */
// void SpecificWorker::draw_mppi_trajectory(const std::vector<MPPIController::State>& trajectory,
//                                           QGraphicsScene* scene)
// {
//     static std::vector<QGraphicsItem*> trajectory_items;
//
//     // Clear previous trajectory
//     for (auto* item : trajectory_items)
//     {
//         scene->removeItem(item);
//         delete item;
//     }
//     trajectory_items.clear();
//
//     // Draw new trajectory
//     const QColor color("Orange");
//     const QPen pen(color, 3);
//
//     for (size_t i = 0; i < trajectory.size() - 1; ++i)
//     {
//         auto* line = scene->addLine(
//             trajectory[i].x, trajectory[i].y,
//             trajectory[i+1].x, trajectory[i+1].y,
//             pen);
//         trajectory_items.push_back(line);
//     }
//
//     // Draw points at each state
//     const QPen point_pen(Qt::red, 8);
//     for (const auto& state : trajectory)
//     {
//         auto* point = scene->addEllipse(state.x - 10, state.y - 10, 20, 20, point_pen);
//         trajectory_items.push_back(point);
//     }
// }


