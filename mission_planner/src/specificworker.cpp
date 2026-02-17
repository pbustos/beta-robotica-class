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

#include <Ice/Exception.h>

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
	viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 0.2, QColor("Blue"));
	viewer->show();

	// Request map and initialize grid
	try
	{ const auto pose = gridder_proxy->getPose();
	  const QRectF initial_view(pose.x - 3000, pose.y - 3000, 6000, 6000);  // 6m x 6m area centered on robot pose
	  viewer->fitToScene(initial_view);
	}
	catch (const Ice::Exception& ex){ std::cout << "Error connecting to gridder proxy: " << ex.what() << std::endl; return;}
	initialize_grid();

	// Connects
	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::left_click_handler);

}

void SpecificWorker::compute()
{
	// get robot_pose from gridder
	try
	{
		const auto robot_pose = gridder_proxy->getPose();

		// Update robot position variables
		robot_x = robot_pose.x;
		robot_y = robot_pose.y;
		robot_theta = robot_pose.theta;


		// Update UI labels
		update_ui();

	}
	catch (const Ice::Exception& ex){std::cout << "Error getting robot pose from gridder: " << ex.what() << std::endl; return;}

}

/////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::update_ui()
{
	viewer->robot_poly()->setPos(robot_x, robot_y);
	viewer->robot_poly()->setRotation(qRadiansToDegrees(robot_theta));
	labelX->setText(QString::asprintf("X: %.2f", robot_x));
	labelY->setText(QString::asprintf("Y: %.2f", robot_y));
	labelTheta->setText(QString::asprintf("Theta: %.2f", robot_theta));
}

void SpecificWorker::initialize_grid()
{
	// get map from gridder
	try
	{
		map = gridder_proxy->getMap();
		std::cout << "Received map from gridder with " << map.cells.size() << " cells " << map.tileSize << " tilesize" << std::endl;
		std::string map_file = "mapa_webots.gridmap";
		params.TILE_SIZE = static_cast<float>(map.tileSize);
		grid_esdf.initialize(map.tileSize, &viewer->scene);

	}
	catch (const Ice::Exception& ex){std::cout << "Error getting map from gridder: " << ex.what() << std::endl; return;}

	// Convert MRPT cells to sparse grid format
    // Apply rotation and offset to align MRPT map with Webots world coordinates
    const float offset_x = params.MRPT_MAP_OFFSET_X;
    const float offset_y = params.MRPT_MAP_OFFSET_Y;
    const float rotation = params.MRPT_MAP_ROTATION;
    const float cos_r = std::cos(rotation);
    const float sin_r = std::sin(rotation);
    const bool mirror_x = params.MRPT_MAP_MIRROR_X;

    if (offset_x != 0.f || offset_y != 0.f || rotation != 0.f || mirror_x)
        qInfo() << "[MRPT Loader] Applying transform: rotation=" << rotation << "rad, offset=(" << offset_x << "," << offset_y << ")mm, mirror_x=" << mirror_x;

    // Track unique keys to detect collisions (multiple MRPT cells mapping to same grid cell)
    std::unordered_set<GridESDF::Key, boost::hash<GridESDF::Key>> unique_keys;
    size_t cells_processed = 0;
    size_t cells_added = 0;
    size_t cells_collided = 0;

    for (const auto &mrpt_cell : map.cells)
    {
        // All cells from loader already have occupancy > 0.78 (filtered by cell_value > 200)
        cells_processed++;

        // Apply mirror if needed (negate X before rotation)
        float orig_x = static_cast<float>(mrpt_cell.x);
        const float orig_y = static_cast<float>(mrpt_cell.y);
        if (mirror_x)
            orig_x = -orig_x;

        // First rotate around origin, then apply offset
        const float cell_x = orig_x * cos_r - orig_y * sin_r + offset_x;
        const float cell_y = orig_x * sin_r + orig_y * cos_r + offset_y;
        const auto key = grid_esdf.point_to_key(Eigen::Vector2f(cell_x, cell_y));

        // Check if this key already exists (collision detection)
        if (unique_keys.find(key) == unique_keys.end())
        {
            unique_keys.insert(key);
            grid_esdf.add_confirmed_obstacle(key);
            cells_added++;
        }
        else
        {
            cells_collided++;  // Multiple MRPT cells map to same grid cell
        }
    }

    // Mark dirty once after loading all obstacles, then update visualization
    grid_esdf.mark_visualization_dirty();
    grid_esdf.update_visualization(true);

    // Report statistics
    qInfo() << "[MRPT Loader] SPARSE_ESDF Statistics:";
    qInfo() << "  - MRPT cells processed:" << cells_processed;
    qInfo() << "  - Grid cells added:" << cells_added;
    qInfo() << "  - Collisions (MRPT->same grid cell):" << cells_collided;
    qInfo() << "  - Final grid obstacles:" << grid_esdf.num_obstacles();
    if (cells_collided > 0)
    {
        float collision_pct = 100.f * cells_collided / cells_processed;
        qWarning() << "[MRPT Loader] WARNING:" << collision_pct << "% cells lost due to resolution mismatch!";
    }
}

////////////////  MOUSE EVENTS  ////////////////////////////////
void SpecificWorker::left_click_handler(QPointF p)
{
    qInfo() << "[MOUSE] New global target arrived:" << p;

    // Use estimated robot position (from localizer) as source for path planning

    const Eigen::Vector2f source{robot_x, robot_y};
	const Eigen::Vector2f target{p.x(), p.y()};

    const auto start_time = std::chrono::high_resolution_clock::now();
	auto path = grid_esdf.compute_path(source, target, params.ROBOT_SEMI_WIDTH, params.SAFETY_FACTOR);
    qInfo() << "A* path computed (ESDF), path size:" << path.size();

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    float elapsed_sec = static_cast<float>(elapsed_ms) / 1000.f;

    if (!path.empty())
    {
        // Calculate path length (sum of segment distances)
        float path_length = 0.f;
        for (size_t i = 1; i < path.size(); ++i)
            path_length += (path[i] - path[i-1]).norm();

        // Update UI displays
        lcdNumber_length->display(static_cast<double>(path_length / 1000.f));  // Show in meters
        lcdNumber_elapsed->display(static_cast<double>(elapsed_sec));  // Show in seconds

        qInfo() << "Path found with" << path.size() << "waypoints, length:"
                << path_length/1000.f << ", time:" << elapsed_sec << "s";
        draw_path(path, &viewer->scene);

        // Draw target marker (solid circle)
        static QGraphicsEllipseItem* target_marker = nullptr;
        if (target_marker)
        {
            viewer->scene.removeItem(target_marker);
            delete target_marker;
        }
        const float marker_size = 200.f;  // mm
        target_marker = viewer->scene.addEllipse(
            -marker_size/2, -marker_size/2, marker_size, marker_size,
            QPen(QColor("Red"), 30),
            QBrush(QColor(255, 0, 0, 150)));  // Semi-transparent red fill
        target_marker->setPos(target.x(), target.y());
        target_marker->setZValue(20);  // On top of everything
    }
    else
    {
        // Clear UI on no path
        lcdNumber_length->display(0);
        lcdNumber_elapsed->display(static_cast<double>(elapsed_sec));  // Show in seconds
        qInfo() << "No path found!";
    }
};

void SpecificWorker::draw_path(const std::vector<Eigen::Vector2f> &path, QGraphicsScene *scene, bool erase_only)
{
	static std::vector<QGraphicsEllipseItem*> points;
	static const QColor color("blue");
	static const QPen pen(color);
	static const QBrush brush(color);
	static constexpr float s = 50;  // Smaller dots for thinner path

	for(auto p : points)
	{
		scene->removeItem(p);
		delete p;
	}
	points.clear();

	if(erase_only) return;

	points.reserve(path.size());
	for(const auto &p: path)
	{
		auto ptr = scene->addEllipse(-s/2, -s/2, s, s, pen, brush);
		ptr->setPos(p.x(), p.y());
		ptr->setZValue(10);
		points.push_back(ptr);
	}
}

/////////////////////////////// EMERGENCY AND RESTORE ////////////////////////////////
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
// From the RoboCompGridder you can call this methods:
// RoboCompGridder::bool this->gridder_proxy->IsPathBlocked(TPath path)
// RoboCompGridder::bool this->gridder_proxy->LineOfSightToTarget(TPoint source, TPoint target, float robotRadius)
// RoboCompGridder::void this->gridder_proxy->cancelNavigation()
// RoboCompGridder::TPoint this->gridder_proxy->getClosestFreePoint(TPoint source)
// RoboCompGridder::TDimensions this->gridder_proxy->getDimensions()
// RoboCompGridder::float this->gridder_proxy->getDistanceToTarget()
// RoboCompGridder::float this->gridder_proxy->getEstimatedTimeToTarget()
// RoboCompGridder::Map this->gridder_proxy->getMap()
// RoboCompGridder::NavigationState this->gridder_proxy->getNavigationState()
// RoboCompGridder::NavigationStatus this->gridder_proxy->getNavigationStatus()
// RoboCompGridder::Result this->gridder_proxy->getPaths(TPoint source, TPoint target, int maxPaths, bool tryClosestFreePoint, bool targetIsHuman, float safetyFactor)
// RoboCompGridder::Pose this->gridder_proxy->getPose()
// RoboCompGridder::TPoint this->gridder_proxy->getTarget()
// RoboCompGridder::bool this->gridder_proxy->hasReachedTarget()
// RoboCompGridder::bool this->gridder_proxy->replanPath()
// RoboCompGridder::bool this->gridder_proxy->resumeNavigation()
// RoboCompGridder::bool this->gridder_proxy->setGridDimensions(TDimensions dimensions)
// RoboCompGridder::Result this->gridder_proxy->setLocationAndGetPath(TPoint source, TPoint target, TPointVector freePoints, TPointVector obstaclePoints)
// RoboCompGridder::bool this->gridder_proxy->setTarget(TPoint target)
// RoboCompGridder::bool this->gridder_proxy->setTargetWithOptions(TPoint target, NavigationOptions options)
// RoboCompGridder::bool this->gridder_proxy->startNavigation()
// RoboCompGridder::void this->gridder_proxy->stopNavigation()

/**************************************/
// From the RoboCompGridder you can use this types:
// RoboCompGridder::TPoint
// RoboCompGridder::TDimensions
// RoboCompGridder::Result
// RoboCompGridder::TCell
// RoboCompGridder::Map
// RoboCompGridder::Pose
// RoboCompGridder::NavigationOptions
// RoboCompGridder::NavigationStatus

