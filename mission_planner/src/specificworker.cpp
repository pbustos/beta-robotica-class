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
#include <limits>

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
	{ const auto pose = navigator_proxy->getRobotPose();
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
		const auto robot_pose = navigator_proxy->getRobotPose();

		// Update robot position variables
		robot_x = robot_pose.x;
		robot_y = robot_pose.y;
		robot_theta = robot_pose.r;


		// Update UI labels
		update_ui();

	}
	catch (const Ice::Exception& ex){std::cout << "Error getting robot pose from gridder: " << ex.what() << std::endl; return;}

}

/////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::update_ui()
{
    if (viewer == nullptr)
    {
        qWarning() << "[update_ui] Viewer is null. Skipping UI update.";
        return;
    }

    if (auto *robot_item = viewer->robot_poly(); robot_item != nullptr)
    {
        robot_item->setPos(robot_x, robot_y);
        robot_item->setRotation(qRadiansToDegrees(robot_theta));
    }
    else
    {
        qWarning() << "[update_ui] Robot graphics item is null. Skipping robot pose draw.";
    }

    if (labelX != nullptr)
        labelX->setText(QString::asprintf("X: %.2f", robot_x));
    if (labelY != nullptr)
        labelY->setText(QString::asprintf("Y: %.2f", robot_y));
    if (labelTheta != nullptr)
        labelTheta->setText(QString::asprintf("Theta: %.2f", robot_theta));
}

void SpecificWorker::initialize_grid()
{
    try
    {
        map = navigator_proxy->getLayout();
    }
    catch (const Ice::Exception& ex)
    {
        std::cout << "Error getting layout from navigator: " << ex.what() << std::endl;
        return;
    }

    // Keep handles to map drawings so a refresh deletes previous map items.
    static std::vector<QGraphicsItem *> map_items;
    for (auto *item : map_items)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    map_items.clear();

    if (map.layout.empty() && map.objects.empty())
    {
        qWarning() << "[initialize_grid] Empty LayoutData received.";
        return;
    }

    // Detect coordinate scale from incoming layout: if values are small, assume meters and convert to mm.
    float max_abs_input = 0.f;
    size_t finite_input_points = 0;
    auto collect_input_stats = [&](float x, float y)
    {
        if (not std::isfinite(x) || not std::isfinite(y))
            return;
        max_abs_input = std::max(max_abs_input, std::max(std::abs(x), std::abs(y)));
        finite_input_points++;
    };
    for (const auto &p : map.layout)
        collect_input_stats(p.x, p.y);
    for (const auto &obj : map.objects)
        for (const auto &v : obj.layout)
            collect_input_stats(v.x, v.y);

    if (finite_input_points == 0)
    {
        qWarning() << "[initialize_grid] LayoutData has no finite points.";
        return;
    }

    const float input_scale = (max_abs_input < 200.f) ? 1000.f : 1.f;
    const float rotation = params.MRPT_MAP_ROTATION;
    const float cos_r = std::cos(rotation);
    const float sin_r = std::sin(rotation);
    const bool mirror_x = params.MRPT_MAP_MIRROR_X;
    const float offset_x = params.MRPT_MAP_OFFSET_X;
    const float offset_y = params.MRPT_MAP_OFFSET_Y;

    auto to_scene = [&](float x, float y)
    {
        float sx = x * input_scale;
        const float sy = y * input_scale;
        if (mirror_x)
            sx = -sx;
        const float rx = sx * cos_r - sy * sin_r + offset_x;
        const float ry = sx * sin_r + sy * cos_r + offset_y;
        return QPointF(rx, ry);
    };

    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    auto update_bounds = [&](float x, float y)
    {
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
    };

    size_t layout_points_drawn = 0;
    size_t object_polygons_drawn = 0;

    // 1) Draw global layout as polyline + points.
    QPolygonF global_layout_poly;
    global_layout_poly.reserve(static_cast<int>(map.layout.size()));

    const float point_size = std::max(20.f, params.TILE_SIZE * 0.2f);
    const QPen point_pen(QColor("DarkCyan"), 30);
    const QBrush point_brush(QColor(0, 180, 180, 180));
    for (const auto &p : map.layout)
    {
        if (not std::isfinite(p.x) || not std::isfinite(p.y))
            continue;

        const QPointF sp = to_scene(p.x, p.y);
        global_layout_poly << sp;

        auto *dot = viewer->scene.addEllipse(-point_size / 2.f, -point_size / 2.f,
                                                     point_size, point_size,
                                                     point_pen, point_brush);
        dot->setPos(sp);
        dot->setZValue(2);
        map_items.push_back(dot);
        update_bounds(static_cast<float>(sp.x()), static_cast<float>(sp.y()));
        layout_points_drawn++;
    }

    if (global_layout_poly.size() >= 2)
    {
        QPen layout_pen(QColor("SteelBlue"), 65);
        layout_pen.setJoinStyle(Qt::RoundJoin);
        layout_pen.setCapStyle(Qt::RoundCap);
        auto *layout_path = viewer->scene.addPolygon(global_layout_poly, layout_pen, QBrush(Qt::NoBrush));
        layout_path->setZValue(1);
        map_items.push_back(layout_path);
    }

    // 2) Draw each object as a filled polygon.
    for (size_t i = 0; i < map.objects.size(); ++i)
    {
        const auto &obj = map.objects[i];
        if (obj.layout.empty())
            continue;

        QPolygonF polygon;
        polygon.reserve(static_cast<int>(obj.layout.size()));
        for (const auto &v : obj.layout)
        {
            if (not std::isfinite(v.x) || not std::isfinite(v.y))
                continue;
            const QPointF sv = to_scene(v.x, v.y);
            polygon << sv;
            update_bounds(static_cast<float>(sv.x()), static_cast<float>(sv.y()));
        }

        if (polygon.size() < 3)
            continue;

        const QColor color = QColor::fromHsv(static_cast<int>((i * 47) % 360), 170, 220, 90);
        auto *poly_item = viewer->scene.addPolygon(polygon, QPen(color.darker(150), 35), QBrush(color));
        poly_item->setZValue(3);
        map_items.push_back(poly_item);
        object_polygons_drawn++;
    }

    viewer->scene.update();
    if (min_x <= max_x && min_y <= max_y)
    {
        const float margin = std::max(500.f, params.TILE_SIZE * 3.f);
        viewer->fitToScene(QRectF(min_x - margin, min_y - margin,
                              (max_x - min_x) + 2.f * margin,
                              (max_y - min_y) + 2.f * margin));
    }

        qInfo() << "[initialize_grid] scale=" << input_scale
            << "layout points drawn=" << layout_points_drawn
            << "object polygons drawn=" << object_polygons_drawn;
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

