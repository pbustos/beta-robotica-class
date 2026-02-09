/*
 *    Copyright (C) 2024 by YOUR NAME HERE
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
#include <cppitertools/filter.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/range.hpp>

class TPointVector;

/**
* \brief Default constructor
*/
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
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}
void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

//chekpoint robocompUpdater
	std::cout << "Initialize worker" << std::endl;
    int period = 100;
	setPeriod("Compute", period);
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        // Viewer
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        viewer->setSceneRect(params.GRID_MAX_DIM);
        viewer->show();

        // Grid
        grid.initialize(params.GRID_MAX_DIM, static_cast<int>(params.TILE_SIZE), &viewer->scene);

        // Lidar thread is created
        read_lidar_th = std::thread(&SpecificWorker::read_lidar,this);
        std::cout << __FUNCTION__ << " Started lidar reader" << std::endl;

        // mouse
        connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, [this](QPointF p)
        {
            qInfo() << "[MOUSE] New global target arrived:" << p;
            mutex_path.lock();
            auto [success, msg, source_key, target_key] =
            grid.validate_source_target(Eigen::Vector2f{0.0, 0.0},
                                        750,
                                        Eigen::Vector2f{p.x(), p.y()},
                                        750);
            if (success) {

                auto paths = grid.compute_k_paths(source_key,
                                                  target_key,
                                                  std::clamp(3, 1, params.NUM_PATHS_TO_SEARCH),
                                                  params.MIN_DISTANCE_BETWEEN_PATHS,
                                                  true, true);


                if(not paths.empty())
                {
                    qInfo() << paths.size() << "paths found";
                    draw_path(paths.front(), &viewer->scene);
					//insert_path_node(Eigen::Vector2f(paths.front()));
                    // this->lcdNumber_length->display((int) paths.front().size());
                }

            mutex_path.unlock();
        }
            // get key from point and print key values


        });
        connect(viewer, &AbstractGraphicViewer::right_click, [this](QPointF p)
        {
            qInfo() <<  "RIGHT CLICK. Cancelling target";
            draw_path({}, &viewer->scene, true);
            cancel_from_mouse = true;
        });
        if(not params.DISPLAY)
            hide();
		
	}
}
void SpecificWorker::compute()
{
    // Get robot pose from Webots and return transform
    // robot_pose = get_robot_pose();  // TODO: implement get_robot_pose()

    // read LiDAR
    auto res_ = buffer_lidar_data.try_get();
    if (not res_.has_value())  {   /*qWarning() << "No data Lidar";*/ return; }
    auto points = res_.value();

    /// clear grid and update it
    mutex_path.lock();
        grid.check_and_resize(points);
        grid.clear();  // sets all cells to initial values
        grid.update_map(points, Eigen::Vector2f{0.0, 0.0}, params.MAX_LIDAR_RANGE);
        grid.update_costs( params.ROBOT_SEMI_WIDTH, true);     // not color all cells
    mutex_path.unlock();

    this->hz = fps.print("FPS:", 3000);
    this->lcdNumber_hz->display(this->hz);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Eigen::Affine2f SpecificWorker::get_robot_pose()
// {
//     RoboCompWebots2Robocomp::ObjectPose pose;
//     Eigen::Affine2f robot_pose;
//     try
//     {
//         pose = webots2robocomp_proxy->getObjectPose("shadow");
//         //qInfo() << "Robot pose from Webots: x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z;
//         robot_pose.translation() = Eigen::Vector2f(-pose.position.y, pose.position.x);
//         const auto yaw = yawFromQuaternion(pose.orientation);
//         robot_pose.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();
//     }
//     catch (const Ice::Exception &e){ std::cout<<e.what()<<std::endl; return {};}
//     return robot_pose;
// }

void SpecificWorker::read_lidar()
{
    auto wait_period = std::chrono::milliseconds (getPeriod("Compute"));
    while(true)
    {
        try
        {
            auto data = lidar3d_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_LOW,
                                                                   params.MAX_LIDAR_LOW_RANGE,
                                                                   params.LIDAR_LOW_DECIMATION_FACTOR);
            auto data_helios = lidar3d1_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_HIGH,
                                                                           params.MAX_LIDAR_HIGH_RANGE,
                                                                           params.LIDAR_HIGH_DECIMATION_FACTOR);
            // concatenate both lidars
            data.points.insert(data.points.end(), data_helios.points.begin(), data_helios.points.end());
            // compute the period to read the lidar based on the current difference with the lidar period. Use a hysteresis of 2ms
            if (wait_period > std::chrono::milliseconds((long) data.period + 2)) wait_period--;
            else if (wait_period < std::chrono::milliseconds((long) data.period - 2)) wait_period++;
            std::vector<Eigen::Vector3f> eig_data(data.points.size());
            for (const auto &[i, p]: data.points | iter::enumerate)
                eig_data[i] = {p.x, p.y, p.z};
            buffer_lidar_data.put(std::move(eig_data));
        }
        catch (const Ice::Exception &e)
        { std::cout << "Error reading from Lidar3D" << e << std::endl; }
        std::this_thread::sleep_for(wait_period);
    }
} // Thread to read the lidar

void SpecificWorker::draw_path(const std::vector<Eigen::Vector2f> &path, QGraphicsScene *scene, bool erase_only)
{
    static std::vector<QGraphicsEllipseItem*> points;
    for(auto p : points)
        scene->removeItem(p);
    points.clear();

    if(erase_only) return;

    float s = 100;
    auto color = QColor("green");
    for(const auto &p: path)
    {
        auto ptr = scene->addEllipse(-s/2, -s/2, s, s, QPen(color), QBrush(color));
        ptr->setPos(QPointF(p.x(), p.y()));
        points.push_back(ptr);
    }
}

void SpecificWorker::draw_paths(const std::vector<std::vector<Eigen::Vector2f>> &paths, QGraphicsScene *scene, bool erase_only)
{
    static std::vector<QGraphicsEllipseItem*> points;
    static QColor colors[] = {QColor("cyan"), QColor("blue"), QColor("red"), QColor("orange"), QColor("magenta"), QColor("cyan")};
    for(auto p : points)
        scene->removeItem(p);
    points.clear();

    if(erase_only) return;

    float s = 80;
    for(const auto &[i, path]: paths | iter::enumerate)
    {
        // pick a consecutive color
        auto color = colors[i];
        for(const auto &p: path)
        {
            auto ptr = scene->addEllipse(-s/2.f, -s/2.f, s, s, QPen(color), QBrush(color));
            ptr->setPos(QPointF(p.x(), p.y()));
            points.push_back(ptr);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
RoboCompGridder::Result SpecificWorker::Gridder_getPaths(RoboCompGridder::TPoint source,
                                                         RoboCompGridder::TPoint target,
                                                         int max_paths,
                                                         bool tryClosestFreePoint,
                                                         bool targetIsHuman)
{
    //TODO: improve this method to try to find a path even if the target is not free by using the closest free point
    //TODO: if target is human, set safe area around as free
    RoboCompGridder::Result result;
    std::vector<std::vector<Eigen::Vector2f>> paths;

    auto begin = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    qInfo() << __FUNCTION__ << " New plan request: source [" << source.x << source.y << "], target [" << target.x << target.y << "]"
            << " max_paths: " << max_paths;
    mutex_path.lock();
    auto [success, msg, source_key, target_key] =
            grid.validate_source_target(Eigen::Vector2f{source.x, source.y},
                                        source.radius,
                                        Eigen::Vector2f{target.x, target.y},
                                        source.radius);
    if (success)
    {
        //check if is line of sight to target free
        if (grid.is_line_of_sigth_to_target_free(source_key,
                                                 target_key,
                                                 params.ROBOT_SEMI_WIDTH))
        {
            paths.emplace_back(grid.compute_path_line_of_sight(source_key, target_key, params.ROBOT_SEMI_LENGTH));
            if(paths.empty())
                msg = "VLOS path not found";
            else
                msg = "VLOS path";
        }
        else
        {
            paths = grid.compute_k_paths(source_key, target_key,
                                         std::clamp(max_paths, 1, params.NUM_PATHS_TO_SEARCH),
                                         params.MIN_DISTANCE_BETWEEN_PATHS,
                                         tryClosestFreePoint,
                                         targetIsHuman);
            if(paths.empty())
                msg = "Djikstra path not found";
            else
                msg = "Djikstra path";
        }
    }
    mutex_path.unlock();
    result.errorMsg = msg;
    result.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // If not success return result with empty paths, error message and timestamp
    if (not success)
        return result;
    else
    {
        // fill Result with data
        result.paths.resize(paths.size());
        for (const auto &[i, path]: paths | iter::enumerate)
        {
            result.paths[i].resize(path.size());
            for (const auto &[j, point]: path | iter::enumerate)
            {
                result.paths[i][j].x = point.x();
                result.paths[i][j].y = point.y();
            }
        }
        qInfo() << __FUNCTION__ << " " << paths.size() << " paths computed in " <<
                std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count() - begin << " ms" << "Status:" << msg.c_str();
        return result;
    }
}
bool SpecificWorker::Gridder_LineOfSightToTarget(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, float robotRadius)
{
    std::lock_guard<std::mutex> lock(mutex_path);
    auto [success, msg, source_key, target_key] =
            grid.validate_source_target(Eigen::Vector2f{source.x, source.y}, source.radius,
                                        Eigen::Vector2f{target.x, target.y}, target.radius);

    if(success)
        return grid.is_line_of_sigth_to_target_free(source_key, target_key, robotRadius);
    else
        return false;
}
RoboCompGridder::TPoint SpecificWorker::Gridder_getClosestFreePoint(RoboCompGridder::TPoint source)
{
    std::lock_guard<std::mutex> lock(mutex_path);
    if(const auto &p = grid.closest_free({source.x, source.y}); p.has_value())
        return {static_cast<float>(p->x()), static_cast<float>(p->y())};
    else
        return {0, 0};  // non valid closest point  TODO: Change return type so failure to find can be signaled
}
RoboCompGridder::TDimensions SpecificWorker::Gridder_getDimensions()
{
    return {static_cast<float>(params.GRID_MAX_DIM.x()),
            static_cast<float>(params.GRID_MAX_DIM.y()),
            static_cast<float>(params.GRID_MAX_DIM.width()),
            static_cast<float>(params.GRID_MAX_DIM.height())};
}
bool SpecificWorker::Gridder_setGridDimensions(RoboCompGridder::TDimensions dimensions)
{
    qInfo() << __FUNCTION__ << " Setting grid dimensions to [" << dimensions.left << dimensions.top << dimensions.width << dimensions.height << "]";
    params.GRID_MAX_DIM = QRectF(dimensions.left, dimensions.top, dimensions.width, dimensions.height);
    //TODO: update grid, clear and reinitialize
    return true;
}
RoboCompGridder::Result SpecificWorker::Gridder_setLocationAndGetPath(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, RoboCompGridder::TPointVector freePoints, RoboCompGridder::TPointVector obstaclePoints)
{

    std::vector<std::tuple<std::pair<int, int>, Grid::T>> submap_copy;

    //Lambda to get grid keys from TPointVector
    auto get_key_vector = [this](const RoboCompGridder::TPointVector &v)
    {
        std::vector<std::tuple<Grid::Key, float>> keys;
        for(const auto &p: v)
            keys.push_back(std::make_tuple(grid.point_to_key(Eigen::Vector2f(p.x, p.y)), p.radius));
        return keys;
    };

    //get keys from TPointVector
    auto free_keys = get_key_vector(freePoints);
    auto obstacle_keys = get_key_vector(obstaclePoints);

    mutex_path.lock();
    //Copy submap from actual grid
    // iterate over free keys and obstacle keys to copy submap
    for (auto &key: free_keys){
        auto cells = grid.copy_submap(std::get<0>(key), std::get<1>(key));
        std::move(cells.begin(), cells.end(), std::back_inserter(submap_copy));

    }
    for (auto &key: obstacle_keys){
        auto cells = grid.copy_submap(std::get<0>(key), std::get<1>(key));
        std::move(cells.begin(), cells.end(), std::back_inserter(submap_copy));

    }

    //Iterate over free keys and obstacle keys to set submap
    for (auto &key: free_keys){
        grid.set_submap(std::get<0>(key), std::get<1>(key), true);
    }
    for (auto &key: obstacle_keys){
        grid.set_submap(std::get<0>(key), std::get<1>(key), false);
    }

    //get paths
    auto result = Gridder_getPaths(source, target, 1, true, true);
    
    //restore submap
    grid.paste_submap(submap_copy);
    mutex_path.unlock();

    return result;
}
bool SpecificWorker::Gridder_IsPathBlocked(RoboCompGridder::TPath path)
{
    std::lock_guard<std::mutex> lock(mutex_path);
    std::vector<Eigen::Vector2f> path_;
    for(const auto &p: path)
        path_.emplace_back(p.x, p.y);
    return grid.is_path_blocked(path_);
}
//////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
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
