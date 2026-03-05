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
#include <algorithm>
#include <limits>
#include <QtMath>
#include <QSettings>

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
	QSettings settings("robocomp", "ainf_mission_planner");
	settings.setValue("window/geometry", this->saveGeometry());
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

	viewer = new AbstractGraphicViewer(this->frame, QRectF(-8, -8, 16, 16), true);
	viewer->add_robot(0.46f, 0.48f, 0.0f, 0.2f, QColor("Blue"));
	viewer->show();
	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::slot_new_target);

	connect(pushButton_startFollow, &QPushButton::clicked, this, [this]()
	{
		if(not has_target)
		{
			qWarning() << "No target selected. Shift+Right click on the map first.";
			return;
		}
		try
		{
			navigator_proxy->gotoPoint(last_target);
			qInfo() << "Path following started toward" << last_target.x << last_target.y;
		}
		catch(const Ice::Exception &e)
		{
			qWarning() << "Error starting path following:" << e.what();
		}
	});

	connect(pushButton_stopFollow, &QPushButton::clicked, this, [this]()
	{
		try
		{
			navigator_proxy->stop();
			qInfo() << "Path following stopped";
		}
		catch(const Ice::Exception &e)
		{
			qWarning() << "Error stopping path following:" << e.what();
		}
	});

	connect(pushButton_resumeFollow, &QPushButton::clicked, this, [this]()
	{
		try
		{
			navigator_proxy->resume();
			qInfo() << "Path following resumed";
		}
		catch(const Ice::Exception &e)
		{
			qWarning() << "Error resuming path following:" << e.what();
		}
	});

	QSettings settings("robocomp", "ainf_mission_planner");
	if(settings.contains("window/geometry"))
		this->restoreGeometry(settings.value("window/geometry").toByteArray());

	try
	{
		const auto map_data = navigator_proxy->getLayout();

		auto draw_polyline = [this](const RoboCompNavigator::TLayout &polyline, const QPen &pen)
		{
			if(polyline.size() < 2)
				return;

			for(size_t i = 0; i < polyline.size() - 1; ++i)
			{
				const auto &p1 = polyline[i];
				const auto &p2 = polyline[i + 1];
				viewer->scene.addLine(QLineF(p1.x, p1.y, p2.x, p2.y), pen);
			}
			const auto &first = polyline.front();
			const auto &last = polyline.back();
			viewer->scene.addLine(QLineF(last.x, last.y, first.x, first.y), pen);
		};

		draw_polyline(map_data.layout, QPen(QColor("magenta"), 0.16));

		for(size_t i = 0; i < map_data.objects.size(); ++i)
		{
			const auto &obj = map_data.objects[i];
			const QColor object_color = QColor::fromHsv(static_cast<int>((i * 47) % 360), 210, 230);

			if(obj.layout.size() >= 3)
			{
				QPolygonF polygon;
				polygon.reserve(static_cast<int>(obj.layout.size()));
				for(const auto &point : obj.layout)
					polygon << QPointF(point.x, point.y);

				viewer->scene.addPolygon(polygon,
				                         QPen(object_color, 0.07),
				                         QBrush(object_color, Qt::SolidPattern));
			}
			else
			{
				draw_polyline(obj.layout, QPen(object_color, 0.07));
			}

			if(not obj.layout.empty())
			{
				const auto &p = obj.layout.front();
				auto *text = viewer->scene.addText(QString::fromStdString(obj.name));
				QFont font = text->font();
				font.setBold(true);
				text->setFont(font);
				text->setDefaultTextColor(QColor("black"));
				text->setTransform(QTransform::fromScale(0.012, -0.012));
				text->setPos(p.x, p.y);
				text->setZValue(120);
			}
		}

		if(not map_data.layout.empty())
		{
			float min_x = std::numeric_limits<float>::max();
			float max_x = std::numeric_limits<float>::lowest();
			float min_y = std::numeric_limits<float>::max();
			float max_y = std::numeric_limits<float>::lowest();

			for(const auto &p : map_data.layout)
			{
				min_x = std::min(min_x, p.x);
				max_x = std::max(max_x, p.x);
				min_y = std::min(min_y, p.y);
				max_y = std::max(max_y, p.y);
			}

			auto *legend = viewer->scene.addText("Room: magenta (thick) | Objects: unique filled colors");
			legend->setDefaultTextColor(QColor("darkBlue"));
			legend->setTransform(QTransform::fromScale(0.02, -0.02));
			legend->setPos(min_x, max_y + 0.15f);

			viewer->fitToScene(QRectF(min_x, min_y, max_x - min_x, max_y - min_y));
		}
	}
	catch(const Ice::Exception &e)
	{
		qWarning() << "Error requesting map from Navigator:" << e.what();
	}

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

}



void SpecificWorker::compute()
{

	if(viewer == nullptr or viewer->robot_poly() == nullptr)
		return;

	try
	{
		const auto pose = navigator_proxy->getRobotPose();
		viewer->robot_poly()->setPos(pose.x, pose.y);
		viewer->robot_poly()->setRotation(qRadiansToDegrees(pose.r));
		if(not planned_path_points.empty())
			redraw_planned_path(RoboCompNavigator::TPoint{pose.x, pose.y});
		label_robotCoordsValue->setText(QString("x=%1  y=%2  θ=%3")
		                               .arg(pose.x, 0, 'f', 2)
		                               .arg(pose.y, 0, 'f', 2)
		                               .arg(pose.r, 0, 'f', 2));

	}
	catch(const Ice::Exception &e) { qWarning() << "Error requesting robot pose from Navigator:" << e.what(); }	
	try
	{
		auto status = navigator_proxy->getStatus();
		QString state_text = "UNKNOWN";
		switch(status.state)
		{
			case RoboCompNavigator::NavigationState::IDLE: state_text = "IDLE"; break;
			case RoboCompNavigator::NavigationState::NAVIGATING: state_text = "NAVIGATING"; break;
			case RoboCompNavigator::NavigationState::PAUSED: state_text = "PAUSED"; break;
			case RoboCompNavigator::NavigationState::REACHED: state_text = "REACHED"; break;
			case RoboCompNavigator::NavigationState::BLOCKED: state_text = "BLOCKED"; break;
			case RoboCompNavigator::NavigationState::ERROR: state_text = "ERROR"; break;
		}
		if(not status.statusMessage.empty())
		{
			state_text += QString(" (%1)").arg(QString::fromStdString(status.statusMessage));
		}
		label_missionStatusValue->setText(state_text);

		label_distanceToTargetValue->setText(QString::number(status.distanceToTarget, 'f', 2));
		label_etaValue->setText(QString::number(status.estimatedTime, 'f', 2));
		label_currentSpeedValue->setText(QString::number(status.currentSpeed, 'f', 2));
	}
	catch(const Ice::Exception &e)
	{
		qWarning() << "Error requesting mission status from Navigator:" << e.what();
		if(label_missionStatusValue->text().isEmpty())
			label_missionStatusValue->setText("IDLE");
		label_distanceToTargetValue->setText("--");
		label_etaValue->setText("--");
		label_currentSpeedValue->setText("--");
	}
}

////////////////////////////////////////////////////////////////////////////////


void SpecificWorker::slot_new_target(QPointF target)
{
	if(viewer == nullptr)
		return;

	try
	{
		const auto pose = navigator_proxy->getRobotPose();
		RoboCompNavigator::TPoint source{pose.x, pose.y};
		RoboCompNavigator::TPoint destination{static_cast<float>(target.x()), static_cast<float>(target.y())};
		last_target = destination;
		has_target = true;

		constexpr float safety = 0.25f;
		const auto result = navigator_proxy->getPath(source, destination, safety);

		if(not result.valid)
		{
			qWarning() << "Path request failed:" << QString::fromStdString(result.errorMsg);
			return;
		}

		if(result.path.size() < 2)
		{
			qWarning() << "Path request returned an empty or single-point path";
			planned_path_points.clear();
			return;
		}

		planned_path_points = result.path;
		redraw_planned_path(source);

		qInfo() << "Path planned with" << result.path.size() << "points";
	}
	catch(const Ice::Exception &e)
	{
		qWarning() << "Error requesting path to clicked target:" << e.what();
	}
}

void SpecificWorker::redraw_planned_path(const RoboCompNavigator::TPoint &current_source)
{
	if(viewer == nullptr)
		return;

	for(auto *item : planned_path_items)
	{
		viewer->scene.removeItem(item);
		delete item;
	}
	planned_path_items.clear();

	if(planned_path_points.size() < 2)
		return;

	QPen path_pen(QColor("orange"), 0.09);
	RoboCompNavigator::TPoint p1 = current_source;
	const auto &p2_first = planned_path_points[1];
	auto *first_segment = viewer->scene.addLine(QLineF(p1.x, p1.y, p2_first.x, p2_first.y), path_pen);
	first_segment->setZValue(200);
	planned_path_items.push_back(first_segment);

	for(size_t i = 1; i < planned_path_points.size() - 1; ++i)
	{
		const auto &p1_mid = planned_path_points[i];
		const auto &p2_mid = planned_path_points[i + 1];
		auto *segment = viewer->scene.addLine(QLineF(p1_mid.x, p1_mid.y, p2_mid.x, p2_mid.y), path_pen);
		segment->setZValue(200);
		planned_path_items.push_back(segment);
	}

	auto *start_marker = viewer->scene.addEllipse(current_source.x - 0.08, current_source.y - 0.08, 0.16, 0.16,
	                                             QPen(QColor("darkOrange"), 0.03),
	                                             QBrush(QColor("darkOrange"), Qt::SolidPattern));
	start_marker->setZValue(210);
	planned_path_items.push_back(start_marker);

	const auto &goal = planned_path_points.back();
	auto *goal_marker = viewer->scene.addEllipse(goal.x - 0.09, goal.y - 0.09, 0.18, 0.18,
	                                            QPen(QColor("red"), 0.03),
	                                            QBrush(QColor("red"), Qt::SolidPattern));
	goal_marker->setZValue(210);
	planned_path_items.push_back(goal_marker);
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


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}



/**************************************/
// From the RoboCompNavigator you can call this methods:
// RoboCompNavigator::LayoutData this->navigator_proxy->getLayout()
// RoboCompNavigator::Result this->navigator_proxy->getPath(TPoint source, TPoint target, float safety)
// RoboCompNavigator::TPoint this->navigator_proxy->gotoObject(string object)
// RoboCompNavigator::TPoint this->navigator_proxy->gotoPoint(TPoint target)
// RoboCompNavigator::void this->navigator_proxy->resume()
// RoboCompNavigator::void this->navigator_proxy->stop()

/**************************************/
// From the RoboCompNavigator you can use this types:
// RoboCompNavigator::TPoint
// RoboCompNavigator::Result
// RoboCompNavigator::Pose
// RoboCompNavigator::NavigationStatus
// RoboCompNavigator::TObject
// RoboCompNavigator::LayoutData

