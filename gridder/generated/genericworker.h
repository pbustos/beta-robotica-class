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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include <stdint.h>

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
#define USE_QTGUI

#include <grafcetStep/GRAFCETStep.h>
#include <ConfigLoader/ConfigLoader.h>
#include <QStateMachine>
#include <QEvent>
#include <QString>
#include <functional>
#include <atomic>
#include <QtCore>
#include <variant>
#include <unordered_map>


#include <GenericBase.h>
#include <Gridder.h>
#include <Lidar3D.h>
#include <OmniRobot.h>
#include <Webots2Robocomp.h>

#define BASIC_PERIOD 100

using TuplePrx = std::tuple<RoboCompLidar3D::Lidar3DPrxPtr,RoboCompLidar3D::Lidar3DPrxPtr,RoboCompOmniRobot::OmniRobotPrxPtr,RoboCompWebots2Robocomp::Webots2RobocompPrxPtr>;


class GenericWorker : public QWidget, public Ui_guiDlg
{
Q_OBJECT
public:
	GenericWorker(const ConfigLoader& configLoader, TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();

	void setPeriod(const std::string& state, int period);
	int getPeriod(const std::string& state);

	QStateMachine statemachine;
	QTimer hibernationChecker;
	std::atomic_bool hibernation = false;


	RoboCompLidar3D::Lidar3DPrxPtr lidar3d_proxy;
	RoboCompLidar3D::Lidar3DPrxPtr lidar3d1_proxy;
	RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy;
	RoboCompWebots2Robocomp::Webots2RobocompPrxPtr webots2robocomp_proxy;

	virtual bool Gridder_IsPathBlocked(RoboCompGridder::TPath path) = 0;
	virtual bool Gridder_LineOfSightToTarget(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, float robotRadius) = 0;
	virtual RoboCompGridder::TPoint Gridder_getClosestFreePoint(RoboCompGridder::TPoint source) = 0;
	virtual RoboCompGridder::TDimensions Gridder_getDimensions() = 0;
	virtual RoboCompGridder::Map Gridder_getMap() = 0;
	virtual RoboCompGridder::Result Gridder_getPaths(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, int maxPaths, bool tryClosestFreePoint, bool targetIsHuman, float safetyFactor) = 0;
	virtual RoboCompGridder::Pose Gridder_getPose() = 0;
	virtual bool Gridder_setGridDimensions(RoboCompGridder::TDimensions dimensions) = 0;
	virtual RoboCompGridder::Result Gridder_setLocationAndGetPath(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, RoboCompGridder::TPointVector freePoints, RoboCompGridder::TPointVector obstaclePoints) = 0;


protected:
	std::unordered_map<std::string, std::unique_ptr<GRAFCETStep>> states;
	ConfigLoader configLoader;




private:

public slots:
	virtual void initialize() = 0;
	virtual void compute() = 0;
	virtual void emergency() = 0;
	virtual void restore() = 0;
	void hibernationCheck();
	void hibernationTick();
	
signals:
	void kill();
	void goToEmergency();
	void goToRestore();
};

#endif
