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
#ifndef NAVIGATOR_H
#define NAVIGATOR_H

// Ice includes
#include <Ice/Ice.h>
#include <Navigator.h>

#include "../src/specificworker.h"


class NavigatorI : public virtual RoboCompNavigator::Navigator
{
public:
	NavigatorI(GenericWorker *_worker, const size_t id);
	~NavigatorI();

	RoboCompNavigator::LayoutData getLayout(const Ice::Current&);
	RoboCompNavigator::Result getPath(RoboCompNavigator::TPoint source, RoboCompNavigator::TPoint target, float safety, const Ice::Current&);
	RoboCompNavigator::TPose getRobotPose(const Ice::Current&);
	RoboCompNavigator::NavigationStatus getStatus(const Ice::Current&);
	RoboCompNavigator::TPoint gotoObject(std::string object, const Ice::Current&);
	RoboCompNavigator::TPoint gotoPoint(RoboCompNavigator::TPoint target, const Ice::Current&);
	void resume(const Ice::Current&);
	void stop(const Ice::Current&);

private:

	GenericWorker *worker;
	size_t id;

	// Array handlers for each method
	std::array<std::function<RoboCompNavigator::LayoutData(void)>, 1> getLayoutHandlers;
	std::array<std::function<RoboCompNavigator::Result(RoboCompNavigator::TPoint, RoboCompNavigator::TPoint, float)>, 1> getPathHandlers;
	std::array<std::function<RoboCompNavigator::TPose(void)>, 1> getRobotPoseHandlers;
	std::array<std::function<RoboCompNavigator::NavigationStatus(void)>, 1> getStatusHandlers;
	std::array<std::function<RoboCompNavigator::TPoint(std::string)>, 1> gotoObjectHandlers;
	std::array<std::function<RoboCompNavigator::TPoint(RoboCompNavigator::TPoint)>, 1> gotoPointHandlers;
	std::array<std::function<void(void)>, 1> resumeHandlers;
	std::array<std::function<void(void)>, 1> stopHandlers;

};

#endif
