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
#include "navigatorI.h"

NavigatorI::NavigatorI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getLayoutHandlers = {
		[this]() { return worker->Navigator_getLayout(); }
	};

	getPathHandlers = {
		[this](auto a, auto b, auto c) { return worker->Navigator_getPath(a, b, c); }
	};

	getRobotPoseHandlers = {
		[this]() { return worker->Navigator_getRobotPose(); }
	};

	getStatusHandlers = {
		[this]() { return worker->Navigator_getStatus(); }
	};

	gotoObjectHandlers = {
		[this](auto a) { return worker->Navigator_gotoObject(a); }
	};

	gotoPointHandlers = {
		[this](auto a) { return worker->Navigator_gotoPoint(a); }
	};

	resumeHandlers = {
		[this]() { return worker->Navigator_resume(); }
	};

	stopHandlers = {
		[this]() { return worker->Navigator_stop(); }
	};

}


NavigatorI::~NavigatorI()
{
}


RoboCompNavigator::LayoutData NavigatorI::getLayout(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getLayoutHandlers.size())
		return  getLayoutHandlers[id]();
	else
		throw std::out_of_range("Invalid getLayout id: " + std::to_string(id));

}

RoboCompNavigator::Result NavigatorI::getPath(RoboCompNavigator::TPoint source, RoboCompNavigator::TPoint target, float safety, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getPathHandlers.size())
		return  getPathHandlers[id](source, target, safety);
	else
		throw std::out_of_range("Invalid getPath id: " + std::to_string(id));

}

RoboCompNavigator::TPose NavigatorI::getRobotPose(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getRobotPoseHandlers.size())
		return  getRobotPoseHandlers[id]();
	else
		throw std::out_of_range("Invalid getRobotPose id: " + std::to_string(id));

}

RoboCompNavigator::NavigationStatus NavigatorI::getStatus(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getStatusHandlers.size())
		return  getStatusHandlers[id]();
	else
		throw std::out_of_range("Invalid getStatus id: " + std::to_string(id));

}

RoboCompNavigator::TPoint NavigatorI::gotoObject(std::string object, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < gotoObjectHandlers.size())
		return  gotoObjectHandlers[id](object);
	else
		throw std::out_of_range("Invalid gotoObject id: " + std::to_string(id));

}

RoboCompNavigator::TPoint NavigatorI::gotoPoint(RoboCompNavigator::TPoint target, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < gotoPointHandlers.size())
		return  gotoPointHandlers[id](target);
	else
		throw std::out_of_range("Invalid gotoPoint id: " + std::to_string(id));

}

void NavigatorI::resume(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < resumeHandlers.size())
		 resumeHandlers[id]();
	else
		throw std::out_of_range("Invalid resume id: " + std::to_string(id));

}

void NavigatorI::stop(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < stopHandlers.size())
		 stopHandlers[id]();
	else
		throw std::out_of_range("Invalid stop id: " + std::to_string(id));

}

