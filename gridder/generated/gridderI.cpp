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
#include "gridderI.h"

GridderI::GridderI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	IsPathBlockedHandlers = {
		[this](auto a) { return worker->Gridder_IsPathBlocked(a); }
	};

	LineOfSightToTargetHandlers = {
		[this](auto a, auto b, auto c) { return worker->Gridder_LineOfSightToTarget(a, b, c); }
	};

	cancelNavigationHandlers = {
		[this]() { return worker->Gridder_cancelNavigation(); }
	};

	getClosestFreePointHandlers = {
		[this](auto a) { return worker->Gridder_getClosestFreePoint(a); }
	};

	getDimensionsHandlers = {
		[this]() { return worker->Gridder_getDimensions(); }
	};

	getDistanceToTargetHandlers = {
		[this]() { return worker->Gridder_getDistanceToTarget(); }
	};

	getEstimatedTimeToTargetHandlers = {
		[this]() { return worker->Gridder_getEstimatedTimeToTarget(); }
	};

	getMapHandlers = {
		[this]() { return worker->Gridder_getMap(); }
	};

	getNavigationStateHandlers = {
		[this]() { return worker->Gridder_getNavigationState(); }
	};

	getNavigationStatusHandlers = {
		[this]() { return worker->Gridder_getNavigationStatus(); }
	};

	getPathsHandlers = {
		[this](auto a, auto b, auto c, auto d, auto e, auto f) { return worker->Gridder_getPaths(a, b, c, d, e, f); }
	};

	getPoseHandlers = {
		[this]() { return worker->Gridder_getPose(); }
	};

	getTargetHandlers = {
		[this]() { return worker->Gridder_getTarget(); }
	};

	hasReachedTargetHandlers = {
		[this]() { return worker->Gridder_hasReachedTarget(); }
	};

	replanPathHandlers = {
		[this]() { return worker->Gridder_replanPath(); }
	};

	resumeNavigationHandlers = {
		[this]() { return worker->Gridder_resumeNavigation(); }
	};

	setGridDimensionsHandlers = {
		[this](auto a) { return worker->Gridder_setGridDimensions(a); }
	};

	setLocationAndGetPathHandlers = {
		[this](auto a, auto b, auto c, auto d) { return worker->Gridder_setLocationAndGetPath(a, b, c, d); }
	};

	setTargetHandlers = {
		[this](auto a) { return worker->Gridder_setTarget(a); }
	};

	setTargetWithOptionsHandlers = {
		[this](auto a, auto b) { return worker->Gridder_setTargetWithOptions(a, b); }
	};

	startNavigationHandlers = {
		[this]() { return worker->Gridder_startNavigation(); }
	};

	stopNavigationHandlers = {
		[this]() { return worker->Gridder_stopNavigation(); }
	};

}


GridderI::~GridderI()
{
}


bool GridderI::IsPathBlocked(RoboCompGridder::TPath path, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < IsPathBlockedHandlers.size())
		return  IsPathBlockedHandlers[id](path);
	else
		throw std::out_of_range("Invalid IsPathBlocked id: " + std::to_string(id));

}

bool GridderI::LineOfSightToTarget(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, float robotRadius, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < LineOfSightToTargetHandlers.size())
		return  LineOfSightToTargetHandlers[id](source, target, robotRadius);
	else
		throw std::out_of_range("Invalid LineOfSightToTarget id: " + std::to_string(id));

}

void GridderI::cancelNavigation(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < cancelNavigationHandlers.size())
		 cancelNavigationHandlers[id]();
	else
		throw std::out_of_range("Invalid cancelNavigation id: " + std::to_string(id));

}

RoboCompGridder::TPoint GridderI::getClosestFreePoint(RoboCompGridder::TPoint source, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getClosestFreePointHandlers.size())
		return  getClosestFreePointHandlers[id](source);
	else
		throw std::out_of_range("Invalid getClosestFreePoint id: " + std::to_string(id));

}

RoboCompGridder::TDimensions GridderI::getDimensions(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getDimensionsHandlers.size())
		return  getDimensionsHandlers[id]();
	else
		throw std::out_of_range("Invalid getDimensions id: " + std::to_string(id));

}

float GridderI::getDistanceToTarget(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getDistanceToTargetHandlers.size())
		return  getDistanceToTargetHandlers[id]();
	else
		throw std::out_of_range("Invalid getDistanceToTarget id: " + std::to_string(id));

}

float GridderI::getEstimatedTimeToTarget(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getEstimatedTimeToTargetHandlers.size())
		return  getEstimatedTimeToTargetHandlers[id]();
	else
		throw std::out_of_range("Invalid getEstimatedTimeToTarget id: " + std::to_string(id));

}

RoboCompGridder::Map GridderI::getMap(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getMapHandlers.size())
		return  getMapHandlers[id]();
	else
		throw std::out_of_range("Invalid getMap id: " + std::to_string(id));

}

RoboCompGridder::NavigationState GridderI::getNavigationState(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getNavigationStateHandlers.size())
		return  getNavigationStateHandlers[id]();
	else
		throw std::out_of_range("Invalid getNavigationState id: " + std::to_string(id));

}

RoboCompGridder::NavigationStatus GridderI::getNavigationStatus(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getNavigationStatusHandlers.size())
		return  getNavigationStatusHandlers[id]();
	else
		throw std::out_of_range("Invalid getNavigationStatus id: " + std::to_string(id));

}

RoboCompGridder::Result GridderI::getPaths(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, int maxPaths, bool tryClosestFreePoint, bool targetIsHuman, float safetyFactor, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getPathsHandlers.size())
		return  getPathsHandlers[id](source, target, maxPaths, tryClosestFreePoint, targetIsHuman, safetyFactor);
	else
		throw std::out_of_range("Invalid getPaths id: " + std::to_string(id));

}

RoboCompGridder::Pose GridderI::getPose(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getPoseHandlers.size())
		return  getPoseHandlers[id]();
	else
		throw std::out_of_range("Invalid getPose id: " + std::to_string(id));

}

RoboCompGridder::TPoint GridderI::getTarget(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getTargetHandlers.size())
		return  getTargetHandlers[id]();
	else
		throw std::out_of_range("Invalid getTarget id: " + std::to_string(id));

}

bool GridderI::hasReachedTarget(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < hasReachedTargetHandlers.size())
		return  hasReachedTargetHandlers[id]();
	else
		throw std::out_of_range("Invalid hasReachedTarget id: " + std::to_string(id));

}

bool GridderI::replanPath(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < replanPathHandlers.size())
		return  replanPathHandlers[id]();
	else
		throw std::out_of_range("Invalid replanPath id: " + std::to_string(id));

}

bool GridderI::resumeNavigation(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < resumeNavigationHandlers.size())
		return  resumeNavigationHandlers[id]();
	else
		throw std::out_of_range("Invalid resumeNavigation id: " + std::to_string(id));

}

bool GridderI::setGridDimensions(RoboCompGridder::TDimensions dimensions, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setGridDimensionsHandlers.size())
		return  setGridDimensionsHandlers[id](dimensions);
	else
		throw std::out_of_range("Invalid setGridDimensions id: " + std::to_string(id));

}

RoboCompGridder::Result GridderI::setLocationAndGetPath(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, RoboCompGridder::TPointVector freePoints, RoboCompGridder::TPointVector obstaclePoints, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setLocationAndGetPathHandlers.size())
		return  setLocationAndGetPathHandlers[id](source, target, freePoints, obstaclePoints);
	else
		throw std::out_of_range("Invalid setLocationAndGetPath id: " + std::to_string(id));

}

bool GridderI::setTarget(RoboCompGridder::TPoint target, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setTargetHandlers.size())
		return  setTargetHandlers[id](target);
	else
		throw std::out_of_range("Invalid setTarget id: " + std::to_string(id));

}

bool GridderI::setTargetWithOptions(RoboCompGridder::TPoint target, RoboCompGridder::NavigationOptions options, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setTargetWithOptionsHandlers.size())
		return  setTargetWithOptionsHandlers[id](target, options);
	else
		throw std::out_of_range("Invalid setTargetWithOptions id: " + std::to_string(id));

}

bool GridderI::startNavigation(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < startNavigationHandlers.size())
		return  startNavigationHandlers[id]();
	else
		throw std::out_of_range("Invalid startNavigation id: " + std::to_string(id));

}

void GridderI::stopNavigation(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < stopNavigationHandlers.size())
		 stopNavigationHandlers[id]();
	else
		throw std::out_of_range("Invalid stopNavigation id: " + std::to_string(id));

}

