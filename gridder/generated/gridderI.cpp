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

	getClosestFreePointHandlers = {
		[this](auto a) { return worker->Gridder_getClosestFreePoint(a); }
	};

	getDimensionsHandlers = {
		[this]() { return worker->Gridder_getDimensions(); }
	};

	getMapHandlers = {
		[this]() { return worker->Gridder_getMap(); }
	};

	getPathsHandlers = {
		[this](auto a, auto b, auto c, auto d, auto e, auto f) { return worker->Gridder_getPaths(a, b, c, d, e, f); }
	};

	setGridDimensionsHandlers = {
		[this](auto a) { return worker->Gridder_setGridDimensions(a); }
	};

	setLocationAndGetPathHandlers = {
		[this](auto a, auto b, auto c, auto d) { return worker->Gridder_setLocationAndGetPath(a, b, c, d); }
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

