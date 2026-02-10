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
#ifndef GRIDDER_H
#define GRIDDER_H

// Ice includes
#include <Ice/Ice.h>
#include <Gridder.h>

#include "../src/specificworker.h"


class GridderI : public virtual RoboCompGridder::Gridder
{
public:
	GridderI(GenericWorker *_worker, const size_t id);
	~GridderI();

	bool IsPathBlocked(RoboCompGridder::TPath path, const Ice::Current&);
	bool LineOfSightToTarget(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, float robotRadius, const Ice::Current&);
	RoboCompGridder::TPoint getClosestFreePoint(RoboCompGridder::TPoint source, const Ice::Current&);
	RoboCompGridder::TDimensions getDimensions(const Ice::Current&);
	RoboCompGridder::Map getMap(const Ice::Current&);
	RoboCompGridder::Result getPaths(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, int maxPaths, bool tryClosestFreePoint, bool targetIsHuman, float safetyFactor, const Ice::Current&);
	bool setGridDimensions(RoboCompGridder::TDimensions dimensions, const Ice::Current&);
	RoboCompGridder::Result setLocationAndGetPath(RoboCompGridder::TPoint source, RoboCompGridder::TPoint target, RoboCompGridder::TPointVector freePoints, RoboCompGridder::TPointVector obstaclePoints, const Ice::Current&);

private:

	GenericWorker *worker;
	size_t id;

	// Array handlers for each method
	std::array<std::function<bool(RoboCompGridder::TPath)>, 1> IsPathBlockedHandlers;
	std::array<std::function<bool(RoboCompGridder::TPoint, RoboCompGridder::TPoint, float)>, 1> LineOfSightToTargetHandlers;
	std::array<std::function<RoboCompGridder::TPoint(RoboCompGridder::TPoint)>, 1> getClosestFreePointHandlers;
	std::array<std::function<RoboCompGridder::TDimensions(void)>, 1> getDimensionsHandlers;
	std::array<std::function<RoboCompGridder::Map(void)>, 1> getMapHandlers;
	std::array<std::function<RoboCompGridder::Result(RoboCompGridder::TPoint, RoboCompGridder::TPoint, int, bool, bool, float)>, 1> getPathsHandlers;
	std::array<std::function<bool(RoboCompGridder::TDimensions)>, 1> setGridDimensionsHandlers;
	std::array<std::function<RoboCompGridder::Result(RoboCompGridder::TPoint, RoboCompGridder::TPoint, RoboCompGridder::TPointVector, RoboCompGridder::TPointVector)>, 1> setLocationAndGetPathHandlers;

};

#endif
