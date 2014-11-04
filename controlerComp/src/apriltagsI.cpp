/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#include "apriltagsI.h"

AprilTags0I::AprilTags0I(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


AprilTags0I::~AprilTags0I()
{
	// Free component resources here
}

// Component functions, implementation
void AprilTags0I::newAprilTag(const tagsList& tags, const Ice::Current&){
	worker->newAprilTag0(tags);
}



AprilTags1I::AprilTags1I(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


AprilTags1I::~AprilTags1I()
{
	// Free component resources here
}

// Component functions, implementation
void AprilTags1I::newAprilTag(const tagsList& tags, const Ice::Current&){
	worker->newAprilTag1(tags);
}


