/*
 *    Copyright (C) 2019 by YOUR NAME HERE
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
#include "simplearmI.h"

SimpleArmI::SimpleArmI(GenericWorker *_worker)
{
	worker = _worker;
}


SimpleArmI::~SimpleArmI()
{
}

void SimpleArmI::openFingers(const float  d, const Ice::Current&)
{
	worker->SimpleArm_openFingers(d);
}

void SimpleArmI::moveTo(Pose6D pose, const Ice::Current&)
{
	worker->SimpleArm_moveTo(pose);
}

void SimpleArmI::stop(const Ice::Current&)
{
	worker->SimpleArm_stop();
}

void SimpleArmI::closeFingers(const float  d, const Ice::Current&)
{
	worker->SimpleArm_closeFingers(d);
}

