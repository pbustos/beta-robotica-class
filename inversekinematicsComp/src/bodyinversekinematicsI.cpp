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
#include "bodyinversekinematicsI.h"

BodyInverseKinematicsI::BodyInverseKinematicsI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


BodyInverseKinematicsI::~BodyInverseKinematicsI()
{
	// Free component resources here
}

// Component functions, implementation
void BodyInverseKinematicsI::setTargetPose6D(const string& bodyPart, const Pose6D& target, const WeightVector& weights, Ice::Float radius, const Ice::Current&){
	worker->setTargetPose6D(bodyPart,target,weights,radius);
}

void BodyInverseKinematicsI::pointAxisTowardsTarget(const string& bodyPart, const Pose6D& target, const Axis& ax, bool axisConstraint, Ice::Float axisAngleConstraint, const Ice::Current&){
	worker->pointAxisTowardsTarget(bodyPart,target,ax,axisConstraint,axisAngleConstraint);
}

void BodyInverseKinematicsI::advanceAlongAxis(const string& bodyPart, const Axis& ax, Ice::Float dist, const Ice::Current&){
	worker->advanceAlongAxis(bodyPart,ax,dist);
}

void BodyInverseKinematicsI::setFingers(Ice::Float d, const Ice::Current&){
	worker->setFingers(d);
}

void BodyInverseKinematicsI::goHome(const string& part, const Ice::Current&){
	worker->goHome(part);
}

void BodyInverseKinematicsI::setRobot(Ice::Int type, const Ice::Current&){
	worker->setRobot(type);
}

TargetState BodyInverseKinematicsI::getState(const string& part, const Ice::Current&){
	return worker->getState(part);
}

void BodyInverseKinematicsI::stop(const string& part, const Ice::Current&){
	worker->stop(part);
}

void BodyInverseKinematicsI::setNewTip(const string& part, const string& transform, const Pose6D& pose, const Ice::Current&){
	worker->setNewTip(part,transform,pose);
}

void BodyInverseKinematicsI::setJoint(const string& joint, Ice::Float speed, Ice::Float maxSpeed, const Ice::Current&){
	worker->setJoint(joint,speed,maxSpeed);
}


