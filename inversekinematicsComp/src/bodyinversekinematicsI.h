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
#ifndef BODYINVERSEKINEMATICSI_H
#define BODYINVERSEKINEMATICSI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <BodyInverseKinematics.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompBodyInverseKinematics;

class BodyInverseKinematicsI : public QObject , public virtual RoboCompBodyInverseKinematics::BodyInverseKinematics
{
Q_OBJECT
public:
	BodyInverseKinematicsI( GenericWorker *_worker, QObject *parent = 0 );
	~BodyInverseKinematicsI();
	void  setTargetPose6D(const string& bodyPart, const Pose6D& target, const WeightVector& weights, Ice::Float radius, const Ice::Current& = Ice::Current());
void  pointAxisTowardsTarget(const string& bodyPart, const Pose6D& target, const Axis& ax, bool axisConstraint, Ice::Float axisAngleConstraint, const Ice::Current& = Ice::Current());
void  advanceAlongAxis(const string& bodyPart, const Axis& ax, Ice::Float dist, const Ice::Current& = Ice::Current());
void  setFingers(Ice::Float d, const Ice::Current& = Ice::Current());
void  goHome(const string& part, const Ice::Current& = Ice::Current());
void  setRobot(Ice::Int type, const Ice::Current& = Ice::Current());
TargetState getState(const string& part, const Ice::Current& = Ice::Current());
void  stop(const string& part, const Ice::Current& = Ice::Current());
void  setNewTip(const string& part, const string& transform, const Pose6D& pose, const Ice::Current& = Ice::Current());
void  setJoint(const string& joint, Ice::Float speed, Ice::Float maxSpeed, const Ice::Current& = Ice::Current());


	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif