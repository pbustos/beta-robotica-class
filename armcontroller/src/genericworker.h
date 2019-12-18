/*
 *    Copyright (C)2019 by YOUR NAME HERE
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

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>

#include <QStateMachine>
#include <QState>
#include <CommonBehavior.h>

#include <SimpleArm.h>
#include <JointMotor.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompSimpleArm;
using namespace RoboCompJointMotor;

using TuplePrx = std::tuple<RoboCompJointMotor::JointMotorPrxPtr>;


class GenericWorker :
public QObject
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	JointMotorPrxPtr jointmotor_proxy;

	virtual void SimpleArm_closeFingers(float d) = 0;
	virtual void SimpleArm_moveTo(Pose6D pose) = 0;
	virtual void SimpleArm_openFingers(float d) = 0;
	virtual void SimpleArm_stop() = 0;

protected:
//State Machine
	QStateMachine defaultMachine;

	QState *computeState;
	QState *initializeState;
	QFinalState *finalizeState;

//-------------------------

	QTimer timer;
	int Period;

private:


public slots:
//Slots funtion State Machine
	virtual void sm_compute() = 0;
	virtual void sm_initialize() = 0;
	virtual void sm_finalize() = 0;

//-------------------------
	virtual void compute() = 0;
    virtual void initialize(int period) = 0;
	
signals:
	void kill();
//Signals for State Machine
	void t_initialize_to_compute();
	void t_compute_to_compute();
	void t_compute_to_finalize();

//-------------------------
};

#endif
