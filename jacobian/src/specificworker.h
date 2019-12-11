/*
 *    Copyright (C)2017 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#define INCREMENT 10

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute();
	void leftSlot();
	void rightSlot();
	void upSlot();
	void downSlot();
	void frontSlot();
	void backSlot();
	void rotLeftSlot();
	void rotRightSlot();
	void goHome();
	void changeSpeed(int);
	void initialize(int period);
	
private:
	std::shared_ptr<InnerModel> innerModel;
	RoboCompJointMotor::MotorParamsList mList;
	QStringList joints;
	QVec motores;
	QVec error;
	bool pushedButton = false;
	int FACTOR = 1;
	
	bool isPushed();
};

#endif
