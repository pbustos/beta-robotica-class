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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	innerModel = std::make_shared<InnerModel>("/home/robocomp/robocomp/files/innermodel/betaWorldArm.xml");

	goHome();
	sleep(1);
	
	try 
	{ mList = jointmotor_proxy->getAllMotorParams(); }
	catch(const Ice::Exception &e)
	{ std::cout << e << std::endl;}
	
	joints << "shoulder_right_1"<<"shoulder_right_2"<<"shoulder_right_3"<<"elbow_right" << "wrist_right_1" << "wrist_right_2";
	// Check that these names are in mList
	motores = QVec::zeros(joints.size());

	connect(pushButton_left, SIGNAL(pressed()), this, SLOT(leftSlot()));
	connect(pushButton_right, SIGNAL(pressed()), this, SLOT(rightSlot()));
	connect(pushButton_up, SIGNAL(pressed()), this, SLOT(upSlot()));
	connect(pushButton_down, SIGNAL(pressed()), this, SLOT(downSlot()));
	connect(pushButton_front, SIGNAL(pressed()), this, SLOT(frontSlot()));
	connect(pushButton_back, SIGNAL(pressed()), this, SLOT(backSlot()));
	connect(pushButton_home, SIGNAL(pressed()), this, SLOT(goHome()));
	connect(pushButton_rot_left, SIGNAL(pressed()), this, SLOT(rotLeftSlot()));
	connect(pushButton_rot_right, SIGNAL(pressed()), this, SLOT(rotRightSlot()));
	connect(doubleSpinBox, SIGNAL(valueChanged(int)), this, SLOT(changeSpeed(int))); 
	
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
	RoboCompJointMotor::MotorStateMap mMap;
 	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap)
			innerModel->updateJointValue(QString::fromStdString(m.first),m.second.pos);			
		//std::cout << "--------------------------" << std::endl;
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}
	
	//Compute Jacobian for the chain of joints and using as tip "cameraHand" 
 	QMat jacobian = innerModel->jacobian(joints, motores, "cameraHand");
	
	RoboCompJointMotor::MotorGoalVelocityList vl;
	if( isPushed() )
	{
		try
		{
			QVec incs = jacobian.invert() * error;	
			int i=0;
			for(auto m: joints)
			{
				//RoboCompJointMotor::MotorGoalPosition mg = {mMap.find(m.toStdString())->second.pos + incs[i], 1.0, m.toStdString()};
				RoboCompJointMotor::MotorGoalVelocity vg{FACTOR*incs[i], 1.0, m.toStdString()};
				//ml.push_back(mg);
				vl.push_back(vg);
				i++;
			}
		}	
		catch(const QString &e)
		{ qDebug() << e << "Error inverting matrix";}
	}
	else //Stop the arm
	{
		for(auto m: joints)
		{
			RoboCompJointMotor::MotorGoalVelocity vg{0.0, 1.0, m.toStdString()};
			vl.push_back(vg);
		}
	}
	//Do the thing
	try
	{ 
		jointmotor_proxy->setSyncVelocity(vl);
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}
} 	
	
	
void SpecificWorker::goHome()
{
	RoboCompJointMotor::MotorStateMap mMap;
	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap)
		{
			RoboCompJointMotor::MotorGoalPosition mg = { innerModel->getJoint(m.first)->home, 1.0, m.first };
			jointmotor_proxy->setPosition(mg);
		}
		sleep(1);
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}	
}


bool SpecificWorker::isPushed()
{
	return (pushButton_left->isDown() or pushButton_right->isDown() 
			or pushButton_up->isDown() or pushButton_down->isDown()
			or pushButton_front->isDown() or pushButton_back->isDown());
}

//////////////////
/// SLOTS
/////////////////

void SpecificWorker::leftSlot()
{
	error = QVec::vec6(INCREMENT,0,0,0,0,0);
}

void SpecificWorker::rightSlot()
{
	error = QVec::vec6(-INCREMENT,0,0,0,0,0);
}

void SpecificWorker::frontSlot()
{
	error = QVec::vec6(0,-INCREMENT,0,0,0,0);
}

void SpecificWorker::backSlot()
{
	error = QVec::vec6(0,INCREMENT,0,0,0,0);
}

void SpecificWorker::upSlot()
{
	error = QVec::vec6(0,0,INCREMENT,0,0,0);
}

void SpecificWorker::downSlot()
{
	error = QVec::vec6(0,0,-INCREMENT,0,0,0);
}

void SpecificWorker::rotRightSlot()
{
	error = QVec::vec6(0,0,0,0,0,-0.1);
}

void SpecificWorker::rotLeftSlot()
{
	error = QVec::vec6(0,0,0,0,0,0.1);
}

void SpecificWorker::changeSpeed(int s)
{
	FACTOR = s;
}
