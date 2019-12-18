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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit t_compute_to_finalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//  THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }

	defaultMachine.start();
	
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	try 
	{ mList = jointmotor_proxy->getAllMotorParams(); }
	catch(const Ice::Exception &e)
	{ std::cout << e << std::endl;	}

	joints << "shoulder_right_1"<<"shoulder_right_2"<<"shoulder_right_3"<<"elbow_right" << "wrist_right_1" << "wrist_right_2";
	// Check that these names are in mList
	motores = QVec::zeros(joints.size());

	this->Period = period;
	timer.start(Period);
	emit this->t_initialize_to_compute();

}

void SpecificWorker::compute()
{
	try
	{
		readArmState();
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Arm" << e << std::endl;
	}
}

void SpecificWorker::sm_compute()
{
	std::cout<<"Entered state compute"<<std::endl;
	compute();
}

void SpecificWorker::sm_initialize()
{
	std::cout<<"Entered initial state initialize"<<std::endl;
}

void SpecificWorker::sm_finalize()
{
	std::cout<<"Entered final state finalize"<<std::endl;
}

//////////////////////////////////////////////////////////////////////77

void SpecificWorker::readArmState()
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
}
////////////////////////////////////////////////////////////////////////


void SpecificWorker::SimpleArm_openFingers(float d)
{
//implementCODE

}

void SpecificWorker::SimpleArm_moveTo(Pose6D pose)
{
//implementCODE

}

void SpecificWorker::SimpleArm_stop()
{
//implementCODE

}

void SpecificWorker::SimpleArm_closeFingers(float d)
{
//implementCODE

}


