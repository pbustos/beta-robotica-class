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

	innerModel = InnerModelMgr(std::make_shared<InnerModel>("/home/robocomp/robocomp/files/innermodel/betaWorldArm.xml"));
	//innerModel = InnerModel("/home/robocomp/robocomp/files/innermodel/betaWorldArm.xml");  

	try 
	{ mList = jointmotor_proxy->getAllMotorParams();}
	catch(const Ice::Exception &e){ std::cout << e << std::endl;}
	
	joints << "shoulder_right_1"<<"shoulder_right_2"<<"shoulder_right_3"<<"elbow_right";
	// Check that these names are in mList
	motores = QVec::zeros(joints.size());

	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
 	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap)
		{
			innerModel->updateJointValue(QString::fromStdString(m.first),m.second.pos);
			std::cout << m.first << "		" << m.second.pos << std::endl;
		}
		std::cout << "--------------------------" << std::endl;
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}
	
 	QMat jacobian = innerModel->jacobian(joints, motores, "arm_right_6");
 	jacobian.print("jacobian");
} 	
	
	



