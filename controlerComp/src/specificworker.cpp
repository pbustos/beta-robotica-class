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
 
 #include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
	try
	{
		RoboCompBodyInverseKinematics::Axis axis;
		RoboCompBodyInverseKinematics::WeightVector weights;
		axis.x = 0; axis.y = -1; axis.z=-1;
		//bodyinversekinematics_proxy->setTargetPose6D("ARM", target, weights, 0);
		qDebug() << "enviando";
		bodyinversekinematics_proxy->advanceAlongAxis("ARM",axis, 300);
		qFatal("enviado");
		
	}
	catch(const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
	
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};
void SpecificWorker::newAprilTag0(const tagsList& tags){
	
}

void SpecificWorker::newAprilTag1(const tagsList& tags){
	
}
