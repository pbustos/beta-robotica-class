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
	qDebug() << __FUNCTION__;
	try
	{
		bodyinversekinematics_proxy->goHome("ARM");
	}
	catch(const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
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
		if( falcon.getAxis().norm2() > 0.2 and falcon.getButton() )
		{
			RoboCompBodyInverseKinematics::Axis axis;
			QVec ax = falcon.getAxis();
			//QVec axN = ax.normalize();
			axis.x = ax.x(); axis.y = ax.y(); axis.z = ax.z();		
			//RoboCompBodyInverseKinematics::WeightVector weights;
			qDebug() << "enviando" << ax ;
			bodyinversekinematics_proxy->advanceAlongAxis("ARM",axis, ax.norm2()*200 );
		}
		//else
		//	bodyinversekinematics_proxy->stop("ARM");			
	}
	catch(const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
	
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};


void SpecificWorker::sendData(const RoboCompJoystickAdapter::TData& data)
{
	falcon.update(data);
	//falcon.print();
}
