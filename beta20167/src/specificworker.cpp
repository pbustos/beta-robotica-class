/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
	target = QVec::vec2(0,1000);
	timer.start(Period);
	return true;
	
}

void SpecificWorker::compute()
{
	if( active) 
	{
		try
		{
			differentialrobot_proxy->getBaseState(bState);
			qDebug() << bState.x << bState.z << bState.alpha;
			
			//Transform TARGET to Robot RS
			Rot2DC rot(bState.alpha);
			QVec res = (rot * ( target - QVec::vec2(bState.x, bState.z)));
			res.print("res");
				
			float vrot = atan2(res.x(), res.y());
			float vadv = res.norm2();
			
			if( vadv < 10)
			{
				vrot = 0;
				vadv = 0;
				active = false;
			}
			if( fabs(vrot) > 0.1 )
				vadv = 0;
			differentialrobot_proxy->setSpeedBase(vadv, vrot);
			
		}
		catch(const Ice::Exception &e)
		{
			std::cout << "Error reading from Robot" << e << std::endl;
		}
		catch(...)
		{
			std::cout << "Error in QMat probably" << std::endl;
		}
	}
}

////////////////////////////////////////////
////////////////////////////////////////////

void SpecificWorker::setPick(const Pick& myPick)
{
	target[0] = myPick.x;
	target[1] = myPick.z;
	active = true;
}






