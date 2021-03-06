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
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

    COUNT_DOWN = stoi(params["max_time"].value);
	return true;
}


void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	connect(pushButton, &QPushButton::clicked, this, &SpecificWorker::reset_time );

	time.start();
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        auto current = fm.addStep(bState.x, bState.z, bState.alpha);
        lcdNumber_percent->display(current);
        lcdNumber_time->display(COUNT_DOWN - time.elapsed()/1000);

    }
    catch(const Ice::Exception &ex)
    {
	    std::cout << ex << std::endl;
    }
}

void SpecificWorker::reset_time()
{
    time.restart();

}




