/*
 *    Copyright (C) 2024 by YOUR NAME HERE
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
#include "genericworker.h"
/**
* \brief Default constructor
*/
GenericWorker::GenericWorker(TuplePrx tprx) : Ui_guiDlg()
{

	
	lidar3d_proxy = std::get<0>(tprx);
	omnirobot_proxy = std::get<1>(tprx);

	states.resize(STATES::NumberOfStates);
	states[STATES::Initialize] = new GRAFCETStep("Initialize", BASIC_PERIOD, nullptr, std::bind(&GenericWorker::initialize, this));
	states[STATES::Compute] = new GRAFCETStep("Compute", BASIC_PERIOD, std::bind(&GenericWorker::compute, this));
	states[STATES::Emergency] = new GRAFCETStep("Emergency", BASIC_PERIOD, std::bind(&GenericWorker::emergency, this));
	states[STATES::Restore] = new GRAFCETStep("Restore", BASIC_PERIOD, nullptr, std::bind(&GenericWorker::restore, this));

	states[STATES::Initialize]->addTransition(states[STATES::Initialize], SIGNAL(entered()), states[STATES::Compute]);
	states[STATES::Compute]->addTransition(this, SIGNAL(goToEmergency()), states[STATES::Emergency]);
	states[STATES::Emergency]->addTransition(this, SIGNAL(goToRestore()), states[STATES::Restore]);
	states[STATES::Restore]->addTransition(states[STATES::Restore], SIGNAL(entered()), states[STATES::Compute]);

	statemachine.addState(states[STATES::Initialize]);
	statemachine.addState(states[STATES::Compute]);
	statemachine.addState(states[STATES::Emergency]);
	statemachine.addState(states[STATES::Restore]);

	statemachine.setChildMode(QState::ExclusiveStates);;
	statemachine.setInitialState(states[STATES::Initialize]);


	#ifdef USE_QTGUI
		setupUi(this);
		show();
	#endif
}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{
	for (auto state : states) {
        delete state;
    }

}
void GenericWorker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
}

void GenericWorker::initializeWorker()
{
	statemachine.start();

	connect(&hibernationChecker, SIGNAL(timeout()), this, SLOT(hibernationCheck()));

	auto error = statemachine.errorString();
    if (error.length() > 0){
        qWarning() << error;
        throw error;
    }

}

/**
* \brief Change compute period
* @param nameState name state "Compute" or "Emergency"
* @param per Period in ms
*/
void GenericWorker::setPeriod(STATES state, int p)
{
	switch (state)
	{
	case STATES::Compute:
		this->period = p;
		states[STATES::Compute]->setPeriod(this->period);
		std::cout << "Period Compute changed " << p  << "ms" << std::endl<< std::flush;
		break;

	case STATES::Emergency:
		states[STATES::Emergency]->setPeriod(this->period);
		std::cout << "Period Emergency changed " << p << "ms" << std::endl<< std::flush;
		break;
	
	default:
		std::cerr<<"No change in the period, the state parameter must be 'Compute' or 'Emergency'."<< std::endl<< std::flush;
		break;
	}
}

int GenericWorker::getPeriod(STATES state)
{
	if (state < 0 || state >= STATES::NumberOfStates) {
        std::cerr << "Invalid state parameter." << std::endl << std::flush;
        return -1;
    }
	return states[state]->getPeriod();
}

void GenericWorker::hibernationCheck()
{
	//Time between activity to activate hibernation
    static const int HIBERNATION_TIMEOUT = 5000;

    static std::chrono::high_resolution_clock::time_point lastWakeTime = std::chrono::high_resolution_clock::now();
	static int originalPeriod = this->period;
    static bool isInHibernation = false;

	// Update lastWakeTime by calling a function
    if (hibernation)
    {
        hibernation = false;
        lastWakeTime = std::chrono::high_resolution_clock::now();

		// Restore period
        if (isInHibernation)
        {
            this->setPeriod(STATES::Compute, originalPeriod);
            isInHibernation = false;
        }
    }

    auto now = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastWakeTime);

	//HIBERNATION_TIMEOUT exceeded, change period
    if (elapsedTime.count() > HIBERNATION_TIMEOUT && !isInHibernation)
    {
        isInHibernation = true;
		originalPeriod = this->getPeriod(STATES::Compute);
        this->setPeriod(STATES::Compute, 500);
    }
}

