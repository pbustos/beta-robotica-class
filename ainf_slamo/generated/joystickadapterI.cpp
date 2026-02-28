/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
#include "joystickadapterI.h"

JoystickAdapterI::JoystickAdapterI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	sendDataHandlers = {
		[this](auto a) { return worker->JoystickAdapter_sendData(a); }
	};

}


JoystickAdapterI::~JoystickAdapterI()
{
}


void JoystickAdapterI::sendData(RoboCompJoystickAdapter::TData data, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < sendDataHandlers.size())
		 sendDataHandlers[id](data);
	else
		throw std::out_of_range("Invalid sendData id: " + std::to_string(id));

}

