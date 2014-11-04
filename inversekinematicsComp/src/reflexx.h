/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  Pablo Bustos <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef REFLEXX_H
#define REFLEXX_H

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <QtCore>
#include <JointMotor.h>
#include <innermodel/innermodel.h>

#define CYCLE_TIME_IN_SECONDS                   0.020
#define MAX_VELOCITY	0.3

class Reflexx : public QThread
{
	Q_OBJECT
	public:
		Reflexx(RoboCompJointMotor::JointMotorPrx jointmotor_proxy, const QList<QVec> &jointValues, const QStringList &selectedMotors);
		virtual ~Reflexx();
		void setSyncPosition(const RoboCompJointMotor::MotorGoalPositionList &listGoals);
		void updateMotorState(RoboCompJointMotor::MotorStateMap motors);
		void virtual run();
		
	private:
		int                         ResultValue;    
		ReflexxesAPI                *RML;        
		RMLPositionInputParameters  *IP;           
		RMLPositionOutputParameters *OP;                      
		RMLPositionFlags            Flags;            
		RoboCompJointMotor::JointMotorPrx jointmotor_proxy;
		QHash<QString,int> hashMotors;
		int NUMBER_OF_DOFS;
		QTimer timer;
		QStringList selectedMotors;
				
};

#endif // REFLEXX_H
