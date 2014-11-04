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


#include "reflexx.h"

Reflexx::Reflexx(RoboCompJointMotor::JointMotorPrx jointmotor_proxy, const QList<QVec> &jointValues, const QStringList &selectedMotors)
{
	this->jointmotor_proxy = jointmotor_proxy;
	this->selectedMotors = selectedMotors;
	
	ResultValue =   0 ;
	RML  =   NULL ;
	IP  =   NULL ;	
	OP =   NULL;	
	
	//Set number of motors
	NUMBER_OF_DOFS = selectedMotors.size();

	// ********************************************************************
	// Creating all relevant objects of the Reflexxes Motion Library    
	// ********************************************************************
	
	RML =   new ReflexxesAPI( NUMBER_OF_DOFS,  CYCLE_TIME_IN_SECONDS );
	IP  =   new RMLPositionInputParameters( NUMBER_OF_DOFS );
	OP  =   new RMLPositionOutputParameters( NUMBER_OF_DOFS );
	Flags.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;
	
	//Create a HASH table associating names with correlative indexes i
	int i=0;
	foreach( QString name, this->selectedMotors )
	{
		hashMotors[name] = i;
		i++;
	}
	
	//Initialize the internal structrures
	for(int i=0; i<NUMBER_OF_DOFS; i++)
	{
		IP->CurrentPositionVector->VecData[i] = jointValues[0][i];
		IP->CurrentVelocityVector->VecData[i] = 0.f;   
		IP->MaxVelocityVector->VecData[i] = MAX_VELOCITY;
		IP->SelectionVector->VecData[i] = true;
		IP->MaxAccelerationVector->VecData[i] = 300.0;
		IP->MaxJerkVector->VecData[i] = 300.0;
	}
}

Reflexx::~Reflexx()
{
// ********************************************************************
	// Deleting the objects of the Reflexxes Motion Library end terminating
	// the process

	delete  RML;
	delete  IP ;
	delete  OP ;
}

// void Reflexx::updateMotorState(RoboCompJointMotor::MotorStateMap motors)
// {
// 		// ********************************************************************
// 	// Set-up the input parameters
// 
// 	foreach( QString name, this->selectedMotors )
// 	{
// 		//QString name = QString::fromStdString(m.name);
// 		IP->CurrentPositionVector->VecData[hashMotors[name]] = motors[name.toStdString()].pos;
// 		IP->CurrentVelocityVector->VecData[hashMotors[name]] = motors[name.toStdString()].vel;   
// 		IP->MaxVelocityVector->VecData[hashMotors[name]] =  MAX_VELOCITY;
// 	}
// }

void Reflexx::setSyncPosition(const RoboCompJointMotor::MotorGoalPositionList &listGoals)
{
	foreach( RoboCompJointMotor::MotorGoalPosition mg, listGoals)
	{
		QString name = QString::fromStdString(mg.name);
		//qDebug() << "Reflexx::setSyncPosition -> goal set for " << QString::fromStdString(mg.name) << hashMotors[name] << IP->SelectionVector->VectorDimension;
		IP->TargetPositionVector->VecData[hashMotors[name]] = mg.position;
		IP->TargetVelocityVector->VecData[hashMotors[name]] = MAX_VELOCITY;
		IP->SelectionVector->VecData[hashMotors[name]] =  true;
	}
	//qDebug() << "Check " << IP->CheckForValidity();
}

void Reflexx::run()
{
	// ********************************************************************
	// Starting the control loop

	while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
	{
		// Calling the Reflexxes OTG algorithm
		ResultValue =   RML->RMLPosition( *IP, OP, Flags );
		
		if (ResultValue < 0)
		{
				printf("An error occurred (%d).\n", ResultValue );
				break;
		}
			
		foreach( QString name, this->selectedMotors )
		{
			qDebug() << "" << name << OP->NewPositionVector->VecData[hashMotors[name]] << OP->NewVelocityVector->VecData[hashMotors[name]];
		}
		
		
		// ****************************************************************
		// Feed the output values of the current control cycle back to 
		// input values of the next control cycle
		
		*IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
		*IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
		*IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;
		
		usleep(CYCLE_TIME_IN_SECONDS * 1000000);
	}

	//Deactivate motors
	for(int i=0; i<NUMBER_OF_DOFS; i++)
	{
		IP->SelectionVector = false;
	}
	
}