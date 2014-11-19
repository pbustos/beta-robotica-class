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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void  sendData(const RoboCompJoystickAdapter::TData& data);
	

public slots:
 	void compute(); 	
	
private:
	
	struct FalconData
	{
		void update( const RoboCompJoystickAdapter::TData &data )
		{
			QMutexLocker m(&mu);
			falconData = data;
			if((getAxis() - falconDataAnt).norm2() < 0.1)
				newdata = false;
			else
				newdata = true;
			falconDataAnt = getAxis();
		};
		bool newData()
		{
			if( newdata )
				return true;
			else
				return false;
		}
		bool getButton()
		{
			for(auto b: falconData.buttons)
				if( b.clicked )
					return true;
			else 
				return false;
		}
		QVec getAxis()
		{
			QVec axes(3);
			int k=0;
			for( auto i: falconData.axes)
			{
				axes[k] = i.value;
				k++;
			}
			return axes;
		}
		void print()
		{	
			std::cout << "Id:" << falconData.id << std::endl;
			std::cout << "Axes:" << std::endl;
			for(auto i: falconData.axes)
				std::cout << "	axe:" << i.name << ". Valor:" << i.value << std::endl;
			std::cout << "Buttons:" << std::endl;
			for(auto i: falconData.buttons)
				std::cout << "	button:" << i.clicked << std::endl;
			std::cout << "velAxisIndex:" << falconData.velAxisIndex << std::endl;
			std::cout << "dirAxisIndex:" << falconData.dirAxisIndex << std::endl;
			
		}
		RoboCompJoystickAdapter::TData falconData;
		QVec falconDataAnt;
		QMutex mu;
		bool newdata;
	};

	FalconData falcon;
};

#endif