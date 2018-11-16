/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>
#include <thread>

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>

#include <CommonBehavior.h>

#include <Laser.h>
#include <GenericBase.h>
#include <DifferentialRobot.h>
#include <GenericBase.h>
#include <RCISMousePicker.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompDifferentialRobot;
using namespace RoboCompGenericBase;
using namespace RoboCompLaser;
using namespace RoboCompRCISMousePicker;

class CallBackTimer
{
	public:
			CallBackTimer()	:_execute(false)
			{}

			~CallBackTimer() 
			{
					if( _execute.load(std::memory_order_acquire) ) 
						stop();
			}
			void stop()
			{
					_execute.store(false, std::memory_order_release);
					if( _thd.joinable() )
							_thd.join();
			}
			void start(int interval, std::function<void(void)> func)
			{
					if( _execute.load(std::memory_order_acquire) ) 
						stop();
					
					_execute.store(true, std::memory_order_release);
					_thd = std::thread([this, interval, func]()
					{
							while (_execute.load(std::memory_order_acquire)) 
							{
									func();                   
									std::this_thread::sleep_for(
									std::chrono::milliseconds(interval));
							}
					});
			}
			bool is_running() const noexcept 
			{
					return ( _execute.load(std::memory_order_acquire) && 
									_thd.joinable() );
			}
	private:
			std::atomic<bool> _execute;
			std::thread _thd;
};


typedef map <string,::IceProxy::Ice::Object*> MapPrx;

class GenericWorker :
	#ifdef USE_QTGUI
	public QWidget, public Ui_guiDlg
	#else
	public QObject
	#endif
{
	Q_OBJECT
	public:
		GenericWorker(MapPrx& mprx);
		virtual ~GenericWorker();
		virtual void killYourSelf();
		virtual void setPeriod(int p);

		virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
		//QMutex *mutex;

		DifferentialRobotPrx differentialrobot_proxy;
		LaserPrx laser_proxy;

		virtual void setPick(const Pick &myPick) = 0;
		//virtual void compute() = 0;
		
		CallBackTimer cpptimer;

	protected:
		//QTimer *timer;
		int Period;

	private:
		

	public slots:
		virtual void compute() = 0;
		
	signals:
		void kill();
};

#endif
