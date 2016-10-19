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
	innermodel = new InnerModel("/home/robocomp/files/innermodel/simpleworld.xml");
	
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		qDebug() << bState.x << bState.z << bState.alpha;
		
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Camera" << e << std::endl;
	}
}


void SpecificWorker::checkRobotInLaser()
{
	QVec wd;
	std::vector<Point> points, res;
	for (auto &ld : laserData)
	{
		wd = innermodel->laserTo("world", "laser", ld.dist, ld.angle); //OPTIMIZE THIS FOR ALL CLASS METHODS
		points.push_back(Point(wd.x(), wd.z()));
	}
	res = simPath.simplifyWithRDP(points, 70);  ///PARAMS
	//qDebug() << __FUNCTION__ << "laser polygon after simp" << res.size();

	// Create a QPolygon so we can check if robot outline falls inside
	QPolygonF polygon;
	for (auto &p: res)
		polygon << QPointF(p.x, p.y);

	
}




