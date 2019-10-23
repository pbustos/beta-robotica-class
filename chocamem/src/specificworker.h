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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <iostream>
#include <fstream>
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "grid.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <stdlib.h>
#include <random>
const float threshold = 200; // millimeters
const float rot = -0.75;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	enum class State {idle, walk, turn, findObj, spiral};
	SpecificWorker::State actual_state;
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setState(SpecificWorker::State a_state);
	void RCISMousePicker_setPick(const Pick &myPick);

	/// Grid cell definition
		struct TCell
		{
			bool free;
			bool visited;
			QGraphicsRectItem* rect;
			
			// method to save the value
			void save(std::ostream &os) const {	os << free << " " << visited; };
			void read(std::istream &is) {	is >> free >> visited ;};
		};

public slots:
	void compute();
	void initialize(int period);

private:
	std::shared_ptr<InnerModel> innerModel;
	using TDim = Grid<TCell>::Dimensions;
	Grid<TCell> grid;
	QGraphicsScene scene;
	QGraphicsView view;
	void draw();
	QGraphicsRectItem *robot;
	QGraphicsEllipseItem *noserobot;
	
	int tilesize = 70;
	int xmin, xmax, ymin, ymax;

	RoboCompGenericBase::TBaseState bState;
	RoboCompLaser::TLaserData ldata;

	void updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata);
	void readRobotState(RoboCompLaser::TLaserData ldata);
	void idle();
	void walk(RoboCompLaser::TLaserData ldata);
	void turn(RoboCompLaser::TLaserData ldata);
	void findObstacle(RoboCompLaser::TLaserData ldata);
};

#endif
