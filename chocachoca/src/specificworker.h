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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "grid.h"

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		SpecificWorker(MapPrx& mprx);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		
		// Ice subscription
		void setPick(const Pick &myPick);

	public slots:
		void compute();

	private:
		std::shared_ptr<InnerModel> innerModel;
		QGraphicsScene scene;
		QGraphicsView view;
		void draw();
		QGraphicsRectItem *robot;
		QGraphicsEllipseItem *noserobot;
		QVec target;
		
		void updateVisitedCells(int x, int z);
		void updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata);
		void checkTransform(const RoboCompGenericBase::TBaseState &bState);
		
		/// Grid
		using TCell = std::tuple<uint, bool, QGraphicsRectItem*, bool>;
		constexpr static int cid = 0, cvisited = 1, crect = 2, cfree = 3;
		using TDim = Grid<TCell>::Dimensions;
		Grid<TCell> grid;
		

};

#endif
