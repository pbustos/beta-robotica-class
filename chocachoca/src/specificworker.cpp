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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	std::cout << std::boolalpha;   
	cpptimer.start(100, std::function<void(void)>(std::bind(&SpecificWorker::compute, this)));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	myparams = params;
	readyToGo = true;
	return true;
}

void SpecificWorker::initialize()
{
	try
	{
		RoboCompCommonBehavior::Parameter par = myparams.at("InnerModelPath");
		innerModel = std::make_shared<InnerModel>(par.value);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	
	// Scene
// 	scene.setSceneRect(-2500, -2500, 5000, 5000);
// 	view.setScene(&scene);
// 	view.scale(1, -1);
// 	view.setParent(scrollArea);
// 	//view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
// 	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
// 	
		grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{true, false, nullptr} );
 
		qDebug() << "Grid initialize ok";
	
	
// 		for(auto &[key, value] : grid)
// 		{
// 			value.rect = ui->scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));
// 			value.rect->setPos(key.x,key.z);
// 			//std::cout << key << std::endl;
// 		}

// 	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
// 	noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
// 	noserobot->setBrush(Qt::magenta);

			qDebug() << "Grid initialize ok ok ";
		
	target = QVec::vec3(0,0,0);
	
	//qDebug() << __FILE__ << __FUNCTION__ << "CPP " << __cplusplus;
	/*
 	connect(saveButton, SIGNAL(clicked()), this, SLOT(saveToFile()));
 	connect(pushButton, SIGNAL(clicked()), this, SLOT(readFromFile()));*/
	connect(this, SIGNAL(robotSetPoseSIGNAL(float, float, float) ), ui, SLOT(robotSetPoseSLOT(float,float,float)), Qt::QueuedConnection);
	connect(this, SIGNAL(uiShowSIGNAL()), ui, SLOT(showSLOT()), Qt::QueuedConnection);
	//connect(this, SIGNAL(initGridSIGNAL(int, Key&, TCell&)), ui, SLOT(initGridSLOT(int, Key&, TCell&)), Qt::QueuedConnection);
	
// 	for(auto &[key, value] : grid)
// 		emit initGridSIGNAL(tilesize, key, value);

}

void SpecificWorker::compute()
{
	if(!readyToGo) return;
	else if(firstTime) { initialize(); firstTime = false; }
	
	static RoboCompGenericBase::TBaseState bState;
 	try
 	{
 		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		
		//draw robot
//   		ui->robot->setPos(bState.x, bState.z);
//   		ui->robot->setRotation(-180.*bState.alpha/M_PI);
// 		ui->view.show();
		
			emit robotSetPoseSIGNAL(bState.x, bState.z, -180.*bState.alpha/M_PI);
			emit uiShowSIGNAL();
		
			//updateVisitedCells(bState.x, bState.z);
		//updateOccupiedCells(bState, ldata);
		
		//checkTransform(bState);
	
	}
 	catch(const Ice::Exception &e)
	{	std::cout  << e << std::endl; }
	
	//Resize world widget if necessary, and render the world
// 	if (view.size() != scrollArea->size())
// 			view.setFixedSize(scrollArea->width(), scrollArea->height());
// 	draw();
	
}

void SpecificWorker::saveToFile()
{
	grid.saveToFile(fileName);
}


void SpecificWorker::readFromFile()
{
// 	std::ifstream myfile;
// 	myfile.open(fileName, std::ifstream::in);
// 	if(!myfile.fail())
// 	{
// 		//grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{true, false, nullptr} );
// 		for( auto &[k,v] : grid)
// 			delete v.rect;
// 		grid.clear();
// 		Grid<TCell>::Key key; TCell value;
// 		myfile >> key >> value;
// 		while(!myfile.eof()) 
// 		{
// 			auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));;
// 			tile->setPos(key.x,key.z);
// 			value.rect = tile;
// 			grid.insert<TCell>(key,value);
// 			myfile >> key >> value;
// 		}
// 		myfile.close();	
// 		robot->setZValue(1);
// 		std::cout << grid.size() << " elements read from " << fileName << std::endl;
// 	}
// 	else
// 		throw std::runtime_error("Cannot open file");
}
		
void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	auto *n = innerModel->getNode<InnerModelLaser>(std::string("laser"));
	for(auto l: ldata)
	{
		auto r = n->laserTo(std::string("world"), l.dist, l.angle);	// r is in world reference system
		// we set the cell corresponding to r as occupied 
		auto [valid, cell] = grid.getCell(r.x(), r.z()); 
		if(valid)
			cell.free = false;
	}
}

void SpecificWorker::updateVisitedCells(int x, int z)
{
	static unsigned int cont = 0;
	auto [valid, cell] = grid.getCell(x, z); 
	if(valid)
	{
		auto &occupied = cell.visited;
		if(occupied)
		{
			occupied = false;
			cont++;
		}
		float percentOccupacy = 100. * cont / grid.size();
	}
}

//void SpecificWorker::draw()
//{
// 	for(auto &[key, value] : grid)
// 	{
// // 		if(value.visited == false)
// // 			value.rect->setBrush(Qt::lightGray);
// 		if(value.free == false)
// 			value.rect->setBrush(Qt::darkRed);
// 	}
// 			qDebug() << __FILE__ << __FUNCTION__;
// 
// 	view.show();
//}

void SpecificWorker::checkTransform(const RoboCompGenericBase::TBaseState &bState)
{
	auto r = innerModel->transform("base", target, "world");		// using InnerModel
	
	Rot2D rot(bState.alpha);																		// create a 2D clockwise rotation matrix
	QVec t = QVec::vec2(bState.x, bState.z);									  // create a 2D vector for robot translation
	QVec t2 = QVec::vec2(target.x(), target.z());								// create a 2D vector from the 3D target
	QVec q = rot.transpose() * ( t2 - t);												// multiply R_t * (y - T)
	qDebug() << target << r << q;
}

/////////////////////////////////////////////////////////77
///////// ROBOCOMP SUBSCRITPION
//////////////////////////////////////////////////////////

void SpecificWorker::setPick(const Pick &myPick)
{
	target[0] = myPick.x;
	target[2] = myPick.z;
	target[1] = 0;
	qDebug() << __FILE__ << __FUNCTION__ << myPick.x << myPick.z ;
}
