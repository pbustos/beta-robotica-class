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
SpecificWorker::SpecificWorker ( MapPrx& mprx ) : GenericWorker ( mprx )
{
	state = State::INIT;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{
	innerModel= new InnerModel ( "/home/robocomp/robocomp/files/innermodel/simpleworld.xml" );
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
// 	const float threshold = 420; //millimeters
// 	float rot = 0.6;  //rads per second

	try 
	{
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
		RoboCompDifferentialRobot::TBaseState bState;
		differentialrobot_proxy->getBaseState ( bState );
		innerModel->updateTransformValues ( "base", bState.x,0,bState.z,0,bState.alpha,0 );
		QVec ini;
		
		switch ( state ) 
		{
			case State::INIT:
				if ( pick.active ) 
				{
					qDebug() << "INIT to GOTO";
					ini = QVec::vec3(bState.x, 0, bState.z);
					linea = QLine2D( ini, pick.getPose() );
					state=State::GOTO;
				}
					break;
			case State::GOTO:
					move( ldata );
					break;
			case State::BUGINIT:
					buginit( ldata,bState );
					break;
			case State::BUG:
					bug( ldata,bState );
					break;
			case State::END:
					break;
			}
		} 
		catch ( const Ice::Exception &ex ) 
		{
			std::cout << ex << std::endl;
		}
}

void SpecificWorker::move( const TLaserData &tLaser )
{
	QVec tr = innerModel->transform ( "base",pick.getPose(),"world" );

	float angle = atan2 ( tr.x(),tr.z() );
	float distance = tr.norm2();

	//Check if already at target
	if ( distance <= 100 )
	{
		pick.setActive ( false );
		qDebug() << "FINISH: GOTO TO INIT";
		state= State::INIT;
		differentialrobot_proxy->stopBase();
		return;
	}

	//Check if obstacle ahead
	if( obstacle ( tLaser ) )
	{
		state=State::BUGINIT;
		qDebug() << "Obstacle detected, GOTO to BUGINIT";
		return;
	}

	if ( abs ( angle ) > 0.05 )
		distance = 0;
	if( distance > 300) distance = 300;

	try
	{
		differentialrobot_proxy->setSpeedBase(distance, angle);
	}
	catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
	}


void SpecificWorker::buginit ( const TLaserData& ldata,const TBaseState& bState )
{
	QVec posi = QVec::vec3(bState.x, 0., bState.z);
	distanciaAnterior = fabs(linea.perpendicularDistanceToPoint(posi));
	if( obstacle(ldata) == false)
	{
		state = State::BUG;
		qDebug() << "from BUGINIT to BUG";
		return;
	}
	try
	{
		differentialrobot_proxy->setSpeedBase(0, 0.3);
	}
	catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
	}

void SpecificWorker::bug( const TLaserData &ldata,const TBaseState &bState )
{
	const float alpha = log ( 0.1 ) /log ( 0.3 ); //amortigua /corte
	float dist = obstacleLeft(ldata);
	float diffToline = distanceToLine(bState);
	//qDebug()<<diffToline;
	
	//Check if target is visible and the robot is approaching the line, to a
	if ( targetAtSight (ldata) )
	{
		state = State::GOTO;
		qDebug() << "Target visible: from BUG to GOTO";
		return;
	}
	
	//Check if close to the line and distance is reducing
	if (distanciaAnterior < 100 and diffToline < 0)
	{
		state = State::GOTO;
		qDebug() << "Crossing the line: from BUG to GOTO";
		return;
	}
	
	//Check if there is an obstacle ahead
	if ( obstacle (ldata) )
	{
		state = State::BUGINIT;
		qDebug() << "from BUG to BUGINIT";
		return;
	}

	float k=0.1;  // pendiente de la sigmoide
	float vrot =  -((1./(1. + exp(-k*(dist - 450.))))-1./2.);		//sigmoide para meter vrot entre -0.5 y 0.5. La k ajusta la pendiente.
	float vadv = 350 * exp ( - ( fabs ( vrot ) * alpha ) ); 		//gaussiana para amortiguar la vel. de avance en funcion de vrot
	qDebug() << vrot << vadv;
	//vrot *= 0.3;
	differentialrobot_proxy->setSpeedBase ( vadv ,vrot );
}

bool SpecificWorker::targetAtSight ( TLaserData ldata )
{
	QPolygon poly;
	for ( auto l: ldata )
	{
		QVec r = innerModel->laserTo ( "world","laser",l.dist,l.angle );
		QPoint p ( r.x(),r.z() );
		poly << p;
	}
	QVec targetInRobot = innerModel->transform("base", pick.getPose(), "world");
	float dist = targetInRobot.norm2();
	int veces = int(dist / 200);  //number of times the robot semilength fits in the robot-to-target distance
	float landa = 1./veces;
	
	QList<QPoint> points;
	points << QPoint(pick.getPose().x(),pick.getPose().z());  //Add target
	
	//Add points along lateral lines of robot
	for (float i=landa; i<= 1.; i+=landa)
	{
		QVec point = targetInRobot*(T)landa;
		QVec pointW = innerModel->transform("world", point ,"base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innerModel->transform("world", point - QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innerModel->transform("world", point + QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
	}
	foreach( QPoint p, points)
	{
		if( poly.containsPoint(p , Qt::OddEvenFill) == false)
			return false;
	}
	return true;
}

/////////////
//AUXILIARY METHODS
/////////////

float SpecificWorker::distanceToLine(const TBaseState& bState)
{
	QVec posi = QVec::vec3(bState.x, 0., bState.z);
	float distanciaEnPunto = fabs(linea.perpendicularDistanceToPoint(posi));
	float diff = distanciaEnPunto - distanciaAnterior;
	distanciaAnterior = distanciaEnPunto;
	return diff;
}  

float SpecificWorker::obstacleLeft(const TLaserData& tlaser)
{
	const int laserpos = 85;
	float min = tlaser[laserpos].dist;
	for(int i=laserpos-2; i<laserpos+2;i++)
	{
		if (tlaser[i].dist < min)
			min = tlaser[i].dist;
	}
	return min;
}

void SpecificWorker::stopRobot()
{
  try
  {
		differentialrobot_proxy->stopBase();
  }
  catch ( const Ice::Exception &ex )
  {	std::cout << ex << std::endl; }
}

bool SpecificWorker::obstacle ( TLaserData tLaser )
{
	const int offset = 30;
	const int minDist = 350;
	
	//sort laser data from small to large distances using a lambda function.
	std::sort ( tLaser.begin() + offset, tLaser.end()- offset, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ){	return a.dist < b.dist;});
	return ( tLaser[offset].dist < minDist );
}

///////////////////////////////////
//// SERVANTS FOR ICE INTERFACE
//////////////////////////////////

void SpecificWorker::setPick ( const Pick &mypick )
{
    qDebug() << "New target selected: " << mypick.x << mypick.z;
    pick.copy ( mypick.x,mypick.z );
    pick.setActive ( true );
    state = State::INIT;
}


