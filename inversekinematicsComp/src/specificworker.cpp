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

#include "specificworker.h"
#include <qt4/QtCore/QMutexLocker>


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 							CONSTRUCTORES Y DESTRUCTORES DE LA CLASE 								*
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 * @brief Default Constructor
 */
SpecificWorker::SpecificWorker(MapPrx& mprx, QWidget *parent) : GenericWorker(mprx)
{
	correlativeID = 0;		//Unique ID to name provisional targets
	hide();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	fichero.close();
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 							MÉTODOS DE INICIALIZACIÓN DE LA CLASE 									*
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * @brief SET PARAMS
 * Method called by the thread Monitor to pass the configuration parmaeters read from the config file
 * TODO Config con la lista de joints de las distintas partes del robot y crear el mapa de las partes
 * del cuerpo del robot AQUÍ.
 *
 * @param params ...
 * @return bool
 */
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("LEFTARM") ;
//
// 		qFatal("FARY");
//
// 	}catch(std::exception e) {qFatal("Error al leer la cadena de motores");	}
//

	try
	{
		// Leemos donde está el InnerModel. Si existe cargamos el InnerModel en nuestra variable de clase
		// y convertimos los milímitros en metros. Si no existe lanzamos mensage de error y salimos.
		RoboCompCommonBehavior::Parameter par = params.at("BIK.InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
			convertInnerModelFromMilimetersToMeters(innerModel->getRoot());
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;
		}
		else
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
			qFatal("Exiting now.");
		}
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}


	//timer.start(Period);
	init();
	timer.start(50);
	return true;
}

/**
 * @brief Initializing procedures to be done once params are read
 *
 * @return void
 */
void SpecificWorker::init()
{
	
	qDebug() <<__FUNCTION__;
	
	// RECONFIGURABLE PARA CADA ROBOT: Listas de motores de las distintas partes del robot
	listaBrazo 	<< "shoulder_right_1" << "shoulder_right_2" << "shoulder_right_3" << "elbow_right"<< "wrist_right_1" << "wrist_right_2" <<  "finger_right_1"  << "finger_right_2";
	listaMotores 	= listaBrazo;

	// PREPARA LA CINEMATICA INVERSA: necesita el innerModel, los motores y el tip:
	QString tip = "tip";

	IK_Brazo = new Cinematica_Inversa(innerModel, listaBrazo, tip);
	
	// CREA EL MAPA DE PARTES DEL CUERPO: por ahora los brazos.
	bodyParts.insert("ARM", BodyPart(innerModel, IK_Brazo, "ARM", tip, listaBrazo));

	qDebug() << "hola";
	
	//Initialize proxy to RCIS
	proxy = jointmotor0_proxy;
	actualizarInnermodel(listaMotores);  // actualizamos los ángulos de los motores del brazo derecho
	qDebug() << "hola2";
	//goHomePosition(listaMotores);
 	foreach(BodyPart p, bodyParts)
		goHome(p.getPartName().toStdString());
		
	//setFingers(0);    //XML DEPENDEND!!!
	//sleep(1);
	
	actualizarInnermodel(listaMotores);

// 	innerModel->transform("world", QVec::zeros(3),tipRight).print("RightTip in World");
// 	innerModel->transform("world", QVec::zeros(3),tipLeft).print("LeftTip in World");
// 	innerModel->transform("world", QVec::zeros(3),"mugTag").print("mug in World");


	//Open file to write errors
	fichero.open("errores.txt", ios::out);

	//OMPL path-Planning initialization
	QList<QPair<float, float > > limits;
	limits.append(qMakePair((float)-0.4,(float)0.4)); 	 //x in robot RS
	limits.append(qMakePair((float)0.2,(float)1.4)); 	 //y
	limits.append(qMakePair((float)-0.2,(float)1.f));  	 //z

	sampler.initialize3D(innerModel, limits);
	planner = new PlannerOMPL(*innerModel);
	planner->initialize(&sampler);


	qDebug();
	qDebug() << "---------------------------------";
	qDebug() << "BodyInverseKinematics --> Waiting for requests!";
}


/**
 * @brief Transforms all InnerModel in mm to meters until BIK can operate in meters directly
 *
 * @param node starting node of InnerModel
 * @return void
 */
void SpecificWorker::convertInnerModelFromMilimetersToMeters(InnerModelNode* node)
{
	const  float FACTOR = 1000.f;

	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelTransform *transformation;
	InnerModelJoint *joint;

	// Find out which kind of node are we dealing with
	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))   //Aquí se incluyen Transform, Joint, PrismaticJoint, DifferentialRobot
	{
		if( (joint = dynamic_cast<InnerModelJoint *>(node)) == false)
		{
			transformation->setTr(transformation->getTr() / FACTOR);
			qDebug() << transformation->id << transformation->getTr();
		}
		//qDebug() << node->id << node->getTr();
		for(int i=0; i<node->children.size(); i++)
		{
			convertInnerModelFromMilimetersToMeters(node->children[i]);
		}
	}
	else if ((plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		plane->point = plane->point / FACTOR;
		plane->width /= FACTOR;
		plane->height /= FACTOR;
		plane->depth /= FACTOR;
		printf("%s --------------------> %f %f %f\n", plane->id.toStdString().c_str(), plane->width, plane->height, plane->depth);
plane->collisionObject->computeAABB();
fcl::AABB a1 = plane->collisionObject->getAABB();
fcl::Vec3f v1 = a1.center();
printf("%s -------------------->      (%f,  %f,  %f) --- [%f , %f , %f]\n", plane->id.toStdString().c_str(), v1[0], v1[1], v1[2], a1.width(), a1.height(), a1.depth());


		// SCALE FACTOR
		{
				std::vector<fcl::Vec3f> vertices;
				vertices.push_back(fcl::Vec3f(-plane->width/2., +plane->height/2., -plane->depth/2.)); // Front NW
				vertices.push_back(fcl::Vec3f(+plane->width/2., +plane->height/2., -plane->depth/2.)); // Front NE
				vertices.push_back(fcl::Vec3f(-plane->width/2., -plane->height/2., -plane->depth/2.)); // Front SW
				vertices.push_back(fcl::Vec3f(+plane->width/2., -plane->height/2., -plane->depth/2.)); // Front SE
				vertices.push_back(fcl::Vec3f(-plane->width/2., +plane->height/2., +plane->depth/2.)); // Back NW
				vertices.push_back(fcl::Vec3f(+plane->width/2., +plane->height/2., +plane->depth/2.)); // Back NE
				vertices.push_back(fcl::Vec3f(-plane->width/2., -plane->height/2., +plane->depth/2.)); // Back SW
				vertices.push_back(fcl::Vec3f(+plane->width/2., -plane->height/2., +plane->depth/2.)); // Back SE

				osg::Matrix r;
				r.makeRotate(osg::Vec3(0, 0, 1), osg::Vec3(plane->normal(0), plane->normal(1), -plane->normal(2)));
				QMat qmatmat(4,4);
				for (int rro=0; rro<4; rro++)
					for (int cco=0; cco<4; cco++)
						qmatmat(rro,cco) = r(rro,cco);

				for (size_t i=0; i<vertices.size(); i++)
				{
					fcl::Vec3f v = vertices[i];
					const QVec rotated = (qmatmat*(QVec::vec3(v[0], v[1], v[2]).toHomogeneousCoordinates())).fromHomogeneousCoordinates();
					vertices[i] = fcl::Vec3f(rotated(0)+plane->point(0), rotated(1)+plane->point(1), rotated(2)+plane->point(2));
				}

				std::vector<fcl::Triangle> triangles;
				triangles.push_back(fcl::Triangle(0,1,2)); // Front
				triangles.push_back(fcl::Triangle(1,2,3));
				triangles.push_back(fcl::Triangle(4,5,6)); // Back
				triangles.push_back(fcl::Triangle(5,6,7));
				triangles.push_back(fcl::Triangle(4,0,6)); // Left
				triangles.push_back(fcl::Triangle(0,6,2));
				triangles.push_back(fcl::Triangle(5,1,7)); // Right
				triangles.push_back(fcl::Triangle(1,7,3));
				triangles.push_back(fcl::Triangle(5,1,4)); // Top
				triangles.push_back(fcl::Triangle(1,4,0));
				triangles.push_back(fcl::Triangle(2,3,6)); // Bottom
				triangles.push_back(fcl::Triangle(3,6,7));

				fclMesh = FCLModelPtr(new FCLModel());
				fclMesh->beginModel();
				fclMesh->addSubModel(vertices, triangles);
				fclMesh->endModel();
				plane->collisionObject = new fcl::CollisionObject(fclMesh);
		}
plane->collisionObject->computeAABB();
a1 = plane->collisionObject->getAABB();
v1 = a1.center();
printf("%s -------------------->      (%f,  %f,  %f) --- [%f , %f , %f]\n", plane->id.toStdString().c_str(), v1[0], v1[1], v1[2], a1.width(), a1.height(), a1.depth());
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)))
	{
		mesh->tx /= FACTOR; mesh->ty /= FACTOR; mesh->tz /= FACTOR;
		mesh->scalex /= FACTOR; mesh->scaley /= FACTOR; mesh->scalez /= FACTOR;
		// SCALE FACTOR
		{
			osg::Node *osgnode_ = osgDB::readNodeFile(mesh->meshPath.toStdString());
			if (not osgnode_)
			{
				printf("Could not open: '%s'.\n", mesh->meshPath.toStdString().c_str());
			}
			else
			{
				// Instanciate the vector of vertices and triangles (that's what we are looking for)
				std::vector<fcl::Vec3f> vertices;
				std::vector<fcl::Triangle> triangles;
				CalculateTriangles calcTriangles(&vertices, &triangles);
				osgnode_->accept(calcTriangles);

				// Get the internal transformation matrix of the mesh
				RTMat rtm(mesh->rx, mesh->ry, mesh->rz, mesh->tx, mesh->ty, mesh->tz);
				// Transform each of the read vertices
				for (size_t i=0; i<vertices.size(); i++)
				{
					fcl::Vec3f v = vertices[i];
					const QMat v2 = (rtm * QVec::vec3(v[0]*mesh->scalex, v[1]*mesh->scaley, -v[2]*mesh->scalez).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
					vertices[i] = fcl::Vec3f(v2(0), v2(1), v2(2));
				}
				// Associate the read vertices and triangles vectors to the FCL collision model object
				fclMesh = FCLModelPtr(new FCLModel());
				fclMesh->beginModel();
				fclMesh->addSubModel(vertices, triangles);
				fclMesh->endModel();
				mesh->collisionObject = new fcl::CollisionObject(fclMesh);
			}
		}
	}
}

void SpecificWorker::compute2()
{
	static int i=0;
	printf("%d\n", i++);
	
	actualizarInnermodel(listaMotores); //actualizamos TODOS los motores y la posicion de la base.
	if (sampler.isStateValidQ(innerModel->transform("world","munon_t")))
	{
			
	}
		else
			qDebug() << __FUNCTION__ << "collide ";;
}

	//if (innerModel->collide("munonMesh", restNodes[out]))
	
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										SLOTS DE LA CLASE											*
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void SpecificWorker::compute( )
{
	actualizarInnermodel(listaMotores); //actualizamos TODOS los motores y la posicion de la base.
	QMap<QString, BodyPart>::iterator iterador;
	
	for( iterador = bodyParts.begin(); iterador != bodyParts.end(); ++iterador)
	{
		if(iterador.value().noTargets() == false)
		{
			Target &target = iterador.value().getHeadFromTargets();
			target.getPose().print("TARGET");
	
 			if ( target.getType() == Target::TargetType::ALIGNAXIS or target.getType() == Target::TargetType::ADVANCEAXIS or  
						targetHasAPlan( *innerModel, target ) == true)
				{
					target.annotateInitialTipPose();
					target.setInitialAngles(iterador.value().getMotorList());
					createInnerModelTarget(target);  	//Crear "target" online y borrarlo al final para no tener que meterlo en el xml
					target.print("BEFORE PROCESSING");

					iterador.value().getInverseKinematics()->resolverTarget(target);
				
					if(target.getError() <= 0.9 and target.isAtTarget() == false) //local goal achieved: execute the solution
					{
						moveRobotPart(target.getFinalAngles(), iterador.value().getMotorList());
						//Acumulamos los angulos en una lista en bodyPart para lanzarlos con Reflexx
						iterador.value().addJointStep(target.getFinalAngles());
						usleep(20000);
						target.setExecuted(true);
					}
					else
					{
						target.markForRemoval(true);
					}
					actualizarInnermodel(listaMotores); 			//actualizamos TODOS los motores.	La translación de la base no se actualiza!!!!
					target.annotateFinalTipPose();
					removeInnerModelTarget(target);
					target.print("AFTER PROCESSING");
		//		}
			}
// 				if( target.isChopped() == false)
// 				{
// 					doReflexxes( iterador.value().getJointStepList(), iterador.value().getMotorList());
// 				}
			if(target.isChopped() == false or target.isMarkedforRemoval() == true or target.isAtTarget() )
			{
					mutex->lock();
						iterador.value().removeHeadFromTargets(); //eliminamos el target resuelt
						iterador.value().cleanJointStep();
						
					mutex->unlock();
			}
		}//if
	}//for
}


bool SpecificWorker::targetHasAPlan(InnerModel &innerModel,  Target& target)
{
	//Convert to ROBOT referece frame to faciitate SAMPLING procedure
	QVec origin = innerModel.transform("robot","grabPositionHandR");
	QVec targetR = innerModel.transform("robot", target.getPose(), "world");

	qDebug() << __FUNCTION__ << "Origin:" << origin << ". Target:" << target.getTranslation() << target.getHasPlan();

	if( target.getHasPlan() == true )
		return true;

	clearDraw();

	if( (origin-target.getTranslation()).norm2() < 0.020 )  //already there !!!!
	{
		qDebug() << __FUNCTION__ << "Origin and target too close. Diff: "  << (origin-target.getTranslation()).norm2() << origin << target.getTranslation() << ". Returning void";
		return true;
	}

	QVec point;
// 	if( sampler.checkRobotValidDirectionToTargetOneShot( origin, target ) )
// 	{
// 		path << origin << target;
// 		qDebug() << __FUNCTION__ << "Found target directly in line of sight";
// 		return true;
// 	}
// 	else
//	{
		qDebug() << __FUNCTION__ << "Calling Full Power of RRTConnect OMPL planner. This may take a while";
		Target lastTarget = target;
		if (planner->computePath(origin, targetR.subVector(0,2), 6) == false)  //5 secs max
			return false;
//	}

	qDebug() << __FUNCTION__ << "Plan length: " << planner->getPath().size();
	QList<QVec> path = planner->getPath();
	
	qDebug() << path;
	
	//Convert back to world reference system
	for(int i= 0; i<path.size(); i++)
		path[i] = innerModel.transform("world",path[i],"robot");
	
	draw(innermodelmanager_proxy,path);
	
	//qFatal("fary");
	QVec w(6);
	w[0]  = 1; 	w[1]  = 1;  w[2]  = 1; w[3]  = 0; w[4] = 0; w[5] = 0;
	for(int i=0; i<path.size(); ++i)
	{
		QVec pp(6,0.f);
		pp.inject(path[i],0);
		Target t(Target::POSE6D, &innerModel, bodyParts["RIGHTARM"].getTip(), pp, w, false);
		t.setHasPlan(true);
		//t.print();
		if(i==0)
			target = t;
// 		if(i==path.size()-1)																					///NO ROTATION DURING PATH PLANNING
// 			t.setWeights( lastTarget.getWeights() );
		bodyParts["RIGHTARM"].addTargetToList(t);
	}	
	return true;
}



void SpecificWorker::doReflexxes( const QList<QVec> &jointValues, const QStringList &motors )
{
	Reflexx *reflexx = new Reflexx(proxy, jointValues, motors);
	//reflexx->updateMotorState(motors);
	//reflexx->setSyncPosition( listGoals );
	reflexx->start();
	qDebug() << __FUNCTION__ << "Waiting for Reflexx...";
	reflexx->wait(5000);
}

/**
 * @brief Targets have to be defined in the WORLD reference SYSTEM
 * 		  Creates a target element inside InnerModel to be used by IK. Avoids having a "target" in the XML file.
 * 		  Each "target" node in InnerModel is created for each target that arrives here, and deleted when finished.
 * 		  Each bodypart may have a different target and BIK eliminates its dependence of InnerModelManager
 *
 * @param target ...
 * @return void
 */
void SpecificWorker::createInnerModelTarget(Target &target)
{
	InnerModelNode *nodeParent = innerModel->getNode("world");
	target.setNameInInnerModel(QString::number(correlativeID++));
	InnerModelTransform *node = innerModel->newTransform(target.getNameInInnerModel(), "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
	nodeParent->addChild(node);
	QVec p = target.getPose();
	innerModel->updateTransformValues(target.getNameInInnerModel(),p.x(), p.y(), p.z(), p.rx(), p.ry(), p.rz(), "world");

}
void SpecificWorker::removeInnerModelTarget(const Target& target)
{
	innerModel->removeNode(target.getNameInInnerModel());
}


///////////////////////////////////////////////
/// SERVANTS  OJO se ejecuta en el  hilo de ICE
//////////////////////////////////////////////



/**
 * @brief Takes bodyPart to target pose, defined in the WORLD reference system
 *
 * @param bodyPart ...
 * @param target ...
 * @param weights ...
 * @param radius NOT USED
 * @return void
 */
void SpecificWorker::setTargetPose6D(const string& bodyPart, const Pose6D& target, const WeightVector& weights, float radius)
{
	//setPose6D(bodyPart, target, weights, radius);

	QString partName = QString::fromStdString(bodyPart);
	if ( this->bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}

	//Se deben comprobar condiciones del target cuando las tengamos.
	QVec tar(6);
	tar[0] = target.x;	tar[1] = target.y;	tar[2] = target.z;
	tar[3] = target.rx;	tar[4] = target.ry;	tar[5] = target.rz;

	//PASAMOS A METROS
	tar[0] = tar[0] / (T)1000;
	tar[1] = tar[1] / (T)1000;
	tar[2] = tar[2] / (T)1000;

	//Weights vector
	QVec w(6);
	w[0]  = weights.x; 	w[1]  = weights.y; w[2]  = weights.z; w[3]  = weights.rx; w[4] = weights.ry; w[5] = weights.rz;

   Target t(Target::POSE6D, innerModel, bodyParts[partName].getTip(), tar, w, radius);
   t.setRadius(radius/1000.f);
	 t.setHasPlan(!radius);  //Ñapa para que no planifique si este campo viene a false

    mutex->lock();
        bodyParts[partName].addTargetToList(t);
    mutex->unlock();

	qDebug() << "--------------------------------------------------------------------------";
	qDebug() << __FUNCTION__<< "New target arrived: " << partName << ". For target:" << tar << ". With weights: " << w;
	
}

/**
 * @brief ...
 *
 * @param bodyPart ...
 * @param target ...
 * @param axis ...
 * @param axisConstraint ...
 * @param axisAngleConstraint ...
 * @return void
 */
void SpecificWorker::pointAxisTowardsTarget(const string& bodyPart, const Pose6D& target, const Axis& axis, bool axisConstraint, float axisAngleConstraint)
{
	QString partName = QString::fromStdString(bodyPart);

	if ( bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part or incorrect axis";
		throw ex;
	}

	//Se deben comprobar condiciones del target cuando las tengamos.
	QVec tar(6);
	tar[0] = target.x;	tar[1] = target.y;	tar[2] = target.z;	tar[3] = target.rx;	tar[4] = target.ry;	tar[5] = target.rz;

	//PASAMOS A METROS
	tar[0] = tar[0] / (T)1000;
	tar[1] = tar[1] / (T)1000;
	tar[2] = tar[2] / (T)1000;

	//Weights vector ONLY ROTATION
	QVec w(6);
	w[0] = 0; w[1] = 0; w[2] = 0; w[3] = 1; w[4] = 1; w[5] = 1;

	QVec ax = QVec::vec3( axis.x , axis.y, axis.z);

	Target t(Target::ALIGNAXIS, innerModel, bodyParts[partName].getTip(), tar, ax, w);

	mutex->lock();
		bodyParts[partName].addTargetToList(t);
	mutex->unlock();

	qDebug() << "-----------------------------------------------------------------------";
	qDebug() << __FUNCTION__ << __LINE__<< "New target arrived: " << partName;
}


/**
 * @brief Make the body part advance along a given direction. It is meant to work as a simple translational joystick to facilitate grasping operations
 *
 * @param bodyPart ...
 * @param ax ...
 * @param dist step to advance un milimeters
 * @return void
 */
void SpecificWorker::advanceAlongAxis(const string& bodyPart, const Axis& ax, float dist)
{
	QString partName = QString::fromStdString(bodyPart);

	if ( bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}

	//Code axis as a dist norm vector in tip reference frame
	QVec axis = QVec::vec3(	ax.x, ax.y,	ax.z).normalize();

	//Bigger jump admisible
	if(dist > 300) dist = 300;
	if(dist < -300) dist = -300;
	dist = dist / 1000.;   //PASANDO A METROS

	Target t(Target::ADVANCEAXIS, innerModel, bodyParts[partName].getTip(), axis, dist);

	mutex->lock();
		bodyParts[partName].addTargetToList(t);
	mutex->unlock();

	qDebug() << "-----------------------------------------------------------------------";
	qDebug() <<  __FILE__ << __FUNCTION__ << __LINE__<< "New target arrived: " << partName;
}

/**
 * @brief Set the fingers of the right hand position so there is d mm between them
 *
 * @param d millimeters between fingers
 * @return void
 */
void SpecificWorker::setFingers(float d)  ///ONLY RIGHT HAND. FIX
{
	qDebug() << __FUNCTION__;

	//float len = 0.07; // check
	//float len = innerModel->getTranslationVectorTo("rightFinger1","finger_right_1_1_tip").norm2();   //CONSULTAR CON LUIS POR QUÉ NO VA Y EL DE ABAJO SI
	float len = innerModel->transform("finger_right_1", QVec::zeros(3), "finger_right_1_1_tip").norm2();
	//qDebug() << "len" << len;
	float D = (d/1000)/2.; 			// half distnace in meters
	float s = D/len;
	if( s > 1) s = 1;
	if( s < -1) s = -1;
	float ang = asin(s); 	// 1D inverse kinematics
	QVec angles = QVec::vec2( ang - 1, -ang + 1);

	/// OJO!!! DO THAT WITH innerModel->getNode("rightFinger1")->min
	QStringList joints;
	joints << "finger_right_1" << "finger_right_2";
	moveRobotPart(angles, joints);

	// fingerRight1, fingerRight2 are the joints going from -1 to 0 (left) and from 1 to 0 (right)
	// se anclan en "arm_right_8"
	// necesitamos la longitud total desde fingerRightX hasta la punta (l)
	// entonces D sería la distancia desde el eje central hasta la punta y se calcularía como D = l*sin(ang) where and debe ser 0 para D = 0 y pi/2 para D=l
	// ang = ang1 + 1 y ang = -ang2+1
	// dedo izquierdo: D=l*sin(ang)
	// dedo derecho: D=l*sin(ang)
	// Ahora cinemática inversa: ang = asin(D/l)
	// ang1  = ang - 1;
	// ang2 = -ang + 1;

}

/**
 * @brief ...
 *
 * @param part ...
 * @return void
 */
void SpecificWorker::goHome(const string& part)
{
	QString partName = QString::fromStdString(part);
	
	//clearDraw();
	
	if ( bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	qDebug() << "----------------------------------------";
	qDebug() << "Go gome" << QString::fromStdString(part);
	qDebug() << bodyParts[partName].getMotorList();

	QStringList lmotors = bodyParts[partName].getMotorList();
	for(int i=0; i<lmotors.size(); i++){
		try {
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = lmotors.at(i).toStdString();
			nodo.position = innerModel->getJoint(lmotors.at(i))->home;
			nodo.maxSpeed = 1; //radianes por segundo
			mutex->lock();
				proxy->setPosition(nodo);
			mutex->unlock();

		} catch (const Ice::Exception &ex) {
			cout<<"Excepción en mover Brazo: "<<ex<<endl;
		}
	}
	//goHomePosition( bodyParts[partName].getMotorList());
	//sleep(1);

}

/**
 * @brief Sets the variable Proxy to the robot or RCIS Ice proxy
 *
 * @param type ...
 * @return void
 */
void SpecificWorker::setRobot(const int t)
{
	mutex->lock();
	this->typeR = t;
	if (this->typeR == 0)
		proxy = jointmotor0_proxy;
	else if (this->typeR == 1)
		proxy = jointmotor1_proxy;
	mutex->unlock();
}


/**
 * @brief
 *
 * @param part ...
 * @return RoboCompBodyInverseKinematics::Statedesigner
 *
 */
TargetState SpecificWorker::getState(const std::string &part)
{
	TargetState state;
	state.finish = false;
	state.elapsedTime = 0;
	state.estimatedEndTime = -1;

	QString partName = QString::fromStdString(part);

	if ( bodyParts.contains(partName) == false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	else
	{
 		qDebug() << "----------------------------------------";
 		qDebug() << "New COMMAND arrived "<< __FUNCTION__ << partName;

		mutex->lock();
		if( bodyParts[partName].getTargets().isEmpty() )
			state.finish = true;
		else
		{
			state.elapsedTime = bodyParts[partName].getHeadFromTargets().getElapsedTime();
			state.finish = false;
		}
		
		mutex->unlock();
	}
	return state;
}


/**
 * @brief Stops robot part and cleans it queue
 *
 * @param part ...
 * @return void
 */
void SpecificWorker::stop(const std::string& part)
{
	QString partName = QString::fromStdString(part);

	if ( bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	else
	{
		qDebug() << "-------------------------------------------------------------------";
		qDebug() << "New COMMAND arrived " << __FUNCTION__ << QString::fromStdString(part);
		mutex->lock();
			bodyParts[partName].markForRemoval();
		mutex->unlock();
	}
}


/**
 * @brief Changes the current tip of the corresponding part
 *
 * @param part ...
 * @return void
 */
void SpecificWorker::setNewTip(const std::string &part, const std::string &transform, const Pose6D &pose)
{
	QString partName = QString::fromStdString(part);
	QString transformName = QString::fromStdString(transform);

	//Check if tip is a valid new tip
	if (bodyParts.contains(partName)==false)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Not recognized body part";
		RoboCompBodyInverseKinematics::BIKException ex;
		ex.text = "Not recognized body part";
		throw ex;
	}
	else
	{
		qDebug() << "-------------------------------------------------------------------";
		qDebug() << "New COMMAND arrived " << __FUNCTION__ << QString::fromStdString(part);

		mutex->lock();
		//bodyParts[partName].setNewVisualTip(pose);
		//innerModel->transform("world", QVec::zeros(3),part).print("antes setNewTip");
		innerModel->updateTransformValues(transformName, pose.x/1000., pose.y/1000., pose.z/1000., pose.rx, pose.ry, pose.rz);
		//innerModel->updateTransformValues( "grabPositionHandR", pose.x/1000., pose.y/1000., pose.z/1000., 0,0,0);

		//innerModel->transform("world", QVec::zeros(3), part).print("despues setNewTipo");
		mutex->unlock();
	}
}


/**
 * @brief Direct reposition of a joint
 *
 * @param joint InnerModel name of the joint
 * @param value value in proper units
 * @return void
 */
void SpecificWorker::setJoint(const std::string& joint, float value, float maxSpeed)
{
	try
	{
		RoboCompJointMotor::MotorGoalPosition nodo;
		nodo.name = joint;
		nodo.position = value; 		// posición en radianes
		nodo.maxSpeed = maxSpeed; 			// radianes por segundo
		proxy->setPosition(nodo);
	}
	catch (const Ice::Exception &ex)
	{	cout<< ex << "Exception moving " << joint << endl;	}
}


/*-----------------------------------------------------------------------------*
 * 			                MÉTODOS    PRIVADOS                                *
 *-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*
 * 			                MÉTODOS    ACTUALIZADORES                          *
 *-----------------------------------------------------------------------------*/
/*
 * Método actualizarInnermodel
 * Actualiza el InnerModel con las nuevas posiciones de los motores del robot.
 * FUNCIONA.
 */
void SpecificWorker::actualizarInnermodel(const QStringList &listaJoints)
{
	
	//qDebug() << listaJoints << listaJoints.size();
	
	try
	{
		MotorList mList;
		for (int i=0; i<listaJoints.size(); i++)
		{
// 			qDebug() << i;
			mList.push_back(listaJoints[i].toStdString());
		}

		
		RoboCompJointMotor::MotorStateMap mMap = proxy->getMotorStateMap(mList);
	/*	
		for(auto i: mMap)
			std::cout << i.first << std::endl;*/
		
		for (int j=0; j<listaJoints.size(); j++)
		{
 			//qDebug() << j << listaJoints[j];
			innerModel->updateJointValue(listaJoints[j], mMap.at(listaJoints[j].toStdString()).pos);
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Excepción en actualizar InnerModel: "<<ex<<endl;
	}

	try
	{
// 		qDebug() << "<";
		RoboCompDifferentialRobot::TBaseState bState;
		differentialrobot_proxy->getBaseState( bState );
		innerModel->updateTransformValues("robot", bState.x/1000, 0, bState.z/1000, 0, bState.alpha, 0);
// 		qDebug() << ">";
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Excepción reading DifferentialRobot: "<<ex<<endl;
	}
}


/*-----------------------------------------------------------------------------*
 * 			                MÉTODOS PARA MOVER                                 *
 *-----------------------------------------------------------------------------*/


/*
 * Método moverBrazo.
 * Mueve el brazo cambiando los ángulos que forman con el eje X
 * (el primer segmento) y con el primer segmento (el segundo segmento).
 * FUNCIONA.
 */
void SpecificWorker::moveRobotPart(QVec angles, const QStringList &listaJoints)
{
	for(int i=0; i<angles.size(); i++)
	{
		try
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = listaJoints.at(i).toStdString();
			nodo.position = angles[i]; // posición en radianes
			nodo.maxSpeed = 0.5; //radianes por segundo TODO Bajar velocidad.
			proxy->setPosition(nodo);
		} catch (const Ice::Exception &ex) {
			cout<<"Excepción en mover Brazo: "<<ex<<endl;
		}
	}
}


/*
 * Método moverTarget versión 2.
 * Mueve el target a una posición que se le pasa como parámetro de entrada.
 * Crea una pose3D a cero y actualiza sus traslaciones tx, ty y tz y sus
 * rotaciones rx, ry y rz con los datos del parámetro de entrada.
 * Sirve para colocar el target en el innerModel. Para nada más.
 */
// void SpecificWorker::moverTarget(const QVec &pose)
// {
// 	try
// 	{
// 		RoboCompInnerModelManager::Pose3D p;
// 		p.x=p.y=p.z=p.rx=p.ry=p.rz=0.0; //Primero inicializamos a cero.
//
// 		p.x = pose[0]; p.y = pose[1]; p.z = pose[2];
// 		p.rx = pose[3]; p.ry = pose[4]; p.rz = pose[5];
//
// 		innermodelmanager_proxy->setPoseFromParent("target",p);
// 		innerModel->updateTransformValues("target",p.x,p.y,p.z,p.rx,p.ry,p.rz);
// 		}
// 	catch (const Ice::Exception &ex)
// 	{
// 		cout<<"Excepción en moverTarget: "<<ex<<endl;
// 	}
// }



/*-----------------------------------------------------------------------------*
 * 			                MÉTODOS    AUXILIARES                              *
 *-----------------------------------------------------------------------------*/
/**
 * @brief ...
 *
 * @param t ...
 * @return float
 */
float SpecificWorker::standardRad(float t)
{
	if (t >= 0.) {
		t = fmod(t+M_PI, M_PI/2.) - M_PI;
	} else {
		t = fmod(t-M_PI, -M_PI/2.) + M_PI;
	}
	return t;
}


void SpecificWorker::calcularModuloFloat(QVec &angles, float mod)
{
    for(int i=0; i<angles.size(); i++)
    {
        int cociente = (int)(angles[i] / mod);
        angles[i] = angles[i] -(cociente*mod);

        if(angles[i] > M_PI)
            angles[i] = angles[i]- M_PI;
        else
            if(angles[i] < -M_PI)
                angles[i] = angles[i] + M_PI;
    }
}

void SpecificWorker::clearDraw()
{
		
	try
	{	
		innermodelmanager_proxy->removeNode("road");		
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ std::cout << ex << std::endl;}
	
}

bool SpecificWorker::draw(InnerModelManagerPrx innermodelmanager_proxy, const QList<QVec> &path)
{
	RoboCompInnerModelManager::Pose3D pose;
	pose.y = 0;	pose.x = 0;	pose.z = 0;
	pose.rx = pose.ry = pose.z = 0.;
	RoboCompInnerModelManager::Plane3D plane;
	plane.px = 0; plane.py = 0;	plane.pz = 0;
	plane.nx = 0; plane.ny = 1; plane.nz = 0;
	plane.width = 20; 	plane.height = 20;	plane.thickness = 20;
	plane.texture = "#05ff00";
	
	try
	{	std::string  parentAll = "road";
		innermodelmanager_proxy->addTransform(parentAll,"static","floor", pose);		
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ std::cout << ex << std::endl;}
	
	for(int i=0; i<path.size(); i++)
	{
		QString item = "p_" + QString::number(i);		
		pose.x = path[i].x()*1000;	pose.y = path[i].y()*1000; pose.z = path[i].z()*1000 ;
		try
		{
			//	qDebug() << "punto" << path[i];
			innermodelmanager_proxy->addTransform(item.toStdString(),"static","road",pose);
			innermodelmanager_proxy->addPlane(item.toStdString() + "_point", item.toStdString(), plane);	
		}
		catch(const Ice::Exception &ex){ std::cout << ex << std::endl;}
		//RcisDraw::drawLine(innermodelmanager_proxy, item + "_point", item, normal, 250, 50, "#005500" );
	}
	return false;
}
/*
 * Método getRotacionMano
 * Devuelve la rotación de la mano del robot
 */
// QVec SpecificWorker::getRotacionMano (QString puntaMano)
// {
// 	QMat matriz = innerModel->getRotationMatrixTo("world", puntaMano);
// 	QVec ManoEnMundo = innerModel->getTransformationMatrix("world", puntaMano).extractAnglesR3(matriz);
// 	QVec angulos1 = QVec::vec3(ManoEnMundo[0], ManoEnMundo[1], ManoEnMundo[2]);
// 	QVec angulos2 = QVec::vec3(ManoEnMundo[3], ManoEnMundo[4], ManoEnMundo[5]);
// 	QVec rot;
// 	if(angulos1.norm2() < angulos2.norm2())
// 		rot = angulos1;
// 	else
// 		rot = angulos2;
//
// 	return rot;
// }



// /*
//  * Método moverTarget.
//  * Mueve el target (la esfera objetivo del innerModel) a una posición guardada
//  * en la lista de posiciones de Targets listaTargets. Crea una pose a cero y le
//  * actualiza las traslaciones tx, ty y tz y las rotaciones rx, ry y rz con los
//  * datos almacenados en la lista.
//  */
// void SpecificWorker::moverTarget(int contador)
// {
// 	if(contador < listaPosicionTarget.size())
// 	{
// 		try{
// 			RoboCompInnerModelManager::Pose3D p;
// 			p.x=p.y=p.z=p.rx=p.ry=p.rz=0.0; //Primero inicializamos a cero.
// 			ptarget = listaPosicionTarget.at(contador); //sacamos posicion del target de la lista de posiciones.
//
// // 			p.x = ptarget[0]; p.y = ptarget[1]; p.z = ptarget[2];
// // 			p.rx = ptarget[3]; p.ry = ptarget[4]; p.rz = ptarget[5];
// 			innerModel->transform("world", QVec::zeros(3), "tip");
// 			p.x = 0.35; p.y = 0.8; p.z = 0.2;
// 			p.rx = 0; p.ry = 0; p.rz = 0; //rot a 0
//
// 			innermodelmanager_proxy->setPoseFromParent("target",p);
// 			innerModel->updateTransformValues("target",p.x,p.y,p.z,p.rx,p.ry,p.rz);
//
// 			}catch (const Ice::Exception &ex) {
// 				cout<<"Excepción en moverTarget: "<<ex<<endl;
// 			}
// 	}
// 	else
// 		qDebug()<<"ERROR al mover target: Fuera de la lista de posiciones";
// }

// /*
//  * Metodo goHomePosition.
//  * Lleva al brazo a una posicion determinada para comenzar a moverlo.
//  */
// void SpecificWorker::goHomePosition(const QStringList &listaJoints )
// {
//
// 	for(int i=0; i<listaJoints.size(); i++){
// 		try {
// 			RoboCompJointMotor::MotorGoalPosition nodo;
// 			nodo.name = listaJoints.at(i).toStdString();
// 			nodo.position = innerModel->getJoint(listaJoints.at(i))->home;
// 			nodo.maxSpeed = 5; //radianes por segundo
// 			proxy->setPosition(nodo);
//
// 		} catch (const Ice::Exception &ex) {
// 			cout<<"Excepción en mover Brazo: "<<ex<<endl;
// 		}
// 	}
// }
//
