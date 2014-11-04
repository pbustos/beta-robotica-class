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

 
 //**************************************************************************//
 //						CONTRUCTORES Y DESTRUCTORES							 //
 //**************************************************************************//
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{	
	// Inicializamos las banderas de los targets a FALSE:
	//	- NO hay trayectoria asignada
	//	- NO hay target para RCIS
	flagListTargets = existTarget = false;

	// ÑAPA PARA PODER ENVIAR UN MISMO TARGET A DOS O TRES PARTES DEL CUERPO A LA VEZ
	// Por defecto siempre empezamos con una única parte del cuerpo.
	//	- 1: está activada.
	//	- 0: está desactivada.
	partesActivadas = QVec(3); //vector de partes
	partesActivadas[0] = 1;		partesActivadas[1] = 0;		partesActivadas[2] = 0; 
	
	osgView = new OsgView (frame);
	osgView->setCameraManipulator(new osgGA::TrackballManipulator); 	
	osgView->getCameraManipulator()->setHomePosition(osg::Vec3(0.,0.,-2.),osg::Vec3(0.,0.,4.),osg::Vec3(0.,1,0.));
	
	// Llamamos al método que se encarga de conectar los botones de la interfaz de usuario:
	connectButtons();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	qDebug()<<"Cerrando el tester...";
}


//**************************************************************************//
//							MÉTODOS PÚBLICOS								//
//**************************************************************************//
/**
 * @brief Method SET PARAMS. It's called for the MONITOR thread, which initialize the component with the 
 * parameters of the correspondig config file.
 * 
 * @return bool
 */ 
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	// Guardamos el innerModel que se nos pasa como parámetro de inicialización.
	// ¡CUIDADO CON EL INNERMODEL! Debe ser el mismo que el que utiliza LOKIARM!!!
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("BIK.InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;		
		}
		else
		{	qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
			qFatal("Exiting now.");
		}
	}
	catch(std::exception e)	{ qFatal("Error reading config params"); }

	//Inicializamos los valores de la pestaña inverseKinematicsComp
	initDirectKinematicsFlange();
	
	// Ponemos también una ventana del innerModel en la última pestaña:
	imv = new InnerModelViewer (innerModel, "root", osgView->getRootGroup());
	timer.start(Period);
	return true;
};



//**************************************************************************//
//								SLOTS										//
//**************************************************************************//
/**
 * @brief SLOT COMPUTE.
 * Este método, con el motivo de que la GUI no se quede colgada, se encarga de:
 * 	--> Hacer un PING al inverseKinematicsComp, para evr si funciona o no.
 *	--> Revisar en qué pestaña estamos de la interfaz y mostrar su información correspondiente.
 * 	--> Enviar LISTAS de targets.
 * 	--> Actualizar el innerModel y los distintos elementos para pintarlo.
 * 
 * @return void
 */
void SpecificWorker::compute( )
{
// 	static QTime reloj = QTime::currentTime();
// 	// Hacemos un ping al inverseKinematicsComp cada cierto tiempo. 
// 	// Si no responde es que el componente no está levantado.
// 	if (reloj.elapsed() > 3000)
// 	{
// 		try
// 		{ 
// 			bodyinversekinematics_proxy->ice_ping(); 
// 			
// 		}catch(const Ice::Exception &ex){ std::cout << "Warning! BIK not responding" << std::endl; };
// 		reloj.restart();
// 	}

 	if(existTarget)	sendTarget(); //TODO ARREGLAR TRAYECTORIAS
	
	// Si estamos mirando la pestaña número 5 mostramos los datos de los valores angulares
	// de los motores así como sus límites. Evitamos que la interfaz se cuelgue.
	if(pestanias->currentIndex()==5)	showKinematicData();

	// Actualizamos el innerModel y la ventada del viewer
	actualizarInnerModel();
	imv->update();
	osgView->frame();
}

//**************************************************************************//
//						SLOTS DE LOS SUBMIT BUTTONS							//
//**************************************************************************//
/**
 * @brief SLOT público STOP. 
 * Este método se encarga de llamar al método de la interfaz stop de inverseKinematicsComp
 * con las (por ahora) tres partes del cuerpo del robot, el brazo derecho, el izquierdo y
 * la cabeza. Este método de la interfaz debería encargarse de parar la ejecución del target
 * asignado a las partes del robot.
 * Además el SLOT stop baja la bandera de trayectorias para anular los targets que se envían.
 * 
 * @return void
 */ 
void SpecificWorker::stop()
{
	qDebug()<<"Parando todos los motores del robot...";
	try 
	{
		bodyinversekinematics_proxy->stop("LEFTARM");
		bodyinversekinematics_proxy->stop("RIGHTARM");
		bodyinversekinematics_proxy->stop("HEAD");	
		
		flagListTargets=false;
	} 
	catch (const Ice::Exception &ex) {	cout<<"Excepción en STOP: "<<ex<<endl;	}
}

/**
 * @brief SLOT ENVIAR HOME. Se invoca al pulsar el botón HOME de los botones de control.
 * Es un envoltorio que llama al método goHome indicando que vayan a la posición de home
 * TODAS las partes del robot.
 */
void SpecificWorker::enviarHome()
{
	goHome("All");
}

/**
 * @brief SLOT ENVIAR RCIS. 
 * Marca como proxy objetivo el del RCIS, levanta la bandera indicando que existen targets a resolver
 * y llama a la función enviar, que se encarga de ver en qué pestaña está el usuario de la GUI y enviar
 * el target del tipo que sea, al proxy indicado.
 * 
 * También marca el botón RCIS con un color distinto.
 * 
 * @return void
 */ 
void SpecificWorker::enviarRCIS()
{
	changeText(0);
	try
	{
		bodyinversekinematics_proxy->setRobot(0);
	}
	catch(const Ice::Exception &ex){std::cout <<"Enviar RCIS:"<< ex << std::endl;};
}

/**
 * @brief SLOT ENVIAR ROBOT. Marca como proxy objetivo el del ROBOT REAL  y llama a la función enviar, 
 * que se encarga de ver en qué pestaña está el usuario de la GUI y enviar el target del tipo que sea, 
 * al proxy indicado.
 * 
 * @return void
 */ 
void SpecificWorker::enviarROBOT()
{
	changeText(1);
	try
	{
		bodyinversekinematics_proxy->setRobot(1);
	}
	catch(const Ice::Exception &ex){std::cout <<"Enviar ROBOT: "<< ex << std::endl;};
}

//**************************************************************************//
//					SLOTS ÚTILES PARA FACILITAR LA INTERFAZ					//
//**************************************************************************//
/**
 * @brief SLOT UPDATE BODY PARTS BOX.
 * Activa o desactiva en TODAS las pestañas de la interfaz las cajas donde estan las distintas partes 
 * del cuerpo del robot. Sirve para poder enviar un mismo target a dos o más partes al mismo tiempo. 
 * Para ello, pone un 1 en el vector de partesActivadas en aquellas cajas que esten seleccionadas o 
 * un 0 si no están activadas. 
 * 
 * TODO Se puede añadir el control para eliminar las partes ya seleccionadas en las cajas anteriores.
 * 
 * @return void
 */ 
void SpecificWorker:: updateBodyPartsBox()
{
	// Si están activadas alguna de las tres primeras cajas de partes del robot ACTIVAMOS TODAS.
	// Si no lo está, DESACTIVAMOS TODAS. Así tenemos la misma configuración cuando cambiemos de página	
	if((Part1_pose6D->isChecked() and pestanias->currentIndex()==0) or 
	   (Part1_AxisAlign->isChecked() and pestanias->currentIndex()==1) or 
	   (Part1_AlongAxis->isChecked() and pestanias->currentIndex()==2))
	{
		//Activamos las cajas y marcamos los checkBox
		partBox1_pose6D->setEnabled(true);		partBox1_pose6D->repaint();		Part1_pose6D->setCheckState(Qt::Checked);
		partBox1_AxisAlign->setEnabled(true);	partBox1_AxisAlign->repaint();	Part1_AxisAlign->setCheckState(Qt::Checked);
		partBox1_AlongAxis->setEnabled(true);	partBox1_AlongAxis->repaint();	Part1_AlongAxis->setCheckState(Qt::Checked);
		partesActivadas[0] = 1;
	}
	else
	{
		// Desactivamos las cajas y desmarcamos los chekbox
		partBox1_pose6D->setEnabled(false);		partBox1_pose6D->repaint();		Part1_pose6D->setCheckState(Qt::Unchecked);
		partBox1_AxisAlign->setEnabled(false);	partBox1_AxisAlign->repaint();	Part1_AxisAlign->setCheckState(Qt::Unchecked);
		partBox1_AlongAxis->setEnabled(false);	partBox1_AlongAxis->repaint(); 	Part1_AlongAxis->setCheckState(Qt::Unchecked);
		partesActivadas[0] = 0;
	}

	//----------------------------------------------------------------------------//
	if((Part2_pose6D->isChecked() and pestanias->currentIndex()==0) or 
	   (Part2_AxisAlign->isChecked() and pestanias->currentIndex()==1) or 
	   (Part2_AlongAxis->isChecked() and pestanias->currentIndex()==2))
	{
		partBox2_pose6D->setEnabled(true);		partBox2_pose6D->repaint();		Part2_pose6D->setCheckState(Qt::Checked);
		partBox2_AxisAlign->setEnabled(true);	partBox2_AxisAlign->repaint();	Part2_AxisAlign->setCheckState(Qt::Checked);
		partBox2_AlongAxis->setEnabled(true);	partBox2_AlongAxis->repaint();	Part2_AlongAxis->setCheckState(Qt::Checked);
		partesActivadas[1] = 1;
	}

	else
	{
		partBox2_pose6D->setEnabled(false);		partBox2_pose6D->repaint();		Part2_pose6D->setCheckState(Qt::Unchecked);
		partBox2_AxisAlign->setEnabled(false);	partBox2_AxisAlign->repaint();	Part2_AxisAlign->setCheckState(Qt::Unchecked);
		partBox2_AlongAxis->setEnabled(false);	partBox2_AlongAxis->repaint();	Part2_AlongAxis->setCheckState(Qt::Unchecked);
		partesActivadas[1] = 0;
	}

	//----------------------------------------------------------------------------//
	if((Part3_pose6D->isChecked() and pestanias->currentIndex()==0) or 
	   (Part3_AxisAlign->isChecked() and pestanias->currentIndex()==1) or 
	   (Part3_AlongAxis->isChecked() and pestanias->currentIndex()==2))
	{
		partBox3_pose6D->setEnabled(true);		partBox3_pose6D->repaint();		Part3_pose6D->setCheckState(Qt::Checked);	
		partBox3_AxisAlign->setEnabled(true);	partBox3_AxisAlign->repaint();	Part3_AxisAlign->setCheckState(Qt::Checked);	
		partBox3_AlongAxis->setEnabled(true);	partBox3_AlongAxis->repaint();	Part3_AlongAxis->setCheckState(Qt::Checked);
		partesActivadas[2] = 1;
	}
	else
	{
		partBox3_pose6D->setEnabled(false);		partBox3_pose6D->repaint();		Part3_pose6D->setCheckState(Qt::Unchecked);
		partBox3_AxisAlign->setEnabled(false);	partBox3_AxisAlign->repaint();	Part3_AxisAlign->setCheckState(Qt::Unchecked);
		partBox3_AlongAxis->setEnabled(false);	partBox3_AlongAxis->repaint();	Part3_AlongAxis->setCheckState(Qt::Unchecked);
		partesActivadas[2] = 0;
	}
}

/**
 * @brief SLOT SEND
 * Cuando los botones "send" de las pestañas son pulsados, levantan la bandera existTarget, indicando
 * que existe uno o varios targets a resolver. De esta forma SEPARAMOS los botones RCIS y URSUS del 
 * envío de Targets, función que llevará a cabo el compute evitando que la interfaz vaya tan lenta
 * y facilitando su depuración.
 * 
 * @return void
 */ 
void SpecificWorker::send()
{
	existTarget = true;
}


/*------------------------------------------------------------------------------------------------------*
 * 									SLOTS PARA LA PESTAÑA POSE6D										*
 *------------------------------------------------------------------------------------------------------*/
/**
 * @brief Metodo CAMARERO ZURDO. Crea una trayectoria de poses para un camarero con una bandeja en
 * la mano izquierda. Guarda esa trayectoria en un atributo de la clase, trayectoria, para luego 
 * enviarla a que la ejecute el innerModel o el robot real. Debemos limpiar la trayectoria siempre
 * para que no se acumulen las trayectorias y levanta la flagListTargets, indicando que existe 
 * una trayectoria lista para enviar al RCIS o al ROBOT.
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * Desactiva el resto de banderas para el resto de targets.
 * 
 * @return void
 */ 
void SpecificWorker::camareroZurdo()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.

	//DEFINIMOS VARIABLES:
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Aunque las 
	//				rotaciones se dejan a CERO.
	//		- SALTO: salto en X e Y para pasar de una pose a otra. Fijado a 10mm.
	//		- xAux e yAux: auxiliares para crear el marco donde se mueve la mano del camarero.
    QVec pose = QVec::zeros(6);
	float salto = 10;
    float xAux, yAux;

	 // Trasladamos hacia la derecha en X:
	for(float i=-150; i<=150; i=i+salto)
	{
		pose[0] = i; pose[1] = 900; pose[2] = 350;
		trayectoria.enqueue(pose);
		xAux = i;
	}
	// Subimos en Y:
	for(float j=900; j<1100; j=j+salto)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 350;
		trayectoria.enqueue(pose);
		yAux = j;
	}
	// Trasladamos hacia la izquierda en X:
	for(float i=xAux; i>=-150; i=i-salto)
	{
		pose[0] = i; pose[1] = yAux; pose[2] = 350;
		trayectoria.enqueue(pose);
		xAux=i;
	}
	// Bajamos en Y:
	for(float j=yAux; j>=900; j=j-salto)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 350;
		trayectoria.enqueue(pose);
		yAux = j;
	}

	flagListTargets = true; //Indicamos que hay una trayectoria lista para enviar.
}

/**
 * @brief Metodo CAMARERO DIESTRO. Crea una trayectoria de poses para un camarero con una bandeja 
 * en la mano derecha. Guarda esa trayectoria en un atributo de la clase, trayectoria, para luego 
 * enviarla a que la ejecute el innerModel o el robot real. Debemos limpiar la trayectoria siempre
 * para que no se acumulen las trayectorias y levanta la flagListTargets, indicando que existe 
 * una trayectoria lista para enviar al RCIS o al ROBOT.
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * Desactiva el resto de banderas para el resto de targets
 * 
 * @return void
 */ 
void SpecificWorker::camareroDiestro()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.

	//DEFINIMOS VARIABLES:
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Aunque las 
	//				rotaciones se dejan a CERO.
	//		- SALTO: salto en X e Y para pasar de una pose a otra. Fijado a 10mm.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- xAux e yAux: auxiliares para crear el marco donde se mueve la mano del camarero.
    QVec pose = QVec::zeros(6);
	float salto = 10;
    float xAux, yAux;

	//guardamos la posicion de partida
 	QVec home(6);
 	home.inject(innerModel->transform("world", QVec::zeros(3),"grabPositionHandR"),0);
 	home.inject(innerModel->getRotationMatrixTo("world","grabPositionHandR").extractAnglesR_min(),3);
 	
	// Trasladamos en X hacia la izquierda: 
	for(float i=-100; i<300; i=i+salto)
	{
		pose[0] = i; pose[1] = 900; pose[2] = 350;
		pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		trayectoria.append(pose);
		xAux = i;
	}
	// Subimos en Y:
	for(float j=900; j<1100; j=j+salto)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 350;
		pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		trayectoria.append(pose);
		yAux = j;
	}
	// Trasladamos en X hacia la derecha:
	for(float i=xAux; i>=-100; i=i-salto)
	{
		pose[0] = i; pose[1] = yAux; pose[2] = 350;
		pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		trayectoria.append(pose);
		xAux = i;
	}
	// Bajamos en Y:
	for(float j=yAux; j>=900; j=j-salto)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 350;
		pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		trayectoria.append(pose);
		yAux = j;
	}

	//go back to initial grabPositionHandR
	trayectoria.append(home);

	flagListTargets = true; //Indicamos que hay una trayectoria lista para enviar.
}

/**
 * @brief Metodo CAMARERO CENTRO. Crea una trayectoria de poses para un camarero con una bandeja 
 * en moviendola por el centro del pecho. Guarda esa trayectoria en un atributo de la clase, 
 * trayectoria, para luego enviarla a que la ejecute el innerModel o el robot real. Debemos limpiar
 * la trayectoria siempre para que no se acumulen las trayectorias y levanta la flagListTargets, 
 * indicando que existe una trayectoria lista para enviar al RCIS o al ROBOT.
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * Desactiva el resto de banderas para el resto de targets.
 * 
 * @return void
 */ 
void SpecificWorker::camareroCentro()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.

	//DEFINIMOS VARIABLES:
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Aunque las 
	//				rotaciones se dejan a CERO.
	//		- SALTO: salto en X e Y para pasar de una pose a otra. Fijado a 10mm.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- xAux e yAux: auxiliares para crear el marco donde se mueve la mano del camarero.
  QVec pose = QVec::zeros(6);

	pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();

	pose[0] = -150; pose[1] = 900; pose[2] = 300;
	trayectoria.append(pose);

	pose[0] = 150; pose[1] = 900; pose[2] = 300;
	trayectoria.append(pose);

	pose[0] = 150; pose[1] = 1100; pose[2] = 300;
	trayectoria.append(pose);

	pose[0] = -150; pose[1] = 1100; pose[2] = 300;
	trayectoria.append(pose);

	pose[0] = -150; pose[1] = 900; pose[2] = 300;
	trayectoria.append(pose);

	flagListTargets = true; //Indicamos que hay una trayectoria lista para enviar.
}

/**
 * @brief Método PUNTOS ESFERA. Crea una trayectoria, formando una esfera cerca del efector final
 * del robot.Está pensada para que la parte del robot a la que se le envíe llegue sin problemas, 
 * relativamente, se guarden los errores y el tiempo de ejecución de cada target resuleto y se pinte
 * una gráfica del error y del tiempo de ejecución en MATLAB.
 *  -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * Calcula la esfera de radio 100mm para el efector seleccionado en el primer recuadro!!!
 * 
 * @return void
 */
void SpecificWorker::puntosEsfera()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.

	//DEFINIMOS VARIABLES:
	//		- LISTAPUNTOS: lista auxiliar donde se guardan las traslaciones de los puntos calculados. 
	//		  Se utiliza para no repetir puntos con las mismas traslaciones.
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Le guardamos 
	//		  las rotaciones como la interfaz indica
	//		- PAUX: vector auxiliar para calcular traslaciones alrededor de la esfera.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- TOTAL: entero auxiliar para calcular una pose de la esfera.
	//		- RADIO: float que indica el radio de la esfera.
	//		- PART: parte del robot seleccionada.
	//		- EFECTOR:	nombre del efector final de la parte del robot. Sirve como centro de la esfera
	QList<QVec> listaPuntos;
	QVec pose = QVec::zeros(6);	pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
	QVec paux = QVec::zeros(3);
	int total = 0;
	float radio = 100.0; //10cm

	// Sacamos la parte del robot de la primera caja y obtenemos su efector final para situar
	// el centro de la esfera
	std::string part = partBox1_pose6D->currentText().toStdString();
    QString efector;
	if(part=="LEFTARM") 	efector="grabPositionHandL";
	if(part=="RIGHTARM")	efector="grabPositionHandR";
	if(part=="HEAD")	 	efector="head3";

	// Mientras que no rellenemos la lista con todos los targets que queremos calcular:
	// 	- Por una parte calculamos las traslaciones, que serán alrededor de una esfera.
	// 	- Por otra parte dejamos las rotaciones como están.
	while(trayectoria.size()<100)
	{
		// Preparamos las traslaciones:
		while(total < 1)
		{
			QVec d1 = QVec::uniformVector(1, -1, 1);
			QVec d2 = QVec::uniformVector(1, -1, 1);

			if (QVec::vec2(d1[0],d2[0]).norm2() < 1.f)
			{
				paux[0]	=	2*d1[0] - sqrt(1.f-d1[0]*d1[0] - d2[0]*d2[0]);	
				paux[1]	=	2*d2[0] - sqrt(1.f-d1[0]*d1[0] - d2[0]*d2[0]);	
				paux[2]	=	1.f -2.f*(d1[0]*d1[0] + d2[0]*d2[0]);
				paux = (paux * (T)radio) + innerModel->transform("world", QVec::zeros(3),efector);
				total++;
			}
		}
		total = 0;

		if(!listaPuntos.contains(paux))
		{
			listaPuntos.append(paux);
			pose[0] = paux[0];	pose[1] = paux[1];	pose[2] = paux[2];
			trayectoria.enqueue(pose);
		}
	}
	flagListTargets = true; //Indicamos que hay una trayectoria lista para enviar.
// 	banderaNiapa = true;
}

/**
 * @brief Método PUNTOS CUBO.Crea una trayectoria de puntos aleatorios pertenecientes al cubo
 * tridimensional formado por los vértices:
 * 			x    y    z
 * 		- (400, 800, 200)	(400,1200, 200)		(400, 800, 600) 	(400, 1200, 600)
 * 		- (-400, 800, 200)	(-400,1200, 200)	(-400, 800, 600) 	(-400, 1200, 600)
 * 
 * Está pensado para que la parte del robot a la que se le envíe llegue sin muchos problemas, 
 * relativamente.
 *  -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * 
 * @return void
 */ 
void SpecificWorker::puntosCubo()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.

	//DEFINIMOS VARIABLES:
	//		- LISTAPUNTOS: lista auxiliar donde se guardan las traslaciones de los puntos calculados. 
	//		  Se utiliza para no repetir puntos con las mismas traslaciones.
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Le guardamos 
	//		  las rotaciones como la interfaz indica
	//		- PAUX: vector auxiliar para calcular traslaciones alrededor de la esfera.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- TOTAL: entero auxiliar para calcular una pose de la esfera.
	//		- RADIO: float que indica el radio de la esfera.
	//		- PART: parte del robot seleccionada.
	//		- EFECTOR:	nombre del efector final de la parte del robot. Sirve como centro de la esfera
	QList<QVec> listaPuntos;
	QVec pose = QVec::zeros(6);	pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
	QVec paux = QVec::zeros(3);
	
	srand((unsigned) time(NULL)); 
	
	// Mientras que no rellenemos la lista con todos los targets que queremos calcular:
	// calculamos las traslaciones, que serán alrededor de una esfera.
	while(trayectoria.size()<100)
	{
		paux[0]= (rand()%801)-400; 			// X entre -400 y 400
		paux[1]= (rand()%(1200-800))+800; 	// Y entre 800 y 1200
		paux[2]= (rand()%(600-200))+200;  	// Z entre 200 y 600

		if(!listaPuntos.contains(paux))
		{
			listaPuntos.append(paux);
			pose[0] = paux[0];	pose[1] = paux[1];	pose[2] = paux[2];
			trayectoria.enqueue(pose);
		}
	}
	flagListTargets = true; //Indicamos que hay una trayectoria lista para enviar.
}


/*--------------------------------------------------------------------------------------------------*
 * 							SLOTS PARA LA PESTAÑA DE HOME											*
 *--------------------------------------------------------------------------------------------------*/
/**
 * @brief Metodo GO HOME. Envia a la posicion de HOME la parte del cuerpo del robot que este 
 * seleccionada en la pestaña "Home". Existe la opción "All" dentro de esta pestaña, en la que
 * hay que enviar TODAS las partes del cuerpo al HOME.
 * 
 * @return void
 */
void SpecificWorker::goHome(QString partName)
{
	std::string part = partName.toStdString();
	qDebug() << "Go gome" << partName;
	try 
	{	

		if(partName=="All")
		{
				bodyinversekinematics_proxy->goHome("HEAD");
				bodyinversekinematics_proxy->goHome("LEFTARM");
				bodyinversekinematics_proxy->goHome("RIGHTARM");
		}
		else
			bodyinversekinematics_proxy->goHome(part);
	} 
	catch (Ice::Exception ex) 
	{
		cout << ex << endl;
	}
	
	existTarget=false;
}

/*--------------------------------------------------------------------------------------------------*
 * 							SLOTS PARA LA PESTAÑA SET FINGERS								*
 *--------------------------------------------------------------------------------------------------*/
/**
 * @brief ..
 * @return void
 */ 
void SpecificWorker::closeFingers()
{
	try 
	{	
		qDebug() << __FUNCTION__ << "Set fingers";
		bodyinversekinematics_proxy->setFingers((T)fingersDistanceSB->value());
	} 
	catch (Ice::Exception ex) 
	{
		cout << ex << endl;
	}

	existTarget=false;
}

/*--------------------------------------------------------------------------------------------------*
 * 							SLOTS PARA LA PESTAÑA ADVANCE ALONG AXIS								*
 *--------------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*
 * 	MÉTODOS PRIVADOS DE LA CLASE									*
 *----------------------------------------------------------------------------------------------*/




/**
 * @brief Método ACTUALIZAR INNERMODEL. Actualiza el InnerModel con las nuevas posiciones 
 * de los motores del robot.  
 * 
 * @return void
 */ 
void SpecificWorker::actualizarInnerModel()
{
	try 
	{
		RoboCompJointMotor::MotorStateMap mMap = jointmotor_proxy->getMotorStateMap( this->motorList);

		for(uint j=0; j<motorList.size(); j++)
		{
			//printf("%s\n", motorList[j].c_str());
			innerModel->updateJointValue(QString::fromStdString(motorList[j]), mMap.at(motorList[j]).pos);
		}
	} catch (const Ice::Exception &ex) {cout<<"--> Excepción en actualizar InnerModel: "<<ex<<endl;	}
}

/**
 * @brief Metodo MOVER TARGET. Mueve el target dentro del innerModel a una posicion que se le pasa 
 * como parametro de entrada. Crea una pose3D a cero y actualiza sus traslaciones tx, ty y tz y sus 
 * rotaciones rx, ry y rz con los datos del parámetro de entrada. 
 * Sirve para colocar el target en el innerModel. Para nada más.
 * @param QVec pose es la posicion en la que se debe pintar el target en el innerModel. 
 * 
 * @return void
 */ 
void SpecificWorker::moverTargetEnRCIS(const QVec &pose)
{
	try
	{
		RoboCompInnerModelManager::Pose3D p;
		p.x=p.y=p.z=p.rx=p.ry=p.rz=0.0; //Primero inicializamos a cero.

		p.x = pose[0]; p.y = pose[1]; p.z = pose[2];
		p.rx = pose[3]; p.ry = pose[4]; p.rz = pose[5];
		try
		{
			innermodelmanager_proxy->setPoseFromParent("target",p);
		}
		catch (const Ice::Exception &ex){ cout<<"RCIS connection problem "<<ex<<endl; }
		innerModel->updateTransformValues("target",p.x,p.y,p.z,p.rx,p.ry,p.rz);        ////CREO QUE SE PUEDE QUITAR

	}catch (const Ice::Exception &ex){ cout<<"Excepción en moverTarget: "<<ex<<endl; }
}

/**
 * @brief Metodo privado ENVIAR POSE6D. Es llamado por el SLOT enviarRCIS cuando la flagListTargets
 * no esta levantada y se está trabajando en la primera pestaña (Pose6D). Tiene capacidad para enviar 
 * un mismo target a las tres partes del cuerpo con las que se estan trabajando ahora: el brazo derecho,
 * el izquierdo y la cabeza, con las opciones de la interfaz.
 * @param QVec p es el vector de traslaciones y posiciones que tiene que enviar al inverseKinematicsComp
 * 
 * @return void
 */ 
void SpecificWorker::enviarPose6D(QVec p)
{
	// DEFINIMOS VARIABLES:
	//		- poseMetros: auxiliar para pasar las traslaciones de las poses de milímetros a metros
	//					  (para pintar el cubo o target en el RCIS de METROS)
	//		- pesos: vector de pesos de las traslaciones y las rotaciones.
	//		- part1, par2 y par3: partes del cuerpo del robot al que va dirigido el/los target/s
	//		- partes: cola donde colocar las partes del cuerpo e ir consultándolas.
	//		- type: tipo de target que se le está enviando.
	//		- pose6D: tipo de variable para enviar traslaciones y rotaciones al inverseKinematicsComp
	//		- weights: tipo de variable para enviar los pesos al inverseKinematicsComp
	QVec poseMetros = QVec::zeros(6);
	QVec pesos(6);
	std::string part1, part2, part3;
	QQueue <std::string> partes;
	QString type;	
	RoboCompBodyInverseKinematics::Pose6D pose6D;
	RoboCompBodyInverseKinematics::WeightVector weights;

	try
	{
		//colocamos el target en el RCIS. HAY QUE PASAR A METROS LAS TRASLACIONES
		//poseMetros[0] = p[0]/1000; 	poseMetros[1]=p[1]/1000; 	poseMetros[2]=p[2]/1000;
		//poseMetros[0] = p[0]; 	poseMetros[1]=p[1]; 	poseMetros[2]=p[2];   //POSE A RCIS EN mm

		//moverTargetEnRCIS(p); 
			
		//Creamos la pose6D para pasárselo al componente lokiArm (pasamos MILÍMETROS)
		pose6D.x = p[0];     pose6D.y = p[1];     pose6D.z = p[2];
		//antes de pasar las rotaciones las "normalizamos"
		QVec aux (3); 
		aux[0] = p[3]; 	     aux[1] = p[4];	  aux[2] = p[5];
		//calcularModuloFloat(aux, 2*M_PI);
		pose6D.rx = aux[0];    pose6D.ry = aux[1];    pose6D.rz = aux[2];

		//Ponemos los pesos mirando la interfaz.
		pesos.set((T)0);
		if(wTX->isChecked()) pesos[0] = 1;	if(wTY->isChecked()) pesos[1] = 1;	if(wTZ->isChecked()) pesos[2] = 1; //TRASLACIONES
		if(wRX->isChecked()) pesos[3] = 1;	if(wRY->isChecked()) pesos[4] = 1;	if(wRZ->isChecked()) pesos[5] = 1; //ROTACIONES

		//Creamos la variable del tipo weightvector para pasarselo al componente lokiArm
		weights.x = pesos[0];		weights.y = pesos[1];		weights.z = pesos[2];  
		
		weights.rx = pesos[3];		weights.ry = pesos[4];		weights.rz = pesos[5];
		//weights.rx = 1;		weights.ry = 0;		weights.rz = 1;
   	
		//Sacamos la/s parte/s del cuerpo a la que va destinado el target y las encolamos para
		//luego ir sacándolas en un bucle. También obtenemos el tipo de target
		// enviarPose6D MIRA LAS PARTES MARCADAS EN LA PESTAÑA DE POSE6D!!!!
		part1 = part2 = part3 = "NO_NAME";
		if(partesActivadas[0] == 1)	part1 = partBox1_pose6D->currentText().toStdString();
		if(partesActivadas[1] == 1) 	part2 = partBox2_pose6D->currentText().toStdString();
		if(partesActivadas[2] == 1) 	part3 = partBox3_pose6D->currentText().toStdString();
		partes.enqueue(part1); partes.enqueue(part2); partes.enqueue(part3);
		type = "POSE6D"; //Fijamos el tipo

		for(int i=0; i<3; i++)
		{
			std::string part = partes.dequeue();
			if (part != "NO_NAME")
			{
				if(part == "HEAD")
				{
					RoboCompBodyInverseKinematics::Axis axis;
					axis.x = 0; axis.y = -1; axis.z = 0;
					bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
				}				
				else
				{ 	
					qDebug()<<"---> TARGET ENVIADO con traslaciones ("<<pose6D.x<<pose6D.y<<pose6D.z<<") y rotaciones ("<<pose6D.rx<<pose6D.ry<<pose6D.rz<<")";
					bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);
				}
				usleep(50000);
			}
		}
	}catch(Ice::Exception ex){ std::cout<<"Error al pasar TARGET POSE6D: "<<ex<<endl;}
	
	// Terminamos de procesar el target
	if(flagListTargets==false or (flagListTargets==true and trayectoria.isEmpty()==true))
	{
		existTarget=false;
		qDebug()<<"FIN DEL TARGET";
	}
}


/**
 * @brief Metodo ENVIAR AXIS ALIGN. Saca de la segunda pestaña de la interfaz del usuario los datos para 
 * componer un target del tipo AXISALIGN y enviarselo a una parte o a varias partes del cuerpo del robot.
 * 
 * @return void
 */
void SpecificWorker::enviarAxisAlign()
{
	QQueue <QString> partes;

	if(partesActivadas[0] == 1)	
		partes.enqueue(partBox1_AxisAlign->currentText());
	if(partesActivadas[1] == 1) 
		partes.enqueue(partBox2_AxisAlign->currentText());
	if(partesActivadas[2] == 1) 
		partes.enqueue(partBox3_AxisAlign->currentText());

	foreach(QString parte, partes)
	{
		try
		{
			RoboCompBodyInverseKinematics::Axis axis;
			axis.x = TipAxis_X->value();
			axis.y = TipAxis_Y->value();
			axis.z = TipAxis_Z->value();		
			RoboCompBodyInverseKinematics::Pose6D pose6D;
			pose6D.x = axisAlignXSB->value();
			pose6D.y = axisAlignYSB->value();
			pose6D.z = axisAlignZSB->value();
			pose6D.rx = axisAlignRXSB->value();
			pose6D.ry = axisAlignRYSB->value();
			pose6D.rz = axisAlignRZSB->value();	

			QVec pose = QVec::zeros(6);
			pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;
			moverTargetEnRCIS(pose);

			bodyinversekinematics_proxy->pointAxisTowardsTarget(parte.toStdString(), pose6D, axis, false, 0 );
 		}
 		catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "Error al pasar el target tipo ALIGNAXIS: "<<ex<<endl;}
	}

	existTarget=false;
	qDebug()<<"FIN DEL TARGET";
}

/**
 * @brief Metodo MOVE ALONG AXIS. Saca de la tercera pestaña de la interfaz del usuario los datos para
 * componer un target del tipo ALONGAXIS y se lo envia a una o a varias partes del cuerpo del robot.
 * 
 * @return void
 */
void SpecificWorker::moveAlongAxis()
{
	QQueue <QString> partes;


	qDebug() << __FUNCTION__ << "Processing command";
	if(partesActivadas[0] == 1)	
		partes.enqueue(partBox1_AlongAxis->currentText());
	if(partesActivadas[1] == 1) 
		partes.enqueue(partBox2_AlongAxis->currentText());
	if(partesActivadas[2] == 1) 
		partes.enqueue(partBox3_AlongAxis->currentText());

	foreach(QString parte, partes)
	{
		try
		{
			RoboCompBodyInverseKinematics::Axis axis;
			axis.x = TipAxis_X2->value();
			axis.y = TipAxis_Y2->value();
			axis.z = TipAxis_Z2->value();
			float dist = distanceSpinBox->value();

			bodyinversekinematics_proxy->advanceAlongAxis(parte.toStdString(), axis, dist);
		} 
		catch(Ice::Exception ex){ std::cout<<"Error al pasar el target tipo ADVANCEALONGAXIS: "<<ex<<endl;}
	}
	existTarget=false;
	qDebug()<<"FIN DEL TARGET";
}





/*
* AÑADIDO DESDE CINEMÁTICA INVERSA.
*  Metodo moduloFloat
* Devuelve el m��dulo entre dos n��meros reales.   ///HAS PROBADO FMOD? NO, NO TENGO TANTO CONOCIMIENTO DE C++
* FUNCIONA.
*/
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



/*----------------------------------------------------------------------------------*/

void SpecificWorker::abrirPinza()
{	
	try
	{	
		qDebug() << __FUNCTION__ << "Open fingers";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setFingers((T)abrirPinzaValor->value());

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setFingers((T)abrirPinzaValor->value());
	} catch (Ice::Exception ex) {cout <<"ERROR EN ABRIR PINZA: "<< ex << endl;}
}


void SpecificWorker::cerrarPinza()
{
	try
	{	
		qDebug() << __FUNCTION__ << "Close fingers";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setFingers((T)cerrarPinzaValor->value());

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setFingers((T)cerrarPinzaValor->value());
	} catch (Ice::Exception ex) { cout << "ERROR EN CERRAR PINZA: "<<ex << endl;}
}


void SpecificWorker::posicionInicial()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX->value();
		pose6D.y = PY->value();
		pose6D.z = PZ->value();
		pose6D.rx = RX->value();
		pose6D.ry = RY->value();
		pose6D.rz = RZ->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "RIGHTARM";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		part = "HEAD";
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x = 0; axis.y = -1; axis.z = 0;
// 		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR POSICION INICIAL: "<<ex<<endl;}

}

void SpecificWorker::posicionCoger()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_2->value();
		pose6D.y = PY_2->value();
		pose6D.z = PZ_2->value();
		pose6D.rx = RX_2->value();
		pose6D.ry = RY_2->value();
		pose6D.rz = RZ_2->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "RIGHTARM";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		part = "HEAD";
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x = 0; axis.y = -1; axis.z = 0;
// 		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN POSICION COGER: "<<ex<<endl;}
}


void SpecificWorker::posicionSoltar()
{

	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_3->value();
		pose6D.y = PY_3->value();
		pose6D.z = PZ_3->value();
		pose6D.rx = RX_3->value();
		pose6D.ry = RY_3->value();
		pose6D.rz = RZ_3->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "RIGHTARM";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);
// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		part = "HEAD";
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x = 0; axis.y = -1; axis.z = 0;
// 		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN POSICION SOLTAR: "<<ex<<endl;}

}



void SpecificWorker::izquierdoRecoger()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_4->value();
		pose6D.y = PY_4->value();
		pose6D.z = PZ_4->value();
		pose6D.rx = RX_4->value();
		pose6D.ry = RY_4->value();
		pose6D.rz = RZ_4->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 


		std::string part = "LEFTARM";

		/* AL ROBOT :*/
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		/* AL RCIS*/
// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN IZQUIERDO RECOGER: "<<ex<<endl;}

}


void SpecificWorker::retroceder()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_5->value();
		pose6D.y = PY_5->value();
		pose6D.z = PZ_5->value();
		pose6D.rx = RX_5->value();
		pose6D.ry = RY_5->value();
		pose6D.rz = RZ_5->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "RIGHTARM";

		/* AL ROBOT :*/
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		/* AL RCIS */
// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN RETROCEDER: "<<ex<<endl;}

}

void SpecificWorker::goHomeR()
{
	goHome("RIGHTARM");
}


void SpecificWorker::izquierdoOfrecer()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_6->value();
		pose6D.y = PY_6->value();
		pose6D.z = PZ_6->value();
		pose6D.rx = RX_6->value();
		pose6D.ry = RY_6->value();
		pose6D.rz = RZ_6->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "LEFTARM";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		part = "HEAD";
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x = 0; axis.y = -1; axis.z = 0;
// 		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN IZQUIERDO OFRECER: "<<ex<<endl;}

}

/// SUBSCRIBED METHOD FROM APRILTAGS

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	marcaBote = QVec::zeros(6);
	manoApril = QVec::zeros(6);
	
 	for(uint i=0; i<tags.size(); i++)
 	{
 		//qDebug() << tags[i].id << tags[i].tx << tags[i].ty << tags[i].tz << tags[i].rx << tags[i].ry << tags[i].rz;
		if( tags[i].id == 12 )  //marcaBote
		{
			mutex->lock();
			
				if ((tags[i].tx == 0) && (tags[i].ty == 0) && (tags[i].tz == 0) && (tags[i].rx == 0) && (tags[i].ry == 0) && (tags[i].rz == 0))
					qDebug() << "LA MARCA 12 SE HA LEIDO MAL (todo a cero)";
				else
				{
					marcaBote[0] = tags[i].tx; marcaBote[1] = tags[i].ty; marcaBote[2] = tags[i].tz; marcaBote[3] = tags[i].rx; marcaBote[4] = tags[i].ry; marcaBote[5] = tags[i].rz;    
				}
				
			mutex->unlock();
			
			
				InnerModelNode *nodeParent = innerModel->getNode("rgbd");
				InnerModelTransform *node = innerModel->newTransform("marca3", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
				nodeParent->addChild(node);
				mutex->lock();
					innerModel->updateTransformValues("marca3",marcaBote.x(), marcaBote.y(), marcaBote.z(), marcaBote.rx(), marcaBote.ry(), marcaBote.rz(), "rgbd");	
				mutex->unlock();
				
								
				QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "marca3");
			 	QVec marcaRInWorld = innerModel->getRotationMatrixTo("world","marca3").extractAnglesR_min();
				
				marcaRInWorld.print("ROT marcaInWorld PRE");
				
				
				QMat matMarcaRInWorld = innerModel->getRotationMatrixTo("world","marca3");
				Rot3D matZeroErrorWorld(-M_PI/2.0,0,0); // Para que las rotaciones de la marca en el mundo sean cero cuando el plano de la marca está paralelo al suelo 
				Rot3D matLateralGrip(M_PI,-M_PI/2.0,0); // Para coger un objeto desde el lateral
								
				QMat matResul =  matMarcaRInWorld * matZeroErrorWorld * matLateralGrip.invert();
				marcaRInWorld = matResul.extractAnglesR_min();				
				
// 				Rot3D matZero(0,0.4,0);
// 				
// 				QMat matResul1 =  matMarcaRInWorld * matZeroErrorWorld;
// 				qDebug() << "Angulos 1" << matResul1.extractAnglesR_min();
// 				QMat matResul2 =  matLateralGrip * matZero;
// 				qDebug() << "Angulos 2" << matResul2.extractAnglesR_min();
// 				QMat matResul3 =  matZero * matLateralGrip.invert();
// 				qDebug() << "Angulos 3" << matResul3.extractAnglesR_min();
				
				QVec marcaInWorld(6);
				marcaInWorld.inject(marcaTInWorld,0);
				marcaInWorld.inject(marcaRInWorld,3);
				
				marcaInWorld.print("marcaInWorld POS");
								
				QVec targetInWorld = marcaInWorld;	
				targetInWorld[0] -= 100;   ///OJO ESTO SOLO VALE PARA LA MANO izquierda
				targetInWorld[2] -= 100;   
// 				targetInWorld[3] = 3.1415;
// 				targetInWorld[4] = -1.57; 
// 				targetInWorld[5] = 0; 
				
				
				qDebug() << "Target" << targetInWorld;
				
				innerModel->removeNode("marca3");
			
			
		}
		if( tags[i].id == 11 )  //Mano
		{
			mutex->lock();
				manoApril[0] = tags[i].tx; manoApril[1] = tags[i].ty; manoApril[2] = tags[i].tz; manoApril[3] = tags[i].rx; manoApril[4] = tags[i].ry; manoApril[5] = tags[i].rz;    
			mutex->unlock();
			
			
			qDebug() << "\n";
			
			// Escribimos por pantalla como está el grab en el mundo antes de hacer las modificaciones
			QVec grabTInWorld = innerModel->transform("world", QVec::zeros(3), "grabPositionHandL");
			QVec grabRInWorld = innerModel->getRotationMatrixTo("world","grabPositionHandL").extractAnglesR_min();
			QVec grabInWorld(6);
			grabInWorld.inject(grabTInWorld,0);
			grabInWorld.inject(grabRInWorld,3);
			qDebug() << "Grab en el mundo antes de modificar" << grabInWorld;
			
			
			// Inicio de los cálculos
				
			// Creamos el nodo de la marca vista desde la cámara
			InnerModelNode *nodeParent = innerModel->getNode("rgbd");
			InnerModelTransform *node = innerModel->newTransform("marcaHandInCamera3", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
			nodeParent->addChild(node);
			
			mutex->lock();
				innerModel->updateTransformValues("marcaHandInCamera3",manoApril.x(), manoApril.y(), manoApril.z(), manoApril.rx(), manoApril.ry(), manoApril.rz());	
			mutex->unlock();


			// Esto es sólo para mostrar la posición de la marca vista desde la cámara y desde el innermodel expresadas en el sistema de referencia del mundo
			QVec marca2TInWorld = innerModel->transform("world", QVec::zeros(3), "marcaHandInCamera3");
			QVec marca2RInWorld = innerModel->getRotationMatrixTo("world","marcaHandInCamera3").extractAnglesR_min();
			QVec marca2InWorld(6);
			marca2InWorld.inject(marca2TInWorld,0);
			marca2InWorld.inject(marca2RInWorld,3);
			qDebug() << "Marca de la mano en el mundo vista desde la camara" << marca2InWorld;
			
			QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "ThandMesh1");
			QVec marcaRInWorld = innerModel->getRotationMatrixTo("world","ThandMesh1").extractAnglesR_min();
			QVec marcaInWorld(6);
			marcaInWorld.inject(marcaTInWorld,0);
			marcaInWorld.inject(marcaRInWorld,3);
			qDebug() << "ThandMesh1 en el mundo vista desde RCIS" << marcaInWorld;
			qDebug() << "Diferencia" <<  marca2InWorld - marcaInWorld;
			
			
			// Calculamos el error de la marca
			// Ponemos la marca vista desde la cámara en el sistma de coordenadas de la marca de la mano, si no hay error debería ser todo cero
			QVec visualMarcaTInHandMarca = innerModel->transform("ThandMesh1", QVec::zeros(3), "marcaHandInCamera3");
			QVec visualMarcaRInHandMarca = innerModel->getRotationMatrixTo("ThandMesh1","marcaHandInCamera3").extractAnglesR_min();
			QVec visualMarcaInHandMarca(6);
			visualMarcaInHandMarca.inject(visualMarcaTInHandMarca,0);
			visualMarcaInHandMarca.inject(visualMarcaRInHandMarca,3);
			qDebug() << "Marca vista por la camara en el sistema de la marca de la mano (deberia ser cero si no hay errores)" << visualMarcaInHandMarca;
			
			// Cogemos la matriz de rotación dek tHandMesh1 (marca en la mano) con respecto al padre para que las nuevas rotaciones y translaciones que hemos calculado (visualMarcaInHandMarca) sean añadidas a las ya esistentes en ThandMesh1
			QMat visualMarcaRInHandMarcaMat = innerModel->getRotationMatrixTo("ThandMesh1","marcaHandInCamera3");
			QMat handMarcaRInParentMat = innerModel->getRotationMatrixTo("ThandMesh1_pre","ThandMesh1");
				
			// Multiplicamos las matrices de rotación para sumar la nueva rotación visualMarcaRInHandMarcaMat a la ya existente con respecto al padre
			QMat finalHandMarcaRMat = handMarcaRInParentMat * visualMarcaRInHandMarcaMat;
			QVec finalHandMarcaR = finalHandMarcaRMat.extractAnglesR_min();
				
			// Pasamos también las translaciones nuevas (visualMarcaTInHandMarca) al padre y las sumamos con las existentes
			QVec handMarcaTInParent = innerModel->transform("ThandMesh1_pre", QVec::zeros(3), "ThandMesh1");
			QVec finalHandMarcaT = handMarcaTInParent + (handMarcaRInParentMat* visualMarcaTInHandMarca);
			
			// Esto es sólo para mostar como está el ThandMesh1 respecto al padre antes de las modificaciones
			QVec inicialHandMarca(6);
			inicialHandMarca.inject(handMarcaTInParent,0);
			inicialHandMarca.inject(handMarcaRInParentMat.extractAnglesR_min(),3);	
			qDebug() << "Posicion inicial del ThandMesh1 respecto al padre" << inicialHandMarca;
			
			// Creamos el vector final con las rotaciones y translaciones del tHandMesh1 con respecto al padre
			QVec finalHandMarca(6);
			finalHandMarca.inject(finalHandMarcaT,0);
			finalHandMarca.inject(finalHandMarcaR,3);
			
			qDebug() << "Posicion final si se corrigiese del ThandMesh1 respecto al padre" << finalHandMarca;
			
			//Escribimos por pantalla como está el grab en el mundo despues de hacer las modificaciones
			grabTInWorld = innerModel->transform("world", QVec::zeros(3), "grabPositionHandL");
			grabRInWorld = innerModel->getRotationMatrixTo("world","grabPositionHandL").extractAnglesR_min();
			grabInWorld.inject(grabTInWorld,0);
			grabInWorld.inject(grabRInWorld,3);
			qDebug() << "Grab en el mundo despues de modificar" << grabInWorld;
				
			//Eliminamos el nodo creado
			innerModel->removeNode("marcaHandInCamera3");
	
			qDebug() << "\n";
			
					
		}
 	}
}

///BOTONES DE APRIL

void SpecificWorker::ballisticPartToAprilTarget(int xoffset)
{
	qDebug() << __FUNCTION__;
	
	InnerModelNode *nodeParent = innerModel->getNode("rgbd");
	InnerModelTransform *node = innerModel->newTransform("marca", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
	nodeParent->addChild(node);
	mutex->lock();
		innerModel->updateTransformValues("marca",marcaBote.x(), marcaBote.y(), marcaBote.z(), marcaBote.rx(), marcaBote.ry(), marcaBote.rz(), "rgbd");	
	mutex->unlock();
	
	QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "marca");
// 	QVec marcaRInWorld = innerModel->getRotationMatrixTo("world","marca").extractAnglesR_min();
	QVec marcaRInWorld = QVec::vec3(3.1415,-1.57,0);
	QVec marcaInWorld(6);
	marcaInWorld.inject(marcaTInWorld,0);
	marcaInWorld.inject(marcaRInWorld,3);
	
	marcaBote.print("marcaInHead");
	marcaInWorld.print("marcaInWorld");
	
	QVec targetInWorld = marcaInWorld;	
	targetInWorld[0] -= 100;   ///OJO ESTO SOLO VALE PARA LA MANO izquierda
	targetInWorld[2] -= 100;   
	if (targetInWorld[2] < 200) targetInWorld[2] = 200;   
	
	
	try 
	{
		qDebug() << "Sent to target1" << targetInWorld;
		
		RoboCompBodyInverseKinematics::Pose6D pose;
		pose.x = targetInWorld.x();pose.y = targetInWorld.y();pose.z = targetInWorld.z();
		pose.rx = targetInWorld.rx();pose.ry = targetInWorld.ry();pose.rz = targetInWorld.rz();
				
		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 
 
 		bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "LEFTARM", pose, weights,0);
		
		sleep(4);
		
		bodyinversekinematics_proxy->setRobot(1); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "LEFTARM", pose, weights,0);
	} 
	catch (const Ice::Exception &ex) 
	{
		std::cout << ex << endl;
	}
	
	
	innerModel->removeNode("marca");
}


void SpecificWorker::finePartToAprilTarget()
{
	qDebug() << "\n\n";
	qDebug() << __FUNCTION__;
	
// 	//Here we should have both apriltags on sight
// 	// The target and the grabPositionHandL
// 
// 	//Now we want to tell BIK to change its endEffector to manoApril once transformed to the current endEffector reference system	
//  	InnerModelNode *nodeParent = innerModel->getNode("rgbd");
//   InnerModelTransform *node = innerModel->newTransform("handInCamera", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
//  	nodeParent->addChild(node);
// 	
// 	mutex->lock();
// 		innerModel->updateTransformValues("handInCamera",manoApril.x(), manoApril.y(), manoApril.z(), manoApril.rx(), manoApril.ry(), manoApril.rz());	
// 	mutex->unlock();
// 	
// 	QVec manoTInEndEffector = innerModel->transform("pre-grabPositionHandR", QVec::zeros(3), "handInCamera");
// 	QVec manoRInEndEffector = innerModel->getRotationMatrixTo("pre-grabPositionHandR","handInCamera").extractAnglesR_min();
// 	QVec manoInEndEffector(6);
// 	manoInEndEffector.inject(manoTInEndEffector,0);
// 	manoInEndEffector.inject(manoRInEndEffector,3);  //This is the 6D pose from current endEffector to mano apriltags
// // 	QMat m  = innerModel->getTransformationMatrix("grabPositionHandR","handInCamera");
// // 	m.print("m");
// // 	QMat m2  = innerModel->getTransformationMatrix("handInCamera","grabPositionHandR");
// // 	m2.print("m2");
// 		
// 	innerModel->transform("world",QVec::zeros(3),"handInCamera").print("handInCamera in World");
// 	
// 	mutex->lock();
// 		innerModel->updateTransformValues("grabPositionHandR",manoInEndEffector.x(), manoInEndEffector.y(), manoInEndEffector.z(), manoInEndEffector.rx(), manoInEndEffector.ry(), manoInEndEffector.rz());	
// 	mutex->unlock();
// 	
// 	//We send now to BIK the new endEffector pose
// 	try 
// 	{
// 		RoboCompBodyInverseKinematics::Pose6D pose;
// 		pose.x = manoInEndEffector.x();pose.y = manoInEndEffector.y();pose.z = manoInEndEffector.z();
// 		pose.rx = manoInEndEffector.rx();pose.ry = manoInEndEffector.ry();pose.rz = manoInEndEffector.rz();
// 		bodyinversekinematics_proxy->setNewTip("RIGHTARM", pose);
// 		
// 		qDebug() << __FUNCTION__ << manoInEndEffector;
// 		
// //		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", pose, pesos,10);
// 		
// 			
// 	} 
// 	catch (const Ice::Exception &ex) 
// 	{
// 		std::cout << ex << endl;
// 	}
// 	
// 	//And finally, we tell BIK to move the new endEffector to marcaApril, already in rgbd reference frame
// 	// ballisticPartToAprilTarget(0);
// 	
// 	innerModel->removeNode("handInCamera");
// 	
	qDebug() << "\n-----------------------------------------------------------------------------------------------------";
	
	// Escribimos por pantalla como está el grab en el mundo antes de hacer las modificaciones
	QVec grabTInWorld = innerModel->transform("world", QVec::zeros(3), "grabPositionHandL");
	QVec grabRInWorld = innerModel->getRotationMatrixTo("world","grabPositionHandL").extractAnglesR_min();
	QVec grabInWorld(6);
	grabInWorld.inject(grabTInWorld,0);
	grabInWorld.inject(grabRInWorld,3);
	qDebug() << "Grab en el mundo antes de modificar" << grabInWorld;
	
	
	// Inicio de los cálculos
	
	
	// Creamos el nodo de la marca vista desde la cámara
	InnerModelNode *nodeParent = innerModel->getNode("rgbd");
	InnerModelTransform *node = innerModel->newTransform("marcaHandInCamera", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
 	nodeParent->addChild(node);
	
	mutex->lock();
		innerModel->updateTransformValues("marcaHandInCamera",manoApril.x(), manoApril.y(), manoApril.z(), manoApril.rx(), manoApril.ry(), manoApril.rz());	
	mutex->unlock();


	// Esto es sólo para mostrar la posición de la marca vista desde la cámara y desde el innermodel expresadas en el sistema de referencia del mundo
	QVec marca2TInWorld = innerModel->transform("world", QVec::zeros(3), "marcaHandInCamera");
	QVec marca2RInWorld = innerModel->getRotationMatrixTo("world","marcaHandInCamera").extractAnglesR_min();
	QVec marca2InWorld(6);
	marca2InWorld.inject(marca2TInWorld,0);
	marca2InWorld.inject(marca2RInWorld,3);
	qDebug() << "Marca de la mano en el mundo vista desde la camara" << marca2InWorld;
	
	QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "ThandMesh1");
	QVec marcaRInWorld = innerModel->getRotationMatrixTo("world","ThandMesh1").extractAnglesR_min();
	QVec marcaInWorld(6);
	marcaInWorld.inject(marcaTInWorld,0);
	marcaInWorld.inject(marcaRInWorld,3);
	qDebug() << "ThandMesh1 en el mundo vista desde RCIS" << marcaInWorld;
	qDebug() << "Diferencia" <<  marca2InWorld - marcaInWorld;
	
	
	// Calculamos el error de la marca
	// Ponemos la marca vista desde la cámara en el sistma de coordenadas de la marca de la mano, si no hay error debería ser todo cero
	QVec visualMarcaTInHandMarca = innerModel->transform("ThandMesh1", QVec::zeros(3), "marcaHandInCamera");
	QVec visualMarcaRInHandMarca = innerModel->getRotationMatrixTo("ThandMesh1","marcaHandInCamera").extractAnglesR_min();
	QVec visualMarcaInHandMarca(6);
	visualMarcaInHandMarca.inject(visualMarcaTInHandMarca,0);
	visualMarcaInHandMarca.inject(visualMarcaRInHandMarca,3);
	qDebug() << "Marca vista por la camara en el sistema de la marca de la mano (deberia ser cero si no hay errores)" << visualMarcaInHandMarca;
	
	// Cogemos la matriz de rotación dek tHandMesh1 (marca en la mano) con respecto al padre para que las nuevas rotaciones y translaciones que hemos calculado (visualMarcaInHandMarca) sean añadidas a las ya esistentes en ThandMesh1
	QMat visualMarcaRInHandMarcaMat = innerModel->getRotationMatrixTo("ThandMesh1","marcaHandInCamera");
	QMat handMarcaRInParentMat = innerModel->getRotationMatrixTo("ThandMesh1_pre","ThandMesh1");
		
	// Multiplicamos las matrices de rotación para sumar la nueva rotación visualMarcaRInHandMarcaMat a la ya existente con respecto al padre
	QMat finalHandMarcaRMat = handMarcaRInParentMat * visualMarcaRInHandMarcaMat;
	QVec finalHandMarcaR = finalHandMarcaRMat.extractAnglesR_min();
		
	// Pasamos también las translaciones nuevas (visualMarcaTInHandMarca) al padre y las sumamos con las existentes
	QVec handMarcaTInParent = innerModel->transform("ThandMesh1_pre", QVec::zeros(3), "ThandMesh1");
	QVec finalHandMarcaT = handMarcaTInParent + (handMarcaRInParentMat* visualMarcaTInHandMarca);
	
	// Esto es sólo para mostar como está el ThandMesh1 respecto al padre antes de las modificaciones
	QVec inicialHandMarca(6);
	inicialHandMarca.inject(handMarcaTInParent,0);
	inicialHandMarca.inject(handMarcaRInParentMat.extractAnglesR_min(),3);	
	qDebug() << "Posicion inicial del ThandMesh1 respecto al padre" << inicialHandMarca;
	
	// Creamos el vector final con las rotaciones y translaciones del tHandMesh1 con respecto al padre
	QVec finalHandMarca(6);
	finalHandMarca.inject(finalHandMarcaT,0);
	finalHandMarca.inject(finalHandMarcaR,3);
	
	qDebug() << "Posicion final corregida del ThandMesh1 respecto al padre" << finalHandMarca;
	
	//Actualizamos el transform de la marca en la mano (ThandMesh1) con las rotaciones y translaciones calculadas
	mutex->lock();
		innerModel->updateTransformValues("ThandMesh1",finalHandMarca.x(), finalHandMarca.y(), finalHandMarca.z(), finalHandMarca.rx(), finalHandMarca.ry(), finalHandMarca.rz());	
	mutex->unlock();
	
	
	//Escribimos por pantalla como está el grab en el mundo despues de hacer las modificaciones
	grabTInWorld = innerModel->transform("world", QVec::zeros(3), "grabPositionHandL");
	grabRInWorld = innerModel->getRotationMatrixTo("world","grabPositionHandL").extractAnglesR_min();
	grabInWorld.inject(grabTInWorld,0);
	grabInWorld.inject(grabRInWorld,3);
	qDebug() << "Grab en el mundo despues de modificar" << grabInWorld;
		
	qDebug() << "-----------------------------------------------------------------------------------------------------\n";
	
	
	
	//Actualizamos el inermodel del Bik con los datos del inermodel del tester
	//We send now to inermodel BIK the new endEffector pose
	try 
	{
		RoboCompBodyInverseKinematics::Pose6D pose;
		pose.x = finalHandMarca.x();pose.y = finalHandMarca.y();pose.z = finalHandMarca.z();
		pose.rx = finalHandMarca.rx();pose.ry = finalHandMarca.ry();pose.rz = finalHandMarca.rz();
		bodyinversekinematics_proxy->setNewTip("LEFTARM", "ThandMesh1", pose);
		
		qDebug() << __FUNCTION__ << visualMarcaInHandMarca;
		
//		bodyinversekinematics_proxy->setTargetPose6D("LEFTARM", pose, pesos,10);
		
			
	} 
	catch (const Ice::Exception &ex) 
	{
		std::cout << ex << endl;
	}
	
	
	
	
	//Eliminamos el nodo creado
	innerModel->removeNode("marcaHandInCamera");

	
}


/**
 * 
 */ 
void SpecificWorker::boton_1()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = -100;
		pose6D.y = 1000;
		pose6D.z = 400;
		pose6D.rx = 3.14;
		pose6D.ry = -1.57;
		pose6D.rz = 0;	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

// 		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "LEFTARM";
		
		bodyinversekinematics_proxy->setRobot(0);
 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 0);
		
		sleep(1);
		
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 0);

	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN POSICION COGER: "<<ex<<endl;}
	
}

void SpecificWorker::boton_2()
{
qDebug() << __FUNCTION__;
	
	InnerModelNode *nodeParent = innerModel->getNode("rgbd");
	InnerModelTransform *node = innerModel->newTransform("marca", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
	nodeParent->addChild(node);
	mutex->lock();
		innerModel->updateTransformValues("marca",marcaBote.x(), marcaBote.y(), marcaBote.z(), marcaBote.rx(), marcaBote.ry(), marcaBote.rz(), "rgbd");	
	mutex->unlock();
	
	QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "marca");

	QMat matMarcaRInWorld = innerModel->getRotationMatrixTo("world","marca");
	Rot3D matZeroErrorWorld(-M_PI/2.0,0,0); // Para que las rotaciones de la marca en el mundo sean cero cuando el plano de la marca está paralelo al suelo 
	Rot3D matLateralGrip(M_PI,-M_PI/2.0,0); // Para coger un objeto desde el lateral
					
	QMat matResul =  matMarcaRInWorld * matZeroErrorWorld * matLateralGrip.invert();
	QVec marcaRInWorld = matResul.extractAnglesR_min();	
	
	//QVec marcaRInWorld = QVec::vec3(3.1415,-1.57,0);
	QVec marcaInWorld(6);
	marcaInWorld.inject(marcaTInWorld,0);
	marcaInWorld.inject(marcaRInWorld,3);
	
	marcaBote.print("marcaInHead");
	marcaInWorld.print("marcaInWorld");
	
	QVec targetInWorld = marcaInWorld;	
	targetInWorld[0] -= 200;   ///OJO ESTO SOLO VALE PARA LA MANO izquierda
	targetInWorld[2] -= 200;   
	if (targetInWorld[2] < 200) targetInWorld[2] = 200;   
	
	
	try 
	{
		qDebug() << "Sent to target1" << targetInWorld;
		
		RoboCompBodyInverseKinematics::Pose6D pose;
		pose.x = targetInWorld.x();pose.y = targetInWorld.y();pose.z = targetInWorld.z();
		pose.rx = targetInWorld.rx();pose.ry = targetInWorld.ry();pose.rz = targetInWorld.rz();
				
		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 
 
 		bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "LEFTARM", pose, weights,0);
		
		sleep(4);
		
		bodyinversekinematics_proxy->setRobot(1); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "LEFTARM", pose, weights,0);
	} 
	catch (const Ice::Exception &ex) 
	{
		std::cout << ex << endl;
	}
	
	
	innerModel->removeNode("marca");	
}

void SpecificWorker::boton_3()
{
qDebug() << __FUNCTION__;
	
	InnerModelNode *nodeParent = innerModel->getNode("rgbd");
	InnerModelTransform *node = innerModel->newTransform("marca", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
	nodeParent->addChild(node);
	mutex->lock();
		innerModel->updateTransformValues("marca",marcaBote.x(), marcaBote.y(), marcaBote.z(), marcaBote.rx(), marcaBote.ry(), marcaBote.rz(), "rgbd");	
	mutex->unlock();
	
	QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "marca");
	//QVec marcaRInWorld = innerModel->getRotationMatrixTo("world","marca").extractAnglesR_min();
	QMat matMarcaRInWorld = innerModel->getRotationMatrixTo("world","marca");
	Rot3D matZeroErrorWorld(-M_PI/2.0,0,0); // Para que las rotaciones de la marca en el mundo sean cero cuando el plano de la marca está paralelo al suelo 
	Rot3D matLateralGrip(M_PI,-M_PI/2.0,0); // Para coger un objeto desde el lateral
					
	QMat matResul =  matMarcaRInWorld * matZeroErrorWorld * matLateralGrip.invert();
	QVec marcaRInWorld = matResul.extractAnglesR_min();
	//QVec marcaRInWorld = QVec::vec3(3.1415,-1.57,0);
	QVec marcaInWorld(6);
	marcaInWorld.inject(marcaTInWorld,0);
	marcaInWorld.inject(marcaRInWorld,3);
	
	marcaBote.print("marcaInHead");
	marcaInWorld.print("marcaInWorld");
	
	QVec targetInWorld = marcaInWorld;	
	targetInWorld[0] -= 120;   ///OJO ESTO SOLO VALE PARA LA MANO izquierda
	targetInWorld[1] -= 40;
	targetInWorld[2] -= 0;   
	if (targetInWorld[2] < 200) targetInWorld[2] = 200;   
	
	
	
	
	try 
	{
		qDebug() << "Sent to target2" << targetInWorld;
		
		RoboCompBodyInverseKinematics::Pose6D pose;
		pose.x = targetInWorld.x();pose.y = targetInWorld.y();pose.z = targetInWorld.z();
		pose.rx = targetInWorld.rx();pose.ry = targetInWorld.ry();pose.rz = targetInWorld.rz();
				
		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 
 
 		bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "LEFTARM", pose, weights,0);
		
		sleep(4);
		
		bodyinversekinematics_proxy->setRobot(1); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "LEFTARM", pose, weights,0);
	} 
	catch (const Ice::Exception &ex) 
	{
		std::cout << ex << endl;
	}
	
	
	innerModel->removeNode("marca");	
}

void SpecificWorker::boton_4()
{
qDebug() << __FUNCTION__;
	
	InnerModelNode *nodeParent = innerModel->getNode("rgbd");
	InnerModelTransform *node = innerModel->newTransform("marca", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
	nodeParent->addChild(node);
	mutex->lock();
		innerModel->updateTransformValues("marca",marcaBote.x(), marcaBote.y(), marcaBote.z(), marcaBote.rx(), marcaBote.ry(), marcaBote.rz(), "rgbd");	
	mutex->unlock();
	
	QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "marca");
// 	QVec marcaRInWorld = innerModel->getRotationMatrixTo("world","marca").extractAnglesR_min();
	QMat matMarcaRInWorld = innerModel->getRotationMatrixTo("world","marca");
	Rot3D matZeroErrorWorld(-M_PI/2.0,0,0); // Para que las rotaciones de la marca en el mundo sean cero cuando el plano de la marca está paralelo al suelo 
	Rot3D matLateralGrip(M_PI,-M_PI/2.0,0); // Para coger un objeto desde el lateral
					
	QMat matResul =  matMarcaRInWorld * matZeroErrorWorld * matLateralGrip.invert();
	QVec marcaRInWorld = matResul.extractAnglesR_min();
	//QVec marcaRInWorld = QVec::vec3(3.1415,-1.57,0);
	QVec marcaInWorld(6);
	marcaInWorld.inject(marcaTInWorld,0);
	marcaInWorld.inject(marcaRInWorld,3);
	
	marcaBote.print("marcaInHead");
	marcaInWorld.print("marcaInWorld");
	
	QVec targetInWorld = marcaInWorld;	
	targetInWorld[0] -= 40;   ///OJO ESTO SOLO VALE PARA LA MANO izquierda
	targetInWorld[1] -= 40;
	targetInWorld[2] -= 0;   
	if (targetInWorld[2] < 200) targetInWorld[2] = 200;   
	
	
	
	
	try 
	{
		qDebug() << "Sent to target2" << targetInWorld;
		
		RoboCompBodyInverseKinematics::Pose6D pose;
		pose.x = targetInWorld.x();pose.y = targetInWorld.y();pose.z = targetInWorld.z();
		pose.rx = targetInWorld.rx();pose.ry = targetInWorld.ry();pose.rz = targetInWorld.rz();
				
		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 
 
 		bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "LEFTARM", pose, weights,0);
		
		sleep(4);
		
		bodyinversekinematics_proxy->setRobot(1); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "LEFTARM", pose, weights,0);
	} 
	catch (const Ice::Exception &ex) 
	{
		std::cout << ex << endl;
	}
	
	
	innerModel->removeNode("marca");	
}

void SpecificWorker::boton_5()
{
	
}

//**************************************************************************************************//
//						 MÉTODOS PRIVADOS MUY IMPORTANTES PARA QUE ESTO FUNCIONE 					//
//**************************************************************************************************//
/**
 * @brief Método SEND TARGET
 * Cuando se le llama, mira en qué pestaña está el usuario dentro de la interfaz gráfica. En el caso
 * de enviar un único target, lo compone con los datos de la pestaña y luego lo envía a través de 
 * cada método particular para enviar POSES6D, ALINGAXIS, ADVACEAXIS, HOME y FINGERS.
 * Si no son targets sueltos, entonces envía una lista de targets desencolándolos uno a uno y enviándolos
 * al método POSE6D.
 * Mira la pestaña en la que se encuentra el usuario de la GUI:
 * 	- Si está en la primera pestaña envía targets de tipo POSE6D sueltos, leyendo los 
 *    parámetros de traslación y rotación de la GUI. Las trayectorias las envía 
 * 	  directamente el compute para que el tester no se quede bloqueado.
 * 	- Si está en la segunda pestaña envía targets de tipo AXISALIGN.
 * 	- Si está en la tercera pestaña envía targets de tipo ALONGAXIS.
 * 	- Si está en la cuarta pestaña envía al robot a la posición de HOME.
 * 	- Si está en la quinta pestaña envía al robot la apertura de los dedos del robot.
 * 
 * @return void
 */ 
void SpecificWorker::sendTarget()
{
	//PRIMERO LAS TRAYECTORIAS DE TARGETS.
	if(flagListTargets and !trayectoria.isEmpty())
	{
		qDebug()<<"Enviamos trayectoria";
		enviarPose6D( trayectoria.dequeue() ); 
		sleep(1);	
	}
	
	// Si estamos en la pestaña 0 (la de Pose6D), entonces podemos enviar una pose suelta 
	// de tipo Pose6D. Para enviar la pose suelta leemos de la interfaz del usuario
	if(pestanias->tabText(pestanias->currentIndex()) == "Pose6D")
	{
		QVec p = QVec(6);
		p[0] = poseTX->value();		p[1] = poseTY->value();		p[2] = poseTZ->value(); //TRASLACIONES
		p[3] = poseRX->value();		p[4] = poseRY->value();		p[5] = poseRZ->value(); //ROTACIONES
		enviarPose6D(p);
	}

	// Estamos en la segunda pestaña: Axis Align. Enviamos target del tipo AXISALIGN.
	if(pestanias->tabText(pestanias->currentIndex()) == "Axis Align" )
		enviarAxisAlign();
	if(pestanias->tabText(pestanias->currentIndex()) == "Move Along Axis" )
		moveAlongAxis();
	if(pestanias->tabText(pestanias->currentIndex()) == "Home" )
		goHome(part_home->currentText());
	if(pestanias->tabText(pestanias->currentIndex()) == "Fingers" )
		closeFingers();
}


//**************************************************************************************************//
// MÉTODOS MUY SIMPLES Y AUXILIARES TOTALES QUE NO TIENEN MAYOR IMPORTANCIA PERO QUE LIMPIAN CÓDIGO //
//**************************************************************************************************//
/**
 * @brief Método CONNECT BUTTONS.
 * Se encarga de conectar TODOS los botones de la interfaz de usuario (de TODAS las pestañas). Es
 * llamado en el constructor de la clase para hacer todas las conexiones de los botones a sus 
 * correspondientes slots.
 * 
 * @return void
 */ 
void SpecificWorker::connectButtons()
{
	qDebug()<<"Conectando botones...";
	
	//BOTONES DE EJECUCIÓN: SUBMIT BUTTONS.
	connect(	stopButton, 	SIGNAL(clicked()), 	this, 	SLOT(stop())		); //botón de parada obligada.
	connect(	homeButton,		SIGNAL(clicked()), 	this,	SLOT(enviarHome())	); //botón de envío a la posición de HOME.

	connect(	rcisButton, 	SIGNAL(clicked()), 	this,	SLOT(enviarRCIS())	); //botón de enviar al robot simulado RCIS.
	connect(	robotButton,	SIGNAL(clicked()), 	this, 	SLOT(enviarROBOT())	); //botón de enviar al robot real Ursus.
	
	// BOTONES DE LA PESTAÑA POSE6D:
	connect(	sendButton1,			SIGNAL(clicked()),	this,	SLOT(send())				);
	connect(	Part1_pose6D, 			SIGNAL(clicked()), 	this, 	SLOT(updateBodyPartsBox())	);
	connect(	Part2_pose6D, 			SIGNAL(clicked()), 	this, 	SLOT(updateBodyPartsBox())	);
	connect(	Part3_pose6D, 			SIGNAL(clicked()), 	this, 	SLOT(updateBodyPartsBox())	);
	connect(	camareroZurdoButton, 	SIGNAL(clicked()),	this, 	SLOT(camareroZurdo())		);
	connect(	camareroDiestroButton, 	SIGNAL(clicked()), 	this, 	SLOT(camareroDiestro())		);
	connect(	camareroCentroButton, 	SIGNAL(clicked()),	this, 	SLOT(camareroCentro())		);
	connect(	esfera, 				SIGNAL(clicked()),	this, 	SLOT(puntosEsfera())		);
	connect(	cubo,					SIGNAL(clicked()), 	this, 	SLOT(puntosCubo())			);
	
	// BOTONES DE LA PESTAÑA AXIS ALING:
	connect(	sendButton2,		SIGNAL(clicked()),	this,	SLOT(send())				);
	connect(	Part1_AxisAlign, 	SIGNAL(clicked()), 	this, 	SLOT(updateBodyPartsBox())	);
	connect(	Part2_AxisAlign, 	SIGNAL(clicked()), 	this, 	SLOT(updateBodyPartsBox())	);
	connect(	Part3_AxisAlign, 	SIGNAL(clicked()), 	this, 	SLOT(updateBodyPartsBox())	);
	
	// BOTONES DE LA PESTAÑA ADVANCE AXIS:
	connect(	sendButton3,		SIGNAL(clicked()),	this,	SLOT(send())				);
	connect(	Part1_AlongAxis, 	SIGNAL(clicked()), 	this, 	SLOT(updateBodyPartsBox())	);
	connect(	Part2_AlongAxis, 	SIGNAL(clicked()), 	this, 	SLOT(updateBodyPartsBox())	);
	connect(	Part3_AlongAxis, 	SIGNAL(clicked()), 	this, 	SLOT(updateBodyPartsBox())	);
	
	//BOTONES DE LA PESTAÑA HOME:
	connect(	sendButton4,	SIGNAL(clicked()),	this,	SLOT(send())	);
	
	// BOTONES DE LA PESTAÑA FINGERS:
	connect(	sendButton5,	SIGNAL(clicked()),	this,	SLOT(send())	);

	// BOTONERA AÑADIDA: UNIT TEST
	connect(test1Button, SIGNAL(clicked()), this, SLOT(abrirPinza()));
	connect(test2Button, SIGNAL(clicked()), this, SLOT(posicionInicial()));
	connect(test3Button, SIGNAL(clicked()), this, SLOT(posicionCoger()));
	connect(test4Button, SIGNAL(clicked()), this, SLOT(cerrarPinza()));
	connect(test5Button, SIGNAL(clicked()), this, SLOT(posicionSoltar()));
	connect(test6Button, SIGNAL(clicked()), this, SLOT(izquierdoRecoger()));
	connect(test7Button, SIGNAL(clicked()), this, SLOT(abrirPinza()));
	connect(test8Button, SIGNAL(clicked()), this, SLOT(retroceder()));
	connect(test9Button, SIGNAL(clicked()), this, SLOT(goHomeR()));	
	connect(test10Button, SIGNAL(clicked()), this, SLOT(izquierdoOfrecer()));
	connect(test11Button, SIGNAL(clicked()), this, SLOT(enviarHome()));
	
	//
	connect(boton1, SIGNAL(clicked()), this, SLOT(boton_1()));
	connect(boton2, SIGNAL(clicked()), this, SLOT(boton_2()));
	connect(boton3, SIGNAL(clicked()), this, SLOT(boton_3()));
	connect(boton4, SIGNAL(clicked()), this, SLOT(boton_4()));
	connect(boton5, SIGNAL(clicked()), this, SLOT(boton_5()));
	
	connect(aprilSendButton, SIGNAL(clicked()), this, SLOT(ballisticPartToAprilTarget()));
	connect(aprilFineButton, SIGNAL(clicked()), this, SLOT(finePartToAprilTarget()));
	//Esta señal la emite el QTabWidget cuando el usuario cambia el tab activo

}


/**
 * @brief Método INIT DIRECT KINEMATICS FLANGE.
 * Inicializa los valores angulares de los motores del robot, así como sus límites máximos y mínimos.
 * Ampliable para añadir funcionalidad a la velocidad de movimiento, el nuevo ángulo y esas cosillas 
 * extras que quedan muy monas...
 * 
 * @return void
 */ 
void SpecificWorker::initDirectKinematicsFlange()
{
	qDebug()<<"Inicializando valores angulares...";
	try
	{
		// Sacamos todos los parámetros de los motores del robot. Esto nos servirá para inicializar la pestaña 
		// de direct Kinematics de la interfaz de usuario.
		motorparamList = jointmotor_proxy->getAllMotorParams();

		foreach(RoboCompJointMotor::MotorParams motorParam, motorparamList)
		{
			// Ponemos los datos de los límites min y max a TODOS los motores. (por eso lo apiñurgo un poco, porque quedaría
			// un tochaco de código insufrible --y aún así lo es--)
			motorList.push_back(motorParam.name);

			// BRAZO IZQUIERDO:
			if(motorParam.name == shoulder1Left->text().toStdString()){	angleMaxSL1->display(motorParam.maxPos);	angleMinSL1->display(motorParam.minPos);	velocityNewSL1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == shoulder2Left->text().toStdString()){	angleMaxSL2->display(motorParam.maxPos);	angleMinSL2->display(motorParam.minPos);	velocityNewSL2->setValue(motorParam.maxVelocity);}
			if(motorParam.name == shoulder3Left->text().toStdString()){	angleMaxSL3->display(motorParam.maxPos);	angleMinSL3->display(motorParam.minPos);	velocityNewSL3->setValue(motorParam.maxVelocity);}
			if(motorParam.name == elbowLeft->text().toStdString()){		angleMaxEL->display(motorParam.maxPos);		angleMinEL->display(motorParam.minPos);		velocityNewEL->setValue(motorParam.maxVelocity);}
			if(motorParam.name == foreArmLeft->text().toStdString()){	angleMaxFAL->display(motorParam.maxPos);	angleMinFAL->display(motorParam.minPos);	velocityNewFAL->setValue(motorParam.maxVelocity);}
			if(motorParam.name == wristLeft1->text().toStdString()){	angleMaxWL1->display(motorParam.maxPos);	angleMinWL1->display(motorParam.minPos);	velocityNewWL1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == wristLeft2->text().toStdString()){	angleMaxWL2->display(motorParam.maxPos);	angleMinWL2->display(motorParam.minPos);	velocityNewWL2->setValue(motorParam.maxVelocity);}

			// BRAZO DERECHO:	
			if(motorParam.name == shoulder1Right->text().toStdString()){angleMaxSR1->display(motorParam.maxPos);	angleMinSR1->display(motorParam.minPos);	velocityNewSR1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == shoulder2Right->text().toStdString()){angleMaxSR2->display(motorParam.maxPos);	angleMinSR2->display(motorParam.minPos);	velocityNewSR2->setValue(motorParam.maxVelocity);}
			if(motorParam.name == shoulder3Right->text().toStdString()){angleMaxSR3->display(motorParam.maxPos);	angleMinSR3->display(motorParam.minPos);	velocityNewSR3->setValue(motorParam.maxVelocity);}
			if(motorParam.name == elbowRight->text().toStdString()){	angleMaxER->display(motorParam.maxPos);		angleMinER->display(motorParam.minPos);		velocityNewER->setValue(motorParam.maxVelocity);}
			if(motorParam.name == foreArmRight->text().toStdString()){	angleMaxFAR->display(motorParam.maxPos);	angleMinFAR->display(motorParam.minPos);	velocityNewFAR->setValue(motorParam.maxVelocity);}
			if(motorParam.name == wristRight1->text().toStdString()){	angleMaxWR1->display(motorParam.maxPos);	angleMinWR1->display(motorParam.minPos);	velocityNewWR1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == wristLeft2->text().toStdString()){	angleMaxWR2->display(motorParam.maxPos);	angleMinWR2->display(motorParam.minPos);	velocityNewWR2->setValue(motorParam.maxVelocity);}

			// CABEZA:
			if(motorParam.name == head1->text().toStdString()){	angleMaxH1->display(motorParam.maxPos);	angleMinH1->display(motorParam.minPos);	velocityNewH1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == head2->text().toStdString()){	angleMaxH2->display(motorParam.maxPos);	angleMinH2->display(motorParam.minPos);	velocityNewH2->setValue(motorParam.maxVelocity);}
			if(motorParam.name == head3->text().toStdString()){	angleMaxH3->display(motorParam.maxPos);	angleMinH3->display(motorParam.minPos);	velocityNewH3->setValue(motorParam.maxVelocity);}

			// BASE:
			// TODO
		}

	} catch(const Ice::Exception &ex) {cout<<"--> Excepción en initDirectKinematicsFlange al tomar datos del robot: "<<ex<<endl;}
}

/**
 * @brief Método SHOW KINEMATICS DATA. 
 * Este método se encarga de mostrar los datos de los motores por la pestaña DirectKinematics de la interfaz
 * gráfica de usuario: el nombre del motor, los limites mínimo y máximo que tiene, la posicion angular en 
 * radianes actual y la velocidad en radianes por segundo.
 * 
 * @return void
 */ 
void SpecificWorker::showKinematicData()
{
	try
	{
		// Recuerda: motorparamList ya lo tenemos relleno de datos desde el setParams.
		foreach(RoboCompJointMotor::MotorParams motorParam, motorparamList)
		{
			// Sacamos parámetros dinámicos del motor.
			RoboCompJointMotor::MotorState motorState = jointmotor_proxy->getMotorState(motorParam.name);

			// BRAZO IZQUIERDO:
			if(motorParam.name == shoulder1Left->text().toStdString()){	angleCurrentSL1->display(motorState.pos);	velocityNewSL1->setValue(motorState.vel); }
			if(motorParam.name == shoulder2Left->text().toStdString()){	angleCurrentSL2->display(motorState.pos);	velocityNewSL2->setValue(motorState.vel); }
			if(motorParam.name == shoulder3Left->text().toStdString()){	angleCurrentSL3->display(motorState.pos);	velocityNewSL3->setValue(motorState.vel); }
			if(motorParam.name == elbowLeft->text().toStdString()){		angleCurrentEL->display(motorState.pos);	velocityNewEL->setValue(motorState.vel); }
			if(motorParam.name == foreArmLeft->text().toStdString()){	angleCurrentFAL->display(motorState.pos);	velocityNewFAL->setValue(motorState.vel); }
			if(motorParam.name == wristLeft1->text().toStdString()){	angleCurrentWL1->display(motorState.pos);	velocityNewWL1->setValue(motorState.vel); }
			if(motorParam.name == wristLeft2->text().toStdString()){	angleCurrentWL2->display(motorState.pos);	velocityNewWL2->setValue(motorState.vel); }

			// BRAZO DERECHO
			if(motorParam.name == shoulder1Right->text().toStdString()){angleCurrentSR1->display(motorState.pos);	velocityNewSR1->setValue(motorState.vel); }
			if(motorParam.name == shoulder2Right->text().toStdString()){angleCurrentSR2->display(motorState.pos);	velocityNewSR2->setValue(motorState.vel); }
			if(motorParam.name == shoulder3Right->text().toStdString()){angleCurrentSR3->display(motorState.pos);	velocityNewSR3->setValue(motorState.vel); }
			if(motorParam.name == elbowRight->text().toStdString()){	angleCurrentER->display(motorState.pos);	velocityNewER->setValue(motorState.vel); }
			if(motorParam.name == foreArmRight->text().toStdString()){	angleCurrentFAR->display(motorState.pos);	velocityNewFAR->setValue(motorState.vel); }
			if(motorParam.name == wristRight1->text().toStdString()){	angleCurrentWR1->display(motorState.pos);	velocityNewWR1->setValue(motorState.vel); }
			if(motorParam.name == wristRight2->text().toStdString()){	angleCurrentWR2->display(motorState.pos);	velocityNewWR2->setValue(motorState.vel); }

			// CABEZA:
			if(motorParam.name == head1->text().toStdString()){	angleCurrentH1->display(motorState.pos);	velocityNewH1->setValue(motorState.vel); }
			if(motorParam.name == head2->text().toStdString()){	angleCurrentH2->display(motorState.pos);	velocityNewH2->setValue(motorState.vel); }
			if(motorParam.name == head3->text().toStdString()){	angleCurrentH3->display(motorState.pos);	velocityNewH3->setValue(motorState.vel); }
		}				
	} catch(const Ice::Exception &ex) {cout<<"--> Excepción en showKinematicData: "<<ex<<endl;}
}

/**
 * @brief Método SHOW KINEMATICS DATA. 
 * Este método se encarga de mostrar los datos de los motores por la pestaña DirectKinematics de la interfaz
 * gráfica de usuario: el nombre del motor, los limites mínimo y máximo que tiene, la posicion angular en 
 * radianes actual y la velocidad en radianes por segundo.
 * 
 * @param type entero que indica que texto hay que poner en las ventanitas.
 * 
 * @return void
 */ 
void SpecificWorker::changeText(int type)
{
	if(type==0)
	{
		aDonde1->clear(); aDonde1->insertPlainText("El target se enviara al RCIS .\n Para cambiar el destino del target pulse los botones superiores.");
		aDonde2->clear(); aDonde2->insertPlainText("El target se enviara al RCIS .\n Para cambiar el destino del target pulse los botones superiores.");
		aDonde3->clear(); aDonde3->insertPlainText("El target se enviara al RCIS .\n Para cambiar el destino del target pulse los botones superiores.");
		aDonde4->clear(); aDonde4->insertPlainText("El target se enviara al RCIS .\n Para cambiar el destino del target pulse los botones superiores.");
		aDonde5->clear(); aDonde5->insertPlainText("El target se enviara al RCIS .\n Para cambiar el destino del target pulse los botones superiores.");

	}
	else
	{
		aDonde1->clear(); aDonde1->insertPlainText("El target se enviara al ROBOT REAL .\n Para cambiar el destino del target pulse los botones superiores.");
		aDonde2->clear(); aDonde2->insertPlainText("El target se enviara al ROBOT REAL .\n Para cambiar el destino del target pulse los botones superiores.");
		aDonde3->clear(); aDonde3->insertPlainText("El target se enviara al ROBOT REAL .\n Para cambiar el destino del target pulse los botones superiores.");
		aDonde4->clear(); aDonde4->insertPlainText("El target se enviara al ROBOT REAL .\n Para cambiar el destino del target pulse los botones superiores.");
		aDonde5->clear(); aDonde5->insertPlainText("El target se enviara al ROBOT REAL .\n Para cambiar el destino del target pulse los botones superiores.");
	}
}
