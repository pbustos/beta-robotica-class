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

#include "../../inversekinematicsComp/src/target.h"
#include <qt4/Qt/qcheckbox.h>
#include <qt4/Qt/qwidget.h>
#include <qt4/QtGui/qcombobox.h>
#include <qt4/QtGui/qlabel.h>
#include <qt4/QtGui/qspinbox.h>
#include <qt4/QtGui/QFrame>
#include <time.h>
#include <osgviewer/osgview.h>
#include <innermodel/innermodelviewer.h>
// #include <innermodel/innermodelreader.h>


/**
       \brief INVERSE KINEMATICS TESTER COMP
       @author authorname
*/


class SpecificWorker : public GenericWorker
{
	
Q_OBJECT
public:
	// Constructores y destructores:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	
	// Métodos públicos:
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void  newAprilTag(const tagsList& tags);

public slots:

	void 	compute				(); 
	
	//// SLOTS DE LOS BOTONES DE EJECUCIÓN DE LA INTERFAZ. ////
	void 	stop				(); 		// Botón de parada segura. Para abortar la ejecución del movimiento tanto en RCIS como en Robot real
	
	//// SLOTS DE LA PESTAÑA POSE6D ////

	
	//// SLOTS ÚTILES PARA FACILITAR EL USO DE LA GUI ////
	void 	updateBodyPartsBox	(); 		//Activa/desactiva la opción de escoger una o más partes del cuerpo.
	void 	send				();			// Botón que indica que existe un target para ser resuelto.

	
	void 	enviarRCIS(); 		// select RCIS.
	void 	enviarROBOT(); 	// select robot
	
	// Métods AÑADIDOS +++
	void boton_1();
	void boton_2();
	void boton_3();
	void boton_4();
	void boton_5();

	
	
	void enviarHome();
	void actualizarInnerModel();


	// SLOTS DE LA PESTAÑA POSE6D:
	void camareroZurdo();
	void camareroDiestro();
	void camareroCentro();
	void puntosEsfera();
	void puntosCubo();
	
	void closeFingers();
	void goHome(QString partName);

	// MÉTODOS AÑADIDOS: REVISAR
	void abrirPinza();
	void cerrarPinza();
	void posicionInicial();
	void posicionCoger();
	void posicionSoltar();
	void retroceder();
	void goHomeR();
	void izquierdoRecoger();
	void izquierdoOfrecer();
	



	void ballisticPartToAprilTarget(int xoffset = 100);
	void finePartToAprilTarget();
private:

	//// ATRIBUTOS PRIVADOS DE LA CLASE ////
	RoboCompJointMotor::MotorParamsList 	motorparamList;			// Lista de parámetros de los motores del robot. Para sacar valores angulares.
	RoboCompJointMotor::MotorList 			motorList;				// Lista con los nombres de los motores del robot.

	InnerModel 			*innerModel;								// Puntero para trabajar con el innerModel (pintar el target y obtener valores angulares)
	InnerModelViewer 	*imv;										// Puntero para pintar el innerModel en la pestaña DirectKinematics
	OsgView 			*osgView;									// Puntero para pintar innerModel en una de las pestañas.
	QFrame				*frameOsg;									// Puntero para pintar innerModel en una de las pestañas.
	QQueue<QVec>		trayectoria;								// Cola de poses donde se guardan las trayectorias de los Camareros
	QVec				partesActivadas;							// Vector de partes (se ponen a 0 si NO se les envía target y a 1 si SÍ se les envía target)
	QVec 				marcaBote;
	QVec 				manoApril;
	
	bool				flagListTargets;							// Se pone a TRUE si hay una trayectoria para enviar. FALSE si no hay trayectoria.
	bool				existTarget;								// Se pone a TRU cuando hay un target o una lista de targets a enviar al RCIS o al ROBOT
	QString 			tabName;									//Name of current tab
	int 				tabIndex;									//Index of current tabIndex	

	
		
	// MÉTODOS
	void moverTargetEnRCIS(const QVec &pose);
	void enviarPose6D(QVec p);
	void enviarAxisAlign();
	void moveAlongAxis();
	void calcularModuloFloat(QVec &angles, float mod);
	
	
	
	////////////////  MÉTODOS PRIVADOS  ////////////////
	/// MÉTODOS PRIVADOS IMPORTANTES ///
	void 	sendTarget					();				// Envia los targets, HOME o FINGERS al RCIS o al ROBOT.

	/// MÉTODOS MUY SIMPLES Y AUXILIARES TOTALES QUE NO TIENEN MAYOR IMPORTANCIA PERO QUE LIMPIAN CÓDIGO ///
	void 	connectButtons				();				// Conecta botones de la interfaz de usuario con sus SLOTS correspondientes.
	void 	initDirectKinematicsFlange	();				// Inicializa datos de la pestaña DirectKinematics. 
	void 	showKinematicData			();				// Mantiene actualizados los valores de la pestaña DirectKinematics.
	void 	changeText					(int type);		// Cambia el texto de las ventanitas aDonde

};

#endif
