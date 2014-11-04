/*
 * Copyright 2014 pbustos <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef BODYPART_H
#define BODYPART_H

#include "cinematica_inversa.h"
#include <qt4/QtCore/qqueue.h>
#include <boost/graph/graph_concepts.hpp>
#include "target.h"
#include <innermodel/innermodel.h>
#include <qt4/QtCore/qstringlist.h>


class BodyPart
{
	public: 
		
		/**
		 * \brief Constructor Parametrizado. Recibe como parámetros de entrada para inicializar atributos:
		 * 			- el innerModel (que por ahora no lo necesita porque no hace ningún cálculo con él).
		 * 			- invKin: variable de cinemática inversa que tiene asociada la clase.
		 * 			- p: nombre de la parte del cuerpo del robot (LEFTARM, RIGHTARM o HEAD).
		 * 			- ml: lista de motores asociados a esa parte del cuerpo.
		 * 			- t: nombre del efector final de la parte del cuerpo.
		 */ 
		BodyPart(InnerModel *in, Cinematica_Inversa *invkin, QString p, QString t, QStringList ml)
		{ 
			inner = in; 	
			ik=invkin;
			part=p;
			motorList=ml; 	
			tip=t;
		};

 		BodyPart(){}; 	// Constructor por defecto
		~BodyPart(){};	// destructor por defecto.
		
		//// MÉTODOS GET ////
		QString getPartName() const 						{ return part;}; // Devuelve el nombre de la parte del cuerpo. 
		QString getTip() const 								{ return tip; };	// Devuelve el nombre del efector final de la parte del cuerpo.
		QStringList getMotorList() const 					{ return motorList;}; // Devuelve la lista de motores de la parte del cuerpo.
		QQueue<Target> getTargets() const 					{ return listaTargets;}; // Devuelve toda la cola de targets de la parte del cuerpo.
		Target& getHeadFromTargets() 					 	{ return listaTargets.head(); }; //Devuelve el primer target de la cola de targets.
		Cinematica_Inversa* getInverseKinematics()			{ return ik;}; // Devuelve la variable de cinematica_inversa asignada a la partedel cuerpo.
		void addListaTarget(const QQueue<Target> &lt)		{ listaTargets = lt;}; //Guarda la lista de targets que se le asigna en su atributo.
		void addTargetToList(const Target &t)				{ listaTargets.enqueue(t);};
		void removeHeadFromTargets()						{ if (listaTargets.size() > 0) listaTargets.dequeue();}; // Elimina el primer target de la cola de targets
		bool noTargets() const								{ return listaTargets.isEmpty();};
		void markForRemoval()								{ for( int i=0; i<listaTargets.size(); i++) listaTargets[i].markForRemoval(true);};
		void addJointStep(const QVec &joints)				{ jointStepList.append(joints);};
		QList<QVec> getJointStepList() const 				{ return jointStepList;};
		void cleanJointStep()								{ jointStepList.clear();};
		void setNewVisualTip(const RoboCompBodyInverseKinematics::Pose6D &pose)			{ ik->setNewTip(pose);}; 
		
		
	private:
		QString part;													// Nombre de la parte del cuerpo.
		QString tip;													// Nombre del efector final de esa parte del cuerpo.
		QStringList motorList;											// Lista de motores asignados a esa parte del cuerpo
		Cinematica_Inversa *ik;											// Puntero a la cinemática inversa de esa parte del cuerpo
		InnerModel *inner;												// POR AHORA INÚTIL.
		QQueue<Target> listaTargets;									// Lista de targets para esa parte del cuerpo.
		QList<QVec> jointStepList;										// List of 
};





#endif // BODYPART_H
