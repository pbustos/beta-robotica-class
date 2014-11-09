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
		BodyPart(InnerModel *in, Cinematica_Inversa *invkin, QString p, QString t, QStringList ml);
 		BodyPart()=default; 						// Constructor por defecto
		BodyPart(const BodyPart& bp);
		BodyPart(BodyPart&& bp);
 		BodyPart& operator=(BodyPart bp);
		friend void swap(BodyPart &first, BodyPart &second);
		~BodyPart(){};	
	
		QMutex mutex;
		//// MÉTODOS GET ////
		QString getPartName()		 									{ QMutexLocker ml(&mutex); return part; } // Devuelve el nombre de la parte del cuerpo. 
		QString getTip()  												{ QMutexLocker ml(&mutex); return tip; };	// Devuelve el nombre del efector final de la parte del cuerpo.
		QStringList getMotorList()										{ QMutexLocker ml(&mutex); return motorList;}; // Devuelve la lista de motores de la parte del cuerpo.
		QQueue<Target> getTargets()  									{ QMutexLocker ml(&mutex); return listaTargets;}; // Devuelve toda la cola de targets de la parte del cuerpo.
		Target& getHeadFromTargets() 					 				{ QMutexLocker ml(&mutex); return listaTargets.head(); }; //Devuelve el primer target de la cola de targets.
		Cinematica_Inversa* getInverseKinematics()						{ QMutexLocker ml(&mutex); return ik;}; // Devuelve la variable de cinematica_inversa asignada a la partedel cuerpo.
		
		void addListaTarget(const QQueue<Target> &lt)					{ QMutexLocker ml(&mutex); listaTargets = lt;}; //Guarda la lista de targets que se le asigna en su atributo.
		void addTargetToList(const Target &t, bool replace = false)		{ QMutexLocker ml(&mutex); if( replace ) listaTargets.clear(); listaTargets.enqueue(t); };
		void removeHeadFromTargets()									{ QMutexLocker ml(&mutex); if (listaTargets.size() > 0) listaTargets.dequeue();}; // Elimina el primer target de la cola de targets
		bool noTargets() 												{ QMutexLocker ml(&mutex); return listaTargets.isEmpty();};
		void markForRemoval()											{ QMutexLocker ml(&mutex); for( int i=0; i<listaTargets.size(); i++) listaTargets[i].markForRemoval(true);};
		void addJointStep(const QVec &joints)							{ QMutexLocker ml(&mutex); jointStepList.append(joints);};
		QList<QVec> getJointStepList() 		 							{ QMutexLocker ml(&mutex); return jointStepList;};
		void cleanJointStep()											{ QMutexLocker ml(&mutex); jointStepList.clear();};
		void setNewVisualTip(const RoboCompBodyInverseKinematics::Pose6D &pose)			{ QMutexLocker ml(&mutex); ik->setNewTip(pose);}; 
		
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
