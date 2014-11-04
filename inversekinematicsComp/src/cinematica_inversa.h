/*
    Copyright (c) 2014 <copyright holder> <email>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef CINEMATICA_INVERSA_H
#define CINEMATICA_INVERSA_H

/*
 * CLASE AÑADIDA: CINEMATICA INVERSA.
 * Se encarga de calcular todo lo referente a la cinemática. 
 *        - Realiza los cálculos de Levenber-Marquart 
 *        - Devuelve la lista de ángulos que hay que cambiar para alcanzar el target.
 * (Programo en español y punto!!!)
 */
#include <QtCore>
#include <qt4/QtCore/QList>
#include <qt4/QtCore/qstring.h>
#include <qt4/QtCore/qdebug.h>

#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>
#include "target.h"
#include <osgviewer/osgview.h>
#include <eigen3/Eigen/SVD>
#include <BodyInverseKinematics.h>

using namespace std;

class Cinematica_Inversa
{
public:
	Cinematica_Inversa(InnerModel *inner_, QStringList joints_, QString endEffector_);  ///QUITAR ENDEFFECTOR DE AQUI. YA VA EN EL TARGET
	~Cinematica_Inversa();
	
	
	///// MÉTODOS PÚBLICOS /////
	void setNewTip( const RoboCompBodyInverseKinematics::Pose6D &tip);
	void resolverTarget( Target &target);			// Fija el punto objetivo.
	
private:
	
	///// VARIABLES DE CLASE /////
	
	InnerModel *inner;				// Inermodel para sacar información del robot
	QStringList listaJoints;		// Lista de motores que entra en liza
	QString endEffector;			// Nombre del efector final (la mano del robot)
	bool lowIncrementFirst;
	int firstTime;

	///// MÉTODOS PRIVADOS /////
	// ----------------- PARA TRASLACIÓN Y ROTACIÓN ---------------//
	QMat jacobian(QVec motores);				// devuelve la matriz jacobiana de la función.
	QVec computeErrorVector(const Target &target);		//devuelve el vector error de traslaciones y rotaciones
	void levenbergMarquardt(Target &target);		// algoritmo de Levenberg-Marquart completo.
	void levenbergMarquardt2(Target &target); //alternative implementaiton
	void chopPath(Target &target);
	bool comprobarBucleChop(QList<QVec> listaSubtargets, QVec subTarget);
	//_-----------------------------------------------------------------------------
	
	// DE CÁLCULO.....
	QVec calcularAngulos(); //devuelve el vcetor de todos los ángulos de los motores
	void calcularModuloFloat(QVec &angles, float mod);

	// DE ACTUALIZACIÓN...
	void actualizarAngulos(QVec angulos_nuevos);
	
	// DE CONSULTA...
	bool outLimits(QVec &angulos, QVec &motores); //devuelve si los ángulos para los motores no superan los límites
	QVec computeH(const QVec &angs);
};

#endif // CINEMATICA_INVERSA_H
