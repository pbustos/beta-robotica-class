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


#ifndef GENERADOR_H
#define GENERADOR_H

/*
 * CLASE AÑADIDA: GENERADOR .
 * Se encarga de calcular puntos:
 * 		- puntos donde colocar el nodo efector (la mano del brazo robótico)
 * 		- puntos alrededor de la mano.
 */

#include <QtCore>
#include <QTime>
#include <qt4/QtCore/QList>
#include <qt4/QtCore/qstring.h>
#include <qt4/QtCore/qstringlist.h>
#include <qt4/QtCore/qmap.h>
#include <qt4/QtCore/qpair.h>
#include <qt4/QtCore/qqueue.h>
#include <innermodel/innermodel.h> //si no lo pngo no reconoce QVec
#include<stdlib.h>
#include<time.h>
#include "target.h"
#include "bodypart.h"

using namespace std;
class SpecificWorker;
class Generador
{

public:
	
	Generador();
	~Generador();
	
	//// MÉTODOS PÚBLICOS ////
	QQueue<Target> generarListaTargets(InnerModel *inner, const  BodyPart &bodyParts, const QVec &weights);
	
	QList<QVec> generarPuntosTargetRectaY(QVec coordenadasRobot, QVec rotacionRobot, int nPuntos, float Nm); //genera puntos a lo largo de una recta en ele eje Y
	QList<QVec> generarPuntosTargetRectaZ(QVec coordenadasRobot, QVec rotacionRobot, int nPuntos, float Nm); //genera puntos a lo largo de una recta en ele eje Z
	QList<QVec> generarPuntosTargetEsfera(QVec centroRobot, QVec rotacionRobot, int nPuntos, float radio); //genera puntos de una circunferencia de radio "radio" y centro en "centroRobot".
	QList<QVec> generarAngulosMotores(QStringList motores, int nMuestras, InnerModel *inner); //genera ángulos aleatorios para nMotores.
	
private:
		
	//// MÉTODOS PRIVADOS ////
	QVec generarPunto(int i, int nPuntos, QVec centro, int radio); 	//crea puntos de una circunferencia.
	QVec generarPuntoEsfera(int i, QVec centro, int radio);			// añadido
	QList<QVec> generarPuntosCamareroDiestro();
	QList<QVec> generarPuntosCamareroZurdo();
	QQueue< Target > generarPuntosCamareroCentro();
	QQueue< Target > generarPuntosCabeza();
	QQueue< Target > generarPuntosCabezaCentro();
	InnerModel* inner;
	BodyPart bodyPart;
	QVec weights;

protected:
	void time(int arg1);
};

#endif // GENERADOR_H
