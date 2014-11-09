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


#ifndef TARGET_H
#define TARGET_H

/*
 * CLASE AÑADIDA: TARGET.
 * Sirve para marcar puntos objetivos donde llevar diferentes partes del robot, como el brazo
 * derecho, el brazo izquierdo o la cabeza... Necesita varias cosas:
 * 		- Un vector POSE que le indique cuál es su traslación y su rotación.
 * 		- El tip o endEffector al que está asignado.
 * 		- El innerModel parapoder trocear la trayectoria desde el endEffector hasta el target.
 */ 
#include <innermodel/innermodel.h>
#include <qt4/QtCore/qstring.h>
#include <qt4/QtCore/QTime>
#include <qt4/QtCore/qmap.h>
#include <qt4/QtCore/qqueue.h>

using namespace std;

class Target
{
	
public:
	
	enum TargetType {POSE6D, ALIGNAXIS, ADVANCEAXIS};
	enum FinishStatus { LOW_ERROR, KMAX, LOW_INCS, NAN_INCS, START };
	
	Target()=default;
	Target(TargetType tt, InnerModel *inner, const QString &tip, const QVec &pose6D, const QVec &weights, bool chop=true);											// For Pose6D
	Target(TargetType tt, InnerModel *inner, const QString &tip, const QVec &pose6D, const QVec &axis, const QVec &weights);		// For AlingAxis
	Target(TargetType tt, InnerModel* inner, const QString &tip, const QVec& axis, float step);																	// For ADVANCEALONGAXIS
	Target& operator=( Target tmp );
	Target(const Target & t);
	~Target();
	
	// MÉTODOS GET:
	QString getTipName() const           { return this->tip; }            		 	// Devuelve el nombre del TIP.
	QVec getPose() const                 { return this->pose6D; }          			// Devuelve el vector pose del target
	QVec getTranslation() const			 { return this->pose6D.subVector(0,2);}  	//Returns the translation component of Pose6D
	QVec getRotation() const			 { return this->pose6D.subVector(3,5);}  	//Returns the rotation component of Pose6D
	QTime getStartTime() const           { return this->start; }           			// Devuelve el tiempo del target.
	bool isActive() const          	     { return this->activo; }          			// Devuelve el estado del target
	QVec getWeights() const              { return this->weights; } 
	TargetType getType() const           { return targetType; }
	bool getAxisConstraint() const       { return axisConstraint; }
	float getAxisAngleConstraint() const { return axisAngleConstraint; }
	QString getNameInInnerModel() const  { return nameInInnerModel; }
	float getError() const               { return error; }
	QVec getAxis() const                 { return axis; }
	float getStep() const                { return step; }
	QTime getRunTime() const             { return runTime; }
	int getElapsedTime() const           { return runTime.elapsed(); }
	FinishStatus getStatus() const       { return finish; }
	QVec getErrorVector() const          { return errorVector; }
	QVec getFinalAngles() const          { return finalAngles; }
	QVec getIncrementalAngles() const	 { return finalAngles - initialAngles;}
	bool getExecuted() const          	 { return executed; }
	bool isChopped() const				 { return chopped; }
	bool isMarkedforRemoval() const		 { return removal; }
	float getRadius() const 			 { return radius; }
	bool isAtTarget() const 			 { return atTarget; };
	bool getHasPlan() const 			 { return hasPlan; };

	// MÉTODOS SET:
	void setPose(const QVec &newPose)				{ this->pose6D = newPose;};
	void setChoppedPose(const QVec &newPose)		{ this->pose6DChopped = newPose;}
	void setStartTime (QTime newStart)				{ this->startTime = newStart;}
	void setActivo (bool newActivo)					{ this->activo = newActivo; };
	void setWeights(const QVec &weights)  			{ this->weights = weights; };
	void setNameInInnerModel(const QString &name)	{ this->nameInInnerModel = name;};
	void setError(float error)           			{ this->error = error; };
	void setStatus(FinishStatus status)  			{ finish = status; };
	void setElapsedTime(ulong e)         			{ elapsedTime = e; };
	void setRunTime(const QTime &t)      			{ runTime = t; };
	void setIter(uint it)                			{ iter = it; }
	void setErrorVector(const QVec &e)   			{ errorVector = e; }
	void setFinalAngles(const QVec &f)  			{ finalAngles = f; }
	void setExecuted(bool e)             			{ executed = e; }
	void setChopped(bool c)							{ chopped = c; }
	
	void annotateInitialTipPose()
	{
      initialTipPose.inject(inner->transform("world", QVec::zeros(3), getTipName()),0);
      initialTipPose.inject(inner->getRotationMatrixTo("world",getTipName()).extractAnglesR_min(),3);
	};
	void annotateFinalTipPose()
	{
        finalTipPose.inject(inner->transform("world", QVec::zeros(3), getTipName()),0);
        finalTipPose.inject(inner->getRotationMatrixTo("world",getTipName()).extractAnglesR_min(),3);
	};
	void setInitialAngles(const QStringList &motors)
	{
		initialAngles.clear();
        foreach( QString motor, motors)
            initialAngles.append(inner->getJoint(motor)->getAngle());
	}
	void markForRemoval( bool m) 					{ removal = m; };
	void setRadius(float r)       					{ radius = r; };
	void setAtTarget(bool a)  						{ atTarget = a; };
	void setHasPlan(bool a)							{ hasPlan = a;};
	
	// OTROS MÉTODOS
	void print(const QString &msg = QString());
	
private:
	
    QString tip;                     		// Nombre del efector final al que está asociado el target
    QTime start;							// Tiempo en que comenzó a trabajar el robot con el target original.
    bool activo;							// Bandera para indicar si el target es válido y el robot debe trabajar con él o no.
    QVec pose6D; 							// Vector de 6 elementos, 3 traslaciones y 3 rotaciones: tx, ty, tz, rx, ry, rz
    QVec pose6DChopped;						// Vector de 6 elementos, 3 traslaciones y 3 rotaciones: tx, ty, tz, rx, ry, rz
    QVec axis;								// Target axis to be aligned with
    InnerModel *inner;						// Innermodel para calcular cosas.
    QVec weights;							// Pesos para restringir translaciones, rotaciones o ponderar el resultado
    TargetType targetType;					//
    bool axisConstraint;					// True if constraint is to be applien on axis for ALINGAXIS mode
	float axisAngleConstraint;				// constraint oto be applied to axis in case axisConstrint is TRUE
    QString nameInInnerModel;				// generated name to create provisional targets
    float error;							// Error after IK
    QVec errorVector;						// Error vector
    float step;								// step to advance along axis
    QTime startTime;						// timestamp indicating when the target is created
    QTime runTime;							// timestamp indicating when the target is executed
	ulong elapsedTime;          			// timestamp indicating when the duration of the target execution in milliseconds.
	FinishStatus finish;        			// Enumerated to show finish reason
    uint iter;								// Number of iterations before completing
    QVec finalAngles;						// Mercedes lo documenta luego
    QVec initialAngles;               		// Angles before executing this target
    QVec initialTipPose;            		// Tip position in world reference frame BEFORE processing
    QVec finalTipPose;              		// Tip position in world reference frame after processing
    bool executed;                    		// true if finally executed in real arm
	bool chopped;							// true if the target  has been chopped
	float radius; 							// if > 0 the robot stops when reaching a ball of radius "radius" with center in Pose6D
	bool removal;           		         // 
	bool atTarget;
	bool hasPlan;							//if false a plan has to be computed and stored
};


#endif // TARGET_H
