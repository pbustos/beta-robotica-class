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

#include "cinematica_inversa.h"
#include <qt4/QtCore/qstringlist.h>
#include <boost/graph/graph_concepts.hpp>
#include <eigen3/Eigen/src/Eigenvalues/ComplexSchur.h>

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * @brief Default Constructor
 */
Cinematica_Inversa::Cinematica_Inversa(InnerModel *inner_, QStringList joints_, QString endEffector_) 
		:	inner(inner_), listaJoints(joints_), endEffector(endEffector_)
{
	// Inicializa los atributos de la clase a partir de los : //
	
}

/**
 * @brief Default Destructor
 */
Cinematica_Inversa::~Cinematica_Inversa()
{
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS PÚBLICOS													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 

void Cinematica_Inversa::setNewTip(const RoboCompBodyInverseKinematics::Pose6D &tip)
{
	inner->updateTransformValues(endEffector,tip.x, tip.y, tip.z, tip.rx, tip.ry, tip.rz);
}



/**
 * @brief Asigna el valor de la variable de entrada (target) al atributo puntoObjetivo.
 * Es el punto al que queremos llevar el nodo effector o nodo final.
 * Hay que llamar a este metodo cada vez que queramos llevar la mano del robot a 
 * un objetivo distinto: cada vez que llamemos al Levenberg-Marquardt.
 * 
 * @param target ...
 * @return void
 */
void Cinematica_Inversa::resolverTarget(Target& target)
{
	target.setRunTime(QTime::currentTime());
	
	if(target.getType() == Target::ALIGNAXIS)  
	{
		 levenbergMarquardt(target);
	}
	else if(target.getType() == Target::ADVANCEAXIS) 
	{
		//Scale the unitary vector along direction by distance
		QVec axis = target.getAxis() * target.getStep();
		//axis.print("axis scaled");
		
		QVec p = inner->transform("world", axis, this->endEffector);
		//p.print("target to reach in world");
		inner->transform("world", QVec::zeros(3), this->endEffector).print("position of tip");
		
		QMat mat = inner->getRotationMatrixTo("world", this->endEffector);
		QVec rot = mat.extractAnglesR();
		QVec r(3);
		if(rot.subVector(0,2).norm2() < rot.subVector(3,5).norm2())
			r = rot.subVector(0,2);
		else
			r = rot.subVector(3,5);
		inner->updateTransformValues(target.getNameInInnerModel(),p.x(), p.y(), p.z(), r.x(), r.y(), r.z(), "world");
	
		levenbergMarquardt(target);
	}
	else  //POSE6D
	{
	
	//	chopPath(target);	
		// Si el target no ha sido resuelto llamamos Levenberg-Marquardt
		if( target.isAtTarget() == false )
		{
			levenbergMarquardt(target);			
			// Si hemos salido nada más ejecutar el target por incrementos pequeños volvemos a ejecutar el target
			// cambiando la matriz de pesos. LM devuelve lowIncrementFirst a true si hay incrementos pequeños e 
			// incrementa en una unidad fisrtTime, que entra a cero, sale a uno y si vuelve a entrar aquí saldrá con dos.
// 			if(lowIncrementFirst and firstTime==1)
// 			{
// 				qDebug()<<"\nENTRA POR SEGUNDA VEZ\n";
// 				levenbergMarquardt(target);
// 				lowIncrementFirst = false;
// 			}
			firstTime=0;
		}
	}
		
	target.setElapsedTime(target.getRunTime().elapsed());
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS PRIVADOS													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 

/**
 * @brief Método TROCEAR TARGET.  Crea una recta entre el tip y el target colocando subtargets cada distanciaMax
 * permitida. Troceamos con la ecuación paramétrica de una segmento entre P y Q: R= (1-Landa)*P + Landa*Q
 * 
 * @return void
 */
void Cinematica_Inversa::chopPath(Target &target)
{	
	const float step = 0.1;
	static QList<QVec> listaSubtargets;
	QVec targetTotal(6);
	
	//Si hay un target encolado previo, deberia tomar el punto de partida como la pose6d de ese target	
	QVec targetTInTip = inner->transform(this->endEffector, QVec::zeros(3), target.getNameInInnerModel());				
	QVec targetRInTip = inner->getRotationMatrixTo(this->endEffector, target.getNameInInnerModel()).extractAnglesR_min();		//orientation of tip in world
	targetTotal.inject(targetTInTip,0);
	targetTotal.inject(targetRInTip,3);
	
	float dist = (QMat::makeDiagonal(target.getWeights()) * targetTotal ).norm2();  //Error is weighted with weight matr
	
	if( dist > step)  
	{
		int nPuntos = floor(dist / step);
		T landa = 1./nPuntos;
		QVec R(6);
		QVec P(6);
		P.inject(inner->transform("world", QVec::zeros(3), this->endEffector),0);
		P.inject(inner->getRotationMatrixTo("world", this->endEffector).extractAnglesR_min(),3);
		R = (P * (T)(1.0-landa)) + (target.getPose() * landa);	
		
		
		// Añadido por Mercedes y Agustín 		
		// Target visto desde end effector
		QVec R2(6);
		R2 = (targetTotal * (T)(landa));	
		//qDebug() << "Error total visto desde end effector" << targetTotal << " R2" << R2;
				
		QMat matTipInWorld = inner->getRotationMatrixTo("world", this->endEffector);
		Rot3D matErrorInTip(R2[3], R2[4], R2[5]);
		QMat matResul = matTipInWorld * matErrorInTip;
		
		R.inject(inner->transform("world", R2.subVector(0,2), this->endEffector),0) ;
		R.inject(matResul.extractAnglesR_min(),3);
		qDebug() << "P" << P << " Rnueva" << R;
		
		if(comprobarBucleChop(listaSubtargets, R)==true)
		{
			//qDebug()<<"Incrementos muy pequeños, salimos del chop";
			target.setChopped(false);
			listaSubtargets.clear();
		}
		else
		{
			listaSubtargets.append(R);
			//Update virtual target in innermodel to chopped postion
			inner->updateTransformValues(target.getNameInInnerModel(), R.x(), R.y(), R.z(), R.rx(), R.ry(), R.rz());
			target.setChopped(true);
			target.setChoppedPose(R);
			target.setExecuted(false);
			target.setHasPlan(true);
		}
	}
	else
	{
		target.setChopped(false);
		listaSubtargets.clear();
	}
}
	
/**
 * @brief Método COMPROBAR BUCLE CHOP
 * Mira si en la lista existe algún target idéntico al nuevo subtarget calculado por el chopPath
 * o muy parecido a alguno recorriendo la lista y restando elementos hasta que alguno no supere el
 * umbral de traslaciones y de rotaciones.
 * @param listaSubtargets lista con todos los subtargets calculados por el chopPath
 * @param subTarget subtarget nuevo calculado por el chopPath.
 * 
 * @return bool
 */ 
bool Cinematica_Inversa::comprobarBucleChop(QList< QVec > listaSubtargets, QVec subTarget)
{
	//Booleano para detectar patrones y bucles
	bool subtargetRepetido=false;
	const float minTraslaciones = 0.01;
	const float minRotaciones = 0.01;
	
	//Si la lista de subtargets no está vacía hace las comprobaciones
	if(listaSubtargets.isEmpty()==false)
	{
		// Si el nuevo subtarget calculado por el chopPath ya está dentro de la lista, 
		// levanta la bandera del subtargetRepetido.
		if(listaSubtargets.contains(subTarget))
			subtargetRepetido=true;
		else
		{
			// Si no está idéntico, recorremos la lista y vamos comparando con los elementos almacenados. 
			// Si encuantra alguno parecido al subtarget recién calculado levanta bandera y rompe el bucle.
			for(int i=0; i<listaSubtargets.size(); i++)
			{
				QVec anterior=listaSubtargets.at(i);
				
				if(fabs(subTarget[0]-anterior[0])<minTraslaciones and fabs(subTarget[1]-anterior[1])<minTraslaciones and fabs(subTarget[2]-anterior[2])<minTraslaciones and
				   fabs(subTarget[3]-anterior[3])<minRotaciones   and fabs(subTarget[4]-anterior[4])<minRotaciones   and fabs(subTarget[5]-anterior[5])<minRotaciones)
				{
					subtargetRepetido=true; 
					break;
				}
			}
		}
	}

	return subtargetRepetido;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS DE TRASLACIÓN Y ROTACIÓN									   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * @brief Metodo jacobian.
 * Crea la matriz jacobiana de la función de cinematica directa con los datos que saca del innerModel. 
 * La va rellenando por columnas, siendo las columnas los motores y las filas la tx, ty, tz, rx, ry y rz.
 * Se le pasa un vector de tantos elementos como motores con los que trabaja. Si hay cero en la posición
 * del motor calcula traslaciones y rotaciones. Si hay un 1 en la posición del motor rellena la columna a 0
 * 
 * @param motores es un vector de 0 y 1, con tantos elementos como motores con los que esté trabajando la 
 * 		  inversa. El 0 significa que el motor NO está bloqueado (calcula la columna del jacobiano asociado
 * 		  al motor) y el 1 significa que el motor SI está bloqueado, rellenado la columna del jacobiano con 0.
 * 
 * @return QMat la matriz jacobiana
 */ 
QMat Cinematica_Inversa::jacobian(QVec motores)
{
	// Inicializamos la matriz jacobiana de tamaño el del punto objetivo que tiene 6 ELEMENTOS [tx, ty, tz, rx, ry, rz]
	// por el número de motores de la lista de articulaciones: 6 filas por n columnas. También inicializamos un vector de ceros
	
	QMat jacob(6, this->listaJoints.size(), 0.f);  //6 output variables
 	QVec zero = QVec::zeros(3);
 	int j=0; //índice de columnas de la matriz: MOTORES
	
 	foreach(QString linkName, this->listaJoints)
 	{
		if(motores[j] == 0)
		{
/*			// TRASLACIONES: con respecto al último NO traslada
			QVec axisTip = this->inner->getJoint(linkName)->unitaryAxis(); //vector de ejes unitarios
			axisTip = this->inner->transform(this->listaJoints.last(), axisTip, linkName);
			QVec axisBase = this->inner->transform(this->listaJoints.last(), zero, linkName);
			QVec axis = axisBase - axisTip;
			QVec toEffector = (axisBase - this->inner->transform(this->listaJoints.last(), zero, this->endEffector) );		
			QVec res = toEffector.crossProduct(axis);

			jacob(0,j) = res.x();
			jacob(1,j) = res.y();
			jacob(2,j) = res.z();
			
			// ROTACIONES
			QVec axisTip2 = this->inner->getJoint(linkName)->unitaryAxis(); //vector de ejes unitarios en el que gira
			axisTip2 = this->inner->transform(this->listaJoints.last(), axisTip2, linkName); //vector de giro pasado al hombro.
			QVec axisBase2 = this->inner->transform(this->listaJoints.last(), zero, linkName); //motor al hombro
			QVec axis2 = axisBase2 - axisTip2; //vector desde el eje de giro en el sist. hombro, hasta la punta del eje de giro en el sist. hombro. 
			
			jacob(3,j) = axis2.x(); 
			jacob(4,j) = axis2.y();
			jacob(5,j) = axis2.z();*/	
			
			
			
			QString frameBase;
			//frameBase = "sensor_transform2";
			frameBase = this->listaJoints.last();
						
			// TRASLACIONES: con respecto al último NO traslada
			QVec axisTip = this->inner->getJoint(linkName)->unitaryAxis(); //vector de ejes unitarios
			axisTip = this->inner->transform(frameBase, axisTip, linkName);
			QVec axisBase = this->inner->transform(frameBase, zero, linkName);
			QVec axis = axisBase - axisTip;
			QVec toEffector = (axisBase - this->inner->transform(frameBase, zero, this->endEffector) );		
			QVec res = toEffector.crossProduct(axis);

			jacob(0,j) = res.x();
			jacob(1,j) = res.y();
			jacob(2,j) = res.z();
			
			// ROTACIONES
			QVec axisTip2 = this->inner->getJoint(linkName)->unitaryAxis(); //vector de ejes unitarios en el que gira
			axisTip2 = this->inner->transform(frameBase, axisTip2, linkName); //vector de giro pasado al hombro.
			QVec axisBase2 = this->inner->transform(frameBase, zero, linkName); //motor al hombro
			QVec axis2 = axisBase2 - axisTip2; //vector desde el eje de giro en el sist. hombro, hasta la punta del eje de giro en el sist. hombro. 
			
			jacob(3,j) = axis2.x(); 
			jacob(4,j) = axis2.y();
			jacob(5,j) = axis2.z();
			
			
			
			
		}
 		j++;
 	}
 	return jacob;
}

/**
 * @brief Metodo calcularVectorError.
 * Calcula el vector de error resultante de dos operaciones:
 *		1) la operacion para calcular traslaciones: puntoObjetivo-endEffector.
 * 		2) la operacion para calcular el error de rotaciones.
 * 
 * @param target
 * 
 * @return QVec vector de error.
 */ 
QVec Cinematica_Inversa::computeErrorVector(const Target &target)
{
	QVec errorTotal = QVec::zeros(6); //Vector error final
	
	if(target.getType() == Target::POSE6D or target.getType() == Target::ADVANCEAXIS)
	{
		
// 		// ORIGINAL
// 		
// 		
// 		// ---> TRASLACIONES: Al punto objetivo le restamos las coordenadas del nodo final endEffector
// 		QVec errorTraslaciones = QVec::zeros(3);
// 		
// 		QVec targetInRoot = inner->transform(listaJoints.last(), QVec::zeros(3), target.getNameInInnerModel());	
// 		QVec tip = inner->transform(this->listaJoints.last(), QVec::zeros(3), this->endEffector);			
// 		errorTraslaciones = targetInRoot - tip;
// 	
// 		//ROTATIONS
//  		QMat matriz = inner->getRotationMatrixTo(this->endEffector, target.getNameInInnerModel());
// 		
// 		QVec TARGETenMANO = matriz.extractAnglesR();
// 		QVec angulos1 = QVec::vec3(TARGETenMANO[0], TARGETenMANO[1], TARGETenMANO[2]);
// 		QVec angulos2 = QVec::vec3(TARGETenMANO[3], TARGETenMANO[4], TARGETenMANO[5]);
// 		QVec errorRotaciones;
// 		if(angulos1.norm2() < angulos2.norm2())
// 			errorRotaciones = angulos1;
// 		else
// 			errorRotaciones = angulos2;
// 
// 		// Montamos el ERROR FINAL:
// 		errorTotal.inject(errorTraslaciones,0);
// 		errorTotal.inject(errorRotaciones,3);
// 		
			
		
		//---> PRUEBAS
		
// 		QVec targetTInLastJoint = inner->transform(listaJoints.last(), QVec::zeros(3), target.getNameInInnerModel());	
// 		QVec tipTInLastJoint = inner->transform(this->listaJoints.last(), QVec::zeros(3), this->endEffector);			
// 		QVec errorTInLastJoint = targetTInLastJoint - tipTInLastJoint;
// 		
// 		// Calculamos el error de rotación: ¿Cúanto debe girar last Joint para que el tip quede orientado como el target?
// 		// 1) Calculamos la matriz de rotación que nos devuelve los ángulos que debe girar el tip para orientarse como el target:
// 		QMat matTargetInTip = inner->getRotationMatrixTo(this->endEffector, target.getNameInInnerModel()); 
// 		// 2) Calculamos la matriz de rotación que nos devuelve los ángulos que debe girar el lastJoint para orientarse como el tip:
// 		QMat matTipInLastJoint = inner->getRotationMatrixTo(listaJoints.last(), this->endEffector);
// 		// 3) Calculamos el error de rotación entre el target y el tip:
// 		QVec targetRInTip = matTargetInTip.extractAnglesR_min();	
// 		// 4) Pasamos los errores de rotación al last joint.
// 		QVec anglesRot = matTipInLastJoint * targetRInTip;
// 		
// 		QVec firstRot = matTipInLastJoint * QVec::vec3(targetRInTip[0],0,0);
// 		Rot3D matFirtRot(firstRot[0], firstRot[1], firstRot[2]);
// 		
// 		QVec secondRot = matTipInLastJoint * QVec::vec3(0,targetRInTip[1],0);
// 		Rot3D matSecondRot(secondRot[0], secondRot[1], secondRot[2]);
// 		
// 		QVec thirdRot = matTipInLastJoint * QVec::vec3(0,0,targetRInTip[2]);
// 		Rot3D matThirdRot(thirdRot[0], thirdRot[1], thirdRot[2]);
// 		
// 		QMat matResulInLastJoint =  (matFirtRot * matSecondRot) * matThirdRot;
// 		QVec errorRInLastJoint = matResulInLastJoint.extractAnglesR_min();
// 	
// 		errorTotal.inject(errorTInLastJoint,0);
// 		errorTotal.inject(errorRInLastJoint, 3);
		
		
		QString frameBase; // Frame where the errors will be referred
		//frameBase = "sensor_transform2";
		frameBase = this->listaJoints.last();
		
		QVec targetTInFrameBase = inner->transform(frameBase, QVec::zeros(3), target.getNameInInnerModel());	
		QVec tipTInFrameBase = inner->transform(frameBase, QVec::zeros(3), this->endEffector);			
		QVec errorTInFrameBase = targetTInFrameBase - tipTInFrameBase;
		
		// Calculamos el error de rotación: ¿Cúanto debe girar last Joint para que el tip quede orientado como el target?
		// 1) Calculamos la matriz de rotación que nos devuelve los ángulos que debe girar el tip para orientarse como el target:
		QMat matTargetInTip = inner->getRotationMatrixTo(this->endEffector, target.getNameInInnerModel()); 
		// 2) Calculamos la matriz de rotación que nos devuelve los ángulos que debe girar el lastJoint para orientarse como el tip:
		QMat matTipInFrameBase = inner->getRotationMatrixTo(frameBase, this->endEffector);
		// 3) Calculamos el error de rotación entre el target y el tip:
		QVec targetRInTip = matTargetInTip.extractAnglesR_min();	
		// 4) Pasamos los errores de rotación al last joint.
		QVec anglesRot = matTipInFrameBase * targetRInTip;
		
		QVec firstRot = matTipInFrameBase * QVec::vec3(targetRInTip[0],0,0);
		Rot3D matFirtRot(firstRot[0], firstRot[1], firstRot[2]);
		
		QVec secondRot = matTipInFrameBase * QVec::vec3(0,targetRInTip[1],0);
		Rot3D matSecondRot(secondRot[0], secondRot[1], secondRot[2]);
		
		QVec thirdRot = matTipInFrameBase * QVec::vec3(0,0,targetRInTip[2]);
		Rot3D matThirdRot(thirdRot[0], thirdRot[1], thirdRot[2]);
		
		QMat matResulInFrameBase =  (matFirtRot * matSecondRot) * matThirdRot;
		QVec errorRInFrameBase = matResulInFrameBase.extractAnglesR_min();
	
		errorTotal.inject(errorTInFrameBase,0);
		errorTotal.inject(errorRInFrameBase, 3);
		
		
		
		
	}
	
	if(target.getType() == Target::ALIGNAXIS)
	{
		// compute a vector going from tip to target
		QVec targetInTip = inner->transform(target.getTipName(),QVec::zeros(3),target.getNameInInnerModel()).normalize();
		
		QVec a = target.getAxis();
		//a.print("axis from target");
		QVec o = a^targetInTip; // axis to rotate
		
		float co = (a * targetInTip);
		float si = (o.norm2());  //Angle to rotate
		float ang = atan2(si,co);	
		QMat c = o.crossProductMatrix();
		QMat r = QMat::identity(3) + (c * (T)sin(ang)) + (c*c)*(T)(1.f-cos(ang));  ///Rodrigues formula to compute R from <vector-angle>
		QVec erroRotaciones = r.extractAnglesR_min();
		errorTotal.inject(erroRotaciones,3);
	}
	
	//qDebug() << __FUNCTION__ << errorTotal;
	return errorTotal;
}


/**
 * @brief Metodo levenbergMarquardt.
 * Realiza el algoritmo de Levenberg-Marquardt extendido para aquellos casos en
 * los que la matriz que ha de ser invertida salga singular. No para de ejecutar
 * hasta que se alcanza un umbral de aceptacion de la solucion marcado por el
 * usuario. Devuelve los valores de los angulos en orden que han de cambiar.
 * SI FUNCIONA
 * 			- Jt*J + nu*I*Incrementos = Jt*e
 * 				A  + nu*I*Incrementos = g*Ep
 * 
 * @param target ...
 * 
 * @return void
 */
void Cinematica_Inversa::levenbergMarquardt(Target &target)
{
	//incrementamos el número d eveces que se ejecuta el Levenberg-Marquardt. firstTime solo puede valer
	// CERO cuando entra por primera vez, uno cuando sale a primera vez y dos cuando sale por segunda vez.
	firstTime=firstTime+1; 
	
	//e1 0.0001  0.000001
	//e3 0.0004 0.000004
	const float e1 = 0.0001 , e2 = 0.00000001, e3 = 0.0004, e4 = 0.f, t = pow(10, -3);
	const int kMax = 100;
	const QMat Identidad = QMat::identity(this->listaJoints.size());
	
	// VARIABLES:
	int k=0, v=2, auxInt; //iterador, variable para descenso y un entero auxiliar
	QVec incrementos, aux; //vector de incrementos y vector auxiliar para guardar cambios
	QVec motores (this->listaJoints.size()); // lista de motores para rellenar el jacobiano.
	QVec angulos = calcularAngulos(); // ángulos iniciales de los motores.	
	
	// Creamos la matriz de pesos: Si antes hubo incrementos pequeños cuando se ejecutó por vez primera
	// ponemos TODAS las restricciones. Si no fue así, toma los pesos del usuario:
	QMat We;
	if(lowIncrementFirst and firstTime==2)
		We= QMat::identity(6);
	else
		We = QMat::makeDiagonal(target.getWeights());  //matriz de pesos para compensar milímietros con radianes.
	
	QVec error = We * computeErrorVector(target); //error de la posición actual con la deseada.
	QMat J = jacobian(motores);
	QMat H = J.transpose()*(We*J);			
	QVec g = J.transpose()*(error);		
	bool stop = (g.maxAbs(auxInt) <= e1);
	bool smallInc = false;
	bool nanInc = false;
	float ro = 0; 
	float n = t*H.getDiagonal().max(auxInt); 
			
	while((stop==false) and (k<kMax) and (smallInc == false) and (nanInc == false))
	{
		k++;
		do{
			try 
			{
				incrementos = (H + (Identidad*n)).invert() * g;   
				for(int i=0; i<incrementos.size(); i++)
					if(isnan(incrementos[i])) 						///NAN increments
					{
						nanInc = true;
						target.setStatus(Target::NAN_INCS);
						break;
					}
				if(nanInc == true) break;
			}
			catch(QString str){ qDebug()<< __FUNCTION__ << __LINE__ << "SINGULAR MATRIX EXCEPTION"; }
			
// 			if(incrementos.norm2() <= (e2*(angulos.norm2()+e2)))   ///Too small increments
			if(incrementos.norm2() <= e2)   ///Too small increments	
			{
				stop = true;
				smallInc = true; 
				//qDebug() << " LOW_INCS" << incrementos.norm2() << e2<<"n: "<<n<<" v: "<<v;
				target.setStatus(Target::LOW_INCS);
				lowIncrementFirst = true;
				break;
			}
			else
			{
				aux = angulos-incrementos; 
				calcularModuloFloat(aux, 2*M_PI); // NORMALIZAMOS

				if(outLimits(aux, motores) == true)		///COMPROBAR SI QUEDAN MOTORES LIBRES, SINO SALIR!!!!!!!!!!!
				{
					// Recalculamos el Jacobiano, el Hessiano y el vector g. El error es el mismo que antes
					// puesto que NO aplicamos los cambios (los ángulos nuevos).
					J = jacobian(motores);
					H = J.transpose()*(We*J);
					g = J.transpose()*(error);
				}
				
				//motores.set((T)0);
				actualizarAngulos(aux); // Metemos los nuevos angulos LUEGO HAY QUE DESHACER EL CAMBIO.
				//qDebug() << "Cambiar angulos";
				ro = ((error).norm2() - (We*computeErrorVector(target)).norm2()) /*/ (incrementos3*(incrementos3*n3 + g3))*/;
										
				if(ro > 0)
				{
					motores.set((T)0);
					// Estamos descendiendo correctamente --> errorAntiguo > errorNuevo. 
					stop = ((error).norm2() - (We*computeErrorVector(target)).norm2()) < e4*(error).norm2();
					angulos = aux;
					// Recalculamos con nuevos datos.
					error = We*computeErrorVector(target);						
					J = jacobian(motores);
					H = J.transpose()*(We*J);
					g = J.transpose()*(error);
		
					stop = (stop) or (g.maxAbs(auxInt)<=e1);
					n = n * std::max(1.f/3.f, (float)(1.f-pow(2*ro - 1,3)));		
					v=2;
				}
				else
				{
					actualizarAngulos(angulos); //volvemos a los ángulos viejos.
					n = n*v;
					v= 2*v;
				}
				
			}//fin else incrementos no despreciables.
		}while(ro<=0 and stop==false);
		stop = error.norm2() <= e3;
	}
	
	// Metemos información de estado al target
	if (stop == true) target.setStatus(Target::LOW_ERROR);
	else if ( k>=kMax) target.setStatus(Target::KMAX);
	else if ( smallInc == true) target.setStatus(Target::LOW_INCS);
	else if ( nanInc == true) target.setStatus(Target::NAN_INCS);
	
	target.setError(error.norm2());
	target.setErrorVector(error);
  target.setFinalAngles(angulos);
	
}


void Cinematica_Inversa::levenbergMarquardt2(Target &target)
{
// 	//qDebug()<<"\n--ALGORITMO DE LEVENBERG-MARQUARDT --\n";
// 	//e3 = 10
// 	const float e1 = 0.0001, e2 = 0.00000001, e3 = 0.0004, e4 = 0.f, t = pow(10, -3);
// 	const int kMax = 100;
// 	const QMat Identidad = QMat::identity(this->listaJoints.size());
// 	
// 	// VARIABLES:
// 	int k=0, v=2, auxInt; //iterador, variable para descenso y un entero auxiliar
// 	QVec incrementos(this->listaJoints.size()), aux; //vector de incrementos y vector auxiliar para guardar cambios
// 	Eigen::Matrix<float,7,1> nsIncs;
// 	QVec motores (this->listaJoints.size()); // lista de motores para rellenar el jacobiano.
// 	QVec angulos = calcularAngulos(); // ángulos iniciales de los motores.	
// 	QMat We = QMat::makeDiagonal(target.getWeights());  //matriz de pesos para compensar milímietros con radianes.
// 	QVec error = We * computeErrorVector(target); //error de la posición actual con la deseada.
// 	QMat J = jacobian(motores);
// 	QMat H = J.transpose()*(We*J);			
// 	QMat projector0 = Identidad;
// 	QMat projector = Identidad;
// 	
// 	QVec g = J.transpose()*(error);		
// 	bool stop = (g.maxAbs(auxInt) <= e1);
// 	bool smallInc = false;
// 	bool nanInc = false;
// 	float ro = 0; 
// 	float n = t*H.getDiagonal().max(auxInt); 
// 	
// 	while((stop==false) and (k<kMax) and (smallInc == false) and (nanInc == false))
// 	{
// 		k++;
// 		do{
// 			try
// 			{
// 				Eigen::Matrix<float,6,Eigen::Dynamic> nj = Eigen::Map<Eigen::Matrix<float,6,7, Eigen::RowMajor> >(J.getWriteData());
// 				Eigen::JacobiSVD<Eigen::Matrix<float,6,Eigen::Dynamic> >svdNS(nj, Eigen::ComputeThinU | Eigen::ComputeThinV);
// 				cout << "Its singular values are:" << endl << svdNS.singularValues() << endl;
// 				Eigen::Matrix<float,7,7> ns = Eigen::Matrix<float,7,7>::Zero();
// 				
// 				for(int i=0; i<svdNS.singularValues().size(); i++)
// 					if(svdNS.singularValues()(i) > 1E-3)
// 						ns += svdNS.matrixV().col(i) * svdNS.matrixV().col(i).transpose();
// 				Eigen::Matrix<float,7,7> proy = Eigen::Matrix<float,7,7>::Identity(7,7) - ns; 
// 				//Eigen::Matrix<float,7,1> alfa = Eigen::Matrix<float,7,1>::Constant(0.2);
// 				
// 				Eigen::Matrix<float,7,Eigen::Dynamic> alfas = Eigen::Map<Eigen::Matrix<float,7,1> >(computeH(angulos).getWriteData());
// 				cout << "proy " << proy << endl;			
// 				nsIncs = proy * alfas;
// 				computeH(angulos).print("H");
// 				cout << "NSIncs " << nsIncs << endl;
// 			}
// 			catch(QString str){ qDebug()<< __FUNCTION__ << __LINE__ << "SINGULAR MATRIX EXCEPTION IN H"; }
// 			try    //Solve NORMAL equations using two-sided Jacobi R-SVD decomposition
// 			{
// 				Eigen::Matrix<float,7,Eigen::Dynamic> md = Eigen::Map<Eigen::Matrix<float,7,7, Eigen::RowMajor> >((H+(Identidad*n)).getWriteData());
// 	
// 				Eigen::JacobiSVD<Eigen::Matrix<float,7,Eigen::Dynamic> >svd(md, Eigen::ComputeThinU | Eigen::ComputeThinV);
// 				cout << "Its singular values are:" << endl << svd.singularValues() << endl;
// 				cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
// 				cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
// 				
// 				Eigen::Matrix<float,7,Eigen::Dynamic> rhs = Eigen::Map<Eigen::Matrix<float,7,1> >(g.getWriteData());
// 		
// 				cout << "Now consider this rhs vector:" << endl << rhs << endl;
// 				cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(rhs) << endl;
// 				Eigen::Matrix<float,7,Eigen::Dynamic> suma = svd.solve(rhs) + nsIncs;
// 				Eigen::Map<Eigen::Matrix<float,7,1> >(incrementos.getWriteData(),7,1) = suma;
// 				incrementos.print("incrementos");
// // 				qFatal("fary");
// 				for(int i=0; i<incrementos.size(); i++)
// 					if(isnan(incrementos[i])) 													///NAN increments
// 					{
// 						nanInc = true;
// 						target.setStatus(Target::NAN_INCS);
// 						break;
// 					}
// 				if(nanInc == true) break;
// 			}
// 			catch(QString str){ qDebug()<< __FUNCTION__ << __LINE__ << "SINGULAR MATRIX EXCEPTION"; }
// 			
// 			if(incrementos.norm2() <= (e2*(angulos.norm2()+e2)))   ///Too small increments
// 			{
// 				stop = true;
// 				smallInc = true; 
// 				target.setStatus(Target::LOW_INCS);
//  				//qDebug()<< __FUNCTION__ << "Increments too small" << incrementos << "in iter" << k;
// 				break;
// 			}
// 			else
// 			{
// 				aux = angulos-incrementos; 
// 				calcularModuloFloat(aux, 2*M_PI); // NORMALIZAMOS
// 
// 				if(outLimits(aux, motores) == true)		///COMPROBAR SI QUEDAN MOTORES LIBRES, SINO SALIR!!!!!!!!!!!
// 				{
// 					//qDebug()<<"FUERA DE LOS LIMITES";
// 					// Recalculamos el Jacobiano, el Hessiano y el vector g. El error es el mismo que antes
// 					// puesto que NO aplicamos los cambios (los ángulos nuevos).
// 					J = jacobian(motores);
// 					H = J.transpose()*(We*J);
// 					g = J.transpose()*(error);
// 					for(int i=0;i<motores.size();i++)
// 						if(motores[i] == 1)
// 							projector0(i,i) = 0.f;
// 				}
// 				
// 				else
// 				{
// 					motores.set((T)0);
// 					actualizarAngulos(aux); // Metemos los nuevos angulos LUEGO HAY QUE DESHACER EL CAMBIO.
// 					ro = ((error).norm2() - (We*computeErrorVector(target)).norm2()) /*/ (incrementos3*(incrementos3*n3 + g3))*/;
// 					
// 					//qDebug() << __FUNCTION__ << "ro" << ro << "error anterior" << (We*error).norm2() << "error actual" << (We*calcularVectorError()).norm2();
// 					
// 					if(ro > 0)
// 					{
// 						// Estamos descendiendo correctamente --> errorAntiguo > errorNuevo. 
// 						stop = ((error).norm2() - (We*computeErrorVector(target)).norm2()) < e4*(error).norm2();
// 						//qDebug()<<"HAY MEJORA ";
// 						angulos = aux;
// 						// Recalculamos con nuevos datos.
// 						error = computeErrorVector(target);						
// 						J = jacobian(motores);
// 						H = J.transpose()*(We*J);
// 						g = J.transpose()*(error);
// 		
// 						stop = (stop) or (g.maxAbs(auxInt)<=e1);
// 						n = n * std::max(1.f/3.f, (float)(1.f-pow(2*ro - 1,3)));		
// 						v=2;
// 					}
// 					else
// 					{
// 						//qDebug() << __FUNCTION__ << __LINE__ << "NO IMPROVEMENT";
// 						actualizarAngulos(angulos); //volvemos a los ángulos viejos.
// 						n = n*v;
// 						v= 2*v;
// 					}
// 				}//fin else dentro límites
// 			}//fin else incrementos no despreciables.
// 		}while(ro<=0 and stop==false);
// 		stop = error.norm2() <= e3;
// 	}
// 	if ( stop == true) target.setStatus(Target::LOW_ERROR);
// 	else if ( k>=kMax) target.setStatus(Target::KMAX);
// 	else if ( smallInc == true) target.setStatus(Target::LOW_INCS);
// 	else if ( nanInc == true) target.setStatus(Target::NAN_INCS);
// 	target.setError((error).norm2());
// 	target.setErrorVector(error);
// 	target.setFinalAngles(angulos);
// // 	qDebug() << "---OUT-----------------------------------------------------------";
// // 	qDebug() << "Error: "<< We*error << "E norm: " << (We*error).norm2();
// // 	qDebug() << "Stop" << stop << ". Ro" << ro << ". K" << k << ". SmallInc" << smallInc << ". NanInc" << nanInc;
// // 	
// // 	angulos.print("angulos");
// // qDebug() << "-----------------------------------------------------------------";
	
}
QVec Cinematica_Inversa::computeH(const QVec &angs)
{
	QVec alfas(angs.size());
	InnerModelJoint *joint;
	for(int i=0; i< angs.size(); i++)
	{
		joint = dynamic_cast<InnerModelJoint *>(inner->getNode(listaJoints[i]));
		float mid = (joint->max - joint->min)/2.f;
		alfas[i] = 0.5*(angs[i] - mid);
	}
	return alfas;
	
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS DE CÁLCULO													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*
 * Metodo calcularAngulos
 * Devuelve en un vector todos los angulos de los motores del robot.
 * FUNCIONA
 */ 
QVec Cinematica_Inversa::calcularAngulos()
{
	QVec angulos;
	
	for(int i=0; i<this->listaJoints.size(); i++)
	{
		float angle = inner->getJoint(listaJoints[i])->getAngle();
		angulos.push_back(angle);
	}
	return angulos;
}

/*
* Metodo moduloFloat
* Devuelve el m��dulo entre dos n��meros reales.   ///HAS PROBADO FMOD?
* FUNCIONA.
*/ 
void Cinematica_Inversa::calcularModuloFloat(QVec &angles, float mod)
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


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS DE ACTUALIZACIÓN											   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*
 * Metodo actualizarAngulos.
 * Actualiza la lista de motores del brazo con los nuevos angulos
 * que recibe como parametro (un vector) de entrada.
 */ 
void Cinematica_Inversa::actualizarAngulos(QVec angulos_nuevos)
{
	for(int i=0; i<this->listaJoints.size(); i++)
	{
		this->inner->updateJointValue(this->listaJoints[i], angulos_nuevos[i]);
	}
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS DE CONSULTA													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*
 * Método dentroLimites
 * Devuelve TRUE si los ángulos que se le pasan para mover los motores están correctos y no superan
 * los límites de cada motor, o FALSE si alguno de los ángulos para algún motor ha sobrepasado el
 * límite. Si se supera el límite pone un 1 en el vector de motores, para que cuando calcule la matriz
 * jacobiana, ponga ceros en la columna del motor.
 */ 
bool Cinematica_Inversa::outLimits(QVec &angulos, QVec &motores)
{
	bool noSupera = true;
	float limiteMin, limiteMax;
	
	for(int i=0; i<listaJoints.size(); i++)
	{
		// Obtenemos los límites mínimo y máximo de cada motor y lo comparamos con el ángulo obtenido
		// por el algoritmo de Levenberg-Marquardt.
		limiteMin = inner->getJoint(listaJoints[i])->min;
		limiteMax = inner->getJoint(listaJoints[i])->max;
		
		if(angulos[i]<limiteMin or angulos[i]>limiteMax)
		{
			noSupera = false;
			motores[i] = 1;
			
			if(angulos[i]<limiteMin)
				angulos[i] = limiteMin;
			if(angulos[i]>limiteMax)
				angulos[i] = limiteMax;
			
			qDebug()<< __FUNCTION__ << "MIN: "<<limiteMin<<" MAX: "<<limiteMax<<" ANGLE: "<<angulos[i]<<" MOTORES: "<<listaJoints[i];
		}
	}
	
	return !noSupera;
}


///  CODE TO ALIGN A BODYPART WITHOUT CREATING A VIRTUAL APPEX (NOT WORKING)

// 	if(target.getType() == Target::ALIGNAXIS)
// 	{
// 		// compute a vector going from tip to target
// 		qDebug() << target.getTipName();
// 		QVec targetInTip = inner->transform(target.getTipName(),QVec::zeros(3),"target").normalize();
// 		//targetInTip.print("targetInTip");
// 		
// 		QString axis = target.getAxisName();
// 		QVec a(3);
// 		if(axis == "x" or axis == "X")
// 			a = QVec::vec3(1,0,0);
// 		else if (axis == "y" or axis == "Y")
// 			a = QVec::vec3(0,1,0);
// 		else if (axis == "z" or axis == "Z")
// 			a = QVec::vec3(0,0,1);
// 		else
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Target axis not recognized";
// 			return QVec();
// 		}
// 		QVec o = a^targetInTip; // axis to rotate
// 		float ang = asin(o.norm2());  //Angle to rotate
// 		//o.print("o");
// 		//qDebug()<< "ang " << ang;
// 		QMat c = o.crossProductMatrix();
// 		//c.print("c");
// 		QMat r = QMat::identity(3) + (c * (T)sin(ang)) + (c*c)*(T)(1.f-cos(ang));
// 		//r.print("r");
// 		QVec rotaciones = r.extractAngles(r);
// 		QVec errorRotaciones(3);
// 		//rotaciones.print("rotaciones");
// 		//rotaciones.subVector(0,2).print("sb");
// 		//rotaciones.subVector(3,5).print("sb2");
// 		if(rotaciones.subVector(0,2).norm2() < rotaciones.subVector(3,5).norm2())
// 			errorRotaciones = rotaciones.subVector(0,2);
// 		else
// 			errorRotaciones = rotaciones.subVector(3,5);
// 		//errorRotaciones.print("rotaciones");
// 	
// 		errorTotal[3] = errorRotaciones[0];
// 		errorTotal[4] = errorRotaciones[1];
// 		errorTotal[5] = errorRotaciones[2];
// 	}



// 	//AXIS ALIGN BY INSERTING A VIRTUAL APPEX
// 	if(target.getType() == Target::ALIGNAXIS)
// 	{
// 		QVec errorTraslaciones = QVec::zeros(3);
// 		QVec auxTraslaciones = QVec::vec3(target.getPose()[0], target.getPose()[1], target.getPose()[2]);		
// 		QVec targetInRoot = inner->transform(this->listaJoints[0], auxTraslaciones ,"world");
// 		QVec tip = inner->transform(this->listaJoints[0], QVec::zeros(3), "appex");
// 		errorTraslaciones = targetInRoot - tip;	
// 		errorTotal.inject(errorTraslaciones,0);
// 		
// 		//Compute rotation error at the original tip, not at the tip of the virtual appex
// 		if( target.getAxisConstraint() == true )
// 		{
// 			QMat matriz = inner->getRotationMatrixTo(listaJoints.last(), target.getNameInInnerModel());  //ESTO NO ESTA DEL TODO BIEN. La restriccón debe ser sobre el axisName directamente
// 			QVec ang = matriz.extractAnglesR3(matriz);		
// 			QString axisName = target.getAxisName();
// 			if(axisName == "x" or axisName == "X")
// 				errorTotal[3] = ang[0];
// 			if(axisName == "y" or axisName == "Y")
// 				errorTotal[4] = ang[1];
// 			if(axisName == "z" or axisName == "Z")	
// 				errorTotal[5] = ang[2];
// 		}
// 	}
// 	

//		float len = inner->transform(this->endEffector,QVec::zeros(3),target.getNameInInnerModel()).norm2();   //distance from tip to target
/*		InnerModelNode *nodeTip = inner->getNode(this->endEffector);
		InnerModelTransform *nodeAppex = inner->newTransform("appex", "static", nodeTip, 0, 0, 0, 0, 0, 0, 0);
		nodeTip->addChild(nodeAppex);
		QString axisName = target.getAxisName();
		if(axisName == "x" or axisName == "X")
			inner->updateTransformValues("appex", len, 0, 0, 0, 0, 0, this->endEffector);
		if(axisName == "y" or axisName == "Y")
			inner->updateTransformValues("appex", 0, len, 0, 0, 0, 0, this->endEffector);
		if(axisName == "z" or axisName == "Z")
			inner->updateTransformValues("appex", 0, 0, len, 0, 0, 0, this->endEffector);
	*/	
