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

#include "generador.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
Generador::Generador()
{
}

Generador::~Generador()
{

}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS PÚBLICOS													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
QQueue< Target > Generador::generarListaTargets(InnerModel* inner, const BodyPart &bodyPart, const QVec &weigths)
{
	QQueue<Target> listaTargets;
	QVec traslaciones(3), rotaciones(3), punto(6);
	QList<QVec> listaTraslaciones;
	this->inner = inner;
	this->bodyPart = bodyPart;
	this->weights = weigths;

 	if(bodyPart.getPartName() == "RIGHTARM")
  		listaTargets = generarPuntosCamareroCentro();
//			listaTargets = generarPuntosCabeza();
 	else if(bodyPart.getPartName() == "LEFTARM")
 		listaTargets = generarPuntosCamareroCentro();
		//listaTargets = generarPuntosCabeza();
	else if(bodyPart.getPartName() == "HEAD")
		//listaTargets = generarPuntosCabeza();
		listaTargets = generarPuntosCabezaCentro();
	
	else return QQueue<Target>();
	

	return listaTargets;
}


/*
 * Método generarPuntosTargetRectaY.
 * (---> Para probar betaWorldArmMetros.xml)
 * Genera nPuntos puntos completos (con sus traslaciones y rotaciones) siguiendo
 * una recta sobre el eje Y, dejando un espacio de Nm metros entre cada punto.
 * El primer punto estará en las coordenadas de la mano del robot y con su orientacion.
 * El siguiente punto estará desplazado Nm metros del primero y con una rotación de 0.05 radianes en el eje X.
 * A partir de entonces el resto de puntos se desplazan Nm metros y rotan 0.05 rad con respecto al anterior.
 */ 
QList< QVec > Generador::generarPuntosTargetRectaY(QVec coordenadasRobot, QVec rotacionRobot, int nPuntos, float Nm)
{
	// Coordenadas X y Z constantes, cambia la Y en el mundo.
	QVec traslaciones(3), punto (6), rot2;
	QList<QVec> listaPuntos;
	int i=0;
	
	float a=0.0;
	
	while(listaPuntos.size()<nPuntos)
	{
		traslaciones[0] = coordenadasRobot[0]; 	
		traslaciones[1] = coordenadasRobot[1]+(Nm*i); 	
		traslaciones[2]=coordenadasRobot[2];
		
		rot2=QVec::uniformVector(3, -0.5, 0.5); // 3 rotaciones.
		rot2[0]= a+rotacionRobot[0]; 
		rot2[1] = rotacionRobot[1]; 
		rot2[2]=rotacionRobot[2];
		a=a+0.05;
	
		// Componemos el punto primero con las traslaciones y después con las rotaciones
		for(int h=0; h<3; h++)
		{
			punto[h] = traslaciones[h];
			punto[h+3] = rot2[h];
		}
		
		if(!listaPuntos.contains(punto))
		{
			listaPuntos.append(punto);
			i++;
		}
	}
	return listaPuntos;
}

/*
 * Método generarPuntosTargetRectaZ.
 * (---> Para probar betaWorldArmMetros.xml)
 * Genera nPuntos puntos completos (con sus traslaciones y rotaciones) siguiendo
 * una recta sobre el eje Z, dejando un espacio de Nm metros entre cada punto.
 * El primer punto estará en las coordenadas de la mano del robot y con su orientacion.
 * El siguiente punto estará desplazado Nm metros del primero y con una rotación de 0.05 radianes en el eje X.
 * A partir de entonces el resto de puntos se desplazan Nm metros y rotan 0.05 rad con respecto al anterior.
 */ 
QList< QVec > Generador::generarPuntosTargetRectaZ(QVec coordenadasRobot, QVec rotacionRobot, int nPuntos, float Nm)
{
	// Coordenadas X e y constantes, cambia la Z en el mundo.
	QVec traslaciones(3), punto (6), rot2;
	QList<QVec> listaPuntos;
	int i=0;
	
	float a=0.0;
	
	while(listaPuntos.size()<nPuntos)
	{
		traslaciones[0] = coordenadasRobot[0]; traslaciones[1] = coordenadasRobot[1]; traslaciones[2]=coordenadasRobot[2]+(Nm*i);
		
		rot2=QVec::uniformVector(3, -0.5, 0.5); // 3 rotaciones entre -0.5 y 0.5 radianes 8en el fondo no se usa)
		rot2[0]= a+rotacionRobot[0]; rot2[1] = rotacionRobot[1]; rot2[2]=rotacionRobot[2];
		a=a+0.05; //Para cambiar el sentido de la rotación.
	
		// Componemos el punto primero con las traslaciones y después con las rotaciones
		for(int h=0; h<3; h++)
		{
			punto[h] = traslaciones[h];
			punto[h+3] = rot2[h];
		}
		
		if(!listaPuntos.contains(punto))
		{
			listaPuntos.append(punto);
			i++;
		}
	}
	return listaPuntos;
}

/*
 * Metodo generarPuntosTargetEsfera.
 * Genera nPuntos puntos completos (cons us traslaciones y rotaciones) alrededor de una esfera
 * de radio "radio" metros y cuyo punto central ("centroRobot") es la posicion del endEffector.
 * Las rotaciones pueden calcularse de dos formas:
 * 		1) Aleatoriamente: genera un vector de 3 rotaciones con ángulos aleatorios entre -PI/2 y PI/2
 * 		2) Va rotando los ejes 0.05 radianes en cada iteración.
 * FUNCIONA
 */ 
QList< QVec > Generador::generarPuntosTargetEsfera(QVec centroRobot, QVec rotacionRobot, int nPuntos, float radio)
{
	QVec traslaciones, rotaciones, punto (6);
	QList<QVec> listaPuntos;
	int i = 0;
	float a = 0.0;
	
	// Mientras que no rellenemos la lista con todos los puntos que queremos calcular.
	// Por una parte calculamos las traslaciones, que serán alrededor de una esfera.
	// Por otra parte calculamos las rotaciones con ángulos aleatorios.
	while(listaPuntos.size()<nPuntos)
	{
		traslaciones = this->generarPuntoEsfera(i, centroRobot, radio); //3 traslaciones alrededor de centroRobot.
		//1) Rotaciones aleatorias:
		rotaciones = QVec::uniformVector(3, -0.5, 0.5); // 3 rotaciones, entre -PI/2 y PI/2
		//2) Rotaciones controladas:
		rotaciones[0]= a+rotacionRobot[0]; rotaciones[1] = a+rotacionRobot[1]; rotaciones[2]=a+rotacionRobot[2];
//  		rotaciones[0]= 0; rotaciones[1] = 0; rotaciones[2] = 0;
		a=a+0.05;
		
		// Componemos el punto primero con las traslaciones y después con las rotaciones
		for(int h=0; h<traslaciones.size(); h++)
		{
			punto[h] = traslaciones[h];
			punto[h+3] = rotaciones[h];
		}
		
		if(!listaPuntos.contains(punto))
		{
			listaPuntos.append(punto);
			i++;
		}
	}
	
	return listaPuntos;
}


/*
 * Metodo generarAngulosMotores.
 * Por cada motor genera un ángulo distinto de forma aleatoria.
 * MODIFICADO
 */ 
QList< QVec > Generador::generarAngulosMotores(QStringList motores, int nMuestras, InnerModel *inner)
{
	QList<QVec> listaangulos;
	QVec angulos(motores.size());
	
	// Por cada motor generamos un ángulo
	for(int i=0;i<nMuestras;i++)
	{
		int j=0;
		foreach(QString name, motores)
		{
			QVec an = QVec::uniformVector(1,inner->getJoint(name)->min, inner->getJoint(name)->max);
			angulos[j++] = an[0];
		}
		listaangulos.push_back(angulos);
	}
	
	return listaangulos;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS PRIVADOS													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/*
 * Metodo generarPunto.
 * Genera el i-esimo punto de nPuntos perteneciente a una circunferencia de radio "radio" y centro en el punto 
 * "centro".
 */ 
QVec Generador::generarPunto(int i, int nPuntos, QVec centro, int radio)
{
	// Dividimos 2PI radianes entre el número de puntos que queremos obtener:
	float aux = (2*M_PI)/nPuntos;
	// Para sacar el siguiente ángulo con que calcular la posición (X,Y),
	// multiplicamos el auxiliar anterior por el número de la iteración.
	float angle = aux*i;
	
	QVec paux = QVec::vec3();
	paux[0] = centro[0] + radio*cos(angle);
	paux[2] = centro [2]+ radio*sin(angle);
	paux[1] = centro[1];
	
	return paux;
}

/*
 * Metodo generarPunto sobre esfera 3D de radio R, extraido de Wolfram Research http://mathworld.wolfram.com/SpherePointPicking.html
 * Genera el i-esimo punto de nPuntos perteneciente a la superficie de una esfera circunferencia de radio "radio" y centro en el punto "centro"
 * FUNCIONA.

 */ 
QVec Generador::generarPuntoEsfera(int i, QVec centro, int radio)
{
	int total = 0;
	
	QVec paux = QVec::vec3();
	while(total < 1)
	{
		QVec d1 = QVec::uniformVector(1, -1, 1);
		QVec d2 = QVec::uniformVector(1, -1, 1);

		if (QVec::vec2(d1[0],d2[0]).norm2() < 1.f)
		{
			paux[0]	=	2*d1[0] - sqrt(1.f-d1[0]*d1[0] - d2[0]*d2[0]);	
			paux[1]	=	2*d2[0] - sqrt(1.f-d1[0]*d1[0] - d2[0]*d2[0]);	
			paux[2]	=	1.f -2.f*(d1[0]*d1[0] + d2[0]*d2[0]);
			paux = (paux * (T)radio) + centro;
			total++;
		}
	}			
	return paux;
}

/*
 * Método generarPuntosCamareroDiestro
 * Crea una trayectoria cuadrada para la mano derecha del robot. Datos tomados del espacio de trabajo de ursus.
 * Las longitudes están en metros
 */ 
QList<QVec> Generador::generarPuntosCamareroDiestro()
{
	QVec traslacion(3);
	QList<QVec> traslaciones;
	float xAux, yAux;
	
	// lado inferior en X 1: 
	for(float i=-0.10; i<0.3; i=i+0.01)
	{
		traslacion[0] = i; traslacion[1] = 0.9; traslacion[2] = 0.400;
		traslaciones.append(traslacion);
		xAux = i;
	}
	
	// Y 1:
	for(float j=0.9; j<1.10; j=j+0.01)
	{
		traslacion[0] = xAux; traslacion[1] = j; traslacion[2] = 0.400;
		traslaciones.append(traslacion);
		yAux = j;
	}
	
	// X 2:
	for(float i=xAux; i>=-0.10; i=i-0.01)
	{
		traslacion[0] = i; traslacion[1] = yAux; traslacion[2] = 0.400;
		traslaciones.append(traslacion);
		xAux = i;
	}
	// Y 2:
	for(float j=yAux; j>=0.9; j=j-0.01)
	{
		traslacion[0] = xAux; traslacion[1] = j; traslacion[2] = 0.400;
		traslaciones.append(traslacion);
		yAux = j;
	}

	
	return traslaciones;
}

/*
 * Método generarPuntosCamareroZurdo
 * Crea una trayectoria cuadrada para el brazo izquierdo del robot. Datos tomados del espacio de trabajo de ursus.
 * Las longitudes están en metros.
 */ 
QList<QVec> Generador::generarPuntosCamareroZurdo()
{
	QVec traslacion(3);
	QList<QVec> traslaciones;
	float xAux, yAux;
	
	// lado inferior en X 1: 
	for(float i=-0.40; i>=-0.15; i=i+0.01)
	{
		traslacion[0] = i; traslacion[1] = 0.9; traslacion[2] = 0.350;
		traslaciones.append(traslacion);
		xAux = i;
	}
	
	// Y 1:
	for(float j=0.9; j<1.10; j=j+0.01)
	{
		traslacion[0] = xAux; traslacion[1] = j; traslacion[2] = 0.350;
		traslaciones.append(traslacion);
		yAux = j;
	}
	
	// X 2:
	for(float i=xAux; i>=-0.40; i=i-0.01)
	{
		traslacion[0] = i; traslacion[1] = yAux; traslacion[2] = 0.35;
		traslaciones.append(traslacion);
		xAux = i;
	}
	// Y 2:
	for(float j=yAux; j>=0.9; j=j-0.01)
	{
		traslacion[0] = xAux; traslacion[1] = j; traslacion[2] = 0.35;
		traslaciones.append(traslacion);
		yAux = j;
	}
	
	return traslaciones;
}

/*
 * Método generarPuntosCamareroCentro.
 *Crea una trayectoria para los dos brazos del robot. Los coloca aproximadamente por el pecho.
 *Las longitudes están en metros. MADE IN PABLO. 
 */ 
QQueue< Target > Generador::generarPuntosCamareroCentro()
{
	QQueue<Target> targets;
	QVec weights(6);
	QVec pose = QVec::zeros(6);
	weights.set((T)1);
	float xAux, yAux;
	
	// lado inferior en X 1: 
	for(float i=-0.15; i<=0.15; i=i+0.01)
	{
		pose[0] = i; pose[1] = 0.9; pose[2] = 0.350;
		Target target( inner, pose, this->bodyPart.getTip(), weights, Target::POSE6D);
		targets.append(target);
		xAux = i;
	}
	
	// Y 1:
	for(float j=0.9; j<1.10; j=j+0.01)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 0.350;
		Target target( inner, pose, this->bodyPart.getTip(), weights, Target::POSE6D);
		targets.append(target);
		yAux = j;
	}
	
	// X 2:
	for(float i=xAux; i>=-0.15; i=i-0.01)
	{
		pose[0] = i; pose[1] = yAux; pose[2] = 0.35;
		Target target( inner, pose, this->bodyPart.getTip(), weights, Target::POSE6D);
		targets.append(target);
		xAux = i;
	}
	// Y 2:
	for(float j=yAux; j>=0.9; j=j-0.01)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 0.35;
		Target target( inner, pose, this->bodyPart.getTip(), weights, Target::POSE6D);
		targets.append(target);
		yAux = j;
	}
	return targets;
}

QQueue< Target > Generador::generarPuntosCabeza()
{
	float xAux, yAux;
	Target target;
	QVec pose(6);
	QQueue<Target> targets;
	QVec weights = QVec::zeros(6);
	weights[0] = 1.f;
	weights[1] = 1.f;
	weights[2] = 1.f;
	
	// lado inferior en X 1: 
	for(float i=-0.5; i<=0.5; i=i+0.01)
	{
		pose[0] = i; pose[1] = 0.9; pose[2] = 0.350;
		pose[3] = 0; pose[4] = 0; pose[5] = 0;
		Target target( inner, pose, this->bodyPart.getTip(), weights, Target::ALIGNAXIS, "z");
		targets.append(target);
		xAux = i;
	}
	
	// Y 1:
	for(float j=0.9; j<1.10; j=j+0.01)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 0.350;
		pose[3] = 0; pose[4] = 0; pose[5] = 0;
		Target target(inner, pose, this->bodyPart.getTip(), weights, Target::ALIGNAXIS, "z");
		targets.append(target);
		yAux = j;
	}
	
	// X 2:
	for(float i=xAux; i>=-0.15; i=i-0.01)
	{
		pose[0] = i; pose[1] = yAux; pose[2] = 0.35;
  	pose[3] = 0; pose[4] = 0; pose[5] = 0;
		Target target(inner, pose, this->bodyPart.getTip(), weights, Target::ALIGNAXIS, "z");
		targets.append(target);
		xAux = i;
	}
	// Y 2:
	for(float j=yAux; j>=0.9; j=j-0.01)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 0.35;
		pose[3] = 0; pose[4] = 0; pose[5] = 0;
		Target target(inner, pose, this->bodyPart.getTip(), weights, Target::ALIGNAXIS, "z");
		targets.append(target);

		yAux = j;
	}
	return targets;

}

QQueue< Target > Generador::generarPuntosCabezaCentro()
{
	float xAux, yAux;
	Target target;
	QVec pose(6);
	QQueue<Target> targets;
	QVec weights = QVec::zeros(6);
	weights[0] = 1.f;
	weights[1] = 1.f;
	weights[2] = 1.f;
	
	// lado inferior en X 1: 
	for(float i=-0.15; i<=0.15; i=i+0.01)
	{
		pose[0] = i; pose[1] = 0.9; pose[2] = 0.350;
		pose[3] = 0; pose[4] = 0; pose[5] = 0;
		Target target( inner, pose, this->bodyPart.getTip(), weights, Target::ALIGNAXIS, "z");
		targets.append(target);
		xAux = i;
	}
	
	// Y 1:
	for(float j=0.9; j<1.10; j=j+0.01)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 0.350;
		pose[3] = 0; pose[4] = 0; pose[5] = 0;
		Target target(inner, pose, this->bodyPart.getTip(), weights, Target::ALIGNAXIS, "z");
		targets.append(target);
		yAux = j;
	}
	
	// X 2:
	for(float i=xAux; i>=-0.15; i=i-0.01)
	{
		pose[0] = i; pose[1] = yAux; pose[2] = 0.35;
  	pose[3] = 0; pose[4] = 0; pose[5] = 0;
		Target target(inner, pose, this->bodyPart.getTip(), weights, Target::ALIGNAXIS, "z");
		targets.append(target);
		xAux = i;
	}
	// Y 2:
	for(float j=yAux; j>=0.9; j=j-0.01)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 0.35;
		pose[3] = 0; pose[4] = 0; pose[5] = 0;
		Target target(inner, pose, this->bodyPart.getTip(), weights, Target::ALIGNAXIS, "z");
		targets.append(target);

		yAux = j;
	}
	return targets;

}


