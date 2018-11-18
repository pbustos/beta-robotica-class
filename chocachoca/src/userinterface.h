/*
 * Copyright 2018 <copyright holder> <email>
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
 */

#ifndef USERINTERFACE_H
#define USERINTERFACE_H

#include <QtWidgets>

#include "grid.h"
#include <ui_mainUI.h>

/**
 * @todo write docs
 */

class MyInterface : public QWidget, public Ui_guiDlg
{
	Q_OBJECT
	public:
		void initialize()
		{
			 setupUi(this); show(); 
			// Scene
			scene.setSceneRect(-2500, -2500, 5000, 5000);
			view.setScene(&scene);
			view.scale(1, -1);
			view.setParent(scrollArea);
			//view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
			view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
			
			robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
			noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
			noserobot->setBrush(Qt::magenta);
			view.show();
		}

		QGraphicsScene scene;
 		QGraphicsView view;
 		QGraphicsRectItem *robot;
 		QGraphicsEllipseItem *noserobot;
		
	public slots:
		void robotSetPoseSLOT(float x, float z, float alpha) 
		{ 	
				robot->setPos(x, z);
				robot->setRotation(alpha);
		};
		void showSLOT()
		{
			view.show();
		}
// 		void initGridSLOT(int tilesize, Grid::Key key, const TCell &value)
// 		{
// 			int tilesize = 70;
// 			value.rect = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));
// 			value.rect->setPos(key.x,key.z);
// 			std::cout << key << std::endl;
// 		}
};

#endif // USERINTERFACE_H
