#
# Copyright (C) 2017 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
# 	Authors:
# 		Cristian Vazquez Cordero
# 		Cristina Mendoza Gutierrez
#

import sys
import os
import traceback
import time

from PySide import QtGui, QtCore
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 300
		self.timer.start(self.Period)

	def setParams(self, params):
		self.listState = ["PICK", "RELEASE"]
		self.state = self.listState[0]
		self.listaBoxes = ["C10", "C11", "C12"]
		self.currentBox = None
		return True

	@QtCore.Slot()
	def compute(self):
		# State machine
		if (self.state.find(self.listState[0]) > -1):
			self.detectPick()
		elif (self.state.find(self.listState[1]) > -1):
			self.detectRelease()
		return True

	def detectPick(self):
		print "PICK"
		for box in self.listaBoxes:
			for car in range(1, 6):
				boxname = box + "_" + str(car)
				try:
					if self.innermodelmanager_proxy.collide("finger_right_2_mesh3", boxname):
						print "Muevo cosas "
						self.innermodelmanager_proxy.moveNode(box, "cameraHand")  # IF MOVING TO A MESH PETA
						pose = Pose3D()
						pose.x=0
						pose.y=-50
						pose.z=50
						pose.rx=0
						pose.ry=0
						pose.rz=0
						self.innermodelmanager_proxy.setPoseFromParent(box, pose)
						self.state = self.listState[1]
						self.currentBox = box
				except Ice.Exception, e:
					traceback.print_exc()
					print e
					sys.exit()

		if (self.state.find(self.listState[1]) > -1):
			time.sleep(5)

	def detectRelease(self):
		print "RELEASE"
		for car in range(1, 6):
			boxname = self.currentBox + "_" + str(car)
			try:
				if self.innermodelmanager_proxy.collide(boxname, "ddG"):
					print "Muevo cosas"
					pose = {}
					accept,pose=self.innermodelmanager_proxy.getPose("world",self.currentBox,pose)
					self.innermodelmanager_proxy.moveNode(self.currentBox, "world")  # IF MOVING TO A MESH PETA
					pose.y=5
					pose.rx=0
					pose.rz=0
					self.innermodelmanager_proxy.setPoseFromParent(self.currentBox, pose)
					self.state = self.listState[0]
			except Ice.Exception, e:
				traceback.print_exc()
				print e
				sys.exit()

		if (self.state.find(self.listState[0]) > -1):
			self.currentBox = None
			time.sleep(5)
