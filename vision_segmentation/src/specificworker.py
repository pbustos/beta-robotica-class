#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by YOUR NAME HERE
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

import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"

from PySide6.QtCore import QTimer, QByteArray
from PySide6.QtGui import QVector3D, QColor, QImage, QPixmap
from PySide6.QtWidgets import QApplication, QHBoxLayout, QWidget, QLabel
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DRender import Qt3DRender
from PySide6.Qt3DExtras import Qt3DExtras
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import cv2
import numpy as np
import os
import time
from ultralytics import YOLO

# Workaround for OpenCV Qt font warning
cv2_qt_font_dir = os.path.join(os.path.dirname(cv2.__file__), 'qt', 'fonts')
os.makedirs(cv2_qt_font_dir, exist_ok=True)

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


class WebotsStyleCameraController(Qt3DExtras.QAbstractCameraController):
    def __init__(self, parent=None):
        super().__init__(parent)

    def moveCamera(self, state, dt):
        cam = self.camera()
        if not cam:
            return

        # ---- Orbit: left button drag ------------------------------------------
        if state.leftMouseButtonActive and not state.rightMouseButtonActive:
            cam.panAboutViewCenter(state.rxAxisValue * self.lookSpeed() * dt)
            cam.tiltAboutViewCenter(-state.ryAxisValue * self.lookSpeed() * dt)

        # ---- Pan: right button drag -------------------------------------------
        if state.rightMouseButtonActive:
            cam.translate(QVector3D(-state.rxAxisValue * self.linearSpeed() * dt,
                                    -state.ryAxisValue * self.linearSpeed() * dt,
                                    0.0),
                          Qt3DRender.QCamera.TranslateViewCenter)

        # ---- Zoom: scroll wheel -----------------------------------------------
        if abs(state.tzAxisValue) > 0.0:
            cam.translate(QVector3D(0.0, 0.0, state.tzAxisValue * self.linearSpeed() * dt),
                          Qt3DRender.QCamera.DontTranslateViewCenter)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.Display = configData["Display"] if "Display" in configData else "True"
        self.display_enabled = str(self.Display).lower() in ["true", "1", "yes", "on"]

        # Load the latest YOLO model (YOLO26 large) for object detection
        print("Loading YOLO model...")
        self.yolo_model = YOLO('yolo26l-seg.pt')
        print("YOLO loaded.")

        # FPS tracking variables
        self.fps_frames = 0
        self.fps_last_time = time.time()
        
        # State variables for serving to clients
        self.current_segmented_objects = []
        self.current_image_out = None

        # Display toggle from config
        if self.display_enabled:
            self.setup_qt3d()
        
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

   

    @QtCore.Slot()
    def compute(self):
        # FPS Counter logic
        self.fps_frames += 1
        current_time = time.time()
        if (current_time - self.fps_last_time) >= 3.0:
            fps = self.fps_frames / (current_time - self.fps_last_time)
            print(f"Current FPS: {fps:.2f}")
            self.fps_frames = 0
            self.fps_last_time = current_time

        try:
            # Read proxy data
            rgbd = self.camerargbdsimple_proxy.getAll("camera")
            img_struct = rgbd.image
            depth_struct = rgbd.depth
            #print(f"Received RGBD data: Image({img_struct.width}x{img_struct.height}), Depth({depth_struct.width}x{depth_struct.height})")
            
            # Convert to OpenCV image (already in RGB from ICE)
            img = np.frombuffer(img_struct.image, dtype=np.uint8)
            img = img.reshape(img_struct.height, img_struct.width, 3)

            # --- YOLO Prediction ---
            results = self.yolo_model(img, verbose=False)
            
            # Annotated image for the 2D view (RGB format)
            annotated_img = img.copy()
            
            # Create a segmentation mask for all objects
            seg_mask = np.zeros((img_struct.height, img_struct.width), dtype=np.uint8)

            # Extract segmentation masks (contours) if available
            if hasattr(results[0], 'masks') and results[0].masks is not None:
                contours = results[0].masks.xy
                classes = results[0].boxes.cls.cpu().numpy()
                confs = results[0].boxes.conf.cpu().numpy()
                
                for contour, cls, conf in zip(contours, classes, confs):
                    if len(contour) == 0:
                        continue
                    points = np.int32([contour])
                    # Draw contour (green)
                    cv2.polylines(annotated_img, points, isClosed=True, color=(0, 255, 0), thickness=2)
                    
                    # Fill polygon on the seg_mask
                    cv2.fillPoly(seg_mask, points, 1)
                    
                    # Draw label (red) near the first point of the contour
                    label = f"{self.yolo_model.names[int(cls)]} {conf:.2f}"
                    cv2.putText(annotated_img, label, tuple(points[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                annotated_img = results[0].plot()

            # Get valid depth points that correspond to segmented objects
            v, u = np.where(seg_mask > 0)
            
            if len(u) > 0:
                # Assuming depth is 32-bit float in standard RoboComp structure
                depth_np = np.frombuffer(depth_struct.depth, dtype=np.float32).reshape((depth_struct.height, depth_struct.width))
                Z = depth_np[v, u]
                
                W, H = depth_struct.width, depth_struct.height
                fx = depth_struct.focalx if depth_struct.focalx > 0 else 585.756
                fy = depth_struct.focaly if depth_struct.focaly > 0 else 585.756
                cx, cy = W / 2.0, H / 2.0
                
                # Filter out zeroes or nans
                valid = (Z > 0) & np.isfinite(Z) & ~np.isnan(Z)
                Z_val = Z[valid]
                u_val = u[valid]
                v_val = v[valid]
                
                if len(Z_val) > 0:
                    # Inverse projection from 2D (u,v) to 3D (X,Y,Z) based on Webots coordinate frames (+X right, +Y up, +Z into screen)
                    X = (u_val - cx) * Z_val / fx
                    Y = (cy - v_val) * Z_val / fy
                    if self.display_enabled:
                        # Stack to Nx3 shape
                        pts_array = np.column_stack((X, Y, Z_val)).astype(np.float32)
                        
                        # 1. Position buffer
                        ba_pos = QByteArray(pts_array.tobytes())
                        self.vertex_buffer.setData(ba_pos)
                        self.pos_attr.setCount(len(Z_val))
                        
                        # 2. Color buffer mapping
                        point_colors = img[v_val, u_val] / 255.0
                        flattened_colors = point_colors.astype(np.float32)
                        
                        ba_color = QByteArray(flattened_colors.tobytes())
                        self.color_buffer.setData(ba_color)
                        self.color_attr.setCount(len(Z_val))
                else:
                    if self.display_enabled:
                        self.pos_attr.setCount(0)
                        self.color_attr.setCount(0)
            else:
                # Clear points if none selected
                if self.display_enabled:
                    self.pos_attr.setCount(0)
                    self.color_attr.setCount(0)
            # Show RGB image in QLabel (left panel)
            if self.display_enabled:
                h, w, ch = img.shape
                bytes_per_line = ch * w
                q_img = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.image_label.setPixmap(QPixmap.fromImage(q_img))
            
        except Exception as e:
            print(f"Error reading image: {e}")

    ###############################################################################

    def setup_qt3d(self):
        # Set up a layout if the UI didn't create one
        if not self.layout():
            self.setLayout(QHBoxLayout())
            
        # 1. 2D Image Label
        self.image_label = QLabel()
        self.image_label.setMinimumSize(400, 300)
        self.layout().addWidget(self.image_label)
            
        # 2. Qt3D Window Setup
        self.view_3d = Qt3DExtras.Qt3DWindow()
        self.view_3d.defaultFrameGraph().setClearColor(QColor("black"))
        self.container = QWidget.createWindowContainer(self.view_3d, self)
        self.container.setMinimumSize(400, 300)
        self.layout().addWidget(self.container)
        
        self.root_entity = Qt3DCore.QEntity()
        self.view_3d.setRootEntity(self.root_entity)

        # Camera
        self.camera = self.view_3d.camera()
        self.camera.lens().setPerspectiveProjection(45.0, 16.0 / 9.0, 0.1, 1000.0)
        
        # Position camera at origin (0,0,0) matching the physical RGBD sensor
        # Look forward towards positive Z axis to match the depth direction usually used in robotics
        self.camera.setPosition(QVector3D(0, 0, 0))
        self.camera.setViewCenter(QVector3D(0, 0, 5.0))
        
        # In webots pointcloud, Y axis commonly points Upwards (ROS/Webots standard).
        # We need the camera Up vector to point +Y.
        self.camera.setUpVector(QVector3D(0, 1, 0))

        # Camera controls
        self.cam_controller = WebotsStyleCameraController(self.root_entity)
        self.cam_controller.setCamera(self.camera)

        # Point cloud setup
        self.pc_entity = Qt3DCore.QEntity(self.root_entity)
        self.geom_renderer = Qt3DRender.QGeometryRenderer(self.pc_entity)
        self.geometry = Qt3DCore.QGeometry(self.geom_renderer)
        
        # Position buffer
        self.vertex_buffer = Qt3DCore.QBuffer(self.geometry)
        self.pos_attr = Qt3DCore.QAttribute()
        self.pos_attr.setName(Qt3DCore.QAttribute.defaultPositionAttributeName())
        self.pos_attr.setAttributeType(Qt3DCore.QAttribute.VertexAttribute)
        self.pos_attr.setBuffer(self.vertex_buffer)
        self.pos_attr.setVertexBaseType(Qt3DCore.QAttribute.Float)
        self.pos_attr.setVertexSize(3)
        self.pos_attr.setByteOffset(0)
        self.pos_attr.setByteStride(12)
        
        # Color buffer
        self.color_buffer = Qt3DCore.QBuffer(self.geometry)
        self.color_attr = Qt3DCore.QAttribute()
        self.color_attr.setName(Qt3DCore.QAttribute.defaultColorAttributeName())
        self.color_attr.setAttributeType(Qt3DCore.QAttribute.VertexAttribute)
        self.color_attr.setBuffer(self.color_buffer)
        self.color_attr.setVertexBaseType(Qt3DCore.QAttribute.Float)
        self.color_attr.setVertexSize(3)
        self.color_attr.setByteOffset(0)
        self.color_attr.setByteStride(12)
        
        self.geometry.addAttribute(self.pos_attr)
        self.geometry.addAttribute(self.color_attr)
        self.geom_renderer.setGeometry(self.geometry)
        self.geom_renderer.setPrimitiveType(Qt3DRender.QGeometryRenderer.PrimitiveType.Points)

        self.material = Qt3DExtras.QPerVertexColorMaterial(self.root_entity)

        self.pc_entity.addComponent(self.geom_renderer)
        self.pc_entity.addComponent(self.material)


    ###################################################################################33333
    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.Point3D from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.Point3D()
        print(f"Testing RoboCompCameraRGBDSimple.TPoints from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TPoints()
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        QTimer.singleShot(200, QApplication.instance().quit)

    ################################################################################

    # IMPLEMENTATION of getSegmentedObjects method from ImageSegmentation interface
    #
    def ImageSegmentation_getSegmentedObjects(self):
        # We return the currently cached segmented list
        return self.current_segmented_objects
    
    # IMPLEMENTATION of getAll method from ImageSegmentation interface
    def ImageSegmentation_getAll(self):

        ret = ifaces.RoboCompImageSegmentation.TData()
        ret.segmentedObjects = self.current_segmented_objects
        ret.image = self.current_image_out if self.current_image_out is not None else ifaces.RoboCompImageSegmentation.TImage()
        ret.timestamp = int(time.time() * 1000)  # Current time in milliseconds
        return ret
    
    #
    
    # IMPLEMENTATION of getImage method from ImageSegmentation interface
    #
    def ImageSegmentation_getImage(self):

        ret = ifaces.RoboCompImageSegmentation.TImage()
        ret.image = self.current_image_out if self.current_image_out is not None else ifaces.RoboCompImageSegmentation.TImage()
        return ret
    

    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # RoboCompCameraRGBDSimple.TRGBD self.camerargbdsimple_proxy.getAll(str camera)
    # RoboCompCameraRGBDSimple.TDepth self.camerargbdsimple_proxy.getDepth(str camera)
    # RoboCompCameraRGBDSimple.TImage self.camerargbdsimple_proxy.getImage(str camera)
    # RoboCompCameraRGBDSimple.TPoints self.camerargbdsimple_proxy.getPoints(str camera)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # ifaces.RoboCompCameraRGBDSimple.Point3D
    # ifaces.RoboCompCameraRGBDSimple.TPoints
    # ifaces.RoboCompCameraRGBDSimple.TImage
    # ifaces.RoboCompCameraRGBDSimple.TDepth
    # ifaces.RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompImageSegmentation you can use this types:
    # ifaces.RoboCompImageSegmentation.Point2D
    # ifaces.RoboCompImageSegmentation.Point3D
    # ifaces.RoboCompImageSegmentation.SegmentedObject
