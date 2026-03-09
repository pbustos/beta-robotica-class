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
import sys
os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"

from PySide6 import QtCore
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
import queue
import threading
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
        self.use_proxy_thread = str(configData.get("ProxyThread", "False")).lower() in ["true", "1", "yes", "on"]
        self.proxy_poll_period = max(float(self.Period) / 1000.0, 0.005)

        # Load the latest YOLO model (YOLO26 large) for object detection
        print("Loading YOLO model...")
        self.yolo_model = YOLO('yolo26m-seg.pt')
        print("YOLO loaded.")

        # FPS tracking variables
        self.fps_frames = 0
        self.fps_last_time = time.time()
        
        # State variables for serving to clients
        self.current_segmented_objects = []
        self.current_image_out = ifaces.RoboCompImageSegmentation.TImage()
        self.last_image_request_time = 0.0
        self.last_objects_request_time = 0.0
        self.image_request_hold_seconds = 1.0
        self.objects_request_hold_seconds = 1.0

        # Latest published snapshot for ICE consumers
        initial_timestamp = int(time.time() * 1000)
        self._latest_snapshot = (
            self.current_segmented_objects,
            self.current_image_out,
            initial_timestamp,
        )

        # Background proxy reader queue (decouples network latency from compute loop)
        self._proxy_queue = queue.Queue(maxsize=2)
        self._proxy_last_rgbd = None
        self._proxy_thread_stop = threading.Event()
        self._proxy_thread = None

        # Display toggle from config
        if self.display_enabled:
            self.setup_qt3d()
        else:
            self.hide()
        
        if startup_check:
            self.startup_check()
        else:
            if self.use_proxy_thread:
                self._start_proxy_thread()
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""
        self._stop_proxy_thread()

    @QtCore.Slot()
    def compute(self):
        self._update_fps()

        try:
            if self.use_proxy_thread:
                rgbd = self._get_latest_rgbd()
                if rgbd is None:
                    rgbd = self.camerargbdsimple_proxy.getAll("camera")
            else:
                rgbd = self.camerargbdsimple_proxy.getAll("camera")
            img_struct = rgbd.image
            depth_struct = rgbd.depth

            img = np.frombuffer(img_struct.image, dtype=np.uint8)
            img = img.reshape(img_struct.height, img_struct.width, 3)

            results = self.yolo_model(img, verbose=False)
            
            annotated_img = img.copy()

            depth_np, fx, fy, cx, cy = self._get_depth_intrinsics(depth_struct)

            annotated_img, seg_mask, segmented_objects = self._process_segmentation(
                results,
                annotated_img,
                depth_np,
                fx,
                fy,
                cx,
                cy,
                img_struct.height,
                img_struct.width,
            )

            self._update_qt3d_from_mask(seg_mask, depth_np, fx, fy, cx, cy, img)

            image_out = self._build_output_image(img_struct, annotated_img.tobytes())

            self._publish_frame(segmented_objects, image_out, int(time.time() * 1000))

            self._update_2d_display(annotated_img)
            
        except Exception as e:
            print(f"Error reading image: {e}")

    def _start_proxy_thread(self):
        if self._proxy_thread is not None:
            return

        self._proxy_thread = threading.Thread(target=self._proxy_loop, daemon=True)
        self._proxy_thread.start()

    def _stop_proxy_thread(self):
        self._proxy_thread_stop.set()
        if self._proxy_thread is not None and self._proxy_thread.is_alive():
            self._proxy_thread.join(timeout=0.5)

    def _proxy_loop(self):
        while not self._proxy_thread_stop.is_set():
            try:
                rgbd = self.camerargbdsimple_proxy.getAll("camera")
                if self._proxy_queue.full():
                    try:
                        self._proxy_queue.get_nowait()
                    except queue.Empty:
                        pass
                self._proxy_queue.put_nowait(rgbd)
                time.sleep(self.proxy_poll_period)
            except Exception as e:
                print(f"Proxy thread error: {e}")
                time.sleep(0.01)

    def _get_latest_rgbd(self):
        latest = None
        try:
            while True:
                latest = self._proxy_queue.get_nowait()
        except queue.Empty:
            pass

        if latest is not None:
            self._proxy_last_rgbd = latest

        return self._proxy_last_rgbd

    ###############################################################################

    def _update_fps(self):
        self.fps_frames += 1
        current_time = time.time()
        if (current_time - self.fps_last_time) >= 3.0:
            fps = self.fps_frames / (current_time - self.fps_last_time)
            print(f"Current FPS: {fps:.2f}")
            self.fps_frames = 0
            self.fps_last_time = current_time

    def _build_output_image(self, img_struct, image_bytes):
        img_out = ifaces.RoboCompImageSegmentation.TImage()
        img_out.compressed = img_struct.compressed
        img_out.cameraID = img_struct.cameraID
        img_out.width = img_struct.width
        img_out.height = img_struct.height
        img_out.depth = img_struct.depth
        img_out.focalx = img_struct.focalx
        img_out.focaly = img_struct.focaly
        img_out.alivetime = img_struct.alivetime
        img_out.period = img_struct.period
        img_out.image = image_bytes
        return img_out

    def _is_image_output_requested(self):
        return (time.time() - self.last_image_request_time) <= self.image_request_hold_seconds

    def _is_objects_output_requested(self):
        return (time.time() - self.last_objects_request_time) <= self.objects_request_hold_seconds

    def _publish_frame(self, segmented_objects, image_out, timestamp_ms):
        self._latest_snapshot = (segmented_objects, image_out, timestamp_ms)
        self.current_segmented_objects = segmented_objects
        self.current_image_out = image_out

    def _read_front_buffer(self):
        return self._latest_snapshot

    def _get_depth_intrinsics(self, depth_struct):
        depth_np = np.frombuffer(depth_struct.depth, dtype=np.float32).reshape(
            (depth_struct.height, depth_struct.width)
        )
        fx = depth_struct.focalx if depth_struct.focalx > 0 else 585.756
        fy = depth_struct.focaly if depth_struct.focaly > 0 else 585.756
        cx = depth_struct.width / 2.0
        cy = depth_struct.height / 2.0
        return depth_np, fx, fy, cx, cy

    def _project_depth_points(self, u, v, depth_np, fx, fy, cx, cy):
        if len(u) == 0:
            return None, None, None, None, None

        z = depth_np[v, u]
        valid = (z > 0) & np.isfinite(z) & ~np.isnan(z)
        if not np.any(valid):
            return None, None, None, None, None

        z_val = z[valid]
        u_val = u[valid]
        v_val = v[valid]
        x_val = (u_val - cx) * z_val / fx
        y_val = (cy - v_val) * z_val / fy
        return x_val, y_val, z_val, u_val, v_val

    def _build_segmented_object(self, contour_i32, label_name, conf, depth_np, fx, fy, cx, cy):
        seg_obj = ifaces.RoboCompImageSegmentation.SegmentedObject()
        seg_obj.label = label_name
        seg_obj.score = float(conf)

        contour_points = contour_i32.reshape(-1, 2)
        seg_obj.imagePolygon = [
            ifaces.RoboCompImageSegmentation.Point2D(x=int(p[0]), y=int(p[1]))
            for p in contour_points
        ]

        x0, y0, width, height = cv2.boundingRect(contour_i32)
        if width <= 0 or height <= 0:
            seg_obj.points3D = []
            return seg_obj

        local_mask = np.zeros((height, width), dtype=np.uint8)
        shifted = (contour_points - np.array([x0, y0], dtype=np.int32)).reshape(-1, 1, 2)
        cv2.fillPoly(local_mask, [shifted], 1)

        local_v, local_u = np.where(local_mask > 0)
        obj_u = local_u + x0
        obj_v = local_v + y0

        x_val, y_val, z_val, _, _ = self._project_depth_points(obj_u, obj_v, depth_np, fx, fy, cx, cy)
        if z_val is None:
            seg_obj.points3D = []
            return seg_obj

        seg_obj.points3D = [
            ifaces.RoboCompImageSegmentation.Point3D(x=float(x), y=float(y), z=float(z))
            for x, y, z in zip(x_val, y_val, z_val)
        ]
        return seg_obj

    def _process_segmentation(self, results, annotated_img, depth_np, fx, fy, cx, cy, img_height, img_width):
        seg_mask = np.zeros((img_height, img_width), dtype=np.uint8)
        segmented_objects = []
        draw_overlay = self.display_enabled or self._is_image_output_requested()
        need_objects = self._is_objects_output_requested()
        need_seg_mask = self.display_enabled

        if not (hasattr(results[0], 'masks') and results[0].masks is not None):
            return annotated_img, seg_mask, segmented_objects

        contours = results[0].masks.xy
        classes = results[0].boxes.cls.cpu().numpy()
        confs = results[0].boxes.conf.cpu().numpy()
        names = self.yolo_model.names

        for contour, cls, conf in zip(contours, classes, confs):
            contour_i32 = np.asarray(contour, dtype=np.int32)
            if contour_i32.size == 0:
                continue

            contour_poly = contour_i32.reshape(-1, 1, 2)
            if draw_overlay:
                cv2.polylines(annotated_img, [contour_poly], isClosed=True, color=(0, 255, 0), thickness=2)
            if need_seg_mask:
                cv2.fillPoly(seg_mask, [contour_poly], 1)

            class_id = int(cls)
            label_name = names[class_id] if isinstance(names, (list, tuple)) else names.get(class_id, str(class_id))
            if draw_overlay:
                cv2.putText(
                    annotated_img,
                    f"{label_name} {float(conf):.2f}",
                    tuple(contour_i32[0]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )

            if need_objects:
                segmented_objects.append(
                    self._build_segmented_object(contour_poly, label_name, conf, depth_np, fx, fy, cx, cy)
                )

        return annotated_img, seg_mask, segmented_objects

    def _update_qt3d_from_mask(self, seg_mask, depth_np, fx, fy, cx, cy, img):
        if not self.display_enabled:
            return

        v, u = np.where(seg_mask > 0)
        x_val, y_val, z_val, u_val, v_val = self._project_depth_points(u, v, depth_np, fx, fy, cx, cy)

        if z_val is None:
            self.pos_attr.setCount(0)
            self.color_attr.setCount(0)
            return

        pts_array = np.column_stack((x_val, y_val, z_val)).astype(np.float32)
        self.vertex_buffer.setData(QByteArray(pts_array.tobytes()))
        self.pos_attr.setCount(len(z_val))

        point_colors = (img[v_val, u_val] / 255.0).astype(np.float32)
        self.color_buffer.setData(QByteArray(point_colors.tobytes()))
        self.color_attr.setCount(len(z_val))

    def _update_2d_display(self, annotated_img):
        if not self.display_enabled:
            return

        h, w, ch = annotated_img.shape
        bytes_per_line = ch * w
        q_img = QImage(annotated_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(q_img))


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
        self.last_objects_request_time = time.time()
        objects, _, _ = self._read_front_buffer()
        return objects
    
    # IMPLEMENTATION of getAll method from ImageSegmentation interface
    def ImageSegmentation_getAll(self):
        self.last_image_request_time = time.time()
        self.last_objects_request_time = time.time()
        objects, image, timestamp = self._read_front_buffer()

        ret = ifaces.RoboCompImageSegmentation.TData()
        ret.objects = objects
        ret.image = image
        ret.timestamp = timestamp
        return ret
    
    #
    
    # IMPLEMENTATION of getImage method from ImageSegmentation interface
    #
    def ImageSegmentation_getImage(self):
        self.last_image_request_time = time.time()
        _, image, _ = self._read_front_buffer()
        return image
    

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
