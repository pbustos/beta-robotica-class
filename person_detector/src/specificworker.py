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
import threading
import time

import numpy as np
from PySide6 import QtCore
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces

# the pure modules live under src/, imported as a package from the component root
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.detector import PersonDetector
from src.merge import merge
from src.publish import to_visual_objects
from src.tracker import Tracker

console = Console(highlight=False)

try:
    import setproctitle
    setproctitle.setproctitle(os.path.basename(os.getcwd()))
except Exception:
    pass


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        cfg = configData.get("Detector", {})
        self.strip_count = int(cfg.get("StripCount", 3))
        self.strip_width = int(cfg.get("StripWidth", 768))
        self.iou_threshold = float(cfg.get("IouThreshold", 0.5))
        self.track_iou_threshold = float(cfg.get("TrackIouThreshold", 0.3))
        self.track_max_age = int(cfg.get("TrackMaxAge", 5))
        self.edge_tolerance = int(cfg.get("EdgeTolerance", 2))
        self.vertical_overlap_min = float(cfg.get("VerticalOverlapMin", 0.5))
        self.publish_roi_pixels = str(cfg.get("PublishRoiPixels", "false")).lower() in ("true", "1", "yes")
        self.confidence = float(cfg.get("Confidence", 0.5))
        self.model_path = cfg.get("Model", "yolo26l-seg.pt")
        self.display_enabled = str(
            configData.get("Display", {}).get("Enabled", "true")
        ).lower() in ("true", "1", "yes")

        # one-slot frame buffer, written by the capture thread, read by compute()
        self._frame = None
        self._frame_lock = threading.Lock()
        self._running = True
        # poll a little faster than the tick so a fresh frame is always waiting, but far
        # slower than a fetch takes, so the capture thread is never the GIL's main consumer
        self._capture_interval = float(cfg.get("CapturePeriod", self.Period / 2.0)) / 1000.0

        # published snapshot, written by compute(), read by Ice servant threads
        self._snapshot = ifaces.RoboCompVisualElements.TObjects()
        self._snapshot.objects = []
        self._snapshot.timestampimage = 0
        self._snapshot.timestampgenerated = 0
        self._snapshot.period = 0.0
        self._snapshot_lock = threading.Lock()
        self._last_publish_ms = time.time() * 1000.0
        self._setvisual_warned = False

        self.detector = None
        self.tracker = None
        self.pano_width = None

        if startup_check:
            self.startup_check()
        else:
            console.log(f"Loading {self.model_path} ...")
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            console.log("model loaded")

            self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._capture_thread.start()

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        self._running = False

    def _capture_loop(self):
        """Keep the newest camera frame available to the tick, without spinning.

        Overwrites rather than queues: a tick that runs late must see the CURRENT world, not
        work through a backlog. Runs off the Qt thread so the proxy call never blocks the
        event loop.

        PACED DELIBERATELY. A fetch returns in ~4 ms but the camera only publishes at
        ~30 Hz, so an unpaced loop re-fetches the same frame ~275 times a second and spends
        that whole time holding the GIL -- which starves the Qt thread and pushed the
        measured tick from 88 ms to 110-130 ms. Sleeping to just above the source rate costs
        nothing in freshness and hands the GIL back.
        """
        while self._running:
            started = time.perf_counter()
            try:
                frame = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
                with self._frame_lock:
                    self._frame = frame
            except Exception as e:
                console.log(f"[red]camera error[/red]: {e}")
                time.sleep(0.5)
                continue
            time.sleep(max(0.0, self._capture_interval - (time.perf_counter() - started)))

    def _take_frame(self):
        with self._frame_lock:
            frame, self._frame = self._frame, None
            return frame

    @QtCore.Slot()
    def compute(self):
        frame = self._take_frame()
        if frame is None:
            return True

        image = np.frombuffer(bytes(frame.image), dtype=np.uint8)
        image = image.reshape(frame.height, frame.width, frame.depth)

        # lazily built: the panorama width is a property of the stream, not of the config
        if self.detector is None or self.pano_width != frame.width:
            self.pano_width = frame.width
            self.detector = PersonDetector(
                self.model, frame.width,
                strip_count=self.strip_count, strip_width=self.strip_width,
                conf=self.confidence, edge_tolerance=self.edge_tolerance,
            )
            self.tracker = Tracker(frame.width,
                                   iou_threshold=self.track_iou_threshold,
                                   max_age=self.track_max_age)

        dets = self.detector.detect(image)
        dets = merge(dets, frame.width, self.strip_count,
                     iou_threshold=self.iou_threshold,
                     edge_tolerance=self.edge_tolerance,
                     vertical_overlap_min=self.vertical_overlap_min)
        ids = self.tracker.update([d.box for d in dets])

        now_ms = time.time() * 1000.0
        period = now_ms - self._last_publish_ms
        self._last_publish_ms = now_ms

        snapshot = to_visual_objects(dets, ids, frame, now_ms, period,
                                     self.publish_roi_pixels)
        with self._snapshot_lock:
            self._snapshot = snapshot

        self.draw(image, dets, ids, period)
        return True

    def draw(self, image, dets, ids, period_ms):
        """Downscaled panorama with ROI boxes, track ids and a measured-rate readout."""
        if not self.display_enabled:
            return
        import cv2
        from PySide6.QtGui import QImage, QPixmap

        # copy, not a view: the frame comes from np.frombuffer and is READ-ONLY, which
        # cv2.rectangle rejects outright
        canvas = image.copy()
        h, w = canvas.shape[:2]

        # Bearing fiducials.
        #
        # THE IMAGE CENTRE IS 0 deg -- the robot's forward axis. Bearing is measured from
        # there, positive to the right, so the stitching seam at the frame edges is +/-180
        # deg (directly behind). This must stay in step with yaw_of() in chocachoca's
        # person_tracker; the two are one convention and disagreeing would send the robot
        # the wrong way.
        cx = w // 2
        cv2.line(canvas, (cx, 0), (cx, h), (255, 0, 0), 2)
        cv2.putText(canvas, "x=960  0 deg (forward)", (cx + 8, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        for sx in (1, w - 2):
            cv2.line(canvas, (sx, 0), (sx, h), (255, 128, 0), 2)
        cv2.putText(canvas, "180 deg (behind)", (8, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0), 2)

        for det, track_id in zip(dets, ids):
            # A seam-straddling box has right > width. Drawn as one rectangle it would span
            # the whole panorama backwards, so it is split into the two pieces it really is.
            spans = [(int(det.left), int(min(det.right, canvas.shape[1] - 1)))]
            if det.right > canvas.shape[1]:
                spans.append((0, int(det.right - canvas.shape[1])))
            for x1, x2 in spans:
                cv2.rectangle(canvas, (x1, int(det.top)), (x2, int(det.bot)), (0, 255, 0), 2)
            # bearing of this person under the raw convention (offset 0), which is
            # exactly the number to negate into PersonTracker.YawOffsetRad
            # signed bearing from the image centre, matching person_tracker's yaw_of()
            bearing = 360.0 * (((det.left + det.right) / 2.0 % w) - w / 2.0) / w
            cv2.putText(canvas, f"{track_id} {det.score:.2f}  {bearing:+.1f}deg",
                        (int(det.left), max(12, int(det.top) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        small = cv2.resize(canvas, (canvas.shape[1] // 2, canvas.shape[0] // 2))
        sh, sw, _ = small.shape
        qimg = QImage(small.data, sw, sh, 3 * sw, QImage.Format_RGB888)
        self.ui.label_image.setPixmap(QPixmap.fromImage(qimg.copy()))
        rate = 1000.0 / period_ms if period_ms > 0 else 0.0
        self.ui.label_stats.setText(
            f"{len(dets)} people   {rate:.1f} Hz   ({period_ms:.0f} ms)")

    def startup_check(self):
        print(f"Testing RoboCompVisualElements.TObjects from ifaces.RoboCompVisualElements")
        test = ifaces.RoboCompVisualElements.TObjects()
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getVisualObjects method from VisualElements interface
    #
    def VisualElements_getVisualObjects(self, objects):
        """Return the latest snapshot.

        `objects` is ignored -- this component is a pure producer and has nothing to filter
        against. Stated in the README because a consumer cannot infer it from the IDSL.
        """
        with self._snapshot_lock:
            return self._snapshot

    #
    # IMPLEMENTATION of setVisualObjects method from VisualElements interface
    #
    def VisualElements_setVisualObjects(self, objects):
        """No-op. Logged once so a misrouted producer is visible rather than silent."""
        if not self._setvisual_warned:
            console.log("[yellow]setVisualObjects called on a producer-only component; ignoring[/yellow]")
            self._setvisual_warned = True

    # ===================================================================
    # ===================================================================

    ######################
    # From the RoboCompCamera360RGB you can call this methods:
    # RoboCompCamera360RGB.TImage self.camera360rgb_proxy.getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

    ######################
    # From the RoboCompCamera360RGB you can use this types:
    # ifaces.RoboCompCamera360RGB.TRoi
    # ifaces.RoboCompCamera360RGB.TImage

    ######################
    # From the RoboCompVisualElements you can use this types:
    # ifaces.RoboCompVisualElements.TRoi
    # ifaces.RoboCompVisualElements.TObject
    # ifaces.RoboCompVisualElements.TObjects
