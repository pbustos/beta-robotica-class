"""Run a segmentation model over the panorama's strips and return panorama-space boxes.

The model is INJECTED rather than constructed here, so the strip/coordinate plumbing -- the
part with real bugs in it -- is testable without a GPU or a 100 MB weights file.
"""
import numpy as np

from src.merge import Detection
from src.panorama import box_strip_to_pano, extract_strip, strip_layout


class PersonDetector:
    def __init__(self, model, pano_width, strip_count=3, strip_width=768,
                 conf=0.5, edge_tolerance=2):
        self.model = model
        self.pano_width = pano_width
        self.strip_width = strip_width
        self.conf = conf
        self.edge_tolerance = edge_tolerance
        self.strips = strip_layout(pano_width, strip_count, strip_width)

    def detect(self, image):
        """One batched inference over every strip. Returns panorama-space `Detection`s."""
        crops = [extract_strip(image, s) for s in self.strips]
        results = self.model.predict(
            crops,
            imgsz=(image.shape[0], self.strip_width),
            classes=[0],
            conf=self.conf,
            verbose=False,
        )

        dets = []
        for strip, result in zip(self.strips, results):
            masks = getattr(result, "masks", None)
            polys = list(masks.xy) if masks is not None else []
            for i, box in enumerate(result.boxes):
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                score = float(box.conf[0])

                # The mask is tighter than the detector box -- it excludes the background
                # the box necessarily includes. That is the entire reason for -seg weights;
                # the mask itself is never published.
                if i < len(polys) and len(polys[i]) > 0:
                    pts = np.asarray(polys[i], dtype=float)
                    x1, y1 = float(pts[:, 0].min()), float(pts[:, 1].min())
                    x2, y2 = float(pts[:, 0].max()), float(pts[:, 1].max())

                left, top, right, bot = box_strip_to_pano(
                    (x1, y1, x2, y2), strip, self.pano_width
                )
                dets.append(Detection(
                    left=left, top=top, right=right, bot=bot,
                    score=score,
                    strip=strip.index,
                    touches_left=x1 <= self.edge_tolerance,
                    touches_right=x2 >= self.strip_width - self.edge_tolerance,
                ))
        return dets
