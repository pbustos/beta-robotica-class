"""Persistent ids for detections, by predicted-position association across frames.

Deliberately NOT a re-identification tracker: a person who is occluded for longer than
`max_age` frames comes back with a new id. Appearance matching is a much larger problem
and this component reports ROIs, not identities.

★ WHY THIS IS NOT IoU-BASED, WHICH IS THE OBVIOUS CHOICE.
  The camera is on a robot that TURNS, and its own rotation dominates the apparent motion
  of everything it sees. Measured on the rig: turning at ~1 rad/s moves a person ~45 px per
  tick, while a person a few metres away is only ~39 px wide. Consecutive boxes then do not
  overlap AT ALL -- IoU is exactly 0, not merely small -- so no IoU threshold can associate
  them, and the tracker issued 27 distinct ids in 6 seconds of turning. Downstream, a
  consumer locking onto an id lost its lock every few degrees.

  So association is by DISTANCE between a track's PREDICTED position and the detection,
  gated by box size. Prediction matters because the motion is smooth and largely constant
  over a turn: carrying each track's velocity forward removes most of the offset before the
  gate is even applied.
"""
from src.panorama import cyclic_iou


def _wrap(delta, width):
    """Shortest signed distance on the cylinder."""
    delta %= width
    if delta > width / 2:
        delta -= width
    return delta


def _span_overlap(a, b):
    """Fraction of the smaller vertical span that the two share."""
    inter = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    smaller = min(a[3] - a[1], b[3] - b[1])
    return inter / smaller if smaller > 0 else 0.0


class Tracker:
    def __init__(self, pano_width, iou_threshold=0.3, max_age=5,
                 gate_scale=2.0, min_gate_px=60.0, min_vertical_overlap=0.3):
        self.pano_width = pano_width
        self.iou_threshold = iou_threshold      # kept: still used to break ties
        self.max_age = max_age
        # The gate is proportional to box size, with a floor. A distant person is small AND
        # moves fastest in pixels, so a gate tied only to width would be tightest exactly
        # where it needs to be loosest.
        self.gate_scale = gate_scale
        self.min_gate_px = min_gate_px
        # Two boxes in the same column but at very different rows are different objects,
        # not one that jumped. Distance alone cannot tell them apart.
        self.min_vertical_overlap = min_vertical_overlap
        self._tracks = {}      # id -> {"box", "age", "vx"}
        self._next_id = 0

    def _centre(self, box):
        return (box[0] + box[2]) / 2.0

    def update(self, boxes):
        """Assign a track id to each box. Returns ids in the same order as `boxes`."""
        assigned = [None] * len(boxes)

        # Score every (track, detection) pair that passes the gate, then take them
        # best-first. Greedy on a global ordering rather than per-detection, so a strong
        # match is never stolen by an earlier detection with a weaker claim on the track.
        pairs = []
        for tid, track in self._tracks.items():
            predicted = self._centre(track["box"]) + track["vx"]
            t_w = track["box"][2] - track["box"][0]
            for di, box in enumerate(boxes):
                if _span_overlap(track["box"], box) < self.min_vertical_overlap:
                    continue
                d_w = box[2] - box[0]
                gate = max(self.min_gate_px, self.gate_scale * 0.5 * (t_w + d_w))
                dist = abs(_wrap(self._centre(box) - predicted, self.pano_width))
                if dist <= gate:
                    # Normalised so a near miss on a small box does not outrank a solid
                    # match on a large one; IoU breaks ties between equally close matches.
                    iou = cyclic_iou(track["box"], box, self.pano_width)
                    pairs.append((dist / gate - 0.25 * iou, tid, di))
        pairs.sort()

        used_tracks, used_dets = set(), set()
        for _, tid, di in pairs:
            if tid in used_tracks or di in used_dets:
                continue
            prev_c = self._centre(self._tracks[tid]["box"])
            measured = _wrap(self._centre(boxes[di]) - prev_c, self.pano_width)
            # Smoothed velocity: one noisy frame should not fling the prediction away.
            self._tracks[tid] = {"box": boxes[di], "age": 0,
                                 "vx": 0.5 * self._tracks[tid]["vx"] + 0.5 * measured}
            assigned[di] = tid
            used_tracks.add(tid)
            used_dets.add(di)

        for di, box in enumerate(boxes):
            if assigned[di] is None:
                assigned[di] = self._next_id
                self._tracks[self._next_id] = {"box": box, "age": 0, "vx": 0.0}
                self._next_id += 1

        for tid in list(self._tracks):
            if tid not in used_tracks and tid not in set(assigned):
                self._tracks[tid]["age"] += 1
                if self._tracks[tid]["age"] > self.max_age:
                    del self._tracks[tid]

        return assigned
