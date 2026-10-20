import numpy as np

from src.detector import PersonDetector

W, H = 1920, 960


class FakeBox:
    def __init__(self, xyxy, conf):
        self.xyxy = [np.array(xyxy, dtype=float)]
        self.conf = [float(conf)]


class FakeMasks:
    def __init__(self, polys):
        self.xy = [np.array(p, dtype=float) for p in polys]


class FakeResult:
    def __init__(self, boxes, masks=None):
        self.boxes = boxes
        self.masks = masks


class FakeModel:
    """Returns one canned Result per strip and records what it was called with."""

    def __init__(self, results):
        self.results = results
        self.calls = []

    def predict(self, images, **kwargs):
        self.calls.append((images, kwargs))
        return self.results


def blank():
    return np.zeros((H, W, 3), dtype=np.uint8)


def test_detect_batches_all_strips_in_one_call():
    model = FakeModel([FakeResult([]), FakeResult([]), FakeResult([])])
    PersonDetector(model, W).detect(blank())
    assert len(model.calls) == 1
    images, kwargs = model.calls[0]
    assert len(images) == 3
    assert kwargs["classes"] == [0]


def test_detect_maps_box_into_panorama_coordinates():
    model = FakeModel([
        FakeResult([]),
        FakeResult([FakeBox([10, 20, 110, 220], 0.9)]),
        FakeResult([]),
    ])
    dets = PersonDetector(model, W).detect(blank())
    assert len(dets) == 1
    assert (dets[0].left, dets[0].top, dets[0].right, dets[0].bot) == (266, 20, 366, 220)
    assert dets[0].strip == 1


def test_detect_prefers_the_mask_extent_over_the_detector_box():
    model = FakeModel([
        FakeResult([]),
        FakeResult(
            [FakeBox([10, 20, 110, 220], 0.9)],
            FakeMasks([[[30, 40], [90, 40], [90, 200], [30, 200]]]),
        ),
        FakeResult([]),
    ])
    dets = PersonDetector(model, W).detect(blank())
    assert (dets[0].left, dets[0].top, dets[0].right, dets[0].bot) == (286, 40, 346, 200)


def test_detect_falls_back_to_the_box_when_masks_are_absent():
    model = FakeModel([
        FakeResult([]),
        FakeResult([FakeBox([10, 20, 110, 220], 0.9)], masks=None),
        FakeResult([]),
    ])
    dets = PersonDetector(model, W).detect(blank())
    assert (dets[0].left, dets[0].right) == (266, 366)


def test_detect_flags_boxes_touching_strip_edges():
    model = FakeModel([
        FakeResult([]),
        FakeResult([FakeBox([0, 20, 50, 220], 0.9), FakeBox([700, 20, 768, 220], 0.8)]),
        FakeResult([]),
    ])
    dets = PersonDetector(model, W).detect(blank())
    assert dets[0].touches_left and not dets[0].touches_right
    assert dets[1].touches_right and not dets[1].touches_left


def test_detect_straddling_seam_reports_right_beyond_width():
    model = FakeModel([
        FakeResult([FakeBox([350, 0, 450, 100], 0.9)]),
        FakeResult([]),
        FakeResult([]),
    ])
    dets = PersonDetector(model, W).detect(blank())
    assert dets[0].left == 1886
    assert dets[0].right == 1986
    assert dets[0].right > dets[0].left
