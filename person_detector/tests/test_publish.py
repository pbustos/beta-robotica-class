import sys

sys.path.append("generated")

import interfaces as ifaces

from src.merge import Detection
from src.publish import to_visual_objects


def make_frame():
    f = ifaces.RoboCompCamera360RGB.TImage()
    f.width, f.height, f.depth = 1920, 960, 3
    f.cameraID = 0
    f.focalx, f.focaly = 0, 0
    f.period = 33.0
    f.timestamp = 1234
    f.compressed = False
    f.image = []
    return f


def test_produces_one_object_per_detection():
    dets = [Detection(100, 10, 200, 300, 0.9, 0), Detection(900, 20, 1000, 310, 0.8, 1)]
    out = to_visual_objects(dets, [7, 9], make_frame(), now_ms=5000, period_ms=100.0)
    assert len(out.objects) == 2
    assert [o.id for o in out.objects] == [7, 9]


def test_roi_box_and_score_are_filled():
    out = to_visual_objects([Detection(100, 10, 200, 300, 0.9, 0)], [0],
                            make_frame(), now_ms=5000, period_ms=100.0)
    o = out.objects[0]
    assert (o.left, o.top, o.right, o.bot) == (100, 10, 200, 300)
    assert abs(o.score - 0.9) < 1e-6
    assert o.type == 0


def test_three_d_fields_are_left_at_zero():
    out = to_visual_objects([Detection(100, 10, 200, 300, 0.9, 0)], [0],
                            make_frame(), now_ms=5000, period_ms=100.0)
    o = out.objects[0]
    assert (o.depth, o.x, o.y, o.z) == (0, 0, 0, 0)
    assert (o.vx, o.vy, o.vz, o.vrx, o.vry, o.vrz) == (0, 0, 0, 0, 0, 0)
    assert list(o.metrics) == []


def test_image_carries_roi_parameters_but_no_pixels_by_default():
    out = to_visual_objects([Detection(100, 10, 200, 300, 0.9, 0)], [0],
                            make_frame(), now_ms=5000, period_ms=100.0)
    img = out.objects[0].image
    assert img.roi.xcenter == 150
    assert img.roi.ycenter == 155
    assert img.roi.xsize == 100
    assert img.roi.ysize == 290
    assert img.width == 1920 and img.height == 960 and img.depth == 3
    assert len(img.image) == 0


def test_person_has_id_and_timestamp_but_no_joints():
    out = to_visual_objects([Detection(100, 10, 200, 300, 0.9, 0)], [7],
                            make_frame(), now_ms=5000, period_ms=100.0)
    p = out.objects[0].person
    assert p.id == 7
    assert p.timestamp == 1234
    assert len(p.joints) == 0


def test_timestamps_and_period_are_carried():
    out = to_visual_objects([], [], make_frame(), now_ms=5000, period_ms=100.0)
    assert out.timestampimage == 1234
    assert out.timestampgenerated == 5000
    assert abs(out.period - 100.0) < 1e-6


def test_seam_straddling_box_keeps_right_greater_than_left():
    out = to_visual_objects([Detection(1886, 0, 1986, 100, 0.9, 0)], [0],
                            make_frame(), now_ms=5000, period_ms=100.0)
    o = out.objects[0]
    assert o.right > o.left
    assert o.right == 1986
