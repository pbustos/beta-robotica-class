import numpy as np
import pytest

from src.panorama import (
    Strip, strip_layout, extract_strip, box_strip_to_pano, cyclic_iou, canonical,
)

W, H = 1920, 960


def test_strip_layout_centres_and_edges():
    strips = strip_layout(W, count=3, width=768)
    assert [s.centre for s in strips] == [0, 640, 1280]
    assert [s.left for s in strips] == [-384, 256, 896]


def test_strip_layout_overlap_is_uniform():
    strips = strip_layout(W, count=3, width=768)
    step = strips[1].left - strips[0].left
    assert step == 640
    assert 768 - step == 128


def test_extract_strip_wraps_the_seam():
    img = np.zeros((4, W, 3), dtype=np.uint8)
    img[:, :, 0] = (np.arange(W) % 256).astype(np.uint8)
    s0 = strip_layout(W, 3, 768)[0]
    out = extract_strip(img, s0)
    assert out.shape == (4, 768, 3)
    assert out[0, 0, 0] == 1536 % 256
    assert out[0, -1, 0] == 383 % 256


def test_extract_strip_without_wrap_is_a_plain_slice():
    img = np.zeros((4, W, 3), dtype=np.uint8)
    img[:, :, 0] = (np.arange(W) % 256).astype(np.uint8)
    s1 = strip_layout(W, 3, 768)[1]
    out = extract_strip(img, s1)
    assert out[0, 0, 0] == 256 % 256
    assert out[0, -1, 0] == 1023 % 256


def test_box_strip_to_pano_offsets_by_strip_left():
    s1 = strip_layout(W, 3, 768)[1]
    assert box_strip_to_pano((10, 20, 110, 220), s1, W) == (266, 20, 366, 220)


def test_box_strip_to_pano_straddling_seam_reports_right_beyond_width():
    s0 = strip_layout(W, 3, 768)[0]
    left, top, right, bot = box_strip_to_pano((350, 0, 450, 100), s0, W)
    assert left == 1886
    assert right == 1986
    assert right > left
    assert right - left == 100


def test_cyclic_iou_matches_across_the_seam():
    a = (1900, 0, 2000, 100)
    b = (0, 0, 80, 100)
    assert cyclic_iou(a, b, W) > 0.5


def test_cyclic_iou_is_zero_for_genuinely_distant_boxes():
    a = (100, 0, 200, 100)
    b = (900, 0, 1000, 100)
    assert cyclic_iou(a, b, W) == 0.0


def test_cyclic_iou_identical_boxes_is_one():
    a = (100, 0, 200, 100)
    assert cyclic_iou(a, a, W) == pytest.approx(1.0)


def test_canonical_brings_left_into_range_and_preserves_width():
    assert canonical((1886, 0, 1986, 100), W) == (1886, 0, 1986, 100)
    assert canonical((-34, 0, 66, 100), W) == (1886, 0, 1986, 100)
