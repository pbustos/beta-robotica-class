import pytest

from src.merge import Detection, span_iou, nms, boundary_union, merge

W = 1920


def det(left, top, right, bot, score=0.9, strip=0, tl=False, tr=False):
    return Detection(left, top, right, bot, score, strip, tl, tr)


def test_span_iou_full_overlap():
    assert span_iou(0, 100, 0, 100) == pytest.approx(1.0)


def test_span_iou_disjoint():
    assert span_iou(0, 100, 200, 300) == 0.0


def test_span_iou_half_overlap():
    assert span_iou(0, 100, 50, 150) == pytest.approx(1 / 3)


def test_nms_keeps_single_detection():
    out = nms([det(100, 0, 200, 300)], W)
    assert len(out) == 1


def test_nms_collapses_same_person_seen_in_two_strips():
    a = det(300, 0, 400, 300, score=0.9, strip=0)
    b = det(305, 2, 402, 298, score=0.8, strip=1)
    out = nms([a, b], W)
    assert len(out) == 1
    assert out[0].score == 0.9


def test_nms_keeps_two_distinct_people():
    out = nms([det(100, 0, 200, 300), det(900, 0, 1000, 300)], W)
    assert len(out) == 2


def test_nms_collapses_across_the_seam():
    a = det(1900, 0, 2000, 300, score=0.9)
    b = det(0, 0, 80, 300, score=0.7)
    out = nms([a, b], W)
    assert len(out) == 1


def test_boundary_union_joins_a_person_truncated_in_two_strips():
    a = det(300, 100, 384, 400, score=0.7, strip=0, tr=True)
    b = det(256, 105, 460, 395, score=0.6, strip=1, tl=True)
    out = boundary_union([a, b], W, strip_count=3)
    assert len(out) == 1
    assert out[0].left == 256
    assert out[0].right == 460
    assert out[0].top == 100
    assert out[0].bot == 400
    assert out[0].score == pytest.approx(0.7)


def test_boundary_union_ignores_detections_not_touching_an_edge():
    a = det(300, 100, 380, 400, strip=0, tr=False)
    b = det(260, 105, 460, 395, strip=1, tl=False)
    out = boundary_union([a, b], W, strip_count=3)
    assert len(out) == 2


def test_boundary_union_requires_vertical_overlap():
    a = det(300, 0, 384, 100, strip=0, tr=True)
    b = det(256, 700, 460, 900, strip=1, tl=True)
    out = boundary_union([a, b], W, strip_count=3)
    assert len(out) == 2


def test_boundary_union_treats_last_and_first_strip_as_adjacent():
    a = det(1600, 100, 1664, 400, score=0.6, strip=2, tr=True)
    b = det(1536, 105, 1700, 395, score=0.8, strip=0, tl=True)
    out = boundary_union([a, b], W, strip_count=3)
    assert len(out) == 1
    assert out[0].score == pytest.approx(0.8)


def test_merge_runs_nms_then_union():
    a = det(300, 100, 384, 400, score=0.7, strip=0, tr=True)
    b = det(256, 105, 460, 395, score=0.6, strip=1, tl=True)
    c = det(900, 0, 1000, 300, score=0.9, strip=1)
    out = merge([a, b, c], W, strip_count=3)
    assert len(out) == 2
