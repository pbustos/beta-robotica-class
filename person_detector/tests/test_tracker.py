from src.tracker import Tracker

W = 1920


def test_first_frame_assigns_sequential_ids():
    t = Tracker(W)
    assert t.update([(100, 0, 200, 300), (900, 0, 1000, 300)]) == [0, 1]


def test_same_person_keeps_id_across_frames():
    t = Tracker(W)
    first = t.update([(100, 0, 200, 300)])
    second = t.update([(110, 0, 210, 300)])
    assert first == second


def test_distinct_people_keep_distinct_ids():
    t = Tracker(W)
    t.update([(100, 0, 200, 300), (900, 0, 1000, 300)])
    assert t.update([(105, 0, 205, 300), (905, 0, 1005, 300)]) == [0, 1]


def test_new_person_gets_a_new_id():
    t = Tracker(W)
    t.update([(100, 0, 200, 300)])
    ids = t.update([(100, 0, 200, 300), (900, 0, 1000, 300)])
    assert ids[0] == 0
    assert ids[1] == 1


def test_track_retired_after_max_age_then_reappearance_is_a_new_id():
    t = Tracker(W, max_age=2)
    assert t.update([(100, 0, 200, 300)]) == [0]
    t.update([])
    t.update([])
    t.update([])
    assert t.update([(100, 0, 200, 300)]) == [1]


def test_track_survives_a_gap_shorter_than_max_age():
    t = Tracker(W, max_age=5)
    assert t.update([(100, 0, 200, 300)]) == [0]
    t.update([])
    assert t.update([(100, 0, 200, 300)]) == [0]


def test_association_follows_a_person_across_the_seam():
    t = Tracker(W)
    assert t.update([(1880, 0, 1980, 300)]) == [0]
    assert t.update([(1890, 0, 1990, 300)]) == [0]
    assert t.update([(10, 0, 110, 300)]) == [0]


def test_ids_are_never_reused_while_a_track_is_alive():
    t = Tracker(W)
    t.update([(100, 0, 200, 300)])
    t.update([(100, 0, 200, 300), (900, 0, 1000, 300)])
    ids = t.update([(100, 0, 200, 300), (900, 0, 1000, 300)])
    assert len(set(ids)) == 2


# --- robustness against the robot's own rotation -------------------------------------
# Measured on the real rig: turning at ~1 rad/s shifts a person ~45 px per tick while a
# distant person's box is only ~39 px wide. The boxes then do not overlap AT ALL, so any
# IoU-based association fails regardless of threshold, and the detector issued 27 distinct
# ids in 6 seconds. Association has to survive zero overlap.

def test_id_survives_a_jump_larger_than_the_box():
    t = Tracker(W)
    first = t.update([(100, 200, 139, 500)])        # 39 px wide, as measured
    second = t.update([(145, 200, 184, 500)])       # 45 px later: zero overlap
    assert first == second


def test_id_survives_a_sustained_sweep():
    """A full rotation's worth of steps must not renumber even once."""
    t = Tracker(W)
    ids = [t.update([(x, 200, x + 39, 500)])[0] for x in range(100, 900, 45)]
    assert len(set(ids)) == 1, f"renumbered {len(set(ids))} times across the sweep"


def test_prediction_does_not_steal_a_different_person():
    """Two people the same distance apart as the jump must stay distinct."""
    t = Tracker(W)
    a = t.update([(100, 200, 139, 500), (190, 200, 229, 500)])
    b = t.update([(145, 200, 184, 500), (235, 200, 274, 500)])
    assert a == b
    assert len(set(b)) == 2


def test_vertically_disjoint_boxes_are_not_associated():
    """Same column, very different rows: a different object, not the same one moved."""
    t = Tracker(W)
    first = t.update([(100, 100, 139, 300)])
    second = t.update([(110, 700, 149, 900)])
    assert first != second
