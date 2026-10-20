"""Cylindrical geometry for an equirectangular panorama.

An equirectangular frame is a CYLINDER, not a rectangle: column 0 and column `pano_width`
are the same bearing. Every function here respects that. Pure stdlib + numpy on purpose --
no Ice, no torch, no Qt -- so the geometry can be tested without a GPU or a robot.

Boxes are `(left, top, right, bot)` tuples throughout.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Strip:
    """One vertical slice of the panorama.

    `left` is the panorama column of the slice's left edge and MAY BE NEGATIVE: strip 0 is
    centred on the seam, so it starts at -width/2. Callers take it modulo the panorama
    width when they need a real column index.
    """
    index: int
    centre: int
    left: int
    width: int


def strip_layout(pano_width, count, width):
    """`count` strips of `width` px, centred at evenly spaced bearings starting at 0.

    With the defaults (1920, 3, 768) this gives centres 0/640/1280 and 128 px of overlap on
    every boundary including the seam, so no bearing is blind.
    """
    step = pano_width // count
    return [
        Strip(index=k, centre=k * step, left=k * step - width // 2, width=width)
        for k in range(count)
    ]


def extract_strip(image, strip):
    """Columns [strip.left, strip.left + strip.width) taken modulo the panorama width.

    Fancy-indexing rather than slicing, so a strip that crosses the seam comes back as one
    contiguous array instead of needing the caller to concatenate two pieces.
    """
    cols = np.arange(strip.left, strip.left + strip.width) % image.shape[1]
    return image[:, cols]


def box_strip_to_pano(box, strip, pano_width):
    """Map a strip-local box to panorama coordinates.

    The right edge is derived as `left + width` rather than taken modulo, so a box that
    straddles the seam reports `right > pano_width` instead of `right < left`. That keeps
    `right - left` positive for every consumer; they recover the bearing with `x % width`.
    """
    left, top, right, bot = box
    pano_left = (left + strip.left) % pano_width
    return (pano_left, top, pano_left + (right - left), bot)


def _iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0.0 else 0.0


def cyclic_iou(a, b, pano_width):
    """IoU on the cylinder.

    Two boxes describing the same person either side of the seam have zero overlap in flat
    coordinates. Trying all three horizontal shifts and keeping the best is cheap and has no
    tie-breaking subtleties, unlike reasoning about centre distance.
    """
    return max(
        _iou(a, (b[0] + shift, b[1], b[2] + shift, b[3]))
        for shift in (-pano_width, 0, pano_width)
    )


def canonical(box, pano_width):
    """Bring `left` into [0, pano_width) while preserving the width, so `right > left`."""
    left, top, right, bot = box
    width = right - left
    left = left % pano_width
    return (left, top, left + width, bot)
