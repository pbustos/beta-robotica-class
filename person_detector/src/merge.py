"""Collapse per-strip detections into one detection per person.

Two distinct problems, solved in order:

  NMS            -- a person inside the 128 px overlap is detected by BOTH strips, twice.
                    Those two boxes overlap heavily, so greedy IoU suppression removes one.

  boundary union -- a person WIDER than the overlap is truncated at a strip edge in every
                    strip that sees them. The two halves barely overlap, so NMS leaves both.
                    They are rejoined by union when they hug the shared edge.

Pure stdlib + numpy-free on purpose; see src/panorama.py.
"""
from dataclasses import dataclass, replace

from src.panorama import canonical, cyclic_iou


@dataclass(frozen=True)
class Detection:
    left: float
    top: float
    right: float
    bot: float
    score: float
    strip: int
    touches_left: bool = False
    touches_right: bool = False

    @property
    def box(self):
        return (self.left, self.top, self.right, self.bot)


def span_iou(a_top, a_bot, b_top, b_bot):
    """1D IoU of two vertical spans."""
    inter = max(0.0, min(a_bot, b_bot) - max(a_top, b_top))
    union = max(a_bot, b_bot) - min(a_top, b_top)
    return inter / union if union > 0.0 else 0.0


def nms(dets, pano_width, iou_threshold=0.5):
    """Greedy non-maximum suppression using cylindrical IoU."""
    kept = []
    for d in sorted(dets, key=lambda x: x.score, reverse=True):
        if all(cyclic_iou(d.box, k.box, pano_width) <= iou_threshold for k in kept):
            kept.append(d)
    return kept


def _adjacent(strip_a, strip_b, strip_count):
    """True when strip_b is the strip immediately clockwise of strip_a, wrapping."""
    return (strip_a + 1) % strip_count == strip_b


def boundary_union(dets, pano_width, strip_count, edge_tolerance=2, vertical_overlap_min=0.5):
    """Union detections that are two halves of one person split at a shared strip edge.

    Requires all three: one touches its strip's right edge and the other its neighbour's
    left edge, the strips are adjacent on the cylinder, and the vertical spans agree. The
    last condition is what stops two different people at the same bearing being fused.
    """
    merged = list(dets)
    changed = True
    while changed:
        changed = False
        for i in range(len(merged)):
            for j in range(len(merged)):
                if i == j:
                    continue
                a, b = merged[i], merged[j]
                if not (a.touches_right and b.touches_left):
                    continue
                if not _adjacent(a.strip, b.strip, strip_count):
                    continue
                if span_iou(a.top, a.bot, b.top, b.bot) < vertical_overlap_min:
                    continue
                if min(abs(a.right - b.right), abs(a.left - b.left)) > pano_width / 2:
                    continue
                union = Detection(
                    left=min(a.left, b.left),
                    top=min(a.top, b.top),
                    right=max(a.right, b.right),
                    bot=max(a.bot, b.bot),
                    score=max(a.score, b.score),
                    strip=a.strip,
                    touches_left=a.touches_left,
                    touches_right=b.touches_right,
                )
                merged = [m for k, m in enumerate(merged) if k not in (i, j)] + [union]
                changed = True
                break
            if changed:
                break
    return merged


def merge(dets, pano_width, strip_count, iou_threshold=0.5,
          edge_tolerance=2, vertical_overlap_min=0.5):
    """NMS first, then boundary union, then canonicalise every surviving box."""
    kept = nms(dets, pano_width, iou_threshold)
    joined = boundary_union(kept, pano_width, strip_count,
                            edge_tolerance, vertical_overlap_min)
    out = []
    for d in joined:
        left, top, right, bot = canonical(d.box, pano_width)
        out.append(replace(d, left=left, top=top, right=right, bot=bot))
    return out
