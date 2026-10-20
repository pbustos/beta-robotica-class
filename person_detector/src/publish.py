"""Map merged, tracked detections onto the VisualElements wire types.

WHAT IS DELIBERATELY NOT FILLED: depth, x/y/z, every v*, and metrics. The Ricoh reports
focalx/focaly as 32149 / -1973400126 -- garbage -- so no image-to-world projection is
possible from this stream. Emitting a plausible-looking zero would be worse than leaving
the field obviously untouched.
"""
import sys

sys.path.append("generated")
import interfaces as ifaces


def to_visual_objects(dets, ids, frame, now_ms, period_ms, publish_roi_pixels=False):
    """Build a TObjects snapshot. `dets` and `ids` are parallel lists."""
    objects = []
    for det, track_id in zip(dets, ids):
        obj = ifaces.RoboCompVisualElements.TObject()
        obj.id = int(track_id)
        obj.type = 0                      # COCO 'person'
        obj.left = int(det.left)
        obj.top = int(det.top)
        obj.right = int(det.right)        # may exceed frame.width when it straddles the seam
        obj.bot = int(det.bot)
        obj.score = float(det.score)

        obj.depth = 0
        obj.x = obj.y = obj.z = 0
        obj.vx = obj.vy = obj.vz = 0
        obj.vrx = obj.vry = obj.vrz = 0
        obj.metrics = []

        roi = ifaces.RoboCompCamera360RGB.TRoi()
        roi.xsize = int(det.right - det.left)
        roi.ysize = int(det.bot - det.top)
        roi.xcenter = int(det.left) + roi.xsize // 2
        roi.ycenter = int(det.top) + roi.ysize // 2
        roi.finalxsize = roi.xsize
        roi.finalysize = roi.ysize

        image = ifaces.RoboCompCamera360RGB.TImage()
        image.compressed = False
        image.cameraID = frame.cameraID
        image.width = frame.width
        image.height = frame.height
        image.depth = frame.depth
        image.focalx = frame.focalx
        image.focaly = frame.focaly
        image.timestamp = frame.timestamp
        image.period = frame.period
        image.roi = roi
        # Pixels are opt-in: a crop per person per tick is a large continuous byte cost for
        # data the consumer can re-fetch with the roi parameters it was just handed.
        image.image = []
        obj.image = image

        person = ifaces.RoboCompPerson.TPerson()
        person.id = int(track_id)
        person.orientation = 0
        person.joints = {}                # ROI-only component: no pose estimation
        person.timestamp = frame.timestamp
        obj.person = person

        objects.append(obj)

    out = ifaces.RoboCompVisualElements.TObjects()
    out.objects = objects
    out.timestampimage = frame.timestamp
    out.timestampgenerated = int(now_ms)
    out.period = float(period_ms)
    return out
