# person_detector

Detects people in the Ricoh 360 stream with `yolo26l-seg` over three overlapping vertical
strips at 10 Hz, and serves their ROIs over `VisualElements` on port 11105.

## Run

    python3 generated/person_detector.py etc/config.toml

Requires `ricoh_omni_dds` serving `Camera360RGB` on port 10097.

Unit tests (no GPU, no robot needed):

    python3 -m pytest tests/ -v

## Two contracts a consumer cannot infer from the IDSL

**1. `right` may exceed `image.width`.** The panorama is a cylinder: a person standing at
bearing 0 straddles the seam. Such a box is reported as e.g. `left=1850, right=1990` so
that `right > left` always holds and `right - left` is always a positive width. Recover the
bearing with `x % image.width`.

**2. `getVisualObjects` ignores its argument.** This component is a pure producer. It
returns the most recent snapshot regardless of what is passed in. `setVisualObjects` is a
no-op and logs once.

## What is NOT filled

`depth`, `x`, `y`, `z`, every `v*`, and `metrics` are left at zero, and `person.joints` is
empty. The Ricoh reports `focalx`/`focaly` as `32149` / `-1973400126`, so no image-to-world
projection is possible from this stream. This component reports ROI geometry only.

The segmentation mask is used to tighten the reported box and is then discarded -- it is
never published.

## How the strips work

The 1920 px panorama is cut into three 768 px strips centred at x = 0, 640 and 1280:

    S0: [1536..1920] + [0..384]   (wraps the seam)
    S1: [256..1024]
    S2: [896..1664]

Every boundary, including the seam, has 128 px of overlap, so no bearing is blind and a
person standing at bearing 0 is whole inside S0 rather than cut in half.

All three strips go through YOLO in a single batched `predict` call. Duplicates from the
overlap are removed by NMS computed on the cylinder; a person *wider* than the overlap is
truncated in every strip that sees them and is rejoined by the boundary-union rule.

## Measured performance

On an RTX 5070 Ti, against the live Webots Ricoh:

| Stage | ms |
|---|---|
| camera fetch (capture thread, off the tick) | ~4 |
| decode + reshape | <1 |
| strip extraction (3 x 768x960, wrap-indexed + contiguous) | ~15 |
| batched inference, 3 strips @ 960x768 | ~40 |
| mask polygon extraction | ~1 |
| merge + track + publish | <1 |
| **detect() total** | **~56** |
| **measured end-to-end** | **~99 (10.1 Hz)** |

Measured single-process against the live Webots Ricoh with one person in view. The tick
does ~57 ms of work inside a 100 ms period, so there is ~43 ms of headroom; the pipeline
would run at ~17 Hz if the timer allowed it.

Alternative weights and runtimes on the same frame, measured through
`PersonDetector.detect()` with nothing else on the GPU:

| variant | detect() | vs PyTorch |
|---|---|---|
| `yolo26l-seg.pt` | 57.6 ms | -- |
| **`yolo26l-seg.engine`** (TensorRT fp16) | **27.8 ms** | **2.07x** |
| `yolo26s-seg.pt` | 32.7 ms | -- |
| `yolo26s-seg.engine` (TensorRT fp16) | 21.8 ms | 1.50x |

**TensorRT on the large model beats PyTorch on the small one**, so there is no reason to
trade segmentation accuracy for speed here: keep `yolo26l-seg` and build an engine.

`Model` is left as the `.pt` in the committed config because an engine is **not portable**
-- it is tied to this GPU, TensorRT version and driver, and is gitignored for that reason.
To opt in, build one and point `Model` at it:

```python
from ultralytics import YOLO
YOLO("yolo26l-seg.pt").export(format="engine", imgsz=(960, 768), batch=3, half=True, device=0)
```

`imgsz` and `batch` must match how the component calls the model (three 768-wide strips of
a 960-tall frame); an engine built for other shapes will not load. Verified end to end:
with `Model = "yolo26l-seg.engine"` the component loads, detects and holds 10 Hz.

With TensorRT the CPU-side strip extraction (~15 ms) becomes the largest single cost --
more than half of `detect()`. Two of the three strips do not wrap and could be plain
slices (views) instead of fancy-indexed copies, which is the next thing worth doing if the
tick ever needs to be shorter.

The capture thread is **paced** (`CapturePeriod`, default half the tick). Left unpaced it
re-read the same frame ~275 times a second against a 30 Hz source and held the GIL
throughout, starving the Qt thread and dropping the component to 7.6-9.1 Hz.

## Tuning

See `[Detector]` in `etc/config.toml`. `StripWidth` controls the overlap: with
`StripCount = 3` on a 1920 px panorama the step is 640, so the overlap is
`StripWidth - 640`. Only people narrower than that overlap are guaranteed to appear whole
in a single strip; wider ones are rejoined by the boundary-union rule, tuned with
`EdgeTolerance` and `VerticalOverlapMin`.

Set `PublishRoiPixels = true` to include the cropped pixels in `TObject.image.image`. Off
by default: a crop per person per tick is a large continuous byte cost for data the
consumer can re-fetch with the `roi` parameters it was just handed.

## Design documents

- spec: `../docs/superpowers/specs/2026-09-20-person-detector-design.md`
- plan: `../docs/superpowers/plans/2026-09-20-person-detector.md`
