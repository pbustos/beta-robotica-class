#!/usr/bin/env python3
"""Convert Ultralytics YOLO .pt weights to TensorRT .engine files.

A TensorRT engine is built for ONE GPU and ONE TensorRT version: build it on the machine
that will run it. Engines are dynamic by default: they take any batch from 1 to --batch
and any image size from 32 px to twice --imgsz, and run fastest at --imgsz. With
--static they take only --imgsz and --batch exactly, which is a little faster. The
.engine is written next to the .pt.

Without --imgsz the engine is tuned for the Ricoh panorama, read from the robot's PROTO:
webots-bridge joins camera_360_1 and camera_360_2 side by side, so the panorama is their
height x twice their width (960 x 1920 today).

Examples:
  ./pt2engine.py yolo26n.pt                          # tuned for the panorama, batch 1, FP16
  ./pt2engine.py yolo26l-seg.pt --imgsz 480 768 --batch 3
  ./pt2engine.py *.pt --static --imgsz 640 --batch 1 # fixed 640x640, batch 1 only
  ./pt2engine.py yolo26s.pt --int8 --data coco8.yaml # needs calibration images
"""
import argparse
import math
import re
import sys
import time
from pathlib import Path

SHADOW_PROTO = Path("~/robocomp/components/webots-shadow/protos/Shadow.proto").expanduser()
PANO_CAMERAS = ("camera_360_1", "camera_360_2")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("weights", nargs="+", type=Path, help=".pt file(s) to convert")
    p.add_argument("--imgsz", nargs="+", type=int, default=None,
                   help="input size the engine is tuned for (exact size with --static): "
                        "one value (square) or H W; rounded up to a multiple of 32. "
                        "Default: the panorama size read from --proto")
    p.add_argument("--proto", type=Path, default=SHADOW_PROTO,
                   help=f"robot PROTO to read the panorama size from (default {SHADOW_PROTO})")
    p.add_argument("--batch", type=int, default=1,
                   help="maximum batch size (exact batch with --static)")
    precision = p.add_mutually_exclusive_group()
    precision.add_argument("--fp32", action="store_true", help="full precision (default is FP16)")
    precision.add_argument("--int8", action="store_true", help="INT8, calibrated on --data")
    p.add_argument("--data", default=None, help="dataset yaml for INT8 calibration")
    p.add_argument("--static", action="store_true",
                   help="fix the input to exactly --imgsz and --batch")
    p.add_argument("--workspace", type=float, default=None, help="TensorRT workspace, GiB")
    p.add_argument("--device", default="0", help="CUDA device")
    p.add_argument("--no-check", action="store_true", help="skip the test inference")
    return p.parse_args()


def camera_size(proto_text, name):
    """(width, height) of the Camera node called `name`, from the PROTO source."""
    at = proto_text.find(f'name "{name}"')
    if at < 0:
        raise ValueError(f"no camera named {name}")
    start = proto_text.rfind("Camera {", 0, at)
    if start < 0:
        raise ValueError(f"{name} is not inside a Camera node")
    depth, end = 0, start
    for end in range(proto_text.index("{", start), len(proto_text)):
        depth += {"{": 1, "}": -1}.get(proto_text[end], 0)
        if depth == 0:
            break
    # Only the Camera's own fields: drop nested nodes so a child's width cannot match.
    body = proto_text[proto_text.index("{", start) + 1:end]
    while re.search(r"\{[^{}]*\}", body):
        body = re.sub(r"\{[^{}]*\}", "", body)
    size = []
    for field, default in (("width", 64), ("height", 64)):  # Webots Camera defaults
        m = re.search(rf"(?<!\w){field}\s+(\d+)", body)
        size.append(int(m.group(1)) if m else default)
    return tuple(size)


def panorama_imgsz(proto):
    """[H, W] of the panorama webots-bridge builds from the two 360 cameras."""
    text = re.sub(r"#.*", "", proto.read_text())
    sizes = [camera_size(text, name) for name in PANO_CAMERAS]
    if sizes[0] != sizes[1]:
        raise ValueError(f"360 cameras differ in size: {sizes}")
    w, h = sizes[0]
    print(f"panorama from {proto}: 2 cameras of {w}x{h} -> {h} x {2 * w} (H x W)")
    return [h, 2 * w]


def round_imgsz(imgsz):
    if len(imgsz) not in (1, 2):
        sys.exit("--imgsz takes one value or two (H W)")
    rounded = [int(math.ceil(v / 32) * 32) for v in imgsz]
    if rounded != imgsz:
        print(f"imgsz {imgsz} rounded up to {rounded} (YOLO stride is 32)")
    return rounded[0] if len(rounded) == 1 else rounded


def check_engine(engine, task, imgsz, batch, device):
    """Load the engine and run one inference at the built shape."""
    import numpy as np
    from ultralytics import YOLO

    h, w = (imgsz, imgsz) if isinstance(imgsz, int) else imgsz
    frames = [np.zeros((h, w, 3), dtype=np.uint8)] * batch
    model = YOLO(str(engine), task=task)
    model.predict(frames, imgsz=imgsz, device=device, verbose=False)  # warm-up
    t0 = time.perf_counter()
    model.predict(frames, imgsz=imgsz, device=device, verbose=False)
    print(f"  check ok: batch {batch} at {h}x{w} in {(time.perf_counter() - t0) * 1000:.1f} ms")


def main():
    args = parse_args()
    if args.int8 and not args.data:
        sys.exit("--int8 needs --data <dataset.yaml> for calibration")

    import torch
    if not torch.cuda.is_available():
        sys.exit("CUDA is not available: TensorRT engines can only be built on a GPU")
    from ultralytics import YOLO

    if args.imgsz is None:
        try:
            args.imgsz = panorama_imgsz(args.proto)
        except (OSError, ValueError) as e:
            sys.exit(f"cannot read the panorama size from {args.proto}: {e}\n"
                     f"give the size with --imgsz H W")
    imgsz = round_imgsz(args.imgsz)
    failed = []
    for pt in args.weights:
        if pt.suffix != ".pt" or not pt.is_file():
            print(f"skipping {pt}: not a .pt file")
            failed.append(pt)
            continue
        precision = "INT8" if args.int8 else "FP32" if args.fp32 else "FP16"
        print(f"\n{pt}: imgsz {imgsz}, batch {args.batch}, {precision}"
              f"{', static' if args.static else ', dynamic'}")
        try:
            export_kwargs = dict(format="engine", imgsz=imgsz, batch=args.batch,
                                 half=not args.fp32 and not args.int8, int8=args.int8,
                                 dynamic=not args.static, device=args.device)
            if args.data:
                export_kwargs["data"] = args.data
            if args.workspace:
                export_kwargs["workspace"] = args.workspace
            model = YOLO(str(pt))
            engine = Path(model.export(**export_kwargs))
            print(f"  wrote {engine} ({engine.stat().st_size / 1e6:.1f} MB)")
            if not args.no_check:
                check_engine(engine, model.task, imgsz, args.batch, args.device)
        except Exception as e:  # keep converting the rest
            print(f"  FAILED: {e}")
            failed.append(pt)

    if failed:
        sys.exit(f"\n{len(failed)} of {len(args.weights)} failed: {', '.join(map(str, failed))}")


if __name__ == "__main__":
    main()
