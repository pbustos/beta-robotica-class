# vision_segmentation

`vision_segmentation` is a RoboComp component that reads RGBD data, runs YOLO segmentation, projects segmented regions to 3D points, and publishes the results through the `ImageSegmentation` ICE interface.

## What the component does

- Consumes frames from `CameraRGBDSimple`.
- Runs YOLO segmentation (`ultralytics`) over the RGB image.
- Builds segmented objects with:
	- `label`
	- `score`
	- `imagePolygon` (2D contour points)
	- `points3D` (depth-projected 3D points)
- Serves image/objects through ICE methods.
- Optionally renders:
	- left panel: RGB + overlays
	- right panel: Qt3D segmented point cloud

## Implemented ICE interface

Module: `RoboCompImageSegmentation`

Main data types:

- `Point2D { int x; int y; }`
- `Point3D { float x; float y; float z; }`
- `SegmentedObject { string label; float score; Polygon imagePolygon; PointCloud points3D; }`
- `TImage { compressed, cameraID, width, height, depth, focalx, focaly, alivetime, period, image }`
- `TData { ObjectList objects; TImage image; long timestamp; }`

Implemented methods:

- `ObjectList getSegmentedObjects()`
- `TImage getImage()`
- `TData getAll()`

## Requirements

### System

- Linux
- RoboComp installed
- ICE runtime available
- CMake + compiler toolchain

### Python

- Python 3.10+
- `numpy`
- `opencv-python`
- `ultralytics`
- `PySide6`
- `rich`

Install Python dependencies:

```bash
pip install numpy opencv-python ultralytics PySide6 rich
```

## Build

From component root:

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## Configuration

Default config file: `etc/config`

Relevant parameters:

- `Proxies.CameraRGBDSimple`: source RGBD endpoint.
- `Endpoints.ImageSegmentation`: endpoint exposed by this component.
- `Display`: `True`/`False` to enable GUI rendering.
- `ProxyThread`: `True`/`False` to read camera frames in a background thread.
- `Period.Compute`: compute loop period (ms).

Example:

```ini
Proxies.CameraRGBDSimple = "camerargbdsimple:tcp -h localhost -p 10096"
Endpoints.ImageSegmentation = "tcp -p 14111"
Display = False
ProxyThread = True
Period.Compute = 50
```

## Run

Recommended:

```bash
cp etc/config config
bin/vision_segmentation config
```

## Notes

- With `Display=False`, rendering work is skipped to maximize throughput.
- The component prints FPS periodically.

## Troubleshooting

- **Low FPS (expected around 20Hz but getting less)**
	- Set `Display=False` to disable GUI overhead.
	- Try `ProxyThread=False` and compare performance (in some systems direct proxy reads are more stable).
	- Use a lighter segmentation model checkpoint if needed.

- **Proxy errors / no input frames**
	- Verify `Proxies.CameraRGBDSimple` endpoint in `etc/config`.
	- Ensure the camera component is running and reachable.
	- Check ICE network settings and port conflicts.

- **Model not loading**
	- Confirm `ultralytics` is installed in the active Python environment.
	- Ensure model weights are accessible from the runtime environment.

- **No overlays or empty segmented objects**
	- Ensure the input image has valid content and size.
	- If no clients are requesting image/object data, some optional overlay work may be skipped for performance.

- **Qt / display errors**
	- If running headless, keep `Display=False`.
	- Ensure `PySide6` and Qt3D packages are installed when display is enabled.
