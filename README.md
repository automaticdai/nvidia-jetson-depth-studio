# nvidia-jetson-depth-studio
Toolkit to support depth vision with monocular and depth cameras.

Targets an NVIDIA Jetson Nano (JetPack 4.6, Ubuntu 18.04) with an Intel RealSense D435i and the
on-board MIPI CSI camera (IMX219), but the frame/recording layer is plain NumPy + OpenCV.

## Features

- **One `Source` interface** for every camera, yielding `Frame`s (BGR colour, uint16 depth,
  intrinsics, IMU samples):
  - `realsense`: depth + colour (aligned to colour) + accelerometer/gyroscope, started video-first
    so the D435i IMU does not hang on the Nano (see below)
  - `csi[:N]`: CSI camera via `nvarguscamerasrc` (no GStreamer-enabled OpenCV needed)
  - `usb[:N|/dev/videoN]`: UVC webcams
  - a recording directory, played back through the same interface
- **Recording** to a plain directory (JPEG/PNG colour, 16-bit PNG depth, CSV frames + IMU, JSON
  metadata); sustains 30 fps RGB-D + 600 Hz IMU on the Nano.
- **Snapshots** with a coloured binary PLY point cloud.
- **Live preview** over HTTP (MJPEG), for a headless Jetson.

## Setup (Jetson Nano)

1. Build librealsense with Python 3.8 bindings (JetPack's `python3` is 3.6, too old for current
   pyrealsense2). Takes 1–2 h:

   ```bash
   sudo scripts/install_librealsense.sh
   ```

2. Stop the kernel HID sensor drivers from resetting the D435i IMU (already done by step 1 on new
   installs):

   ```bash
   sudo scripts/fix_imu.sh
   ```

3. Install this package for Python 3.8:

   ```bash
   python3.8 -m pip install --user -e '.[dev]'
   ```

   The `depthstudio` command lands in `~/.local/bin` (on `PATH` after re-login).

## Command line

```bash
depthstudio devices                          # list cameras
depthstudio snapshot                         # snapshots/<time>/ colour, depth, points.ply
depthstudio snapshot -s csi:0                # CSI camera frame
depthstudio record -d 10                     # recordings/<time>/, Ctrl-C to stop early
depthstudio record -d 10 --lossless --no-imu
depthstudio info recordings/20261103-153135
depthstudio stream --port 8080               # open http://<jetson>:8080/ in a browser
depthstudio stream -s recordings/20261103-153135   # replay a recording
```

`--width/--height/--fps` select the stream mode, e.g. `--width 848 --height 480`.

## Python

```python
from depthstudio import open_source
from depthstudio.record import Recorder

with open_source("realsense") as cam:
    for frame in cam.frames(duration=5):
        z = frame.depth_m()                      # metres, aligned to frame.color
        pts = frame.point_cloud()                # N x 3, metres
        accel = [s for s in frame.imu if s.kind == "accel"]
```

Frame timestamps (`Frame.t`, `ImuSample.t`) are seconds; RealSense uses its host-synchronised
global time, the other sources `time.time()`, so streams from different cameras can be matched.

## Examples

Standalone pyrealsense2 scripts (no `depthstudio` import), handy for checking an install:

```bash
python3.8 examples/rs_all.py    # depth + colour + IMU (IMU started last), frame counts and gravity
python3.8 examples/rs_test.py   # device info, aligned depth/colour, point cloud, separate IMU session
```

## Recording format

```
meta.json     source, depth scale, intrinsics, counts
frames.csv    index,t,color,depth
imu.csv       frame_index,t,kind,x,y,z      (accel m/s^2, gyro rad/s)
color/        000000.jpg (or .png with --lossless)
depth/        000000.png   uint16, multiply by depth_scale for metres
```

## Tests

```bash
python3.8 -m pytest                          # unit tests, no hardware
DEPTHSTUDIO_HW=1 python3.8 -m pytest -m hardware   # D435i + CSI camera attached
```

## Jetson Nano notes

- **Start the IMU after the video streams.** If the IMU is running while librealsense starts the
  colour stream, colour often gets no frames and the IMU goes silent until the camera is
  hardware-reset. `rs2.pipeline` always starts the IMU first, so the default pipeline
  (`rs-hello-realsense`, `rs-save-to-disk`) and any pipeline with accel/gyro enabled fail with
  "Frame didn't arrive". `RealSenseSource` opens the sensors itself: depth and colour first, the IMU
  once colour frames are arriving, and it resets the camera and retries if a stream stays silent.
  Upgrading librealsense (2.58.4) and changing firmware (5.17.3.10 → 5.17.0.10) did not help.
  Details, measurements and diagnostic tools: [docs/realsense-imu-hang.md](docs/realsense-imu-hang.md)
  and `scripts/diagnostics/`.
- **Kernel HID sensor drivers**: the `hid_sensor_*` drivers re-bind to the IMU interface whenever
  librealsense releases it; keep them blacklisted with `scripts/fix_imu.sh`. This alone does not
  prevent the hang above.
- **Python callbacks**: `sensor.start(callback)` deadlocks in this pyrealsense2 build; the source
  uses a `frame_queue` instead.
- The D435i ships without IMU calibration (gravity reads ~9.57 m/s²); calibrate with
  `/usr/local/src/librealsense/tools/rs-imu-calibration/rs-imu-calibration.py` for accurate motion data.
