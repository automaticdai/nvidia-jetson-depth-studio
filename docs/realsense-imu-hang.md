# RealSense D435i on the Jetson Nano: IMU / colour hang

Investigation from 2026-11-11/12. **Summary:** if the IMU is running while librealsense starts
the colour stream, the camera often hangs: colour delivers no frames and the IMU sends one sample
and then goes silent until a hardware reset. `rs2.pipeline` always starts the IMU first, so the
default pipeline, and any pipeline with accel/gyro enabled, is unreliable on this machine.
`RealSenseSource` therefore starts depth and colour first and the IMU last; with that order, no
run hung in 40+ trials.

## Setup

| | |
|---|---|
| Board | Jetson Nano Developer Kit, L4T R32.7.1, kernel 4.9.253-tegra |
| Camera | RealSense D435I, s/n 141722070627, USB 3.2 link on port 2-1.4 |
| librealsense | 2.57.7 built from source in `/usr/local/src/librealsense`, `FORCE_RSUSB_BACKEND=ON`, CUDA, Python 3.8 bindings |
| Kernel HID sensors | blacklisted (`scripts/fix_imu.sh`); `hid_sensor_*` not loaded during any test below |

## Symptoms

- `rs-hello-realsense`, `rs-save-to-disk`: `Frame didn't arrive within 15000`.
- Debug log of the default pipeline (depth 848x480, colour 1280x720, accel 100 Hz, gyro 200 Hz):
  depth and colour arrive at about 30 fps but no accel/gyro frame ever does, so the pipeline's syncer
  never releases a frameset with every requested stream and `wait_for_frames` times out.
- In the failure mode itself: depth keeps streaming, **colour gets 0 frames, the IMU 1 sample**.
- Once hung, the IMU accepts power-on (the HID feature report reads back power state D0) but sends
  no interrupt reports. Only a hardware reset (`dev.hardware_reset()`) or replug recovers it.
  Restarting the program does not.

## What triggers it

Per-sensor start-order tests (`scripts/diagnostics/sensor_order_test.cpp`; D = depth, C = colour,
M = motion/IMU), each run straight after a hardware reset unless noted:

| Start sequence | Hung |
|---|---|
| IMU alone, repeatedly | 0 |
| depth + IMU, colour + IMU, depth + colour + IMU via explicit sensor start (rs-data-collect) | 0 of 4 |
| D, C, M (video first), 0.3–1.5 s apart | 0 of 7, plus 1 hang in a run straight after using `imu_probe` (see Diagnostics) |
| D, C, M, all opened then all started | 0 of 3 |
| M, D, C | 0 of 1 |
| **M, C, D** (IMU first, then colour), 1 s apart | **2 of 3** |
| **M, C, D**, back to back | **1 of 3** |
| **M, C, D**, open all then start all | **2 of 3** |
| default `rs2.pipeline` with IMU (starts M, C, D) | most runs; every run once hung |

Isolating the trigger with a raw libusb IMU reader (`scripts/diagnostics/imu_probe.c`), which keeps
the IMU running while something else starts the colour stream:

| Colour started by | Hung |
|---|---|
| kernel `uvcvideo` (ffmpeg on `/dev/video3`, plus one IR run on `/dev/video2`) | 0 of 7 |
| a librealsense process (RSUSB backend) | 2 of 5 |

So the hang needs the IMU running while **librealsense's RSUSB backend** brings up the colour
stream; colour streaming itself is not the problem. I did not identify which USB operation causes
it (the RSUSB backend claims the UVC interfaces, sends probe/commit control transfers and clears the
endpoint halt when it starts a stream).

Ruled out:

- **Delay between IMU power-on and the first read:** 0 ms to 15 s, no hang.
- **Kernel HID sensor drivers:** already blacklisted throughout.
- **librealsense version:** 2.58.4 (built in `~/librealsense-2.58`, not installed) fails the same way.
- **Firmware:** 5.17.3.10 and 5.17.0.10 fail the same way.

Results across library and firmware versions (failures / runs; "died" = colour and IMU hung):

| Test | FW 5.17.3.10 + lib 2.57.7 | FW 5.17.3.10 + lib 2.58.4 | FW 5.17.0.10 + lib 2.57.7 |
|---|---|---|---|
| Default pipeline (`rs-hello-realsense`) | every run | 5 / 5 | 5 / 5 |
| Pipeline with explicit depth + colour + IMU | failed | 5 / 5 | 5 / 5 |
| M, C, D one at a time | 5 / 10 died | 2 / 8 died | 2 / 8 died |
| M, C, D, open all then start | 2 / 3 died | 6 / 6 died | 5 / 6 died |

Not tested: a different Nano USB port or a powered USB 3 hub, and the V4L2 (kernel driver) backend,
which needs the L4T kernel patched with librealsense's script.

## Fix in this repo

`depthstudio.sources.realsense.RealSenseSource` no longer uses `rs2.pipeline`. It:

1. opens the depth and colour sensors directly into one `rs2.syncer`, and waits until frames from
   both have arrived;
2. then opens accel and gyro into an `rs2.frame_queue`, and waits until both deliver;
3. if any step times out, stops everything, hardware-resets the camera and retries (`retries=2`);
4. stops in reverse order (IMU first).

Depth is aligned to colour with `rs2.align` as before. `examples/rs_all.py` shows the same order
in plain pyrealsense2.

Verification on the Jetson (640x480 at 30 fps, accel 200 Hz, gyro 400 Hz):

| Test | Result |
|---|---|
| straight after a hardware reset | 8 / 8 started first time in about 1.5 s; 30 fps, about 590 Hz IMU, gravity 9.75 m/s² |
| back to back, no resets | 8 / 8 started first time |
| IMU already hung (default pipeline run first) | 2 / 2 recovered: one automatic reset, up in about 14 s |
| `examples/rs_all.py` straight after a reset | old one-pipeline version hung 2 / 4; new version 0 / 6 |
| `pytest` and `DEPTHSTUDIO_HW=1 pytest -m hardware` | all pass |

## Diagnostics

`scripts/diagnostics/` (build commands are in each file's header):

- `imu_probe.c`: reads the IMU over raw libusb and prints reports per 0.5 s. About 400 means
  healthy, 0 means hung. It tells you whether the IMU itself is hung, independent of librealsense.
  Twice, a librealsense session started right after the probe hung, so reset the camera after using it.
- `sensor_order_test.cpp`: starts the sensors in a given order and prints per-stream frame counts.
- `rs_reset.py`: hardware-resets the camera and waits for it to come back.

## Camera firmware

The camera was flashed from 5.17.3.10 to **5.17.0.10** (the version librealsense 2.57.7
recommends) on 2026-11-11. It did not change the behaviour, and the camera was left on 5.17.0.10.
On the Jetson, `~/rs-fw/` holds both official images (`D4XX_FW_Image-5.17.0.10.bin`,
`D4XX_FW_Image-5.17.3.10.bin`) and a flash backup taken before the downgrade. To go back:

```bash
rs-fw-update -s 141722070627 -f ~/rs-fw/D4XX_FW_Image-5.17.3.10.bin
```

(`rs-fw-update` 2.57.7 segfaults on exit after a successful update; check the version with
`rs-enumerate-devices -s`.)
