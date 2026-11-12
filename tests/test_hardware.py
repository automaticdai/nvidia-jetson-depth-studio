"""Tests against real cameras. Run on the Jetson with:  DEPTHSTUDIO_HW=1 python3.8 -m pytest -m hardware"""

import os

import numpy as np
import pytest

from depthstudio.sources import open_source

pytestmark = [pytest.mark.hardware,
              pytest.mark.skipif(os.environ.get("DEPTHSTUDIO_HW") != "1", reason="set DEPTHSTUDIO_HW=1")]


def test_realsense_depth_color_imu():
    with open_source("realsense") as src:
        frames = list(src.frames(duration=3.0))
    assert len(frames) >= 60  # ~30 fps
    f = frames[-1]
    assert f.depth.shape == f.color.shape[:2] == (480, 640)
    assert (f.depth > 0).mean() > 0.3
    accel = [s for fr in frames for s in fr.imu if s.kind == "accel"]
    gyro = [s for fr in frames for s in fr.imu if s.kind == "gyro"]
    assert len(accel) > 300 and len(gyro) > 600  # 200 Hz / 400 Hz over ~3 s
    g = np.linalg.norm(np.mean([[s.x, s.y, s.z] for s in accel], axis=0))
    assert 8.5 < g < 11.0  # gravity, camera at rest


def test_realsense_after_hardware_reset():
    # Straight after a reset, starting the IMU before colour hung the camera about half the time
    # (docs/realsense-imu-hang.md); the video-first start order must work without a retry.
    from depthstudio.sources.realsense import RealSenseSource
    RealSenseSource()._hardware_reset()
    for _ in range(3):
        src = RealSenseSource()
        with src:
            frames = list(src.frames(duration=1.0))
        assert src.resets == 0
        assert len(frames) >= 20
        assert {s.kind for fr in frames for s in fr.imu} == {"accel", "gyro"}


def test_realsense_restart_cycles():
    # sessions with and without the IMU, back to back; every one must deliver
    for imu in (False, True, False, True):
        with open_source("realsense", imu=imu) as src:
            f = next(src.frames(count=1))
            assert f.depth is not None


def test_csi_camera():
    with open_source("csi:0") as src:
        frames = list(src.frames(count=10))
    assert frames[-1].color.shape == (720, 1280, 3)
    assert frames[-1].color.mean() > 1  # not a black frame
