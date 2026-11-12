"""Intel RealSense (D435i) source: depth + colour (aligned to colour) + IMU.

On the Jetson Nano (librealsense RSUSB backend), switching the IMU on before the colour stream is
running often hangs both: colour delivers nothing and the IMU goes silent until the camera is
hardware-reset. rs2.pipeline always starts the IMU first, so this source opens the sensors itself:
depth and colour first, then (once frames from both have arrived) the accelerometer and gyroscope.
If any stream still fails to deliver, start() resets the camera and retries. Background and
measurements: docs/realsense-imu-hang.md.
"""

import logging
import time
from typing import List, Optional

from ..frames import Frame, ImuSample, Intrinsics
from .base import Source, SourceError

log = logging.getLogger(__name__)


def _rs():
    try:
        import pyrealsense2 as rs
    except ImportError as e:
        raise SourceError("pyrealsense2 is not available for this Python "
                          "(on the Jetson use python3.8; see scripts/install_librealsense.sh)") from e
    return rs


def list_devices() -> List[dict]:
    rs = _rs()
    out = []
    for dev in rs.context().query_devices():
        def info(key):
            return dev.get_info(key) if dev.supports(key) else None
        out.append({
            "name": info(rs.camera_info.name),
            "serial": info(rs.camera_info.serial_number),
            "firmware": info(rs.camera_info.firmware_version),
            "usb": info(rs.camera_info.usb_type_descriptor),
            "imu": any(s.is_motion_sensor() for s in dev.query_sensors()),
        })
    return out


def _intrinsics(profile) -> Intrinsics:
    i = profile.as_video_stream_profile().get_intrinsics()
    return Intrinsics(width=i.width, height=i.height, fx=i.fx, fy=i.fy, cx=i.ppx, cy=i.ppy,
                      model=str(i.model).split(".")[-1], coeffs=list(i.coeffs))




def _find_profile(sensor, stream, fmt, width=None, height=None, fps=None):
    for p in sensor.get_stream_profiles():
        if p.stream_type() != stream or p.format() != fmt or (fps is not None and p.fps() != fps):
            continue
        if width is not None:
            v = p.as_video_stream_profile()
            if (v.width(), v.height()) != (width, height):
                continue
        return p
    return None


class RealSenseSource(Source):
    name = "realsense"

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30,
                 color: bool = True, depth: bool = True, imu: bool = True,
                 align_to_color: bool = True, serial: Optional[str] = None,
                 accel_hz: int = 200, gyro_hz: int = 400,
                 start_timeout: float = 5.0, reset_on_failure: bool = True, retries: int = 2):
        if not (color or depth or imu):
            raise ValueError("enable at least one of color, depth, imu")
        self.width, self.height, self.fps = width, height, fps
        self.color, self.depth, self.imu = color, depth, imu
        self.align_to_color = align_to_color and color and depth
        self.serial = serial
        self.accel_hz, self.gyro_hz = accel_hz, gyro_hz
        self.start_timeout = start_timeout
        self.reset_on_failure = reset_on_failure
        self.retries = retries
        self.resets = 0  # hardware resets performed by start() so far
        self._started = []  # sensors in start order
        self._sync = None  # syncer pairing depth and colour
        self._imu_queue = None  # frame_queue for accel/gyro (Python sensor callbacks can deadlock)
        self._align = None
        self._pending_imu: List[ImuSample] = []
        self._index = 0
        self._last_numbers = (None, None)  # (depth, color) frame numbers of the last emitted Frame
        self._depth_scale = 0.001
        self._color_intr = self._depth_intr = None
        self.device_info: dict = {}

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        attempts = 1 + (self.retries if self.reset_on_failure else 0)
        for attempt in range(attempts):
            try:
                self._start_and_check()
                return
            except SourceError as e:
                self.stop()
                if attempt == attempts - 1:
                    raise
                log.warning("%s; resetting the camera and retrying", e)
                self._hardware_reset()

    def stop(self) -> None:
        # IMU first, then video (reverse start order); stop everything before closing anything.
        for s in reversed(self._started):
            try:
                s.stop()
            except RuntimeError:
                pass
        for s in reversed(self._started):
            try:
                s.close()
            except RuntimeError:
                pass
        self._started = []
        self._sync = self._imu_queue = self._align = None
        self._pending_imu = []
        self._last_numbers = (None, None)

    def _find_device(self):
        rs = _rs()
        for dev in rs.context().query_devices():
            if not self.serial or dev.get_info(rs.camera_info.serial_number) == self.serial:
                return dev
        return None

    def _start_and_check(self) -> None:
        rs = _rs()
        dev = self._find_device()
        if dev is None:
            raise SourceError("no RealSense device found" + (" with serial %s" % self.serial if self.serial else ""))
        self.device_info = {
            "name": dev.get_info(rs.camera_info.name),
            "serial": dev.get_info(rs.camera_info.serial_number),
            "firmware": dev.get_info(rs.camera_info.firmware_version),
        }
        depth_s = color_s = motion_s = None
        for s in dev.query_sensors():
            if s.is_depth_sensor():
                depth_s = s.as_depth_sensor()
            elif s.is_motion_sensor():
                motion_s = s.as_motion_sensor()
            elif s.is_color_sensor():
                color_s = s.as_color_sensor()

        # 1) video first: depth and colour into one syncer
        self._sync = rs.syncer(2)
        if self.depth:
            p = _find_profile(depth_s, rs.stream.depth, rs.format.z16, self.width, self.height, self.fps)
            if p is None:
                raise ValueError("depth mode %dx%d@%d not supported" % (self.width, self.height, self.fps))
            depth_s.open(p)
            self._started.append(depth_s)
            depth_s.start(self._sync)
            self._depth_scale = depth_s.get_depth_scale()
            self._depth_intr = _intrinsics(p)
        if self.color:
            p = _find_profile(color_s, rs.stream.color, rs.format.bgr8, self.width, self.height, self.fps)
            if p is None:
                raise ValueError("colour mode %dx%d@%d not supported" % (self.width, self.height, self.fps))
            color_s.open(p)
            self._started.append(color_s)
            color_s.start(self._sync)
            self._color_intr = _intrinsics(p)
        if self.align_to_color:
            self._align = rs.align(rs.stream.color)
            self._depth_intr = self._color_intr

        deadline = time.monotonic() + self.start_timeout
        want = {k for k, on in (("depth", self.depth), ("color", self.color)) if on}
        seen = set()
        while want - seen:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SourceError("RealSense video did not start: missing %s" % sorted(want - seen))
            ok, f = self._sync.try_wait_for_frames(max(1, int(remaining * 1000)))
            if ok:
                df, cf = self._split(f)
                seen.update(k for k, fr in (("depth", df), ("color", cf)) if fr)

        # 2) IMU last, only once colour is actually streaming
        if self.imu:
            if motion_s is None:
                raise SourceError("this RealSense has no IMU")
            profiles = [_find_profile(motion_s, rs.stream.accel, rs.format.motion_xyz32f, fps=self.accel_hz),
                        _find_profile(motion_s, rs.stream.gyro, rs.format.motion_xyz32f, fps=self.gyro_hz)]
            if None in profiles:
                raise ValueError("IMU rates accel %d Hz / gyro %d Hz not supported" % (self.accel_hz, self.gyro_hz))
            self._imu_queue = rs.frame_queue(2000, keep_frames=True)
            motion_s.open(profiles)
            self._started.append(motion_s)
            motion_s.start(self._imu_queue)
            deadline = time.monotonic() + self.start_timeout
            while not {"accel", "gyro"} <= {s.kind for s in self._pending_imu}:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise SourceError("RealSense IMU did not start: got %d samples" % len(self._pending_imu))
                ok, f = self._imu_queue.try_wait_for_frame(max(1, int(min(remaining, 0.1) * 1000)))
                if ok:
                    self._add_imu(f.as_motion_frame(), rs)
        self._index = 0

    def _hardware_reset(self, wait: float = 15.0) -> None:
        rs = _rs()
        dev = self._find_device()
        if dev is None:
            raise SourceError("no RealSense device found")
        serial = dev.get_info(rs.camera_info.serial_number)
        dev.hardware_reset()
        self.resets += 1
        time.sleep(2.0)  # the device drops off the bus before it re-enumerates
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if any(d.get_info(rs.camera_info.serial_number) == serial for d in rs.context().query_devices()):
                time.sleep(1.0)
                return
            time.sleep(0.5)
        raise SourceError("RealSense %s did not come back after hardware reset" % serial)

    # ------------------------------------------------------------------ reading
    def read(self, timeout: float = 1.0) -> Optional[Frame]:
        if not self._started:
            raise SourceError("source not started")
        rs = _rs()
        deadline = time.monotonic() + timeout
        if not (self.depth or self.color):  # IMU only: wait for one sample, then take all queued
            ok, f = self._imu_queue.try_wait_for_frame(max(1, int(timeout * 1000)))
            if ok:
                self._add_imu(f.as_motion_frame(), rs)
            self._drain_imu(rs)
            return self._make_imu_frame() if self._pending_imu else None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            ok, f = self._sync.try_wait_for_frames(max(1, int(remaining * 1000)))
            if not ok:
                return None
            self._drain_imu(rs)
            frame = self._make_frame(f)
            if frame is not None:
                return frame

    def _drain_imu(self, rs) -> None:
        if self._imu_queue is None:
            return
        while True:
            f = self._imu_queue.poll_for_frame()
            if not f:
                return
            self._add_imu(f.as_motion_frame(), rs)

    def _add_imu(self, mf, rs) -> None:
        d = mf.get_motion_data()
        kind = "accel" if mf.get_profile().stream_type() == rs.stream.accel else "gyro"
        self._pending_imu.append(ImuSample(mf.get_timestamp() / 1000.0, kind, d.x, d.y, d.z))

    def _take_imu(self) -> List[ImuSample]:
        imu, self._pending_imu = self._pending_imu, []
        return imu

    def _make_imu_frame(self) -> Frame:
        imu = self._take_imu()
        frame = Frame(index=self._index, t=imu[-1].t, imu=imu)
        self._index += 1
        return frame

    @staticmethod
    def _split(f):
        """(depth_frame, color_frame) from a syncer output; either may be None."""
        rs = _rs()
        if f.is_frameset():
            fs = f.as_frameset()
            return fs.get_depth_frame() or None, fs.get_color_frame() or None
        st = f.get_profile().stream_type()
        return (f.as_depth_frame() if st == rs.stream.depth else None,
                f.as_video_frame() if st == rs.stream.color else None)

    def _make_frame(self, f) -> Optional[Frame]:
        """Build a Frame, or None if the set lacks a stream or repeats an already emitted image.

        The syncer emits partial sets (new depth + previous colour, then the same depth + new
        colour) while streams settle or the consumer lags; only pairs that are both new pass.
        """
        import numpy as np

        df, cf = self._split(f)
        numbers = (df.get_frame_number() if df else None, cf.get_frame_number() if cf else None)
        if (self.depth and (not df or numbers[0] == self._last_numbers[0])) or \
                (self.color and (not cf or numbers[1] == self._last_numbers[1])):
            return None
        self._last_numbers = numbers

        if self._align is not None:
            aligned = self._align.process(f.as_frameset())
            df, cf = aligned.get_depth_frame(), aligned.get_color_frame()
        depth = np.asanyarray(df.get_data()).copy() if self.depth else None
        color = np.asanyarray(cf.get_data()).copy() if self.color else None
        frame = Frame(index=self._index, t=f.get_timestamp() / 1000.0, color=color, depth=depth,
                      depth_scale=self._depth_scale, color_intrinsics=self._color_intr,
                      depth_intrinsics=self._depth_intr if depth is not None else None,
                      imu=self._take_imu())
        self._index += 1
        return frame

    def describe(self) -> dict:
        return {"source": self.name, "device": self.device_info, "width": self.width,
                "height": self.height, "fps": self.fps, "aligned_to_color": self.align_to_color,
                "accel_hz": self.accel_hz if self.imu else None,
                "gyro_hz": self.gyro_hz if self.imu else None}
