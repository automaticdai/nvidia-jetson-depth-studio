#!/usr/bin/env python3.8
"""Hardware-reset the first (or given) RealSense and wait until it is back on the bus.

usage: python3.8 rs_reset.py [SERIAL]
"""
import sys
import time

import pyrealsense2 as rs


def devices(serial=None):
    return [d for d in rs.context().query_devices()
            if serial is None or d.get_info(rs.camera_info.serial_number) == serial]


serial = sys.argv[1] if len(sys.argv) > 1 else None
devs = devices(serial)
if not devs:
    sys.exit("no RealSense device found")
serial = devs[0].get_info(rs.camera_info.serial_number)
devs[0].hardware_reset()
time.sleep(3)
deadline = time.monotonic() + 15
while time.monotonic() < deadline:
    if devices(serial):
        time.sleep(1)
        print("reset done:", serial)
        sys.exit(0)
    time.sleep(0.5)
sys.exit("camera did not come back")
