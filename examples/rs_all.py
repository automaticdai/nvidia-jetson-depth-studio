# Depth + colour + IMU on a D435i, started in the order that works on the Jetson Nano:
# video first, IMU last. A single rs.pipeline with the IMU enabled starts the IMU first and
# hangs colour + IMU about half the time (docs/realsense-imu-hang.md). Standalone pyrealsense2.
import time, numpy as np, pyrealsense2 as rs

def profile(sensor, stream, fmt, w=None, h=None, fps=None):
    for p in sensor.get_stream_profiles():
        if p.stream_type() == stream and p.format() == fmt and (fps is None or p.fps() == fps):
            v = p.as_video_stream_profile() if w else None
            if not w or (v.width(), v.height()) == (w, h):
                return p

dev = rs.context().query_devices()[0]
for s in dev.query_sensors():
    if s.is_depth_sensor(): depth = s.as_depth_sensor()
    elif s.is_motion_sensor(): motion = s.as_motion_sensor()
    elif s.is_color_sensor(): color = s.as_color_sensor()

# 1) depth + colour into one syncer, wait until both are actually streaming
sync = rs.syncer(2)
depth.open(profile(depth, rs.stream.depth, rs.format.z16, 640, 480, 30)); depth.start(sync)
color.open(profile(color, rs.stream.color, rs.format.bgr8, 640, 480, 30)); color.start(sync)
while True:
    fs = sync.wait_for_frames(5000)
    if fs.is_frameset() and fs.as_frameset().get_depth_frame() and fs.as_frameset().get_color_frame():
        break

# 2) IMU last, into a frame_queue (Python sensor callbacks can deadlock in this build)
imu_q = rs.frame_queue(5000, keep_frames=True)
motion.open([profile(motion, rs.stream.accel, rs.format.motion_xyz32f, fps=200),
             profile(motion, rs.stream.gyro, rs.format.motion_xyz32f, fps=400)])
motion.start(imu_q)

counts = {}; acc = []
align = rs.align(rs.stream.color)
t = time.time()
while time.time() - t < 5:
    ok, fs = sync.try_wait_for_frames(1000)
    if ok:
        for x in (fs.as_frameset() if fs.is_frameset() else [fs]):
            n = x.get_profile().stream_name(); counts[n] = counts.get(n, 0) + 1
    while True:
        f = imu_q.poll_for_frame()
        if not f:
            break
        n = f.get_profile().stream_name(); counts[n] = counts.get(n, 0) + 1
        if f.get_profile().stream_type() == rs.stream.accel:
            m = f.as_motion_frame().get_motion_data(); acc.append((m.x, m.y, m.z))
last = align.process(sync.wait_for_frames(1000).as_frameset())

for s in (motion, color, depth): s.stop()   # IMU first, reverse of start order
for s in (motion, color, depth): s.close()
print("frames in 5 s:", counts)
print("aligned depth %dx%d, centre %.3f m" % (last.get_depth_frame().get_width(), last.get_depth_frame().get_height(),
                                               last.get_depth_frame().get_distance(320, 240)))
if acc:
    a = np.mean(acc, 0); print("mean accel", np.round(a, 2), "|g| = %.2f m/s^2" % np.linalg.norm(a))
