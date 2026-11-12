# Raw pyrealsense2 check: device info, aligned depth+colour, point cloud, then the IMU in a
# separate IMU-only session (safe: the hang needs the IMU running while colour starts, see
# docs/realsense-imu-hang.md). rs_all.py runs video and IMU together, IMU started last.
import time, numpy as np, pyrealsense2 as rs, cv2
print("pyrealsense2", getattr(rs, "__version__", None) or rs.pyrealsense2.__version__)
ctx = rs.context()
for d in ctx.query_devices():
    print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number),
          "fw", d.get_info(rs.camera_info.firmware_version), "usb", d.get_info(rs.camera_info.usb_type_descriptor))

# depth + colour, aligned
p = rs.pipeline(); c = rs.config()
c.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
c.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
prof = p.start(c)
scale = prof.get_device().first_depth_sensor().get_depth_scale()
align = rs.align(rs.stream.color)
for _ in range(30): p.wait_for_frames()
t = time.time(); n = 0
while time.time() - t < 3:
    fs = align.process(p.wait_for_frames()); n += 1
print("depth+color aligned fps: %.1f" % (n / (time.time() - t)))
d = np.asanyarray(fs.get_depth_frame().get_data()); col = np.asanyarray(fs.get_color_frame().get_data())
print("depth scale", scale, "| centre distance %.3f m" % fs.get_depth_frame().get_distance(320, 240),
      "| valid %.1f%%" % (100 * (d > 0).mean()))
pc = rs.pointcloud(); pc.map_to(fs.get_color_frame()); pts = pc.calculate(fs.get_depth_frame())
v = np.asanyarray(pts.get_vertices()).view(np.float32).reshape(-1, 3); v = v[v[:, 2] > 0]
print("pointcloud points:", len(v))
vis = cv2.applyColorMap(cv2.convertScaleAbs(d, alpha=255 / 5000.0), cv2.COLORMAP_JET); vis[d == 0] = 0
cv2.imwrite("/tmp/rs_aligned.png", np.hstack([col, vis]))
p.stop()

# IMU
p = rs.pipeline(); c = rs.config()
c.enable_stream(rs.stream.accel); c.enable_stream(rs.stream.gyro)
p.start(c)
acc, gyr = [], []
t = time.time()
while time.time() - t < 2:
    for f in p.wait_for_frames():
        m = f.as_motion_frame().get_motion_data(); xyz = [m.x, m.y, m.z]
        (acc if f.get_profile().stream_type() == rs.stream.accel else gyr).append(xyz)
p.stop()
a, g = np.mean(acc, 0), np.mean(gyr, 0)
print("accel samples %d, mean %s m/s^2, |g|=%.2f" % (len(acc), np.round(a, 2), np.linalg.norm(a)))
print("gyro  samples %d, mean %s rad/s" % (len(gyr), np.round(g, 3)))
