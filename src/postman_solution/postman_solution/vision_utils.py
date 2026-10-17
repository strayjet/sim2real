"""
vision_utils.py - ROS-free overhead-camera perception for Tier 2 / Tier 3.

Camera model (from the world file + README):
  * camera at (CAM_X, CAM_Y, CAM_Z) looking straight down
  * image "up"    (decreasing v) = +X in the odom/arena frame
  * image "right" (increasing u) = -Y in the odom/arena frame
"""
import math
import numpy as np
import cv2

CAM_X, CAM_Y, CAM_Z = 1.5, 1.5, 4.0       # README: camera pose
DISK_RADIUS = 0.15                        # README: waypoint radius
DISK_TOP_Z = 0.02                         # disk is 2 cm thick, top face seen by camera
DEFAULT_FOV = 1.2217                      # rad, horizontal (world file)


class CameraModel:
    def __init__(self, width=800, height=800):
        self.w, self.h = width, height
        self.fx = self.fy = (width / 2.0) / math.tan(DEFAULT_FOV / 2.0)
        self.cx, self.cy = (width - 1) / 2.0, (height - 1) / 2.0

    def set_from_info(self, k, width, height):
        """k = CameraInfo.k (row-major 3x3)."""
        if k[0] > 1.0 and k[4] > 1.0:
            self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]
            self.w, self.h = width, height

    def pix_to_world(self, u, v, z=DISK_TOP_Z):
        depth = CAM_Z - z
        x = CAM_X + (self.cy - v) * depth / self.fy
        y = CAM_Y + (self.cx - u) * depth / self.fx
        return x, y

    def world_to_pix(self, x, y, z=DISK_TOP_Z):
        depth = CAM_Z - z
        v = self.cy - (x - CAM_X) * self.fy / depth
        u = self.cx - (y - CAM_Y) * self.fx / depth
        return u, v

    def metres_per_pixel(self, z=DISK_TOP_Z):
        return (CAM_Z - z) / self.fx


def image_to_rgb(msg):
    """sensor_msgs/Image -> HxWx3 uint8 RGB numpy array (no cv_bridge needed)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding.lower() == 'bgr8':
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def _blobs(mask, cam, z, expect_area_m2, min_frac, max_frac, open_px):
    """Return world-frame centres of blobs that look like complete round disks."""
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_px, open_px))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)      # kills thin lines (axis lines)
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    s = cam.metres_per_pixel(z)
    expect_px = expect_area_m2 / (s * s)
    out = []
    for i in range(1, n):
        x0, y0, w, h, area = stats[i]
        if not (min_frac * expect_px <= area <= max_frac * expect_px):
            continue                                       # partly hidden / wrong size
        if abs(w - h) > 0.25 * max(w, h):
            continue                                       # not round
        if not (0.60 <= area / float(w * h) <= 0.92):
            continue
        out.append(cam.pix_to_world(cents[i][0], cents[i][1], z))
    return out


def detect_green_disks(rgb, cam):
    """World (x, y) of every fully visible green waypoint disk."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (35, 80, 40), (85, 255, 255))
    return _blobs(mask, cam, DISK_TOP_Z, math.pi * DISK_RADIUS ** 2, 0.70, 1.35, 9)


def detect_home_marker(rgb, cam):
    """Centre of the red 0.30 m start square (sanity check of the pixel->world mapping)."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    m1 = cv2.inRange(hsv, (0, 120, 60), (8, 255, 255))
    m2 = cv2.inRange(hsv, (172, 120, 60), (180, 255, 255))
    mask = cv2.bitwise_or(m1, m2)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return cam.pix_to_world(cents[i][0], cents[i][1], 0.001)


# ----------------------------------------------------------------------------
# Robot localisation from the overhead camera (absolute pose reference)
# ----------------------------------------------------------------------------
ROBOT_TOP_Z = 0.06            # height of the robot's top face (lid sits at ~0.054 m)
BODY_LEN, BODY_WID = 0.214, 0.14   # body footprint seen from above (URDF)
BODY_CENTER_X = 0.096         # body centre is this far ahead of base_link (the axle)


def detect_robot(rgb, cam, pred_xy, radius=0.6):
    """Find the blue-grey robot body near the predicted position.

    Returns dict(cx, cy, axis, aspect, area_ratio) with (cx, cy) the world position of the
    body centre and `axis` the world angle of its long side (modulo pi), or (None, reason).
    """
    s = cam.metres_per_pixel(ROBOT_TOP_Z)
    u0, v0 = cam.world_to_pix(pred_xy[0], pred_xy[1], ROBOT_TOP_Z)
    r = int(radius / s)
    H, W = rgb.shape[:2]
    x0, x1 = max(int(u0) - r, 0), min(int(u0) + r, W)
    y0, y1 = max(int(v0) - r, 0), min(int(v0) + r, H)
    if x1 - x0 < 20 or y1 - y0 < 20:
        return None, 'window outside image'
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb[y0:y1, x0:x1]), cv2.COLOR_RGB2HSV)
    # robot body is a low-saturation BLUE-grey; floor/shadows are neutral, markings are vivid
    mask = cv2.inRange(hsv, (90, 25, 30), (125, 130, 235))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n < 2:
        return None, 'no blue-grey blob'
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = float(stats[i, cv2.CC_STAT_AREA])
    expect = BODY_LEN * BODY_WID / (s * s)
    ratio = area / expect
    if not (0.5 <= ratio <= 1.8):
        return None, f'blob area ratio {ratio:.2f}'
    ys, xs = np.nonzero(labels == i)
    pts = np.column_stack((xs + x0, ys + y0)).astype(np.float32)
    (rcx, rcy), _, _ = cv2.minAreaRect(pts)
    box = cv2.boxPoints(cv2.minAreaRect(pts))
    e0, e1 = box[1] - box[0], box[2] - box[1]
    long_e, short_e = (e0, e1) if np.hypot(*e0) >= np.hypot(*e1) else (e1, e0)
    aspect = float(np.hypot(*long_e) / max(np.hypot(*short_e), 1e-6))
    axis = math.atan2(-long_e[0], -long_e[1])          # pixel (du,dv) -> world angle
    cxw, cyw = cam.pix_to_world(rcx, rcy, ROBOT_TOP_Z)
    return dict(cx=cxw, cy=cyw, axis=axis, aspect=aspect, area_ratio=ratio), 'ok'
