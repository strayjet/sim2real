#!/usr/bin/env python3
"""
tier2_node.py
Tier 2: /waypoints and /dynamic_waypoints are suppressed.
Green waypoint disks are detected in /overhead_camera/image_raw and converted to
odom-frame coordinates. Everything else (odometry + TF, route planning, dwell,
dynamic insertion, return home) is inherited unchanged from the Tier 1 node.
"""
import math
import collections
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

from postman_solution.tier1_node import SolutionNode as Tier1Node
from postman_solution.vision_utils import (
    CameraModel, image_to_rgb, detect_green_disks, detect_home_marker,
    detect_robot, BODY_CENTER_X)

N_STATIC = 5            # task statement: 5 initial waypoints
PROCESS_EVERY = 3       # run detection on every 3rd frame (~10 Hz)
CONFIRM_FRAMES = 3      # a disk must be seen this many times in a row
MATCH_R = 0.06          # m, same-disk matching between frames
DEDUPE_R = 0.25         # m, waypoints are >= 0.5 m apart, so this is safe
START_TIMEOUT = 5.0     # s: start with what we have if fewer than 5 are found
DISK_EVERY = 3          # after start, scan for new disks on every 3rd processed frame
POS_GAIN = 0.45         # camera -> odometry position correction gain per frame
YAW_GAIN = 0.16         # camera -> odometry heading correction gain per frame
MAX_INNOV = 0.50        # m, reject camera pose that disagrees with odometry by more than this


class Tier2Node(Tier1Node):
    def __init__(self):
        super().__init__(subscribe_waypoints=False)
        self.cam = CameraModel()
        self.frame_i = 0
        self.pending = []        # [x, y, count]
        self.confirmed = []      # [(x, y)]
        self.first_img_t = None
        self.home_checked = False
        self.hist = collections.deque(maxlen=400)   # (t, x, y, th) odometry history
        self.disk_calls = 0
        self.fuse_ok = 0
        self.fuse_bad = 0
        self.last_innov = (0.0, 0.0, 0.0)
        self.last_reason = ''
        self.saved_dbg = False
        self.last_rgb = None
        # A separate direct camera measurement is useful for validating arrival.
        # It comes only from the allowed image topic, never Gazebo ground truth.
        self.last_visual_robot = None       # (axle_x, axle_y)
        self.last_visual_stamp = None
        self.create_timer(5.0, self._report_fusion)

        self.create_subscription(CameraInfo, '/overhead_camera/camera_info',
                                 self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/overhead_camera/image_raw',
                                 self._on_image, qos_profile_sensor_data)
        self.get_logger().info('Tier 2: waiting for camera image...')

    # ---------------------------------------------------- pose history for the camera
    def _publish_odom(self, stamp) -> None:
        super()._publish_odom(stamp)
        t = stamp.sec + stamp.nanosec * 1e-9
        self.hist.append((t, self.x, self.y, self.th))

    def _pose_at(self, t):
        """Odometry pose closest in time to t (camera frames arrive late)."""
        if not self.hist:
            return None
        best = min(self.hist, key=lambda e: abs(e[0] - t))
        if abs(best[0] - t) > 0.5:
            best = self.hist[-1]
        return best

    def _fuse_camera(self, rgb, stamp_s) -> None:
        if not self.odom_ready:
            return
        ref = self._pose_at(stamp_s)
        if ref is None:
            return
        _, xh, yh, thh = ref
        det, why = detect_robot(rgb, self.cam, (xh, yh))
        if det is None:
            self.fuse_bad += 1
            self.last_reason = why
            return
        # heading: long axis is known modulo 180 deg -> pick the one nearest to odometry
        dth, use_yaw = 0.0, False
        cand = det['axis']
        for c in (cand, cand + math.pi):
            d = math.atan2(math.sin(c - thh), math.cos(c - thh))
            if abs(d) < 0.5 and det['aspect'] > 1.25:
                dth, use_yaw = d, True
        th_use = thh + dth if use_yaw else thh
        mx = det['cx'] - BODY_CENTER_X * math.cos(th_use)     # body centre -> axle (base_link)
        my = det['cy'] - BODY_CENTER_X * math.sin(th_use)
        ix, iy = mx - xh, my - yh
        if math.hypot(ix, iy) > MAX_INNOV:
            self.fuse_bad += 1
            self.last_reason = f'innovation {math.hypot(ix, iy):.2f} m too large'
            return
        self.last_visual_robot = (mx, my)
        self.last_visual_stamp = stamp_s
        dx, dy = POS_GAIN * ix, POS_GAIN * iy
        dyaw = YAW_GAIN * dth if use_yaw else 0.0
        self.x += dx
        self.y += dy
        self.th = math.atan2(math.sin(self.th + dyaw), math.cos(self.th + dyaw))
        # shift history too, so the next frame does not re-apply the same correction
        self.hist = collections.deque(
            ((t, x + dx, y + dy, th + dyaw) for (t, x, y, th) in self.hist), maxlen=400)
        self.fuse_ok += 1
        self.last_innov = (ix, iy, math.degrees(dth) if use_yaw else float('nan'))

    def _report_fusion(self) -> None:
        if not self.odom_ready:
            return
        ix, iy, dth = self.last_innov
        self.get_logger().info(
            f'camera pose fusion: used={self.fuse_ok} skipped={self.fuse_bad} '
            f'last offset odom-vs-camera=({ix:+.3f},{iy:+.3f}) m, yaw {dth:+.1f} deg '
            f'(last skip reason: {self.last_reason or "-"})')
        if self.fuse_ok == 0 and self.fuse_bad > 20 and not self.saved_dbg and self.last_rgb is not None:
            self.saved_dbg = True
            cv2.imwrite('/tmp/overhead_debug.png', self.last_rgb[:, :, ::-1])
            self.get_logger().warn('Robot never detected - saved /tmp/overhead_debug.png')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.cam.set_from_info(msg.k, msg.width, msg.height)

    def _on_image(self, msg: Image) -> None:
        self.frame_i += 1
        if self.frame_i % PROCESS_EVERY:
            return
        if self.first_img_t is None:
            self.first_img_t = self._now()
        try:
            rgb = image_to_rgb(msg)
        except Exception as exc:                       # unexpected encoding etc.
            self.get_logger().warn(f'bad image: {exc}', throttle_duration_sec=5.0)
            return

        if not self.home_checked:
            self.home_checked = True
            h = detect_home_marker(rgb, self.cam)
            if h is not None:
                ok = math.hypot(h[0], h[1]) < 0.25
                self.get_logger().info(
                    f'Home marker seen at ({h[0]:.2f}, {h[1]:.2f}) m '
                    f'(should be ~0,0) -> pixel mapping {"OK" if ok else "WRONG!"}')

        self.last_rgb = rgb
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_s <= 0.0:
            stamp_s = self._now()
        self._fuse_camera(rgb, stamp_s)

        self.disk_calls += 1
        if (not self.have_static) or self.disk_calls % DISK_EVERY == 0:
            self._track(detect_green_disks(rgb, self.cam))

        # begin driving once the 5 static disks are known (or after a short timeout)
        if not self.have_static:
            waited = self._now() - self.first_img_t
            if len(self.confirmed) >= N_STATIC or (self.confirmed and waited > START_TIMEOUT):
                self.have_static = True
                self.get_logger().info(f'Starting mission with {len(self.confirmed)} visible waypoints.')

    def _track(self, dets) -> None:
        """Require a disk to be seen in several consecutive frames before trusting it."""
        new_pending = []
        for (x, y) in dets:
            if any(math.hypot(x - cx, y - cy) < DEDUPE_R for cx, cy in self.confirmed):
                continue
            for p in self.pending:
                if math.hypot(x - p[0], y - p[1]) < MATCH_R:
                    n = p[2] + 1
                    new_pending.append([(p[0] * p[2] + x) / n, (p[1] * p[2] + y) / n, n])
                    break
            else:
                new_pending.append([x, y, 1])
        self.pending = []
        for x, y, n in new_pending:
            if n >= CONFIRM_FRAMES:
                self.confirmed.append((x, y))
                added = self.mission.add_points([(x, y)])
                tag = 'DYNAMIC' if self.have_static else 'static'
                self.get_logger().info(
                    f'Detected {tag} waypoint #{len(self.confirmed)} at ({x:.3f}, {y:.3f}) '
                    f'(added={added})')
            else:
                self.pending.append([x, y, n])


def main():
    rclpy.init()
    node = Tier2Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
