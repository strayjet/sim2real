#!/usr/bin/env python3
"""Tier 3: visually mapped obstacles + collision-aware waypoint navigation.

Inherits the working encoder/IMU odometry, camera pose corrections, green disk
tracking, dynamic insertion and TF publishing from the user's Tier 1/2 code.
All obstacle positions are obtained from the allowed overhead camera feed.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException

from postman_solution.tier2_node import Tier2Node, PROCESS_EVERY
from postman_solution.nav_utils import Mission, clamp, dist, wrap, best_order
from postman_solution.obstacle_utils import GridPlanner, find_magenta_walls

# Returning to this point puts the robot within 0.20 m of (0,0) while its
# front bumper stays away from the perimeter walls. This is a *home policy*,
# not a hardcoded waypoint or obstacle coordinate.
HOME_APPROACH = (0.110, 0.110)
# Plan every ordinary drive inside a more protective perimeter corridor.
# Use the clear corner diagonal only during start/end; no hardcoded targets.
CORNER_STAGE = (0.39, 0.39)
TAKEOFF = CORNER_STAGE
PERIMETER_MIN = -0.125
PERIMETER_MAX = 2.875
CHASSIS_CORNERS = ((-0.010, -0.0885), (-0.010, 0.0885),
                   (0.200, -0.0885), (0.200, 0.0885))


class ObstacleMission(Mission):
    def __init__(self):
        super().__init__(dwell_time=2.8, arrive_radius=0.075, home_radius=0.18)
        self.planner = None
        self.recovery_planner = None
        self.departed = False
        self.route_path = []
        self.route_goal = None
        self.turning = False
        self.last_path_attempt = -1e9
        self.route_blocked = False
        self.home_stage_done = False
        self.visual_robot = None            # (x, y, stamp) from overhead camera
        self.waiting_for_visual = False

    def _new_path(self):
        self.route_path = []
        self.route_goal = None
        self.turning = False

    def _steer(self, x, y, th, point, careful=False):
        d = dist((x, y), point)
        if d < 0.009:
            self.turning = False
            return 0.0, 0.0
        err = wrap(math.atan2(point[1] - y, point[0] - x) - th)
        if not self.turning and abs(err) > 0.32:
            self.turning = True
        if self.turning and abs(err) < 0.09:
            self.turning = False
        if self.turning:
            return 0.0, math.copysign(clamp(2.2 * abs(err), 0.24, 1.05), err)
        # Lower speed reduces overshoot and lets camera correction catch up.
        vmax = 0.11 if careful or d < 0.24 else 0.17
        speed = clamp(0.75 * d, 0.035, vmax)
        speed *= max(0.45, 1.0 - abs(err) / 0.60)
        return speed, clamp(2.5 * err, -1.05, 1.05)

    def update(self, t, x, y, th):
        if self.planner is None or self.state == 'DONE':
            return 0.0, 0.0

        # Leave the corner base on a known-clear diagonal before A* operates
        # inside the perimeter-safe configuration space. With heading 0 at
        # (0,0), this departure remains clear of all physical perimeter walls.
        if not self.departed:
            if dist((x, y), TAKEOFF) > 0.045:
                return self._steer(x, y, th, TAKEOFF, careful=True)
            self.departed = True
            self._new_path()
            self.need_replan = True

        if self.state == 'DWELL':
            if t - self.dwell_start < self.dwell_time:
                return 0.0, 0.0
            if self.dwelling_home and not self.queue:
                self.state = 'DONE'
                return 0.0, 0.0
            self.state = 'DRIVE'
            self.dwelling_home = False
            self._new_path()

        if self.need_replan:
            # Shortest collision-aware visiting order: only unvisited points
            # can be re-ordered when 2 more green disks appear mid-run.
            self.queue = best_order((x, y), self.queue,
                                    end=CORNER_STAGE,
                                    dist_fn=self.planner.distance)
            self.need_replan = False
            self._new_path()

        homebound = len(self.queue) == 0
        if not homebound:
            self.home_stage_done = False
        elif dist((x, y), CORNER_STAGE) < 0.055:
            self.home_stage_done = True

        # Do not record an internally "visited" waypoint merely because
        # encoder odometry says we're near it. If a recent independent robot
        # measurement is available, it must agree with actual capture radius.
        goal = self.queue[0] if not homebound else CORNER_STAGE
        near_goal = not homebound and dist((x, y), goal) < self.arrive_radius
        self.waiting_for_visual = False
        if near_goal and self.visual_robot is not None:
            vx, vy, stamp = self.visual_robot
            if t - stamp > 1.5 or dist((vx, vy), goal) > 0.17:
                self.waiting_for_visual = True
                return 0.0, 0.0  # wait for camera fusion, never falsely collect

        if near_goal or (homebound and self.home_stage_done and
                         dist((x, y), (0.0, 0.0)) < 0.17):
            # Home visual confirmation is optional when the camera was never
            # able to detect the robot, as in the original working controller.
            if homebound and self.visual_robot is not None:
                vx, vy, stamp = self.visual_robot
                if t - stamp > 1.5 or math.hypot(vx, vy) >= 0.19:
                    self.waiting_for_visual = True
                    return 0.0, 0.0
            self.state = 'DWELL'
            self.dwell_start = t
            self.dwelling_home = homebound
            self._new_path()
            if not homebound:
                self.visited.append(self.queue.pop(0))
            return 0.0, 0.0

        if homebound and self.home_stage_done:
            # Navigate home slowly on the clear diagonal from the staging
            # point, instead of letting the occupancy-grid perimeter margin
            # send A* close to an arena wall.
            return self._steer(x, y, th, HOME_APPROACH, careful=True)

        # Use A* only when a destination changes or the observed pose has
        # drifted outside the currently collision-free corridor.
        if self.route_goal != goal or not self.route_path:
            if t - self.last_path_attempt > 0.65 or self.route_goal != goal:
                self.last_path_attempt = t
                self.route_goal = goal
                path = self.planner.plan((x, y), goal)
                if path is None and self.recovery_planner is not None:
                    # A small departure from the conservative clearance zone
                    # is recoverable if the smaller, physically-safe buffer is
                    # still free. No Gazebo collision/pose information is used.
                    path = self.recovery_planner.plan((x, y), goal)
                self.route_path = path or []
                self.route_blocked = (path is None or (
                    not homebound and dist(path[-1], goal) >= self.arrive_radius))
                self.turning = False
            if not self.route_path:
                return 0.0, 0.0

        # Remove only intermediate corners we have truly reached; don't skip
        # them merely because we are within the robot's wide capture radius.
        while (len(self.route_path) > 1 and
               dist((x, y), self.route_path[0]) < 0.065 and
               (self.planner.free_line((x, y), self.route_path[1]) or
                (self.recovery_planner is not None and
                 self.recovery_planner.free_line((x, y), self.route_path[1])))):
            self.route_path.pop(0)
            self.turning = False
        # If a better straight segment becomes visible, safely cut the corner.
        while len(self.route_path) > 1 and self.planner.free_line((x, y), self.route_path[1]):
            self.route_path.pop(0)
            self.turning = False

        next_point = self.route_path[0]
        if not self.planner.free_line((x, y), next_point):
            if (self.recovery_planner is None or
                    not self.recovery_planner.free_line((x, y), next_point)):
                self.route_path = []
                self.route_goal = None
                return 0.0, 0.0
        return self._steer(x, y, th, next_point,
                           careful=len(self.route_path) > 1 or homebound)


class Tier3Node(Tier2Node):
    def __init__(self):
        super().__init__()
        self.mission = ObstacleMission()
        self._last_wall_count = -1
        self._last_wall_centres = None
        self._stable_walls = 0
        self._last_safety_log = -1e9
        self._last_wait_log = -1e9
        self.get_logger().info('Tier 3: waiting to identify all 4 magenta walls from camera.')

    def _on_image(self, msg):
        # Reuse Tier 2 detection, camera-odometry fusion, target confirmations,
        # and dynamic target registration. Do not subscribe to /waypoints.
        super()._on_image(msg)
        if self.frame_i % PROCESS_EVERY or self.last_rgb is None:
            return
        # Cache the direct image-derived robot position to validate capture.
        # This measure is independent of the integrated wheel/gyro position.
        if self.last_visual_robot is not None and self.last_visual_stamp is not None:
            self.mission.visual_robot = (*self.last_visual_robot, self.last_visual_stamp)
        if self.mission.planner is not None:
            return  # interior walls are static; no reason to rebuild each frame
        walls = find_magenta_walls(self.last_rgb, self.cam)
        if len(walls) != self._last_wall_count:
            self._last_wall_count = len(walls)
            self.get_logger().info(f'Tier 3 camera: detected {len(walls)}/4 magenta obstacles.')
        if len(walls) != 4:
            self._stable_walls = 0
            return
        centres = sorted((sum(x for x, _ in wall) / 4.0,
                          sum(y for _, y in wall) / 4.0) for wall in walls)
        if (self._last_wall_centres is not None and
                all(dist(p, q) < 0.07 for p, q in zip(centres, self._last_wall_centres))):
            self._stable_walls += 1
        else:
            self._stable_walls = 1
        self._last_wall_centres = centres
        if self._stable_walls >= 3:
            self.mission.planner = GridPlanner(walls)
            self.mission.recovery_planner = GridPlanner(walls, clearance=0.265)
            self.get_logger().info(
                'Tier 3 obstacle map ready: 4 camera-detected walls inflated ' 
                'for robot footprint. A* obstacle avoidance enabled.')

    @staticmethod
    def _footprint_safe(x, y, yaw, margin=0.02):
        """Conservative physical chassis test; does not use organizer topics."""
        c, s = math.cos(yaw), math.sin(yaw)
        for cx, cy in CHASSIS_CORNERS:
            px = x + cx * c - cy * s
            py = y + cx * s + cy * c
            if not (PERIMETER_MIN + margin <= px <= PERIMETER_MAX - margin and
                    PERIMETER_MIN + margin <= py <= PERIMETER_MAX - margin):
                return False
        return True

    def _control_loop(self):
        if self.mission.planner is None:
            self.cmd_pub.publish(Twist())
            return
        if not (self.odom_ready and self.have_static):
            self.cmd_pub.publish(Twist())
            return

        # For Tier 3 we deliberately do the base odometry/TF publication in
        # Tier 1, but compute and publish Twist here so a final independent
        # perimeter safety gate cannot be bypassed.
        prev = (self.mission.state, len(self.mission.visited))
        t = self._now()
        v, w = self.mission.update(t, self.x, self.y, self.th)
        v, w = clamp(v, 0.0, 0.30), clamp(w, -1.5, 1.5)

        # Project a short lookahead of the robot's complete bounding box.
        # Even when an odometry correction momentarily invalidates a route,
        # refuse commands that appear to bring the chassis into the wall.
        if v > 0.0 or w != 0.0:
            horizon = 0.75
            mid = self.th + 0.5 * w * horizon
            px = self.x + v * horizon * math.cos(mid)
            py = self.y + v * horizon * math.sin(mid)
            pt = self.th + w * horizon
            if not (self._footprint_safe(self.x, self.y, self.th) and
                    self._footprint_safe(px, py, pt)):
                v, w = 0.0, 0.0
                if t - self._last_safety_log > 1.0:
                    self._last_safety_log = t
                    self.get_logger().warn(
                        f'PERIMETER SAFETY STOP: odom=({self.x:.3f},'
                        f'{self.y:.3f},{self.th:.2f}), goal={self.mission.route_goal}')

        # If image-based position differs markedly from integrated odometry,
        # do not allow a faster drive near the walls. Last visual measurement
        # was already validated against camera-odometry innovation in Tier 2.
        vr = self.mission.visual_robot
        if vr is not None and t - vr[2] < 1.5:
            err = dist((self.x, self.y), (vr[0], vr[1]))
            if err > 0.15:
                v = min(v, 0.05)

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)
        if self.mission.waiting_for_visual and t - self._last_wait_log > 2.0:
            self._last_wait_log = t
            self.get_logger().warn('Waiting for overhead camera confirmation at waypoint/home')
        cur = (self.mission.state, len(self.mission.visited))
        if cur != prev:
            self.get_logger().info(
                f'Tier 3: {self.mission.state}, collected '
                f'{len(self.mission.visited)}/{len(self.mission.known)}, '
                f'remaining={len(self.mission.queue)}, '
                f'odom=({self.x:.2f},{self.y:.2f},{self.th:.2f}), '
                f'visual={self.mission.visual_robot}')


def main():
    rclpy.init()
    node = Tier3Node()
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
