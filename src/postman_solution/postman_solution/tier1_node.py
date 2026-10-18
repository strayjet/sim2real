#!/usr/bin/env python3
"""
tier1_node.py
Tier 1: waypoints are published on /waypoints and /dynamic_waypoints.

  * State estimation: wheel velocities (/joint_states) + gyro (/imu) -> /odom + TF odom->base_link
  * Navigation: visit all points in shortest order, dwell at each, insert dynamic ones, return home
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster

from postman_solution.nav_utils import Mission, clamp, V_MAX, W_MAX


class SolutionNode(Node):
    def __init__(self, subscribe_waypoints=True):
        super().__init__('solution_node')

        # Robot constants
        self.wheel_radius = 0.0225
        self.wheel_separation = 0.159

        # Pose estimate in the odom frame (robot starts at 0,0 facing +X)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v_est = 0.0
        self.w_est = 0.0
        self.gyro_z = 0.0
        self.have_imu = False
        self.odom_ready = False
        self.last_t = None
        self.last_pos = {}          # fallback if velocity field is empty

        self.mission = Mission()
        self.have_static = False

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 50)
        self.create_subscription(Imu, '/imu', self._on_imu, 50)
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        if subscribe_waypoints:
            # Coordinate topics are permitted in Tier 1 only. Tier 2/3 detect
            # target locations from overhead-camera images instead.
            self.create_subscription(PoseArray, '/waypoints', self._on_waypoints, qos_latched)
            self.create_subscription(PoseArray, '/dynamic_waypoints', self._on_dynamic_waypoints, qos_latched)

        self.create_timer(0.05, self._control_loop)   # 20 Hz
        self.get_logger().info('Solution node initialized.')

    # ------------------------------------------------------------ helpers
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------ state estimation
    def _on_imu(self, msg: Imu) -> None:
        self.gyro_z = msg.angular_velocity.z
        self.have_imu = True

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            il = msg.name.index('left_wheel_joint')
            ir = msg.name.index('right_wheel_joint')
        except ValueError:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = stamp if stamp > 0.0 else self._now()
        if self.last_t is None:
            self.last_t = t
            if len(msg.position) > max(il, ir):
                self.last_pos = {'l': msg.position[il], 'r': msg.position[ir]}
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.2:
            return

        # wheel angular velocities (rad/s)
        if len(msg.velocity) > max(il, ir):
            wl, wr = msg.velocity[il], msg.velocity[ir]
        elif len(msg.position) > max(il, ir):       # fallback: differentiate position
            pl, pr = msg.position[il], msg.position[ir]
            wl = (pl - self.last_pos.get('l', pl)) / dt
            wr = (pr - self.last_pos.get('r', pr)) / dt
            self.last_pos = {'l': pl, 'r': pr}
        else:
            return

        v = self.wheel_radius * (wr + wl) / 2.0
        w_enc = self.wheel_radius * (wr - wl) / self.wheel_separation

        if wl == 0.0 and wr == 0.0:
            w = 0.0                     # stopped: don't integrate gyro noise
        elif self.have_imu:
            w = 0.5 * self.gyro_z + 0.5 * w_enc    # blend gyro and encoders
        else:
            w = w_enc

        # integrate (midpoint)
        th_mid = self.th + 0.5 * w * dt
        self.x += v * math.cos(th_mid) * dt
        self.y += v * math.sin(th_mid) * dt
        self.th = math.atan2(math.sin(self.th + w * dt), math.cos(self.th + w * dt))
        self.v_est, self.w_est = v, w
        self.odom_ready = True

        self._publish_odom(msg.header.stamp if stamp > 0.0 else self.get_clock().now().to_msg())

    def _publish_odom(self, stamp) -> None:
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v_est
        odom.twist.twist.angular.z = self.w_est
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    # ------------------------------------------------------------ waypoints
    def _on_waypoints(self, msg: PoseArray) -> None:
        pts = [(p.position.x, p.position.y) for p in msg.poses]
        n = self.mission.add_points(pts)
        if pts:
            self.have_static = True
        if n:
            self.get_logger().info(f'Static waypoints received: {n} new -> {pts}')

    def _on_dynamic_waypoints(self, msg: PoseArray) -> None:
        pts = [(p.position.x, p.position.y) for p in msg.poses]
        n = self.mission.add_points(pts)
        if n:
            self.get_logger().info(f'Dynamic waypoints received: {n} new -> {pts}')

    # -------------------------------------------------------------- control
    def _control_loop(self) -> None:
        cmd = Twist()
        if self.odom_ready and self.have_static:
            prev = (self.mission.state, len(self.mission.visited))
            v, w = self.mission.update(self._now(), self.x, self.y, self.th)
            cmd.linear.x = clamp(v, 0.0, V_MAX)          # reverse is prohibited
            cmd.angular.z = clamp(w, -W_MAX, W_MAX)
            if (self.mission.state, len(self.mission.visited)) != prev:
                self.get_logger().info(
                    f'state={self.mission.state} visited={len(self.mission.visited)} '
                    f'remaining={len(self.mission.queue)} pose=({self.x:.2f},{self.y:.2f},{self.th:.2f})')
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = SolutionNode()
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
