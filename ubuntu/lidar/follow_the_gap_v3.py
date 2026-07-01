#!/usr/bin/env python3
"""
Follow-the-Gap (FGM) reactive corridor / single-path-maze driver for the
TurtleBot 4 (Create 3 base, RPLIDAR A1M8) -- ROS 2 JAZZY.

Subscribes:  /scan      (sensor_msgs/LaserScan)
Publishes:   /cmd_vel   (geometry_msgs/TwistStamped)   <-- Jazzy uses STAMPED

v3 changes vs v2:
  - Steering aims at the CENTER of the deepest "plateau" of the gap, not at the
    first max returned by argmax (which biased the heading toward one edge of an
    open corridor and made the robot drift into a wall).
  - gap_threshold default lowered 1.0 -> 0.5 so corners aren't mistaken for a
    dead end (which made it spin in place).
  - bubble_radius default 0.25 -> 0.30 for a bit more wall clearance.
  - Keeps a debug log line (front / steer / v / w) for tuning.

NOTE: this only improves the STEERING math. If /scan has more than one publisher
(two robots on the same ROS_DOMAIN_ID), the readings are corrupted and NO tuning
will fix the crashing -- isolate the robot's domain first. Check with:
    ros2 topic info /scan --verbose      # Publisher count must be 1

Run it:
    python3 follow_the_gap_v3.py
    python3 follow_the_gap_v3.py --ros-args -p gap_threshold:=0.6 -p max_speed:=0.15
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        # ---- Tunables (override with ros2 params / launch) ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('fov_deg', 180.0)            # frontal arc considered
        self.declare_parameter('forward_offset_deg', 0.0)   # 0 = lidar 0deg is robot-forward
        self.declare_parameter('max_range', 3.0)            # clip lidar (m) ~ corridor scale
        self.declare_parameter('bubble_radius', 0.30)       # safety bubble around nearest pt (m)
        self.declare_parameter('gap_threshold', 0.5)        # range above this = "free" (m)
        self.declare_parameter('plateau_tol', 0.05)         # depth tolerance for the plateau (m)
        self.declare_parameter('smoothing_window', 5)       # mean-filter width (beams)
        self.declare_parameter('max_speed', 0.22)           # m/s (Create 3 safe)
        self.declare_parameter('min_speed', 0.08)           # m/s on sharp turns
        self.declare_parameter('max_yaw', 1.5)              # rad/s cap
        self.declare_parameter('steer_gain', 1.2)           # steering-angle -> yaw gain
        self.declare_parameter('front_stop', 0.30)          # if wall this close ahead, crawl (m)
        self.declare_parameter('debug', True)               # print per-scan log line

        g = lambda n: self.get_parameter(n).value
        self.fov = np.radians(g('fov_deg'))
        self.fwd_off = np.radians(g('forward_offset_deg'))
        self.max_range = float(g('max_range'))
        self.bubble = float(g('bubble_radius'))
        self.gap_thr = float(g('gap_threshold'))
        self.plateau_tol = float(g('plateau_tol'))
        self.win = int(g('smoothing_window'))
        self.v_max = float(g('max_speed'))
        self.v_min = float(g('min_speed'))
        self.w_max = float(g('max_yaw'))
        self.k = float(g('steer_gain'))
        self.front_stop = float(g('front_stop'))
        self.debug = bool(g('debug'))

        self.pub = self.create_publisher(TwistStamped, g('cmd_topic'), 10)
        self.create_subscription(LaserScan, g('scan_topic'), self.cb,
                                 qos_profile_sensor_data)
        self.get_logger().info('Follow-the-Gap v3 running (plateau-center steering).')

    def cb(self, scan: LaserScan):
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        # clean invalid readings
        ranges = np.nan_to_num(ranges, nan=self.max_range,
                               posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)

        # keep only the frontal FOV (wrapped relative to forward direction)
        rel = np.arctan2(np.sin(angles - self.fwd_off), np.cos(angles - self.fwd_off))
        m = np.abs(rel) <= (self.fov / 2.0)
        r, a = ranges[m], rel[m]
        if r.size == 0:
            return

        # sort by angle so "contiguous" = angularly contiguous (handles wrap at array ends)
        order = np.argsort(a)
        a, r = a[order], r[order]

        # smooth to kill spurious one-beam gaps
        if self.win > 1 and r.size >= self.win:
            r = np.convolve(r, np.ones(self.win) / self.win, mode='same')

        # front clearance (for speed control), measured BEFORE bubbling
        front = np.abs(a) < np.radians(25.0)
        front_clear = float(np.min(r[front])) if np.any(front) else self.max_range

        # safety bubble: zero out an angular region around the nearest obstacle
        nearest = int(np.argmin(r))
        d = max(float(r[nearest]), 1e-3)
        half = float(np.arctan2(self.bubble, d))   # closer wall -> wider bubble
        r = r.copy()
        r[np.abs(a - a[nearest]) <= half] = 0.0

        # largest contiguous run of "free" beams
        s, e = self._largest_run(r > self.gap_thr)
        if s is None:
            # boxed in: rotate in place toward the more open side
            left = float(np.mean(r[a > 0])) if np.any(a > 0) else 0.0
            right = float(np.mean(r[a < 0])) if np.any(a < 0) else 0.0
            yaw = self.w_max if left >= right else -self.w_max
            if self.debug:
                self.get_logger().info(
                    f"front={front_clear:.2f}m  BOXED-IN  v=0.00 w={yaw:+.2f}")
            self._publish(0.0, yaw)
            return

        # --- plateau-center steering ---
        # Aim at the CENTER of the deepest band of the gap, not at the first max.
        gap_slice = r[s:e + 1]
        max_depth = float(np.max(gap_slice))
        plateau = np.where(gap_slice >= max_depth - self.plateau_tol)[0]
        center = int(plateau[len(plateau) // 2])
        best = s + center
        steer = float(a[best])

        # speed: ease off on sharp steering and when a wall is close ahead
        turn_factor = 1.0 - min(abs(steer) / (self.fov / 2.0), 1.0)
        speed = self.v_min + (self.v_max - self.v_min) * turn_factor
        if front_clear < self.front_stop:
            speed = self.v_min * 0.5

        yaw = float(np.clip(self.k * steer, -self.w_max, self.w_max))

        if self.debug:
            self.get_logger().info(
                f"front={front_clear:.2f}m  steer={np.degrees(steer):+.0f}deg  "
                f"v={speed:.2f} w={yaw:+.2f}")

        self._publish(speed, yaw)

    @staticmethod
    def _largest_run(mask: np.ndarray):
        """Return (start, end) inclusive indices of the longest True run, or (None, None)."""
        best_len, best = 0, (None, None)
        i, n = 0, mask.size
        while i < n:
            if mask[i]:
                j = i
                while j + 1 < n and mask[j + 1]:
                    j += 1
                if (j - i + 1) > best_len:
                    best_len, best = j - i + 1, (i, j)
                i = j + 1
            else:
                i += 1
        return best

    def _publish(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FollowTheGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # best-effort stop; skip if context already torn down by Ctrl+C
        try:
            if rclpy.ok():
                node._publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
