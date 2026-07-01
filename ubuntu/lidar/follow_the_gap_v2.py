#!/usr/bin/env python3
"""
Follow-the-Gap (FGM) reactive corridor / single-path-maze driver for the
TurtleBot 4 (Create 3 base, RPLIDAR A1M8) -- ROS 2 JAZZY.

Subscribes:  /scan      (sensor_msgs/LaserScan)
Publishes:   /cmd_vel   (geometry_msgs/TwistStamped)   <-- Jazzy uses STAMPED

No map, no training, no memory. Each scan it finds the largest free "gap"
in the frontal field of view and steers toward the deepest point in it, so
it drives through ANY corridor layout it has never seen before.

Run it:
    python3 follow_the_gap.py
or, if your topics are namespaced:
    python3 follow_the_gap.py --ros-args -p scan_topic:=/ns/scan -p cmd_topic:=/ns/cmd_vel
"""

import os
import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class FollowTheGap(Node):
    def __init__(self):
        super().__init__("follow_the_gap")

        # ---- Tunables (override with ros2 params / launch) ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("fov_deg", 180.0)  # frontal arc considered
        self.declare_parameter(
            "forward_offset_deg", 180.0
        )  # set 180 if robot drives "backwards"
        self.declare_parameter("max_range", 3.0)  # clip lidar (m) ~ corridor scale
        self.declare_parameter(
            "bubble_radius", 0.35
        )  # safety bubble around nearest pt (m) (Turtlebot4 radius = 20cm, +15cm margin)
        self.declare_parameter("gap_threshold", 0.60)  # range above this = "free" (m) (must be > 40cm robot diameter)
        self.declare_parameter("plateau_tol", 0.05)  # depth tolerance for the plateau (m)
        self.declare_parameter("smoothing_window", 5)  # mean-filter width (beams)
        self.declare_parameter("max_speed", 0.10)  # m/s
        self.declare_parameter("min_speed", 0.03)  # m/s on sharp turns
        self.declare_parameter("max_yaw", 1.20)  # rad/s cap
        self.declare_parameter("steer_gain", 1.2)  # steering-angle -> yaw gain
        self.declare_parameter("front_deg", 12.0)
        self.declare_parameter(
            "front_stop", 0.55
        )  # if wall this close ahead, turn in place
        self.declare_parameter("front_slow", 0.90)
        self.declare_parameter("side_stop", 0.30)
        self.declare_parameter("side_gain", 0.35)
        self.declare_parameter("max_forward_yaw", 0.22)
        self.declare_parameter("rotate_yaw_threshold", 0.18)
        self.declare_parameter("escape_yaw", 0.90)
        self.declare_parameter("escape_deadband", 0.08)
        self.declare_parameter(
            "steer_sign", 1.0
        )  # set -1 if the robot turns the wrong way
        self.declare_parameter("telemetry_port", 6000)
        self.declare_parameter("telemetry_hz", 5.0)
        self.declare_parameter("send_scan_array", True)
        self.declare_parameter("scan_array_stride", 1)
        self.declare_parameter("robot_name", "turtlebot4_rensso_mora")
        self.declare_parameter("pairing_code", "ROBOT_A_2")

        g = lambda n: self.get_parameter(n).value
        self.fov = np.radians(g("fov_deg"))
        self.fwd_off = np.radians(g("forward_offset_deg"))
        self.max_range = float(g("max_range"))
        self.bubble = float(g("bubble_radius"))
        self.gap_thr = float(g("gap_threshold"))
        self.plateau_tol = float(g("plateau_tol"))
        self.win = int(g("smoothing_window"))
        self.v_max = float(g("max_speed"))
        self.v_min = float(g("min_speed"))
        self.w_max = float(g("max_yaw"))
        self.k = float(g("steer_gain"))
        self.front_angle = np.radians(g("front_deg"))
        self.front_stop = float(g("front_stop"))
        self.front_slow = float(g("front_slow"))
        self.side_stop = float(g("side_stop"))
        self.side_gain = float(g("side_gain"))
        self.max_forward_yaw = float(g("max_forward_yaw"))
        self.rotate_yaw_threshold = float(g("rotate_yaw_threshold"))
        self.escape_yaw = float(g("escape_yaw"))
        self.escape_deadband = float(g("escape_deadband"))
        self.steer_sign = float(g("steer_sign"))
        self.last_escape = 1.0
        self.telemetry_period = 1.0 / max(float(g("telemetry_hz")), 0.1)
        self.send_scan_array = bool(g("send_scan_array"))
        self.scan_array_stride = max(int(g("scan_array_stride")), 1)
        self.robot_name = str(g("robot_name"))
        self.pairing_code = str(g("pairing_code"))
        self.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "2"))
        self.authorized_addr = None
        self.last_telemetry = 0.0
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", int(g("telemetry_port"))))
        self.sock.settimeout(0.2)
        self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.udp_thread.start()

        self.pub = self.create_publisher(TwistStamped, g("cmd_topic"), 10)
        self.create_subscription(
            LaserScan, g("scan_topic"), self.cb, qos_profile_sensor_data
        )
        self.get_logger().info(
            "Follow-the-Gap running. Steering toward the largest free gap."
        )
        self.get_logger().info(
            f"LIDAR telemetry waiting for HELLO on UDP port {int(g('telemetry_port'))}."
        )

    def cb(self, scan: LaserScan):
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        # clean invalid readings
        ranges = np.nan_to_num(
            ranges, nan=self.max_range, posinf=self.max_range, neginf=0.0
        )
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
            r = np.convolve(r, np.ones(self.win) / self.win, mode="same")

        # front clearance (for speed control), measured BEFORE bubbling
        front = np.abs(a) < self.front_angle
        front_clear = float(np.min(r[front])) if np.any(front) else self.max_range
        left_clear = self._sector_min(a, r, 20.0, 85.0)
        right_clear = self._sector_min(a, r, -85.0, -20.0)
        nearest_angle = float(np.degrees(a[int(np.argmin(r))]))

        # safety bubble: zero out an angular region around the nearest obstacle
        nearest = int(np.argmin(r))
        d = max(float(r[nearest]), 1e-3)
        half = float(np.arctan2(self.bubble, d))  # closer wall -> wider bubble
        r = r.copy()
        r[np.abs(a - a[nearest]) <= half] = 0.0

        # largest contiguous run of "free" beams
        s, e = self._largest_run(r > self.gap_thr)
        if s is None:
            # boxed in: rotate in place toward the more open side
            yaw = self._escape_yaw(left_clear, right_clear)
            self._send_lidar_state(
                scan,
                "BOXED",
                front_clear,
                left_clear,
                right_clear,
                d,
                nearest_angle,
                None,
                None,
                float(np.degrees(yaw)),
                0.0,
                yaw,
            )
            self._publish(0.0, yaw)
            return

        gap_start = float(np.degrees(a[s]))
        gap_end = float(np.degrees(a[e]))

        # --- Direction to the center of the deepest plateau (Plateau-Center) ---
        gap_slice = r[s:e + 1]
        max_depth = float(np.max(gap_slice))
        plateau = np.where(gap_slice >= max_depth - self.plateau_tol)[0]
        center = int(plateau[len(plateau) // 2])
        best = s + center
        steer = float(a[best])

        # Compute raw yaw command using steer gain
        target_yaw = float(self.steer_sign * self.k * steer)
        yaw = float(np.clip(target_yaw, -self.max_forward_yaw, self.max_forward_yaw))

        state = "FORWARD"
        if front_clear < self.front_stop:
            # Front blocked: rotate in place towards the gap
            yaw = float(np.sign(target_yaw) * min(self.escape_yaw, self.w_max))
            speed = 0.0
            state = "FRONT_BLOCKED"
        elif abs(target_yaw) >= self.rotate_yaw_threshold:
            # Angle too sharp: rotate in place towards the gap
            yaw = float(np.sign(target_yaw) * min(self.escape_yaw, self.w_max))
            speed = 0.0
            state = "REALIGN"
        else:
            denom = max(self.front_slow - self.front_stop, 1e-3)
            front_factor = float(
                np.clip((front_clear - self.front_stop) / denom, 0.0, 1.0)
            )
            
            # Dynamic velocity scaling when close to side walls
            side_clear = min(left_clear, right_clear)
            side_factor = 1.0
            if side_clear < self.side_stop:
                side_factor = max(0.1, side_clear / self.side_stop)
                state = "SIDE_SLOW"
            
            turn_factor = 1.0 - 0.5 * min(
                abs(yaw) / max(self.max_forward_yaw, 1e-3), 1.0
            )
            speed = (self.v_min + (self.v_max - self.v_min) * front_factor * turn_factor) * side_factor

        self._send_lidar_state(
            scan,
            state,
            front_clear,
            left_clear,
            right_clear,
            d,
            nearest_angle,
            gap_start,
            gap_end,
            float(np.degrees(steer)),
            speed,
            yaw,
        )
        self._publish(speed, yaw)

    def _udp_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            parts = data.decode("utf-8", errors="ignore").strip().split()
            if len(parts) < 3 or parts[0] != "HELLO":
                continue
            try:
                desired_domain = int(parts[1])
            except ValueError:
                continue
            if desired_domain != self.ros_domain_id or parts[2] != self.pairing_code:
                continue
            self.authorized_addr = addr
            ack = f"ACK {self.ros_domain_id} {self.robot_name}".encode("utf-8")
            self.sock.sendto(ack, addr)
            self.get_logger().info(f"LIDAR telemetry paired with {addr}.")

    @staticmethod
    def _sector_min(a, r, start_deg, end_deg):
        m = (a >= np.radians(start_deg)) & (a <= np.radians(end_deg))
        return float(np.min(r[m])) if np.any(m) else float("nan")

    def _escape_yaw(self, left_clear, right_clear):
        if abs(left_clear - right_clear) > self.escape_deadband:
            self.last_escape = 1.0 if left_clear > right_clear else -1.0
        return float(
            np.clip(
                self.steer_sign * self.last_escape * self.escape_yaw,
                -self.w_max,
                self.w_max,
            )
        )

    def _send_lidar_state(
        self,
        scan,
        state,
        front_clear,
        left_clear,
        right_clear,
        nearest_dist,
        nearest_angle,
        gap_start,
        gap_end,
        target_angle,
        speed,
        yaw,
    ):
        if self.authorized_addr is None:
            return
        now = time.monotonic()
        if now - self.last_telemetry < self.telemetry_period:
            return
        self.last_telemetry = now
        fields = [
            "LIDAR",
            str(self.ros_domain_id),
            self.robot_name,
            str(scan.header.stamp.sec),
            str(scan.header.stamp.nanosec),
            state,
            f"{front_clear:.3f}",
            f"{left_clear:.3f}",
            f"{right_clear:.3f}",
            f"{nearest_dist:.3f}",
            f"{nearest_angle:.1f}",
            "nan" if gap_start is None else f"{gap_start:.1f}",
            "nan" if gap_end is None else f"{gap_end:.1f}",
            "nan" if target_angle is None else f"{target_angle:.1f}",
            f"{speed:.3f}",
            f"{yaw:.3f}",
        ]
        try:
            self.sock.sendto(" ".join(fields).encode("utf-8"), self.authorized_addr)
            if self.send_scan_array:
                self._send_scan_array(scan)
        except OSError as e:
            self.get_logger().warn(f"Error sending LIDAR telemetry: {e}")

    def _send_scan_array(self, scan):
        ranges = scan.ranges[:: self.scan_array_stride]
        fields = [
            "SCAN_ARRAY",
            str(self.ros_domain_id),
            self.robot_name,
            str(scan.header.stamp.sec),
            str(scan.header.stamp.nanosec),
            f"{scan.angle_min:.6f}",
            f"{scan.angle_increment:.6f}",
            str(self.scan_array_stride),
            str(len(ranges)),
        ]
        fields.extend(self._range_text(r) for r in ranges)
        self.sock.sendto(" ".join(fields).encode("utf-8"), self.authorized_addr)

    @staticmethod
    def _range_text(value):
        value = float(value)
        if np.isposinf(value):
            return "inf"
        if np.isneginf(value):
            return "-inf"
        if np.isnan(value):
            return "nan"
        return f"{value:.3f}"

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
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


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


if __name__ == "__main__":
    main()
