#!/usr/bin/env python3
"""
Reactive navigation from the OAK-D RGB preview only.

This does not use real stereo depth. It estimates a conservative pseudo-depth
from visual floor/obstacle contrast so the TurtleBot can be tested without
changing the internal OAK-D RGB/RGBD configuration.

Subscribes: /oakd/rgb/preview/image_raw (sensor_msgs/Image)
Publishes:  /cmd_vel                    (geometry_msgs/TwistStamped)
"""

import os
import socket
import sys
import threading
import time

import cv2
import numpy as np

if "--self-test" not in sys.argv:
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
else:
    rclpy = None
    CvBridge = None
    TwistStamped = None
    qos_profile_sensor_data = None
    Image = object
    Node = object


def estimate_visual_scan(
    bgr: np.ndarray,
    num_bins: int,
    roi_top_ratio: float,
    roi_bottom_ratio: float,
    floor_sample_height_ratio: float,
    floor_sample_width_ratio: float,
    color_threshold: float,
    edge_boost: float,
    row_occupancy: float,
    min_depth: float,
    max_depth: float,
):
    height, width = bgr.shape[:2]
    top = int(np.clip(height * roi_top_ratio, 0, height - 2))
    bottom = int(np.clip(height * roi_bottom_ratio, top + 1, height))
    roi = bgr[top:bottom]
    roi_h, roi_w = roi.shape[:2]

    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
    sample_h = max(4, int(roi_h * floor_sample_height_ratio))
    sample_w = max(4, int(roi_w * floor_sample_width_ratio))
    x0 = (roi_w - sample_w) // 2
    floor_sample = lab[roi_h - sample_h : roi_h, x0 : x0 + sample_w]
    floor_color = np.median(floor_sample.reshape(-1, 3), axis=0)

    color_dist = np.linalg.norm(lab - floor_color, axis=2)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 120)
    obstacle_score = color_dist + edge_boost * (edges > 0)
    obstacle_mask = (obstacle_score > color_threshold).astype(np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
    obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)

    bin_width = max(1, roi_w // num_bins)
    scan = np.full(num_bins, max_depth, dtype=np.float32)
    for i in range(num_bins):
        x_start = i * bin_width
        x_end = roi_w if i == num_bins - 1 else min(roi_w, (i + 1) * bin_width)
        col = obstacle_mask[:, x_start:x_end]
        occupied_rows = np.where(np.mean(col, axis=1) >= row_occupancy)[0]
        if occupied_rows.size == 0:
            continue
        closest_row = float(np.max(occupied_rows))
        closeness = closest_row / max(float(roi_h - 1), 1.0)
        scan[i] = max_depth - closeness * (max_depth - min_depth)

    return scan, obstacle_mask, (top, bottom)


class FollowTheGapRgbDepth(Node):
    def __init__(self):
        super().__init__("follow_the_gap_rgb_depth")

        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("number_of_bins", 40)
        self.declare_parameter("roi_top_ratio", 0.35)
        self.declare_parameter("roi_bottom_ratio", 0.95)
        self.declare_parameter("floor_sample_height_ratio", 0.18)
        self.declare_parameter("floor_sample_width_ratio", 0.50)
        self.declare_parameter("color_threshold", 32.0)
        self.declare_parameter("edge_boost", 28.0)
        self.declare_parameter("row_occupancy", 0.08)
        self.declare_parameter("min_depth", 0.35)
        self.declare_parameter("max_depth", 4.0)
        self.declare_parameter("front_stop_distance", 0.85)
        self.declare_parameter("gap_threshold", 1.20)
        self.declare_parameter("gap_min_bins", 5)
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("max_linear_speed", 0.08)
        self.declare_parameter("min_linear_speed", 0.02)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("show_debug", False)
        self.declare_parameter("telemetry_port", 6000)
        self.declare_parameter("telemetry_hz", 5.0)
        self.declare_parameter("send_scan_array", True)
        self.declare_parameter("scan_array_stride", 1)
        self.declare_parameter("robot_name", "turtlebot4_rensso_mora")
        self.declare_parameter("pairing_code", "ROBOT_A_2")

        g = lambda n: self.get_parameter(n).value
        self.image_topic = str(g("image_topic"))
        self.cmd_topic = str(g("cmd_topic"))
        self.num_bins = int(g("number_of_bins"))
        self.roi_top_ratio = float(g("roi_top_ratio"))
        self.roi_bottom_ratio = float(g("roi_bottom_ratio"))
        self.floor_sample_height_ratio = float(g("floor_sample_height_ratio"))
        self.floor_sample_width_ratio = float(g("floor_sample_width_ratio"))
        self.color_threshold = float(g("color_threshold"))
        self.edge_boost = float(g("edge_boost"))
        self.row_occupancy = float(g("row_occupancy"))
        self.min_depth = float(g("min_depth"))
        self.max_depth = float(g("max_depth"))
        self.front_stop = float(g("front_stop_distance"))
        self.gap_threshold = float(g("gap_threshold"))
        self.gap_min_bins = int(g("gap_min_bins"))
        self.kp = float(g("kp"))
        self.max_v = float(g("max_linear_speed"))
        self.min_v = float(g("min_linear_speed"))
        self.max_w = float(g("max_angular_speed"))
        self.show_debug = bool(g("show_debug"))
        if self.show_debug and not os.environ.get("DISPLAY"):
            self.get_logger().warn("show_debug=true but DISPLAY is not set; disabling OpenCV window.")
            self.show_debug = False
        self.telemetry_period = 1.0 / max(float(g("telemetry_hz")), 0.1)
        self.send_scan_array = bool(g("send_scan_array"))
        self.scan_array_stride = max(int(g("scan_array_stride")), 1)
        self.robot_name = str(g("robot_name"))
        self.pairing_code = str(g("pairing_code"))

        self.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "2"))
        self.authorized_addr = None
        self.last_telemetry = 0.0
        self.last_image_msg_time = None
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", int(g("telemetry_port"))))
            self.sock.settimeout(0.2)
            self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
            self.udp_thread.start()
            self.get_logger().info(f"UDP Telemetry listening on port {int(g('telemetry_port'))}")
        except Exception as e:
            self.get_logger().error(f"Failed to bind UDP socket: {e}")

        self.bridge = CvBridge()
        self.pub_cmd = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.sub_image = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.status_timer = self.create_timer(5.0, self._status_check)

        self.get_logger().info("Follow-the-Gap RGB pseudo-depth node initialized.")
        self.get_logger().info(f"Subscribed to RGB image: {self.image_topic}")
        self.get_logger().warn("Using pseudo-depth from RGB only; move slowly and supervise the robot.")

    def image_callback(self, msg: Image):
        self.last_image_msg_time = time.monotonic()
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")
            self.publish_cmd(0.0, 0.0)
            return

        scan, obstacle_mask, roi_bounds = estimate_visual_scan(
            bgr,
            self.num_bins,
            self.roi_top_ratio,
            self.roi_bottom_ratio,
            self.floor_sample_height_ratio,
            self.floor_sample_width_ratio,
            self.color_threshold,
            self.edge_boost,
            self.row_occupancy,
            self.min_depth,
            self.max_depth,
        )

        best_bin, gap = self.find_best_gap(scan)
        front_clear = self.front_clearance(scan)
        nearest_idx = int(np.argmin(scan))
        nearest_dist = float(scan[nearest_idx])
        nearest_angle = self.bin_to_angle_deg(nearest_idx)

        if front_clear < self.front_stop:
            speed, yaw, state = 0.0, self.escape_yaw(scan), "RGB_FRONT_BLOCKED"
        elif best_bin is None:
            speed, yaw, state = 0.0, self.escape_yaw(scan), "RGB_BLOCKED"
        else:
            target_angle = np.radians(self.bin_to_angle_deg(best_bin))
            yaw = float(np.clip(self.kp * target_angle, -self.max_w, self.max_w))
            dist_factor = np.clip(
                (float(scan[best_bin]) - self.front_stop) / max(self.max_depth - self.front_stop, 1e-3),
                0.0,
                1.0,
            )
            turn_factor = 1.0 - 0.5 * min(abs(yaw) / max(self.max_w, 1e-3), 1.0)
            speed = float(self.min_v + (self.max_v - self.min_v) * dist_factor * turn_factor)
            state = "RGB_FORWARD"

        self.publish_cmd(speed, yaw)
        self._send_state(msg, state, scan, front_clear, nearest_dist, nearest_angle, gap, speed, yaw)

        if self.show_debug:
            self.draw_debug(bgr, obstacle_mask, roi_bounds, scan, best_bin, speed, yaw, state)

    def find_best_gap(self, scan: np.ndarray):
        free = scan > self.gap_threshold
        best = None
        best_len = 0
        i = 0
        while i < free.size:
            if free[i]:
                start = i
                while i + 1 < free.size and free[i + 1]:
                    i += 1
                end = i
                length = end - start + 1
                if length >= self.gap_min_bins and length > best_len:
                    best = (start, end)
                    best_len = length
            i += 1
        if best is None:
            return None, None

        start, end = best
        center = (self.num_bins - 1) / 2.0
        candidates = range(start, end + 1)
        best_bin = max(candidates, key=lambda idx: float(scan[idx]) - 0.04 * abs(idx - center))
        return int(best_bin), best

    def front_clearance(self, scan: np.ndarray):
        mid = self.num_bins // 2
        span = max(1, self.num_bins // 10)
        return float(np.min(scan[mid - span : mid + span + 1]))

    def escape_yaw(self, scan: np.ndarray):
        left = float(np.mean(scan[: self.num_bins // 2]))
        right = float(np.mean(scan[self.num_bins // 2 :]))
        return float(np.clip(0.45 if left > right else -0.45, -self.max_w, self.max_w))

    def bin_to_angle_deg(self, idx: int):
        center = (self.num_bins - 1) / 2.0
        hfov_deg = 69.0
        return float((center - idx) * (hfov_deg / self.num_bins))

    def publish_cmd(self, speed: float, yaw: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(speed)
        msg.twist.angular.z = float(yaw)
        self.pub_cmd.publish(msg)

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
            self.sock.sendto(f"ACK {self.ros_domain_id} {self.robot_name}".encode("utf-8"), addr)
            self.get_logger().info(f"RGB pseudo-depth telemetry paired with {addr}.")
            self._send_log("WARN", "RGB pseudo-depth is approximate, not real metric depth.")

    def _send_state(self, msg, state, scan, front_clear, nearest_dist, nearest_angle, gap, speed, yaw):
        if self.authorized_addr is None:
            return
        now = time.monotonic()
        if now - self.last_telemetry < self.telemetry_period:
            return
        self.last_telemetry = now

        left_clear = float(np.min(scan[: self.num_bins // 3]))
        center_clear = float(np.min(scan[self.num_bins // 3 : 2 * self.num_bins // 3]))
        right_clear = float(np.min(scan[2 * self.num_bins // 3 :]))
        gap_start = "nan" if gap is None else f"{self.bin_to_angle_deg(gap[0]):.1f}"
        gap_end = "nan" if gap is None else f"{self.bin_to_angle_deg(gap[1]):.1f}"

        fields = [
            "LIDAR",
            str(self.ros_domain_id),
            self.robot_name,
            str(msg.header.stamp.sec),
            str(msg.header.stamp.nanosec),
            state,
            f"{front_clear:.3f}",
            f"{left_clear:.3f}",
            f"{right_clear:.3f}",
            f"{nearest_dist:.3f}",
            f"{nearest_angle:.1f}",
            gap_start,
            gap_end,
            f"{np.degrees(yaw):.1f}",
            f"{speed:.3f}",
            f"{yaw:.3f}",
        ]
        try:
            self.sock.sendto(" ".join(fields).encode("utf-8"), self.authorized_addr)
            if self.send_scan_array:
                self._send_scan_array(msg, scan)
        except OSError as e:
            self.get_logger().warn(f"Error sending telemetry: {e}")

    def _send_scan_array(self, msg, scan):
        ranges = scan[::-1][:: self.scan_array_stride]
        fields = [
            "SCAN_ARRAY",
            str(self.ros_domain_id),
            self.robot_name,
            str(msg.header.stamp.sec),
            str(msg.header.stamp.nanosec),
            f"{np.radians(-34.5):.6f}",
            f"{np.radians(69.0 / self.num_bins):.6f}",
            str(self.scan_array_stride),
            str(len(ranges)),
        ]
        fields.extend(f"{float(value):.3f}" for value in ranges)
        self.sock.sendto(" ".join(fields).encode("utf-8"), self.authorized_addr)

    def _send_log(self, level: str, message: str):
        if self.authorized_addr is None:
            return
        fields = ["LOG", str(self.ros_domain_id), self.robot_name, level, message]
        try:
            self.sock.sendto(" ".join(fields).encode("utf-8"), self.authorized_addr)
        except OSError as e:
            self.get_logger().warn(f"Error sending UDP log: {e}")

    def _status_check(self):
        if self.last_image_msg_time is None:
            publishers = len(self.get_publishers_info_by_topic(self.image_topic))
            msg = f"No RGB frames received on {self.image_topic}; image_publishers={publishers}"
            self.get_logger().warn(msg)
            self._send_log("WARN", msg)
        if self.pub_cmd.get_subscription_count() == 0:
            msg = f"No subscribers detected on {self.cmd_topic}; Create 3 will not receive velocity commands."
            self.get_logger().warn(msg)
            self._send_log("WARN", msg)

    def draw_debug(self, bgr, obstacle_mask, roi_bounds, scan, best_bin, speed, yaw, state):
        top, bottom = roi_bounds
        vis = bgr.copy()
        mask_color = cv2.cvtColor(obstacle_mask * 255, cv2.COLOR_GRAY2BGR)
        vis[top:bottom] = cv2.addWeighted(vis[top:bottom], 0.65, mask_color, 0.35, 0)
        h, w = vis.shape[:2]
        bin_w = max(1, w // self.num_bins)
        for i, distance in enumerate(scan):
            x = i * bin_w + bin_w // 2
            y = int(bottom - (distance / self.max_depth) * max(bottom - top, 1))
            cv2.circle(vis, (x, y), 3, (255, 255, 255), -1)
        if best_bin is not None:
            x = best_bin * bin_w + bin_w // 2
            cv2.line(vis, (x, top), (x, bottom), (0, 255, 255), 2)
        cv2.putText(vis, f"{state} v={speed:.2f} w={yaw:+.2f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("RGB pseudo-depth follow-the-gap", vis)
        cv2.waitKey(1)

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def _self_test():
    img = np.full((120, 160, 3), (80, 80, 80), dtype=np.uint8)
    cv2.rectangle(img, (68, 45), (92, 118), (210, 210, 210), -1)
    scan, _, _ = estimate_visual_scan(img, 20, 0.25, 1.0, 0.15, 0.5, 30.0, 28.0, 0.08, 0.35, 4.0)
    assert float(np.min(scan[8:12])) < 1.0
    assert float(np.mean(scan[:4])) > 3.0


def main(args=None):
    if "--self-test" in sys.argv:
        _self_test()
        print("self-test ok")
        return
    rclpy.init(args=args)
    node = FollowTheGapRgbDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
