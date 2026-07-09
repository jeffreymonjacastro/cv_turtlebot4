#!/usr/bin/env python3
"""
Competition navigation from OAK-D camera depth/RGB plus LiDAR safety.

It uses OAK-D stereo depth when available. If depth is not publishing, it falls
back to conservative RGB pseudo-depth from visual floor/obstacle contrast.

Subscribes: /oakd/rgb/preview/image_raw (sensor_msgs/Image)
            /scan                       (sensor_msgs/LaserScan)
Publishes:  /cmd_vel                    (geometry_msgs/TwistStamped)

This variant uses a forward-first corridor controller instead of Follow-the-Gap:
camera depth/RGB estimates the front corridor, while LiDAR validates the
TurtleBot-sized safety envelope and whether a blocked-front turn can fit.
"""

from collections import deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
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
    from sensor_msgs.msg import Image, LaserScan
else:
    rclpy = None
    CvBridge = None
    TwistStamped = None
    qos_profile_sensor_data = None
    Image = object
    LaserScan = object
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
    bright_wall_l_threshold: float,
    bright_wall_delta_l: float,
    obstacle_dilate_iters: int,
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
    light_delta = lab[:, :, 0] - float(floor_color[0])
    bright_wall_mask = (lab[:, :, 0] >= bright_wall_l_threshold) & (light_delta >= bright_wall_delta_l)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 120)
    obstacle_score = color_dist + edge_boost * (edges > 0)
    obstacle_mask = ((obstacle_score > color_threshold) | bright_wall_mask).astype(np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
    obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)
    if obstacle_dilate_iters > 0:
        obstacle_mask = cv2.dilate(obstacle_mask, kernel, iterations=obstacle_dilate_iters)

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


def front_wall_occupied(
    obstacle_mask: np.ndarray,
    center_width_ratio: float,
    bottom_height_ratio: float,
    occupancy_threshold: float,
) -> bool:
    if obstacle_mask.size == 0:
        return False
    height, width = obstacle_mask.shape[:2]
    center_width = max(1, int(width * np.clip(center_width_ratio, 0.1, 1.0)))
    x0 = max(0, (width - center_width) // 2)
    y0 = int(height * (1.0 - np.clip(bottom_height_ratio, 0.1, 1.0)))
    center_bottom = obstacle_mask[y0:height, x0 : x0 + center_width]
    return float(np.mean(center_bottom)) >= float(occupancy_threshold)


def depth_image_to_scan(
    depth_m: np.ndarray,
    num_bins: int,
    roi_top_ratio: float,
    roi_bottom_ratio: float,
    min_depth: float,
    max_depth: float,
) -> np.ndarray:
    height, width = depth_m.shape[:2]
    top = int(np.clip(height * roi_top_ratio, 0, height - 2))
    bottom = int(np.clip(height * roi_bottom_ratio, top + 1, height))
    roi = depth_m[top:bottom].astype(np.float32)
    invalid = ~np.isfinite(roi) | (roi <= 0.0)
    roi = np.where(invalid, min_depth, roi)
    roi = np.clip(roi, min_depth, max_depth)

    bin_width = max(1, width // num_bins)
    scan = np.full(num_bins, max_depth, dtype=np.float32)
    for i in range(num_bins):
        x_start = i * bin_width
        x_end = width if i == num_bins - 1 else min(width, (i + 1) * bin_width)
        pixels = roi[:, x_start:x_end]
        if pixels.size:
            scan[i] = float(np.percentile(pixels, 20))
    return scan


def lidar_sector_min(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    start_deg: float,
    end_deg: float,
    forward_offset_rad: float = 0.0,
    percentile: float = 0.0,
) -> float:
    ranges = np.asarray(ranges, dtype=np.float32)
    if ranges.size == 0:
        return 0.0

    angles = angle_min + np.arange(ranges.size, dtype=np.float32) * angle_increment
    rel = np.arctan2(np.sin(angles - forward_offset_rad), np.cos(angles - forward_offset_rad))
    start = np.radians(start_deg)
    end = np.radians(end_deg)
    if start <= end:
        mask = (rel >= start) & (rel <= end)
    else:
        mask = (rel >= start) | (rel <= end)

    sector = ranges[mask]
    if sector.size == 0:
        return 0.0
    valid = sector[~np.isnan(sector)]
    if valid.size == 0:
        return 0.0

    range_min = float(range_min if range_min > 0.0 else 0.02)
    range_max = float(range_max if range_max > range_min else 12.0)
    too_close = valid < range_min
    too_far = (~too_close) & (np.isinf(valid) | (valid > range_max))
    processed = np.where(too_close, range_min, valid)
    processed = np.where(too_far, range_max, processed)
    if percentile > 0.0:
        return float(np.percentile(processed, np.clip(percentile, 0.0, 100.0)))
    return float(np.min(processed))


class StableSignalDebouncer:
    def __init__(self, window_size: int, confirm_count: int, cooldown_s: float):
        self.window = deque(maxlen=max(1, int(window_size)))
        self.confirm_count = max(1, int(confirm_count))
        self.cooldown_s = max(0.0, float(cooldown_s))
        self.cooldown_until = 0.0
        self.consumed = set()

    def update(self, signal: dict, now: float):
        if now < self.cooldown_until:
            self.window.append("none")
            return None
        direction = signal.get("direction", "none")
        if direction not in ("left", "right", "stop", "meta"):
            self.window.append("none")
            return None
        event_id = signal.get("event_id") or f"{direction}:{signal.get('timestamp', 0.0)}"
        if event_id in self.consumed:
            self.window.append("none")
            return None
        self.window.append(direction)
        if sum(1 for item in self.window if item == direction) < self.confirm_count:
            return None
        self.consumed.add(event_id)
        if len(self.consumed) > 64:
            self.consumed = set(list(self.consumed)[-32:])
        self.cooldown_until = now + self.cooldown_s
        self.window.clear()
        return direction


class JsonlQRLogger:
    def __init__(self, path: str, confirm_count: int):
        self.path = Path(path)
        self.confirm_count = max(1, int(confirm_count))
        self.recent = deque(maxlen=max(3, self.confirm_count + 2))
        self.seen = set()
        self._load_seen()

    def _load_seen(self):
        if not self.path.exists():
            return
        try:
            for line in self.path.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                payload = json.loads(line)
                content = payload.get("qr_content")
                if content:
                    self.seen.add(str(content))
        except (OSError, json.JSONDecodeError):
            return

    def observe(self, content: str, *, robot_state: str, frame_id: str | None, context: dict):
        content = str(content or "").strip()
        if not content:
            return None
        self.recent.append(content)
        if sum(1 for item in self.recent if item == content) < self.confirm_count:
            return None
        if content in self.seen:
            return {"content": content, "logged": False, "duplicate": True}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "qr_content": content,
            "source": "camera",
            "frame_id": frame_id,
            "robot_state": robot_state,
            "confidence": None,
            "context": context,
        }
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=True) + "\n")
        self.seen.add(content)
        return {"content": content, "logged": True, "duplicate": False}


class CompetitionRgbGapQrSignalNav(Node):
    def __init__(self):
        super().__init__("competition_rgb_gap_qr_signal_nav")

        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("depth_topic", "/oakd/stereo/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("number_of_bins", 40)
        self.declare_parameter("roi_top_ratio", 0.35)
        self.declare_parameter("roi_bottom_ratio", 0.95)
        self.declare_parameter("floor_sample_height_ratio", 0.18)
        self.declare_parameter("floor_sample_width_ratio", 0.50)
        self.declare_parameter("color_threshold", 32.0)
        self.declare_parameter("edge_boost", 28.0)
        self.declare_parameter("row_occupancy", 0.08)
        self.declare_parameter("side_blind_bins", 4)
        self.declare_parameter("bright_wall_l_threshold", 178.0)
        self.declare_parameter("bright_wall_delta_l", 18.0)
        self.declare_parameter("obstacle_dilate_iters", 1)
        self.declare_parameter("min_depth", 0.35)
        self.declare_parameter("max_depth", 4.0)
        self.declare_parameter("front_stop_distance", 0.65)
        self.declare_parameter("front_wall_center_width_ratio", 0.45)
        self.declare_parameter("front_wall_bottom_height_ratio", 0.55)
        self.declare_parameter("front_wall_occupancy_threshold", 0.18)
        self.declare_parameter("align_forward_speed", 0.06)
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("max_linear_speed", 0.12)
        self.declare_parameter("min_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("target_filter_alpha", 0.45)
        self.declare_parameter("robot_width_m", 0.339)
        self.declare_parameter("robot_length_m", 0.341)
        self.declare_parameter("straight_safety_margin_m", 0.03)
        self.declare_parameter("turn_safety_margin_m", 0.12)
        self.declare_parameter("corridor_centering_gain", 0.35)
        self.declare_parameter("corridor_yaw_limit", 0.18)
        self.declare_parameter("blocked_turn_yaw", 0.35)
        self.declare_parameter("escape_yaw", 0.45)
        self.declare_parameter("escape_hold_seconds", 0.80)
        self.declare_parameter("escape_deadband_m", 0.08)
        self.declare_parameter("lidar_enabled", True)
        self.declare_parameter("lidar_timeout_s", 0.60)
        self.declare_parameter("lidar_forward_offset_deg", 0.0)
        self.declare_parameter("lidar_front_sector_deg", 24.0)
        self.declare_parameter("lidar_front_center_deg", 8.0)
        self.declare_parameter("lidar_front_stop", 0.45)
        self.declare_parameter("lidar_front_slow", 0.75)
        self.declare_parameter("lidar_emergency_front_stop", 0.22)
        self.declare_parameter("lidar_side_stop", 0.18)
        self.declare_parameter("lidar_turn_clearance", 0.34)
        self.declare_parameter("lidar_front_percentile", 20.0)
        self.declare_parameter("image_timeout_s", 1.0)
        self.declare_parameter("depth_timeout_s", 0.50)
        self.declare_parameter("qr_enabled", True)
        self.declare_parameter("qr_check_every_n_frames", 3)
        self.declare_parameter("qr_confirm_count", 2)
        self.declare_parameter("qr_hold_seconds", 0.9)
        self.declare_parameter("qr_log_path", "output/qr_log.jsonl")
        self.declare_parameter("signal_state_path", "output/signals/latest_signal.json")
        self.declare_parameter("signal_max_age_s", 1.0)
        self.declare_parameter("signal_min_confidence", 0.70)
        self.declare_parameter("signal_min_area_ratio", 0.025)
        self.declare_parameter("signal_center_min", 0.15)
        self.declare_parameter("signal_center_max", 0.85)
        self.declare_parameter("signal_confirm_window", 5)
        self.declare_parameter("signal_confirm_count", 2)
        self.declare_parameter("signal_cooldown_s", 4.0)
        self.declare_parameter("turn_speed", 0.55)
        self.declare_parameter("turn_seconds", 1.65)
        self.declare_parameter("turn_settle_seconds", 0.20)
        self.declare_parameter("stop_signal_seconds", 2.0)
        self.declare_parameter("show_debug", False)
        self.declare_parameter("telemetry_port", 6000)
        self.declare_parameter("telemetry_hz", 5.0)
        self.declare_parameter("send_scan_array", True)
        self.declare_parameter("scan_array_stride", 1)
        self.declare_parameter("robot_name", "turtlebot4_rensso_mora")
        self.declare_parameter("pairing_code", "ROBOT_A_2")

        g = lambda n: self.get_parameter(n).value
        self.image_topic = str(g("image_topic"))
        self.depth_topic = str(g("depth_topic"))
        self.scan_topic = str(g("scan_topic"))
        self.cmd_topic = str(g("cmd_topic"))
        self.num_bins = int(g("number_of_bins"))
        self.roi_top_ratio = float(g("roi_top_ratio"))
        self.roi_bottom_ratio = float(g("roi_bottom_ratio"))
        self.floor_sample_height_ratio = float(g("floor_sample_height_ratio"))
        self.floor_sample_width_ratio = float(g("floor_sample_width_ratio"))
        self.color_threshold = float(g("color_threshold"))
        self.edge_boost = float(g("edge_boost"))
        self.row_occupancy = float(g("row_occupancy"))
        self.side_blind_bins = max(0, int(g("side_blind_bins")))
        self.bright_wall_l_threshold = float(g("bright_wall_l_threshold"))
        self.bright_wall_delta_l = float(g("bright_wall_delta_l"))
        self.obstacle_dilate_iters = int(g("obstacle_dilate_iters"))
        self.min_depth = float(g("min_depth"))
        self.max_depth = float(g("max_depth"))
        self.front_stop = float(g("front_stop_distance"))
        self.front_wall_center_width_ratio = float(g("front_wall_center_width_ratio"))
        self.front_wall_bottom_height_ratio = float(g("front_wall_bottom_height_ratio"))
        self.front_wall_occupancy_threshold = float(g("front_wall_occupancy_threshold"))
        self.kp = float(g("kp"))
        self.max_v = float(g("max_linear_speed"))
        self.min_v = float(g("min_linear_speed"))
        self.max_w = float(g("max_angular_speed"))
        self.target_filter_alpha = float(g("target_filter_alpha"))
        self.robot_width_m = float(g("robot_width_m"))
        self.robot_length_m = float(g("robot_length_m"))
        self.robot_half_width = self.robot_width_m * 0.5
        self.robot_corner_radius = float(np.hypot(self.robot_width_m * 0.5, self.robot_length_m * 0.5))
        self.straight_safety_margin_m = float(g("straight_safety_margin_m"))
        self.turn_safety_margin_m = float(g("turn_safety_margin_m"))
        self.side_hard_min = self.robot_half_width + self.straight_safety_margin_m
        self.side_turn_min = self.robot_corner_radius + self.turn_safety_margin_m
        self.corridor_centering_gain = float(g("corridor_centering_gain"))
        self.corridor_yaw_limit = float(g("corridor_yaw_limit"))
        self.blocked_turn_yaw = min(abs(float(g("blocked_turn_yaw"))), self.max_w)
        self.align_forward_speed = min(abs(float(g("align_forward_speed"))), self.max_v)
        self.escape_w = min(abs(float(g("escape_yaw"))), self.max_w)
        self.escape_hold_seconds = max(0.0, float(g("escape_hold_seconds")))
        self.escape_deadband_m = max(0.0, float(g("escape_deadband_m")))
        self.lidar_enabled = bool(g("lidar_enabled"))
        self.lidar_timeout_s = max(0.05, float(g("lidar_timeout_s")))
        self.lidar_forward_offset = np.radians(float(g("lidar_forward_offset_deg")))
        self.lidar_front_sector_deg = float(g("lidar_front_sector_deg"))
        self.lidar_front_center_deg = float(g("lidar_front_center_deg"))
        self.lidar_front_stop = float(g("lidar_front_stop"))
        self.lidar_front_slow = float(g("lidar_front_slow"))
        self.lidar_emergency_front_stop = float(g("lidar_emergency_front_stop"))
        self.lidar_side_stop = float(g("lidar_side_stop"))
        self.lidar_turn_clearance = float(g("lidar_turn_clearance"))
        self.lidar_front_percentile = float(g("lidar_front_percentile"))
        self.image_timeout_s = max(0.1, float(g("image_timeout_s")))
        self.depth_timeout_s = max(0.1, float(g("depth_timeout_s")))
        self.qr_enabled = bool(g("qr_enabled"))
        self.qr_check_every_n_frames = max(1, int(g("qr_check_every_n_frames")))
        self.qr_hold_seconds = max(0.0, float(g("qr_hold_seconds")))
        self.signal_state_path = Path(str(g("signal_state_path")))
        self.signal_max_age_s = float(g("signal_max_age_s"))
        self.signal_min_confidence = float(g("signal_min_confidence"))
        self.signal_min_area_ratio = float(g("signal_min_area_ratio"))
        self.signal_center_min = float(g("signal_center_min"))
        self.signal_center_max = float(g("signal_center_max"))
        self.turn_speed = min(abs(float(g("turn_speed"))), self.max_w)
        self.turn_seconds = max(0.1, float(g("turn_seconds")))
        self.turn_settle_seconds = max(0.0, float(g("turn_settle_seconds")))
        self.stop_signal_seconds = max(0.0, float(g("stop_signal_seconds")))
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
        self.last_depth_msg_time = None
        self.last_depth_scan = None
        self.last_camera_source = "rgb"
        self.frame_count = 0
        self.active_maneuver = None
        self.mission_complete = False
        self.maneuver_until = 0.0
        self.qr_hold_until = 0.0
        self.last_qr_event = None
        self.last_signal = {"direction": "none", "reason": "missing"}
        self.last_target_angle = 0.0
        self.last_scan_msg_time = None
        self.last_scan_ranges = None
        self.last_scan_angle_min = 0.0
        self.last_scan_angle_increment = 1e-6
        self.last_scan_range_min = 0.02
        self.last_scan_range_max = 12.0
        self.last_escape_sign = 1.0
        self.escape_hold_until = 0.0
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
        self.qr_detector = cv2.QRCodeDetector() if self.qr_enabled else None
        self.qr_logger = JsonlQRLogger(str(g("qr_log_path")), int(g("qr_confirm_count")))
        self.signal_debouncer = StableSignalDebouncer(
            int(g("signal_confirm_window")),
            int(g("signal_confirm_count")),
            float(g("signal_cooldown_s")),
        )
        self.pub_cmd = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.sub_image = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data
        )
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.status_timer = self.create_timer(5.0, self._status_check)
        self.safety_timer = self.create_timer(0.20, self.safety_watchdog)

        self.get_logger().info("Competition RGB gap + QR + signal node initialized.")
        self.get_logger().info(f"Subscribed to RGB image: {self.image_topic}")
        self.get_logger().info(f"Subscribed to camera depth: {self.depth_topic}")
        self.get_logger().info(f"Subscribed to LiDAR safety scan: {self.scan_topic}")
        self.get_logger().warn("Using RGB for visual mapping and LiDAR as mandatory safety.")

    def safety_watchdog(self):
        now = time.monotonic()
        if self.last_image_msg_time is not None and now - self.last_image_msg_time > self.image_timeout_s:
            self.publish_cmd(0.0, 0.0)
            return
        if self.lidar_enabled and (
            self.last_scan_msg_time is None or now - self.last_scan_msg_time > self.lidar_timeout_s
        ):
            self.publish_cmd(0.0, 0.0)

    def scan_callback(self, msg: LaserScan):
        self.last_scan_msg_time = time.monotonic()
        self.last_scan_ranges = np.asarray(msg.ranges, dtype=np.float32)
        self.last_scan_angle_min = float(msg.angle_min)
        self.last_scan_angle_increment = float(msg.angle_increment) if msg.angle_increment else 1e-6
        self.last_scan_range_min = float(msg.range_min) if msg.range_min > 0.0 else 0.02
        self.last_scan_range_max = float(msg.range_max) if msg.range_max > 0.0 else 12.0

    def depth_callback(self, msg: Image):
        try:
            if getattr(msg, "encoding", "") == "16UC1":
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                depth_m = depth_raw.astype(np.float32) / 1000.0
            else:
                depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")
            return
        scan = depth_image_to_scan(
            depth_m,
            self.num_bins,
            self.roi_top_ratio,
            self.roi_bottom_ratio,
            self.min_depth,
            self.max_depth,
        )
        if self.side_blind_bins > 0:
            scan[: self.side_blind_bins] = self.min_depth
            scan[-self.side_blind_bins :] = self.min_depth
        self.last_depth_scan = scan
        self.last_depth_msg_time = time.monotonic()

    def image_callback(self, msg: Image):
        self.last_image_msg_time = time.monotonic()
        self.frame_count += 1
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")
            self.publish_cmd(0.0, 0.0)
            return

        rgb_scan, obstacle_mask, roi_bounds = estimate_visual_scan(
            bgr,
            self.num_bins,
            self.roi_top_ratio,
            self.roi_bottom_ratio,
            self.floor_sample_height_ratio,
            self.floor_sample_width_ratio,
            self.color_threshold,
            self.edge_boost,
            self.row_occupancy,
            self.bright_wall_l_threshold,
            self.bright_wall_delta_l,
            self.obstacle_dilate_iters,
            self.min_depth,
            self.max_depth,
        )
        depth_fresh = (
            self.last_depth_scan is not None
            and self.last_depth_msg_time is not None
            and time.monotonic() - self.last_depth_msg_time <= self.depth_timeout_s
        )
        if depth_fresh:
            scan = self.last_depth_scan.copy()
            self.last_camera_source = "depth"
            front_wall = False
        else:
            scan = rgb_scan
            self.last_camera_source = "rgb"
            if self.side_blind_bins > 0:
                scan[: self.side_blind_bins] = self.min_depth
                scan[-self.side_blind_bins :] = self.min_depth
            front_wall = front_wall_occupied(
                obstacle_mask,
                self.front_wall_center_width_ratio,
                self.front_wall_bottom_height_ratio,
                self.front_wall_occupancy_threshold,
            )

        best_bin = None
        gap = None
        front_clear = self.front_clearance(scan)
        nearest_idx = int(np.argmin(scan))
        nearest_dist = float(scan[nearest_idx])
        nearest_angle = self.bin_to_angle_deg(nearest_idx)
        signal = self.read_latest_signal()
        self.last_signal = signal
        qr_event = self.maybe_decode_qr(msg, bgr, front_clear, nearest_dist, nearest_angle)
        if qr_event is not None:
            self.last_qr_event = qr_event
            self.qr_hold_until = time.monotonic() + self.qr_hold_seconds

        lidar = self.lidar_snapshot()
        speed, yaw, state = self.decide_motion(scan, front_clear, front_wall, signal)
        speed, yaw, state = self.apply_lidar_safety(speed, yaw, state, lidar)
        self.publish_cmd(speed, yaw)
        self._send_state(msg, state, scan, front_clear, nearest_dist, nearest_angle, gap, speed, yaw, lidar)

        if self.show_debug:
            self.draw_debug(bgr, obstacle_mask, roi_bounds, scan, best_bin, speed, yaw, state)

    def decide_motion(self, scan, front_clear, front_wall, signal):
        now = time.monotonic()
        if self.mission_complete:
            return 0.0, 0.0, "MISSION_COMPLETE_META"
        if front_wall or front_clear < self.front_stop:
            self.active_maneuver = None
            state = "RGB_FRONT_WALL" if front_wall else "RGB_FRONT_BLOCKED"
            return 0.0, 0.0, state

        if self.active_maneuver is not None:
            kind, direction = self.active_maneuver
            if now < self.maneuver_until:
                if kind == "turn":
                    sign = 1.0 if direction == "left" else -1.0
                    return 0.0, sign * self.turn_speed, f"SIGNAL_TURN_{direction.upper()}"
                if kind == "stop":
                    return 0.0, 0.0, "SIGNAL_STOP"
                if kind == "settle":
                    return 0.0, 0.0, "SETTLING_AFTER_TURN"
            if kind == "turn" and self.turn_settle_seconds > 0.0:
                self.active_maneuver = ("settle", direction)
                self.maneuver_until = now + self.turn_settle_seconds
                return 0.0, 0.0, "SETTLING_AFTER_TURN"
            self.active_maneuver = None

        if now < self.qr_hold_until:
            return 0.0, 0.0, "QR_SCAN"

        confirmed = self.signal_debouncer.update(signal, now)
        if confirmed in ("left", "right"):
            self.active_maneuver = ("turn", confirmed)
            self.maneuver_until = now + self.turn_seconds
            sign = 1.0 if confirmed == "left" else -1.0
            return 0.0, sign * self.turn_speed, f"SIGNAL_TURN_{confirmed.upper()}"
        if confirmed == "stop":
            self.active_maneuver = ("stop", "stop")
            self.maneuver_until = now + self.stop_signal_seconds
            return 0.0, 0.0, "SIGNAL_STOP"
        if confirmed == "meta":
            self.mission_complete = True
            self.active_maneuver = ("stop", "meta")
            self.maneuver_until = now + 24 * 60 * 60
            return 0.0, 0.0, "SIGNAL_META_FINAL_STOP"

        yaw = self.corridor_yaw_from_scan(scan)
        dist_factor = np.clip(
            (front_clear - self.front_stop) / max(self.max_depth - self.front_stop, 1e-3),
            0.0,
            1.0,
        )
        turn_factor = 1.0 - 0.35 * min(abs(yaw) / max(self.corridor_yaw_limit, 1e-3), 1.0)
        speed = float(self.min_v + (self.max_v - self.min_v) * dist_factor * turn_factor)
        return speed, yaw, "CORRIDOR_FORWARD"

    def maybe_decode_qr(self, msg, bgr, front_clear, nearest_dist, nearest_angle):
        if self.qr_detector is None or self.frame_count % self.qr_check_every_n_frames != 0:
            return None
        try:
            content, _, _ = self.qr_detector.detectAndDecode(bgr)
        except Exception as exc:
            self.get_logger().warn(f"QR decode error: {exc}")
            return None
        event = self.qr_logger.observe(
            content,
            robot_state=self.active_maneuver[0] if self.active_maneuver else "RGB_FORWARD",
            frame_id=getattr(msg.header, "frame_id", None),
            context={
                "front_clear_m": float(front_clear),
                "nearest_dist_m": float(nearest_dist),
                "nearest_angle_deg": float(nearest_angle),
            },
        )
        if event is None:
            return None
        if event["logged"]:
            self.get_logger().info(f"QR logged: {event['content']} -> {self.qr_logger.path}")
            self._send_log("INFO", f"QR logged: {event['content']}")
        return event

    def read_latest_signal(self):
        path = self.signal_state_path
        if not path.exists():
            return {"direction": "none", "reason": f"missing:{path}"}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            return {"direction": "none", "reason": f"read_error:{exc}"}

        timestamp = float(payload.get("timestamp") or 0.0)
        age = time.time() - timestamp if timestamp > 0.0 else float("inf")
        direction = self.normalize_signal_direction(payload)
        confidence = float(payload.get("confidence") or 0.0)
        area = float(payload.get("bbox_area_ratio") or payload.get("area_ratio") or 0.0)
        center_x = float(payload.get("bbox_center_x_ratio") or payload.get("center_x_ratio") or 0.5)
        actionable = (
            age <= self.signal_max_age_s
            and direction in ("left", "right", "stop", "meta")
            and confidence >= self.signal_min_confidence
            and area >= self.signal_min_area_ratio
            and self.signal_center_min <= center_x <= self.signal_center_max
        )
        if not actionable:
            direction = "none"
        return {
            "direction": direction,
            "confidence": confidence,
            "area": area,
            "center_x": center_x,
            "age": age,
            "event_id": (
                f"{payload.get('direction')}:{payload.get('source_frame_time')}:"
                f"{timestamp:.6f}:{payload.get('bbox_xyxy') or payload.get('bbox')}"
            ),
            "reason": "fresh" if age <= self.signal_max_age_s else f"stale:{age:.2f}s",
        }

    def normalize_signal_direction(self, payload):
        raw = payload.get("direction") or payload.get("class_name") or payload.get("label") or "none"
        normalized = str(raw).lower().replace("-", "_").replace(" ", "_")
        if "left" in normalized or "izquierda" in normalized:
            return "left"
        if "right" in normalized or "derecha" in normalized:
            return "right"
        if "stop" in normalized or "alto" in normalized:
            return "stop"
        if "meta" in normalized or "finish" in normalized or "goal" in normalized:
            return "meta"
        return "none"

    def front_clearance(self, scan: np.ndarray):
        mid = self.num_bins // 2
        span = max(1, self.num_bins // 10)
        return float(np.min(scan[mid - span : mid + span + 1]))

    def corridor_yaw_from_scan(self, scan: np.ndarray):
        n = scan.size
        if n < 6:
            return 0.0
        left = float(np.percentile(scan[int(n * 0.15) : int(n * 0.45)], 35))
        right = float(np.percentile(scan[int(n * 0.55) : int(n * 0.85)], 35))
        if not np.isfinite(left) or not np.isfinite(right):
            return 0.0
        raw = self.corridor_centering_gain * (left - right)
        yaw = float(np.clip(raw, -self.corridor_yaw_limit, self.corridor_yaw_limit))
        yaw = self.target_filter_alpha * yaw + (1.0 - self.target_filter_alpha) * self.last_target_angle
        if abs(yaw) < 0.03:
            yaw = 0.0
        self.last_target_angle = yaw
        return yaw

    def lidar_snapshot(self):
        if not self.lidar_enabled:
            return {"enabled": False, "fresh": False, "reason": "disabled"}
        if self.last_scan_ranges is None or self.last_scan_msg_time is None:
            return {"enabled": True, "fresh": False, "reason": "missing"}
        age = time.monotonic() - self.last_scan_msg_time
        if age > self.lidar_timeout_s:
            return {"enabled": True, "fresh": False, "reason": f"stale:{age:.2f}s", "age": age}

        def sector(start, end, percentile=0.0):
            return lidar_sector_min(
                self.last_scan_ranges,
                self.last_scan_angle_min,
                self.last_scan_angle_increment,
                self.last_scan_range_min,
                self.last_scan_range_max,
                start,
                end,
                self.lidar_forward_offset,
                percentile,
            )

        return {
            "enabled": True,
            "fresh": True,
            "reason": "fresh",
            "age": age,
            "front": sector(-self.lidar_front_sector_deg, self.lidar_front_sector_deg),
            "front_center": sector(
                -self.lidar_front_center_deg,
                self.lidar_front_center_deg,
                self.lidar_front_percentile,
            ),
            "front_left": sector(20.0, 70.0),
            "front_right": sector(-70.0, -20.0),
            "left": sector(70.0, 110.0),
            "right": sector(-110.0, -70.0),
        }

    def held_escape_sign(self, left_clear: float, right_clear: float):
        now = time.monotonic()
        if now < self.escape_hold_until:
            return self.last_escape_sign
        if abs(left_clear - right_clear) > self.escape_deadband_m:
            self.last_escape_sign = 1.0 if left_clear > right_clear else -1.0
            self.escape_hold_until = now + self.escape_hold_seconds
        return self.last_escape_sign

    def apply_lidar_safety(self, speed: float, yaw: float, state: str, lidar):
        if not lidar.get("enabled", False):
            return speed, yaw, state
        if not lidar.get("fresh", False):
            return 0.0, 0.0, f"LIDAR_STOP_{lidar.get('reason', 'missing').upper()}"

        front = float(lidar["front"])
        front_center = float(lidar["front_center"])
        front_left = float(lidar["front_left"])
        front_right = float(lidar["front_right"])
        left = float(lidar["left"])
        right = float(lidar["right"])

        camera_blocked = state in ("RGB_FRONT_WALL", "RGB_FRONT_BLOCKED", "RGB_BLOCKED")
        if min(front, front_center) <= self.lidar_emergency_front_stop:
            self.active_maneuver = None
            if camera_blocked:
                yaw = self.blocked_turn_yaw_from_lidar(lidar)
                return 0.0, yaw, "CAMERA_BLOCKED_LIDAR_ESCAPE" if yaw else "BLOCKED_NO_TURN_CLEARANCE"
            return 0.0, 0.0, "LIDAR_EMERGENCY_STOP"
        if camera_blocked and speed == 0.0:
            yaw = self.blocked_turn_yaw_from_lidar(lidar)
            state = "CAMERA_BLOCKED_LIDAR_TURN" if yaw else "BLOCKED_NO_TURN_CLEARANCE"

        if front_center < self.lidar_front_slow and speed > 0.0:
            span = max(self.lidar_front_slow - self.lidar_emergency_front_stop, 1e-3)
            scale = float(np.clip((front_center - self.lidar_emergency_front_stop) / span, 0.0, 1.0))
            speed = min(speed, self.max_v * scale)
            state = f"{state}+LIDAR_SLOW"

        left_close = left <= self.lidar_side_stop
        right_close = right <= self.lidar_side_stop
        if left_close and right_close and speed > 0.0:
            return min(speed, self.align_forward_speed), 0.0, "LIDAR_NARROW_FORWARD"
        if left_close and speed > 0.0:
            speed = min(speed, self.align_forward_speed)
            yaw = min(yaw, 0.0)
            state = "LIDAR_LEFT_WALL_SLOW"
        if right_close and speed > 0.0:
            speed = min(speed, self.align_forward_speed)
            yaw = max(yaw, 0.0)
            state = "LIDAR_RIGHT_WALL_SLOW"

        if yaw > 0.0 and front_left < self.lidar_turn_clearance:
            yaw = 0.0
            state = f"{state}+LIDAR_LEFT_VETO"
        elif yaw < 0.0 and front_right < self.lidar_turn_clearance:
            yaw = 0.0
            state = f"{state}+LIDAR_RIGHT_VETO"

        return float(np.clip(speed, 0.0, self.max_v)), float(np.clip(yaw, -self.max_w, self.max_w)), state

    def blocked_turn_yaw_from_lidar(self, lidar):
        left = float(lidar.get("left", 0.0))
        right = float(lidar.get("right", 0.0))
        front_left = float(lidar.get("front_left", 0.0))
        front_right = float(lidar.get("front_right", 0.0))
        left_ok = min(left, front_left) >= self.side_turn_min
        right_ok = min(right, front_right) >= self.side_turn_min
        if not left_ok and not right_ok:
            return 0.0
        if left_ok and right_ok:
            sign = self.held_escape_sign(max(left, front_left), max(right, front_right))
        else:
            sign = 1.0 if left_ok else -1.0
        return float(sign * self.blocked_turn_yaw)

    def escape_yaw(self, scan: np.ndarray):
        left = float(np.mean(scan[: self.num_bins // 2]))
        right = float(np.mean(scan[self.num_bins // 2 :]))
        return float(np.clip(self.held_escape_sign(left, right) * self.escape_w, -self.max_w, self.max_w))

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

    def _send_state(self, msg, state, scan, front_clear, nearest_dist, nearest_angle, gap, speed, yaw, lidar):
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
            f"signal={self.last_signal.get('direction', 'none')}",
            f"sig_reason={self.last_signal.get('reason', 'missing')}",
            f"qr={self.last_qr_event['content'] if self.last_qr_event else 'none'}",
            f"camera={self.last_camera_source}",
            f"lidar={lidar.get('reason', 'disabled')}",
        ]
        if lidar.get("fresh", False):
            fields.extend(
                [
                    f"lf={float(lidar['front']):.3f}",
                    f"lfc={float(lidar['front_center']):.3f}",
                    f"lfl={float(lidar['front_left']):.3f}",
                    f"lfr={float(lidar['front_right']):.3f}",
                    f"ll={float(lidar['left']):.3f}",
                    f"lr={float(lidar['right']):.3f}",
                ]
            )
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
        if self.lidar_enabled and self.last_scan_msg_time is None:
            publishers = len(self.get_publishers_info_by_topic(self.scan_topic))
            msg = f"No LiDAR scans received on {self.scan_topic}; scan_publishers={publishers}"
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
        cv2.imshow("Competition RGB gap QR signal nav", vis)
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
    scan, _, _ = estimate_visual_scan(
        img,
        20,
        0.25,
        1.0,
        0.15,
        0.5,
        30.0,
        28.0,
        0.08,
        178.0,
        18.0,
        1,
        0.35,
        4.0,
    )
    assert float(np.min(scan[8:12])) < 1.0
    assert float(np.mean(scan[:4])) > 3.0
    depth = np.full((60, 100), 2.5, dtype=np.float32)
    depth[:, 45:55] = 0.45
    depth_scan = depth_image_to_scan(depth, 20, 0.0, 1.0, 0.35, 4.0)
    assert float(np.min(depth_scan[8:12])) < 0.6
    mask = np.zeros((80, 100), dtype=np.uint8)
    mask[30:, 35:65] = 1
    assert front_wall_occupied(mask, 0.4, 0.6, 0.3)
    ranges = np.full(361, 3.0, dtype=np.float32)
    ranges[180] = 0.32
    assert lidar_sector_min(ranges, -np.pi, np.radians(1.0), 0.05, 8.0, -5.0, 5.0) < 0.4
    ranges[180] = np.inf
    assert lidar_sector_min(ranges, -np.pi, np.radians(1.0), 0.05, 8.0, -5.0, 5.0) >= 3.0


def main(args=None):
    if "--self-test" in sys.argv:
        _self_test()
        print("self-test ok")
        return
    rclpy.init(args=args)
    node = CompetitionRgbGapQrSignalNav()
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
