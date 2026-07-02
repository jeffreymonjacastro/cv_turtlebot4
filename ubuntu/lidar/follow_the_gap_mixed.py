#!/usr/bin/env python3
"""
Autonomous Reactive Navigation Node for TurtleBot 4 (Create 3 base, OAK-D camera)
using Stereo Depth (or RGB pseudo-depth fallback) for local planning AND a 2D
LiDAR as a mandatory safety layer. Works on ROS 2 JAZZY.

Subscribes:  /oakd/stereo/image_raw        (sensor_msgs/Image, depth in mm or m)  [preferred]
             /oakd/rgb/preview/image_raw   (sensor_msgs/Image, BGR)               [fallback]
             /scan                         (sensor_msgs/LaserScan, real 2D LiDAR) [mandatory]
Publishes:   /cmd_vel                      (geometry_msgs/TwistStamped)

Camera modes (auto-selected at runtime):
  stereo      -- OAK-D stereo depth is publishing; full metric depth pipeline.
  rgb_fallback-- No stereo depth publisher; uses visual pseudo-depth from RGB.
  lidar_only  -- No camera data at all; navigates with LiDAR-only FtG.

Architecture (two independent layers):
  1. PLANNER (camera when available): Follow-the-Gap over a virtual scan built
     from depth (stereo or pseudo). When camera is unavailable, LiDAR scan is
     used directly for planning (lidar_only mode).
  2. SAFETY (LiDAR, 360-degree real range data): every command passes through
     apply_lidar_safety() before publishing. Can only brake/stop, never speed up.
  A safety_watchdog timer forces zero-velocity if the LiDAR goes stale.
  The camera is treated as optional: its absence downgrades the mode but does
  NOT stop the robot as long as the LiDAR is live.
"""

import os
import socket
import threading
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge


def _estimate_visual_scan(
    bgr: np.ndarray,
    num_bins: int,
    roi_top_ratio: float,
    roi_bottom_ratio: float,
    floor_sample_height_ratio: float,
    floor_sample_width_ratio: float,
    color_threshold: float,
    edge_boost: float,
    row_occupancy: float,
    side_blind_bins: int,
    min_depth: float,
    max_depth: float,
) -> np.ndarray:
    """Pseudo-depth scan from an RGB image (no stereo required).

    Returns a 1-D array of length `num_bins` with estimated distances [m].
    Obstacles detected by color/edge differences from the floor sample are
    mapped conservatively: the closer to the bottom of the ROI the detected
    row is, the shorter the estimated distance.
    """
    height, width = bgr.shape[:2]
    top = int(np.clip(height * roi_top_ratio, 0, height - 2))
    bottom = int(np.clip(height * roi_bottom_ratio, top + 1, height))
    roi = bgr[top:bottom]
    roi_h, roi_w = roi.shape[:2]

    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
    sample_h = max(4, int(roi_h * floor_sample_height_ratio))
    sample_w = max(4, int(roi_w * floor_sample_width_ratio))
    x0 = (roi_w - sample_w) // 2
    floor_sample = lab[roi_h - sample_h: roi_h, x0: x0 + sample_w]
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

    if side_blind_bins > 0:
        scan[:side_blind_bins] = min_depth
        scan[-side_blind_bins:] = min_depth

    return scan


class FollowTheGapDepth(Node):
    def __init__(self):
        super().__init__("follow_the_gap_depth")

        # ---- Declare ROS 2 Parameters ----
        self.declare_parameter("depth_topic", "/oakd/stereo/image_raw")
        self.declare_parameter("rgb_topic", "/oakd/rgb/preview/image_raw")  # RGB fallback
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("number_of_bins", 72)
        self.declare_parameter("roi_top", 280)         # Top row of ROI (assuming 640x480)
        self.declare_parameter("roi_bottom", 440)      # Bottom row of ROI
        self.declare_parameter("max_depth", 4.0)       # Maximum reliable depth (m)
        self.declare_parameter("min_depth", 0.4)       # Minimum depth (m)
        self.declare_parameter("bubble_radius_min", 0.32) # Safe min bubble (m) — TurtleBot4 radius=0.17m
        self.declare_parameter("bubble_radius_max", 1.20) # Max bubble for close/fast obstacles (m)
        self.declare_parameter("bubble_k", 0.10)       # Scaling factor for dynamic bubble
        self.declare_parameter("bubble_speed_factor", 0.20) # m of bubble per m/s of speed
        self.declare_parameter("kp", 1.4)              # Proportional gain (higher for wider FOV bins)
        self.declare_parameter("max_linear_speed", 0.12)  # m/s — conservative for indoor
        self.declare_parameter("min_linear_speed", 0.025)  # m/s
        self.declare_parameter("max_angular_speed", 0.60)  # rad/s
        self.declare_parameter("distance_weight", 1.0) # alpha
        self.declare_parameter("gap_weight", 0.8)      # beta
        self.declare_parameter("steering_weight", 0.5)  # gamma
        self.declare_parameter("closeness_weight", 0.6) # delta
        self.declare_parameter("temporal_filter_alpha", 0.35)  # EMA smoothing (higher = faster response)
        self.declare_parameter("emergency_angle_deg", 25.0) # Skip smoothing if error > this (deg)
        self.declare_parameter("median_kernel", 3)     # 1D median filter kernel size for bins
        self.declare_parameter("front_stop_distance", 0.55) # Camera frontal stop (m)
        self.declare_parameter("minimum_gap_width", 0.55)   # Min traversable gap width (m)
        self.declare_parameter("minimum_gap_width_straight", 0.50)
        self.declare_parameter("minimum_gap_width_turn", 0.75)
        self.declare_parameter("hfov_deg", 69.0)       # OAK-D stereo FOV (used only in stereo mode)
        self.declare_parameter("lidar_planning_fov_deg", 180.0) # LiDAR planning FOV (rgb_fallback/lidar_only)
        self.declare_parameter("show_debug", False)
        self.declare_parameter("telemetry_port", 6000)
        self.declare_parameter("telemetry_hz", 5.0)
        self.declare_parameter("send_scan_array", True)
        self.declare_parameter("scan_array_stride", 1)
        self.declare_parameter("robot_name", "turtlebot4_rensso_mora")
        self.declare_parameter("pairing_code", "ROBOT_A_2")

        # ---- RGB pseudo-depth fallback parameters ----
        self.declare_parameter("rgb_roi_top_ratio", 0.35)
        self.declare_parameter("rgb_roi_bottom_ratio", 0.95)
        self.declare_parameter("rgb_floor_sample_height_ratio", 0.18)
        self.declare_parameter("rgb_floor_sample_width_ratio", 0.50)
        self.declare_parameter("rgb_color_threshold", 32.0)
        self.declare_parameter("rgb_edge_boost", 28.0)
        self.declare_parameter("rgb_row_occupancy", 0.08)
        self.declare_parameter("rgb_side_blind_bins", 4)
        self.declare_parameter("rgb_front_stop_distance", 0.40)
        self.declare_parameter("rgb_minimum_gap_width", 0.40)

        # ---- LiDAR safety-layer parameters (mandatory obstacle layer) ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("lidar_timeout", 1.0)
        self.declare_parameter("camera_timeout", 1.5)
        self.declare_parameter("stereo_check_interval", 10.0)
        self.declare_parameter("lidar_front_stop", 0.38)     # m — TB4 radius 0.17m + 0.21m margin
        self.declare_parameter("lidar_slow_distance", 0.85)  # m — start slowing earlier
        self.declare_parameter("lidar_side_stop", 0.28)      # m — more lateral clearance
        self.declare_parameter("lidar_turn_stop", 0.30)      # m — lateral only; diagonal is 0.75x
        self.declare_parameter("front_sector_deg", 35.0)
        self.declare_parameter("robot_width_m", 0.339)
        self.declare_parameter("robot_length_m", 0.341)
        self.declare_parameter("straight_side_margin", 0.07)
        self.declare_parameter("turn_side_margin", 0.12)
        self.declare_parameter("corridor_centering_gain", 0.45)
        self.declare_parameter("corridor_mode_yaw_limit", 0.20)
        self.declare_parameter("hard_turn_yaw_threshold", 0.35)
        self.declare_parameter("narrow_corridor_speed", 0.055)
        self.declare_parameter("min_corridor_width", 0.48)
        self.declare_parameter("turn_required_clearance", 0.36)
        self.declare_parameter("front_corner_clearance", 0.42)

        # ---- Retrieve Parameters ----
        g = lambda n: self.get_parameter(n).value
        self.depth_topic = str(g("depth_topic"))
        self.rgb_topic = str(g("rgb_topic"))
        self.cmd_topic = str(g("cmd_topic"))
        self.num_bins = int(g("number_of_bins"))
        self.roi_top = int(g("roi_top"))
        self.roi_bottom = int(g("roi_bottom"))
        self.max_depth = float(g("max_depth"))
        self.min_depth = float(g("min_depth"))
        self.bubble_rad_min = float(g("bubble_radius_min"))
        self.bubble_rad_max = float(g("bubble_radius_max"))
        self.bubble_k = float(g("bubble_k"))
        self.bubble_speed_factor = float(g("bubble_speed_factor"))
        self.kp = float(g("kp"))
        self.max_v = float(g("max_linear_speed"))
        self.min_v = float(g("min_linear_speed"))
        self.max_w = float(g("max_angular_speed"))
        self.alpha = float(g("distance_weight"))
        self.beta = float(g("gap_weight"))
        self.gamma = float(g("steering_weight"))
        self.delta = float(g("closeness_weight"))
        self.filter_alpha = float(g("temporal_filter_alpha"))
        self.emergency_angle = np.radians(float(g("emergency_angle_deg")))
        self.med_kernel = int(g("median_kernel"))
        self.front_stop = float(g("front_stop_distance"))
        self.min_gap_w = float(g("minimum_gap_width"))
        self.min_gap_w_straight = float(g("minimum_gap_width_straight"))
        self.min_gap_w_turn = float(g("minimum_gap_width_turn"))
        self.hfov = np.radians(float(g("hfov_deg")))
        self.lidar_planning_fov = np.radians(float(g("lidar_planning_fov_deg")))
        self.show_debug = bool(g("show_debug"))
        if self.show_debug and not os.environ.get("DISPLAY"):
            self.get_logger().warn("show_debug=true but DISPLAY is not set; disabling OpenCV window.")
            self.show_debug = False
        self.telemetry_period = 1.0 / max(float(g("telemetry_hz")), 0.1)
        self.send_scan_array = bool(g("send_scan_array"))
        self.scan_array_stride = max(int(g("scan_array_stride")), 1)
        self.robot_name = str(g("robot_name"))
        self.pairing_code = str(g("pairing_code"))

        # ---- RGB pseudo-depth parameters ----
        self.rgb_roi_top_ratio = float(g("rgb_roi_top_ratio"))
        self.rgb_roi_bottom_ratio = float(g("rgb_roi_bottom_ratio"))
        self.rgb_floor_sample_h = float(g("rgb_floor_sample_height_ratio"))
        self.rgb_floor_sample_w = float(g("rgb_floor_sample_width_ratio"))
        self.rgb_color_threshold = float(g("rgb_color_threshold"))
        self.rgb_edge_boost = float(g("rgb_edge_boost"))
        self.rgb_row_occupancy = float(g("rgb_row_occupancy"))
        self.rgb_side_blind_bins = max(0, int(g("rgb_side_blind_bins")))
        self.rgb_front_stop = float(g("rgb_front_stop_distance"))
        self.rgb_min_gap_w = float(g("rgb_minimum_gap_width"))

        # ---- LiDAR safety-layer parameters ----
        self.scan_topic = str(g("scan_topic"))
        self.lidar_timeout = float(g("lidar_timeout"))
        self.camera_timeout = float(g("camera_timeout"))
        self.stereo_check_interval = float(g("stereo_check_interval"))
        self.lidar_front_stop = float(g("lidar_front_stop"))
        self.lidar_slow_distance = float(g("lidar_slow_distance"))
        self.lidar_side_stop = float(g("lidar_side_stop"))
        self.lidar_turn_stop = float(g("lidar_turn_stop"))
        self.front_sector_deg = float(g("front_sector_deg"))
        self.robot_width_m = float(g("robot_width_m"))
        self.robot_length_m = float(g("robot_length_m"))
        self.robot_half_width = self.robot_width_m / 2.0
        self.robot_corner_radius = float(
            np.hypot(self.robot_width_m / 2.0, self.robot_length_m / 2.0)
        )
        self.straight_side_margin = float(g("straight_side_margin"))
        self.turn_side_margin = float(g("turn_side_margin"))
        self.corridor_centering_gain = float(g("corridor_centering_gain"))
        self.corridor_mode_yaw_limit = float(g("corridor_mode_yaw_limit"))
        self.hard_turn_yaw_threshold = float(g("hard_turn_yaw_threshold"))
        self.narrow_corridor_speed = float(g("narrow_corridor_speed"))
        self.min_corridor_width = float(g("min_corridor_width"))
        self.turn_required_clearance = float(g("turn_required_clearance"))
        self.front_corner_clearance = float(g("front_corner_clearance"))
        self.side_hard_min = self.robot_half_width + self.straight_side_margin
        self.side_turn_min = max(
            self.turn_required_clearance,
            self.robot_corner_radius + self.turn_side_margin,
        )

        # Convenience: last commanded speed for speed-adaptive bubble
        self._last_speed = 0.0

        # ---- UDP socket and thread ----
        self.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "2"))
        self.authorized_addr = None
        self.last_telemetry = 0.0
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

        # ---- CV Bridge and Internal State ----
        self.bridge = CvBridge()
        self.prev_time = time.time()
        self.fps = 0.0
        self.last_target_angle = 0.0  # For steering temporal smoothing
        self.last_depth_msg_time = None
        self.last_rgb_msg_time = None

        # Camera mode: 'stereo' | 'rgb_fallback' | 'lidar_only'
        # Starts as 'lidar_only' so the robot can navigate immediately
        # using LiDAR while waiting to see if camera data arrives.
        # The first safety_watchdog tick will promote the mode if camera
        # frames are already streaming.
        self.camera_mode = "lidar_only"
        self._last_stereo_check = 0.0  # Force immediate evaluation on first tick

        # ---- LiDAR internal state (populated by scan_callback) ----
        self.last_scan_ranges = None          # np.ndarray of raw ranges (meters)
        self.last_scan_msg_time = None        # monotonic time of last /scan message
        self.last_scan_angle_min = 0.0
        self.last_scan_angle_increment = 1.0
        self.last_scan_range_min = 0.02
        self.last_scan_range_max = 30.0

        # ---- Subscribers & Publisher ----
        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data
        )
        self.sub_rgb = self.create_subscription(
            Image, self.rgb_topic, self.rgb_callback, qos_profile_sensor_data
        )
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.pub_cmd = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.status_timer = self.create_timer(5.0, self._status_check)

        # Independent safety watchdog: runs on its own timer so a dead LiDAR
        # topic cannot leave the robot coasting on the last received command.
        # NOTE: camera absence only downgrades the mode; it does NOT stop the
        # robot (the LiDAR safety layer is sufficient for obstacle avoidance).
        self.watchdog_timer = self.create_timer(0.15, self.safety_watchdog)

        self.get_logger().info("Follow-the-Gap Mixed (Stereo+RGB+LiDAR) Node Initialized.")
        self.get_logger().info(f"Subscribed to stereo depth: {self.depth_topic}")
        self.get_logger().info(f"Subscribed to RGB fallback: {self.rgb_topic}")
        self.get_logger().info(f"Subscribed to LiDAR: {self.scan_topic}")
        self.get_logger().info(f"Publishing TwistStamped to: {self.cmd_topic}")

    def depth_callback(self, msg: Image):
        """Stereo depth callback: decodes the depth image, builds a virtual
        scan, then delegates to _process_virtual_scan for the shared
        bubble/gap/control/telemetry pipeline.
        """
        self.last_depth_msg_time = time.monotonic()

        # Only act if we are in stereo mode (rgb_callback handles rgb_fallback).
        if self.camera_mode not in ("stereo", "rgb_fallback"):
            # lidar_only is handled by the watchdog; skip camera processing.
            return

        # FPS tracking
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        # Decode depth image
        try:
            if msg.encoding == "16UC1":
                cv_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                cv_depth = cv_raw.astype(np.float32) / 1000.0  # mm -> m
            else:
                cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")
            self.publish_cmd(0.0, 0.0)
            return

        # Build virtual scan from depth image
        processed_depth = self.preprocess_depth(cv_depth)
        roi_depth = self.extract_roi(processed_depth)
        virtual_scan = self.build_virtual_scan(roi_depth)
        if self.med_kernel > 1:
            virtual_scan = cv2.medianBlur(
                virtual_scan.astype(np.float32), self.med_kernel
            ).flatten()

        self._process_virtual_scan(
            virtual_scan,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            debug_depth=processed_depth if self.show_debug else None,
        )

    def _process_virtual_scan(
        self,
        virtual_scan: np.ndarray,
        stamp_sec: int,
        stamp_nsec: int,
        active_hfov: float = None,
        debug_depth=None,
    ):
        """Shared planning pipeline for all camera modes.

        Parameters
        ----------
        virtual_scan  : num_bins floats [m], pre-built from stereo or LiDAR
        active_hfov   : FOV [rad] used to build virtual_scan (69° for stereo,
                        180° for LiDAR-based modes). Drives angle calculations.
        """
        if active_hfov is None:
            active_hfov = self.hfov  # stereo mode default

        # --- Speed-adaptive safety bubble (Bug B fix) ----------------------
        # bubble_radius = robot_radius + static_margin + dynamic_margin(speed)
        # dynamic margin: how far the robot travels in ~0.15s reaction time
        nearest_idx = np.argmin(virtual_scan)
        d_min = float(virtual_scan[nearest_idx])
        current_speed = abs(self._last_speed)
        bubble_radius = (
            0.17              # TB4 base radius
            + 0.08            # static safety margin
            + current_speed * self.bubble_speed_factor  # dynamic margin
            + self.bubble_k / max(d_min, 0.1)           # proximity boost
        )
        bubble_radius = np.clip(bubble_radius, self.bubble_rad_min, self.bubble_rad_max)
        bin_width_at_d = 2.0 * max(d_min, 0.1) * np.tan(active_hfov / (2.0 * self.num_bins))
        half_bins = int(np.ceil(bubble_radius / max(bin_width_at_d, 1e-3)))

        cleaned_scan = virtual_scan.copy()
        start_bubble = max(0, nearest_idx - half_bins)
        end_bubble = min(self.num_bins - 1, nearest_idx + half_bins)
        cleaned_scan[start_bubble: end_bubble + 1] = 0.0

        # --- Follow-the-Gap (paper-correct: widest gap, deepest point) ------
        scores, best_bin, debug_gaps = self.find_best_gap(
            cleaned_scan, self.front_stop, self.min_gap_w_straight, active_hfov
        )

        if best_bin is not None:
            speed, yaw = self.compute_control(best_bin, cleaned_scan, active_hfov)
        else:
            self.get_logger().warn(f"NO VALID GAP [{self.camera_mode}] - ROTATING TO SEARCH")
            speed = 0.0
            yaw = self.choose_open_turn_yaw(virtual_scan)

        # --- Frontal planner override ----------------------------------------
        mid = self.num_bins // 2
        span = max(1, self.num_bins // 10)
        front_clearance = float(np.min(virtual_scan[mid - span: mid + span + 1]))
        if front_clearance < self.front_stop:
            self.get_logger().warn(
                f"FRONT BLOCKED [{self.camera_mode}]: "
                f"{front_clearance:.2f}m < {self.front_stop}m"
            )
            speed = 0.0
            if abs(yaw) < 0.05:
                yaw = self.choose_open_turn_yaw(virtual_scan)

        # --- Mandatory LiDAR safety layer -----------------------------------
        speed, yaw = self.apply_lidar_safety(speed, yaw)
        self._last_speed = speed
        self.publish_cmd(speed, yaw)

        # --- UDP Telemetry ---
        third = max(1, self.num_bins // 3)
        left_clear  = float(np.min(virtual_scan[0:third]))
        front_clear = float(np.min(virtual_scan[third: 2 * third]))
        right_clear = float(np.min(virtual_scan[2 * third: self.num_bins]))
        target_angle_deg = float(np.degrees(self.last_target_angle))

        gap_start_deg = None
        gap_end_deg = None
        if best_bin is not None:
            for s, e, _ in debug_gaps:
                if s <= best_bin <= e:
                    center_bin = (self.num_bins - 1) / 2.0
                    gap_start_deg = float(
                        np.degrees((center_bin - s) * (active_hfov / self.num_bins))
                    )
                    gap_end_deg = float(
                        np.degrees((center_bin - e) * (active_hfov / self.num_bins))
                    )
                    break

        if speed == 0.0:
            state_str = "FRONT_BLOCKED" if front_clearance < self.front_stop else "BLOCKED"
        else:
            state_str = f"FORWARD[{self.camera_mode}]"

        self._send_telemetry_state(
            stamp_sec, stamp_nsec, state_str,
            front_clear, left_clear, right_clear,
            d_min,
            float(np.degrees((self.num_bins // 2 - nearest_idx) * (active_hfov / self.num_bins))),
            gap_start_deg, gap_end_deg, target_angle_deg,
            speed, yaw, virtual_scan,
        )

        if self.show_debug and debug_depth is not None:
            self.draw_debug(
                debug_depth, virtual_scan, cleaned_scan,
                best_bin, scores, debug_gaps, speed, yaw
            )

    def choose_open_turn_yaw(self, virtual_scan: np.ndarray) -> float:
        """
        Choose a slow in-place turn toward the side with more free space.

        Convention:
          yaw positive = left
          yaw negative = right
        """
        if virtual_scan is None or len(virtual_scan) == 0:
            return 0.0

        mid = len(virtual_scan) // 2
        left_values = virtual_scan[:mid]
        right_values = virtual_scan[mid:]

        left_clear = float(np.nanmedian(left_values))
        right_clear = float(np.nanmedian(right_values))

        if not np.isfinite(left_clear) and not np.isfinite(right_clear):
            return 0.0
        if not np.isfinite(left_clear):
            return -min(0.45, self.max_w * 0.45)
        if not np.isfinite(right_clear):
            return min(0.45, self.max_w * 0.45)

        turn_dir = 1.0 if left_clear >= right_clear else -1.0
        return float(turn_dir * min(0.45, self.max_w * 0.45))

    def preprocess_depth(self, cv_depth: np.ndarray) -> np.ndarray:
        """
        Cleans NaNs, Infs and non-positive depth values.

        SAFETY-CRITICAL CHANGE: invalid pixels (NaN, Inf, or <= 0) are stereo
        failures -- texture-less walls, glare, out-of-range, sensor noise.
        They must NEVER be interpreted as free space. The previous version
        replaced them with max_depth (i.e. "clear path ahead"), which is
        exactly backwards: a light-colored or overexposed wall that the
        stereo matcher fails on would look like open corridor. Invalid
        pixels are now conservatively treated as the closest possible
        obstacle (min_depth), so a sensor failure looks like "something is
        right in front of me" and the planner slows/stops instead of
        confidently driving into unknown space.
        """
        invalid_mask = ~np.isfinite(cv_depth) | (cv_depth <= 0.0)
        cleaned = np.copy(cv_depth)
        cleaned[invalid_mask] = self.min_depth
        # Clip ranges between min and max depth
        return np.clip(cleaned, self.min_depth, self.max_depth)

    def extract_roi(self, cv_depth: np.ndarray) -> np.ndarray:
        """Extracts the vertical Region of Interest (ROI)."""
        height, width = cv_depth.shape
        top = np.clip(self.roi_top, 0, height - 1)
        bottom = np.clip(self.roi_bottom, top + 1, height)
        return cv_depth[top:bottom, :]

    def build_virtual_scan(self, cv_depth_roi: np.ndarray) -> np.ndarray:
        """
        Reduces the ROI image to a set of vertical bins.
        Calculates a robust distance metric (Percentil 20) for each bin.
        Percentile 20 is chosen because it is conservative: it registers small
        obstacles within the bin area while filtering single-pixel sensor noise.
        """
        width = cv_depth_roi.shape[1]
        bin_width = width // self.num_bins
        virtual_scan = np.zeros(self.num_bins, dtype=np.float32)

        for i in range(self.num_bins):
            col_start = i * bin_width
            col_end = (i + 1) * bin_width
            bin_pixels = cv_depth_roi[:, col_start:col_end]
            
            # Robust distance using 20th percentile
            if bin_pixels.size > 0:
                virtual_scan[i] = np.percentile(bin_pixels, 20)
            else:
                virtual_scan[i] = self.max_depth

        return virtual_scan

    def find_best_gap(
        self,
        cleaned_scan: np.ndarray,
        front_stop: float = None,
        min_gap_w: float = None,
        active_hfov: float = None,
    ):
        """Follow-the-Gap gap detection — paper-correct implementation.

        Strategy (Sezer & Gokasan 2012):
          1. Find all free-space runs above front_stop.
          2. Filter gaps narrower than the robot width (min_gap_w).
          3. Select the WIDEST gap (most room to manoeuvre).
          4. Within that gap, target the DEEPEST point (max distance)
             to steer toward the most open space.
          5. A lightweight score array is returned for telemetry/debug.
        """
        if front_stop is None:
            front_stop = self.front_stop
        if min_gap_w is None:
            min_gap_w = self.min_gap_w
        if active_hfov is None:
            active_hfov = self.hfov

        scores = np.zeros(self.num_bins, dtype=np.float32)
        best_bin = None

        # Identify free-space runs
        free_mask = cleaned_scan > front_stop
        gaps = []
        i, n = 0, free_mask.size
        while i < n:
            if free_mask[i]:
                s = i
                while i + 1 < n and free_mask[i + 1]:
                    i += 1
                gaps.append((s, i))
            i += 1

        # Filter by minimum physical width. Straight corridors can be narrower
        # than turns because the TurtleBot4 sweeps its corners while rotating.
        candidate_gaps = []
        center_bin = (self.num_bins - 1) / 2.0
        for s, e in gaps:
            gap_width_m = sum(
                2.0 * cleaned_scan[idx] * np.tan(active_hfov / (2.0 * self.num_bins))
                for idx in range(s, e + 1)
            )
            if gap_width_m < min_gap_w:
                continue

            gap_ranges = cleaned_scan[s: e + 1]
            max_d = float(np.max(gap_ranges))
            plateau_mask = gap_ranges >= max_d - 0.05
            plateau_indices = np.where(plateau_mask)[0]
            gap_center = len(gap_ranges) // 2
            best_local = plateau_indices[np.argmin(np.abs(plateau_indices - gap_center))]
            candidate_bin = s + int(best_local)
            target_angle = abs((center_bin - candidate_bin) * (active_hfov / self.num_bins))
            required_gap = (
                self.min_gap_w_turn
                if target_angle > self.hard_turn_yaw_threshold
                else self.min_gap_w_straight
            )
            if gap_width_m >= required_gap:
                candidate_gaps.append((s, e, gap_width_m, candidate_bin))

        valid_gaps = [(s, e, w) for s, e, w, _ in candidate_gaps]
        if not candidate_gaps:
            return scores, None, valid_gaps

        # Prefer going straight when a near-widest gap exists near the center.
        widest = max(g[2] for g in candidate_gaps)
        near_widest = [g for g in candidate_gaps if g[2] >= widest * 0.85]
        s, e, width_m, best_bin = min(
            near_widest,
            key=lambda g: abs(g[3] - center_bin),
        )

        # Populate scores for telemetry (1 = selected gap, 0.5 = other valid gaps)
        for gs, ge, _ in valid_gaps:
            scores[gs: ge + 1] = 0.5
        scores[best_bin] = 1.0

        return scores, best_bin, valid_gaps

    def compute_control(
        self,
        best_bin: int,
        cleaned_scan: np.ndarray,
        active_hfov: float = None,
    ):
        """Proportional steering + adaptive speed.

        Uses active_hfov for the bin→angle mapping so the gain is correct
        regardless of whether the scan was built with 69° or 180° FOV.
        Bypasses temporal smoothing when the required heading change is large
        (emergency turn, Bug D fix).
        """
        if active_hfov is None:
            active_hfov = self.hfov

        center_bin = (self.num_bins - 1) / 2.0

        # Bin → angle: positive = left, negative = right (REP-103)
        target_angle = (center_bin - best_bin) * (active_hfov / self.num_bins)

        # Bug D fix: skip EMA smoothing for emergency turns (large heading error)
        delta = abs(target_angle - self.last_target_angle)
        if delta > self.emergency_angle:
            smoothed_angle = target_angle  # immediate response
        else:
            smoothed_angle = (
                self.filter_alpha * target_angle
                + (1.0 - self.filter_alpha) * self.last_target_angle
            )
        self.last_target_angle = smoothed_angle

        yaw = float(np.clip(self.kp * smoothed_angle, -self.max_w, self.max_w))

        # Adaptive speed: slow near obstacles AND when turning sharply
        d_steer = float(cleaned_scan[best_bin])
        dist_factor = np.clip(
            (d_steer - self.front_stop) / max(self.max_depth - self.front_stop, 1e-3),
            0.0, 1.0
        )
        turn_factor = 1.0 - 0.6 * (abs(yaw) / self.max_w)  # more aggressive slowdown in turns
        speed = float(np.clip(
            self.min_v + (self.max_v - self.min_v) * dist_factor * turn_factor,
            self.min_v, self.max_v
        ))

        return speed, yaw

    # =====================================================================
    # LIDAR SAFETY LAYER
    # =====================================================================
    # The methods below implement an independent, mandatory safety layer
    # built on top of a real 2D LiDAR (sensor_msgs/LaserScan on /scan).
    # The camera-based planner above only reasons about what is inside the
    # OAK-D's ~69 degree horizontal field of view; it is structurally blind
    # to walls that are to the side, behind a corner, or otherwise outside
    # that cone. The LiDAR normally covers 360 degrees around the robot, so
    # it is used here strictly as a brake/veto: it can zero the speed, scale
    # it down, or cancel a turn, but it never invents extra speed or steering.
    # =====================================================================

    def scan_callback(self, msg: LaserScan):
        """
        Stores the latest LaserScan so apply_lidar_safety()/sector_min() can
        use it. Kept intentionally minimal (no heavy processing here) so the
        safety layer always sees the freshest possible range data.
        """
        self.last_scan_msg_time = time.monotonic()
        # range_min/range_max define the sensor's trustworthy measurement
        # envelope; fall back to sane defaults if the driver reports zeros.
        self.last_scan_range_min = float(msg.range_min) if msg.range_min > 0.0 else 0.02
        self.last_scan_range_max = float(msg.range_max) if msg.range_max > 0.0 else 30.0
        self.last_scan_angle_min = float(msg.angle_min)
        self.last_scan_angle_increment = (
            float(msg.angle_increment) if msg.angle_increment != 0.0 else 1e-6
        )
        self.last_scan_ranges = np.asarray(msg.ranges, dtype=np.float32)

    def sector_min(self, start_deg: float, end_deg: float) -> float:
        """
        Returns the closest *trustworthy* range (meters) within an angular
        sector of the most recent LaserScan.

        Convention: 0 deg = straight ahead, positive = left (CCW), negative
        = right, matching REP-103 / standard LaserScan orientation.

        Invalid-data handling (deliberately conservative, mirrors the depth
        camera policy in preprocess_depth):
          - NaN samples            -> excluded from the min (no information).
          - value < range_min      -> sensor below its measurable floor,
                                       which typically means an object is
                                       extremely close; clamped to range_min
                                       (treated as a close obstacle, NOT
                                       ignored and NOT treated as free).
          - value > range_max/Inf  -> standard "no return within sensor
                                       range" reading; this is legitimate
                                       information that the sector is clear
                                       up to range_max, so it is clamped to
                                       range_max (free), NOT max-range-as-huge.
          - sector has zero valid  -> unknown sector; return 0.0 (worst case)
            samples at all            so callers fail safe (stop) instead of
                                       assuming clearance with no evidence.
        """
        if self.last_scan_ranges is None or self.last_scan_ranges.size == 0:
            return 0.0  # No LiDAR data yet -> cannot verify safety -> unsafe.

        ranges = self.last_scan_ranges
        n = ranges.size
        angle_min = self.last_scan_angle_min
        angle_inc = self.last_scan_angle_increment

        start_rad = np.radians(start_deg)
        end_rad = np.radians(end_deg)

        i_start = int(round((start_rad - angle_min) / angle_inc))
        i_end = int(round((end_rad - angle_min) / angle_inc))
        if i_start > i_end:
            i_start, i_end = i_end, i_start
        i_start = max(0, min(n - 1, i_start))
        i_end = max(0, min(n - 1, i_end))

        sector = ranges[i_start : i_end + 1]
        if sector.size == 0:
            return 0.0

        valid = sector[~np.isnan(sector)]
        if valid.size == 0:
            return 0.0  # Entire sector unreadable -> treat as unsafe.

        range_min = self.last_scan_range_min
        range_max = self.last_scan_range_max

        too_close_mask = valid < range_min
        far_mask = (~too_close_mask) & (np.isinf(valid) | (valid > range_max))

        processed = np.where(too_close_mask, range_min, valid)
        processed = np.where(far_mask, range_max, processed)

        return float(np.min(processed))

    def apply_lidar_safety(self, speed: float, yaw: float):
        """
        Mandatory LiDAR safety gate. Every (speed, yaw) command computed by
        the camera-based planner MUST pass through this function before
        publish_cmd() is called.

        Sectors (degrees, 0 = forward, + = left, - = right):
          front       : -front_sector_deg .. +front_sector_deg
          front-left  :  20 .. 80
          front-right : -80 .. -20
          left        :  70 .. 110
          right       : -110 .. -70

        Narrow corridors are allowed only when the robot fits, the front is
        clear, and the command is mostly straight. Turns require extra corner
        clearance because the rectangular base sweeps more space while rotating.
        """
        # Layer 0: no LiDAR data -> full stop.
        if self.last_scan_ranges is None:
            return 0.0, 0.0

        now = time.monotonic()
        if self.last_scan_msg_time is None or (now - self.last_scan_msg_time) > self.lidar_timeout:
            self.get_logger().warn("[LIDAR SAFETY] /scan data is stale -> STOP")
            return 0.0, 0.0

        front       = self.sector_min(-self.front_sector_deg, self.front_sector_deg)
        front_center = self.sector_min(-12.0, 12.0)
        front_left  = self.sector_min(20.0, 80.0)
        front_right = self.sector_min(-80.0, -20.0)
        left        = self.sector_min(70.0, 110.0)
        right       = self.sector_min(-110.0, -70.0)

        safe_speed = speed
        safe_yaw   = yaw

        # --- Layer 1: Hard frontal stop -----------------------------------
        if front_center <= self.front_stop or front <= self.lidar_front_stop:
            self.get_logger().warn(
                f"[LIDAR SAFETY] FRONT STOP: center={front_center:.2f}m "
                f"wide={front:.2f}m"
            )
            safe_speed = 0.0
            if abs(safe_yaw) < 0.05:
                left_clear = max(left, front_left)
                right_clear = max(right, front_right)
                safe_yaw = (1.0 if left_clear >= right_clear else -1.0) * min(0.35, self.max_w * 0.35)

        # --- Layer 2: Progressive frontal slowdown -------------------------
        slow_front = min(front, front_center)
        if slow_front < self.lidar_slow_distance:
            span = max(self.lidar_slow_distance - self.lidar_front_stop, 1e-3)
            scale = float(np.clip((slow_front - self.lidar_front_stop) / span, 0.0, 1.0))
            safe_speed = min(safe_speed, self.max_v * scale)
            if front_center <= self.front_stop and abs(safe_yaw) < 0.05:
                left_clear = max(left, front_left)
                right_clear = max(right, front_right)
                safe_yaw = (1.0 if left_clear >= right_clear else -1.0) * min(0.35, self.max_w * 0.35)

        corridor_width = left + right
        corridor_fits = corridor_width >= max(
            self.min_corridor_width,
            self.robot_width_m + 2.0 * self.straight_side_margin,
        )
        left_ok_for_straight = left > self.side_hard_min
        right_ok_for_straight = right > self.side_hard_min
        going_mostly_straight = abs(safe_yaw) < self.corridor_mode_yaw_limit
        corridor_front_clear = front_center > self.front_stop and front > self.lidar_front_stop
        narrow_corridor_ok = (
            corridor_fits
            and left_ok_for_straight
            and right_ok_for_straight
            and corridor_front_clear
            and going_mostly_straight
        )

        # --- Layer 3: Narrow corridor handling -----------------------------
        if narrow_corridor_ok:
            safe_speed = min(safe_speed, self.narrow_corridor_speed)
            center_error = left - right
            safe_yaw += self.corridor_centering_gain * center_error
            safe_yaw = float(np.clip(safe_yaw, -self.max_w * 0.45, self.max_w * 0.45))
        elif left <= self.side_hard_min or right <= self.side_hard_min:
            self.get_logger().warn(
                f"[LIDAR SAFETY] SIDE TOO CLOSE FOR STRAIGHT: "
                f"left={left:.2f}m right={right:.2f}m <= hard_min={self.side_hard_min:.2f}m"
            )
            safe_speed = 0.0

        # --- Layer 4: Block dangerous turns --------------------------------
        turning_hard = abs(safe_yaw) > self.hard_turn_yaw_threshold
        near_wall_for_turn = min(left, right, front_left, front_right) < self.side_turn_min
        if turning_hard and near_wall_for_turn:
            self.get_logger().warn(
                f"[LIDAR SAFETY] HARD TURN NEAR WALL: left={left:.2f}m "
                f"right={right:.2f}m front_left={front_left:.2f}m "
                f"front_right={front_right:.2f}m < turn_min={self.side_turn_min:.2f}m"
            )
            safe_speed = 0.0

        diag_threshold = max(self.front_corner_clearance, self.side_turn_min)
        if safe_yaw > 0.0:  # Turning left
            if turning_hard:
                left_lat_block = left < self.side_turn_min
                left_diag_block = front_left < diag_threshold
            else:
                left_lat_block = left < self.side_hard_min
                left_diag_block = front_left < self.lidar_front_stop
            if left_lat_block or left_diag_block:
                self.get_logger().warn(
                    f"[LIDAR SAFETY] LEFT TURN BLOCKED: front_left={front_left:.2f}m "
                    f"left={left:.2f}m"
                )
                safe_yaw = 0.0
        if safe_yaw < 0.0:  # Turning right
            if turning_hard:
                right_lat_block = right < self.side_turn_min
                right_diag_block = front_right < diag_threshold
            else:
                right_lat_block = right < self.side_hard_min
                right_diag_block = front_right < self.lidar_front_stop
            if right_lat_block or right_diag_block:
                self.get_logger().warn(
                    f"[LIDAR SAFETY] RIGHT TURN BLOCKED: front_right={front_right:.2f}m "
                    f"right={right:.2f}m"
                )
                safe_yaw = 0.0

        # --- Layer 5: In-place stuck recovery ------------------------------
        blocked_or_stuck = (
            safe_speed == 0.0
            and safe_yaw == 0.0
            and (
                front_center <= self.front_stop
                or front <= self.lidar_front_stop
                or left <= self.side_hard_min
                or right <= self.side_hard_min
                or front_left <= self.front_corner_clearance
                or front_right <= self.front_corner_clearance
            )
        )
        if blocked_or_stuck:
            left_clear = max(left, front_left)
            right_clear = max(right, front_right)
            left_turn_ok = left >= self.side_hard_min and front_left >= self.lidar_front_stop
            right_turn_ok = right >= self.side_hard_min and front_right >= self.lidar_front_stop

            if left_turn_ok and right_turn_ok:
                escape_dir = 1.0 if left_clear >= right_clear else -1.0
            elif left_turn_ok:
                escape_dir = 1.0
            elif right_turn_ok:
                escape_dir = -1.0
            else:
                self.get_logger().warn(
                    f"[LIDAR SAFETY] STUCK: no safe in-place turn "
                    f"(left={left:.2f}m right={right:.2f}m "
                    f"front_left={front_left:.2f}m front_right={front_right:.2f}m)"
                )
                escape_dir = 0.0

            safe_speed = 0.0
            safe_yaw = escape_dir * min(0.35, self.max_w * 0.35)
            if escape_dir != 0.0:
                self.get_logger().warn(
                    f"[LIDAR SAFETY] STUCK RECOVERY: turning "
                    f"{'left' if escape_dir > 0 else 'right'} "
                    f"(left_clear={left_clear:.2f}m right_clear={right_clear:.2f}m)"
                )

        if (front_center <= self.front_stop or front <= self.lidar_front_stop) and safe_speed > 0.0:
            self.get_logger().warn(
                f"[LIDAR SAFETY] FRONT VETO: refusing forward speed with "
                f"center={front_center:.2f}m wide={front:.2f}m"
            )
            safe_speed = 0.0

        safe_speed = float(np.clip(safe_speed, 0.0, self.max_v))
        safe_yaw   = float(np.clip(safe_yaw, -self.max_w, self.max_w))
        return safe_speed, safe_yaw

    def _build_lidar_frontal_scan(self, active_hfov: float = None) -> np.ndarray:
        """Project the LiDAR frontal arc (±active_hfov/2) into num_bins.

        Returns a metric virtual scan [m] identical in format to the stereo
        depth virtual scan.  Each bin takes the minimum range of all LiDAR
        rays that fall inside its angular slice (conservative: closest obstacle
        wins).  Bins with no rays default to max_depth (free space).

        Using the real LiDAR for planning instead of RGB pseudo-depth gives
        accurate obstacle positions → correct gap detection → sharper, timely
        turns.
        """
        if active_hfov is None:
            active_hfov = self.lidar_planning_fov

        if self.last_scan_ranges is None:
            return np.full(self.num_bins, self.max_depth, dtype=np.float32)

        ranges = self.last_scan_ranges.copy()
        n = ranges.size
        angle_min = self.last_scan_angle_min
        angle_inc = self.last_scan_angle_increment
        angles = angle_min + np.arange(n) * angle_inc

        # Only keep rays inside the active FOV
        hfov_half = active_hfov / 2.0
        mask = np.abs(angles) <= hfov_half
        fov_angles = angles[mask]
        fov_ranges = ranges[mask]

        # Clean invalid readings. NaN/-Inf are unsafe; +Inf means no return in range.
        fov_ranges = np.nan_to_num(
            fov_ranges,
            nan=self.min_depth,
            posinf=self.max_depth,
            neginf=self.min_depth,
        )
        fov_ranges = np.clip(fov_ranges, self.min_depth, self.max_depth)

        # Map each ray to a bin.
        # angle=+hfov_half -> bin 0           left
        # angle=0          -> bin num_bins//2 center
        # angle=-hfov_half -> bin num_bins-1  right
        virtual_scan = np.full(self.num_bins, self.max_depth, dtype=np.float32)
        if fov_angles.size == 0:
            return virtual_scan

        bin_width = active_hfov / self.num_bins
        bin_indices = np.clip(
            ((hfov_half - fov_angles) / bin_width).astype(int),
            0, self.num_bins - 1,
        )
        for idx, r in zip(bin_indices, fov_ranges):
            if r < virtual_scan[idx]:
                virtual_scan[idx] = r

        return virtual_scan

    def rgb_callback(self, msg: Image):
        """RGB fallback callback: confirms the camera is alive, then plans
        using the LiDAR frontal projection (metric, accurate) instead of
        RGB pseudo-depth.

        The RGB topic being alive is the condition for rgb_fallback mode.
        Planning with LiDAR data gives correct obstacle distances → the robot
        turns at the right time and with the right radius.
        """
        self.last_rgb_msg_time = time.monotonic()

        if self.camera_mode != "rgb_fallback":
            return

        # Use LiDAR frontal projection as the virtual scan (metric accuracy)
        # using the 180° planning FOV.
        virtual_scan = self._build_lidar_frontal_scan(self.lidar_planning_fov)

        self._process_virtual_scan(
            virtual_scan,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            active_hfov=self.lidar_planning_fov,
        )

    def _update_camera_mode(self):
        """Auto-detect camera mode based on which topics are delivering frames.

        Called periodically from safety_watchdog.  Priority:
          stereo      -- stereo depth frames are arriving (best)
          rgb_fallback-- no stereo, but RGB frames are arriving
          lidar_only  -- no camera frames at all (LiDAR-only navigation)
        """
        now = time.monotonic()
        # Only re-evaluate at stereo_check_interval to avoid log spam.
        if now - self._last_stereo_check < self.stereo_check_interval:
            return
        self._last_stereo_check = now

        stereo_ok = (
            self.last_depth_msg_time is not None
            and (now - self.last_depth_msg_time) <= self.camera_timeout
        )
        rgb_ok = (
            self.last_rgb_msg_time is not None
            and (now - self.last_rgb_msg_time) <= self.camera_timeout
        )

        if stereo_ok:
            new_mode = "stereo"
        elif rgb_ok:
            new_mode = "rgb_fallback"
        else:
            new_mode = "lidar_only"

        if new_mode != self.camera_mode:
            self.camera_mode = new_mode
            self.get_logger().warn(
                f"[MODE] Camera mode changed -> {new_mode}. "
                f"stereo_ok={stereo_ok} rgb_ok={rgb_ok}"
            )
            self._send_log("WARN", f"camera_mode={new_mode}")

    def _lidar_only_navigate(self):
        """Minimal Follow-the-Gap planner that runs entirely from the LiDAR
        scan when no camera data is available. Projects LiDAR to 180° planning FOV
        and delegates to the unified pipeline.
        """
        if self.last_scan_ranges is None:
            self.publish_cmd(0.0, 0.0)
            return

        # Use LiDAR frontal projection as the virtual scan (metric accuracy)
        # using the 180° planning FOV.
        virtual_scan = self._build_lidar_frontal_scan(self.lidar_planning_fov)

        now_sec = int(self.get_clock().now().nanoseconds / 1e9)
        now_nsec = int(self.get_clock().now().nanoseconds % 1e9)

        self._process_virtual_scan(
            virtual_scan,
            now_sec,
            now_nsec,
            active_hfov=self.lidar_planning_fov,
        )

    def safety_watchdog(self):
        """
        Independent safety timer. Behavior by sensor availability:

          LiDAR OK + camera OK   -> normal operation (mode: stereo or rgb_fallback)
          LiDAR OK + camera FAIL -> degraded navigation with LiDAR only (no stop)
          LiDAR FAIL             -> FULL STOP regardless of camera state

        The camera is treated as optional: losing it downgrades the mode but
        does NOT stop the robot.  The LiDAR is mandatory for safety.
        """
        now = time.monotonic()

        # --- Update camera mode (stereo / rgb_fallback / lidar_only) ---
        self._update_camera_mode()

        lidar_stale = (
            self.last_scan_msg_time is None
            or (now - self.last_scan_msg_time) > self.lidar_timeout
        )

        # LiDAR failure -> full stop (mandatory safety sensor)
        if lidar_stale:
            self.get_logger().warn("[WATCHDOG] Forcing ZERO velocity: lidar_stale_or_missing")
            self.publish_cmd(0.0, 0.0)
            self._send_log("WARN", "watchdog_stop reasons=lidar_stale_or_missing")
            return

        # LiDAR is OK: if we are in lidar_only mode, drive with LiDAR planner.
        if self.camera_mode == "lidar_only":
            self._lidar_only_navigate()

    def publish_cmd(self, speed: float, yaw: float):
        """Publishes the command velocity TwistStamped message."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(speed)
        msg.twist.angular.z = float(yaw)
        self.pub_cmd.publish(msg)

    def draw_debug(self, cv_depth, virtual_scan, cleaned_scan, best_bin, scores, debug_gaps, speed, yaw):
        """Generates a debug window with the processed data overlaid on colorized depth."""
        # 1. Colorize depth for visual clarity (Jet colormap)
        depth_norm = np.clip((cv_depth - self.min_depth) / (self.max_depth - self.min_depth) * 255.0, 0, 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        height, width, _ = depth_color.shape
        bin_w = width // self.num_bins

        # Draw ROI Boundary (green lines)
        cv2.line(depth_color, (0, self.roi_top), (width, self.roi_top), (0, 255, 0), 2)
        cv2.line(depth_color, (0, self.roi_bottom), (width, self.roi_bottom), (0, 255, 0), 2)

        # Highlight Gaps and Safety Bubbles
        # Draw all bins
        for i in range(self.num_bins):
            x_start = i * bin_w
            x_end = (i + 1) * bin_w
            
            # Draw bin column dividers (subtle gray)
            cv2.line(depth_color, (x_start, self.roi_top), (x_start, self.roi_bottom), (100, 100, 100), 1)

            # Draw virtual scan depth profile line (white dots)
            d = virtual_scan[i]
            y_plot = int(self.roi_bottom - (d / self.max_depth) * (self.roi_bottom - self.roi_top))
            cv2.circle(depth_color, (x_start + bin_w // 2, y_plot), 3, (255, 255, 255), -1)

            # Draw cleaned scan profile line (showing bubble zeroed out area in red)
            d_clean = cleaned_scan[i]
            if d_clean == 0.0:
                cv2.rectangle(depth_color, (x_start, self.roi_top), (x_end, self.roi_bottom), (0, 0, 150), -1) # Shaded red

        # Draw valid gaps (shaded green overlays)
        for s, e, w_m in debug_gaps:
            cv2.rectangle(depth_color, (s * bin_w, self.roi_top), ((e + 1) * bin_w, self.roi_bottom), (0, 100, 0), 2)
            # Text showing gap width
            mid_x = (s + e) * bin_w // 2
            cv2.putText(depth_color, f"{w_m:.2f}m", (mid_x - 15, self.roi_top + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # Highlight Best Target Bin (yellow vertical bar and arrow)
        if best_bin is not None:
            bx = best_bin * bin_w + bin_w // 2
            cv2.line(depth_color, (bx, self.roi_top), (bx, self.roi_bottom), (0, 255, 255), 3)
            # Arrow pointing in the steering direction
            center_x = width // 2
            cv2.arrowedLine(depth_color, (center_x, height - 20), (bx, height - 40), (0, 255, 255), 2, tipLength=0.3)

        # Center line (dashed white)
        cv2.line(depth_color, (width // 2, 0), (width // 2, height), (200, 200, 200), 1, cv2.LINE_AA)

        # Text Overlay
        cv2.putText(depth_color, f"FPS: {self.fps:.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(depth_color, f"Linear v: {speed:.2f} m/s", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(depth_color, f"Angular w: {yaw:+.2f} rad/s", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        state_str = "FORWARD" if speed > 0.0 else "BLOCKED / REALIGN"
        cv2.putText(depth_color, f"State: {state_str}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if speed > 0.0 else (0, 0, 255), 2)

        cv2.imshow("Follow The Gap Debug", depth_color)
        cv2.waitKey(1)

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
            self.get_logger().info(f"Depth telemetry paired with {addr}.")
            self._send_log("INFO", f"paired addr={addr[0]}:{addr[1]} depth_topic={self.depth_topic} cmd_topic={self.cmd_topic}")

    def _send_telemetry_state(
        self,
        stamp_sec,
        stamp_nsec,
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
        virtual_scan
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
            str(stamp_sec),
            str(stamp_nsec),
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
                self._send_virtual_scan_array(stamp_sec, stamp_nsec, virtual_scan)
        except OSError as e:
            self.get_logger().warn(f"Error sending telemetry: {e}")

    def _send_log(self, level: str, message: str):
        if self.authorized_addr is None:
            return
        fields = ["LOG", str(self.ros_domain_id), self.robot_name, level, message]
        try:
            self.sock.sendto(" ".join(fields).encode("utf-8"), self.authorized_addr)
        except OSError as e:
            self.get_logger().warn(f"Error sending UDP log: {e}")

    def _send_virtual_scan_array(self, stamp_sec, stamp_nsec, virtual_scan):
        # We reverse the order of virtual scan so the array starts from negative angle (right side)
        # to match the range-array format expected by telemetry listeners
        ranges = virtual_scan[::-1][::self.scan_array_stride]
        angle_min = -self.hfov / 2.0
        angle_increment = self.hfov / self.num_bins
        fields = [
            "SCAN_ARRAY",
            str(self.ros_domain_id),
            self.robot_name,
            str(stamp_sec),
            str(stamp_nsec),
            f"{angle_min:.6f}",
            f"{angle_increment:.6f}",
            str(self.scan_array_stride),
            str(len(ranges)),
        ]
        fields.extend(self._range_text(r) for r in ranges)
        self.sock.sendto(" ".join(fields).encode("utf-8"), self.authorized_addr)

    def _status_check(self):
        image_topics = [
            name
            for name, types in self.get_topic_names_and_types()
            if name.startswith("/oakd") and "sensor_msgs/msg/Image" in types
        ]
        published_image_topics = [
            f"{name}({len(self.get_publishers_info_by_topic(name))})"
            for name in image_topics
            if self.get_publishers_info_by_topic(name)
        ]
        depth_publishers = len(self.get_publishers_info_by_topic(self.depth_topic))

        if self.last_depth_msg_time is None:
            if depth_publishers == 0:
                topic_state = "no_depth_publisher"
            elif self.depth_topic in image_topics:
                topic_state = "depth_publisher_no_frames"
            else:
                topic_state = "topic_not_visible"
            msg = (
                f"No depth frames received on {self.depth_topic}; "
                f"state={topic_state}; depth_publishers={depth_publishers}; "
                f"published_image_topics={','.join(published_image_topics) if published_image_topics else 'none'}"
            )
            self.get_logger().warn(msg)
            self._send_log("WARN", msg)

        scan_publishers = len(self.get_publishers_info_by_topic(self.scan_topic))
        if self.last_scan_msg_time is None:
            topic_state = "no_lidar_publisher" if scan_publishers == 0 else "lidar_publisher_no_frames"
            msg = (
                f"No LiDAR scans received on {self.scan_topic}; state={topic_state}; "
                f"lidar_publishers={scan_publishers}. Robot will remain stopped "
                f"(mandatory LiDAR safety layer requires live /scan data)."
            )
            self.get_logger().warn(msg)
            self._send_log("WARN", msg)

        if self.pub_cmd.get_subscription_count() == 0:
            msg = f"No subscribers detected on {self.cmd_topic}; Create 3 will not receive velocity commands."
            self.get_logger().warn(msg)
            self._send_log("WARN", msg)

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

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send safety stop command
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
