#!/usr/bin/env python3
"""
Autonomous Reactive Navigation Node for TurtleBot 4 (Create 3 base, OAK-D camera)
using Stereo Depth for local planning AND a 2D LiDAR as a mandatory safety layer.
Works on ROS 2 JAZZY.

Subscribes:  /oakd/stereo/image_raw   (sensor_msgs/Image, depth in mm or meters)
             /scan                    (sensor_msgs/LaserScan, real 2D LiDAR)
Publishes:   /cmd_vel                 (geometry_msgs/TwistStamped)

Architecture (two independent layers):
  1. PLANNER (camera, depth-only): Follow-the-Gap over a virtual scan built
     from the OAK-D depth image. Decides *where* to go (best gap / heading)
     and a nominal speed. This layer only sees what is inside the camera's
     narrow field of view.
  2. SAFETY (LiDAR, 360-degree real range data): every command produced by
     the planner is passed through apply_lidar_safety() before publishing.
     This layer can only brake, slow down, or cancel a turn -- it can never
     make the robot go faster or turn harder than the planner requested.
     Its job is to catch walls/obstacles that are out of the camera's FOV
     (to the side, behind a corner, etc).
  A safety_watchdog timer independently forces a zero-velocity command if
  either sensor stream goes stale, regardless of what the planner/safety
  layers computed on their last valid cycle.
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


class FollowTheGapDepth(Node):
    def __init__(self):
        super().__init__("follow_the_gap_depth")

        # ---- Declare ROS 2 Parameters ----
        self.declare_parameter("depth_topic", "/oakd/stereo/image_raw")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("number_of_bins", 72)
        self.declare_parameter("roi_top", 280)         # Top row of ROI (assuming 640x480)
        self.declare_parameter("roi_bottom", 440)      # Bottom row of ROI
        self.declare_parameter("max_depth", 4.0)       # Maximum reliable depth (m)
        self.declare_parameter("min_depth", 0.4)       # Minimum depth (m)
        self.declare_parameter("bubble_radius_min", 0.35) # Safe min bubble (m)
        self.declare_parameter("bubble_radius_max", 0.90) # Max bubble for close obstacles (m)
        self.declare_parameter("bubble_k", 0.15)       # Scaling factor for dynamic bubble
        self.declare_parameter("kp", 1.2)              # Proportional gain for yaw steering
        self.declare_parameter("max_linear_speed", 0.20)  # m/s (reduced for safe testing)
        self.declare_parameter("min_linear_speed", 0.03)  # m/s (reduced for safe testing)
        self.declare_parameter("max_angular_speed", 0.75)  # rad/s (reduced for safe testing)
        self.declare_parameter("distance_weight", 1.0) # alpha
        self.declare_parameter("gap_weight", 0.8)      # beta
        self.declare_parameter("steering_weight", 0.5)  # gamma
        self.declare_parameter("closeness_weight", 0.6) # delta
        self.declare_parameter("temporal_filter_alpha", 0.25)
        self.declare_parameter("median_kernel", 3)     # 1D median filter kernel size for bins
        self.declare_parameter("front_stop_distance", 0.65) # Stop and realign if wall closer than this (m)
        self.declare_parameter("minimum_gap_width", 0.70)  # Min traversable width (m)
        self.declare_parameter("hfov_deg", 69.0)       # OAK-D RGB-Depth Horizontal FOV
        self.declare_parameter("show_debug", False)
        self.declare_parameter("telemetry_port", 6000)
        self.declare_parameter("telemetry_hz", 5.0)
        self.declare_parameter("send_scan_array", True)
        self.declare_parameter("scan_array_stride", 1)
        self.declare_parameter("robot_name", "turtlebot4_rensso_mora")
        self.declare_parameter("pairing_code", "ROBOT_A_2")

        # ---- LiDAR safety-layer parameters (mandatory obstacle layer) ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("lidar_timeout", 1.0)     # s; no /scan -> watchdog stop
        self.declare_parameter("camera_timeout", 1.0)    # s; no depth frames -> watchdog stop
        self.declare_parameter("lidar_front_stop", 0.30)     # m; hard frontal stop distance
        self.declare_parameter("lidar_slow_distance", 0.70)  # m; start slowing down frontally
        self.declare_parameter("lidar_side_stop", 0.22)      # m; too close to a side wall -> stop advancing
        self.declare_parameter("lidar_turn_stop", 0.35)      # m; block turning into a near diagonal/side wall
        self.declare_parameter("front_sector_deg", 35.0)     # deg; front sector half-angle (-deg..+deg)

        # ---- Retrieve Parameters ----
        g = lambda n: self.get_parameter(n).value
        self.depth_topic = str(g("depth_topic"))
        self.cmd_topic = str(g("cmd_topic"))
        self.num_bins = int(g("number_of_bins"))
        self.roi_top = int(g("roi_top"))
        self.roi_bottom = int(g("roi_bottom"))
        self.max_depth = float(g("max_depth"))
        self.min_depth = float(g("min_depth"))
        self.bubble_rad_min = float(g("bubble_radius_min"))
        self.bubble_rad_max = float(g("bubble_radius_max"))
        self.bubble_k = float(g("bubble_k"))
        self.kp = float(g("kp"))
        self.max_v = float(g("max_linear_speed"))
        self.min_v = float(g("min_linear_speed"))
        self.max_w = float(g("max_angular_speed"))
        self.alpha = float(g("distance_weight"))
        self.beta = float(g("gap_weight"))
        self.gamma = float(g("steering_weight"))
        self.delta = float(g("closeness_weight"))
        self.filter_alpha = float(g("temporal_filter_alpha"))
        self.med_kernel = int(g("median_kernel"))
        self.front_stop = float(g("front_stop_distance"))
        self.min_gap_w = float(g("minimum_gap_width"))
        self.hfov = np.radians(float(g("hfov_deg")))
        self.show_debug = bool(g("show_debug"))
        if self.show_debug and not os.environ.get("DISPLAY"):
            self.get_logger().warn("show_debug=true but DISPLAY is not set; disabling OpenCV window.")
            self.show_debug = False
        self.telemetry_period = 1.0 / max(float(g("telemetry_hz")), 0.1)
        self.send_scan_array = bool(g("send_scan_array"))
        self.scan_array_stride = max(int(g("scan_array_stride")), 1)
        self.robot_name = str(g("robot_name"))
        self.pairing_code = str(g("pairing_code"))

        # ---- LiDAR safety-layer parameters ----
        self.scan_topic = str(g("scan_topic"))
        self.lidar_timeout = float(g("lidar_timeout"))
        self.camera_timeout = float(g("camera_timeout"))
        self.lidar_front_stop = float(g("lidar_front_stop"))
        self.lidar_slow_distance = float(g("lidar_slow_distance"))
        self.lidar_side_stop = float(g("lidar_side_stop"))
        self.lidar_turn_stop = float(g("lidar_turn_stop"))
        self.front_sector_deg = float(g("front_sector_deg"))

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

        # ---- LiDAR internal state (populated by scan_callback) ----
        self.last_scan_ranges = None          # np.ndarray of raw ranges (meters)
        self.last_scan_msg_time = None        # monotonic time of last /scan message
        self.last_scan_angle_min = 0.0
        self.last_scan_angle_increment = 1.0
        self.last_scan_range_min = 0.02
        self.last_scan_range_max = 30.0

        # ---- Subscriber & Publisher ----
        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data
        )
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.pub_cmd = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.status_timer = self.create_timer(5.0, self._status_check)

        # Independent safety watchdog: runs on its own timer (faster than the
        # camera callback rate) so a dead camera or dead LiDAR topic cannot
        # leave the robot coasting on the last command it ever received.
        self.watchdog_timer = self.create_timer(0.15, self.safety_watchdog)

        self.get_logger().info("Follow-the-Gap Stereo Depth Navigation Node Initialized.")
        self.get_logger().info(f"Subscribed to depth: {self.depth_topic}")
        self.get_logger().info(f"Subscribed to LiDAR: {self.scan_topic}")
        self.get_logger().info(f"Publishing TwistStamped to: {self.cmd_topic}")

    def depth_callback(self, msg: Image):
        self.last_depth_msg_time = time.monotonic()

        # Calculate Loop Frequency (FPS)
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        # Convert Image Msg to OpenCV matrix
        try:
            # Depthai-ros often publishes depth in millimeters (16UC1) or meters (32FC1)
            if msg.encoding == "16UC1":
                cv_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                cv_depth = cv_raw.astype(np.float32) / 1000.0  # Convert mm to meters
            else:
                cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")
            self.publish_cmd(0.0, 0.0)  # Safety Stop
            return

        # 1. Preprocess Depth
        processed_depth = self.preprocess_depth(cv_depth)

        # 2. Extract ROI (Region of Interest)
        roi_depth = self.extract_roi(processed_depth)

        # 3 & 4. Build Virtual Scan (Bins) using Percentile 20
        virtual_scan = self.build_virtual_scan(roi_depth)

        # 5. Apply Med-Filter to Smooth Scan
        if self.med_kernel > 1:
            # 1D median filter to reject spurious obstacle pixels
            virtual_scan = cv2.medianBlur(virtual_scan.astype(np.float32), self.med_kernel).flatten()

        # Find closest obstacle and draw safety bubble
        nearest_idx = np.argmin(virtual_scan)
        d_min = float(virtual_scan[nearest_idx])

        # Dynamic safety bubble calculation
        # bubble_radius = robot_radius (0.20m) + margin (0.05m) + k / distance
        bubble_radius = 0.20 + 0.05 + (self.bubble_k / max(d_min, 0.1))
        bubble_radius = np.clip(bubble_radius, self.bubble_rad_min, self.bubble_rad_max)

        # Map bubble width in meters to number of bins at distance d_min
        # Physical width of 1 bin at distance d_min:
        bin_width_at_d = 2.0 * max(d_min, 0.1) * np.tan(self.hfov / (2.0 * self.num_bins))
        half_bins = int(np.ceil(bubble_radius / max(bin_width_at_d, 1e-3)))

        # Zero out the bins inside the safety bubble
        cleaned_scan = virtual_scan.copy()
        start_bubble = max(0, nearest_idx - half_bins)
        end_bubble = min(self.num_bins - 1, nearest_idx + half_bins)
        cleaned_scan[start_bubble : end_bubble + 1] = 0.0

        # 6. Compute Scores & Find Best Gap
        scores, best_bin, debug_gaps = self.find_best_gap(cleaned_scan)

        # 7. Compute Control & Publish
        if best_bin is not None:
            speed, yaw = self.compute_control(best_bin, cleaned_scan)
        else:
            # Blocked / No valid gap
            self.get_logger().warn("NO VALID GAP FOUND - STOPPING")
            speed, yaw = 0.0, 0.0

        # Safety Override (camera layer): If frontal clearance seen by the
        # depth camera is too low. This only protects against obstacles that
        # are inside the OAK-D's field of view.
        # Central bins are the middle 10% of the scan
        mid = self.num_bins // 2
        span = max(1, self.num_bins // 10)
        front_clearance = np.min(virtual_scan[mid - span : mid + span + 1])
        if front_clearance < self.front_stop:
            self.get_logger().warn(f"FRONT BLOCKED: {front_clearance:.2f}m < {self.front_stop}m. Stopping.")
            speed, yaw = 0.0, 0.0

        # ---------------------------------------------------------------
        # MANDATORY LIDAR SAFETY LAYER
        # This is the final authority before any command reaches the robot.
        # It uses the real 360-degree LiDAR to catch walls/obstacles that
        # are outside the camera's narrow field of view (to the side,
        # behind a corner, etc), which the planner above cannot see at all.
        # It can only brake, slow down, or cancel a turn -- never speed up.
        # ---------------------------------------------------------------
        speed, yaw = self.apply_lidar_safety(speed, yaw)

        self.publish_cmd(speed, yaw)

        # 8. Send UDP Telemetry
        # We simulate LIDAR structure for compatibility with recibidor_datos.py
        # Sectors: left / center / right, split proportionally to num_bins
        # (kept dynamic so it stays correct if number_of_bins is reconfigured).
        third = max(1, self.num_bins // 3)
        left_clear = float(np.min(virtual_scan[0:third]))
        front_clear = float(np.min(virtual_scan[third : 2 * third]))
        right_clear = float(np.min(virtual_scan[2 * third : self.num_bins]))
        
        target_angle_deg = float(np.degrees(self.last_target_angle))
        
        gap_start_deg = None
        gap_end_deg = None
        if best_bin is not None:
            # Find the gap containing the best bin
            for s, e, _ in debug_gaps:
                if s <= best_bin <= e:
                    center_bin = (self.num_bins - 1) / 2.0
                    gap_start_deg = float(np.degrees((center_bin - s) * (self.hfov / self.num_bins)))
                    gap_end_deg = float(np.degrees((center_bin - e) * (self.hfov / self.num_bins)))
                    break

        state_str = "FORWARD"
        if speed == 0.0:
            if front_clearance < self.front_stop:
                state_str = "FRONT_BLOCKED"
            else:
                state_str = "BLOCKED"

        self._send_telemetry_state(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            state_str,
            front_clear,
            left_clear,
            right_clear,
            d_min,
            float(np.degrees((self.num_bins // 2 - nearest_idx) * (self.hfov / self.num_bins))),
            gap_start_deg,
            gap_end_deg,
            target_angle_deg,
            speed,
            yaw,
            virtual_scan
        )

        # Debug Window visualization
        if self.show_debug:
            self.draw_debug(processed_depth, virtual_scan, cleaned_scan, best_bin, scores, debug_gaps, speed, yaw)

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

    def find_best_gap(self, cleaned_scan: np.ndarray):
        """
        Detects contiguous traversable gaps, calculates gap scores, and selects
        the best target bin using a multi-criteria cost function.
        Rejects gaps narrower than the robot's effective width (0.50m).
        """
        scores = np.zeros(self.num_bins, dtype=np.float32)
        best_bin = None
        best_score = -float("inf")

        # Threshold to define "free" space (must be > 0 and greater than stop distance)
        free_mask = (cleaned_scan > self.front_stop)

        # Identify contiguous runs of True in free_mask
        gaps = []
        i = 0
        n = free_mask.size
        while i < n:
            if free_mask[i]:
                s = i
                while i + 1 < n and free_mask[i + 1]:
                    i += 1
                gaps.append((s, i))
                i += 1
            else:
                i += 1

        valid_gaps = []
        # Check physical width of each gap in meters
        for s, e in gaps:
            # Sum of bin widths inside the gap at their respective depths
            # Physical width of bin i: w_i = 2 * d_i * tan(HFOV / (2 * num_bins))
            gap_width_meters = 0.0
            for idx in range(s, e + 1):
                d = cleaned_scan[idx]
                gap_width_meters += 2.0 * d * np.tan(self.hfov / (2.0 * self.num_bins))

            # Only consider gaps wider than the effective robot width (0.50m)
            if gap_width_meters >= self.min_gap_w:
                valid_gaps.append((s, e, gap_width_meters))

        # Evaluate score for each bin inside valid gaps
        center_bin = (self.num_bins - 1) / 2.0
        for s, e, width_m in valid_gaps:
            # Normalize gap width relative to typical safe corridors (max out at 2.0m)
            norm_width = min(width_m, 2.0) / 2.0

            for idx in range(s, e + 1):
                d = cleaned_scan[idx]
                norm_dist = d / self.max_depth

                # Steering penalty: turns closer to center are preferred (normalized 0 to 1)
                steer_penalty = abs(idx - center_bin) / center_bin

                # Closeness penalty: distance to nearest obstacle bin
                # Let's count how far we are from the boundaries of this gap
                dist_to_wall = min(idx - s, e - idx)
                closeness_penalty = 1.0 / (dist_to_wall + 1.0)

                # Cost function
                score = (
                    self.alpha * norm_dist
                    + self.beta * norm_width
                    - self.gamma * steer_penalty
                    - self.delta * closeness_penalty
                )
                
                scores[idx] = score

                if score > best_score:
                    best_score = score
                    best_bin = idx

        return scores, best_bin, valid_gaps

    def compute_control(self, best_bin: int, cleaned_scan: np.ndarray):
        """
        Calculates speed and angular velocity commands.
        Applies temporal steering smoothing and adaptive speed scaling.
        """
        center_bin = (self.num_bins - 1) / 2.0
        
        # Calculate target angle in radians
        # Left bins are positive angle, right bins are negative angle
        target_angle = (center_bin - best_bin) * (self.hfov / self.num_bins)

        # Apply exponential moving average filter for steering smoothness
        # target = alpha * target_actual + (1 - alpha) * target_anterior
        smoothed_angle = self.filter_alpha * target_angle + (1.0 - self.filter_alpha) * self.last_target_angle
        self.last_target_angle = smoothed_angle

        # Steering proportional control
        yaw = self.kp * smoothed_angle
        yaw = np.clip(yaw, -self.max_w, self.max_w)

        # Adaptive Speed Control:
        # Depends on frontal depth and steering angle
        d_steer = cleaned_scan[best_bin]
        
        # Distance factor: how clear is the way forward (0 to 1)
        dist_factor = (d_steer - self.front_stop) / (self.max_depth - self.front_stop)
        dist_factor = np.clip(dist_factor, 0.0, 1.0)

        # Turn factor: slow down when turning sharply
        turn_factor = 1.0 - 0.5 * (abs(yaw) / self.max_w)
        
        speed = self.min_v + (self.max_v - self.min_v) * dist_factor * turn_factor
        speed = np.clip(speed, self.min_v, self.max_v)

        return float(speed), float(yaw)

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
        publish_cmd() is called. It can only reduce/zero speed or cancel a
        turn -- it never increases what the planner requested.

        Sectors checked (degrees, 0 = forward, + = left, - = right):
          front       : -front_sector_deg .. +front_sector_deg
          front-left  :  20 .. 80
          front-right : -80 .. -20
          left        :  70 .. 110
          right       : -110 .. -70
        """
        # Layer 0: no LiDAR data at all -> cannot verify safety -> full stop.
        if self.last_scan_ranges is None:
            return 0.0, 0.0

        # Layer 0b: LiDAR data present but stale (sensor/driver died) -> stop.
        # (The independent safety_watchdog timer also catches this even if
        # the camera stops calling this function at all.)
        now = time.monotonic()
        if self.last_scan_msg_time is None or (now - self.last_scan_msg_time) > self.lidar_timeout:
            self.get_logger().warn("[LIDAR SAFETY] /scan data is stale -> STOP")
            return 0.0, 0.0

        front = self.sector_min(-self.front_sector_deg, self.front_sector_deg)
        front_left = self.sector_min(20.0, 80.0)
        front_right = self.sector_min(-80.0, -20.0)
        left = self.sector_min(70.0, 110.0)
        right = self.sector_min(-110.0, -70.0)

        safe_speed = speed
        safe_yaw = yaw

        # --- Layer 1: Hard frontal stop -------------------------------
        # A wall/obstacle is directly ahead within the hard stop distance:
        # full stop, no partial measures.
        if front <= self.lidar_front_stop:
            self.get_logger().warn(
                f"[LIDAR SAFETY] FRONT STOP: {front:.2f}m <= {self.lidar_front_stop:.2f}m"
            )
            return 0.0, 0.0

        # --- Layer 2: Progressive frontal slowdown ---------------------
        # Approaching a frontal wall: scale the commanded speed down
        # smoothly between lidar_slow_distance (start slowing) and
        # lidar_front_stop (full stop), regardless of what the camera
        # planner asked for.
        if front < self.lidar_slow_distance:
            span = max(self.lidar_slow_distance - self.lidar_front_stop, 1e-3)
            scale = float(np.clip((front - self.lidar_front_stop) / span, 0.0, 1.0))
            safe_speed = min(safe_speed, self.max_v * scale)

        # --- Layer 3: Lateral wall hugging -------------------------------
        # Too close to a side wall while moving: stop advancing so the
        # robot doesn't scrape/squeeze along a wall the camera can't see.
        if left <= self.lidar_side_stop or right <= self.lidar_side_stop:
            self.get_logger().warn(
                f"[LIDAR SAFETY] SIDE TOO CLOSE: left={left:.2f}m right={right:.2f}m "
                f"<= {self.lidar_side_stop:.2f}m -> stopping advance"
            )
            safe_speed = 0.0

        # --- Layer 4: Block dangerous turns -------------------------------
        # Never steer into a lateral/diagonal wall that is too close, even
        # if the planner requested that direction.
        if safe_yaw > 0.0 and (front_left <= self.lidar_turn_stop or left <= self.lidar_turn_stop):
            self.get_logger().warn(
                f"[LIDAR SAFETY] LEFT TURN BLOCKED: front_left={front_left:.2f}m "
                f"left={left:.2f}m <= {self.lidar_turn_stop:.2f}m"
            )
            safe_yaw = 0.0
        if safe_yaw < 0.0 and (front_right <= self.lidar_turn_stop or right <= self.lidar_turn_stop):
            self.get_logger().warn(
                f"[LIDAR SAFETY] RIGHT TURN BLOCKED: front_right={front_right:.2f}m "
                f"right={right:.2f}m <= {self.lidar_turn_stop:.2f}m"
            )
            safe_yaw = 0.0

        safe_speed = float(np.clip(safe_speed, 0.0, self.max_v))
        safe_yaw = float(np.clip(safe_yaw, -self.max_w, self.max_w))
        return safe_speed, safe_yaw

    def safety_watchdog(self):
        """
        Independent safety timer (runs regardless of camera/LiDAR callback
        activity). If the depth camera or the LiDAR stop delivering fresh
        data -- topic died, node crashed, network hiccup -- this forces a
        zero-velocity command immediately. Without this, a dead camera
        would mean depth_callback (and therefore apply_lidar_safety and
        publish_cmd) simply stop being called, leaving the robot coasting
        forever on the last command it ever published.
        """
        now = time.monotonic()
        reasons = []

        if self.last_depth_msg_time is None or (now - self.last_depth_msg_time) > self.camera_timeout:
            reasons.append("camera_stale_or_missing")

        if self.last_scan_msg_time is None or (now - self.last_scan_msg_time) > self.lidar_timeout:
            reasons.append("lidar_stale_or_missing")

        if reasons:
            reason_str = ",".join(reasons)
            self.get_logger().warn(f"[WATCHDOG] Forcing ZERO velocity: {reason_str}")
            self.publish_cmd(0.0, 0.0)
            self._send_log("WARN", f"watchdog_stop reasons={reason_str}")

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
