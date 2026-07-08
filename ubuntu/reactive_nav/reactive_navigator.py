#!/usr/bin/env python3
"""ROS 2 entrypoint for safety-first reactive TurtleBot4 navigation.

The node defaults to dry-run/no-movement. To publish non-zero velocity, launch
with both ``dry_run:=false`` and ``enable_motion:=true`` after validating the
robot-side topics and physical test area.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Dict, Optional

try:
    from .behavior_arbiter import ArbiterInput, BehaviorArbiter, SignalState, SignDebouncer
    from .diagnostics import DiagnosticSnapshot, PersistentJsonlLogger, UdpDiagnostics
    from .lidar_sectors import SectorMap, extract_sectors
    from .qr_logger import QRLogger
    from .turn_controller import TurnController
    from .wall_following import NavigationObservation, TwistCommand, create_navigation_module
except ImportError:  # pragma: no cover - direct script fallback
    from behavior_arbiter import ArbiterInput, BehaviorArbiter, SignalState, SignDebouncer
    from diagnostics import DiagnosticSnapshot, PersistentJsonlLogger, UdpDiagnostics
    from lidar_sectors import SectorMap, extract_sectors
    from qr_logger import QRLogger
    from turn_controller import TurnController
    from wall_following import NavigationObservation, TwistCommand, create_navigation_module


@dataclass(frozen=True)
class SensorFreshness:
    lidar_fresh: bool
    lidar_age_s: float
    image_fresh: bool
    image_age_s: float
    signal_fresh: bool
    signal_age_s: float


def _normalize_signal_direction(payload: Dict[str, Any]) -> str:
    raw = payload.get("direction") or payload.get("class_name") or payload.get("label") or "none"
    normalized = str(raw).lower().replace("-", "_").replace(" ", "_")
    if "left" in normalized or "izquierda" in normalized:
        return "left"
    if "right" in normalized or "derecha" in normalized:
        return "right"
    if "stop" in normalized or "alto" in normalized:
        return "stop"
    return "none"


def read_signal_state(
    path: str | Path,
    *,
    max_age_s: float,
    min_confidence: float,
    min_area_ratio: float,
    center_min: float,
    center_max: float,
) -> SignalState:
    path = Path(path)
    if not path.exists():
        return SignalState(reason=f"missing:{path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        return SignalState(reason=f"read_error:{exc}")

    timestamp = float(payload.get("timestamp") or 0.0)
    age = time.time() - timestamp if timestamp > 0.0 else math.inf
    direction = _normalize_signal_direction(payload)
    confidence = float(payload.get("confidence") or 0.0)
    area_ratio = float(payload.get("bbox_area_ratio") or payload.get("area_ratio") or 0.0)
    center_x = float(payload.get("bbox_center_x_ratio") or payload.get("center_x_ratio") or 0.5)
    stale = age > max_age_s

    if "actionable" in payload:
        actionable = bool(payload.get("actionable"))
    else:
        actionable = (
            direction in ("left", "right", "stop")
            and confidence >= min_confidence
            and area_ratio >= min_area_ratio
            and center_min <= center_x <= center_max
        )

    event_id = (
        f"{direction}:"
        f"{payload.get('source_frame_time')}:{timestamp:.6f}:"
        f"{payload.get('bbox_xyxy') or payload.get('bbox')}"
    )
    reason = "fresh" if not stale else f"stale:{age:.2f}s"
    return SignalState(
        direction=direction,
        confidence=confidence,
        bbox_area_ratio=area_ratio,
        bbox_center_x_ratio=center_x,
        actionable=actionable,
        timestamp=timestamp,
        stale=stale,
        event_id=event_id,
        reason=reason,
    )


def load_profile_parameters(path: str | Path) -> Dict[str, Any]:
    """Load the flat ros__parameters block from a simple ROS YAML profile."""

    profile_path = Path(path)
    parameters: Dict[str, Any] = {}
    inside_ros_parameters = False
    for raw_line in profile_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line:
            continue
        if line.strip() == "ros__parameters:":
            inside_ros_parameters = True
            continue
        if not inside_ros_parameters:
            continue
        if not raw_line.startswith("    ") or ":" not in line:
            continue
        key, value = line.strip().split(":", 1)
        parsed = value.strip()
        if not parsed:
            continue
        lowered = parsed.lower()
        if lowered in ("true", "false"):
            parameters[key] = lowered == "true"
            continue
        try:
            numeric = float(parsed)
        except ValueError:
            parameters[key] = parsed.strip("'\"")
            continue
        parameters[key] = int(numeric) if numeric.is_integer() else numeric
    return parameters


def run_ros_node(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    from geometry_msgs.msg import Twist, TwistStamped
    from sensor_msgs.msg import Image, LaserScan

    class ReactiveNavigatorNode(Node):
        def __init__(self):
            super().__init__("reactive_yolo_lidar_navigator")

            self.declare_parameter("scan_topic", "")
            self.declare_parameter("auto_discover_scan", True)
            self.declare_parameter("cmd_topic", "/cmd_vel")
            self.declare_parameter("cmd_msg_type", "TwistStamped")
            self.declare_parameter("dry_run", True)
            self.declare_parameter("enable_motion", False)
            self.declare_parameter("publish_zero_in_dry_run", True)
            self.declare_parameter("control_hz", 15.0)
            self.declare_parameter("max_scan_age_s", 0.50)
            self.declare_parameter("profile_name", "wall_follow_safe")
            self.declare_parameter("nav_module", "wall_follow")

            self.declare_parameter("base_speed", 0.10)
            self.declare_parameter("narrow_speed", 0.06)
            self.declare_parameter("turn_slow_speed", 0.07)
            self.declare_parameter("turn_slow_yaw_threshold", 0.24)
            self.declare_parameter("max_yaw", 0.65)
            self.declare_parameter("wall_kp", 0.45)
            self.declare_parameter("wall_kd", 0.04)
            self.declare_parameter("front_clear_distance", 0.55)
            self.declare_parameter("recovery_clearance", 0.42)
            self.declare_parameter("corner_slow_speed", 0.035)
            self.declare_parameter("side_avoid_distance", 0.34)
            self.declare_parameter("front_corner_avoid_distance", 0.62)
            self.declare_parameter("avoidance_gain", 0.65)
            self.declare_parameter("enable_corner_yaw_veto", True)
            self.declare_parameter("enable_corner_slowdown", True)
            self.declare_parameter("enable_side_yaw_veto", True)
            self.declare_parameter("enable_anti_spin", False)
            self.declare_parameter("anti_spin_yaw_threshold", 0.42)
            self.declare_parameter("anti_spin_linear_threshold", 0.025)
            self.declare_parameter("anti_spin_trigger_cycles", 8)
            self.declare_parameter("anti_spin_recovery_speed", 0.035)
            self.declare_parameter("angular_smoothing_alpha", 1.0)
            self.declare_parameter("gap_bubble_radius_m", 0.30)
            self.declare_parameter("gap_min_width_deg", 18.0)
            self.declare_parameter("gap_search_min_deg", -120.0)
            self.declare_parameter("gap_search_max_deg", 120.0)
            self.declare_parameter("gap_heading_scale_deg", 75.0)
            self.declare_parameter("gap_distance_score_cap_m", 3.0)
            self.declare_parameter("gap_forward_cone_deg", 18.0)
            self.declare_parameter("robot_width_m", 0.36)
            self.declare_parameter("gap_side_margin_m", 0.08)
            self.declare_parameter("focm_alpha", 40.0)
            self.declare_parameter("focm_goal_heading_deg", 0.0)

            self.declare_parameter("front_stop_distance", 0.32)
            self.declare_parameter("front_stop_clear_distance", 0.40)
            self.declare_parameter("side_stop_distance", 0.14)
            self.declare_parameter("side_stop_clear_distance", 0.20)
            self.declare_parameter("emergency_clear_cycles", 3)
            self.declare_parameter("slow_distance", 0.55)
            self.declare_parameter("turn_clearance", 0.42)
            self.declare_parameter("turn_speed", 0.45)
            self.declare_parameter("turn_degrees", 90.0)
            self.declare_parameter("settle_seconds", 0.25)
            self.declare_parameter("align_max_seconds", 1.6)
            self.declare_parameter("align_yaw_limit", 0.28)
            self.declare_parameter("align_gain", 0.45)
            self.declare_parameter("align_error_threshold", 0.08)
            self.declare_parameter("align_stable_cycles", 3)
            self.declare_parameter("align_same_direction_only", True)

            self.declare_parameter("signal_state_path", "output/signals/latest_signal.json")
            self.declare_parameter("max_signal_age_s", 0.8)
            self.declare_parameter("sign_confirm_window", 8)
            self.declare_parameter("sign_confirm_count", 5)
            self.declare_parameter("sign_min_confidence", 0.70)
            self.declare_parameter("sign_min_area_ratio", 0.03)
            self.declare_parameter("sign_center_x_min", 0.20)
            self.declare_parameter("sign_center_x_max", 0.80)
            self.declare_parameter("sign_cooldown_s", 3.0)

            self.declare_parameter("enable_qr_detection", True)
            self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
            self.declare_parameter("max_image_age_s", 1.5)
            self.declare_parameter("qr_check_every_n_frames", 5)
            self.declare_parameter("qr_log_path", "output/qr_log.jsonl")
            self.declare_parameter("qr_confirm_count", 2)

            self.declare_parameter("telemetry_enabled", True)
            self.declare_parameter("telemetry_port", 6612)
            self.declare_parameter("robot_name", "turtlebot4_rensso_mora")
            self.declare_parameter("pairing_code", "ROBOT_A_2")
            self.declare_parameter("diagnostic_period_s", 0.5)
            self.declare_parameter("persistent_log_enabled", True)
            self.declare_parameter("persistent_log_path", "output/reactive_nav_debug.jsonl")
            self.declare_parameter("persistent_log_period_s", 0.10)
            self.declare_parameter("collision_logging_enabled", True)
            self.declare_parameter("hazard_topic", "/hazard_detection")
            self.declare_parameter("collision_log_path", "output/collision_events.jsonl")
            self.declare_parameter("collision_image_dir", "output/collision_frames")
            self.declare_parameter("collision_cooldown_s", 2.0)

            self.scan_topic = self._param_str("scan_topic")
            self.auto_discover_scan = self._param_bool("auto_discover_scan")
            self.cmd_topic = self._param_str("cmd_topic")
            self.cmd_msg_type = self._param_str("cmd_msg_type").lower()
            self.dry_run = self._param_bool("dry_run")
            self.enable_motion = self._param_bool("enable_motion")
            self.publish_zero_in_dry_run = self._param_bool("publish_zero_in_dry_run")
            self.max_scan_age_s = self._param_float("max_scan_age_s")
            self.profile_name = self._param_str("profile_name")
            self.nav_module_name = self._param_str("nav_module")
            self.signal_state_path = Path(self._param_str("signal_state_path"))
            self.max_signal_age_s = self._param_float("max_signal_age_s")
            self.max_image_age_s = self._param_float("max_image_age_s")
            self.diagnostic_period_s = self._param_float("diagnostic_period_s")

            self.latest_scan = None
            self.latest_sectors: Optional[SectorMap] = None
            self.last_scan_time: Optional[float] = None
            self.scan_count = 0
            self.scan_sub = None
            self.current_scan_topic = ""

            self.last_image_time: Optional[float] = None
            self.latest_image_msg = None
            self.image_count = 0
            self.qr_frame_counter = 0
            self.last_qr_time = 0.0
            self.qr_detector = None
            self.bridge = None
            self.image_sub = None
            self.hazard_sub = None
            self.last_collision_log_time = 0.0

            self.last_signal = SignalState()
            self.last_requested_command = TwistCommand()
            self.last_published_command = TwistCommand()
            self.last_motion_enabled = False
            self.last_loop_time = time.monotonic()
            self.last_diag_time = 0.0
            self.last_persistent_log_time = 0.0
            self.last_state = ""
            self.last_reason = ""
            self._last_scan_discovery_log = 0.0

            nav_kwargs = {
                "base_speed": self._param_float("base_speed"),
                "narrow_speed": self._param_float("narrow_speed"),
                "turn_slow_speed": self._param_float("turn_slow_speed"),
                "turn_slow_yaw_threshold": self._param_float("turn_slow_yaw_threshold"),
                "max_yaw": self._param_float("max_yaw"),
                "kp": self._param_float("wall_kp"),
                "kd": self._param_float("wall_kd"),
                "front_clear_distance": self._param_float("front_clear_distance"),
                "slow_distance": self._param_float("slow_distance"),
                "recovery_clearance": self._param_float("recovery_clearance"),
                "side_avoid_distance": self._param_float("side_avoid_distance"),
                "front_corner_avoid_distance": self._param_float("front_corner_avoid_distance"),
                "avoidance_gain": self._param_float("avoidance_gain"),
                "gap_bubble_radius_m": self._param_float("gap_bubble_radius_m"),
                "gap_min_width_deg": self._param_float("gap_min_width_deg"),
                "gap_search_min_deg": self._param_float("gap_search_min_deg"),
                "gap_search_max_deg": self._param_float("gap_search_max_deg"),
                "gap_heading_scale_deg": self._param_float("gap_heading_scale_deg"),
                "gap_distance_score_cap_m": self._param_float("gap_distance_score_cap_m"),
                "gap_forward_cone_deg": self._param_float("gap_forward_cone_deg"),
                "robot_width_m": self._param_float("robot_width_m"),
                "gap_side_margin_m": self._param_float("gap_side_margin_m"),
                "focm_alpha": self._param_float("focm_alpha"),
                "focm_goal_heading_deg": self._param_float("focm_goal_heading_deg"),
            }
            self.nav_module = create_navigation_module(self.nav_module_name, **nav_kwargs)

            signs = SignDebouncer(
                confirm_window=self._param_int("sign_confirm_window"),
                confirm_count=self._param_int("sign_confirm_count"),
                min_confidence=self._param_float("sign_min_confidence"),
                min_area_ratio=self._param_float("sign_min_area_ratio"),
                cooldown_s=self._param_float("sign_cooldown_s"),
            )
            turns = TurnController(
                turn_speed=self._param_float("turn_speed"),
                turn_degrees=self._param_float("turn_degrees"),
                settle_seconds=self._param_float("settle_seconds"),
                align_max_seconds=self._param_float("align_max_seconds"),
                align_yaw_limit=self._param_float("align_yaw_limit"),
                align_gain=self._param_float("align_gain"),
                align_error_threshold=self._param_float("align_error_threshold"),
                align_stable_cycles=self._param_int("align_stable_cycles"),
                align_same_direction_only=self._param_bool("align_same_direction_only"),
            )
            self.turn_controller = turns
            self.arbiter = BehaviorArbiter(
                front_stop_distance=self._param_float("front_stop_distance"),
                front_stop_clear_distance=self._param_float("front_stop_clear_distance"),
                side_stop_distance=self._param_float("side_stop_distance"),
                side_stop_clear_distance=self._param_float("side_stop_clear_distance"),
                emergency_clear_cycles=self._param_int("emergency_clear_cycles"),
                slow_distance=self._param_float("slow_distance"),
                front_corner_avoid_distance=self._param_float("front_corner_avoid_distance"),
                corner_slow_speed=self._param_float("corner_slow_speed"),
                enable_corner_yaw_veto=self._param_bool("enable_corner_yaw_veto"),
                enable_corner_slowdown=self._param_bool("enable_corner_slowdown"),
                enable_side_yaw_veto=self._param_bool("enable_side_yaw_veto"),
                enable_anti_spin=self._param_bool("enable_anti_spin"),
                anti_spin_yaw_threshold=self._param_float("anti_spin_yaw_threshold"),
                anti_spin_linear_threshold=self._param_float("anti_spin_linear_threshold"),
                anti_spin_trigger_cycles=self._param_int("anti_spin_trigger_cycles"),
                anti_spin_recovery_speed=self._param_float("anti_spin_recovery_speed"),
                angular_smoothing_alpha=self._param_float("angular_smoothing_alpha"),
                turn_clearance=self._param_float("turn_clearance"),
                sign_debouncer=signs,
                turn_controller=turns,
            )
            self.qr_logger = QRLogger(
                self._param_str("qr_log_path"),
                confirm_count=self._param_int("qr_confirm_count"),
            )

            self.diag = UdpDiagnostics(
                self.get_logger(),
                enabled=self._param_bool("telemetry_enabled"),
                port=self._param_int("telemetry_port"),
                robot_name=self._param_str("robot_name"),
                pairing_code=self._param_str("pairing_code"),
            )
            self.run_logger = PersistentJsonlLogger(
                self._param_str("persistent_log_path"),
                enabled=self._param_bool("persistent_log_enabled"),
            )
            self.persistent_log_period_s = max(0.0, self._param_float("persistent_log_period_s"))
            self.collision_logger = PersistentJsonlLogger(
                self._param_str("collision_log_path"),
                enabled=self._param_bool("collision_logging_enabled"),
            )
            self.collision_image_dir = Path(self._param_str("collision_image_dir"))
            self.collision_cooldown_s = max(0.0, self._param_float("collision_cooldown_s"))

            if self.cmd_msg_type in ("twist", "geometry_msgs/msg/twist"):
                self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
                self._cmd_message_kind = "Twist"
            else:
                self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
                self._cmd_message_kind = "TwistStamped"

            self._init_qr_subscription(Image, qos_profile_sensor_data)
            self._init_collision_subscription(qos_profile_sensor_data)
            self._ensure_scan_subscription(LaserScan, qos_profile_sensor_data, force=True)

            period = 1.0 / max(1.0, self._param_float("control_hz"))
            self.timer = self.create_timer(period, self.control_loop)
            self.diag.log(
                "INFO",
                "[INIT] reactive navigator started "
                f"dry_run={self.dry_run} enable_motion={self.enable_motion} "
                f"cmd={self._cmd_message_kind}:{self.cmd_topic} profile={self.profile_name} nav={self.nav_module.name} "
                f"log={self._param_str('persistent_log_path') if self._param_bool('persistent_log_enabled') else 'disabled'}",
            )

        def _param_str(self, name: str) -> str:
            return str(self.get_parameter(name).value)

        def _param_bool(self, name: str) -> bool:
            return bool(self.get_parameter(name).value)

        def _param_int(self, name: str) -> int:
            return int(self.get_parameter(name).value)

        def _param_float(self, name: str) -> float:
            return float(self.get_parameter(name).value)

        def _init_qr_subscription(self, ImageMsg, qos_profile) -> None:
            if not self._param_bool("enable_qr_detection"):
                self.diag.log("INFO", "[QR] local QR detection disabled by parameter")
                return
            try:
                import cv2
                from cv_bridge import CvBridge
            except ImportError as exc:
                self.diag.log("WARN", f"[QR] cv2/cv_bridge unavailable; QR detection disabled: {exc}")
                return

            image_topic = self._param_str("image_topic")
            self.bridge = CvBridge()
            self.qr_detector = cv2.QRCodeDetector()
            self.image_sub = self.create_subscription(
                ImageMsg,
                image_topic,
                self.image_callback,
                qos_profile,
            )
            pub_count = len(self.get_publishers_info_by_topic(image_topic))
            self.diag.log(
                "INFO",
                f"[QR] subscribed image_topic={image_topic} publishers={pub_count}",
            )

        def _init_collision_subscription(self, qos_profile) -> None:
            if not self._param_bool("collision_logging_enabled"):
                self.diag.log("INFO", "[COLLISION] event logging disabled by parameter")
                return
            try:
                from irobot_create_msgs.msg import HazardDetectionVector
            except ImportError as exc:
                self.diag.log("WARN", f"[COLLISION] irobot_create_msgs unavailable; hazard logging disabled: {exc}")
                return

            hazard_topic = self._param_str("hazard_topic")
            self.hazard_sub = self.create_subscription(
                HazardDetectionVector,
                hazard_topic,
                self.hazard_callback,
                qos_profile,
            )
            pub_count = len(self.get_publishers_info_by_topic(hazard_topic))
            self.diag.log(
                "INFO",
                f"[COLLISION] subscribed hazard_topic={hazard_topic} publishers={pub_count}",
            )

        def _laser_scan_topics(self) -> list[str]:
            topics = []
            for name, types in self.get_topic_names_and_types():
                if "sensor_msgs/msg/LaserScan" in types:
                    topics.append(name)
            return sorted(topics)

        def _topic_publisher_count(self, topic: str) -> int:
            try:
                return len(self.get_publishers_info_by_topic(topic))
            except Exception:
                return 0

        def _ensure_scan_subscription(self, LaserScanMsg, qos_profile, *, force: bool = False) -> None:
            now = time.monotonic()
            scan_age = math.inf if self.last_scan_time is None else now - self.last_scan_time
            if not force and self.current_scan_topic and scan_age <= self.max_scan_age_s:
                return

            configured = self.scan_topic.strip()
            candidates = []
            if configured:
                candidates.append(configured)
            if self.auto_discover_scan:
                for topic in self._laser_scan_topics():
                    if topic not in candidates:
                        candidates.append(topic)

            selected = None
            debug_counts = {}
            for topic in candidates:
                count = self._topic_publisher_count(topic)
                debug_counts[topic] = count
                if count > 0:
                    selected = topic
                    break

            if selected is None:
                if now - self._last_scan_discovery_log > 2.0:
                    self.diag.log("WARN", f"[LIDAR] no LaserScan topic with publishers; candidates={debug_counts}")
                    self._last_scan_discovery_log = now
                return

            if selected == self.current_scan_topic and self.scan_sub is not None:
                return

            if self.scan_sub is not None:
                try:
                    self.destroy_subscription(self.scan_sub)
                except Exception:
                    pass
            self.scan_sub = self.create_subscription(
                LaserScanMsg,
                selected,
                self.scan_callback,
                qos_profile,
            )
            self.current_scan_topic = selected
            self.diag.log("INFO", f"[LIDAR] subscribed scan_topic={selected} publishers={debug_counts.get(selected)}")

        def scan_callback(self, msg) -> None:
            self.latest_scan = msg
            self.last_scan_time = time.monotonic()
            self.scan_count += 1
            self.latest_sectors = extract_sectors(msg)

        def image_callback(self, msg) -> None:
            self.last_image_time = time.monotonic()
            self.latest_image_msg = msg
            self.image_count += 1
            self.qr_frame_counter += 1
            if self.qr_detector is None or self.bridge is None:
                return
            every_n = max(1, self._param_int("qr_check_every_n_frames"))
            if self.qr_frame_counter % every_n != 0:
                return
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                content, _, _ = self.qr_detector.detectAndDecode(cv_img)
            except Exception as exc:
                self.diag.log("WARN", f"[QR] decode error: {exc}")
                return
            if not content:
                return

            context = self._distance_context()
            event = self.qr_logger.observe(
                content,
                source="camera",
                frame_id=getattr(msg.header, "frame_id", None),
                robot_state=self.last_state,
                context=context,
            )
            if event is None:
                return
            self.last_qr_time = time.monotonic()
            if event.logged:
                self.diag.log("INFO", f"[QR] logged content={event.content!r} path={event.path}")
            elif event.duplicate:
                self.diag.log("INFO", f"[QR] duplicate ignored content={event.content!r}")

        def hazard_callback(self, msg) -> None:
            detections = list(getattr(msg, "detections", []) or [])
            if not detections:
                return

            now = time.monotonic()
            if now - self.last_collision_log_time < self.collision_cooldown_s:
                return
            self.last_collision_log_time = now

            image_path = self._save_collision_frame()
            hazard_payload = self._ros_message_to_dict(msg)
            sectors = self.latest_sectors
            record = {
                "event": "hazard_detection",
                "hazard_topic": self._param_str("hazard_topic"),
                "hazard": self._json_clean(hazard_payload),
                "detection_count": len(detections),
                "state": self.last_state,
                "reason": self.last_reason,
                "scan_topic": self.current_scan_topic or None,
                "scan_count": self.scan_count,
                "image_count": self.image_count,
                "freshness": {
                    "lidar_age_s": self._json_float(math.inf if self.last_scan_time is None else now - self.last_scan_time),
                    "image_age_s": self._json_float(math.inf if self.last_image_time is None else now - self.last_image_time),
                    "signal_stale": self.last_signal.stale,
                },
                "command": {
                    "last_requested_linear_x": self._json_float(self.last_requested_command.linear_x),
                    "last_requested_angular_z": self._json_float(self.last_requested_command.angular_z),
                    "last_published_linear_x": self._json_float(self.last_published_command.linear_x),
                    "last_published_angular_z": self._json_float(self.last_published_command.angular_z),
                    "motion_enabled_last_cycle": self.last_motion_enabled,
                    "positive_angular_z_means": "left_turn",
                },
                "lidar": self._collision_lidar_snapshot(sectors),
                "signal": {
                    "direction": self.last_signal.direction,
                    "confidence": self._json_float(self.last_signal.confidence),
                    "bbox_area_ratio": self._json_float(self.last_signal.bbox_area_ratio),
                    "bbox_center_x_ratio": self._json_float(self.last_signal.bbox_center_x_ratio),
                    "stale": self.last_signal.stale,
                    "reason": self.last_signal.reason,
                },
                "camera_frame_path": image_path,
            }
            self.collision_logger.write(record)
            self.diag.log(
                "WARN",
                f"[COLLISION] hazard event logged detections={len(detections)} "
                f"log={self._param_str('collision_log_path')} image={image_path or 'none'}",
            )

        def _read_signal(self) -> SignalState:
            return read_signal_state(
                self.signal_state_path,
                max_age_s=self.max_signal_age_s,
                min_confidence=self._param_float("sign_min_confidence"),
                min_area_ratio=self._param_float("sign_min_area_ratio"),
                center_min=self._param_float("sign_center_x_min"),
                center_max=self._param_float("sign_center_x_max"),
            )

        def control_loop(self) -> None:
            from sensor_msgs.msg import LaserScan
            from rclpy.qos import qos_profile_sensor_data

            now = time.monotonic()
            dt = max(0.02, now - self.last_loop_time)
            self.last_loop_time = now
            self._ensure_scan_subscription(LaserScan, qos_profile_sensor_data)

            signal = self._read_signal()
            self.last_signal = signal
            sectors = self.latest_sectors
            lidar_age = math.inf if self.last_scan_time is None else now - self.last_scan_time
            lidar_fresh = lidar_age <= self.max_scan_age_s
            nav_suggestion = None
            if sectors is not None and lidar_fresh:
                nav_suggestion = self.nav_module.compute(NavigationObservation(sectors, now, dt))

            qr_recent = (now - self.last_qr_time) <= 1.0
            output = self.arbiter.decide(
                ArbiterInput(
                    sectors=sectors,
                    lidar_fresh=lidar_fresh,
                    nav_suggestion=nav_suggestion,
                    signal=signal,
                    qr_recent=qr_recent,
                    now=now,
                )
            )
            published_command, motion_enabled = self._publish_command(output.command, output.publish_motion)
            self.last_requested_command = output.command
            self.last_published_command = published_command
            self.last_motion_enabled = motion_enabled
            self._write_persistent_log(
                output=output,
                sectors=sectors,
                lidar_age=lidar_age,
                image_age=math.inf if self.last_image_time is None else now - self.last_image_time,
                signal=signal,
                nav_suggestion=nav_suggestion,
                requested_command=output.command,
                published_command=published_command,
                motion_enabled=motion_enabled,
                dt=dt,
            )
            self._emit_diagnostics(output, sectors, lidar_age)
            self.last_state = output.state
            self.last_reason = output.reason

        def _publish_command(self, command: TwistCommand, publish_motion: bool) -> tuple[TwistCommand, bool]:
            allow_motion = publish_motion and self.enable_motion and not self.dry_run
            command_to_publish = command if allow_motion else TwistCommand()
            if not allow_motion and not self.publish_zero_in_dry_run:
                return command_to_publish, allow_motion

            if self._cmd_message_kind == "TwistStamped":
                from geometry_msgs.msg import TwistStamped

                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = float(command_to_publish.linear_x)
                msg.twist.angular.z = float(command_to_publish.angular_z)
            else:
                from geometry_msgs.msg import Twist

                msg = Twist()
                msg.linear.x = float(command_to_publish.linear_x)
                msg.angular.z = float(command_to_publish.angular_z)
            self.cmd_pub.publish(msg)
            return command_to_publish, allow_motion

        def _distance_context(self) -> Dict[str, float]:
            sectors = self.latest_sectors
            if sectors is None:
                return {}
            return {
                "front_distance_m": self._dist(sectors, "front"),
                "left_distance_m": self._dist(sectors, "left"),
                "right_distance_m": self._dist(sectors, "right"),
                "rear_distance_m": sectors.rear_distance if sectors.rear_distance is not None else -1.0,
            }

        def _write_persistent_log(
            self,
            *,
            output,
            sectors: Optional[SectorMap],
            lidar_age: float,
            image_age: float,
            signal: SignalState,
            nav_suggestion,
            requested_command: TwistCommand,
            published_command: TwistCommand,
            motion_enabled: bool,
            dt: float,
        ) -> None:
            now = time.monotonic()
            changed = output.state != self.last_state or output.reason != self.last_reason
            if (
                not changed
                and self.persistent_log_period_s > 0.0
                and now - self.last_persistent_log_time < self.persistent_log_period_s
            ):
                return
            self.last_persistent_log_time = now

            sector_distances = {}
            sector_counts = {}
            sector_raw_min = {}
            if sectors is not None:
                for name, stats in sectors.sectors.items():
                    sector_distances[name] = self._json_float(stats.distance)
                    sector_counts[name] = stats.valid_count
                    sector_raw_min[name] = self._json_float(stats.min_range)

            left_distance = sectors.distance("left") if sectors is not None else None
            right_distance = sectors.distance("right") if sectors is not None else None
            left_minus_right = (
                left_distance - right_distance
                if left_distance is not None and right_distance is not None
                else None
            )

            nav_debug = dict(nav_suggestion.debug) if nav_suggestion is not None else {}
            output_debug = dict(output.debug) if output.debug else {}
            turn_debug = self._json_clean(self.turn_controller.snapshot())
            emergency_debug = {
                key: output_debug.get(key)
                for key in (
                    "emergency_active",
                    "emergency_trigger_reason",
                    "emergency_clear_counter",
                    "emergency_trigger_count",
                )
            }
            record = {
                "profile_name": self.profile_name,
                "state": output.state,
                "reason": output.reason,
                "previous_state": self.last_state,
                "scan_topic": self.current_scan_topic or None,
                "scan_count": self.scan_count,
                "image_count": self.image_count,
                "dt_s": self._json_float(dt),
                "freshness": {
                    "lidar_age_s": self._json_float(lidar_age),
                    "lidar_fresh": lidar_age <= self.max_scan_age_s,
                    "image_age_s": self._json_float(image_age),
                    "image_fresh": image_age <= self.max_image_age_s,
                    "signal_stale": signal.stale,
                },
                "mode_flags": {
                    "dry_run": self.dry_run,
                    "enable_motion": self.enable_motion,
                    "motion_enabled_this_cycle": motion_enabled,
                    "publish_zero_in_dry_run": self.publish_zero_in_dry_run,
                },
                "command": {
                    "requested_linear_x": self._json_float(requested_command.linear_x),
                    "requested_angular_z": self._json_float(requested_command.angular_z),
                    "published_linear_x": self._json_float(published_command.linear_x),
                    "published_angular_z": self._json_float(published_command.angular_z),
                    "positive_angular_z_means": "left_turn",
                },
                "nav": {
                    "module": self.nav_module.name,
                    "suggestion_mode": nav_suggestion.mode if nav_suggestion is not None else None,
                    "suggestion_reason": nav_suggestion.reason if nav_suggestion is not None else None,
                    "suggested_linear_x": self._json_float(nav_suggestion.command.linear_x) if nav_suggestion is not None else None,
                    "suggested_angular_z": self._json_float(nav_suggestion.command.angular_z) if nav_suggestion is not None else None,
                    "debug": self._json_clean(nav_debug),
                },
                "arbiter_debug": self._json_clean(output_debug),
                "turn": turn_debug,
                "emergency": self._json_clean(emergency_debug),
                "lidar": {
                    "valid_count": sectors.valid_count if sectors is not None else 0,
                    "total_count": sectors.total_count if sectors is not None else 0,
                    "sector_distance_m": sector_distances,
                    "sector_raw_min_m": sector_raw_min,
                    "sector_valid_count": sector_counts,
                    "left_minus_right_m": self._json_float(left_minus_right),
                },
                "signal": {
                    "direction": signal.direction,
                    "confidence": self._json_float(signal.confidence),
                    "bbox_area_ratio": self._json_float(signal.bbox_area_ratio),
                    "bbox_center_x_ratio": self._json_float(signal.bbox_center_x_ratio),
                    "actionable": signal.actionable,
                    "stale": signal.stale,
                    "reason": signal.reason,
                    "event_id": signal.event_id,
                },
            }
            self.run_logger.write(record)

        def _collision_lidar_snapshot(self, sectors: Optional[SectorMap]) -> Dict[str, Any]:
            if sectors is None:
                return {
                    "valid_count": 0,
                    "total_count": 0,
                    "sector_distance_m": {},
                    "left_minus_right_m": None,
                }
            sector_distances = {
                name: self._json_float(stats.distance)
                for name, stats in sectors.sectors.items()
            }
            sector_raw_min = {
                name: self._json_float(stats.min_range)
                for name, stats in sectors.sectors.items()
            }
            sector_counts = {
                name: stats.valid_count
                for name, stats in sectors.sectors.items()
            }
            left = sectors.distance("left")
            right = sectors.distance("right")
            left_minus_right = left - right if left is not None and right is not None else None
            nearest = min(sectors.points, key=lambda point: point.distance_m) if sectors.points else None
            return {
                "valid_count": sectors.valid_count,
                "total_count": sectors.total_count,
                "sector_distance_m": sector_distances,
                "sector_raw_min_m": sector_raw_min,
                "sector_valid_count": sector_counts,
                "left_minus_right_m": self._json_float(left_minus_right),
                "nearest_dist_m": self._json_float(nearest.distance_m if nearest else None),
                "nearest_angle_deg": self._json_float(nearest.angle_deg if nearest else None),
            }

        def _save_collision_frame(self) -> Optional[str]:
            if self.latest_image_msg is None or self.bridge is None:
                return None
            try:
                import cv2
            except ImportError:
                return None
            try:
                self.collision_image_dir.mkdir(parents=True, exist_ok=True)
                cv_img = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding="bgr8")
                filename = f"collision_{int(time.time() * 1000)}.jpg"
                path = self.collision_image_dir / filename
                if cv2.imwrite(str(path), cv_img):
                    return str(path)
            except Exception as exc:
                self.diag.log("WARN", f"[COLLISION] failed to save camera frame: {exc}")
            return None

        def _ros_message_to_dict(self, msg):
            try:
                from rosidl_runtime_py.convert import message_to_ordereddict

                return message_to_ordereddict(msg)
            except Exception:
                return str(msg)

        def _emit_diagnostics(self, output, sectors: Optional[SectorMap], lidar_age: float) -> None:
            now = time.monotonic()
            changed = output.state != self.last_state or output.reason != self.last_reason
            periodic = now - self.last_diag_time >= self.diagnostic_period_s
            if not periodic and not changed:
                return
            self.last_diag_time = now

            image_age = math.inf if self.last_image_time is None else now - self.last_image_time
            signal_age = math.inf if self.last_signal.timestamp <= 0.0 else time.time() - self.last_signal.timestamp
            freshness = SensorFreshness(
                lidar_fresh=lidar_age <= self.max_scan_age_s,
                lidar_age_s=lidar_age,
                image_fresh=image_age <= self.max_image_age_s,
                image_age_s=image_age,
                signal_fresh=not self.last_signal.stale,
                signal_age_s=signal_age,
            )

            if periodic:
                self.diag.log(
                    "INFO",
                    "[STATE] "
                    f"profile={self.profile_name} nav={self.nav_module.name} "
                    f"state={output.state} reason={output.reason} "
                    f"cmd=({output.command.linear_x:.3f},{output.command.angular_z:.3f}) "
                    f"dry_run={self.dry_run} enable_motion={self.enable_motion} "
                    f"scan_topic={self.current_scan_topic or 'none'} scan_count={self.scan_count} "
                    f"lidar_age={self._fmt_age(freshness.lidar_age_s)} "
                    f"image_age={self._fmt_age(freshness.image_age_s)} "
                    f"signal={self.last_signal.direction}/{self.last_signal.reason} "
                    f"emergency={output.debug.get('emergency_trigger_reason', 'NONE')}"
                    f"/{int(float(output.debug.get('emergency_clear_counter', 0.0) or 0.0))}",
                )

            if sectors is not None:
                nearest = min(sectors.points, key=lambda p: p.distance_m) if sectors.points else None
                debug = output.debug or {}
                snapshot = DiagnosticSnapshot(
                    state=output.state,
                    front=self._dist(sectors, "front"),
                    left=self._dist(sectors, "left"),
                    right=self._dist(sectors, "right"),
                    rear=sectors.rear_distance if sectors.rear_distance is not None else -1.0,
                    nearest_dist=nearest.distance_m if nearest else -1.0,
                    nearest_angle=nearest.angle_deg if nearest else 0.0,
                    gap_start=float(debug.get("gap_start", 0.0)),
                    gap_end=float(debug.get("gap_end", 0.0)),
                    turn_hint=float(debug.get("gap_center", output.command.angular_z * 90.0)),
                    speed=output.command.linear_x,
                    yaw=output.command.angular_z,
                )
                self.diag.lidar(snapshot)

        def _dist(self, sectors: SectorMap, name: str) -> float:
            value = sectors.distance(name)
            return float(value) if value is not None else -1.0

        def _fmt_age(self, age: float) -> str:
            return "inf" if math.isinf(age) else f"{age:.2f}s"

        def _json_float(self, value):
            if value is None:
                return None
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                return None
            if math.isnan(numeric) or math.isinf(numeric):
                return None
            return numeric

        def _json_clean(self, value):
            if isinstance(value, dict):
                return {str(key): self._json_clean(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [self._json_clean(item) for item in value]
            if isinstance(value, (str, bool)) or value is None:
                return value
            if isinstance(value, (int, float)):
                cleaned = self._json_float(value)
                return cleaned
            return str(value)

        def destroy_node(self):
            try:
                self._publish_command(TwistCommand(), True)
            except Exception:
                pass
            self.diag.close()
            super().destroy_node()

    rclpy.init(args=args)
    node = ReactiveNavigatorNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("[MAIN] interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_self_test() -> None:
    try:
        from .behavior_arbiter import ArbiterInput, BehaviorArbiter, SignalState
        from .lidar_sectors import extract_sectors
        from .qr_logger import QRLogger
        from .turn_controller import TurnController
        from .wall_following import NavigationObservation, create_navigation_module
    except ImportError:  # pragma: no cover
        from behavior_arbiter import ArbiterInput, BehaviorArbiter, SignalState
        from lidar_sectors import extract_sectors
        from qr_logger import QRLogger
        from turn_controller import TurnController
        from wall_following import NavigationObservation, create_navigation_module

    class FakeScan:
        angle_min = -math.pi
        angle_increment = math.radians(1.0)
        range_min = 0.12
        range_max = 12.0

        def __init__(self):
            self.ranges = [2.0] * 361
            for deg in range(-8, 9):
                self.ranges[deg + 180] = 0.30

    scan = FakeScan()
    sectors = extract_sectors(scan)
    assert sectors.distance("front") is not None
    assert sectors.distance("front_center") < 0.40

    nav = create_navigation_module("wall_follow")
    suggestion = nav.compute(NavigationObservation(sectors, time.monotonic(), 0.1))
    assert suggestion.mode in ("RECOVERY", "CORRIDOR_FOLLOW", "LEFT_WALL_FOLLOW", "RIGHT_WALL_FOLLOW")

    for module_name in ("follow_gap", "largest_gap", "focm"):
        gap_nav = create_navigation_module(
            module_name,
            base_speed=0.05,
            narrow_speed=0.03,
            max_yaw=0.65,
            recovery_clearance=0.42,
            gap_bubble_radius_m=0.25,
            robot_width_m=0.36,
            gap_side_margin_m=0.08,
        )
        gap_suggestion = gap_nav.compute(NavigationObservation(sectors, time.monotonic(), 0.1))
        assert gap_suggestion.mode in ("FOLLOW_GAP", "FOCM", "RECOVERY")
        assert abs(gap_suggestion.command.angular_z) <= 0.65
        assert "gap_width" in gap_suggestion.debug or gap_suggestion.reason.endswith("OPEN_SIDE")

    class FakeSectors:
        valid_count = 100
        points = ()

        def __init__(self, distances):
            self._distances = distances

        def distance(self, name, default=None):
            return self._distances.get(name, default)

    sign_nav = create_navigation_module("wall_follow", base_speed=0.05, narrow_speed=0.03)
    left_close = FakeSectors(
        {
            "front": 2.0,
            "front_left": 2.0,
            "front_right": 2.0,
            "left": 0.25,
            "right": 1.00,
        }
    )
    sign_suggestion = sign_nav.compute(NavigationObservation(left_close, time.monotonic(), 0.1))
    assert sign_suggestion.command.angular_z < 0.0

    right_close = FakeSectors(
        {
            "front": 2.0,
            "front_left": 2.0,
            "front_right": 2.0,
            "left": 1.00,
            "right": 0.25,
        }
    )
    sign_suggestion = sign_nav.compute(NavigationObservation(right_close, time.monotonic(), 0.1))
    assert sign_suggestion.command.angular_z > 0.0

    arbiter = BehaviorArbiter()
    decision = arbiter.decide(
        ArbiterInput(
            sectors=sectors,
            lidar_fresh=True,
            nav_suggestion=suggestion,
            signal=SignalState(),
            qr_recent=False,
            now=time.monotonic(),
        )
    )
    assert decision.state == "EMERGENCY_STOP"

    class FakeTurnSectors:
        valid_count = 100
        points = ()

        def __init__(self, distances):
            self._distances = distances

        def distance(self, name, default=None):
            return self._distances.get(name, default)

    turn_controller = TurnController(
        turn_speed=0.45,
        turn_degrees=90.0,
        settle_seconds=0.05,
        align_max_seconds=0.5,
        align_error_threshold=0.05,
        align_stable_cycles=2,
        align_same_direction_only=True,
    )
    started = time.monotonic()
    assert turn_controller.start("LEFT", started)
    settle_step = turn_controller.step(FakeTurnSectors({}), started + turn_controller.turn_seconds + 0.01)
    assert settle_step.state == "SETTLING_AFTER_TURN"
    align_step = turn_controller.step(
        FakeTurnSectors({"left": 1.0, "right": 0.4, "front": 1.0}),
        started + turn_controller.turn_seconds + turn_controller.settle_seconds + 0.02,
    )
    assert align_step.state == "ALIGNING_AFTER_TURN"
    assert align_step.command.linear_x == 0.0
    assert align_step.command.angular_z >= 0.0
    assert bool(align_step.debug["align_yaw_clamped"]) is True

    hysteresis_arbiter = BehaviorArbiter(
        front_stop_distance=0.30,
        front_stop_clear_distance=0.40,
        side_stop_distance=0.10,
        side_stop_clear_distance=0.18,
        emergency_clear_cycles=3,
    )
    trigger = hysteresis_arbiter.decide(
        ArbiterInput(
            sectors=FakeTurnSectors({"front": 0.25, "front_center": 0.25, "left": 0.50, "right": 0.50}),
            lidar_fresh=True,
            nav_suggestion=suggestion,
            signal=SignalState(),
            qr_recent=False,
            now=time.monotonic(),
        )
    )
    assert trigger.state == "EMERGENCY_STOP"
    safe_sectors = FakeTurnSectors({"front": 0.60, "front_center": 0.60, "left": 0.50, "right": 0.50})
    for _ in range(2):
        hold = hysteresis_arbiter.decide(
            ArbiterInput(
                sectors=safe_sectors,
                lidar_fresh=True,
                nav_suggestion=suggestion,
                signal=SignalState(),
                qr_recent=False,
                now=time.monotonic(),
            )
        )
        assert hold.state == "EMERGENCY_STOP"
    cleared = hysteresis_arbiter.decide(
        ArbiterInput(
            sectors=safe_sectors,
            lidar_fresh=True,
            nav_suggestion=suggestion,
            signal=SignalState(),
            qr_recent=False,
            now=time.monotonic(),
        )
    )
    assert cleared.state != "EMERGENCY_STOP"

    with tempfile.TemporaryDirectory() as tmp:
        logger = QRLogger(Path(tmp) / "qr_log.jsonl", confirm_count=2)
        assert logger.observe("checkpoint-1") is None
        event = logger.observe("checkpoint-1", robot_state="QR_SCAN")
        assert event is not None and event.logged

    profiles_dir = Path(__file__).resolve().parent / "configs"
    wall_profile = load_profile_parameters(profiles_dir / "wall_follow_safe.yaml")
    ftg_profile = load_profile_parameters(profiles_dir / "follow_gap_safe.yaml")
    assert wall_profile["profile_name"] == "wall_follow_safe"
    assert wall_profile["nav_module"] == "wall_follow"
    assert ftg_profile["profile_name"] == "follow_gap_safe"
    assert ftg_profile["nav_module"] == "follow_gap"

    payload_path = Path(tempfile.gettempdir()) / "reactive_nav_signal_test.json"
    payload_path.write_text(
        json.dumps(
            {
                "direction": "LEFT",
                "confidence": 0.95,
                "bbox_area_ratio": 0.10,
                "bbox_center_x_ratio": 0.5,
                "actionable": True,
                "timestamp": time.time(),
            }
        ),
        encoding="utf-8",
    )
    signal = read_signal_state(
        payload_path,
        max_age_s=1.0,
        min_confidence=0.70,
        min_area_ratio=0.03,
        center_min=0.2,
        center_max=0.8,
    )
    assert signal.direction == "left" and not signal.stale and signal.actionable
    payload_path.unlink(missing_ok=True)
    print("reactive_nav self-test passed")


def main() -> None:
    if "--self-test" in sys.argv:
        run_self_test()
        return
    run_ros_node()


if __name__ == "__main__":
    main()
