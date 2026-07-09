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
    from .lidar_sectors import SectorMap, extract_sectors, traversable_gaps
    from .qr_logger import QRLogger
    from .turn_controller import TurnController
    from .wall_following import NavigationObservation, TwistCommand, create_navigation_module
except ImportError:  # pragma: no cover - direct script fallback
    from behavior_arbiter import ArbiterInput, BehaviorArbiter, SignalState, SignDebouncer
    from diagnostics import DiagnosticSnapshot, PersistentJsonlLogger, UdpDiagnostics
    from lidar_sectors import SectorMap, extract_sectors, traversable_gaps
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
            self.declare_parameter("cmd_msg_type", "auto")
            self.declare_parameter("dry_run", True)
            self.declare_parameter("enable_motion", False)
            self.declare_parameter("publish_zero_in_dry_run", True)
            self.declare_parameter("control_hz", 15.0)
            self.declare_parameter("max_scan_age_s", 1.00)
            self.declare_parameter("nav_module", "forward_avoid")

            self.declare_parameter("base_speed", 0.14)
            self.declare_parameter("narrow_speed", 0.08)
            self.declare_parameter("max_yaw", 0.65)
            self.declare_parameter("cruise_yaw_limit", 0.35)
            self.declare_parameter("evasive_yaw_limit", 0.38)
            self.declare_parameter("wall_kp", 0.22)
            self.declare_parameter("wall_kd", 0.02)
            self.declare_parameter("balance_deadband_m", 0.18)
            self.declare_parameter("soft_avoid_distance", 0.58)
            self.declare_parameter("soft_avoid_gain", 0.75)
            self.declare_parameter("curve_heading_gain", 0.55)
            self.declare_parameter("reverse_turn_cooldown_s", 2.0)
            self.declare_parameter("robot_width_m", 0.36)
            self.declare_parameter("robot_length_m", 0.35)
            self.declare_parameter("footprint_margin_m", 0.08)
            self.declare_parameter("footprint_lookahead_m", 0.85)
            self.declare_parameter("footprint_slow_distance", 0.70)
            self.declare_parameter("path_forward_heading_tolerance_deg", 12.0)
            self.declare_parameter("path_required_clearance_m", 0.85)
            self.declare_parameter("drive_heading_limit_deg", 38.0)
            self.declare_parameter("search_turn_yaw", 0.32)
            self.declare_parameter("search_clear_confirm_cycles", 3)
            self.declare_parameter("local_path_heading_deg", 50.0)
            self.declare_parameter("local_path_step_deg", 10.0)
            self.declare_parameter("front_clear_distance", 0.55)
            self.declare_parameter("recovery_clearance", 0.42)

            self.declare_parameter("front_stop_distance", 0.32)
            self.declare_parameter("side_stop_distance", 0.14)
            self.declare_parameter("slow_distance", 0.55)
            self.declare_parameter("turn_clearance", 0.42)
            self.declare_parameter("turn_speed", 0.45)

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
            self.declare_parameter("telemetry_port", 6000)
            self.declare_parameter("robot_name", "turtlebot4_rensso_mora")
            self.declare_parameter("pairing_code", "ROBOT_A_2")
            self.declare_parameter("diagnostic_period_s", 0.5)
            self.declare_parameter("persistent_log_enabled", True)
            self.declare_parameter("persistent_log_path", "output/reactive_nav_debug.jsonl")
            self.declare_parameter("persistent_log_period_s", 0.10)

            self.scan_topic = self._param_str("scan_topic")
            self.auto_discover_scan = self._param_bool("auto_discover_scan")
            self.cmd_topic = self._param_str("cmd_topic")
            self.cmd_msg_type = self._param_str("cmd_msg_type").lower()
            self.dry_run = self._param_bool("dry_run")
            self.enable_motion = self._param_bool("enable_motion")
            self.publish_zero_in_dry_run = self._param_bool("publish_zero_in_dry_run")
            self.max_scan_age_s = self._param_float("max_scan_age_s")
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
            self.image_count = 0
            self.qr_frame_counter = 0
            self.last_qr_time = 0.0
            self.qr_detector = None
            self.bridge = None
            self.image_sub = None

            self.last_signal = SignalState()
            self.last_loop_time = time.monotonic()
            self.last_diag_time = 0.0
            self.last_persistent_log_time = 0.0
            self.last_state = ""
            self.last_reason = ""
            self._last_scan_discovery_log = 0.0
            self._last_cmd_status_log = 0.0
            self._cmd_subscriber_seen = False
            self._search_turn_sign = 0.0
            self._search_clear_cycles = 0
            self._last_motion_turn_sign = 0.0
            self._last_motion_turn_time = 0.0

            nav_kwargs = {
                "base_speed": self._param_float("base_speed"),
                "narrow_speed": self._param_float("narrow_speed"),
                "max_yaw": min(self._param_float("max_yaw"), self._param_float("cruise_yaw_limit")),
                "kp": self._param_float("wall_kp"),
                "kd": self._param_float("wall_kd"),
                "front_clear_distance": self._param_float("front_clear_distance"),
                "recovery_clearance": self._param_float("recovery_clearance"),
                "balance_deadband": self._param_float("balance_deadband_m"),
                "soft_avoid_distance": self._param_float("soft_avoid_distance"),
                "soft_avoid_gain": self._param_float("soft_avoid_gain"),
                "curve_heading_gain": self._param_float("curve_heading_gain"),
                "reverse_turn_cooldown_s": self._param_float("reverse_turn_cooldown_s"),
                "robot_width_m": self._param_float("robot_width_m"),
                "footprint_margin_m": self._param_float("footprint_margin_m"),
                "footprint_lookahead_m": self._param_float("footprint_lookahead_m"),
                "path_forward_heading_tolerance_deg": self._param_float("path_forward_heading_tolerance_deg"),
                "drive_heading_limit_deg": self._param_float("drive_heading_limit_deg"),
                "local_path_heading_deg": self._param_float("local_path_heading_deg"),
                "local_path_step_deg": self._param_float("local_path_step_deg"),
            }
            self.nav_module = create_navigation_module(self._param_str("nav_module"), **nav_kwargs)

            signs = SignDebouncer(
                confirm_window=self._param_int("sign_confirm_window"),
                confirm_count=self._param_int("sign_confirm_count"),
                min_confidence=self._param_float("sign_min_confidence"),
                min_area_ratio=self._param_float("sign_min_area_ratio"),
                cooldown_s=self._param_float("sign_cooldown_s"),
            )
            turns = TurnController(turn_speed=self._param_float("turn_speed"))
            self.arbiter = BehaviorArbiter(
                front_stop_distance=self._param_float("front_stop_distance"),
                side_stop_distance=self._param_float("side_stop_distance"),
                slow_distance=self._param_float("slow_distance"),
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

            self._cmd_message_kind = self._select_cmd_message_kind()
            if self._cmd_message_kind == "Twist":
                self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
            else:
                self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
                self._cmd_message_kind = "TwistStamped"

            self._init_qr_subscription(Image, qos_profile_sensor_data)
            self._ensure_scan_subscription(LaserScan, qos_profile_sensor_data, force=True)

            period = 1.0 / max(1.0, self._param_float("control_hz"))
            self.timer = self.create_timer(period, self.control_loop)
            self.cmd_status_timer = self.create_timer(2.0, self._cmd_status_check)
            self.diag.log(
                "INFO",
                "[INIT] reactive navigator started "
                f"dry_run={self.dry_run} enable_motion={self.enable_motion} "
                f"cmd={self._cmd_message_kind}:{self.cmd_topic} nav={self.nav_module.name} "
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

        def _select_cmd_message_kind(self) -> str:
            requested = self.cmd_msg_type.strip().lower()
            if requested in ("twist", "geometry_msgs/msg/twist"):
                return "Twist"
            if requested in ("twiststamped", "twist_stamped", "geometry_msgs/msg/twiststamped"):
                return "TwistStamped"
            if requested != "auto":
                self.get_logger().warn(f"[CMD] unknown cmd_msg_type={self.cmd_msg_type!r}; falling back to auto")

            topic_types = self._topic_types(self.cmd_topic)
            if "geometry_msgs/msg/Twist" in topic_types:
                return "Twist"
            if "geometry_msgs/msg/TwistStamped" in topic_types:
                return "TwistStamped"
            self.get_logger().warn(
                f"[CMD] could not auto-detect message type for {self.cmd_topic}; "
                "defaulting to TwistStamped. If the robot does not move, run with -p cmd_msg_type:=Twist"
            )
            return "TwistStamped"

        def _topic_types(self, topic: str) -> list[str]:
            for name, types in self.get_topic_names_and_types():
                if name == topic:
                    return list(types)
            return []

        def _cmd_status_check(self) -> None:
            sub_count = self.cmd_pub.get_subscription_count()
            topic_types = self._topic_types(self.cmd_topic)
            if sub_count > 0:
                if not self._cmd_subscriber_seen:
                    self.diag.log(
                        "INFO",
                        f"[CMD] {self.cmd_topic} has subscribers={sub_count}; "
                        f"publishing={self._cmd_message_kind}; graph_types={topic_types or 'unknown'}",
                    )
                self._cmd_subscriber_seen = True
                return

            now = time.monotonic()
            if now - self._last_cmd_status_log < 2.0:
                return
            self._last_cmd_status_log = now
            self.diag.log(
                "WARN",
                f"[CMD] no subscribers detected on {self.cmd_topic}; "
                f"publishing={self._cmd_message_kind}; graph_types={topic_types or 'unknown'}. "
                "The base will not receive velocity commands.",
            )

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
            safe_command, safety_reason = self._adaptive_safety_command(
                output.command,
                sectors,
                lidar_fresh,
                state=output.state,
                reason=output.reason,
            )
            published_command, motion_enabled = self._publish_command(safe_command, output.publish_motion)
            self._write_persistent_log(
                output=output,
                sectors=sectors,
                lidar_age=lidar_age,
                image_age=math.inf if self.last_image_time is None else now - self.last_image_time,
                signal=signal,
                nav_suggestion=nav_suggestion,
                requested_command=output.command,
                safety_command=safe_command,
                safety_reason=safety_reason,
                published_command=published_command,
                motion_enabled=motion_enabled,
                dt=dt,
            )
            self._emit_diagnostics(
                output,
                sectors,
                lidar_age,
                safe_command,
                safety_reason,
                published_command,
                motion_enabled,
            )
            self.last_state = output.state
            self.last_reason = output.reason

        def _adaptive_safety_command(
            self,
            command: TwistCommand,
            sectors: Optional[SectorMap],
            lidar_fresh: bool,
            *,
            state: str = "",
            reason: str = "",
        ) -> tuple[TwistCommand, str]:
            if sectors is None or not lidar_fresh:
                return TwistCommand(), "NO_FRESH_LIDAR"
            if sectors.valid_count == 0:
                return TwistCommand(), "NO_VALID_LIDAR_POINTS"
            if state in ("MANUAL_STOP", "QR_SCAN", "SENSOR_CHECK", "IDLE"):
                return TwistCommand(), f"LOCKED_{state}"

            front = sectors.distance("front")
            front_center = sectors.distance("front_center")
            front_left = sectors.distance("front_left")
            front_right = sectors.distance("front_right")
            left = sectors.distance("left")
            right = sectors.distance("right")

            if front is None and front_center is None:
                return TwistCommand(), "NO_FRONT_LIDAR_POINTS"

            stop_d = self._param_float("front_stop_distance")
            side_stop_d = self._param_float("side_stop_distance")
            slow_d = self._param_float("slow_distance")
            turn_clearance = self._param_float("turn_clearance")
            max_yaw = abs(self._param_float("max_yaw"))
            cruise_yaw = min(max_yaw, abs(self._param_float("cruise_yaw_limit")))
            evasive_yaw = min(max_yaw, abs(self._param_float("evasive_yaw_limit")))
            soft_avoid_distance = max(slow_d, self._param_float("soft_avoid_distance"))
            soft_avoid_gain = max(0.0, self._param_float("soft_avoid_gain"))
            footprint = self._footprint_map(sectors)
            local_path = self._local_path_map(sectors)
            traversable_gap = self._best_traversable_gap(sectors)
            footprint_lane = footprint["lane_nearest_x"]
            footprint_left = footprint["left_clearance"]
            footprint_right = footprint["right_clearance"]
            footprint_turn = 1.0 if footprint_left >= footprint_right else -1.0
            footprint_slow = max(slow_d, self._param_float("footprint_slow_distance"))
            path_tolerance = max(3.0, self._param_float("path_forward_heading_tolerance_deg"))
            drive_heading_limit = max(path_tolerance, self._param_float("drive_heading_limit_deg"))
            path_required = max(
                self._param_float("path_required_clearance_m"),
                self._param_float("footprint_lookahead_m"),
            )
            search_yaw = min(evasive_yaw, abs(self._param_float("search_turn_yaw")))
            search_confirm_cycles = max(1, self._param_int("search_clear_confirm_cycles"))
            base_speed = max(0.0, self._param_float("base_speed"))
            narrow_speed = max(0.0, self._param_float("narrow_speed"))

            front_value = front if front is not None else front_center
            front_center_value = front_center if front_center is not None else front_value
            nearest_front = min(front_value, front_center_value)

            left_score = self._clearance_score(left, front_left)
            right_score = self._clearance_score(right, front_right)
            preferred_turn = 1.0 if left_score >= right_score else -1.0
            preferred_score = max(left_score, right_score)

            left_critical = left is not None and left < side_stop_d
            right_critical = right is not None and right < side_stop_d
            if state == "EMERGENCY_STOP":
                if "LIDAR_STALE" in reason or "NO_VALID" in reason:
                    return TwistCommand(), f"LOCKED_{reason}"
                if left_critical and right_critical:
                    return TwistCommand(), "EMERGENCY_BOTH_SIDES_TOO_CLOSE"
                if left_critical:
                    return TwistCommand(0.0, -evasive_yaw), "EMERGENCY_LEFT_SIDE_STEER_RIGHT"
                if right_critical:
                    return TwistCommand(0.0, evasive_yaw), "EMERGENCY_RIGHT_SIDE_STEER_LEFT"

            linear = max(0.0, min(float(command.linear_x), base_speed))
            yaw_limit = max_yaw if state.startswith(("TURNING_", "ALIGNING_")) else cruise_yaw
            yaw = max(-yaw_limit, min(yaw_limit, float(command.angular_z)))

            straight_path_ready = (
                local_path["straight_clearance_m"] >= path_required
                and nearest_front >= slow_d
                and (footprint_lane is None or footprint_lane >= footprint_slow)
            )
            drive_heading = self._drive_heading(local_path, traversable_gap, drive_heading_limit=drive_heading_limit)
            diagonal_path_ready = (
                abs(drive_heading) <= drive_heading_limit
                and local_path["best_clearance_m"] >= min(path_required, footprint_slow)
                and nearest_front >= stop_d + 0.08
                and (footprint_lane is None or footprint_lane >= stop_d + 0.08)
            )
            front_gap_ready = traversable_gap is not None and abs(traversable_gap.center_deg) <= path_tolerance
            path_ready = straight_path_ready or diagonal_path_ready or front_gap_ready
            search_needed = (
                not path_ready
                and (
                    traversable_gap is None
                    or (not diagonal_path_ready and abs(traversable_gap.center_deg) > drive_heading_limit)
                    or local_path["best_clearance_m"] < path_required
                    or local_path["straight_clearance_m"] < path_required
                )
            )
            if search_needed or self._search_turn_sign != 0.0:
                if path_ready:
                    self._search_clear_cycles += 1
                    if self._search_clear_cycles >= search_confirm_cycles:
                        self._search_turn_sign = 0.0
                        self._search_clear_cycles = 0
                    else:
                        sign = self._search_turn_sign or footprint_turn
                        return TwistCommand(0.0, sign * search_yaw), "SEARCH_CONFIRMING_CLEAR_PATH"
                else:
                    self._search_clear_cycles = 0
                    if traversable_gap is not None and abs(traversable_gap.center_deg) > drive_heading_limit:
                        self._search_turn_sign = 1.0 if traversable_gap.center_deg > 0.0 else -1.0
                        return (
                            TwistCommand(0.0, self._search_turn_sign * search_yaw),
                            "SEGMENTED_GAP_TURN_IN_PLACE",
                        )
                    if self._search_turn_sign == 0.0:
                        best_heading = float(local_path.get("best_heading_deg") or 0.0)
                        self._search_turn_sign = (
                            1.0 if best_heading > 1.0 else -1.0 if best_heading < -1.0 else footprint_turn
                        )
                    return TwistCommand(0.0, self._search_turn_sign * search_yaw), "NO_TRAVERSABLE_GAP_SEARCH_TURN"

            if nearest_front < stop_d:
                if preferred_score < turn_clearance:
                    return TwistCommand(), "BLOCKED_ALL_SIDES"
                return TwistCommand(0.0, preferred_turn * evasive_yaw), "CRITICAL_FRONT_TURN_TO_OPEN_SIDE"

            if footprint_lane is not None and footprint_lane < stop_d:
                if max(footprint_left, footprint_right) < turn_clearance:
                    return TwistCommand(), "FOOTPRINT_PATH_BLOCKED"
                steer = self._local_path_steer_yaw(local_path, yaw_limit=evasive_yaw, fallback=footprint_turn)
                return TwistCommand(0.0, steer), "FOOTPRINT_STOP_TURN_TO_CLEAR_SIDE"

            if left_critical and right_critical:
                return TwistCommand(), "BOTH_SIDES_TOO_CLOSE"
            if left_critical:
                return TwistCommand(0.0, -evasive_yaw), "LEFT_CRITICAL_STEER_RIGHT"
            if right_critical:
                return TwistCommand(0.0, evasive_yaw), "RIGHT_CRITICAL_STEER_LEFT"

            if footprint_lane is not None and footprint_lane < footprint_slow:
                steer = self._local_path_steer_yaw(local_path, yaw_limit=evasive_yaw, fallback=footprint_turn)
                return (
                    TwistCommand(narrow_speed, steer),
                    "FOOTPRINT_PATH_RISK_SLOW_STEER",
                )

            if local_path["straight_clearance_m"] < self._param_float("footprint_lookahead_m"):
                steer = self._heading_to_yaw(drive_heading, yaw_limit=cruise_yaw)
                straight_is_usable = (
                    abs(drive_heading) <= drive_heading_limit
                    and local_path["straight_clearance_m"] >= min(path_required, footprint_slow)
                )
                speed = narrow_speed if straight_is_usable or diagonal_path_ready else 0.0
                return (
                    TwistCommand(speed, steer),
                    "DRIVE_DIAGONAL_TO_OPEN_SPACE" if speed > 0.0 else "LOCAL_PATH_SELECT_HEADING",
                )

            if nearest_front < slow_d:
                slow_yaw = yaw
                if abs(slow_yaw) < 0.10:
                    slow_yaw = preferred_turn * min(evasive_yaw, 0.25)
                else:
                    slow_yaw = max(-evasive_yaw, min(evasive_yaw, slow_yaw))
                return (
                    TwistCommand(narrow_speed, slow_yaw),
                    "FRONT_CLOSE_SLOW_TURN_TO_OPEN_SIDE",
                )

            reason = "NORMAL_ADAPTIVE"
            if not state.startswith(("TURNING_", "ALIGNING_")):
                soft_yaw = self._soft_obstacle_yaw(
                    front_left=front_left,
                    front_right=front_right,
                    left=left,
                    right=right,
                    soft_distance=soft_avoid_distance,
                    gain=soft_avoid_gain,
                    yaw_limit=cruise_yaw,
                )
                if abs(soft_yaw) > 0.02:
                    curve_yaw = self._curve_heading_yaw(
                        traversable_gap,
                        soft_yaw=soft_yaw,
                        yaw_limit=cruise_yaw,
                    )
                    yaw = self._avoid_reverse_turn(soft_yaw + curve_yaw, yaw_limit=cruise_yaw)
                    linear = min(linear, base_speed)
                    reason = "FRONT_CLEAR_ADAPTIVE_CURVE_STEER"
                elif footprint["side_bias"] != 0.0:
                    yaw = self._avoid_reverse_turn(
                        self._local_path_steer_yaw(local_path, yaw_limit=cruise_yaw, fallback=footprint_turn),
                        yaw_limit=cruise_yaw,
                    )
                    reason = "LOCAL_PATH_SOFT_STEER"
                elif abs(yaw) < 0.03:
                    yaw = 0.0
                    reason = "FRONT_CLEAR_GO_STRAIGHT"

            if left is not None and left < side_stop_d * 1.6:
                linear = min(linear, narrow_speed)
                if yaw > 0.0:
                    yaw = -cruise_yaw
                    reason = "LEFT_TOO_CLOSE_STEER_RIGHT"
            if right is not None and right < side_stop_d * 1.6:
                linear = min(linear, narrow_speed)
                if yaw < 0.0:
                    yaw = cruise_yaw
                    reason = "RIGHT_TOO_CLOSE_STEER_LEFT"

            return TwistCommand(linear, yaw), reason

        def _footprint_map(self, sectors: SectorMap) -> Dict[str, Any]:
            half_width = max(0.05, self._param_float("robot_width_m") * 0.5)
            margin = max(0.0, self._param_float("footprint_margin_m"))
            lookahead = max(0.20, self._param_float("footprint_lookahead_m"))
            swept_half_width = half_width + margin
            left_clearance = sectors.distance("front_left", sectors.distance("left", 0.0)) or 0.0
            right_clearance = sectors.distance("front_right", sectors.distance("right", 0.0)) or 0.0
            lane_nearest_x = None
            left_risk = 0.0
            right_risk = 0.0

            for point in sectors.points:
                angle = math.radians(point.angle_deg)
                x = point.distance_m * math.cos(angle)
                y = point.distance_m * math.sin(angle)
                if x <= 0.0 or x > lookahead:
                    continue
                side_risk = max(0.0, (lookahead - x) / lookahead)
                if y >= 0.0:
                    left_risk += side_risk
                else:
                    right_risk += side_risk
                if abs(y) <= swept_half_width:
                    if lane_nearest_x is None or x < lane_nearest_x:
                        lane_nearest_x = x

            side_bias = 0.0
            total_risk = left_risk + right_risk
            if total_risk > 0.0:
                side_bias = (right_risk - left_risk) / total_risk

            return {
                "lane_nearest_x": lane_nearest_x,
                "left_clearance": float(left_clearance),
                "right_clearance": float(right_clearance),
                "left_risk": left_risk,
                "right_risk": right_risk,
                "side_bias": side_bias,
                "local_path": self._local_path_map(sectors),
                "best_gap": self._best_traversable_gap_debug(sectors),
            }

        def _footprint_steer_yaw(self, footprint: Dict[str, Any], *, yaw_limit: float, fallback: float) -> float:
            side_bias = float(footprint.get("side_bias") or 0.0)
            if abs(side_bias) < 0.05:
                side_bias = fallback
            return max(-yaw_limit, min(yaw_limit, side_bias * yaw_limit))

        def _local_path_map(self, sectors: SectorMap) -> Dict[str, float]:
            headings = self._candidate_headings()
            straight_clearance = self._path_clearance(sectors, 0.0)
            best_heading = 0.0
            best_clearance = -1.0
            best_score = -1.0
            for heading in headings:
                clearance = self._path_clearance(sectors, heading)
                score = clearance - abs(heading) * 0.003
                if score > best_score:
                    best_score = score
                    best_heading = heading
                    best_clearance = clearance
            return {
                "best_heading_deg": best_heading,
                "best_clearance_m": best_clearance,
                "straight_clearance_m": straight_clearance,
            }

        def _candidate_headings(self) -> list[float]:
            max_heading = max(10.0, self._param_float("local_path_heading_deg"))
            step = max(2.0, self._param_float("local_path_step_deg"))
            headings = [0.0]
            count = int(max_heading // step)
            for index in range(1, count + 1):
                value = index * step
                headings.extend([value, -value])
            return headings

        def _path_clearance(self, sectors: SectorMap, heading_deg: float) -> float:
            half_width = max(0.05, self._param_float("robot_width_m") * 0.5)
            swept_half_width = half_width + max(0.0, self._param_float("footprint_margin_m"))
            lookahead = max(0.20, self._param_float("footprint_lookahead_m"))
            heading = math.radians(heading_deg)
            cos_h = math.cos(heading)
            sin_h = math.sin(heading)
            clearance = lookahead
            for point in sectors.points:
                angle = math.radians(point.angle_deg)
                x = point.distance_m * math.cos(angle)
                y = point.distance_m * math.sin(angle)
                along = x * cos_h + y * sin_h
                lateral = -x * sin_h + y * cos_h
                if along <= 0.0 or along > lookahead:
                    continue
                if abs(lateral) <= swept_half_width:
                    clearance = min(clearance, along)
            return clearance

        def _local_path_steer_yaw(
            self,
            local_path: Dict[str, float],
            *,
            yaw_limit: float,
            fallback: float,
        ) -> float:
            heading = float(local_path.get("best_heading_deg") or 0.0)
            if abs(heading) < 1.0:
                heading = fallback * max(10.0, self._param_float("local_path_step_deg"))
            max_heading = max(1.0, self._param_float("local_path_heading_deg"))
            yaw = heading / max_heading * yaw_limit
            return max(-yaw_limit, min(yaw_limit, yaw))

        def _heading_to_yaw(self, heading_deg: float, *, yaw_limit: float) -> float:
            max_heading = max(1.0, self._param_float("local_path_heading_deg"))
            yaw = heading_deg / max_heading * yaw_limit
            return max(-yaw_limit, min(yaw_limit, yaw))

        def _drive_heading(self, local_path: Dict[str, float], gap, *, drive_heading_limit: float) -> float:
            path_heading = float(local_path.get("best_heading_deg") or 0.0)
            if gap is None:
                return path_heading
            gap_heading = float(gap.center_deg)
            if abs(gap_heading) <= drive_heading_limit:
                return gap_heading
            return path_heading

        def _curve_heading_yaw(self, gap, *, soft_yaw: float, yaw_limit: float) -> float:
            if gap is None:
                return 0.0
            heading = float(gap.center_deg)
            if abs(heading) <= self._param_float("path_forward_heading_tolerance_deg"):
                return 0.0
            heading_yaw = self._heading_to_yaw(heading, yaw_limit=yaw_limit)
            if abs(soft_yaw) > 0.02 and heading_yaw * soft_yaw < 0.0:
                return 0.0
            gain = max(0.0, self._param_float("curve_heading_gain"))
            return max(-yaw_limit, min(yaw_limit, heading_yaw * gain))

        def _avoid_reverse_turn(self, yaw: float, *, yaw_limit: float) -> float:
            if abs(yaw) < 0.02:
                return 0.0
            now = time.monotonic()
            sign = 1.0 if yaw > 0.0 else -1.0
            cooldown = max(0.0, self._param_float("reverse_turn_cooldown_s"))
            if (
                self._last_motion_turn_sign != 0.0
                and sign != self._last_motion_turn_sign
                and now - self._last_motion_turn_time < cooldown
            ):
                yaw *= 0.35
            else:
                self._last_motion_turn_sign = sign
                self._last_motion_turn_time = now
            return max(-yaw_limit, min(yaw_limit, yaw))

        def _best_traversable_gap(self, sectors: SectorMap):
            gaps = traversable_gaps(
                sectors.points,
                robot_width_m=max(0.10, self._param_float("robot_width_m")),
                margin_m=max(0.0, self._param_float("footprint_margin_m")),
                min_clearance_m=max(
                    self._param_float("path_required_clearance_m"),
                    self._param_float("footprint_lookahead_m"),
                ),
            )
            return gaps[0] if gaps else None

        def _best_traversable_gap_debug(self, sectors: SectorMap) -> Dict[str, Any]:
            gap = self._best_traversable_gap(sectors)
            if gap is None:
                return {"available": False}
            return {
                "available": True,
                "start_deg": gap.start_deg,
                "end_deg": gap.end_deg,
                "center_deg": gap.center_deg,
                "width_deg": gap.width_deg,
                "physical_width_m": gap.physical_width_m,
                "min_distance_m": gap.min_distance_m,
                "score": gap.score,
            }

        def _soft_obstacle_yaw(
            self,
            *,
            front_left: Optional[float],
            front_right: Optional[float],
            left: Optional[float],
            right: Optional[float],
            soft_distance: float,
            gain: float,
            yaw_limit: float,
        ) -> float:
            left_risk = self._risk(front_left, soft_distance) * 1.0 + self._risk(left, soft_distance) * 0.45
            right_risk = self._risk(front_right, soft_distance) * 1.0 + self._risk(right, soft_distance) * 0.45
            yaw = gain * (right_risk - left_risk)
            return max(-yaw_limit, min(yaw_limit, yaw))

        def _risk(self, distance: Optional[float], soft_distance: float) -> float:
            if distance is None or soft_distance <= 0.0:
                return 0.0
            return max(0.0, (soft_distance - distance) / soft_distance)

        def _clearance_score(self, side: Optional[float], front_side: Optional[float]) -> float:
            values = [value for value in (side, front_side) if value is not None]
            if not values:
                return 0.0
            return min(values)

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
            safety_command: TwistCommand,
            safety_reason: str,
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
            footprint_debug = self._footprint_map(sectors) if sectors is not None else {}
            record = {
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
                    "search_turn_sign": self._json_float(self._search_turn_sign),
                    "search_clear_cycles": self._search_clear_cycles,
                    "last_motion_turn_sign": self._json_float(self._last_motion_turn_sign),
                    "last_motion_turn_age_s": self._json_float(time.monotonic() - self._last_motion_turn_time)
                    if self._last_motion_turn_time > 0.0
                    else None,
                },
                "command": {
                    "requested_linear_x": self._json_float(requested_command.linear_x),
                    "requested_angular_z": self._json_float(requested_command.angular_z),
                    "safety_linear_x": self._json_float(safety_command.linear_x),
                    "safety_angular_z": self._json_float(safety_command.angular_z),
                    "safety_reason": safety_reason,
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
                "lidar": {
                    "valid_count": sectors.valid_count if sectors is not None else 0,
                    "total_count": sectors.total_count if sectors is not None else 0,
                    "sector_distance_m": sector_distances,
                    "sector_raw_min_m": sector_raw_min,
                    "sector_valid_count": sector_counts,
                    "left_minus_right_m": self._json_float(left_minus_right),
                },
                "footprint": self._json_clean(footprint_debug),
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

        def _emit_diagnostics(
            self,
            output,
            sectors: Optional[SectorMap],
            lidar_age: float,
            safe_command: TwistCommand,
            safety_reason: str,
            published_command: TwistCommand,
            motion_enabled: bool,
        ) -> None:
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
                    f"state={output.state} reason={output.reason} "
                    f"cmd=({safe_command.linear_x:.3f},{safe_command.angular_z:.3f}) "
                    f"published=({published_command.linear_x:.3f},{published_command.angular_z:.3f}) "
                    f"safety={safety_reason} "
                    f"dry_run={self.dry_run} enable_motion={self.enable_motion} motion={motion_enabled} "
                    f"scan_topic={self.current_scan_topic or 'none'} scan_count={self.scan_count} "
                    f"lidar_age={self._fmt_age(freshness.lidar_age_s)} "
                    f"image_age={self._fmt_age(freshness.image_age_s)} "
                    f"signal={self.last_signal.direction}/{self.last_signal.reason}",
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
                    turn_hint=float(debug.get("gap_center", safe_command.angular_z * 90.0)),
                    speed=safe_command.linear_x,
                    yaw=safe_command.angular_z,
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
        from .wall_following import NavigationObservation, create_navigation_module
    except ImportError:  # pragma: no cover
        from behavior_arbiter import ArbiterInput, BehaviorArbiter, SignalState
        from lidar_sectors import extract_sectors
        from qr_logger import QRLogger
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

    nav = create_navigation_module("forward_avoid")
    suggestion = nav.compute(NavigationObservation(sectors, time.monotonic(), 0.1))
    assert suggestion.mode in ("AVOID_OBSTACLE", "FORWARD_AVOID")

    class FakeCorridorScan(FakeScan):
        def __init__(self):
            self.ranges = [2.0] * 361
            for deg in range(-110, -69):
                self.ranges[deg + 180] = 0.75
            for deg in range(70, 111):
                self.ranges[deg + 180] = 0.80

    corridor_sectors = extract_sectors(FakeCorridorScan())
    corridor_nav = create_navigation_module("forward_avoid")
    corridor_suggestion = corridor_nav.compute(NavigationObservation(corridor_sectors, time.monotonic(), 0.1))
    assert corridor_suggestion.command.linear_x > 0.0
    assert abs(corridor_suggestion.command.angular_z) < 0.01

    class FakeCurveScan(FakeScan):
        def __init__(self):
            self.ranges = [2.0] * 361
            for deg in range(-70, -44):
                self.ranges[deg + 180] = 0.48
            for deg in range(-110, -69):
                self.ranges[deg + 180] = 0.45

    curve_sectors = extract_sectors(FakeCurveScan())
    curve_nav = create_navigation_module("forward_avoid")
    curve_suggestion = curve_nav.compute(NavigationObservation(curve_sectors, time.monotonic(), 0.1))
    assert curve_suggestion.reason == "FRONT_CLEAR_ADAPTIVE_CURVE_STEER"
    assert curve_suggestion.command.linear_x > 0.0
    assert curve_suggestion.command.angular_z > 0.0

    class FakeFootprintRiskScan(FakeScan):
        def __init__(self):
            self.ranges = [2.0] * 361
            for deg in range(-20, -12):
                self.ranges[deg + 180] = 0.68

    footprint_sectors = extract_sectors(FakeFootprintRiskScan())
    footprint_nav = create_navigation_module("forward_avoid")
    footprint_suggestion = footprint_nav.compute(NavigationObservation(footprint_sectors, time.monotonic(), 0.1))
    assert footprint_suggestion.reason in (
        "FOOTPRINT_PATH_RISK",
        "LOCAL_PATH_SELECT_HEADING",
        "TURN_IN_PLACE_TO_TRAVERSABLE_GAP",
        "DRIVE_DIAGONAL_TO_OPEN_SPACE",
    )
    assert footprint_suggestion.command.linear_x > 0.0
    assert footprint_suggestion.command.angular_z > 0.0

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

    with tempfile.TemporaryDirectory() as tmp:
        logger = QRLogger(Path(tmp) / "qr_log.jsonl", confirm_count=2)
        assert logger.observe("checkpoint-1") is None
        event = logger.observe("checkpoint-1", robot_state="QR_SCAN")
        assert event is not None and event.logged

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
