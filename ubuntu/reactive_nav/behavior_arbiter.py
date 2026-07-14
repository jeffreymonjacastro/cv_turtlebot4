#!/usr/bin/env python3
"""Safety-first behavior arbitration for LiDAR, YOLO signs, and QR events."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import time
from typing import Deque, Dict, Optional

try:
    from .lidar_sectors import SectorMap
    from .turn_controller import TurnController
    from .wall_following import NavigationSuggestion, TwistCommand
except ImportError:  # pragma: no cover - direct script fallback
    from lidar_sectors import SectorMap
    from turn_controller import TurnController
    from wall_following import NavigationSuggestion, TwistCommand


@dataclass(frozen=True)
class SignalState:
    direction: str = "none"
    confidence: float = 0.0
    bbox_area_ratio: float = 0.0
    bbox_center_x_ratio: float = 0.0
    actionable: bool = False
    timestamp: float = 0.0
    stale: bool = True
    event_id: str = ""
    reason: str = "missing"
    raw_class: str = ""


@dataclass(frozen=True)
class ArbiterInput:
    sectors: Optional[SectorMap]
    lidar_fresh: bool
    nav_suggestion: Optional[NavigationSuggestion]
    signal: SignalState
    qr_recent: bool
    now: float


@dataclass(frozen=True)
class ArbiterOutput:
    command: TwistCommand
    state: str
    reason: str
    publish_motion: bool
    debug: Dict[str, float | str] = field(default_factory=dict)


class SignDebouncer:
    def __init__(
        self,
        confirm_window: int = 8,
        confirm_count: int = 5,
        min_confidence: float = 0.70,
        min_area_ratio: float = 0.03,
        cooldown_s: float = 3.0,
    ):
        self.confirm_window = max(1, confirm_window)
        self.confirm_count = max(1, confirm_count)
        self.min_confidence = min_confidence
        self.min_area_ratio = min_area_ratio
        self.cooldown_s = cooldown_s
        self._recent: Deque[str] = deque(maxlen=self.confirm_window)
        self._cooldown_until = 0.0
        self._consumed_events: set[str] = set()
        self._last_diagnostics: Dict[str, float | str | bool] = {
            "yolo_event_status": "idle",
            "yolo_rejection_reason": "none",
            "yolo_confirmation_count": 0.0,
            "yolo_confirmation_required": float(self.confirm_count),
            "yolo_confirmation_progress": "0/0",
            "yolo_event": "NONE",
        }

    def update(self, signal: SignalState, now: float) -> Optional[str]:
        if now < self._cooldown_until:
            self._recent.append("none")
            self._set_diagnostics(signal, "rejected", "cooldown_active")
            return None
        if signal.stale or not signal.actionable:
            self._recent.append("none")
            self._set_diagnostics(
                signal,
                "rejected",
                "stale" if signal.stale else "not_actionable",
            )
            return None
        direction = signal.direction.lower()
        if direction not in ("left", "right", "stop"):
            self._recent.append("none")
            self._set_diagnostics(signal, "rejected", f"unsupported_direction:{direction}")
            return None
        if signal.confidence < self.min_confidence:
            self._recent.append("none")
            self._set_diagnostics(signal, "rejected", "low_confidence")
            return None
        if signal.bbox_area_ratio < self.min_area_ratio:
            self._recent.append("none")
            self._set_diagnostics(signal, "rejected", "low_area_ratio")
            return None

        self._recent.append(direction)
        count = sum(1 for item in self._recent if item == direction)
        if count < self.confirm_count:
            self._set_diagnostics(signal, "candidate", "awaiting_confirmation", count=count)
            return None

        if self.event_id(signal) in self._consumed_events:
            self._set_diagnostics(signal, "rejected", "event_already_consumed", count=count)
            return None
        self._set_diagnostics(signal, "validated", "none", count=count, event=direction.upper())
        return direction.upper()

    def event_id(self, signal: SignalState) -> str:
        return signal.event_id or f"{signal.direction}:{signal.timestamp:.3f}"

    def consume(self, signal: SignalState) -> None:
        self._consumed_events.add(self.event_id(signal))
        if len(self._consumed_events) > 64:
            self._consumed_events = set(list(self._consumed_events)[-32:])

    def start_cooldown(self, now: float) -> None:
        self._cooldown_until = now + self.cooldown_s
        self._recent.clear()

    @property
    def cooldown_remaining(self) -> float:
        return max(0.0, self._cooldown_until - time.monotonic())

    @property
    def diagnostics(self) -> Dict[str, float | str | bool]:
        payload = dict(self._last_diagnostics)
        payload["sign_cooldown_remaining_s"] = self.cooldown_remaining
        return payload

    def mark_decision(self, *, status: str, rejection_reason: str = "none") -> None:
        self._last_diagnostics = {
            **self._last_diagnostics,
            "yolo_event_status": status,
            "yolo_rejection_reason": rejection_reason,
        }

    def _set_diagnostics(
        self,
        signal: SignalState,
        status: str,
        rejection_reason: str,
        *,
        count: Optional[int] = None,
        event: Optional[str] = None,
    ) -> None:
        direction = signal.direction.lower()
        observed_count = (
            sum(1 for item in self._recent if item == direction)
            if direction in ("left", "right", "stop")
            else 0
        )
        if count is None:
            count = observed_count
        required = self.confirm_count
        event_value = event or ("NONE" if direction == "none" else direction.upper())
        self._last_diagnostics = {
            "yolo_event_status": status,
            "yolo_rejection_reason": rejection_reason,
            "yolo_confirmation_count": float(count),
            "yolo_confirmation_required": float(required),
            "yolo_confirmation_progress": f"{count}/{required}",
            "yolo_event": event_value,
            "yolo_event_id": signal.event_id,
        }


class BehaviorArbiter:
    def __init__(
        self,
        *,
        front_stop_distance: float = 0.32,
        front_stop_clear_distance: float = 0.40,
        side_stop_distance: float = 0.14,
        side_stop_clear_distance: float = 0.20,
        emergency_clear_cycles: int = 3,
        slow_distance: float = 0.55,
        qr_hold_s: float = 0.8,
        turn_clearance: float = 0.42,
        front_corner_avoid_distance: float = 0.62,
        corner_slow_speed: float = 0.035,
        enable_corner_yaw_veto: bool = True,
        enable_corner_slowdown: bool = True,
        enable_side_yaw_veto: bool = True,
        enable_anti_spin: bool = False,
        anti_spin_yaw_threshold: float = 0.42,
        anti_spin_linear_threshold: float = 0.025,
        anti_spin_trigger_cycles: int = 8,
        anti_spin_recovery_speed: float = 0.035,
        angular_smoothing_alpha: float = 1.0,
        sign_debouncer: Optional[SignDebouncer] = None,
        turn_controller: Optional[TurnController] = None,
    ):
        self.front_stop_distance = front_stop_distance
        self.front_stop_clear_distance = max(front_stop_distance, front_stop_clear_distance)
        self.side_stop_distance = side_stop_distance
        self.side_stop_clear_distance = max(side_stop_distance, side_stop_clear_distance)
        self.emergency_clear_cycles = max(1, emergency_clear_cycles)
        self.slow_distance = slow_distance
        self.qr_hold_s = qr_hold_s
        self.turn_clearance = turn_clearance
        self.front_corner_avoid_distance = front_corner_avoid_distance
        self.corner_slow_speed = max(0.0, corner_slow_speed)
        self.enable_corner_yaw_veto = enable_corner_yaw_veto
        self.enable_corner_slowdown = enable_corner_slowdown
        self.enable_side_yaw_veto = enable_side_yaw_veto
        self.enable_anti_spin = enable_anti_spin
        self.anti_spin_yaw_threshold = max(0.0, anti_spin_yaw_threshold)
        self.anti_spin_linear_threshold = max(0.0, anti_spin_linear_threshold)
        self.anti_spin_trigger_cycles = max(1, anti_spin_trigger_cycles)
        self.anti_spin_recovery_speed = max(0.0, anti_spin_recovery_speed)
        self.angular_smoothing_alpha = max(0.0, min(1.0, angular_smoothing_alpha))
        self.signs = sign_debouncer or SignDebouncer()
        self.turns = turn_controller or TurnController()
        self._qr_hold_until = 0.0
        self._emergency_active = False
        self._emergency_clear_counter = 0
        self._emergency_last_reason = "NONE"
        self._emergency_trigger_count = 0
        self._spin_candidate_cycles = 0
        self._previous_smoothed_yaw: Optional[float] = None
        self._recovery_turn_sign = 0.0
        self._recovery_turn_until = 0.0
        self._last_output_state = "INIT"
        self._state_started_at = time.monotonic()
        self._recovery_started_at: Optional[float] = None

    def decide(self, inputs: ArbiterInput) -> ArbiterOutput:
        sectors = inputs.sectors
        now = inputs.now

        if sectors is None:
            return self._output(TwistCommand(), "SENSOR_CHECK", "NO_LIDAR_SECTOR_MAP", False)
        if not inputs.lidar_fresh:
            return self._output(TwistCommand(), "EMERGENCY_STOP", "LIDAR_STALE_OR_NO_CALLBACK", True)
        if sectors.valid_count == 0:
            return self._output(TwistCommand(), "EMERGENCY_STOP", "NO_VALID_LIDAR_POINTS", True)

        emergency = self._update_emergency_state(sectors)
        if emergency is not None:
            return self._output(TwistCommand(), "EMERGENCY_STOP", emergency, True)

        if self.turns.active:
            step = self.turns.step(sectors, now)
            if not step.active:
                self.signs.start_cooldown(now)
            return self._output(step.command, step.state, step.reason, True, step.debug)

        if inputs.qr_recent:
            self._qr_hold_until = max(self._qr_hold_until, now + self.qr_hold_s)
        if now < self._qr_hold_until:
            return self._output(TwistCommand(), "QR_SCAN", "QR_VISIBLE_OR_RECENTLY_LOGGED", True)

        confirmed = self.signs.update(inputs.signal, now)
        if confirmed in ("LEFT", "RIGHT"):
            turn_block = self._turn_block_reason(sectors, confirmed)
            if turn_block:
                self.signs.mark_decision(status="rejected", rejection_reason=turn_block)
                return self._output(TwistCommand(), "SIGN_CANDIDATE", turn_block, True)
            if self.turns.start(confirmed, now):
                self.signs.consume(inputs.signal)
                self.signs.mark_decision(status="accepted", rejection_reason="none")
                step = self.turns.step(sectors, now)
                return self._output(step.command, step.state, f"SIGN_CONFIRMED_{confirmed}", True, step.debug)
        if confirmed == "STOP":
            if self.turns.start("UTURN", now):
                self.signs.consume(inputs.signal)
                self.signs.mark_decision(status="accepted", rejection_reason="none")
                step = self.turns.step(sectors, now)
                return self._output(step.command, step.state, "STOP_SIGN_CONFIRMED_UTURN", True, step.debug)

        if inputs.nav_suggestion is None:
            return self._output(TwistCommand(), "IDLE", "NO_NAVIGATION_SUGGESTION", True)

        command, safety_debug = self._apply_safety_limits(inputs.nav_suggestion.command, sectors)
        if inputs.nav_suggestion.mode == "RECOVERY":
            command, recovery_debug = self._stabilize_recovery_command(command, sectors, now)
            safety_debug.update(recovery_debug)
        debug = dict(inputs.nav_suggestion.debug)
        debug.update(safety_debug)
        return self._output(
            command,
            inputs.nav_suggestion.mode,
            inputs.nav_suggestion.reason,
            True,
            debug,
        )

    def _emergency_trigger_reason(self, sectors: SectorMap) -> Optional[str]:
        front_center = sectors.distance("front_center")
        front = sectors.distance("front")
        left = sectors.distance("left")
        right = sectors.distance("right")
        if front_center is not None and front_center < self.front_stop_distance:
            return f"FRONT_CENTER_TOO_CLOSE_{front_center:.2f}m"
        front_sector_stop = self.front_stop_distance * 0.75
        if front is not None and front < front_sector_stop:
            return f"FRONT_TOO_CLOSE_{front:.2f}m"
        if left is not None and left < self.side_stop_distance:
            return f"LEFT_SIDE_TOO_CLOSE_{left:.2f}m"
        if right is not None and right < self.side_stop_distance:
            return f"RIGHT_SIDE_TOO_CLOSE_{right:.2f}m"
        return None

    def _emergency_clear_ready(self, sectors: SectorMap) -> bool:
        front_center = sectors.distance("front_center")
        front = sectors.distance("front")
        left = sectors.distance("left")
        right = sectors.distance("right")
        checks = []
        if front_center is not None:
            checks.append(front_center >= self.front_stop_clear_distance)
        if front is not None:
            checks.append(front >= self.front_stop_clear_distance)
        if left is not None:
            checks.append(left >= self.side_stop_clear_distance)
        if right is not None:
            checks.append(right >= self.side_stop_clear_distance)
        return all(checks) if checks else False

    def _update_emergency_state(self, sectors: SectorMap) -> Optional[str]:
        trigger_reason = self._emergency_trigger_reason(sectors)
        if self._emergency_active:
            if trigger_reason:
                self._emergency_clear_counter = 0
                self._emergency_last_reason = trigger_reason
                return trigger_reason
            if self._emergency_clear_ready(sectors):
                self._emergency_clear_counter += 1
                if self._emergency_clear_counter >= self.emergency_clear_cycles:
                    self._emergency_active = False
                    self._emergency_clear_counter = 0
                    self._emergency_last_reason = "CLEARED"
                    return None
                return (
                    "EMERGENCY_LATCH_CLEARING_"
                    f"{self._emergency_clear_counter}_OF_{self.emergency_clear_cycles}"
                )
            self._emergency_clear_counter = 0
            return "EMERGENCY_LATCH_WAIT_CLEAR_THRESHOLDS"

        if trigger_reason:
            self._emergency_active = True
            self._emergency_clear_counter = 0
            self._emergency_last_reason = trigger_reason
            self._emergency_trigger_count += 1
            return trigger_reason
        return None

    def _turn_block_reason(self, sectors: SectorMap, direction: str) -> Optional[str]:
        front = sectors.distance("front")
        side_name = "front_left" if direction == "LEFT" else "front_right"
        side = sectors.distance(side_name)
        if front is not None and front < self.front_stop_distance + 0.05:
            return f"TURN_{direction}_BLOCKED_FRONT_{front:.2f}m"
        if side is not None and side < self.turn_clearance:
            return f"TURN_{direction}_BLOCKED_{side_name.upper()}_{side:.2f}m"
        return None

    def _apply_safety_limits(self, command: TwistCommand, sectors: SectorMap) -> tuple[TwistCommand, Dict[str, float | str | bool]]:
        front = sectors.distance("front")
        front_left = sectors.distance("front_left")
        front_right = sectors.distance("front_right")
        left = sectors.distance("left")
        right = sectors.distance("right")
        linear = command.linear_x
        yaw = command.angular_z
        debug: Dict[str, float | str | bool] = {
            "safety_input_linear_x": command.linear_x,
            "safety_input_angular_z": command.angular_z,
            "corner_yaw_veto": "none",
            "corner_opening_turn": "none",
            "corner_slowdown": False,
            "side_yaw_veto": "none",
            "anti_spin_limited": False,
            "angular_smoothing_applied": False,
        }

        if front is not None and front < self.slow_distance:
            linear = min(linear, max(self.corner_slow_speed, 0.04))
            debug["front_slowdown"] = True
        if front_left is not None and front_left < self.front_corner_avoid_distance:
            if self.enable_corner_slowdown:
                linear = min(linear, self.corner_slow_speed)
                debug["corner_slowdown"] = True
            debug["front_left_risk_m"] = front_left
            if self.enable_corner_yaw_veto and yaw > 0.0:
                yaw = min(yaw, 0.0)
                debug["corner_yaw_veto"] = "front_left"
        if front_right is not None and front_right < self.front_corner_avoid_distance:
            if self.enable_corner_slowdown:
                linear = min(linear, self.corner_slow_speed)
                debug["corner_slowdown"] = True
            debug["front_right_risk_m"] = front_right
            if self.enable_corner_yaw_veto and yaw < 0.0:
                yaw = max(yaw, 0.0)
                debug["corner_yaw_veto"] = "front_right"
        if front is not None and front < self.slow_distance:
            min_curve_yaw = max(0.24, self.turns.turn_speed * 0.55)
            if (
                front_left is not None
                and front_right is not None
                and front_left < self.front_corner_avoid_distance
                and front_right >= self.front_corner_avoid_distance
            ):
                yaw = min(yaw, -min_curve_yaw)
                linear = min(linear, max(self.corner_slow_speed, 0.035))
                debug["corner_opening_turn"] = "right"
            elif (
                front_left is not None
                and front_right is not None
                and front_right < self.front_corner_avoid_distance
                and front_left >= self.front_corner_avoid_distance
            ):
                yaw = max(yaw, min_curve_yaw)
                linear = min(linear, max(self.corner_slow_speed, 0.035))
                debug["corner_opening_turn"] = "left"
        if self.enable_side_yaw_veto and left is not None and left < self.side_stop_distance * 1.6 and yaw > 0.0:
            yaw = min(yaw, 0.0)
            debug["side_yaw_veto"] = "left"
        if self.enable_side_yaw_veto and right is not None and right < self.side_stop_distance * 1.6 and yaw < 0.0:
            yaw = max(yaw, 0.0)
            debug["side_yaw_veto"] = "right"

        front_clear = front is None or front >= self.slow_distance
        if (
            self.enable_anti_spin
            and front_clear
            and abs(yaw) >= self.anti_spin_yaw_threshold
            and abs(linear) <= self.anti_spin_linear_threshold
        ):
            self._spin_candidate_cycles += 1
        else:
            self._spin_candidate_cycles = 0
        debug["anti_spin_candidate_cycles"] = float(self._spin_candidate_cycles)
        if self._spin_candidate_cycles >= self.anti_spin_trigger_cycles:
            yaw *= 0.35
            linear = max(linear, self.anti_spin_recovery_speed)
            debug["anti_spin_limited"] = True

        if self.angular_smoothing_alpha < 1.0:
            previous_yaw = 0.0 if self._previous_smoothed_yaw is None else self._previous_smoothed_yaw
            alpha = self.angular_smoothing_alpha
            smoothed_yaw = alpha * yaw + (1.0 - alpha) * previous_yaw
            if abs(smoothed_yaw - yaw) > 1e-6:
                debug["angular_smoothing_applied"] = True
                debug["angular_smoothing_input_yaw"] = yaw
            yaw = smoothed_yaw
            if debug.get("corner_yaw_veto") == "front_left" or debug.get("side_yaw_veto") == "left":
                yaw = min(yaw, 0.0)
                debug["angular_smoothing_veto_clamped"] = True
            elif debug.get("corner_yaw_veto") == "front_right" or debug.get("side_yaw_veto") == "right":
                yaw = max(yaw, 0.0)
                debug["angular_smoothing_veto_clamped"] = True
            self._previous_smoothed_yaw = yaw
        else:
            self._previous_smoothed_yaw = yaw

        debug["safety_output_linear_x"] = linear
        debug["safety_output_angular_z"] = yaw
        vetoes = [
            str(debug[name])
            for name in ("corner_yaw_veto", "side_yaw_veto")
            if str(debug.get(name, "none")) != "none"
        ]
        debug["arbiter_veto_reason"] = ",".join(vetoes) if vetoes else "none"
        return TwistCommand(linear, yaw), debug

    def _stabilize_recovery_command(
        self, command: TwistCommand, sectors: SectorMap, now: float
    ) -> tuple[TwistCommand, Dict[str, float | str | bool]]:
        debug: Dict[str, float | str | bool] = {
            "recovery_unstick": "none",
            "recovery_turn_latch": "none",
            "recovery_timeout": False,
            "recovery_block_reason": "none",
        }

        front = sectors.distance("front")
        debug["recovery_front_clear"] = front is not None and front >= self.slow_distance
        debug["recovery_exit_candidate"] = (
            "front_clear" if debug["recovery_front_clear"] else "front_still_blocked"
        )
        debug["recovery_block_reason"] = (
            "none" if debug["recovery_front_clear"] else "front_below_slow_distance"
        )
        if front is None or front >= self.slow_distance:
            self._recovery_turn_sign = 0.0
            self._recovery_turn_until = 0.0
            return command, debug

        left_score = self._turn_side_score(sectors.distance("front_left"), sectors.distance("left"))
        right_score = self._turn_side_score(sectors.distance("front_right"), sectors.distance("right"))
        debug["recovery_left_score"] = left_score
        debug["recovery_right_score"] = right_score

        yaw = command.angular_z
        if abs(yaw) > 1e-6:
            desired_sign = 1.0 if yaw > 0.0 else -1.0
            if now < self._recovery_turn_until and self._recovery_turn_sign != 0.0:
                if desired_sign != self._recovery_turn_sign:
                    yaw = abs(yaw) * self._recovery_turn_sign
                    debug["recovery_turn_latch"] = "held_previous_direction"
                else:
                    debug["recovery_turn_latch"] = "same_direction"
            else:
                self._recovery_turn_sign = desired_sign
                debug["recovery_turn_latch"] = "started"
            self._recovery_turn_until = now + 1.2
            return TwistCommand(command.linear_x, yaw), debug

        if now < self._recovery_turn_until and self._recovery_turn_sign != 0.0:
            yaw = self._recovery_turn_sign * max(0.24, self.turns.turn_speed * 0.65)
            debug["recovery_unstick"] = "latched_turn"
            debug["recovery_unstick_yaw"] = yaw
            return TwistCommand(0.0, yaw), debug

        if max(left_score, right_score) < self.turn_clearance:
            debug["recovery_unstick"] = "no_side_clearance"
            return command, debug

        turn_sign = 1.0 if left_score >= right_score else -1.0
        yaw = turn_sign * max(0.24, self.turns.turn_speed * 0.65)
        self._recovery_turn_sign = turn_sign
        self._recovery_turn_until = now + 1.2
        debug["recovery_unstick"] = "turn_toward_left" if turn_sign > 0.0 else "turn_toward_right"
        debug["recovery_unstick_yaw"] = yaw
        return TwistCommand(0.0, yaw), debug

    @staticmethod
    def _turn_side_score(front_side: Optional[float], side: Optional[float]) -> float:
        values = [value for value in (front_side, side) if value is not None]
        if not values:
            return 0.0
        return min(values)

    def _base_debug(self) -> Dict[str, float | str | bool]:
        debug: Dict[str, float | str | bool] = {
            "emergency_active": self._emergency_active,
            "emergency_trigger_reason": self._emergency_last_reason,
            "emergency_clear_counter": float(self._emergency_clear_counter),
            "emergency_trigger_count": float(self._emergency_trigger_count),
            "sign_cooldown_remaining_s": self.signs.cooldown_remaining,
        }
        debug.update(self.turns.snapshot())
        debug.update(self.signs.diagnostics)
        return debug

    @staticmethod
    def _command_source(state: str, reason: str) -> str:
        if state == "EMERGENCY_STOP":
            return "emergency_lidar_stop"
        if state in {"TURNING_LEFT", "TURNING_RIGHT", "TURNING_UTURN", "SETTLING_AFTER_TURN", "ALIGNING_AFTER_TURN"}:
            return "active_maneuver"
        if state == "QR_SCAN":
            return "qr_hold"
        if state == "SIGN_CANDIDATE":
            return "yolo_rejected_or_blocked"
        if reason.startswith("SIGN_CONFIRMED") or reason.startswith("STOP_SIGN_CONFIRMED"):
            return "yolo_sign"
        if state in {"CORRIDOR_FOLLOW", "LEFT_WALL_FOLLOW", "RIGHT_WALL_FOLLOW", "RECOVERY", "FOLLOW_GAP", "FOCM"}:
            return "navigation_module"
        if state in {"SENSOR_CHECK", "IDLE"}:
            return "safe_idle"
        return "arbiter"

    def _output(
        self,
        command: TwistCommand,
        state: str,
        reason: str,
        publish_motion: bool,
        debug: Optional[Dict[str, float | str | bool]] = None,
    ) -> ArbiterOutput:
        now = time.monotonic()
        previous_state = self._last_output_state
        if state != previous_state:
            self._state_started_at = now
        state_duration_s = max(0.0, now - self._state_started_at)
        entering_recovery = state == "RECOVERY" and previous_state != "RECOVERY"
        if entering_recovery:
            self._recovery_started_at = now
        if state != "RECOVERY":
            self._recovery_started_at = None

        merged = self._base_debug()
        active_turn_path = state in {
            "TURNING_LEFT",
            "TURNING_RIGHT",
            "TURNING_UTURN",
            "SETTLING_AFTER_TURN",
            "ALIGNING_AFTER_TURN",
        }
        merged.update(
            {
                "arbiter_previous_state": previous_state,
                "state_duration_s": state_duration_s,
                "active_turn_path": active_turn_path,
                # Active turns run after the emergency latch and before normal navigation
                # recovery/safety shaping. This is an observation, not a behavior change.
                "active_turn_bypasses_navigation_recovery": active_turn_path,
                "active_turn_standard_safety_limits_applied": not active_turn_path,
                "command_source": self._command_source(state, reason),
                "recovery_entry": entering_recovery,
                "recovery_elapsed_s": (
                    max(0.0, now - self._recovery_started_at)
                    if state == "RECOVERY" and self._recovery_started_at is not None
                    else None
                ),
            }
        )
        if debug:
            merged.update(debug)
        self._last_output_state = state
        return ArbiterOutput(command, state, reason, publish_motion, merged)
