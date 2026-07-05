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

    def update(self, signal: SignalState, now: float) -> Optional[str]:
        if now < self._cooldown_until:
            self._recent.append("none")
            return None
        if signal.stale or not signal.actionable:
            self._recent.append("none")
            return None
        direction = signal.direction.lower()
        if direction not in ("left", "right", "stop"):
            self._recent.append("none")
            return None
        if signal.confidence < self.min_confidence or signal.bbox_area_ratio < self.min_area_ratio:
            self._recent.append("none")
            return None

        self._recent.append(direction)
        if sum(1 for item in self._recent if item == direction) < self.confirm_count:
            return None

        if self.event_id(signal) in self._consumed_events:
            return None
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


class BehaviorArbiter:
    def __init__(
        self,
        *,
        front_stop_distance: float = 0.32,
        side_stop_distance: float = 0.14,
        slow_distance: float = 0.55,
        qr_hold_s: float = 0.8,
        turn_clearance: float = 0.42,
        sign_debouncer: Optional[SignDebouncer] = None,
        turn_controller: Optional[TurnController] = None,
    ):
        self.front_stop_distance = front_stop_distance
        self.side_stop_distance = side_stop_distance
        self.slow_distance = slow_distance
        self.qr_hold_s = qr_hold_s
        self.turn_clearance = turn_clearance
        self.signs = sign_debouncer or SignDebouncer()
        self.turns = turn_controller or TurnController()
        self._qr_hold_until = 0.0

    def decide(self, inputs: ArbiterInput) -> ArbiterOutput:
        sectors = inputs.sectors
        now = inputs.now

        if sectors is None:
            return ArbiterOutput(TwistCommand(), "SENSOR_CHECK", "NO_LIDAR_SECTOR_MAP", False)
        if not inputs.lidar_fresh:
            return ArbiterOutput(TwistCommand(), "EMERGENCY_STOP", "LIDAR_STALE_OR_NO_CALLBACK", True)
        if sectors.valid_count == 0:
            return ArbiterOutput(TwistCommand(), "EMERGENCY_STOP", "NO_VALID_LIDAR_POINTS", True)

        emergency = self._emergency_reason(sectors)
        if emergency:
            return ArbiterOutput(TwistCommand(), "EMERGENCY_STOP", emergency, True)

        if self.turns.active:
            step = self.turns.step(sectors, now)
            if not step.active:
                self.signs.start_cooldown(now)
            return ArbiterOutput(step.command, step.state, step.reason, True)

        if inputs.qr_recent:
            self._qr_hold_until = max(self._qr_hold_until, now + self.qr_hold_s)
        if now < self._qr_hold_until:
            return ArbiterOutput(TwistCommand(), "QR_SCAN", "QR_VISIBLE_OR_RECENTLY_LOGGED", True)

        confirmed = self.signs.update(inputs.signal, now)
        if confirmed in ("LEFT", "RIGHT"):
            turn_block = self._turn_block_reason(sectors, confirmed)
            if turn_block:
                return ArbiterOutput(TwistCommand(), "SIGN_CANDIDATE", turn_block, True)
            if self.turns.start(confirmed, now):
                self.signs.consume(inputs.signal)
                step = self.turns.step(sectors, now)
                return ArbiterOutput(step.command, step.state, f"SIGN_CONFIRMED_{confirmed}", True)
        if confirmed == "STOP":
            self.signs.consume(inputs.signal)
            self.signs.start_cooldown(now)
            return ArbiterOutput(TwistCommand(), "MANUAL_STOP", "STOP_SIGN_CONFIRMED", True)

        if inputs.nav_suggestion is None:
            return ArbiterOutput(TwistCommand(), "IDLE", "NO_NAVIGATION_SUGGESTION", True)

        command = self._apply_safety_limits(inputs.nav_suggestion.command, sectors)
        return ArbiterOutput(
            command,
            inputs.nav_suggestion.mode,
            inputs.nav_suggestion.reason,
            True,
            inputs.nav_suggestion.debug,
        )

    def _emergency_reason(self, sectors: SectorMap) -> Optional[str]:
        front_center = sectors.distance("front_center")
        front = sectors.distance("front")
        left = sectors.distance("left")
        right = sectors.distance("right")
        if front_center is not None and front_center < self.front_stop_distance:
            return f"FRONT_CENTER_TOO_CLOSE_{front_center:.2f}m"
        if front is not None and front < self.front_stop_distance:
            return f"FRONT_TOO_CLOSE_{front:.2f}m"
        if left is not None and left < self.side_stop_distance:
            return f"LEFT_SIDE_TOO_CLOSE_{left:.2f}m"
        if right is not None and right < self.side_stop_distance:
            return f"RIGHT_SIDE_TOO_CLOSE_{right:.2f}m"
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

    def _apply_safety_limits(self, command: TwistCommand, sectors: SectorMap) -> TwistCommand:
        front = sectors.distance("front")
        left = sectors.distance("left")
        right = sectors.distance("right")
        linear = command.linear_x
        yaw = command.angular_z

        if front is not None and front < self.slow_distance:
            linear = min(linear, 0.04)
        if front is not None and front < self.front_stop_distance + 0.08:
            linear = min(linear, 0.0)
        if left is not None and left < self.side_stop_distance * 1.6 and yaw > 0.0:
            yaw = min(yaw, 0.0)
        if right is not None and right < self.side_stop_distance * 1.6 and yaw < 0.0:
            yaw = max(yaw, 0.0)
        return TwistCommand(linear, yaw)
