from scripts.replay_nav_scenarios import corridor_scan
from ubuntu.reactive_nav.behavior_arbiter import (
    ArbiterInput,
    BehaviorArbiter,
    SignalState,
    SignDebouncer,
)
from ubuntu.reactive_nav.lidar_sectors import extract_sectors
from ubuntu.reactive_nav.wall_following import NavigationSuggestion, TwistCommand


def _suggest(linear=0.10, yaw=0.0):
    return NavigationSuggestion(TwistCommand(linear, yaw), "CORRIDOR_FOLLOW", "TEST")


def _signal(direction="left", event_id="event-1"):
    return SignalState(
        direction=direction,
        confidence=0.95,
        bbox_area_ratio=0.10,
        actionable=True,
        timestamp=1.0,
        stale=False,
        event_id=event_id,
        reason="test",
    )


def test_stale_lidar_forces_zero_emergency_stop():
    sectors = extract_sectors(corridor_scan(front=2.0, left=0.7, right=0.7))
    arbiter = BehaviorArbiter()

    output = arbiter.decide(
        ArbiterInput(
            sectors=sectors,
            lidar_fresh=False,
            nav_suggestion=_suggest(),
            signal=SignalState(),
            qr_recent=False,
            now=10.0,
        )
    )

    assert output.state == "EMERGENCY_STOP"
    assert output.reason == "LIDAR_STALE_OR_NO_CALLBACK"
    assert output.command == TwistCommand()


def test_front_blocked_emergency_dominates_navigation_suggestion():
    sectors = extract_sectors(corridor_scan(front=0.22, front_center=0.22, left=0.8, right=0.8))
    arbiter = BehaviorArbiter(front_stop_distance=0.28)

    output = arbiter.decide(
        ArbiterInput(
            sectors=sectors,
            lidar_fresh=True,
            nav_suggestion=_suggest(linear=0.10, yaw=0.3),
            signal=SignalState(),
            qr_recent=False,
            now=10.0,
        )
    )

    assert output.state == "EMERGENCY_STOP"
    assert output.command.linear_x == 0.0
    assert output.command.angular_z == 0.0


def test_front_sector_outlier_does_not_stop_when_front_center_is_clear():
    sectors = extract_sectors(corridor_scan(front=0.23, front_center=1.2, left=0.45, right=0.45))
    arbiter = BehaviorArbiter(front_stop_distance=0.28)

    output = arbiter.decide(
        ArbiterInput(
            sectors=sectors,
            lidar_fresh=True,
            nav_suggestion=_suggest(linear=0.07, yaw=0.0),
            signal=SignalState(),
            qr_recent=False,
            now=10.0,
        )
    )

    assert output.state == "CORRIDOR_FOLLOW"
    assert output.command.linear_x > 0.0


def test_recovery_zero_command_turns_toward_open_side_when_front_blocked():
    sectors = extract_sectors(
        corridor_scan(front=0.34, front_center=0.34, front_left=0.80, front_right=0.30, left=0.85, right=0.35)
    )
    arbiter = BehaviorArbiter(front_stop_distance=0.28, slow_distance=0.48, turn_clearance=0.40)

    output = arbiter.decide(
        ArbiterInput(
            sectors=sectors,
            lidar_fresh=True,
            nav_suggestion=NavigationSuggestion(TwistCommand(0.0, 0.0), "RECOVERY", "TEST_RECOVERY_ZERO"),
            signal=SignalState(),
            qr_recent=False,
            now=10.0,
        )
    )

    assert output.state == "RECOVERY"
    assert output.command.linear_x == 0.0
    assert output.command.angular_z > 0.0
    assert output.debug["recovery_unstick"] == "turn_toward_left" or output.debug["corner_opening_turn"] == "left"


def test_side_safety_veto_prevents_yaw_into_close_wall():
    sectors = extract_sectors(corridor_scan(front=1.5, left=0.18, right=0.8))
    arbiter = BehaviorArbiter(side_stop_distance=0.12)

    output = arbiter.decide(
        ArbiterInput(
            sectors=sectors,
            lidar_fresh=True,
            nav_suggestion=_suggest(linear=0.08, yaw=0.4),
            signal=SignalState(),
            qr_recent=False,
            now=10.0,
        )
    )

    assert output.state == "CORRIDOR_FOLLOW"
    assert output.command.angular_z <= 0.0


def test_front_corner_safety_veto_prevents_yaw_into_corner():
    sectors = extract_sectors(corridor_scan(front=1.4, front_left=0.40, front_right=1.2, left=1.0, right=0.35))
    arbiter = BehaviorArbiter(front_corner_avoid_distance=0.62, corner_slow_speed=0.03)

    output = arbiter.decide(
        ArbiterInput(
            sectors=sectors,
            lidar_fresh=True,
            nav_suggestion=_suggest(linear=0.09, yaw=0.45),
            signal=SignalState(),
            qr_recent=False,
            now=10.0,
        )
    )

    assert output.state == "CORRIDOR_FOLLOW"
    assert output.command.angular_z <= 0.0
    assert output.command.linear_x <= 0.03
    assert output.debug["corner_yaw_veto"] == "front_left"


def test_corner_opening_turn_forces_yaw_toward_open_side_on_sharp_curve():
    sectors = extract_sectors(
        corridor_scan(front=0.44, front_left=0.34, front_right=0.90, left=0.75, right=0.75)
    )
    arbiter = BehaviorArbiter(
        slow_distance=0.48,
        front_corner_avoid_distance=0.56,
        corner_slow_speed=0.055,
    )

    output = arbiter.decide(
        ArbiterInput(
            sectors=sectors,
            lidar_fresh=True,
            nav_suggestion=_suggest(linear=0.08, yaw=0.0),
            signal=SignalState(),
            qr_recent=False,
            now=10.0,
        )
    )

    assert output.state == "CORRIDOR_FOLLOW"
    assert output.command.angular_z < -0.20
    assert output.debug["corner_opening_turn"] == "right"


def test_front_corner_veto_can_be_disabled_for_ablation():
    sectors = extract_sectors(corridor_scan(front=1.4, front_left=0.40, front_right=1.2, left=1.0, right=0.35))
    arbiter = BehaviorArbiter(
        front_corner_avoid_distance=0.62,
        corner_slow_speed=0.03,
        enable_corner_yaw_veto=False,
        enable_corner_slowdown=False,
    )

    output = arbiter.decide(
        ArbiterInput(
            sectors=sectors,
            lidar_fresh=True,
            nav_suggestion=_suggest(linear=0.09, yaw=0.45),
            signal=SignalState(),
            qr_recent=False,
            now=10.0,
        )
    )

    assert output.command.angular_z == 0.45
    assert output.command.linear_x == 0.09
    assert output.debug["corner_yaw_veto"] == "none"
    assert output.debug["corner_slowdown"] is False


def test_angular_smoothing_does_not_reintroduce_vetoed_corner_yaw():
    open_sectors = extract_sectors(corridor_scan(front=1.4, front_left=1.2, front_right=1.2, left=1.0, right=1.0))
    corner_sectors = extract_sectors(corridor_scan(front=1.4, front_left=0.40, front_right=1.2, left=1.0, right=1.0))
    arbiter = BehaviorArbiter(
        front_corner_avoid_distance=0.62,
        angular_smoothing_alpha=0.65,
    )

    arbiter.decide(
        ArbiterInput(open_sectors, True, _suggest(linear=0.09, yaw=0.45), SignalState(), False, 10.0)
    )
    output = arbiter.decide(
        ArbiterInput(corner_sectors, True, _suggest(linear=0.09, yaw=0.45), SignalState(), False, 10.1)
    )

    assert output.debug["corner_yaw_veto"] == "front_left"
    assert output.debug["angular_smoothing_veto_clamped"] is True
    assert output.command.angular_z <= 0.0


def test_anti_spin_limiter_requires_repeated_spin_candidate_cycles():
    sectors = extract_sectors(corridor_scan(front=2.0, left=1.0, right=1.0))
    arbiter = BehaviorArbiter(
        enable_anti_spin=True,
        anti_spin_yaw_threshold=0.4,
        anti_spin_linear_threshold=0.03,
        anti_spin_trigger_cycles=2,
        anti_spin_recovery_speed=0.04,
    )

    first = arbiter.decide(
        ArbiterInput(sectors, True, _suggest(linear=0.0, yaw=0.5), SignalState(), False, 10.0)
    )
    second = arbiter.decide(
        ArbiterInput(sectors, True, _suggest(linear=0.0, yaw=0.5), SignalState(), False, 10.1)
    )

    assert first.debug["anti_spin_limited"] is False
    assert second.debug["anti_spin_limited"] is True
    assert second.command.linear_x >= 0.04
    assert abs(second.command.angular_z) < 0.5


def test_sign_debouncer_confirms_once_and_suppresses_cooldown():
    debouncer = SignDebouncer(confirm_window=3, confirm_count=2, cooldown_s=5.0)
    signal = _signal("left", "same-left")

    assert debouncer.update(signal, 10.0) is None
    assert debouncer.update(signal, 10.1) == "LEFT"
    debouncer.consume(signal)
    debouncer.start_cooldown(10.1)
    assert debouncer.update(signal, 11.0) is None


def test_blocked_sign_stays_in_candidate_without_turning():
    sectors = extract_sectors(corridor_scan(front=1.2, left=0.7, right=0.7, front_left=0.30))
    arbiter = BehaviorArbiter(
        turn_clearance=0.40,
        sign_debouncer=SignDebouncer(confirm_window=2, confirm_count=2),
    )

    first = arbiter.decide(
        ArbiterInput(sectors, True, _suggest(), _signal("left", "blocked-left"), False, 1.0)
    )
    second = arbiter.decide(
        ArbiterInput(sectors, True, _suggest(), _signal("left", "blocked-left"), False, 1.1)
    )

    assert first.state == "CORRIDOR_FOLLOW"
    assert second.state == "SIGN_CANDIDATE"
    assert "TURN_LEFT_BLOCKED" in second.reason
    assert second.command == TwistCommand()


def test_lower_turn_clearance_allows_tight_but_safe_turn_candidate():
    sectors = extract_sectors(corridor_scan(front=1.2, left=0.7, right=0.7, front_left=0.36))
    arbiter = BehaviorArbiter(
        turn_clearance=0.35,
        side_stop_distance=0.10,
        front_stop_distance=0.24,
        sign_debouncer=SignDebouncer(confirm_window=2, confirm_count=2),
    )

    arbiter.decide(ArbiterInput(sectors, True, _suggest(), _signal("left", "tight-left"), False, 1.0))
    second = arbiter.decide(
        ArbiterInput(sectors, True, _suggest(), _signal("left", "tight-left"), False, 1.1)
    )

    assert second.state == "TURNING_LEFT"
    assert second.command.angular_z > 0.0


def test_stop_sign_triggers_uturn_instead_of_manual_stop():
    sectors = extract_sectors(corridor_scan(front=1.2, left=0.7, right=0.7))
    arbiter = BehaviorArbiter(sign_debouncer=SignDebouncer(confirm_window=2, confirm_count=2))

    first = arbiter.decide(
        ArbiterInput(sectors, True, _suggest(), _signal("stop", "stop-1"), False, 1.0)
    )
    second = arbiter.decide(
        ArbiterInput(sectors, True, _suggest(), _signal("stop", "stop-1"), False, 1.1)
    )

    assert first.state == "CORRIDOR_FOLLOW"
    assert second.state == "TURNING_UTURN"
    assert second.reason == "STOP_SIGN_CONFIRMED_UTURN"
    assert second.command.linear_x == 0.0
    assert second.command.angular_z > 0.0


def test_active_turn_is_instrumented_and_does_not_use_navigation_recovery():
    sectors = extract_sectors(corridor_scan(front=1.2, left=0.7, right=0.7))
    arbiter = BehaviorArbiter()
    assert arbiter.turns.start("LEFT", now=10.0)

    output = arbiter.decide(
        ArbiterInput(
            sectors,
            True,
            NavigationSuggestion(TwistCommand(0.0, -0.5), "RECOVERY", "TEST_RECOVERY"),
            SignalState(),
            False,
            10.1,
        )
    )

    assert output.state == "TURNING_LEFT"
    assert output.command.angular_z > 0.0
    assert output.debug["active_turn_path"] is True
    assert output.debug["active_turn_bypasses_navigation_recovery"] is True
    assert output.debug["active_turn_standard_safety_limits_applied"] is False


def test_emergency_stop_interrupts_an_active_turn():
    safe = extract_sectors(corridor_scan(front=1.2, left=0.7, right=0.7))
    blocked = extract_sectors(corridor_scan(front=0.20, front_center=0.20, left=0.7, right=0.7))
    arbiter = BehaviorArbiter(front_stop_distance=0.28)
    assert arbiter.turns.start("RIGHT", now=10.0)

    turning = arbiter.decide(ArbiterInput(safe, True, _suggest(), SignalState(), False, 10.1))
    emergency = arbiter.decide(ArbiterInput(blocked, True, _suggest(), SignalState(), False, 10.2))

    assert turning.state == "TURNING_RIGHT"
    assert emergency.state == "EMERGENCY_STOP"
    assert emergency.command == TwistCommand()
