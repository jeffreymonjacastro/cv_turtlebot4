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
