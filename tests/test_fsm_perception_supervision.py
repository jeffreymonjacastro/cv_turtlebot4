from pathlib import Path

from scripts.replay_nav_scenarios import corridor_scan
from ubuntu.reactive_nav.behavior_arbiter import (
    ArbiterInput,
    BehaviorArbiter,
    SignalState,
    SignDebouncer,
)
from ubuntu.reactive_nav.lidar_sectors import extract_sectors
from ubuntu.reactive_nav.qr_logger import QRLogger
from ubuntu.reactive_nav.wall_following import NavigationSuggestion, TwistCommand


def _sectors(**kwargs):
    values = {
        "front": 1.5,
        "front_center": 1.5,
        "front_left": 1.0,
        "front_right": 1.0,
        "left": 0.8,
        "right": 0.8,
    }
    values.update(kwargs)
    return extract_sectors(corridor_scan(**values))


def _suggest(linear=0.08, yaw=0.0, mode="CORRIDOR_FOLLOW"):
    return NavigationSuggestion(TwistCommand(linear, yaw), mode, "TEST_NAV")


def _signal(direction, event_id=None, *, confidence=0.95, area=0.20, stale=False, actionable=True):
    return SignalState(
        direction=direction,
        confidence=confidence,
        bbox_area_ratio=area,
        bbox_center_x_ratio=0.5,
        actionable=actionable,
        timestamp=1.0,
        stale=stale,
        event_id=event_id or f"{direction}-event",
        reason="test",
        raw_class=direction,
    )


def _arbiter():
    return BehaviorArbiter(sign_debouncer=SignDebouncer(confirm_window=2, confirm_count=2, cooldown_s=3.0))


def _decide_twice(arbiter, signal, *, sectors=None, now=10.0, qr_recent=False, nav=None):
    sectors = sectors or _sectors()
    nav = nav or _suggest()
    arbiter.decide(ArbiterInput(sectors, True, nav, signal, qr_recent, now))
    return arbiter.decide(ArbiterInput(sectors, True, nav, signal, qr_recent, now + 0.1))


def _dry_run_publish_gate(command, *, dry_run=True, enable_motion=False):
    return command if enable_motion and not dry_run else TwistCommand()


def test_left_and_right_yolo_events_enter_turn_states_with_accepted_status():
    cases = [
        ("left", "TURNING_LEFT", 1.0),
        ("right", "TURNING_RIGHT", -1.0),
    ]
    for direction, expected_state, yaw_sign in cases:
        output = _decide_twice(_arbiter(), _signal(direction, f"{direction}-1"))

        assert output.state == expected_state
        assert output.reason == f"SIGN_CONFIRMED_{direction.upper()}"
        assert output.debug["yolo_event_status"] == "accepted"
        assert output.debug["yolo_rejection_reason"] == "none"
        assert output.debug["command_source"] == "active_maneuver"
        assert output.command.angular_z * yaw_sign > 0.0


def test_stop_yolo_event_is_supported_as_uturn():
    output = _decide_twice(_arbiter(), _signal("stop", "stop-1"))

    assert output.state == "TURNING_UTURN"
    assert output.reason == "STOP_SIGN_CONFIRMED_UTURN"
    assert output.debug["yolo_event_status"] == "accepted"
    assert output.debug["command_source"] == "active_maneuver"


def test_qr_event_holds_fsm_and_qr_logger_handles_duplicate(tmp_path):
    log_path = Path(tmp_path) / "qr.jsonl"
    logger = QRLogger(log_path, confirm_count=2)

    assert logger.observe("CHECKPOINT_1") is None
    event = logger.observe("CHECKPOINT_1", robot_state="QR_SCAN")
    duplicate = logger.observe("CHECKPOINT_1", robot_state="QR_SCAN")

    assert event is not None and event.logged
    assert duplicate is not None and duplicate.duplicate
    assert logger.confirmation_progress("CHECKPOINT_1") == "2/2"

    output = _arbiter().decide(ArbiterInput(_sectors(), True, _suggest(), SignalState(), True, 10.0))
    assert output.state == "QR_SCAN"
    assert output.debug["command_source"] == "qr_hold"


def test_repeated_sign_during_active_maneuver_does_not_restart_turn():
    arbiter = _arbiter()
    left_turn = _decide_twice(arbiter, _signal("left", "left-active"), now=10.0)
    during_turn = arbiter.decide(
        ArbiterInput(_sectors(), True, _suggest(), _signal("right", "right-during-turn"), False, 10.2)
    )

    assert left_turn.state == "TURNING_LEFT"
    assert during_turn.state == "TURNING_LEFT"
    assert during_turn.command.angular_z > 0.0
    assert during_turn.debug["command_source"] == "active_maneuver"


def test_event_during_cooldown_is_rejected_explicitly():
    arbiter = _arbiter()
    arbiter.signs.start_cooldown(10.0)

    output = arbiter.decide(
        ArbiterInput(_sectors(), True, _suggest(), _signal("left", "left-cooldown"), False, 10.1)
    )

    assert output.state == "CORRIDOR_FOLLOW"
    assert output.debug["yolo_event_status"] == "rejected"
    assert output.debug["yolo_rejection_reason"] == "cooldown_active"


def test_stale_low_confidence_unconfirmed_and_unknown_yolo_are_rejected_or_candidates():
    cases = [
        (_signal("left", "stale", stale=True), "rejected", "stale"),
        (_signal("left", "low-conf", confidence=0.1), "rejected", "low_confidence"),
        (_signal("banana", "unknown"), "rejected", "unsupported_direction:banana"),
    ]
    for signal, status, reason in cases:
        output = _arbiter().decide(ArbiterInput(_sectors(), True, _suggest(), signal, False, 10.0))
        assert output.state == "CORRIDOR_FOLLOW"
        assert output.debug["yolo_event_status"] == status
        assert output.debug["yolo_rejection_reason"] == reason

    first = _arbiter().decide(ArbiterInput(_sectors(), True, _suggest(), _signal("left", "one-frame"), False, 10.0))
    assert first.debug["yolo_event_status"] == "candidate"
    assert first.debug["yolo_rejection_reason"] == "awaiting_confirmation"


def test_emergency_safety_has_priority_over_yolo_event():
    output = _decide_twice(
        _arbiter(),
        _signal("left", "left-emergency"),
        sectors=_sectors(front=0.20, front_center=0.20),
    )

    assert output.state == "EMERGENCY_STOP"
    assert output.command == TwistCommand()
    assert output.debug["command_source"] == "emergency_lidar_stop"


def test_active_maneuver_has_priority_over_qr_and_qr_has_priority_over_sign_and_navigation():
    arbiter = _arbiter()
    assert arbiter.turns.start("RIGHT", now=10.0)
    active = arbiter.decide(ArbiterInput(_sectors(), True, _suggest(), _signal("left", "left"), True, 10.1))
    assert active.state == "TURNING_RIGHT"
    assert active.debug["command_source"] == "active_maneuver"

    qr_first = _arbiter().decide(ArbiterInput(_sectors(), True, _suggest(), _signal("left", "left-qr"), True, 10.0))
    assert qr_first.state == "QR_SCAN"
    assert qr_first.debug["command_source"] == "qr_hold"


def test_dry_run_intended_command_differs_from_safe_published_command():
    output = _decide_twice(_arbiter(), _signal("left", "dry-run-left"))
    published = _dry_run_publish_gate(output.command, dry_run=True, enable_motion=False)

    assert output.command.angular_z != 0.0
    assert published == TwistCommand()
