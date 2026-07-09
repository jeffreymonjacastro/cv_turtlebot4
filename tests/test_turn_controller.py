from scripts.replay_nav_scenarios import corridor_scan
from ubuntu.reactive_nav.lidar_sectors import extract_sectors
from ubuntu.reactive_nav.turn_controller import TurnController


def test_turn_controller_runs_left_turn_then_alignment():
    sectors = extract_sectors(corridor_scan(front=2.0, left=0.8, right=0.8))
    controller = TurnController(
        turn_speed=1.0,
        turn_degrees=20.0,
        settle_seconds=0.05,
        align_max_seconds=0.5,
        align_error_threshold=0.03,
        align_stable_cycles=2,
    )

    assert controller.start("LEFT", now=100.0)
    turning = controller.step(sectors, now=100.01)
    assert turning.state == "TURNING_LEFT"
    assert turning.command.angular_z > 0.0

    settling = controller.step(sectors, now=100.0 + controller.turn_seconds + 0.01)
    assert settling.state == "SETTLING_AFTER_TURN"

    aligning = controller.step(sectors, now=100.0 + controller.turn_seconds + 0.08)
    assert aligning.state in {"ALIGNING_AFTER_TURN", "NAVIGATE"}


def test_turn_controller_uturn_uses_180_degree_timing():
    sectors = extract_sectors(corridor_scan(front=2.0, left=0.8, right=0.8))
    controller = TurnController(
        turn_speed=1.0,
        turn_degrees=90.0,
        settle_seconds=0.05,
        align_max_seconds=0.5,
    )

    assert controller.start("UTURN", now=100.0)
    turning = controller.step(sectors, now=100.0 + controller.turn_seconds + 0.01)
    assert turning.state == "TURNING_UTURN"
    assert turning.command.angular_z > 0.0

    settling = controller.step(sectors, now=100.0 + controller.turn_seconds * 2.0 + 0.01)
    assert settling.state == "SETTLING_AFTER_TURN"


def test_turn_controller_times_out_alignment_when_sides_are_missing():
    sectors = extract_sectors(corridor_scan(front=2.0, left=0.8, right=0.8))
    no_side_sectors = extract_sectors(corridor_scan(front=2.0, left=0.01, right=0.01))
    controller = TurnController(
        turn_speed=1.0,
        turn_degrees=5.0,
        settle_seconds=0.0,
        align_max_seconds=0.05,
        align_stable_cycles=3,
    )

    assert controller.start("RIGHT", now=200.0)
    controller.step(sectors, now=200.0 + controller.turn_seconds + 0.01)
    controller.step(no_side_sectors, now=200.0 + controller.turn_seconds + 0.20)
    timeout = controller.step(no_side_sectors, now=200.0 + controller.turn_seconds + 0.30)

    assert timeout.active is False
    assert timeout.reason == "ALIGNMENT_TIMEOUT"
