from pathlib import Path

from scripts.compare_nav_profiles import summarize
from scripts.replay_nav_scenarios import (
    build_scenarios,
    corridor_scan,
    load_replay_profile,
    nav_kwargs,
    run_scenario,
)
from ubuntu.reactive_nav.lidar_sectors import extract_sectors
from ubuntu.reactive_nav.wall_following import NavigationObservation, create_navigation_module


def test_all_navigation_modules_compute_bounded_open_corridor_commands():
    sectors = extract_sectors(corridor_scan(front=2.0, left=0.65, right=0.65))

    for module_name in ("wall_follow", "follow_gap", "focm"):
        params = load_replay_profile(module_name)
        module = create_navigation_module(module_name, **nav_kwargs(params))
        suggestion = module.compute(NavigationObservation(sectors, now=10.0, dt=0.1))

        assert suggestion.command.linear_x >= 0.0
        assert abs(suggestion.command.angular_z) <= params["max_yaw"] + 1e-6
        assert suggestion.reason


def test_wall_follow_yaws_away_from_close_side_walls():
    left_close = extract_sectors(corridor_scan(front=2.0, left=0.18, right=0.85))
    right_close = extract_sectors(corridor_scan(front=2.0, left=0.85, right=0.18))
    module = create_navigation_module("wall_follow", **nav_kwargs(load_replay_profile("wall_follow")))

    left_suggestion = module.compute(NavigationObservation(left_close, now=20.0, dt=0.1))
    right_suggestion = module.compute(NavigationObservation(right_close, now=20.1, dt=0.1))

    assert left_suggestion.command.angular_z < 0.0
    assert right_suggestion.command.angular_z > 0.0


def test_synthetic_replay_summarizes_stale_lidar_as_safe_stop(tmp_path):
    scenario = build_scenarios(default_duration_s=1.0)["stale_lidar"]
    params = load_replay_profile("wall_follow")

    path = run_scenario(
        scenario=scenario,
        module_name="wall_follow",
        params=params,
        out_dir=Path(tmp_path),
        seed=0,
        dt_s=0.1,
        duration_s=1.0,
    )
    summary = summarize(path)

    assert summary["scenario"] == "stale_lidar"
    assert summary["status"] == "PASS"
    assert summary["stale_lidar_stop_count"] > 0
    assert summary["average_published_linear_speed_mps"] == 0.0


def test_required_failure_scenarios_are_registered():
    scenarios = build_scenarios(default_duration_s=1.0)

    for name in (
        "front_left_corner_blocked",
        "front_right_corner_blocked",
        "corner_left_approach",
        "corner_right_approach",
        "narrow_left_turn",
        "narrow_right_turn",
        "asymmetric_corridor_left_close",
        "asymmetric_corridor_right_close",
        "wall_too_close_left",
        "wall_too_close_right",
        "u_shape_dead_end",
        "spin_trap_open_space",
        "noisy_corridor_with_outliers",
        "oscillatory_corridor",
    ):
        assert name in scenarios


def test_corner_veto_replay_removes_front_left_corner_risk(tmp_path):
    scenario = build_scenarios(default_duration_s=1.0)["front_left_corner_blocked"]
    params = load_replay_profile("wall_follow")

    path = run_scenario(
        scenario=scenario,
        module_name="wall_follow",
        params=params,
        out_dir=Path(tmp_path),
        seed=0,
        dt_s=0.1,
        duration_s=1.0,
    )
    summary = summarize(path)

    assert summary["corner_risk_count"] == 0
    assert summary["status"] == "PASS"
