from pathlib import Path

from scripts.extract_turn_recovery_intervals import extract_intervals
from scripts.run_turn_recovery_ablation import ablation_spec


def _record(state, reason, *, gap_center=None):
    record = {
        "state": state,
        "reason": reason,
        "profile_name": "wall_follow_tuned",
        "nav": {"module": "wall_follow", "debug": {}},
        "lidar": {
            "sector_distance_m": {
                "front": 0.45,
                "front_left": 0.60,
                "front_right": 0.65,
                "left": 0.70,
                "right": 0.72,
            }
        },
        "command": {"published_linear_x": 0.0, "published_angular_z": 0.4},
    }
    if gap_center is not None:
        record["nav"]["debug"]["gap_center"] = gap_center
    return record


def test_extracts_turn_to_recovery_interval_and_gap_flips():
    records = [
        _record("TURNING_LEFT", "TIMED_90_DEGREE_TURN"),
        _record("RECOVERY", "FRONT_BLOCKED_SELECT_FREE_GAP", gap_center=45.0),
        _record("RECOVERY", "FRONT_BLOCKED_SELECT_FREE_GAP", gap_center=-45.0),
        _record("NAVIGATE", "FRONT_CLEAR"),
        _record("NAVIGATE", "FRONT_CLEAR"),
    ]

    intervals = extract_intervals(records, Path("synthetic.jsonl"), dt_s=0.1, min_recovery_s=1.0)

    assert len(intervals) == 1
    assert intervals[0]["contains_turn"] is True
    assert intervals[0]["recovery_entry_count"] == 1
    assert intervals[0]["gap_direction_flip_count"] == 1


def test_extracts_prolonged_recovery_without_claiming_a_turn():
    records = [_record("RECOVERY", "FRONT_BLOCKED_SELECT_FREE_GAP", gap_center=35.0) for _ in range(12)]
    records.extend([_record("NAVIGATE", "FRONT_CLEAR"), _record("NAVIGATE", "FRONT_CLEAR")])

    intervals = extract_intervals(records, Path("synthetic.jsonl"), dt_s=0.1, min_recovery_s=1.0)

    assert len(intervals) == 1
    assert intervals[0]["contains_turn"] is False
    assert intervals[0]["duration_s"] >= 1.0


def test_ablation_specs_do_not_misrepresent_unreplayable_hypotheses():
    base = {"recovery_clearance": 0.44, "front_stop_clear_distance": 0.38}

    angle_params, angle_status = ablation_spec("angle_offset_only", base)
    relaxed_params, relaxed_status = ablation_spec("recovery_exit_relaxed", base)

    assert angle_params == {}
    assert angle_status == "requires_scan_level_replay"
    assert relaxed_params["recovery_clearance"] == 0.40
    assert relaxed_status == "applied_recovery_clearance_only"
