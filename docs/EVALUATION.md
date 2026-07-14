# Evaluation

This project uses layered validation because physical robot time is expensive and real runs are hard to interpret without instrumentation.

```text
unit tests
→ deterministic synthetic LaserScan replay
→ labeled perception/FSM evaluation
→ real-log analysis and sector replay
→ robot dry-run
→ physical movement validation
```

Offline success is not described as physical success. Real robot observations are used as evidence for failure analysis and follow-up tests.

## Current evidence summary

| Layer | What was measured | Result | Evidence path |
| --- | --- | --- | --- |
| Unit tests | Deterministic logic for arbiter, sectors, turn controller, QR, frame stream, event state, and tools | Latest local validation reported `63 passed` | `tests/` |
| Signal/FSM dataset | YOLO direction detection plus robot/laptop actionability gates | 458 images; 438 signal images; direction recall 0.989; FSM accepted recall 0.950; 0 false actionable non-signal images | `output/signal_fsm_eval/final_run_fast_yolo_qr_fix/summary.md` |
| QR decoder benchmark | OpenCV versus ZXing on a live captured QR-facing dataset | ZXing recall 1.000 on 30/30 positive samples; median 1.1 ms; p95 2.1 ms; 0 false positives in that capture | `output/qr_zxing_benchmark/20260710_120051_live_qr_retry_gray3x/summary.md` |
| Synthetic navigation replay | Wall-follow profile over 29 deterministic scenarios | 24 PASS, 5 WARN, 0 FAIL; WARNs include dead-end, oscillatory, and spin-trap stress cases | `output/iter_final_wall_follow_tuned/summary.md` |
| Real-log failure analysis | Failure intervals from robot debug logs | Detected corner risk, emergency bursts, oscillation, recovery loops, spin, state flapping, and yaw saturation intervals | `output/real_log_analysis/summary.md` |
| Turn/recovery targeted captures | New robot logs for angle offset, left/right turn, and front-blocked recovery | 2,387 records; no `TURNING_LEFT`, `TURNING_RIGHT`, or `ALIGNING_AFTER_TURN` states, so captures validate recovery/emergency evidence but not the intended turn path | `output/turn_recovery_analysis/robot_captures_20260709_2240/summary_after_analysis.md` |

`output/` is ignored by git because it contains generated logs, frames, and benchmark artifacts. The table above records the local evidence paths used to produce this project summary.

## Reproduction commands

Run unit tests:

```bash
uv run python -m pytest tests/
```

Run robot-controller self-test:

```bash
uv run python -B ubuntu/reactive_nav/reactive_navigator.py --self-test
```

Run deterministic scenario replay:

```bash
python3 scripts/replay_nav_scenarios.py \
  --nav-modules wall_follow \
  --config ubuntu/reactive_nav/configs/wall_follow_tuned.yaml \
  --profile-name wall_follow_tuned \
  --scenarios all \
  --out-dir output/sim_runs

python3 scripts/compare_nav_profiles.py output/sim_runs/*.jsonl
```

Run perception/FSM evaluation:

```bash
uv run python scripts/evaluate_signal_fsm_dataset.py \
  --dataset labels-gt/dataset \
  --split all \
  --config ubuntu/reactive_nav/configs/wall_follow_less_conservative_1.yaml \
  --model models/signals/best.pt \
  --out-dir output/signal_fsm_eval/local_run
```

Extract turn/recovery intervals from robot logs:

```bash
python3 scripts/extract_turn_recovery_intervals.py \
  output/robot_runs/*/reactive_nav_debug.jsonl \
  --out-dir output/turn_recovery_analysis
```

Replay extracted intervals:

```bash
python3 scripts/replay_turn_recovery_intervals.py \
  --intervals output/turn_recovery_analysis/failure_intervals.jsonl \
  --profiles wall_follow_tuned \
  --out-dir output/turn_recovery_replay
```

## Metrics used

Navigation summaries include safety and progress metrics such as:

- emergency stop count and duration;
- corner and side risk counts;
- unsafe command vetoes;
- recovery entries, loops, and timeout counts;
- spin ratio and yaw saturation ratio;
- angular oscillation score;
- stale/invalid LiDAR safe-stop behavior;
- commanded distance estimate;
- sign turn counts and cooldown suppression.

Perception/FSM summaries include:

- direction recall;
- IoU>=0.50 recall;
- actionability recall;
- FSM accepted recall;
- false actionable non-signal image count;
- per-class accepted counts.

QR summaries include:

- recall;
- false positives;
- median and p95 latency;
- decoder variant.

## Interpretation boundaries

- Synthetic replay verifies deterministic controller choices, not physical dynamics.
- Sector-level real-log replay can expose risky command choices but cannot prove physical recovery.
- QR benchmark results from one live capture do not imply broad robustness across all lighting, distance, angle, blur, and partial-visibility cases.
- A real run showing detections in the camera overlay is insufficient unless logs show fresh accepted FSM events.
- Physical movement should only follow tests, stationary perception validation, and dry-run checks.
