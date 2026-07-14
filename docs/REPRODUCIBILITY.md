# Reproducibility

This guide describes how to reproduce the non-hardware parts of the project and how to prepare for robot-side validation without committing lab-specific secrets.

## Local configuration

Copy the template:

```bash
cp .env.example .env
```

Fill in local values:

```bash
ROBOT_IP=<robot-ip>
ROBOT_SSH_TARGET=<ssh-user>@<robot-host-or-ip>
ROS_DOMAIN_ID=<ros-domain-id>
PAIRING_CODE=<pairing-code>
ROBOT_NAME=<robot-name>
```

Do not commit `.env`, Wi-Fi passwords, SSH keys, robot passwords, private hostnames, or local-only notes.

## Laptop setup

Recommended:

```bash
uv sync --locked
uv run python -m pytest tests/
```

Fallback:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pytest tests/
```

The laptop-side perception stack uses:

- OpenCV;
- ultralytics YOLO;
- zxing-cpp;
- NumPy.

## Robot setup assumptions

Robot-side scripts assume:

- TurtleBot 4 Lite;
- Ubuntu 24.04 / ROS 2 Jazzy;
- TurtleBot 4 bringup packages;
- LiDAR topic such as `/scan`;
- camera topic such as `/oakd/rgb/preview/image_raw`;
- a velocity command topic compatible with the selected `cmd_msg_type`.

Before running autonomy, verify:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /oakd/rgb/preview/image_raw
ros2 topic echo /cmd_vel
```

Topic visibility alone is not enough; callbacks must be fresh in the controller logs.

## Offline checks

```bash
uv run python -m pytest tests/
uv run python -B ubuntu/reactive_nav/reactive_navigator.py --self-test
```

Run synthetic replay:

```bash
python3 scripts/replay_nav_scenarios.py \
  --nav-modules wall_follow \
  --config ubuntu/reactive_nav/configs/wall_follow_tuned.yaml \
  --profile-name wall_follow_tuned \
  --scenarios all \
  --out-dir output/sim_runs

python3 scripts/compare_nav_profiles.py output/sim_runs/*.jsonl
```

Run perception/FSM evaluation if the model and dataset are available:

```bash
uv run python scripts/evaluate_signal_fsm_dataset.py \
  --dataset labels-gt/dataset \
  --split all \
  --config ubuntu/reactive_nav/configs/wall_follow_less_conservative_1.yaml \
  --model models/signals/best.pt \
  --out-dir output/signal_fsm_eval/local_run
```

## Robot dry-run workflow

1. Sync robot-side code with your local SSH target.
2. Start TurtleBot bringup.
3. Start the camera UDP sender.
4. Start laptop perception.
5. Start event sync.
6. Run `reactive_navigator.py` with:

```text
dry_run=true
enable_motion=false
publish_zero_in_dry_run=true
```

7. Inspect `reactive_nav_debug.jsonl` for:

- fresh LiDAR callbacks;
- fresh image callbacks;
- accepted or rejected YOLO events with clear reasons;
- QR semantic event receipt and one-time consumption;
- requested command versus published zero command;
- emergency and stale-sensor behavior.

Only move to physical motion after stationary/dry-run logs look sane.

## Generated artifacts

Generated evidence is written under `output/`, which is ignored by git:

```text
output/sim_runs/
output/signal_fsm_eval/
output/qr_zxing_benchmark/
output/turn_recovery_analysis/
output/turn_recovery_replay/
output/autonomous_runs/
```

If sharing results publicly, either copy concise summaries into `docs/` or provide a separate artifact bundle. Do not publish raw logs that reveal private network or lab identifiers without review.
