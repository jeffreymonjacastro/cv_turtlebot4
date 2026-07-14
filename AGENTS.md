# AGENTS.md — TurtleBot4 CV Navigation Implementation Context

This file is persistent context for coding agents working in this repo.

## Mission

Build a robust autonomous navigation stack for a TurtleBot 4 Lite competition/project where the robot must traverse an unknown indoor circuit, follow visual traffic signals, and scan/log QR checkpoints.

The current strategic architecture is:

```text
LiDAR reactive navigation as the default behavior
+ YOLO traffic-sign event overrides
+ QR checkpoint detection/logging
+ safety-first arbitration
```

Do **not** try to revive full Nav2/SLAM unless explicitly asked. The immediate goal is a reliable, lightweight implementation that can run on the TurtleBot/Raspberry Pi side and/or cooperate with the existing laptop-side YOLO receiver.

The immediate engineering goal is **fast, systematic algorithm iteration**:

```text
offline tests -> synthetic LaserScan replay -> recorded /scan replay -> Gazebo/sim -> physical robot
```

Do not use the physical robot as the first debugging tool for navigation behavior.

---

## Existing repo context

The repo is split by execution side:

```text
ubuntu/   robot-side ROS 2 nodes/scripts
win/      laptop-side Windows helpers/receivers/controllers
kaggle/   YOLO training artifacts
docs/     repo documentation/runbooks/benchmarking notes
scripts/  local/offline evaluation and utility scripts
tests/    unit and offline regression tests
```

Preserve this split.

Important existing components:

```text
ubuntu/original/enviador.py        robot telemetry sender
ubuntu/original/recibidor.py       robot UDP command receiver publishing /cmd_vel
ubuntu/detect_qr/enviador.py       robot telemetry sender with QR support
ubuntu/lidar/                      current navigation experiments
ubuntu/reactive_nav/               stabilized reactive navigation stack, if present
win/original/controller_template.py WASD sender
win/detect_qr/recibidor.py         QR/telemetry receiver only
win/lidar/recibidor.py             LiDAR diagnostics receiver
win/yolo/recibidor.py              YOLO image receiver/detector, writes latest signal state
win/yolo/enviador.py               reads latest signal state and sends robot commands
```

Known constraint from current project history:

- There is a follow-the-gap implementation, but it is not reliable. Do **not** assume it works. It may be inspected only for interfaces, telemetry packet format, or lessons learned.
- There is a working YOLO detection script that performs decently. Preserve and integrate it instead of replacing it without evidence.
- QR receiving and WASD/movement control must stay separate unless the user explicitly asks otherwise.
- Navigation algorithms must be compared by reproducible logs/metrics, not by subjective impressions from one physical run.

---

## Local robot access

Robot access credentials must not be committed.

If `.codex/robot_access.local.md` exists, read it for local-only SSH/IP instructions. Use the SSH alias `turtlebot4` when available.

Never copy robot passwords, Wi-Fi credentials, private keys, or lab network secrets into tracked files.

---

## Critical implementation principle

Do **not** assume hardware, ROS topics, callbacks, UDP, YOLO, LiDAR, camera, or `/cmd_vel` work.

Before implementing or integrating behavior on the robot, test and report:

1. Is ROS running?
2. Which LaserScan topic is live?
3. Are LaserScan callbacks actually firing?
4. Is `/cmd_vel` accepted by the robot?
5. Is the camera topic live?
6. Does the laptop receive telemetry/logs?
7. Does YOLO write a fresh latest-signal state?
8. Does QR decoding/logging work?
9. Are stale data and missing sensor states handled safely?

Topic visibility is not enough. A visible topic with no publisher, stale messages, or no callback activity is a failure.

For algorithm development, prefer:

```text
unit tests and offline replay first
dry-run logs second
simulated movement third
physical movement last
```

---

## Non-negotiable safety hierarchy

The arbiter must use this priority order:

```text
1. Emergency LiDAR stop / collision prevention
2. Active maneuver completion, unless emergency stop is needed
3. QR scan/checkpoint registration behavior
4. Confirmed traffic-sign command
5. Default LiDAR navigation
6. Idle/stop if required sensors are missing or stale
```

YOLO must never directly drive wheels. YOLO produces symbolic events such as:

```text
LEFT
RIGHT
STOP
QR_VISIBLE
NONE
```

The arbiter decides whether and when these events become movement commands.

Every offline benchmark must preserve this hierarchy. A benchmark that bypasses the arbiter is not representative.

---

## Modularity requirement for navigation

Do not implement the default navigation algorithm as a monolithic controller. The project must allow swapping the local navigation algorithm without changing perception, QR logging, sign handling, safety arbitration, or command publishing.

Use a navigation module interface plus a factory. The main controller should call the selected navigation module and then pass its suggested command through the safety/arbiter layer.

The navigation module owns only:

```text
processed LiDAR/navigation observation -> suggested command + debug fields
```

The navigation module must not:

```text
publish /cmd_vel directly
read YOLO detections directly
read QR detections directly
perform UDP telemetry directly
override emergency safety
know whether the input is real, synthetic, bag replay, or simulation
```

Existing Follow-the-Gap files in `ubuntu/lidar/` may be inspected for topic names, parameters, and prior diagnostics, but the broken implementation should not be treated as the foundation for the new architecture.

---

## Recommended target files

Prefer adding a new clean implementation rather than patching unstable legacy files unless necessary.

Suggested robot-side files:

```text
ubuntu/reactive_nav/
  __init__.py
  reactive_navigator.py          main ROS 2 node / entrypoint
  lidar_sectors.py               LaserScan preprocessing and sector min distances
  wall_following.py              PD/PID wall/corridor following
  gap_navigation.py              Follow-the-Gap style recovery/module
  turn_controller.py             90-degree turn and LiDAR alignment logic
  behavior_arbiter.py            state machine and priority rules
  qr_logger.py                   persistent QR evidence logging
  diagnostics.py                 UDP logs/state packets
```

Suggested local/offline files:

```text
tests/
  test_lidar_sectors.py
  test_behavior_arbiter.py
  test_turn_controller.py
  test_qr_logger.py
  test_nav_modules_synthetic.py

scripts/
  replay_nav_scenarios.py        deterministic synthetic LaserScan scenario replay
  compare_nav_profiles.py        compare JSONL logs across profiles/modules/scenarios
  evaluate_nav_profiles.py       extend if already present; do not duplicate unnecessarily
  replay_scan_log.py             optional replay from converted real LaserScan logs
  bag_to_nav_replay.py           optional later, if ROS bag parsing is available

docs/
  NAV_BENCHMARKING.md
  NAV_SCENARIO_REPLAY.md
  NAV_METRICS_AND_SCORE.md
```

Suggested laptop-side integration files, only if needed:

```text
win/yolo/latest_signal_schema.md  document the JSON/state file schema
win/yolo/signal_state_reader.py   reusable reader if needed
```

If the current repo structure makes a package folder difficult, create a single new file first:

```text
ubuntu/lidar/reactive_yolo_lidar_nav.py
```

But keep code modular internally with functions/classes.

---

## Minimum viable behavior

The first real milestone is not a perfect maze solver. It is:

```text
1. Verify LiDAR topic and callback.
2. Verify safe low-speed /cmd_vel.
3. Implement sector map.
4. Implement emergency stop.
5. Implement slow corridor following.
6. Read YOLO latest-signal state.
7. Confirm sign with debounce.
8. Execute safe 90-degree left/right turn.
9. Align using LiDAR after turn.
10. Detect/log QR persistently.
11. Produce logs that can be evaluated offline.
12. Compare navigation modules with the same scenarios.
```

---

## LiDAR sector model

Use smaller sectors in front and wider sectors on the sides.

Suggested sectors:

```text
front_center: -10° to +10°
front:        -20° to +20°
front_left:   +20° to +70°
front_right:  -70° to -20°
left:         +70° to +110°
right:        -110° to -70°
rear:         ±160° to 180°
```

Always filter invalid LaserScan values:

```text
ignore NaN
ignore inf unless treated as max_range
ignore values < range_min
clip values > range_max
use robust min/percentile instead of raw min if noisy
```

Offline tests must include NaN, inf, out-of-range, empty, and stale-scan cases.

---

## Default navigation

Use LiDAR-based reactive navigation, not full maze solving.

Recommended default:

```text
corridor/wall following using PD control
+ largest-free-sector recovery when front is blocked
```

Example controller shape:

```text
error = distance_left - distance_right       # corridor centering
angular_z = -Kp * error - Kd * d_error
linear_x = base_speed if front is clear else 0
```

Tune signs carefully on the real robot. If it turns toward the closer wall, the sign is wrong.

Do not tune only on one physical run. Tune against a scenario suite and compare logs.

---

## Visual sign handling

Traffic signs must be debounced:

```text
confidence >= SIGN_CONF_THRESHOLD
bbox_area_ratio >= SIGN_AREA_THRESHOLD
same class detected in at least N of last M frames
not currently in cooldown
not currently in active maneuver
```

After a sign triggers a command:

```text
1. latch command
2. slow/stop
3. execute maneuver
4. align to corridor/walls using LiDAR
5. start cooldown
6. return to default navigation
```

Do not repeatedly trigger from the same sign. Use a cooldown based on time and/or distance moved.

Offline benchmarks must include fresh, stale, repeated, blocked, and cooldown sign cases.

---

## QR handling

QR codes are project evidence. Treat QR detection as a behavior, not just a console print.

When QR is visible or decoded:

```text
1. slow or stop
2. decode for multiple frames if needed
3. confirm stable content
4. persistently append to a log file
5. include timestamp and optional frame/context
6. ignore already-seen QR contents
7. resume navigation
```

Suggested log path:

```text
output/qr_log.jsonl
```

Suggested record:

```json
{
  "timestamp": "...",
  "qr_content": "...",
  "source": "camera",
  "frame_id": "...",
  "robot_state": "...",
  "confidence": null
}
```

Offline benchmarks should validate that repeated QR content is not logged multiple times.

---

## Diagnostics requirements

Every new navigation node must expose useful diagnostics.

At minimum, log or send through existing UDP `LOG` / `LIDAR` paths:

```text
active state
front / left / right / rear distances
chosen speed and yaw
detected sign state
QR seen/decoded state
sensor freshness
reason for stop
reason for turn
reason for recovery
navigation module name
profile name
dry_run / enable_motion
suggested command
arbiter-requested command
published command
emergency trigger reason
```

Do not hide failures. If a topic is missing or stale, report it clearly and stop safely.

---

## Benchmarking and evaluation requirement

Navigation work is incomplete unless it can be evaluated systematically.

Read:

```text
docs/NAV_BENCHMARKING.md
docs/NAV_SCENARIO_REPLAY.md
docs/NAV_METRICS_AND_SCORE.md
```

Expected test levels:

```text
0. Pure unit tests
1. Synthetic LaserScan scenario replay
2. Recorded real /scan replay or converted scan logs
3. Gazebo/TurtleBot simulation
4. Physical robot validation
```

The first implemented benchmark target is synthetic replay. It must run without the physical robot and preferably without ROS installed.

A valid benchmark loop should support:

```bash
python3 -m pytest tests/

python3 scripts/replay_nav_scenarios.py   --nav-modules wall_follow follow_gap focm   --scenarios all   --out-dir output/sim_runs

python3 scripts/compare_nav_profiles.py output/sim_runs/*.jsonl
```

Synthetic benchmarks must be deterministic unless explicitly run with a seed argument. Always write the seed into the output log metadata.

Do not claim real-world performance from synthetic tests. Label results as offline/simulated validation.

---

## Minimum synthetic scenarios

At minimum, implement deterministic scenarios for:

```text
open_corridor
narrow_corridor
left_wall_close
right_wall_close
front_blocked
dead_end_recovery
left_sign_open
right_sign_open
left_sign_blocked
right_sign_blocked
stale_lidar
all_invalid_lidar
noisy_lidar_nan_inf
qr_visible
repeated_sign_cooldown
```

Each scenario should have explicit acceptance criteria. Examples:

```text
open_corridor:
  no emergency stop
  positive average linear speed
  low oscillation score

front_blocked:
  zero or near-zero forward command while blocked
  emergency or recovery reason logged

stale_lidar:
  published command remains zero
  stop reason clearly identifies stale LiDAR

left_sign_open:
  exactly one left maneuver is triggered
  repeated sign frames do not retrigger during cooldown

all_invalid_lidar:
  stop safely
  no non-zero published linear command
```

---

## Metrics for comparing profiles

At minimum, profile comparison should report:

```text
scenario
profile_name
nav_module
total_runtime_s
time_per_state
emergency_stop_count
emergency_stop_total_time_s
recovery_time_ratio
average_published_linear_speed_mps
mean_abs_angular_speed_radps
angular_sign_changes_per_min
turn_count
turn_timeout_count
alignment_timeout_count
stale_lidar_stop_count
minimum_front_distance_m
minimum_front_center_distance_m
minimum_side_distance_m
commanded_distance_estimate_m
collision_event_count
unsafe_command_veto_count
scenario_score
```

Use a score only as a ranking heuristic. The raw metrics and logs remain the source of truth.

---

## Work protocol for coding agents

Before code edits:

```bash
git status --short --branch
git diff --stat
```

Do not revert user changes casually.

When touching navigation:

1. Inspect current relevant files.
2. Identify actual ROS topics and message types when working on robot integration.
3. Preserve the navigation module boundary.
4. Add or update tests before/with behavior changes.
5. Add small testable changes.
6. Run static checks and unit tests.
7. Run synthetic replay if navigation behavior changed.
8. Report exact commands run, what passed, what failed, and what still requires the physical robot.

Do not claim behavior works from `py_compile` alone.
Do not claim real robot behavior from offline replay alone.
Do not publish motion commands in offline benchmark scripts.

---

## Validation expectation

A patch is not complete unless it includes:

```text
exact files changed
exact commands run
test results
expected runtime command(s)
fallback behavior when sensors fail
clear acceptance criteria
diagnostic logs visible to the user
benchmark/evaluation output when navigation behavior changed
```

For navigation algorithm changes, also include:

```text
which scenarios were run
which profiles/modules were compared
best profile by metrics
failure cases still observed
whether it is safe to try dry-run or physical robot testing next
```

---

## References to keep in repo docs

Recommended links:

- TurtleBot 4 hardware features:
  https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html
- TurtleBot 4 simulator:
  https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html
- F1TENTH Wall Following Lab:
  https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab3.html
- F1TENTH Follow the Gap Lab:
  https://f1tenth-coursekit.readthedocs.io/en/stable/assignments/labs/lab4.html
- Vector Field Histogram paper:
  https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/borenstein_VFHisto.pdf
# AGENTS.md Addendum — Offline Benchmark-Driven Navigation Iteration

Add this section to `AGENTS.md` after the validation/diagnostics sections.

## Offline benchmark-driven improvement

The project must support improving navigation without repeatedly using the physical TurtleBot and without relying on Gazebo.

The primary improvement loop is:

```text
benchmark -> identify problem -> tweak -> benchmark -> decide keep/reject/change direction
```

Navigation changes should be evaluated through deterministic offline tests before any robot run.

### Offline realism priority

Because ROS simulation is not being used, offline realism must come from:

1. multi-step synthetic LaserScan scenarios
2. geometry stress cases such as corners, narrow turns, close walls, dead ends, and asymmetric corridors
3. noisy/invalid/stale sensor data cases
4. real robot log replay when logs exist

Do not rely only on clean corridor scenarios.

### Benchmark requirements

Offline benchmarks must produce machine-readable logs and summaries:

```text
output/<run_group>/
  *.jsonl
  summary.csv
  summary.md
  regressions.md
```

The JSONL logs should be compatible with the real `reactive_nav_debug.jsonl` style where possible.

### Required failure metrics

Benchmark summaries must include metrics that detect real robot failure modes:

```text
corner_risk_count
front_left_risk_count
front_right_risk_count
side_risk_count
spin_ratio
oscillation_score
angular_sign_changes_per_min
yaw_saturation_ratio
recovery_loop_count
low_progress_ratio
unsafe_yaw_veto_count
```

Safety metrics dominate progress metrics. A profile that moves farther by increasing corner risk or weakening emergency stops must not be promoted.

### Profile promotion rule

A tuned profile may be promoted only if:

1. all safety scenarios pass
2. stale/invalid LiDAR stops safely
3. corner risk is reduced or zero
4. spin/oscillation do not regress
5. recovery loops do not regress
6. score improves on the stricter benchmark
7. regressions are documented

Preserve old safe configs. Do not overwrite them without keeping a copy.

### Iteration logging

For every benchmark-driven tuning session, append a short entry to:

```text
output/navigation_iteration_log.md
```

Each entry should include:

```text
hypothesis
change
benchmark directories
improved metrics
regressed metrics
decision: KEEP / REJECT / ADJUST / SPLIT / MEASURE
```

### Reality boundary

Offline benchmark success does not imply physical readiness.

A final report must explicitly distinguish:

```text
offline validation
robot dry-run validation
physical movement validation
```

Only physical robot tests can establish actual movement readiness.

---

## Offline iteration from real robot failures

When physical robot runs underperform, do not jump straight to more physical testing. First convert the failure evidence into offline regression tests.

Preferred loop:

```text
real robot debug log
  -> analyze failure intervals
  -> replay or reconstruct those intervals offline
  -> run current controller on them
  -> compare old vs new response
  -> add regression scenarios
  -> tune / ablate changes
  -> only then dry-run on robot
```

The goal is not to perfectly simulate physics. The goal is to expose whether the current controller would still choose unsafe or unstable commands when given the same LiDAR/sector/sign/QR context that appeared during real failures.

### Priority evidence sources

Use these first when available:

```text
output/reactive_nav_debug*.jsonl
output/collision_events*.jsonl
output/collision_frames/
output/sim_runs*/
output/tuning_runs*/
```

The debug JSONL log is the most important artifact. It should be treated as a regression dataset.

### Required analysis targets

For each real/debug log, identify intervals of:

```text
corner risk
side scrape risk
spin / high-yaw low-progress
oscillation / frequent angular sign changes
yaw saturation
recovery loops
emergency-stop bursts
turn timeout / alignment timeout
state flapping
```

Convert representative intervals into replay cases or synthetic regression scenarios.

### Implementation rules

- Do not require Gazebo or TurtleBot hardware for this loop.
- Do not publish `/cmd_vel`.
- Do not claim physical readiness from offline replay.
- Keep `wall_follow` as the primary candidate unless evidence says otherwise.
- Keep `follow_gap` as the preferred recovery candidate.
- Keep `focm` experimental until it beats the alternatives on harsh scenarios without safety regressions.
- Any promoted config must improve real-log-derived metrics and keep safety scenarios passing.

### Promotion rule

A tuned profile may be promoted only if:

```text
all critical safety scenarios pass
corner/side risk does not regress
spin and oscillation metrics improve or remain acceptable
real-log replay improves relative to baseline
ablation shows which change caused the improvement
```

If a change improves average score but worsens safety, corner risk, or spin behavior, reject it.

---

## Recovery/Turn Iteration Protocol

Current focus: improve turn completion and recovery behavior without using Gazebo, ROS simulator, or unnecessary physical robot trial-and-error.

Known recent context:

- LiDAR physical alignment was imperfect.
- `scan_to_points` now supports an `angle_offset` compensation.
- The current failure mode is that the robot sometimes starts a turn but gets stuck in `FRONT_BLOCKED_SELECT_FREE_GAP` or `RECOVERY` instead of completing the turn and returning to navigation.
- Finding-gap recovery may be choosing poor gaps, failing to exit, or being invoked too aggressively during intentional turns.

Do not begin with maze simulation. First exploit real robot data and targeted logs because the robot is currently available and real failure logs are higher-signal than synthetic geometry.

### Required iteration loop

Use this loop for all changes:

```text
robot/log evidence
-> identify failure interval
-> improve diagnostics or replay
-> make one controlled change
-> benchmark/replay
-> compare before/after
-> keep, reject, or change direction
```

### Planning-first requirement

When asked to work on this iteration in Codex Plan Mode, do not edit code yet. Produce a plan that includes:

- files to inspect
- suspected failure mechanisms
- proposed minimal changes
- expected metrics
- commands to run
- risks and rollback strategy
- what should wait for execution mode

Only implement after the user approves the plan.

### Priority order

1. Improve recovery/gap/turn diagnostics.
2. Analyze real stuck intervals.
3. Replay real stuck intervals offline when possible.
4. Add targeted ablations.
5. Run short robot dry-run or targeted physical tests.
6. Only later add closed-loop maze simulation.

### Must preserve

- Navigation modules suggest commands only.
- Arbiter owns final command decision and safety.
- YOLO does not drive wheels directly.
- QR/sign logic stays outside navigation modules.
- No offline script publishes `/cmd_vel`.
- Offline improvements must not be called physically validated.
