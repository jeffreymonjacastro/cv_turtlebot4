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

## Existing repo context

The repo is split by execution side:

```text
ubuntu/   robot-side ROS 2 nodes/scripts
win/      laptop-side Windows helpers/receivers/controllers
kaggle/   YOLO training artifacts
```

Preserve this split.

Important existing components:

```text
ubuntu/original/enviador.py        robot telemetry sender
ubuntu/original/recibidor.py       robot UDP command receiver publishing /cmd_vel
ubuntu/detect_qr/enviador.py       robot telemetry sender with QR support
ubuntu/lidar/                      current navigation experiments
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

## Local robot access

Robot access credentials must not be committed.

If `.codex/robot_access.local.md` exists, read it for local-only SSH/IP instructions. Use the SSH alias `turtlebot4` when available.

Never copy robot passwords, Wi-Fi credentials, private keys, or lab network secrets into tracked files.

## Critical implementation principle

Do **not** assume hardware, ROS topics, callbacks, UDP, YOLO, LiDAR, camera, or `/cmd_vel` work.

Before implementing or integrating behavior, test and report:

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


---

Add this to `AGENTS.md`:

## Modularity requirement for navigation

Do not implement the default navigation algorithm as a monolithic controller. The project must allow swapping the local navigation algorithm without changing perception, QR logging, sign handling, safety arbitration, or command publishing.

Use a navigation module interface plus a factory. The main controller should call the selected navigation module and then pass its suggested command through the safety/arbiter layer.

Existing Follow-the-Gap files in `ubuntu/lidar/` may be inspected for topic names, parameters, and prior diagnostics, but the broken implementation should not be treated as the foundation for the new architecture.

## Recommended target files

Prefer adding a new clean implementation rather than patching unstable legacy files unless necessary.

Suggested new robot-side files:

```text
ubuntu/reactive_nav/
  __init__.py
  reactive_navigator.py          main ROS 2 node / entrypoint
  lidar_sectors.py               LaserScan preprocessing and sector min distances
  wall_following.py              PD/PID wall/corridor following
  turn_controller.py             90-degree turn and LiDAR alignment logic
  behavior_arbiter.py            state machine and priority rules
  qr_logger.py                   persistent QR evidence logging
  diagnostics.py                 UDP logs/state packets
```

Suggested laptop-side integration files, only if needed:

```text
win/yolo/latest_signal_schema.md  document the JSON/state file schema
win/yolo/signal_state_reader.py   if a reusable reader is needed
```

If the current repo structure makes a package folder difficult, create a single new file first:

```text
ubuntu/lidar/reactive_yolo_lidar_nav.py
```

But keep code modular internally with functions/classes.

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
```

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
```

Do not hide failures. If a topic is missing or stale, report it clearly and stop safely.

## Work protocol for coding agents

Before code edits:

```powershell
git status --short --branch
git diff --stat
```

Do not revert user changes casually.

When touching navigation:

1. Inspect current relevant files.
2. Identify actual ROS topics and message types.
3. Add small testable changes.
4. Run static checks.
5. Run sensor tests.
6. Report what was tested, what passed, what failed, and what still requires the physical robot.

Do not claim behavior works from `py_compile` alone.

## Validation expectation

A patch is not complete unless it includes:

- exact files changed
- exact commands run
- expected runtime command(s)
- fallback behavior when sensors fail
- clear acceptance criteria
- diagnostic logs visible to the user

## References to keep in repo docs

Recommended links:

- TurtleBot 4 hardware features:
  https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html
- F1TENTH Wall Following Lab:
  https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab3.html
- F1TENTH Follow the Gap Lab:
  https://f1tenth-coursekit.readthedocs.io/en/stable/assignments/labs/lab4.html
- Vector Field Histogram paper:
  https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/borenstein_VFHisto.pdf
