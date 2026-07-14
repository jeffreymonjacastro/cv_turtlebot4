# TurtleBot4 Project Handoff

Generated for another coding agent working in:

`C:\Users\jeffr\GitHub\turtlebot`

This document condenses the current project context, repo state, durable memory, and prior chat history that matters for continuing work. It is intentionally descriptive and operational. It is not a full transcript.

## Executive summary

This repo contains TurtleBot4 lab scripts split by execution side:

- `ubuntu/`: robot-side ROS 2 Jazzy nodes intended to run on the TurtleBot or an Ubuntu/WSL2 ROS environment.
- `win/`: laptop-side Windows helpers for UDP telemetry, keyboard control, QR reception, YOLO signal detection, and receivers.
- `kaggle/`: versioned Kaggle training artifacts for YOLO experiments.
- `archive/tutorial_turtlebot4_redacted.md` and `README.md`: local setup/run notes.

The strongest project invariant is the platform and responsibility split. Do not merge robot-side ROS nodes with Windows laptop receivers unless explicitly asked. In particular, keep QR receiving separate from WASD control:

- `win/detect_qr/recibidor.py`: QR/telemetry receiver only.
- `win/original/controller_template.py`: WASD keyboard command sender.
- `ubuntu/*/enviador.py`: robot-side telemetry sender nodes.
- `ubuntu/original/recibidor.py`: robot-side UDP command receiver that publishes `/cmd_vel`.

The recent focus has been reactive navigation in `ubuntu/lidar/`, especially `follow_the_gap_mixed.py`, with UDP diagnostics sent to `win/lidar/recibidor.py`.

## Current git state

As of this handoff, the repo is on:

- Branch: `master`
- Upstream: `origin/master`
- Latest commit seen: `47ae687 Avance yolo y follow`

There are local uncommitted changes. Treat them as user/current-session work and do not revert them casually:

- `ubuntu/lidar/follow_the_gap_mixed.py`: adds automatic LaserScan topic discovery/switching and richer warning logs when no live scan arrives.
- `win/yolo/enviador.py`: changes `ROBOT_IP` from `<robot-ip>` to `<robot-ip>`.
- `win/yolo/recibidor.py`: changes `ROBOT_IP` from `<robot-ip>` to `<robot-ip>` and formats a detection label.

Diff summary at handoff time:

```text
ubuntu/lidar/follow_the_gap_mixed.py | 71 +++++++++++++++++++++++++++++++++++-
win/yolo/enviador.py                 |  2 +-
win/yolo/recibidor.py                |  6 ++-
3 files changed, 75 insertions(+), 4 deletions(-)
```

## Environment assumptions

The user works primarily from Windows/PowerShell in this checkout. The Python laptop-side environment uses `uv`:

- `pyproject.toml` requires Python `>=3.13`.
- Dependencies: `numpy`, `opencv-python`, `ultralytics`.
- `.venv/` is ignored.

ROS 2 dependencies such as `rclpy`, `sensor_msgs`, `geometry_msgs`, and `cv_bridge` are not normal Windows `uv` dependencies for this repo. They belong on the robot or in a ROS 2 Ubuntu environment.

Prior memory says this machine had WSL2 `Ubuntu-24.04` and mirrored networking was enabled through `C:\Users\jeffr\.wslconfig` because ROS 2 discovery depends on multicast. If working on ROS 2 from WSL2, verify current WSL status first.

Do not copy sensitive network credentials into new shared docs. Existing local docs may contain lab network details, but this handoff intentionally avoids reproducing passwords.

## Network and protocol contracts

Common identity and pairing values used across scripts:

- Robot name: `<robot-name>`
- ROS domain commonly used in scripts: `2`
- Pairing code: `<pairing-code>`

Ports:

- UDP `6000`: robot-to-laptop telemetry and handshake.
- UDP `5007`: laptop-to-robot movement commands.

Telemetry handshake:

```text
Laptop -> robot: HELLO <domain_id> <pairing_code>
Robot -> laptop: ACK <domain_id> <robot_name>
```

Telemetry packet types used by receiver workflows:

```text
SCAN <domain_id> <robot_name> <sec> <nsec> <angle_min> <angle_increment> <n> <ranges...>
IMG <domain_id> <robot_name> <sec> <nsec> <base64_jpeg>
QR <domain_id> <robot_name> <sec> <nsec> <content>
LIDAR <domain_id> <robot_name> <sec> <nsec> <state> <front> <left> <right> <speed> <yaw> ...
SCAN_ARRAY <domain_id> <robot_name> <sec> <nsec> <angle_min> <angle_increment> <stride> <n> <ranges...>
LOG <domain_id> <robot_name> <level> <message...>
```

`win/lidar/recibidor.py` already handles `LIDAR`, `SCAN_ARRAY`, and `LOG`. The `LOG` path is important because the user asked for robot diagnostics to be visible on the laptop over UDP.

## Repo map

Top-level:

- `README.md`: lab-specific connection and control notes.
- `archive/tutorial_turtlebot4_redacted.md`: generated tutorial adapted to laptop-terminal usage instead of VM assumptions.
- `pyproject.toml`: Windows/laptop Python dependencies.
- `.gitignore`: ignores `.venv/`, `output/`, `labels-gt/`, and `kaggle/**/outputs/`.
- `carta_de_suicidio_turtlebot2025.txt`: original lab instruction source. Treat as local source context, but avoid propagating secrets.

Robot-side ROS 2 code:

- `ubuntu/original/enviador.py`: robot telemetry sender using `/scan` and `/oakd/rgb/preview/image_raw`.
- `ubuntu/original/recibidor.py`: robot UDP teleop receiver; receives laptop commands on `5007` and publishes `TwistStamped` to `/cmd_vel`.
- `ubuntu/detect_qr/enviador.py`: robot telemetry sender that can transmit QR messages.
- `ubuntu/lidar/follow_the_gap.py`, `follow_the_gap_v2.py`, `follow_the_gap_v3.py`: LiDAR follow-the-gap variants.
- `ubuntu/lidar/follow_the_gap_depth.py`: camera-depth follow-the-gap node. Subscribes to `/oakd/stereo/image_raw` and publishes `/cmd_vel`.
- `ubuntu/lidar/follow_the_gap_rgb_depth.py`: RGB-only pseudo-depth workaround using `/oakd/rgb/preview/image_raw`; includes `--self-test`.
- `ubuntu/lidar/follow_the_gap_mixed.py`: current mixed camera/LiDAR navigator and main recent navigation target.

Laptop-side Windows code:

- `win/original/controller_template.py`: WASD command sender to robot UDP port `5007`.
- `win/original/recibidor_datos.py`: original telemetry receiver.
- `win/detect_qr/recibidor.py`: QR/telemetry receiver only. Keep it separate from movement control.
- `win/lidar/recibidor.py`: LiDAR state receiver and diagnostic log printer.
- `win/yolo/recibidor.py`: receives images/telemetry, loads YOLO model, writes latest signal state.
- `win/yolo/enviador.py`: reads latest signal state and sends robot commands.

Kaggle:

- `kaggle/v1/input/main.ipynb`: first YOLO classification notebook path.
- `kaggle/v2/input/main.py`: detection/pseudo-label experiment.
- `kaggle/v3/input/main.py`: later YOLO training script using manual YOLO labels.
- `kaggle/**/outputs/` is ignored. If an output was already tracked, `.gitignore` alone will not untrack it.

## Current navigation architecture

The most important active file is `ubuntu/lidar/follow_the_gap_mixed.py`.

Despite the filename, its main class is still named `FollowTheGapDepth`. It combines:

- preferred OAK-D stereo depth input,
- RGB fallback/pseudo-depth,
- LiDAR safety layer,
- UDP diagnostic telemetry,
- watchdog/status checks.

Important control flow:

```text
depth_callback() or rgb_callback()
  -> _process_virtual_scan()
  -> find_best_gap() / compute_control()
  -> apply_lidar_safety()
  -> publish_cmd()
```

Important LiDAR methods in `follow_the_gap_mixed.py`:

- `scan_callback()`: stores live LaserScan data.
- `_laser_scan_topics()`: lists visible `sensor_msgs/msg/LaserScan` topics.
- `_maybe_switch_scan_topic()`: current dirty change; switches from `/scan` to another live LaserScan candidate if needed.
- `sector_min()`: computes trusted range by sector.
- `apply_lidar_safety()`: safety veto layer.
- `_build_lidar_frontal_scan()`: creates a planning scan from LiDAR.
- `safety_watchdog()`: stops/warns when camera/LiDAR data goes stale.
- `_send_telemetry_state()`, `_send_log()`, `_send_virtual_scan_array()`: UDP diagnostics.

Prior fixes established that the LiDAR safety layer should be veto-only. It may reduce speed, stop, or veto dangerous turning, but it should not invent extra forward speed.

## Navigation behavior constraints from prior chats

The user cares about real robot behavior over a purely syntactic patch. Observed behavior and pasted logs are the acceptance test.

Specific behavior constraints:

- The robot should not be so conservative that it refuses passable narrow corridors.
- Turning should require more side/corner clearance than going straight.
- If no valid gap is found, rotating toward more open space is better than freezing with zero yaw.
- If an obstacle is truly straight ahead, forward speed should stop, but evasive yaw may still be useful.
- Corridor centering must be gentle. A previous high centering gain made the robot over-turn.
- Prefer centered gaps when several candidates are almost equally wide.
- Separate "straight-ahead obstacle" from "diagonal corridor walls"; otherwise the robot freezes in corridors.

Parameters added or tuned in prior work include:

- `robot_width_m`, `robot_length_m`
- `straight_side_margin`, `turn_side_margin`
- `corridor_centering_gain`
- `corridor_mode_yaw_limit`
- `hard_turn_yaw_threshold`
- `narrow_corridor_speed`
- `min_corridor_width`
- `turn_required_clearance`
- `front_corner_clearance`
- `minimum_gap_width_straight`
- `minimum_gap_width_turn`

## Depth and RGB pseudo-depth context

`ubuntu/lidar/follow_the_gap_depth.py` originally appeared connected but did not move because motion only happened inside `depth_callback()`, and no depth frames were arriving even when the topic looked visible.

Important lesson: topic visibility is not enough. Check publisher counts and actual callback activity.

The user rejected a LiDAR fallback inside the pure depth node with the constraint that it must use the camera. Keep `follow_the_gap_depth.py` camera-depth mandatory unless the user explicitly asks for hybrid behavior.

`ubuntu/lidar/follow_the_gap_rgb_depth.py` was added as a separate no-internal-config workaround. It uses RGB preview, not true metric depth. It should be treated as a low-speed experiment, not a safety-critical depth sensor.

The RGB workaround was hardened with:

- blind side bins,
- larger minimum gap bins,
- center weighting,
- turn-in-place alignment before forward motion.

Validated commands from prior work:

```powershell
python -m py_compile ubuntu/lidar/follow_the_gap_depth.py win/lidar/recibidor.py
python ubuntu/lidar/follow_the_gap_rgb_depth.py --self-test
python -m py_compile ubuntu/lidar/follow_the_gap_rgb_depth.py
python -m py_compile ubuntu/lidar/follow_the_gap_mixed.py
```

## QR and WASD context

The user explicitly corrected an earlier attempt that mixed movement logic into the QR receiver. Preserve this split:

- `win/detect_qr/recibidor.py` only receives QR/telemetry.
- `win/original/controller_template.py` sends WASD commands.

If OpenCV GUI errors such as `qt.qpa.xcb: could not connect to display` appear, it likely means a laptop receiver was run on the robot or another headless environment. Do not keep trying to run that GUI receiver on the TurtleBot. Run the receiver on the laptop or remove display calls if the user asks for a headless receiver.

## YOLO context

The YOLO path is separate from the QR and LiDAR receiver paths:

- `win/yolo/recibidor.py` loads a signal model, processes images, draws detections, and writes a latest-signal JSON/state file.
- `win/yolo/enviador.py` reads the latest signal and sends movement commands.

There are current uncommitted IP edits in both YOLO files from `<robot-ip>` to `<robot-ip>`. Verify the actual robot IP before changing it again.

Kaggle memory indicates earlier work built versioned training artifacts and retrieved models/outputs. Keep outputs versioned under explicit folders and avoid committing generated output directories.

## Prior chat and memory timeline

### 2026-06-02 - Public repo and QR telemetry refactor

Thread id: `019e8960-f9c5-76b3-9529-d86fc7dd4666`

Main outcomes:

- Created public GitHub repo `cv_turtlebot4`.
- Initial push went to `origin/master`.
- Added/refactored QR telemetry path.
- User clarified that the QR receiver runs on the laptop and should not contain movement control.
- Durable rule: QR receiving and WASD sending stay separate.

Useful memory:

- The receiver side expects `ACK 2 <robot-name>` style traffic over UDP `6000`.
- A GUI/display error on the TurtleBot is a deployment-side mistake for laptop receiver scripts.

### 2026-06-02 - uv setup, tutorial, WSL2 mirrored networking

Thread id: `019e8961-2288-76c0-a857-a78cf1124836`

Main outcomes:

- Created a `uv` Python environment and `pyproject.toml`.
- Used Python 3.13 on Windows.
- Added `numpy` and `opencv-python`, later `ultralytics` appears in the current `pyproject.toml`.
- Created `archive/tutorial_turtlebot4_redacted.md`.
- Corrected docs away from VM assumptions because the user was using a laptop terminal.
- Explained robot-side vs laptop-side Python files.
- Recommended WSL2 mirrored networking for ROS 2 discovery.

Durable rule:

- Do not try to force ROS 2 Python packages into the Windows `uv` venv. Use robot/Ubuntu/WSL2 ROS runtime for ROS nodes.

### 2026-06-06 - Ignore Kaggle outputs

Thread id: `019e9de9-0ee7-7dc0-9aa0-463618bce88d`

Main outcome:

- Added `.gitignore` rule `kaggle/**/outputs/`.

Durable rule:

- Keep repo hygiene edits narrowly scoped. If generated outputs are already tracked, use `git rm --cached` separately rather than broad ignore rewrites.

### 2026-06-20 - Kaggle YOLO training and model retrieval

Thread id from memory: `019ee6eb-ab45-7a51-9793-40868543a6ba`

Main memory:

- Built Kaggle notebook/script artifacts under this repo.
- Work involved YOLO26n classification/training and verified output retrieval.
- Future Kaggle work should keep versioned folders and explicit artifacts such as checkpoints, CSVs, ZIPs, and run summaries.

This handoff did not reread that full rollout summary. Re-verify current Kaggle slugs and outputs before rerunning remote jobs.

### 2026-07-01 - Depth troubleshooting, UDP logs, RGB pseudo-depth

Thread id: `019f1f24-8e55-7052-bfe2-7d12890f41ee`

Main outcomes:

- Diagnosed `ubuntu/lidar/follow_the_gap_depth.py` connecting but not moving.
- Added diagnostics for no depth frames and publisher/callback status.
- Added robot-to-laptop UDP `LOG` diagnostics.
- Extended `win/lidar/recibidor.py` to print robot logs.
- Added `ubuntu/lidar/follow_the_gap_rgb_depth.py` as a no-internal-config RGB pseudo-depth workaround.
- Hardened the RGB workaround against side blind spots and bad turns.

Durable rules:

- Use UDP logs to surface robot status on the laptop.
- Do not treat `ros2 topic list`-style visibility as proof frames are arriving.
- Do not add a LiDAR fallback inside the pure camera-depth node unless explicitly requested.

### 2026-07-02 - Mixed follow-the-gap LiDAR safety and turning fix

Thread id: `019f205f-62c2-7cb3-b966-e979cdee1b6e`

Main outcomes:

- Updated `ubuntu/lidar/follow_the_gap_mixed.py` with geometry-aware corridor handling.
- Added TurtleBot4 geometry and turn-clearance parameters.
- Changed no-gap/front-blocked behavior to allow controlled yaw instead of freezing.
- Reworked `apply_lidar_safety()` so passable corridors can still be traversed slowly.
- Fixed over-turning and freezing based on user feedback.
- Corrected inverted corridor centering sign.
- Reduced `corridor_centering_gain` to `0.45`.
- Added a narrow front-center sector to distinguish real frontal blockers from corridor walls.

Durable rule:

- For navigation changes, the user's observed robot behavior is the real acceptance test. Keep iterating against logs and behavior instead of stopping at `py_compile`.

### 2026-07-02 - Current handoff generation

Current task:

- Generate this Markdown handoff with project context, memory, and prior chat summaries for another agent.

No code behavior was intentionally changed by this task.

## Commands another agent should know

Basic repo state:

```powershell
git status --short --branch
git diff --stat
git diff -- ubuntu/lidar/follow_the_gap_mixed.py win/yolo/enviador.py win/yolo/recibidor.py
```

Python/laptop dependencies:

```powershell
uv sync
```

Syntax/self-test checks that have been useful:

```powershell
python -m py_compile ubuntu/lidar/follow_the_gap_mixed.py
python -m py_compile ubuntu/lidar/follow_the_gap_depth.py win/lidar/recibidor.py
python ubuntu/lidar/follow_the_gap_rgb_depth.py --self-test
python -m py_compile ubuntu/lidar/follow_the_gap_rgb_depth.py
```

Ignore-rule verification:

```powershell
git check-ignore -v --no-index kaggle/v1/outputs kaggle/v2/outputs kaggle/v3/outputs
```

WSL2 checks, if ROS 2 from WSL is relevant:

```powershell
wsl --list --verbose
wsl -d Ubuntu-24.04
```

## Working preferences for this user

The user usually wants concrete edits and verification, not plan-only responses.

For this repo:

- Answer with exact files, functions, data flow, and commands.
- If the user asks "en que parte", "que envia", or "que hacen cada uno", give file/function-level tracing.
- Preserve explicit file responsibilities.
- Avoid broad refactors outside the named file or folder.
- When robot behavior is involved, use logs and the user's observed behavior as acceptance tests.
- If the user says not to edit, diagnose only.
- If touching navigation, keep changes tight to `ubuntu/lidar/` unless asked otherwise.
- Do not revert local dirty changes you did not make.

## Open risks and gotchas

- Current local changes are uncommitted. Review them before starting new edits.
- Some repo docs contain lab-specific secrets. Do not propagate credentials into new shared artifacts.
- `follow_the_gap_mixed.py` is long and the class name still says `FollowTheGapDepth`; do not assume it is depth-only.
- `cv2.imshow()` and other GUI calls can fail on headless robot sessions.
- ROS 2 topic visibility can be misleading; use publisher counts and callback activity.
- `.gitignore` does not untrack files already committed.
- A successful `py_compile` does not prove navigation behavior is correct. Real robot feedback matters.
