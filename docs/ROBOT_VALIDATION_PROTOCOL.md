# docs/ROBOT_VALIDATION_PROTOCOL.md

## Purpose

This protocol prevents coding agents from assuming the TurtleBot stack works. Run these checks before claiming that navigation is integrated.

## Rule

A successful import, syntax check, or `ros2 topic list` is not enough.

A component is considered usable only when:

```text
topic exists
publisher exists
messages arrive
callback fires
data is fresh
failure is reported if it becomes stale
```

## 0. Repo and environment checks

Run before editing:

```powershell
git status --short --branch
git diff --stat
```

Do not revert dirty files unless the user explicitly asks.

Laptop Python:

```powershell
uv sync
python --version
python -m py_compile win/yolo/recibidor.py win/yolo/enviador.py
```

Robot/Ubuntu ROS environment:

```bash
echo $ROS_DOMAIN_ID
ros2 node list
ros2 topic list
```

If working from WSL2, verify networking before debugging ROS code:

```powershell
wsl --list --verbose
```

## 1. LiDAR checks

Do not assume `/scan` is correct. Discover live LaserScan topics:

```bash
ros2 topic list -t | grep LaserScan
```

For each candidate:

```bash
ros2 topic info /scan
ros2 topic echo /scan --once
ros2 topic hz /scan
```

Expected:

```text
publisher count > 0
messages arrive
ranges length > 0
angle_min / angle_increment valid
reasonable finite ranges exist
```

If topic exists but no messages arrive, report it and do not move.

## 2. LiDAR callback test

Add or run a small diagnostic node that logs:

```text
scan_count
last_scan_age
range_min
range_max
number of finite ranges
front_min
left_min
right_min
rear_min
```

Acceptance:

```text
scan_count increases continuously
last_scan_age < 0.5 s during run
front/left/right values update when obstacles move
```

## 3. Command velocity test

Before autonomous motion, test minimal movement in open space.

Use extremely low speeds.

Example:

```bash
ros2 topic info /cmd_vel
```

Then publish zero:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

If the robot expects `TwistStamped` or a different topic, identify the correct interface before coding.

Minimal forward test only when safe:

```bash
# adapt message type/topic to actual robot interface
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.03}, angular: {z: 0.0}}"
```

Always immediately stop after test.

## 4. Emergency stop test

With robot lifted or in a controlled area:

```text
place obstacle in front
verify front sector drops below threshold
verify state becomes EMERGENCY_STOP
verify linear_x = 0
verify diagnostic reason is visible
```

Acceptance:

```text
robot never drives forward into obstacle
logs explain stop reason
```

## 5. Camera checks

Find image topics:

```bash
ros2 topic list -t | grep -E "Image|CompressedImage"
```

Check the expected OAK-D topic:

```bash
ros2 topic info /oakd/rgb/preview/image_raw
ros2 topic hz /oakd/rgb/preview/image_raw
```

If using laptop-side receiver, verify images are arriving on the laptop receiver, not just on the robot.

## 6. YOLO checks

The existing YOLO detector works decently. Do not replace it first.

Check:

```text
model loads
frames arrive
detections are printed/displayed
latest signal state file is written
timestamp updates
class name is stable
confidence is logged
bbox area ratio is available or can be computed
```

If the state file does not have all needed fields, extend it minimally and document the schema.

Synthetic test:

```text
manually write latest_signal.json with LEFT
confirm navigation reads it
confirm debouncing/cooldown logic works
confirm robot does not move unless LiDAR is safe
```

## 7. QR checks

QR evidence must persist to disk.

Check:

```text
QR visible to camera
decoded content appears
same QR is confirmed across frames if needed
record appended to output/qr_log.jsonl
duplicate QR is ignored after first valid log
```

Suggested command:

```bash
tail -f output/qr_log.jsonl
```

## 8. UDP diagnostics checks

Existing protocol uses robot-to-laptop telemetry/logs. Verify receiver first:

```powershell
python win/lidar/recibidor.py
```

Expected diagnostic types:

```text
LOG
LIDAR
SCAN_ARRAY
QR
```

The autonomous node should report:

```text
state
sensor freshness
front/left/right/rear distances
chosen linear_x/angular_z
stop reason
turn reason
sign state
QR state
```

## 9. Integration test sequence

### Test A — static, no motion

```text
run sensors
run diagnostics receiver
run navigation node with movement disabled or dry-run
verify logs
```

### Test B — safety movement only

```text
run low-speed forward in clear space
place obstacle
verify stop
remove obstacle
verify recovery or idle
```

### Test C — default navigation

```text
straight corridor
no signs
robot should move slowly and stay centered/parallel
no wall clipping
```

### Test D — synthetic sign

```text
inject LEFT state
robot should debounce
execute safe left turn
align after turn
cooldown prevents duplicate turn
```

### Test E — real sign

```text
show printed sign
YOLO detects it
latest state updates
robot executes exactly one maneuver
```

### Test F — QR

```text
show QR
robot slows/stops
logs QR once
resumes navigation
```

### Test G — circuit attempt

```text
full run
save logs
save QR evidence
write short failure report if it fails
```

## 10. Failure reporting format

When something fails, report in this format:

```text
Component:
Expected:
Observed:
Evidence/log:
Likely cause:
Fix attempted:
Next action:
```

Do not just say "it does not work."

## 11. Acceptance criteria

Minimum acceptable implementation:

```text
LiDAR stale -> stop
front obstacle -> stop
clear corridor -> slow forward navigation
YOLO LEFT/RIGHT -> debounced event
turn command -> safety checked
after turn -> LiDAR alignment
QR decoded -> persistent JSONL log
all major decisions -> visible diagnostics
```
