# FSM, YOLO, and QR Supervision Run

This guide validates the perception-to-action pipeline without beginning with autonomous physical movement.

The workflow is divided into three phases:

```text
Phase 1: synthetic event -> FSM -> intended action
Phase 2: camera input -> validated YOLO/QR event
Phase 3: real perception -> FSM -> intended action, with motion blocked
```

Use the phases in order. Do not skip directly to integrated testing when synthetic event handling has not passed.

> This is a documentation draft for Codex to adapt to the repository after implementation. Script names, flags, paths, state names, and topic names must be replaced with the exact implemented values.

## What this validates

The full chain is:

```text
camera or test injector
  -> raw YOLO/QR result
  -> confidence, debounce, freshness, and duplicate checks
  -> semantic event
  -> behavior arbiter
  -> FSM transition
  -> intended velocity command
  -> motion safety gate
```

The important distinction is:

```text
intended command != physically published command
```

During these tests, the FSM may calculate a turn or stop command, but physical movement must remain disabled.

## Safety state required for all phases

Before starting, confirm:

```text
dry_run = true
enable_motion = false
no direct YOLO command sender is running
arbiter is the only component allowed to produce motion decisions
```

Do not run a legacy YOLO sender that commands the robot directly.

Keep the robot on the floor only if all published physical commands are guaranteed to remain zero. Otherwise, power down actuation or use the project’s approved stationary setup.

## Expected evidence

Each test run should make the following visible:

```text
raw detection
validated event or rejection reason
current and previous FSM state
transition reason
active maneuver
intended linear.x and angular.z
published linear.x and angular.z
dry-run and enable-motion flags
```

A useful record should answer:

```text
What was perceived?
Was it accepted?
Why was it accepted or rejected?
What state changed?
What action did the FSM intend?
Was physical movement blocked?
```

---

# Phase 1 — FSM only

## Goal

Validate FSM interpretation and action selection without using the camera.

```text
synthetic YOLO/QR event -> arbiter -> FSM -> intended command
```

## What this isolates

A failure in this phase belongs to one of these components:

```text
event schema
signal reader
arbiter priority
FSM transition logic
maneuver controller
command safety gate
```

It is not a YOLO model or QR decoder failure.

## Terminal A — Start the navigator safely

Use the implementation-provided dry-run command:

```bash
# Replace with the exact repository command.
python3 <reactive_navigator.py> --ros-args \
  -p dry_run:=true \
  -p enable_motion:=false
```

Expected startup evidence:

```text
dry_run=True
enable_motion=False
published command remains zero or disabled
structured diagnostics active
```

Stop immediately if either safety flag is incorrect.

## Terminal B — Watch supervision output

Use the implemented dashboard, overlay, or structured log follower:

```bash
python3 <supervision_view.py>
```

Expected idle state:

```text
no active event
normal navigation or safe idle state
no active maneuver
published command physically safe
```

## Test 1 — Inject LEFT

```bash
python3 scripts/inject_perception_event.py yolo LEFT
```

Expected:

```text
raw/candidate signal: LEFT
validated event: LEFT
accepted: yes
transition reason: YOLO_LEFT or repository equivalent
active maneuver: left turn
intended angular.z: positive, according to project convention
published motion: zero or blocked
```

Acceptance:

- exactly one new maneuver begins;
- the maneuver is not restarted every control cycle;
- the transition reason identifies the YOLO event;
- intended yaw has the correct sign;
- physical output remains safe.

## Test 2 — Inject RIGHT

```bash
python3 scripts/inject_perception_event.py yolo RIGHT
```

Expected:

```text
accepted: yes
active maneuver: right turn
intended angular.z: negative, according to project convention
published motion: zero or blocked
```

## Test 3 — Inject STOP

Run only if STOP is supported:

```bash
python3 scripts/inject_perception_event.py yolo STOP
```

Expected:

```text
state or command source indicates STOP
intended linear.x = 0
intended angular.z = 0, unless the current safety design requires another safe behavior
published output remains safe
```

## Test 4 — Inject QR payload

```bash
python3 scripts/inject_perception_event.py qr CHECKPOINT_TEST_1
```

Expected:

```text
QR payload decoded/received
QR event accepted
QR hold, scan, or logging state entered as designed
payload persisted once
published output remains safe
```

Confirm the persistent QR log contains the payload.

## Test 5 — Duplicate QR

Inject the same payload again:

```bash
python3 scripts/inject_perception_event.py qr CHECKPOINT_TEST_1
```

Expected:

```text
duplicate recognized
no second checkpoint registration, unless repeat logging is explicitly intended
explicit rejection or duplicate reason
```

## Negative tests

### Stale YOLO event

```bash
python3 scripts/inject_perception_event.py yolo LEFT --stale
```

Expected:

```text
accepted: no
reason: stale
no YOLO-triggered transition
```

### Low-confidence event

```bash
python3 scripts/inject_perception_event.py yolo LEFT --confidence 0.10
```

Expected:

```text
candidate visible in diagnostics
validated event absent
reason: confidence below threshold
```

### Unknown event

```bash
python3 scripts/inject_perception_event.py yolo UNKNOWN
```

Expected:

```text
safe rejection
no crash
no maneuver
```

## Phase 1 acceptance checklist

- [ ] LEFT produces the correct intended turn direction.
- [ ] RIGHT produces the correct intended turn direction.
- [ ] STOP produces the configured stop behavior, if supported.
- [ ] QR produces the configured hold/log action.
- [ ] Duplicate QR behavior is deterministic.
- [ ] Stale, weak, and unknown events are explicitly rejected.
- [ ] Repeated events do not continuously restart a maneuver.
- [ ] Physical motion remains disabled.

Do not continue if event acceptance, transition reason, or command direction is ambiguous.

---

# Phase 2 — Perception only

## Goal

Validate real YOLO and QR perception while excluding FSM actuation from the diagnosis.

```text
physical sign or QR -> raw detection -> validated event
```

## What this isolates

A failure in this phase belongs to:

```text
camera input
YOLO inference
class mapping
confidence/area filtering
debounce and confirmation
signal freshness
QR localization or decoding
QR duplicate handling
```

## Start perception components

### YOLO receiver/viewer

```bash
python3 <yolo_receiver_or_viewer.py>
```

The camera window should show, where implemented:

```text
class
confidence
confirmation count
event emitted or rejected
rejection reason
```

Do not run a script that sends movement commands.

### QR detector

Start the repository’s QR detection path if it is not part of the navigator:

```bash
python3 <qr_detector_or_safe_navigator.py> <safe flags>
```

## Test setup

Use printed or on-screen test assets:

- one supported LEFT sign;
- one supported RIGHT sign;
- STOP, if supported;
- one clearly rendered QR code with a unique payload.

Present one asset at a time. Keep lighting and distance stable before testing difficult conditions.

## Test 1 — YOLO raw detection

Present a LEFT sign.

Expected overlay or logs:

```text
raw class: LEFT
confidence above configured threshold
fresh timestamp
confirmation progress increases
validated LEFT event appears after required confirmation
```

Remove the sign.

Expected:

```text
raw detection disappears
latest signal becomes NONE or stale according to design
old LEFT event does not remain indefinitely actionable
```

Repeat for RIGHT and STOP if supported.

## Test 2 — Rejection visibility

Move the sign farther away, partially occlude it, or show it briefly.

Expected:

```text
raw candidate may appear
validated event is absent when requirements are not met
rejection reason identifies confidence, area, confirmation, or freshness
```

This distinction is essential:

```text
not detected
```

is different from:

```text
detected but rejected
```

## Test 3 — YOLO cooldown

Trigger one validated sign, remove it, and present it again before cooldown ends.

Expected:

```text
second candidate visible
second action event rejected or deferred
reason: cooldown active
```

After cooldown expires, repeat and confirm the event can become actionable again.

## Test 4 — QR decode

Present a QR code steadily.

Expected:

```text
raw QR candidate visible
decoded payload matches exactly
confirmation requirement completes
QR event emitted
payload written to persistent log
```

## Test 5 — Duplicate QR

Keep the same QR visible or present it again.

Expected:

```text
payload recognized as already seen
no uncontrolled repeated action
explicit duplicate status
```

## Phase 2 acceptance checklist

- [ ] Camera frames are fresh.
- [ ] Every supported sign maps to the correct semantic label.
- [ ] Confirmation/debounce prevents one-frame actions.
- [ ] Removed signs do not remain actionable indefinitely.
- [ ] Cooldown behavior is visible and deterministic.
- [ ] QR payload is decoded exactly.
- [ ] Duplicate QR behavior is visible and deterministic.
- [ ] No perception process commands the wheels directly.

Do not continue if class mapping is wrong, timestamps are stale, or rejected events have no visible reason.

---

# Phase 3 — Integrated stationary dry-run

## Goal

Validate the full chain while physically blocking motion:

```text
real perception -> validated event -> arbiter -> FSM -> intended command
```

## Required safety state

Confirm all of the following before presenting a sign or QR:

```text
dry_run=True
enable_motion=False
published command is zero or disabled
no legacy direct-command sender is running
LiDAR safety is active or missing-data behavior is safe
```

Use a second terminal to monitor the actual command output when available:

```bash
ros2 topic echo <cmd_vel_topic>
```

Acceptance before continuing:

```text
physical command remains zero or no actuator command is published
```

## Integrated LEFT test

Present the LEFT sign until it is confirmed.

Expected sequence:

```text
1. raw LEFT detection
2. confirmation progress reaches threshold
3. validated LEFT event
4. event accepted by arbiter
5. FSM transitions into left-turn maneuver
6. intended angular.z has left-turn sign
7. published physical command remains safe
8. maneuver completes or times out deterministically
9. cooldown begins
```

Record the transition reason and command source.

## Integrated RIGHT test

Repeat for RIGHT.

Acceptance:

- event class is correct;
- FSM enters the right maneuver;
- intended yaw direction is correct;
- physical output remains blocked.

## Integrated QR test

Present a new QR payload.

Expected sequence:

```text
1. QR candidate
2. successful decode
3. stable/validated QR event
4. arbiter selects QR behavior according to priority
5. FSM enters QR hold/scan/log state if designed
6. payload is persisted
7. intended command matches QR policy
8. physical output remains blocked
```

## Priority test — Safety over perception

Create only the repository-approved stationary safety condition, such as placing a non-moving obstacle in front of the LiDAR while motion remains disabled.

Then present a turn sign.

Expected:

```text
sign can be detected
semantic event may be produced
arbiter rejects, defers, or safely constrains the action
transition reason identifies emergency or obstacle priority
no unsafe intended forward motion
```

YOLO must never bypass LiDAR emergency safety.

## Priority test — Active maneuver over repeated sign

While a synthetic or real maneuver is active, keep the same sign visible.

Expected:

```text
active maneuver is not restarted every frame
repeated event is ignored, rejected, or deferred with an explicit reason
```

## Phase 3 acceptance checklist

- [ ] Real LEFT causes the correct FSM transition and intended yaw.
- [ ] Real RIGHT causes the correct FSM transition and intended yaw.
- [ ] STOP behaves correctly, if supported.
- [ ] Real QR causes the intended hold/log behavior.
- [ ] Transition reasons match the event source.
- [ ] Command source matches the selected arbiter priority.
- [ ] Repeated events do not restart active maneuvers continuously.
- [ ] Safety conditions dominate YOLO and QR actions.
- [ ] Published physical command remains zero or disabled throughout.

Passing Phase 3 proves stationary integration only. It does not prove that physical maneuvers are safe or correctly calibrated.

---

# Save and summarize evidence

Use the implemented output directory convention:

```bash
python3 scripts/summarize_perception_fsm_run.py output/<run_directory>
```

The summary should include:

```text
detections observed
validated events
accepted and rejected events
rejection reasons
FSM transitions
intended commands
published commands
motion-gate status
QR payloads logged
```

Preserve the run directory before changing parameters or code.

# Troubleshooting map

| Symptom | Most likely boundary |
|---|---|
| Nothing appears in camera view | Camera source or receiver |
| Box appears but class is wrong | YOLO model or class mapping |
| Correct class appears but no event | Confidence, area, confirmation, freshness, or cooldown |
| Event appears but no FSM transition | Arbiter priority or transition guard |
| Transition occurs but wrong turn direction | Maneuver mapping or yaw convention |
| QR is visible but payload is absent | QR localization/decoder |
| Payload appears repeatedly | Duplicate suppression or event lifecycle |
| Intended command is correct but physical command moves in dry-run | Critical command-gate failure |
| Sign overrides emergency safety | Critical arbiter-priority failure |
| Maneuver restarts every frame | Missing edge-trigger, cooldown, or active-maneuver guard |

# Stop conditions

Stop testing immediately when:

- `enable_motion` becomes true unexpectedly;
- a non-zero physical command is published during dry-run;
- YOLO or QR bypasses the arbiter;
- stale perception remains actionable;
- an emergency safety condition loses priority;
- command direction does not match the requested maneuver;
- structured logs cannot explain event rejection or transition cause.

# Cleanup

Stop all test processes and clear only temporary injected state using the implemented cleanup command:

```bash
python3 scripts/inject_perception_event.py reset
```

Do not delete persistent QR evidence or debug logs until the run has been reviewed.
