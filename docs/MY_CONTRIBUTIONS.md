# My Contributions

This was a team project. The repository and commit history should be treated as the source of truth for individual authorship. This document uses cautious wording and avoids claiming that one person independently built the full system.

## Contribution areas supported by repository evidence

The current repository supports describing contributions in these areas:

- contributed to a modular TurtleBot 4 autonomy stack with separate robot-side navigation, laptop-side perception, event sync, QR logging, and behavior arbitration;
- developed and evaluated finite-state / priority-based arbitration for LiDAR safety, active maneuvers, QR events, visual signs, default navigation, and sensor-stale fallback;
- built deterministic offline validation around synthetic LaserScan replay and profile comparison;
- added real-log analysis tooling for corner risk, side scrape risk, oscillation, spin, recovery loops, state flapping, yaw saturation, and turn/recovery intervals;
- iterated on wall-follow, gap-recovery, and turn/recovery profiles using logs and replay rather than only subjective real-robot runs;
- integrated laptop-side YOLO sign detections with robot-side FSM actionability gates;
- contributed to a ZXing-based laptop QR pipeline with validation, semantic event files, sync diagnostics, and robot-side persistent QR logs;
- improved documentation and runbooks for reproducible robot data capture, dry-runs, QR validation, and autonomous runs.

## Suggested CV bullets

- Developed and evaluated a modular TurtleBot 4 autonomy prototype combining LiDAR reactive navigation, YOLO traffic-sign events, QR checkpoint logging, and priority-based behavior arbitration.
- Built offline replay and diagnostics infrastructure for synthetic scenarios, real robot logs, turn/recovery intervals, and perception-to-FSM validation, enabling evidence-driven tuning before physical robot tests.

## 30-second interview explanation

This project was a TurtleBot 4 autonomy stack for an indoor course. The interesting challenge was integrating unreliable real-time perception with safe robot behavior. We split the stack into laptop perception, robot-side LiDAR navigation modules, QR logging, and a finite-state arbiter that owns `/cmd_vel`. A major focus was diagnostics: JSONL logs, deterministic replay, real-log failure extraction, and perception/FSM benchmarks. The main lesson was that robot failures often come from freshness gates, actionability thresholds, recovery loops, or safety arbitration details rather than from one isolated model or controller.

## Likely interview questions

### 1. Why not use Nav2/SLAM?

The project focused on a lightweight reactive stack for an unknown indoor circuit with sign and QR behaviors. Nav2/SLAM could be useful in a different phase, but the immediate goal was to validate perception-to-action behavior, LiDAR safety, and reproducible debugging without depending on full map-building.

### 2. How did you prevent YOLO from directly driving the robot?

YOLO writes symbolic detections to `latest_signal.json`. The robot-side reader checks freshness and gates, the sign debouncer confirms the event, and the behavior arbiter decides whether that event can affect motion. The arbiter remains the only path to `/cmd_vel`.

### 3. What made QR detection difficult?

Robot-side OpenCV decoding was brittle and could block callback timing. The later design decodes frames on the laptop with ZXing, validates repeated payloads into semantic events, syncs those events atomically to the robot, and lets the robot persist accepted checkpoints with event IDs and context.

### 4. How did you debug failures from physical robot runs?

The controller writes structured JSONL diagnostics. Scripts extract intervals such as corner risk, recovery loops, spin, yaw saturation, and state flapping. Those intervals can be replayed at sector level to compare controller variants without immediately repeating physical runs.

### 5. What are the main limitations?

Offline replay is not a physics simulator, QR benchmarks need more varied negative and geometric cases, and recovery behavior still has stress cases with spin/yaw warnings. The real robot remains the final validation layer after unit tests, replay, stationary perception, and dry-run checks.

## Claims to verify before using in a CV or interview

- Which parts you personally implemented versus reviewed, integrated, or tested.
- Whether the latest final-run robot behavior passed a physical course after the documented fixes.
- Whether the cited `output/` summaries are available in the version you share, since generated logs are ignored by default.
- Whether the YOLO model weights are redistributable and present in the public repository.
- Whether all lab-specific credentials have been rotated if the repository was ever shared while they were present.
