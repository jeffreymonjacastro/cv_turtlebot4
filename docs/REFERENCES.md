# docs/REFERENCES.md

## Purpose

These references justify the selected navigation strategy and give implementation guidance.

The project implementation should stay simple:

```text
LiDAR wall/corridor following
+ free-gap recovery
+ YOLO sign events
+ QR evidence logging
```

## Primary implementation reference: F1TENTH Wall Following

URL:

```text
https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab3.html
```

Use for:

- PID/PD wall following
- LiDAR-based distance-to-wall estimation
- keeping a vehicle parallel to a wall/corridor
- practical controller structure

Important adaptation:

```text
F1TENTH output: steering angle
TurtleBot output: angular_z
```

## Free-space recovery reference: F1TENTH Follow the Gap

URL:

```text
https://f1tenth-coursekit.readthedocs.io/en/stable/assignments/labs/lab4.html
```

Paper can be found at `papers/ftg.pdf`

Use for:

- emergency/recovery behavior
- finding the largest safe gap in LiDAR
- avoiding blind freezing when front is blocked
- choosing a heading when wall following cannot proceed

Do not directly trust the existing repo implementation. Reimplement or simplify after validating LiDAR data.

## Conceptual reference: Vector Field Histogram

URL:

```text
https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/borenstein_VFHisto.pdf
```

Paper can be found at `papers/VFHisto.pdf`

Use for:

- justification of angular sector/histogram navigation
- obstacle density/free-space reasoning
- real-time local obstacle avoidance

You do not need to implement full VFH. The project can cite it as the basis for using quantized LiDAR sectors and free-space direction selection.

Suggested paper folder:

```text
docs/references/
  borenstein_koren_1991_vector_field_histogram.pdf
```

## Hardware reference: TurtleBot 4 Features

URL:

```text
https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html
```

Use for:

- TurtleBot 4 Lite hardware description
- OAK-D-Lite camera
- RPLIDAR A1M8
- Raspberry Pi 4
- velocity/sensor platform constraints

## Optional background: Follow-the-Gap papers

Search/add only if the report needs more literature.

Suggested topic:

```text
Follow the Gap Method obstacle avoidance mobile robots
```

Potential use:

- justify largest-free-gap recovery
- explain why the controller is reactive and lightweight

## Follow the Obstacle Circle Method

Paper:
- Houshyari, H., & Sezer, V. "A new gap-based obstacle avoidance approach: follow the obstacle circle method." Robotica, 2021.

Paper can be found at `papers/focm.pdf`

Use:
- Future replacement for simpler Follow-the-Gap or wall-following modules.
- Relevant because it improves classical Follow-the-Gap by selecting gaps based on physical width and computing avoidance headings using obstacle circles.

Implementation note:
- Do not implement FOCM before the baseline robot loop works.
- FOCM requires robust gap-edge extraction from LiDAR and careful conversion from polar scan data to Cartesian obstacle-border points.

## How to cite in report

Suggested wording:

```text
The navigation stack uses a reactive local-control approach. LiDAR scans are converted into angular sectors inspired by histogram-based obstacle avoidance methods such as VFH. During normal motion, a PD wall/corridor-following controller keeps the robot centered. When the frontal sector is blocked or no safe corridor-following command exists, the controller switches to a Follow-the-Gap-style recovery behavior that selects the safest visible free sector. Visual detections are not used as direct motor commands; they are debounced into symbolic events handled by a safety-first behavior arbiter.
```
