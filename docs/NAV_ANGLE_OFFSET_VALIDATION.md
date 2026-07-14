# NAV_ANGLE_OFFSET_VALIDATION.md — Validating LiDAR Angle Offset Compensation

## Purpose

The robot has a physical LiDAR alignment issue. The code compensates using `angle_offset` in `scan_to_points` or equivalent LiDAR preprocessing.

This must be validated explicitly because a wrong offset can make the robot:

```text
think front obstacles are side obstacles
turn into corners
choose wrong recovery gaps
fail to complete turns
oscillate between recovery states
```

---

## Required invariants

After applying `angle_offset`:

```text
obstacle physically in front -> front/front_center sector becomes small
obstacle physically left -> left sector becomes small
obstacle physically right -> right sector becomes small
open space ahead -> front sector remains large
```

---

## Offline tests

Add unit tests for synthetic scans with known obstacles:

```text
test_angle_offset_zero_front_obstacle
test_angle_offset_positive_rotation
test_angle_offset_negative_rotation
test_front_obstacle_maps_to_front_after_offset
test_left_obstacle_maps_to_left_after_offset
test_right_obstacle_maps_to_right_after_offset
```

These tests should verify sector assignment, not robot movement.

---

## Robot dry-run checks

Use physical obstacle placement without enabling movement:

### Front obstacle check

Place a box/wall directly in front of the robot.

Expected:

```text
lidar.front_center low
lidar.front low
lidar.left/right not falsely lower than front
```

### Left obstacle check

Place obstacle to the left.

Expected:

```text
lidar.left low
lidar.front not falsely low unless obstacle overlaps front field
```

### Right obstacle check

Place obstacle to the right.

Expected:

```text
lidar.right low
lidar.front not falsely low unless obstacle overlaps front field
```

---

## Commands

Run dry-run only:

```bash
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_tuned.yaml \
  -p dry_run:=true \
  -p enable_motion:=false \
  -p diagnostic_period_s:=0.5 \
  -p telemetry_port:=6612 \
  -p persistent_log_path:=/home/ubuntu/output/angle_offset_validation.jsonl
```

Copy back:

```bash
scp turtlebot4:/home/ubuntu/output/angle_offset_validation.jsonl output/robot_runs/angle_offset_validation.jsonl
```

---

## Acceptance

Do not continue tuning recovery if angle offset validation fails.

First fix sector alignment, then revisit recovery.
