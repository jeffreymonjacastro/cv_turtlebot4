# Legacy TurtleBot Tutorial (Redacted)

This archived note preserves the useful checklist shape of an older informal tutorial while removing credentials, private IPs, and casual language that is not appropriate for a portfolio repository.

## SSH and ROS domain

```bash
ssh <robot-user>@<robot-ip-or-host>
export ROS_DOMAIN_ID=<domain-id>
source /opt/ros/jazzy/setup.bash
```

Use the same `ROS_DOMAIN_ID` on the robot and laptop.

## Communication check

Robot terminal:

```bash
ros2 run demo_nodes_cpp listener
```

Laptop terminal:

```bash
ros2 run demo_nodes_cpp talker
```

If messages are visible, ROS discovery is working.

## Bringup and topics

```bash
ros2 launch turtlebot4_bringup lite.launch.py
ros2 topic list
```

Expected topics include:

```text
/scan
/cmd_vel
/odom
/oakd/rgb/preview/image_raw
/tf
```

## Manual movement and sensors

```bash
ros2 topic echo /cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
ros2 launch turtlebot4_bringup oakd.launch.py
ros2 launch turtlebot4_bringup rplidar.launch.py
```

For current project workflows, prefer:

- `docs/REPRODUCIBILITY.md`
- `docs/REACTIVE_NAV_RUNBOOK.md`
- `docs/AUTONOMOUS_LESS_CONSERVATIVE_RUN.md`
