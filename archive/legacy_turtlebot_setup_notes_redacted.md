# Legacy TurtleBot Setup Notes (Redacted)

This file replaces an old root-level lab note that contained real network credentials and an unprofessional filename.

The original note covered:

- connecting to a TurtleBot 4 access point;
- SSH into the robot;
- running `turtlebot4-setup`;
- configuring lab Wi-Fi for the robot and Create 3 base;
- installing ROS 2 Jazzy TurtleBot packages;
- launching TurtleBot bringup;
- checking `/scan`, `/cmd_vel`, `/odom`, and OAK-D camera topics;
- testing teleoperation and sensor launch files.

Credentials, private SSIDs, private IPs, and personal references were removed. Use `.env.example` and the official TurtleBot documentation for a clean setup workflow.

Useful commands from the original note, with placeholders:

```bash
ssh <robot-user>@<robot-ip-or-host>
turtlebot4-setup

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=<domain-id>

ros2 launch turtlebot4_bringup lite.launch.py
ros2 topic list
ros2 topic echo /cmd_vel
ros2 launch turtlebot4_bringup rplidar.launch.py
ros2 launch turtlebot4_bringup oakd.launch.py
```

If using these notes, verify them against the current runbooks under `docs/`.
