# ROS 2 / Gazebo Simulation on macOS Note

## Recommendation

Do not prioritize native macOS ROS 2 + Gazebo/TurtleBot4 simulation for the current iteration.

The current objective is faster iteration without using the TurtleBot or ROS simulator. Real-log replay and harsh temporal synthetic scenarios are more directly useful right now because they target the actual failures already observed on the robot.

## Practical alternatives

Preferred order:

```text
1. Offline synthetic temporal benchmark
2. Real robot log replay/regression
3. Ablation and bounded tuning
4. Robot dry-run/no-motion validation
5. Low-speed robot test
6. Optional Gazebo later
```

If simulation becomes necessary later, prefer:

```text
Ubuntu VM or Linux machine with ROS 2 + TurtleBot4 simulator
```

over a native macOS Gazebo/TurtleBot setup.

## Why not now

Native macOS ROS/Gazebo setup can consume significant time on environment issues that do not directly address the known failures:

```text
corner risk
side scrape risk
spin intervals
oscillation intervals
recovery loops
```

These are already visible in logs and can be turned into offline regression tests immediately.

## If Codex proposes simulation

Codex may document a future setup path, but should not block the current offline improvement loop on ROS/Gazebo installation.
