# Navigation Parameter Tuning Guide

This document explains the parameters listed in `docs/NAV_ITERATION_LOOP.md` and how to tune them without making unrelated changes simultaneously.

## How to use this guide

For each observed failure:

1. Identify the dominant symptom.
2. Change one parameter or one tightly related parameter group.
3. Re-run the same regression scenarios.
4. Keep the change only if the target improves without a safety regression.

Values are not universally optimal. LiDAR calibration, control-loop frequency, robot dimensions, floor friction, and commanded velocity scaling all affect the useful range.

## Common conventions

| Term | Meaning |
|---|---|
| Linear speed | Forward velocity, normally in `m/s` |
| Yaw | Angular velocity around the vertical axis, normally in `rad/s` |
| Distance | LiDAR-derived clearance, normally in meters |
| Per cycle | Per navigation-control update; behavior therefore depends on loop frequency |

---

# Parameters shared by multiple navigation modules

## `base_speed`

**Meaning:** Default forward speed when no immediate obstacle or special maneuver requires slowing down.

**Increase when:**

- The robot is safe and stable but makes unnecessarily slow progress.
- Open corridors dominate and stopping distance remains adequate.

**Decrease when:**

- The robot reaches corners too quickly to react.
- Steering oscillations grow with speed.
- LiDAR updates or control commands are delayed.
- The real robot behaves less safely than replay/simulation.

**Typical side effects:** Increasing it improves progress but reduces reaction time and amplifies steering error. Tune it after safety distances and steering behavior are stable.

## `max_yaw`

**Meaning:** Maximum magnitude of commanded angular velocity.

**Increase when:**

- The robot cannot finish turns before reaching a wall.
- It consistently turns too wide.
- Recovery rotation is too weak to escape blocked states.

**Decrease when:**

- The robot overshoots the desired heading.
- It alternates rapidly between left and right commands.
- It spins aggressively near corners.
- Wheel slip or localization instability appears.

**Typical side effects:** A larger value improves turning authority but can produce overshoot, oscillation, and corner strikes. If turns start correctly but do not finish, verify state-transition and completion conditions before increasing this value.

## `front_stop_distance`

**Meaning:** Front clearance below which forward motion must stop or transition into an avoidance/recovery behavior.

**Increase when:**

- The robot stops too late.
- Momentum, command latency, or LiDAR noise causes near-collisions.
- The physical robot is less predictable than replay data.

**Decrease when:**

- The robot stops in passages that are still safely traversable.
- It repeatedly enters `FRONT_BLOCKED_SELECT_FREE_GAP` too early.
- It cannot approach a corner closely enough to perform the intended turn.

**Typical side effects:** Too high causes false blockage and recovery loops; too low creates collision risk. Treat this as a safety parameter and change it conservatively.

## `front_corner_avoid_distance`

**Meaning:** Clearance threshold for treating front-left or front-right obstacles as corner hazards, even when the central front sector is not fully blocked.

**Increase when:**

- The robot clips inside corners.
- It commands yaw toward an obstacle located diagonally ahead.
- Side sectors become unsafe during turns.

**Decrease when:**

- Valid turns are vetoed too early.
- The robot refuses to enter moderately narrow openings.
- Corner avoidance dominates even in wide corridors.

**Typical side effects:** Higher values are safer but more conservative. This should usually be at least as large as `front_stop_distance` because diagonal corner risk appears before a direct frontal collision.

## `gap_side_margin_m`

**Meaning:** Extra clearance added on each side of a candidate gap beyond the robot's nominal footprint.

**Increase when:**

- The selected gap is geometrically valid but too tight in practice.
- The robot scrapes walls or corners while entering gaps.
- LiDAR noise makes gap boundaries unreliable.

**Decrease when:**

- No gap is accepted in corridors the robot can physically traverse.
- The robot remains stuck in gap-selection or recovery states.
- Openings are rejected despite sufficient real clearance.

**Typical side effects:** Higher values improve robustness but reduce the number of usable gaps.

## `gap_min_width_deg`

**Meaning:** Minimum angular width a free LiDAR region must have to be considered a valid gap.

**Increase when:**

- Narrow noisy gaps are selected.
- The target heading jumps between small openings.
- The robot attempts turns that cannot accommodate its body.

**Decrease when:**

- Valid nearby openings are rejected.
- Tight maze corridors produce no candidate gap.
- The robot remains in `FRONT_BLOCKED_SELECT_FREE_GAP` despite an obvious exit.

**Typical side effects:** This is an angular criterion, not a physical-width guarantee. The same angle represents a smaller physical opening near the robot and a larger opening farther away.

---

# `wall_follow` parameters

## `narrow_speed`

**Meaning:** Forward speed used when side clearances indicate a narrow corridor or constrained passage.

**Increase when:**

- Narrow corridors are handled reliably but too slowly.
- The robot remains centered and has sufficient braking margin.

**Decrease when:**

- It oscillates between walls.
- Side corrections cannot settle before the next obstacle measurement.
- It approaches narrow turns too aggressively.

**Relationship:** Normally lower than `base_speed`.

## `corner_speed`

**Meaning:** Forward speed used while approaching or negotiating a corner.

**Increase when:**

- The robot nearly stops during otherwise stable cornering.
- Turns are completed safely but progress is unnecessarily low.

**Decrease when:**

- It clips corners.
- It cannot rotate enough before advancing.
- Forward motion fights the turning maneuver.

**Relationship:** Usually the smallest forward-speed setting: `corner_speed <= narrow_speed <= base_speed`.

## `wall_kp`

**Meaning:** Proportional steering gain for wall-distance error. It controls how strongly current lateral error changes yaw.

**Increase when:**

- The robot reacts too weakly to drifting toward or away from the followed wall.
- It takes too long to recover the target wall distance.

**Decrease when:**

- It zigzags along a straight wall.
- Small LiDAR variations produce large steering commands.
- It repeatedly overshoots the desired wall distance.

**Typical side effects:** Too low gives sluggish correction; too high gives oscillation. Tune this before relying on `wall_kd`.

## `wall_kd`

**Meaning:** Derivative steering gain. It reacts to how quickly the wall-distance error is changing and can damp proportional overshoot.

**Increase when:**

- `wall_kp` gives acceptable responsiveness but the robot overshoots and oscillates.
- Wall-distance error changes rapidly near the setpoint.

**Decrease when:**

- Yaw becomes noisy or jerky.
- LiDAR noise causes command spikes.
- Steering reacts strongly to one-frame measurement changes.

**Typical side effects:** Derivative control is sensitive to noisy or irregularly timed measurements. Keep it small and use filtered error if possible.

## `front_slow_distance`

**Meaning:** Front clearance below which the robot begins reducing forward speed, before reaching `front_stop_distance`.

**Increase when:**

- Braking begins too late.
- The robot reaches corners with too much forward speed.
- Commands or sensor updates have noticeable latency.

**Decrease when:**

- It slows excessively in open but visually cluttered spaces.
- Progress is poor even though stopping remains safe.

**Relationship:** Must be greater than `front_stop_distance`. The gap between them defines the braking region.

## `side_avoid_distance`

**Meaning:** Side clearance below which an additional steering correction pushes the robot away from a nearby wall or obstacle.

**Increase when:**

- The robot travels too close to side walls.
- Rear or side body clearance becomes unsafe during turns.

**Decrease when:**

- It is over-repulsed from both walls in narrow corridors.
- Side avoidance fights the intended wall-following behavior.
- It cannot pass through traversable passages.

**Typical side effects:** Excessively high values can make a narrow corridor appear blocked from both sides.

## `avoidance_gain`

**Meaning:** Strength of the corrective yaw generated by side/corner obstacle avoidance.

**Increase when:**

- The robot detects a side or corner hazard but does not turn away strongly enough.
- Corner-clearance violations persist despite a sensible threshold.

**Decrease when:**

- Avoidance causes abrupt heading changes.
- It bounces between left and right obstacles.
- Avoidance overwhelms the normal wall-following command.

**Tuning order:** Set detection distances first, then tune this gain. A gain cannot compensate for a badly chosen activation threshold.

## `max_angular_delta_per_cycle`

**Meaning:** Maximum amount by which the yaw command may change between consecutive control cycles.

**Increase when:**

- Steering is too sluggish to react to sudden obstacles.
- A turn command ramps up too slowly and the robot advances into danger.

**Decrease when:**

- Commands switch sharply between left and right.
- Mechanical motion is jerky.
- LiDAR noise causes sudden angular-command changes.

**Important:** This value depends directly on control-loop frequency. At 10 Hz, `0.10 rad/s per cycle` permits a `1.0 rad/s` change over one second; at 20 Hz it permits twice that rate. Prefer a time-based angular acceleration limit if loop timing varies.

## `spin_detection_window_s`

**Meaning:** Time window over which the system evaluates whether the robot is rotating without meaningful translation.

**Increase when:**

- Legitimate long turns are incorrectly classified as spinning.
- Recovery activates before a commanded turn can finish.

**Decrease when:**

- The robot wastes too much time circling before recovery begins.
- Spin detection reacts too slowly to persistent zero-progress rotation.

**Typical side effects:** Too short causes false positives during valid turns; too long delays recovery from actual loops.

## `spin_yaw_threshold`

**Meaning:** Minimum angular-speed magnitude considered significant rotation for spin detection.

**Increase when:**

- Normal steering corrections are falsely classified as spinning.
- Curved forward motion triggers recovery.

**Decrease when:**

- Slow persistent rotation is not detected.
- The robot can remain in a low-yaw loop indefinitely.

**Relationship:** Evaluated together with `spin_linear_threshold` and `spin_detection_window_s`.

## `spin_linear_threshold`

**Meaning:** Maximum forward-speed magnitude under which rotation is treated as occurring with insufficient translational progress.

**Increase when:**

- The robot spins while creeping forward and avoids detection because its linear velocity is slightly above the threshold.

**Decrease when:**

- Slow but valid cornering is classified as spinning.
- Recovery activates during deliberate low-speed maneuvers.

**Typical condition:** Spin detection generally means `|yaw| > spin_yaw_threshold` and `|linear| < spin_linear_threshold` for enough of `spin_detection_window_s`.

---

# `follow_gap` parameters

## `gap_bubble_radius_m`

**Meaning:** Radius around the nearest obstacle that is masked out before searching for the best free gap. It prevents selecting headings too close to that obstacle.

**Increase when:**

- The chosen gap passes too close to the nearest obstacle.
- The robot cuts across obstacle edges or inside corners.
- LiDAR uncertainty requires more clearance.

**Decrease when:**

- The bubble removes most or all usable free space.
- Narrow corridors produce no valid gap.
- The robot repeatedly falls into gap-selection or recovery states.

**Typical side effects:** Too small is unsafe; too large makes the planner overly conservative. It interacts strongly with `gap_side_margin_m`.

## Follow-gap speed and safety parameters

`base_speed`, `max_yaw`, `gap_min_width_deg`, `gap_side_margin_m`, `front_stop_distance`, and `front_corner_avoid_distance` have the shared meanings described earlier.

For follow-gap specifically:

- Reduce `base_speed` when the selected target angle changes frequently.
- Increase `max_yaw` only when the target is stable but the robot cannot align quickly enough.
- Modify gap geometry parameters when no safe target is found; do not use higher yaw as a substitute for invalid gap selection.

---

# `focm` parameters

FOCM should remain experimental until its outputs are validated against the same safety and progress metrics as the other modules.

## `focm_alpha`

**Meaning:** Algorithm-specific weighting that controls the strength or shape of the FOCM steering response. Its exact physical interpretation depends on the implemented FOCM equation.

**Increase when:**

- Obstacle-circle steering is too weak or changes direction too slowly.
- The generated path does not bend sufficiently around an obstacle.

**Decrease when:**

- Steering becomes overly aggressive or unstable.
- Small LiDAR changes produce large target-angle changes.
- The robot follows unnecessarily wide or oscillatory arcs.

**Required verification:** Check the implementation before tuning. Confirm where `focm_alpha` appears in the equation and whether increasing it truly increases steering aggressiveness. Do not rely only on the parameter name.

## `robot_width_m`

**Meaning:** Width used by FOCM when determining whether a gap or obstacle-clearance path can physically accommodate the robot.

**Increase when:**

- The planner accepts gaps that are too narrow on the real robot.
- Body clearance, not LiDAR-center clearance, causes collisions.

**Decrease when:**

- Physically traversable gaps are rejected.
- The configured width already includes excessive duplicate margins.

**Important:** Prefer the measured maximum body width, including any protruding components relevant to collision. Add uncertainty using `gap_side_margin_m`; do not intentionally falsify robot width to tune general conservatism.

## Other FOCM parameters

`gap_side_margin_m`, `gap_min_width_deg`, `base_speed`, and `max_yaw` retain the shared meanings above. Validate FOCM geometry first, then tune motion limits.

---

# Symptom-to-parameter guide

| Observed symptom | First parameters to inspect | Likely direction |
|---|---|---|
| Clips front-left/front-right corners | `front_corner_avoid_distance`, `corner_speed`, `avoidance_gain` | Increase distance; reduce speed; then increase gain if needed |
| Stops too early before turns | `front_stop_distance`, `front_corner_avoid_distance` | Decrease cautiously |
| Enters gap-selection state with no accepted gap | `gap_bubble_radius_m`, `gap_side_margin_m`, `gap_min_width_deg` | Decrease one at a time |
| Selects unsafe narrow gaps | `gap_side_margin_m`, `gap_min_width_deg`, `gap_bubble_radius_m` | Increase |
| Starts a turn but never completes it | Turn completion/state logic, then `max_yaw`, `corner_speed` | Fix transition logic first; increase yaw or reduce forward speed only if authority is insufficient |
| Overshoots turns | `max_yaw`, `max_angular_delta_per_cycle` | Decrease yaw and/or smooth more |
| Zigzags while wall following | `wall_kp`, `wall_kd`, `max_angular_delta_per_cycle` | Reduce `kp`; add small `kd`; reduce delta limit |
| Wall correction is too weak | `wall_kp`, `avoidance_gain` | Increase relevant gain |
| Spins too long before recovery | `spin_detection_window_s`, `spin_yaw_threshold`, `spin_linear_threshold` | Shorter window, lower yaw threshold, or higher linear threshold |
| Valid turn incorrectly triggers anti-spin | Same spin parameters | Longer window, higher yaw threshold, or lower linear threshold |
| Narrow corridor causes left-right repulsion | `side_avoid_distance`, `avoidance_gain`, `wall_kp` | Reduce side threshold/gain or proportional gain |
| Safe but generally too slow | Speed parameters | Increase only after safety behavior is stable |

---

# Recommended tuning order

Use this order to avoid tuning downstream behavior around unsafe geometry:

1. **Sensor validity and calibration** — angle offset, sector definitions, invalid/stale scan handling.
2. **Robot geometry** — measured width and clearance margins.
3. **Hard safety thresholds** — stop and corner distances.
4. **Gap validity** — bubble radius, minimum angular width, side margin.
5. **State transitions** — maneuver completion, blocked-state timeout, recovery entry and exit.
6. **Steering response** — proportional/derivative/avoidance gains and yaw limit.
7. **Command smoothing** — angular delta or acceleration limit.
8. **Spin detection** — only after valid turns have clear completion criteria.
9. **Forward speeds** — raise progress last.

A parameter search should not compensate for incorrect state logic. In particular, persistent `FRONT_BLOCKED_SELECT_FREE_GAP` or `RECOVERY` states require logging why transitions fail: no valid gap, unstable selected gap, unmet heading tolerance, stale scan, timeout reset, or contradictory safety veto.

# Minimum logging needed for useful tuning

Each control cycle or state transition should make the following inspectable:

- active state and time in state
- front, front-left, front-right, and side clearances
- raw desired yaw and final limited yaw
- commanded linear velocity
- selected gap start/end/width and target angle
- reason each candidate gap was rejected
- active safety vetoes
- spin detector inputs and accumulated duration
- recovery entry and exit reason

Without these values, changing thresholds is guesswork rather than controlled tuning.
