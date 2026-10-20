# Thymio Wall Follower (ROS 2 Jazzy)

Reference code for a **wall-following** robot built on the **Thymio II**, using **ROS 2** and **Aseba**.
The project is meant for learning how to work with the Thymio and ROS 2.

The node reads the **center** and **right** proximity sensors and is organized as a **finite state machine (FSM)**
with three states. While following the wall, it keeps the robot at a fixed distance from the wall on its right using a
**PD controller** (proportional + derivative). When it meets a wall in front (inside corner) it turns left in place,
and when it loses the wall it runs a search maneuver (move forward 15 cm, then turn 90° to the right).
The linear speed is automatically reduced near obstacles and in sharp curves.

## How it works

1. The driver publishes the distance measured by each sensor (in meters) on `/proximity/center` and `/proximity/right`
   (`sensor_msgs/Range` messages).
2. **Sensor validation:** invalid readings (not finite or `<= 0`) become `None` ("nothing detected").
   A reading older than `sensor_timeout` is also treated as `None`.
3. Every 0.1 s (10 Hz) the control loop computes three conditions from the readings, updates the **state** and
   publishes a `Twist` command on `/cmd_vel` according to the current state.
4. In `FOLLOW_WALL`, the **angular speed** comes from a PD controller on the distance error to the right wall,
   and the **linear speed** is reduced near front obstacles and in strong curves.

> In ROS, positive `angular.z` = turning **left**; negative = turning **right**.

## States

| State         | What the robot does                                                                                         | How it enters                                                                                                                    | How it leaves                                                                                                      |
| ------------- | ----------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `FOLLOW_WALL` | Moves forward and corrects its heading with the PD controller to keep `target_wall_dist` from the right wall | Front not blocked **and** wall detected on the right; or leaving `TURN_LEFT` with the front clear and a wall detected            | Front becomes blocked (`TURN_LEFT`) or the wall is lost (`SEARCH_WALL`)                                            |
| `TURN_LEFT`   | Rotates in place to the left (`linear.x = 0`, `angular.z = min(1.0, max_angular_speed)`)                     | Front blocked                                                                                                                    | Front becomes clear: goes to `FOLLOW_WALL` if a wall is detected on the right, otherwise to `SEARCH_WALL`           |
| `SEARCH_WALL` | Runs the search maneuver: moves forward 15 cm, then turns 90° to the right                                   | Front not blocked **and** no wall on the right; or leaving `TURN_LEFT` with the front clear and no wall                          | When the maneuver ends: goes to `FOLLOW_WALL` if a wall is detected, otherwise the maneuver restarts               |

The initial state is `FOLLOW_WALL`. Every state change is logged by the node (`Estado: <STATE>`).

### Conditions

| Condition       | Definition                                                         | Default values                    |
| --------------- | ------------------------------------------------------------------ | --------------------------------- |
| `front_blocked` | `front <= front_stop_dist`                                         | `front <= 0.06 m`                 |
| `front_clear`   | no front reading, or `front >= front_release_dist`                | `front >= 0.08 m`                 |
| `wall_detected` | `right < wall_lost_dist`                                           | `right < 0.12 m`                  |

Since the blocked (0.06 m) and release (0.08 m) distances are different, there is a 2 cm **hysteresis**.
This prevents the robot from switching back and forth between states when the distance fluctuates near the limit.

### Transition priority

Evaluated in this order on every cycle:

1. If the state is `TURN_LEFT`, it stays there until the front is clear.
2. If the front is blocked, go to `TURN_LEFT` (this also interrupts a search maneuver).
3. If the state is `SEARCH_WALL` and the maneuver has not finished, keep doing the maneuver.
4. If there is no wall on the right, go to (or restart) `SEARCH_WALL`.
5. Otherwise, go to `FOLLOW_WALL`.

Any unexpected state stops the robot for safety.

```
                     front blocked
   FOLLOW_WALL / SEARCH_WALL  ---------------->  TURN_LEFT
        ^        |                                  |
        |        | wall lost                        | front clear
        |        v                                  |
        |   SEARCH_WALL  <--- no wall --------------+
        |        |
        +--------+  wall detected (after the maneuver)
        ^
        +---------- wall detected (after TURN_LEFT)
```

## Wall following (PD controller)

In `FOLLOW_WALL` the right sensor is used to keep the robot at `target_wall_dist` from the wall:

```
error      = right - target_wall_dist         # positive = wall too far
derivative = (error - previous_error) / dt
d_filtered = 0.25 * derivative + 0.75 * d_filtered_previous     # low-pass filter against sensor noise
angular.z  = -(kp * error + kd * d_filtered)                    # clamped to ±max_angular_speed
```

| Distance to the right | `error` | `angular.z` (P term only, `kp = 8`) | Result                                |
| --------------------- | ------- | ----------------------------------- | ------------------------------------- |
| 0.05 m (too close)    | -0.02   | +0.16                               | Turns left (moves away from the wall) |
| 0.07 m (target)       | 0.00    | 0.00                                | Goes straight                         |
| 0.09 m (too far)      | +0.02   | -0.16                               | Turns right (moves towards the wall)  |

The derivative term acts as damping: it reacts to how fast the error changes and reduces oscillations.

### Linear speed

`v = linear_speed * min(front_factor, turn_factor)`

- **Front factor:** 1.0 if nothing is detected in front or the distance is at least `front_release_dist`;
  it drops linearly to 0.0 at `front_stop_dist`.
  `front_factor = (front - front_stop) / (slow_dist - front_stop)`, clamped between 0 and 1, with
  `slow_dist = max(front_release, front_stop + 0.01)`.
- **Turn factor:** `1 - 0.5 * |angular.z| / max_angular_speed`. The stronger the curve, the slower the robot
  (down to 50% of the speed).

### Search maneuver (`SEARCH_WALL`)

When the wall is lost, the robot runs two phases, **controlled by time** (there is no odometry):

1. **Advance:** `linear.x = 0.10 m/s` for `0.15 m / 0.10 m/s = 1.5 s`.
2. **Turn:** `angular.z = -0.8 rad/s` (right) for `(π/2) / 0.8 ≈ 1.96 s`.

The maneuver is not interrupted when the wall shows up halfway; only a blocked front interrupts it.
At the end, the state machine re-evaluates: it goes to `FOLLOW_WALL` if a wall is detected, otherwise the
maneuver restarts. Entering `SEARCH_WALL` always restarts the maneuver from the beginning.
Because it is time-based, the real distance and angle are approximate and may need fine tuning.

## Structure

This file contains the whole node (a single script). A typical ROS 2 package structure:

```
wall_follower/                # ROS 2 package (adjust the name to your project)
├── README.md                 # this file
├── package.xml
├── setup.py
└── wall_follower/
    └── wall_follower_node.py # wall follower node (state machine + PD controller)
```

## Requirements

- Ubuntu 24.04 with **ROS 2 Jazzy**

- The `ros-thymio` package (`thymio_driver` and `asebaros`) built in the same workspace

- Python/ROS dependencies used by the node: `rclpy`, `geometry_msgs`, `sensor_msgs`

  ```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  ```

- Thymio II turned on and connected over USB (shows up as `/dev/ttyACM0`)

## Build

Always run `colcon` commands from the **workspace root** (adjust the package name):

```bash
cd ~/thymio_ws
colcon build --packages-select wall_follower --symlink-install
source install/setup.bash
```

## Run

Use one terminal for each step (in all of them, first run `source /opt/ros/jazzy/setup.bash` and `source ~/thymio_ws/install/setup.bash`).

**1. Thymio driver**

```bash
export DEVICE="ser:name=Thymio"
export SIMULATION="False"
ros2 launch thymio_driver main.launch device:=$DEVICE simulation:=$SIMULATION
```

**2. Wall follower** (with the driver already running)

```bash
ros2 run wall_follower wall_follower_node
```

Or, without installing it as a package, directly from the script:

```bash
python3 wall_follower_node.py
```

Place the robot with the wall **on its right**. To stop, use `Ctrl+C`: the node publishes a zero-velocity command before shutting down.

## Parameters

All the main constants are declared as ROS 2 parameters and are read on every control cycle,
so they can be changed **while the robot is running** (see below).

| Parameter            | Default | What it does                                                                                                                                  |
| -------------------- | ------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `linear_speed`       | 0.10    | Maximum linear speed in `FOLLOW_WALL` (m/s)                                                                                                   |
| `wall_lost_dist`     | 0.12    | Right distance from which the wall is considered lost, which makes the robot enter `SEARCH_WALL` (m)                                          |
| `front_stop_dist`    | 0.06    | Front distance at or below which the front is blocked (enters `TURN_LEFT`); the linear speed in `FOLLOW_WALL` reaches zero here (m)           |
| `front_release_dist` | 0.08    | Front distance from which the front is clear again (leaves `TURN_LEFT`). Being larger than `front_stop_dist`, it creates the hysteresis (m)   |
| `sensor_timeout`     | 0.5     | Maximum age of a sensor reading; older readings are treated as "no data" (s)                                                                   |
| `target_wall_dist`   | 0.07    | Desired distance to the right wall; the setpoint of the PD controller (m)                                                                     |
| `max_angular_speed`  | 0.8     | Saturation of the PD output; also the rotation speed of `TURN_LEFT` (capped at 1.0) (rad/s)                                                   |
| `kp`                 | 8.0     | Proportional gain: how strongly the robot corrects the distance error (rad/s per meter of error)                                              |
| `kd`                 | 0.25    | Derivative gain: damping that reduces oscillations by reacting to how fast the error changes                                                  |

The search maneuver values are **attributes set in `__init__`** (not ROS parameters). To change them, edit the code:

| Attribute               | Value      | What it does                                           |
| ----------------------- | ---------- | ------------------------------------------------------ |
| `search_advance_dist`   | 0.15 m     | Distance moved forward in `SEARCH_WALL`                |
| `search_advance_speed`  | 0.10 m/s   | Forward speed during the search (not tied to `linear_speed`) |
| `search_turn_angle`     | π/2 rad    | Angle turned after advancing (90°)                     |
| `search_turn_speed`     | -0.8 rad/s | Angular speed of the search turn (negative = right)    |

Other internal values: control period 0.1 s and derivative filter factor `alpha = 0.25`.

## Viewing the parameters

```bash
# List the node parameters
ros2 param list /thymio_controller

# Current value of a parameter
ros2 param get /thymio_controller kp

# All values at once
ros2 param dump /thymio_controller
```

## Changing the parameters (while the robot is running)

Always use a decimal point (`0.10`, not `1`):

```bash
ros2 param set /thymio_controller linear_speed 0.10
ros2 param set /thymio_controller kp 6.0
ros2 param set /thymio_controller kd 0.3
```

## Viewing the sensor values

Each `Range` message carries the distance to the obstacle in **meters**.

```bash
# Continuous reading of the center sensor
ros2 topic echo /proximity/center --field range

# Continuous reading of the right sensor
ros2 topic echo /proximity/right --field range

# A single reading
ros2 topic echo /proximity/right --field range --once

# See the commands sent to the robot
ros2 topic echo /cmd_vel
```

**How to adjust the parameters:**

1. Put the robot at the distance you want from the wall and check that `/proximity/right` shows a value close to `target_wall_dist`.
2. Bring an obstacle closer to the center sensor and write down the distance at which it starts being detected.
3. Adjust `front_stop_dist`, `front_release_dist`, `wall_lost_dist` and `target_wall_dist` according to what you measured.
4. Tune `kp` first (until the robot follows the wall without too much delay), then increase `kd` to remove oscillations.

The values depend on your environment (wall color and material, lighting), so measure before adjusting.

## Code overview

| Part                                           | What it does                                                                                                                              |
| ---------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `__init__`                                     | Declares the parameters, creates the `/cmd_vel` publisher, the sensor subscribers, the 0.1 s timer and the initial state                    |
| `_read_range`                                  | Converts missing/invalid readings into `None`                                                                                             |
| `center_sensor_callback`, `right_sensor_callback` | Store the last distance read and the time of the reading                                                                               |
| `_sensor_is_fresh`                             | Checks that the last reading is not older than `sensor_timeout`                                                                           |
| `change_state`                                 | Changes the state and logs it; entering `SEARCH_WALL` restarts the search maneuver                                                        |
| `reset_search`                                 | Restarts the search maneuver (advance phase, timer cleared)                                                                               |
| `control_loop`                                 | Runs 10 times per second: computes the conditions, updates the state machine, computes `linear.x`/`angular.z` and publishes the `Twist`    |
| `main`                                         | Starts ROS 2, creates the node and spins it; on exit it publishes a zero `Twist` to stop the robot                                        |

## Notes about the code

- The search maneuver is **time-based (open loop)**: there is no odometry, so the 15 cm and 90° are approximate.
- The search values (`search_*`) are not ROS parameters, so changing them requires editing the code.
- The old state names in the comments (`TURN_RIGHT`, `FIND_WALL`) are not used; the real states are
  `FOLLOW_WALL`, `TURN_LEFT` and `SEARCH_WALL`.
- If a sensor stops publishing, its reading expires after `sensor_timeout` and is treated as "nothing detected".
  Without the right sensor the robot enters `SEARCH_WALL`.

## Common problems

| Symptom                                                    | Likely cause                                                                                   | What to do                                                                           |
| ---------------------------------------------------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `Cannot bind socket to port` on launch                     | An old `asebaros` is still running                                                             | Close the old launch with `Ctrl+C` or use `pkill -f asebaros` and try again          |
| Robot keeps moving 15 cm forward and turning right         | No wall on the right (or no recent reading), so it stays in `SEARCH_WALL`                      | Check that `/proximity/right` is publishing and that the wall is closer than `wall_lost_dist` |
| `topic echo` hangs and shows nothing                       | The driver is not publishing (no `asebaros` or no connection to the robot)                     | Restart the launch and check `ls /dev/ttyACM*`; use `timeout 3` in the commands      |
| Robot sways from side to side (zigzags)                    | `kp` too high or `kd` too low for your robot/wall                                              | Reduce `kp` (for example, to `6.0`) and/or increase `kd`                             |
| Robot is slow to approach the wall                         | `kp` too low                                                                                   | Increase `kp`                                                                        |
| Robot hits the wall in front                               | `front_stop_dist`/`front_release_dist` too small for your speed                                | Increase those distances or reduce `linear_speed`                                    |
| Robot turns left and never leaves `TURN_LEFT`              | The front never reaches `front_release_dist` (close wall or noisy reading)                     | Check `/proximity/center` and adjust `front_release_dist`                            |
| The search maneuver goes too far / turns too little or too much | Time-based maneuver; real speed differs from the commanded one                            | Adjust `search_advance_dist` and `search_turn_angle` in the code                     |
| Robot turns the wrong way                                  | Wall placed on the left side                                                                   | The node follows the **right** wall; place the robot with the wall on its right      |
