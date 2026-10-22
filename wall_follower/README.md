# Thymio Wall Follower (ROS 2 Jazzy)

Reference code for a **wall-following** robot built on the **Thymio II**, using **ROS 2** and **Aseba**.
The project is meant for learning how to work with the Thymio and ROS 2.

The node reads the **center** and **right** proximity sensors, keeps the robot at a fixed distance from the wall on its right using **proportional (P) control**, and makes a **sharp left turn** when it meets a wall in front
(inside corner). The linear speed is automatically reduced near obstacles.

## How it works

1. The driver publishes the distance measured by each sensor (in meters) on `/proximity/center` and `/proximity/right`
   (`sensor_msgs/Range` messages).
2. **Linear speed:** computed from two factors (from 0 to 1), one frontal and one lateral.
   The most restrictive one wins: `v = linear_speed * min(front_factor, side_factor)`.
3. **Angular speed:** P control on the distance error to the right wall:
   `angular.z = -10 * (right_dist - 0.05)`. The target distance is **5 cm**.
4. **Wall in front + wall on the right:** the robot enters "turn left" mode, rotates in place at 1.0 rad/s, and only goes back to following the wall when the front is clear.
5. **No wall on the right:** the robot rotates to the right at 0.5 rad/s to look for it.

| Distance to the right | Calculation         | `angular.z` | Result                                |
| --------------------- | ------------------- | ----------- | ------------------------------------- |
| 0.03 m (too close)    | (0.03 - 0.05) * -10 | +0.2        | Turns left (moves away from the wall) |
| 0.05 m (target)       | 0                   | 0.0         | Goes straight                         |
| 0.07 m (too far)      | (0.07 - 0.05) * -10 | -0.2        | Turns right (moves towards the wall)  |
| 0.10 m or more        | -0.5 (fixed value)  | -0.5        | Looks for the wall by rotating right  |

> In ROS, positive `angular.z` = turning **left**; negative = turning **right**.

## Structure

This file contains the whole node (a single script). A typical ROS 2 package structure:

```
wall_follower/                # ROS 2 package (adjust the name to your project)
├── README.md                 # this file
├── package.xml
├── setup.py
└── wall_follower/
    └── wall_follower_node.py # wall follower node
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

Place the robot with the wall **on its right**. To stop, always use `Ctrl+C`.

## Parameters

| Parameter       | Default | What it does                                                                                                |
| --------------- | ------- | ----------------------------------------------------------------------------------------------------------- |
| `linear_speed`  | 0.14    | Maximum linear speed (m/s)                                                                                  |
| `angular_speed` | 0.0     | Internal use: the code stores the last computed angular speed here. **It cannot be used to tune the robot** |

The other values (distances and turning speeds) are **constants inside `control_loop`**.
To change them you need to edit the code:

| Constant          | Value      | What it does                                                 |
| ----------------- | ---------- | ------------------------------------------------------------ |
| `front_slow`      | 0.10 m     | Below this front distance the robot starts to slow down      |
| `front_stop`      | 0.06 m     | Below this front distance the linear speed is zero           |
| `side_slow`       | 0.05 m     | Target distance to the wall; above it the side speed is 100% |
| `side_near`       | 0.03 m     | Below this distance the side speed is at its minimum         |
| `side_min`        | 0.3        | Minimum side factor (30% of the speed)                       |
| `front_release`   | 0.12 m     | The front is considered clear above this value               |
| `left_turn_speed` | 1.0 rad/s  | Rotation speed during the left turn                          |
| gain `-10`        | -10        | Proportional gain (kp) of the wall control                   |
| search rotation   | -0.5 rad/s | Angular speed when there is no wall on the right             |

### How the linear speed is calculated

- **Front factor** (`front_factor`): 1.0 if nothing is detected or if the distance is >= 0.10 m; it drops
  linearly to 0.0 at 0.06 m.
  `front_factor = (center_dist - 0.06) / (0.10 - 0.06)`, clamped between 0 and 1.
- **Side factor** (`side_factor`): 0.0 if there is no wall on the right (the robot does not move forward, it only rotates looking for it);
  otherwise `(right_dist - 0.03) / (0.05 - 0.03)`, clamped between **0.3** and 1.0.
- `v = linear_speed * min(front_factor, side_factor)`.

## Viewing the parameters

```bash
# List the node parameters
ros2 param list /thymio_controller

# Current value of a parameter
ros2 param get /thymio_controller linear_speed

# All values at once
ros2 param dump /thymio_controller
```

## Changing the parameters (while the robot is running)

Always use a decimal point (`0.10`, not `1`):

```bash
ros2 param set /thymio_controller linear_speed 0.10
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

**How to adjust the constants:**

1. Put the robot 5 cm from the wall and check that `/proximity/right` shows a value close to 0.05.
2. Bring an obstacle closer to the center sensor and write down the distance at which it starts being detected.
3. Adjust `front_slow`, `front_stop` and `side_slow` in the code according to what you measured.

The values depend on your environment (wall color and material, lighting), so measure before adjusting.

## Code overview

| Part                                          | What it does                                                                                                                     |
| --------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| `__init__`                                    | Declares the parameters, creates the `/cmd_vel` publisher, the sensor subscribers and the 0.1 s timer                            |
| `center_wall_detected`, `right_wall_detected` | Store the last distance read. They start at `-1.0`, which means "nothing detected"                                               |
| `center_sensor_callback`                      | Updates the center sensor distance on every message                                                                              |
| `right_sensor_callback`                       | Updates the right sensor distance on every message                                                                               |
| `control_loop`                                | Runs 10 times per second: computes the linear speed, the angular speed, decides whether it is turning, and publishes the `Twist` |
| `turning_left`                                | State of the left turn (switches on/off with hysteresis)                                                                         |
| `main`                                        | Starts ROS 2, creates the node and spins it                                                                                      |

### Left turn (state machine)

```
              front blocked AND wall on the right
  FOLLOWING  ------------------------------------->  TURNING LEFT
  THE WALL   <-------------------------------------  (rotates in place at 1.0 rad/s, v = 0)
                      front clear
```

- **Front blocked:** `0.0 <= center_dist < 0.10`
- **Wall on the right:** `0.0 < right_dist < 0.10`
- **Front clear:** no detection, or `center_dist > 0.12`

Since the entry (0.10 m) and exit (0.12 m) values are different, there is a 2 cm **hysteresis**.
This prevents the robot from switching back and forth between the two states when the distance fluctuates near the limit.

If you want the robot to move forward while turning, replace `v = 0.0` with `v = v * 0.3` (there is a comment in the code pointing this out).

## Notes about the code

- The lines `self.center_wall_detected = -1.0` and `self.right_wall_detected = -1.0` appear **twice**
  in `__init__`; the second copy can be removed.
- The comment "Lecture seule" (read-only) says the parameters are only read, but the code calls
  `set_parameters` to write `angular_speed`. That value is **overwritten** right afterwards, so
  changing `angular_speed` from the command line has no real effect on the movement.
- The node has **no protection against sensor loss** (timeout): if the driver stops publishing, the robot keeps
  using the last reading.
- When exiting with `Ctrl+C`, no stop command is sent. The Thymio may keep the last
  speed; check that it has stopped.

## Common problems

| Symptom                                                   | Likely cause                                                                         | What to do                                                                            |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------- |
| `Cannot bind socket to port` on launch                    | An old `asebaros` is still running                                                   | Close the old launch with `Ctrl+C` or use `pkill -f asebaros` and try again           |
| Robot does not move forward and only rotates to the right | There is no wall on the right (or no reading has arrived yet), so `side_factor` is 0 | Check that `/proximity/right` is publishing and that the wall is less than 10 cm away |
| `topic echo` hangs and shows nothing                      | The driver is not publishing (no `asebaros` or no connection to the robot)           | Restart the launch and check `ls /dev/ttyACM*`; use `timeout 3` in the commands       |
| Robot sways from side to side (zigzags)                   | Gain (-10) too high for your robot/wall                                              | Reduce the `-10` value in the code (for example, to `-6`)                             |
| Robot is slow to approach the wall                        | Gain too low                                                                         | Increase the `-10` value in the code                                                  |
| Robot hits the wall in front                              | `front_slow`/`front_stop` too small for your speed                                   | Increase those distances or reduce `linear_speed`                                     |
| Robot turns left and never leaves the turn                | The front never goes above 0.12 m (close wall or noisy reading)                      | Check `/proximity/center` and adjust `front_release`                                  |
| Robot turns the wrong way                                 | Wall placed on the left side                                                         | The node follows the **right** wall; place the robot with the wall on its right       |
