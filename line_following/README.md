# Thymio Line Follower (ROS 2 Jazzy)

Reference code for a line-following robot built on the **Thymio II**, using **ROS 2** and **Aseba**.
The project is meant for learning how to work with the Thymio and ROS 2.

The node reads the **raw values of the two ground sensors**, turns them into a "how much line do I see"
level, and steers the robot with **proportional (P) control**. All the tuning values are ROS parameters,
so they can be **changed in real time**, without stopping the robot and without editing the code.

## How it works

1. The driver publishes the raw reflection of each ground sensor (0 to 1023) on `/aseba/events/ground_reflected`.
2. Each raw value is converted to a level between **0.0** (floor) and **1.0** (line), using the
   calibration parameters `line_value` and `floor_value`.
3. The error is the difference between the two sensors: `error = left level - right level`.
4. The robot turns proportionally to the error: `angular.z = kp * error`. The bigger the error,
   the stronger the turn, and the slower the robot goes.
5. If no sensor sees the line, the robot goes straight for a short time and then rotates towards the
   side where the line was last seen.

| Left level | Right level | Error | Result (with `kp` = 0.8) |
|---|---|---|---|
| 1.0 | 0.0 | +1.0 | Turns left at 0.8 rad/s |
| 1.0 | 1.0 | 0.0 | Goes straight |
| 0.0 | 1.0 | -1.0 | Turns right at 0.8 rad/s |
| 0.7 | 0.2 | +0.5 | Turns left at 0.4 rad/s (gentler correction) |

## Structure

This repository holds more than one project. This project lives in the `line_following/` folder:

```
line_following/
├── README.md                 # this file
└── line_follower/            # ROS 2 package
    ├── package.xml
    ├── setup.py
    ├── resource/
    ├── test/
    └── line_follower/
        └── line_follower.py  # line follower node
```

## Requirements

- Ubuntu 24.04 with **ROS 2 Jazzy**
- The `ros-thymio` package (`thymio_driver` and `asebaros`) built in the same workspace
- The `asebaros_msgs` package (comes with `ros-thymio`), declared in the `package.xml` of `line_follower`:
  ```xml
  <exec_depend>asebaros_msgs</exec_depend>
  ```
- Thymio II turned on and connected over USB (shows up as `/dev/ttyACM0`)

## Get the project

Clone the repository inside the `src/` folder of your workspace:

```bash
cd ~/thymio_ws/src
git clone https://github.com/EduSF00/Thymio.git
```

`colcon` finds the package in the subfolders. Since the repository has other projects, build only this
package (command below).

## Build

Always run `colcon` commands from the **workspace root**:

```bash
cd ~/thymio_ws
colcon build --packages-select line_follower --symlink-install
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

**2. Line follower** (with the driver already running)

```bash
ros2 run line_follower line_follower
```

To stop, always use `Ctrl+C` and wait for the processes to finish.

## Parameters

| Parameter | Default | Range | What it does |
|---|---|---|---|
| `kp` | 0.8 | 0 to 5 | Proportional gain: rad/s of turn per unit of error |
| `forward_speed` | 0.10 | 0 to 0.5 | Linear speed (m/s) with the line centered |
| `turn_linear_speed` | 0.06 | 0 to 0.5 | Linear speed (m/s) when the error is maximum |
| `line_value` | 250.0 | 0 to 1023 | Raw sensor reading over the line |
| `floor_value` | 950.0 | 0 to 1023 | Raw sensor reading over the floor |
| `lost_level` | 0.3 | 0 to 1 | Level below which the line is considered lost |

The defaults assume a **black line over a white floor**. How they are used:

- `line_value` and `floor_value` are the two reference points of the calibration: the raw value over the
  line becomes level 1.0, the raw value over the floor becomes level 0.0, and everything in between is proportional.
- `lost_level`: if neither sensor reaches this level, the robot stops following the line and starts the search.
- The speed goes from `forward_speed` (error 0) down to `turn_linear_speed` (error 1) as the error grows.

## Viewing the parameters

```bash
# List the node parameters
ros2 param list /line_follower

# Current value of a parameter
ros2 param get /line_follower kp

# Description and allowed range
ros2 param describe /line_follower kp

# All values at once
ros2 param dump /line_follower
```

## Changing the parameters (while the robot is running)

Always use a decimal point (`0.6`, not `1`):

```bash
ros2 param set /line_follower kp 0.6
ros2 param set /line_follower forward_speed 0.08
ros2 param set /line_follower turn_linear_speed 0.05
ros2 param set /line_follower line_value 250.0
ros2 param set /line_follower floor_value 950.0
ros2 param set /line_follower lost_level 0.25
```

If a value is outside the allowed range, or if `line_value` becomes equal to `floor_value`, the node
rejects the change and shows the reason in its terminal.

Tuning `kp`: if the robot zigzags, lower it; if it is slow to return to the line, raise it.

## Viewing the floor colors (raw values)

Each number is the infrared reflection read by a sensor, from 0 to 1023. **A lower value means a darker surface.**
The message carries two numbers: the first is the left sensor and the second is the right sensor.

```bash
# Continuous reading of both sensors
ros2 topic echo /aseba/events/ground_reflected --field data

# A single reading
ros2 topic echo /aseba/events/ground_reflected --field data --once

# Full message (with stamp and source)
ros2 topic echo /aseba/events/ground_reflected --once
```

**How to calibrate:**

1. Put both sensors over the **line** and write down the values.
2. Repeat over the **floor**.
3. Apply them:
   ```bash
   ros2 param set /line_follower line_value <value_over_the_line>
   ros2 param set /line_follower floor_value <value_over_the_floor>
   ```

Measure both sensors, because each one reads slightly differently. The bigger the difference between
`line_value` and `floor_value`, the less noisy the control is.

**Following a white line:** just swap what the two values mean. Set `line_value` to the reading over
the white line and `floor_value` to the reading over the floor (for example, `line_value` 957 and
`floor_value` 265 for a white line over a black floor). The formula works in both directions.

Example of raw values measured in our setup (your track may give different numbers):

| Surface | Approximate raw value |
|---|---|
| Air (robot lifted) | 26 |
| Black | 258-273 |
| Brown | 367-390 |
| White | 957 |

## Code overview

| Part | What it does |
|---|---|
| `PARAM_INFO` | Table with the default value, description and limits of every parameter. A loop declares them all in ROS |
| `GroundSensor` | Stores the last raw value of one sensor and tells whether it has received data |
| `SensorDatabase` | Registers the sensors in one place. Offers `all_ready()` and `snapshot()` (current values of all sensors) |
| `ground_callback` | Receives each message from the driver and updates the sensors in the database |
| `calculate_darkness` | Converts a raw value to a level from 0.0 (floor) to 1.0 (line) |
| `control_loop` | Runs 10 times per second: reads the sensors, computes the error and chooses between following and searching |
| `follow_line` | P control: `angular.z = kp * error`, and reduces the speed as the error grows |
| `search_line` | Line lost: goes straight for 0.5 s, then rotates towards the side where the line was last seen |
| `parameters_callback` | Runs before a parameter changes and rejects invalid combinations |

About the locks (`threading.Lock`): they protect the sensor dictionary and the navigation state
(`last_direction` and `line_lost_time`). The node uses the default single-threaded executor (`rclpy.spin`),
so callbacks never run at the same time; the locks become necessary if you switch to a `MultiThreadedExecutor`.

## Common problems

| Symptom | Likely cause | What to do |
|---|---|---|
| `Cannot bind socket to port` on launch | An old `asebaros` is still running | Close the old launch with `Ctrl+C` or use `pkill -f asebaros` and try again |
| `thymio_driver` dies with `PackageNotFoundError` | Driver built with `--symlink-install` | Rebuild it without that option: `colcon build --packages-select thymio_driver` |
| `unknown package 'line_follower'` on `colcon build` | Command run outside the workspace root | Run it from the root: `cd ~/thymio_ws` |
| `topic echo` hangs and shows nothing | The driver is not publishing (no `asebaros` or no connection to the robot) | Restart the launch and check `ls /dev/ttyACM*`; use `timeout 3` in the commands |
| Robot does not move at startup | It waits for the first sensor message | Check that `/aseba/events/ground_reflected` is publishing |
| Robot turns the wrong way | The sensor order is swapped (the node assumes `data[0]` is the left sensor) | Cover only the left sensor with the line and check which value drops; swap the two indexes in `ground_callback` if needed |
| Robot zigzags | `kp` too high | Lower `kp` |
| Robot is slow to return to the line | `kp` too low | Raise `kp` |
| Robot keeps spinning or stays still | The sensors do not see the line (wrong calibration or low contrast) | Check the raw values and set `line_value` and `floor_value` again |
