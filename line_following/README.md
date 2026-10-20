# Thymio Line Follower (ROS 2 Jazzy)

Reference code for a line-following robot built on the **Thymio II**, using **ROS 2** and **Aseba**.
The project is meant for learning how to work with the Thymio and ROS 2.

The node reads the two ground sensors (`/ground/left` and `/ground/right`), decides how to move, and
publishes the velocity to `/cmd_vel`. The **sensor detection threshold (`ground.threshold`) can be changed
in real time**, without stopping the robot and without editing the code.

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
- The [`ros-thymio`](#mandatory-change-in-thymio_driver) package (`thymio_driver` and `asebaros`) built in the same workspace
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

## The `ground_threshold` parameter

The node has the `ground_threshold` parameter, which forwards its value to the `ground.threshold` parameter of `thymio_driver`.

```bash
# See the current value
ros2 param get /line_follower ground_threshold

# Change it while the robot is running (use a decimal point)
ros2 param set /line_follower ground_threshold 320.0

# Check what the driver received
ros2 param get /thymio_driver ground.threshold
```

How it works:

- On startup, the node reads the `ground.threshold` the driver is currently using and starts with that value. Starting the node never changes the driver by itself.
- If the driver is not running at startup, a warning is shown and the initial value is `200.0`.
- Accepted range: `0.0` to `1023.0` (scale of the raw values of the Thymio ground sensors).
- Sending the value to the driver does not block the node. The result shows up in the terminal (`Driver ground.threshold updated.` or a refusal warning).
- The value is **not saved**: when the driver restarts, the threshold goes back to the default (`200`).

You can also change it directly on the driver, without the line follower node running:

```bash
ros2 param set /thymio_driver ground.threshold 320
```

## Mandatory change in `thymio_driver`

> **Warning:** the original `thymio_driver` reads `ground.threshold` **only once**, at startup, and
> stores the value in an internal variable. After that, the sensor calculation only uses that copy.
> Result: `ros2 param set` answers `Set parameter successful` and `ros2 param get` shows the new
> value, but **the sensor readings do not change**.
>
> Without the change below, the `ground_threshold` of this project **has no real-time effect**.

The change goes in the `ros-thymio` package, which is **not part of this repository**. Everyone who uses the
project needs to apply it.

**1. Edit the file**

`~/thymio_ws/src/ros-thymio/thymio_driver/thymio_driver/thymio_driver_node.py`

Find the line:

```python
ir_threshold = self.ground_threshold
```

and replace it with:

```python
ir_threshold = self.get_parameter("ground.threshold").value
```

Or from the terminal (makes a backup first):

```bash
F=~/thymio_ws/src/ros-thymio/thymio_driver/thymio_driver/thymio_driver_node.py
cp $F /tmp/thymio_driver_node.py.bak
sed -i 's/ir_threshold = self.ground_threshold/ir_threshold = self.get_parameter("ground.threshold").value/' $F
grep -n "ir_threshold" $F
```

**2. Rebuild the driver (without `--symlink-install`)**

```bash
pkill -f asebaros          # only if an old asebaros is still running
cd ~/thymio_ws
rm -rf build/thymio_driver install/thymio_driver
colcon build --packages-select thymio_driver
source install/setup.bash
```

> Do not use `--symlink-install` on `thymio_driver`: the driver stops starting, with the error
> `PackageNotFoundError: No package metadata was found for thymio-driver`.

**3. Restart the driver launch** and test:

```bash
# With one sensor held over any surface
ros2 param set /thymio_driver ground.threshold 1000
timeout 3 ros2 topic echo /ground/right --field range --once    # expected: 1.0

ros2 param set /thymio_driver ground.threshold 5
timeout 3 ros2 topic echo /ground/right --field range --once    # expected: 0.0
```

If both results are the same, the driver still has the original code.

After any new edit to the driver code, you need to rebuild `thymio_driver` and restart the launch.

## How to choose the threshold (calibration)

The driver publishes **1.0 when the measured reflection is lower than the threshold** and **0.0 when it is higher**.
The threshold must sit between the line value and the background value:

```
dark line  <  THRESHOLD  <  lighter background
```

**1. Measure the raw values** on each surface (`data[0]` = left sensor, `data[1]` = right sensor):

```bash
ros2 topic echo /aseba/events/ground_reflected
```

**2. Compute the average** between the highest line value and the lowest background value, considering both
sensors (the driver uses a single threshold for both).

**3. Apply it and check** with both sensors, one at a time:

```bash
ros2 param set /line_follower ground_threshold 320.0
ros2 topic echo /ground/left --field range
ros2 topic echo /ground/right --field range
```

Expected: **1.0 over the line** and **0.0 over the background**, on both sensors.

Example of raw values measured in our setup (your track may give different numbers):

| Surface | Approximate raw value |
|---|---|
| Air (robot lifted) | 26 |
| Black | 258–273 |
| Brown | 367–390 |
| White | 957 |

The bigger the difference between the line and the background, the more robust the detection. Matte black
electrical tape on white paper gives much more margin than black on brown.

## Common problems

| Symptom | Likely cause | What to do |
|---|---|---|
| `Cannot bind socket to port` on launch | An old `asebaros` is still running | Close the old launch with `Ctrl+C` or use `pkill -f asebaros` and try again |
| `thymio_driver` dies with `PackageNotFoundError` | Driver built with `--symlink-install` | Rebuild without that option (see the driver section) |
| `ros2 param set` works but the reading does not change | Driver without the mandatory change | Apply the change to `thymio_driver` |
| `unknown package 'line_follower'` on `colcon build` | Command run outside the workspace root | Run it from the root: `cd ~/thymio_ws` |
| `topic echo` hangs and shows nothing | The driver is not publishing (no `asebaros` or no connection to the robot) | Restart the launch and check `ls /dev/ttyACM*`; use `timeout 3` in the commands |
| Robot reads 0.0 on any surface | Threshold below the line value | Raise `ground_threshold` (see calibration) |
| Robot keeps spinning or stays still | The sensors do not see the line (wrong threshold or low contrast) | Check the readings and adjust the threshold |
