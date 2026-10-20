import threading                       
from typing import Dict, Tuple, Optional   #Type hints 
import rclpy
from rclpy.node import Node

# FloatingPointRange / ParameterDescriptor: describe a parameter and its limits
# SetParametersResult: the answer ("accepted" or "rejected") of the parameter callback
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from asebaros_msgs.msg import Event    # Aseba event message that carries the raw sensor values
from geometry_msgs.msg import Twist    # Velocity command (linear and angular speed)


# Parameters that can be changed while the robot is running
# name: (default value, description, lowest value, highest value)
# Refuses any value outside the limits
PARAM_INFO = {
    'kp':                (0.8,   'Proportional gain (rad/s of turn per unit of error)', 0.0, 5.0),
    'forward_speed':     (0.10,  'Linear speed with the line centered (m/s)', 0.0, 0.5),
    'turn_linear_speed': (0.06,  'Linear speed when the error is maximum (m/s)', 0.0, 0.5),
    'line_value':        (250.0, 'Raw sensor reading over the line', 0.0, 1023.0),
    'floor_value':       (950.0, 'Raw sensor reading over the floor', 0.0, 1023.0),
    'lost_level':        (0.3,   'Darkness (0 to 1) below which the line is considered lost', 0.0, 1.0),
}


class GroundSensor:
    """Represents an individual ground sensor with its raw value and readiness state"""
    def __init__(self, name: str):
        self.name = name                            # 'left' or 'right'
        self.raw_value: Optional[float] = None      # None = no message received yet

    def update(self, value: float):
        # Stores the newest raw reading
        self.raw_value = value

    def read(self) -> Tuple[Optional[float], bool]:
        """Returns (current_value, has_received_data)"""
        return self.raw_value, self.raw_value is not None


class SensorDatabase:
    """Place where all sensors are registered"""
    def __init__(self):
        self._sensors: Dict[str, GroundSensor] = {}   # name -> sensor object
        # Mutex of the database: only one piece of code at a time can use the dictionary.
        # NOTE: it covers the dictionary access (add/get/all_ready/snapshot). The call to
        # sensor.update() in ground_callback happens after get() returns, outside this lock.
        self.lock = threading.Lock()

    def add(self, sensor: GroundSensor) -> GroundSensor:
        # Registers a sensor in the database
        with self.lock:                       # 'with' locks on entering and unlocks on leaving
            self._sensors[sensor.name] = sensor
        return sensor

    def get(self, name: str) -> GroundSensor:
        # Finds a sensor by its name ('left' or 'right')
        with self.lock:
            return self._sensors[name]

    def all_ready(self) -> bool:
        """True when ALL sensors have already sent at least one value."""
        with self.lock:
            if not self._sensors:
                return False
            return all(sensor.read()[1] for sensor in self._sensors.values())

    def snapshot(self) -> Dict[str, float]:
        """Snapshot of current values: {'left': 250.0, 'right': 950.0}"""
        # Only sensors that already received data are included
        with self.lock:
            return {name: sensor.read()[0] for name, sensor in self._sensors.items() if sensor.read()[1]}


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # Initialize the Sensor Database and register the left and right sensors
        # the control loop asks the database for the values instead of storing them itself
        self.sensor_db = SensorDatabase()
        self.sensor_db.add(GroundSensor('left'))
        self.sensor_db.add(GroundSensor('right'))

        # Mutex to protect the robot's navigation state (last_direction and line_lost_time)
        self.lock = threading.Lock()

        # Declares the parameters with description and limits
        # One loop declares all of them, using the PARAM_INFO table
        for name, (default, description, low, high) in PARAM_INFO.items():
            self.declare_parameter(
                name,
                default,
                ParameterDescriptor(
                    description=description,
                    floating_point_range=[FloatingPointRange(from_value=low, to_value=high, step=0.0)]
                )
            )

        # Called every time someone runs: ros2 param set /line_follower <name> <value>
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Publishes to the robot velocity topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscription to the raw ground sensors
        # The message has a list called "data": data[0] = left sensor, data[1] = right sensor
        self.ground_sub = self.create_subscription(
            Event, '/aseba/events/ground_reflected', self.ground_callback, 10
        )

        # Stores the last direction in function of line detection
        # 1 = line was last seen on the left | -1 = on the right | 0 = unknown
        self.last_direction = 0

        # Errors smaller than this are ignored when remembering the last direction
        self.direction_deadband = 0.1

        # Moment when the line was lost (None while the line is visible)
        self.line_lost_time = None

        # Time going straight after losing the line, before starting to search (s)
        self.search_delay = 0.5

        # Rotation speed while searching for the line (rad/s)
        self.search_speed = 0.8

        # Timer running the control loop 10 times per second (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def param(self, name):
        """Returns the current value of a parameter (always up to date)"""
        # Reading the parameter every time is what makes changes take effect immediately
        return self.get_parameter(name).value

    def parameters_callback(self, changes):
        """Called by ROS BEFORE a parameter changes. Returning False rejects the change"""
        # Uses the new value if it is being changed now, otherwise the current one
        values = {c.name: c.value for c in changes}
        line = values.get('line_value', self.param('line_value'))
        floor = values.get('floor_value', self.param('floor_value'))

        # Equal values would make the normalization divide by zero
        if line == floor:
            reason = 'line_value and floor_value cannot be equal'
            self.get_logger().warn(f'Parameter rejected: {reason}')
            return SetParametersResult(successful=False, reason=reason)

        return SetParametersResult(successful=True)

    def ground_callback(self, msg):
        """Updates the sensor database with the received values"""
        # Runs every time a new reading arrives (about 10 times per second)
        if len(msg.data) >= 2:
            self.sensor_db.get('left').update(float(msg.data[0]))
            self.sensor_db.get('right').update(float(msg.data[1]))

    def calculate_darkness(self, raw, line, floor):
        """Converts a raw reading to 0.0 (floor) and 1.0 (line)"""
        # Extra protection against dividing by zero
        if line == floor:
            return 0.0
        # Linear scale: raw == floor gives 0.0, raw == line gives 1.0
        # It also works when the line is lighter than the floor (line > floor)
        value = (floor - raw) / (floor - line)
        # Keeps the result between 0.0 and 1.0 (readings outside the range are clipped)
        return max(0.0, min(1.0, value))

    def control_loop(self):
        # Runs 10 times per second: reads the sensors, decides and publishes the command

        # Checks if all sensors have sent data using the SensorDatabase
        # (until the first message arrives, the robot stays stopped)
        if not self.sensor_db.all_ready():
            self.cmd_vel_pub.publish(Twist())     # Empty Twist = zero speed
            return

        # Gets the updated sensor snapshot cleanly
        snap = self.sensor_db.snapshot()
        left_val = snap.get('left')
        right_val = snap.get('right')

        # Extra safety: stays stopped if any value is still missing
        if left_val is None or right_val is None:
            self.cmd_vel_pub.publish(Twist())
            return

        # Reads the calibration parameters (they can change while the robot runs)
        line_val = self.param('line_value')
        floor_val = self.param('floor_value')

        # Converts each raw reading to 0.0 (floor) and 1.0 (line)
        left_dark = self.calculate_darkness(left_val, line_val, floor_val)
        right_dark = self.calculate_darkness(right_val, line_val, floor_val)

        # Error: positive = line on the left | negative = line on the right | 0 = centered
        error = left_dark - right_dark

        # At least one sensor sees the line -> proportional control
        if max(left_dark, right_dark) >= self.param('lost_level'):
            cmd = self.follow_line(error)
        # No sensor sees the line -> search
        else:
            cmd = self.search_line()

        # Publishes the command
        self.cmd_vel_pub.publish(cmd)

    def follow_line(self, error):
        """Proportional (P) control: the bigger the error, the stronger the turn"""
        cmd = Twist()

        # Protected by the mutex because last_direction and line_lost_time are shared state
        with self.lock:
            # The line is visible, so the "line lost" timer is reset
            self.line_lost_time = None
            # Remembers on which side the line is (ignores very small errors)
            if abs(error) > self.direction_deadband:
                self.last_direction = 1 if error > 0 else -1

        forward = self.param('forward_speed')
        turn = self.param('turn_linear_speed')

        # P control: turn speed proportional to the error
        # (error +1 with kp 0.8 turns left at 0.8 rad/s; error 0 goes straight)
        cmd.angular.z = self.param('kp') * error
        # The bigger the error, the slower the robot (forward_speed -> turn_linear_speed)
        cmd.linear.x = forward - (forward - turn) * abs(error)

        return cmd

    def search_line(self):
        """No sensor over the line: keeps moving forward for a while, then searches"""
        cmd = Twist()

        with self.lock:
            # First moment when the line is lost: starts the timer
            if self.line_lost_time is None:
                self.line_lost_time = self.get_clock().now()

            # How long the line has been lost (s)
            elapsed_time = (self.get_clock().now() - self.line_lost_time).nanoseconds / 1e9
            # Copies the direction to use it after leaving the lock
            direction = self.last_direction

        # Not enough time has passed: continue moving forward
        if elapsed_time < self.search_delay:
            cmd.linear.x = self.param('forward_speed')
        # Enough time has passed: rotate towards the side where the line was last seen
        # (direction is 0 if unknown, so the robot stays stopped)
        else:
            cmd.angular.z = self.search_speed * direction

        return cmd


def main(args=None):
    rclpy.init(args=args)

    node = LineFollower()

    try:
        rclpy.spin(node)        # Keeps the node running until Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()