import threading
from dataclasses import dataclass, replace

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


# ROS topics
LEFT_TOPIC = '/ground/left'
RIGHT_TOPIC = '/ground/right'
CMD_VEL_TOPIC = '/cmd_vel'



# DYNAMIC PARAMETERS

@dataclass
class ControlParams:
    forward_speed: float = 0.10        # Straight-line speed (m/s)
    turn_linear_speed: float = 0.06    # Linear speed while turning (m/s)
    turn_angular_speed: float = 0.8    #Angular speed while turning (rad/s)
    search_delay: float = 0.5          # Time moving straight before starting the search (s)
    search_speed: float = 0.8          # Rotation speed during the search (rad/s)
    line_min: float = 0.5              # Lowest reading value for line (0.0 to 1.0)
    line_max: float = 1.0              # Highest reading value for line (0.0 to 1.0)


# Parameter metadata for validation and limits in ROS 2
PARAM_INFO = {
    'forward_speed':      ('Linear speed on straights (m/s)', 0.0, 0.5),
    'turn_linear_speed':  ('Linear speed on curves (m/s)', 0.0, 0.5),
    'turn_angular_speed': ('Angular speed on curves (rad/s)', 0.0, 3.0),
    'search_delay':       ('Time without line until search starts (s)', 0.0, 10.0),
    'search_speed':       ('Angular speed during search (rad/s)', 0.0, 3.0),
    'line_min':           ('Minimum threshold for line detection', 0.0, 1.0),
    'line_max':           ('Maximum threshold for line detection', 0.0, 1.0),
}



# SENSORS AND MUTEX
class GroundSensor:

    def __init__(self, name: str):
        self.name = name
        self._lock = threading.Lock()
        self._value = 0.0

    def update(self, msg: Range) -> None:
        """Sensor write callback with Mutex."""
        with self._lock:
            self._value = msg.range

    def read(self) -> float:
        """Read protected with Mutex."""
        with self._lock:
            return self._value

#Manages the collection of the robot's sensors
class SensorDatabase:
    def __init__(self):
        self._sensors = {}

    def add(self, sensor: GroundSensor) -> GroundSensor:
        self._sensors[sensor.name] = sensor
        return sensor

    def snapshot(self) -> dict:
        #Captures the instantaneous state of all registered sensors
        return {name: sensor.read() for name, sensor in self._sensors.items()}



# MAIN NODE
class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        #Callback Group -> parallel callbacks
        self.cb_group = ReentrantCallbackGroup()

        #Parameter Management
        self._declare_parameters()
        self._params_lock = threading.Lock()
        self._params = ControlParams(**{
            name: self.get_parameter(name).value for name in PARAM_INFO
        })
        self.add_on_set_parameters_callback(self._on_parameters_change)

        #Sensor Database Initialization
        self.sensor_db = SensorDatabase()
        left = self.sensor_db.add(GroundSensor('left'))
        right = self.sensor_db.add(GroundSensor('right'))

        self.create_subscription(
            Range, LEFT_TOPIC, left.update, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            Range, RIGHT_TOPIC, right.update, 10, callback_group=self.cb_group
        )

        #Velocity Command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        #Control Loop Internal State
        self.last_direction = 0     # 1 = left | -1 = right | 0 = unknown
        self.line_lost_time = None  # Marks the moment the line was lost

        #Control Timer (10 Hz)
        self.create_timer(0.1, self.control_loop, callback_group=self.cb_group)
        self.get_logger().info('LineFollower node (Parameterized with Mutex) started.')

    # PARAMETER METHODS
    #Declares parameters with description and limits in ROS 2
    def _declare_parameters(self):
        defaults = ControlParams()
        for name, (description, low, high) in PARAM_INFO.items():
            self.declare_parameter(
                name,
                getattr(defaults, name),
                ParameterDescriptor(
                    description=description,
                    floating_point_range=[FloatingPointRange(from_value=low, to_value=high, step=0.0)]
                )
            )

    def _on_parameters_change(self, changes) -> SetParametersResult:
        new_values = {c.name: float(c.value) for c in changes if c.name in PARAM_INFO}

        with self._params_lock:
            candidate = replace(self._params, **new_values)

        # Extra integrity validation between parameters
        if candidate.line_min > candidate.line_max:
            reason = 'line_min cannot be greater than line_max'
            self.get_logger().warn(f'Parameter rejected: {reason}')
            return SetParametersResult(successful=False, reason=reason)

        with self._params_lock:
            self._params = candidate
            
        for name, val in new_values.items():
            self.get_logger().info(f'Parameter changed: {name} = {val}')

        return SetParametersResult(successful=True)

    def _get_params(self) -> ControlParams:
        with self._params_lock:
            return self._params

    # CONTROL AND DECISION METHODS
    def control_loop(self):
        params = self._get_params()
        readings = self.sensor_db.snapshot()
        
        left_val = readings['left']
        right_val = readings['right']

        # Checks whether the reading is within the range configured for the line
        left_on = self.is_on_line(left_val, params)
        right_on = self.is_on_line(right_val, params)

        cmd = Twist()

        #Case 1: Only the left sensor is over the line
        if left_on and not right_on:
            self.last_direction = 1
            self.line_lost_time = None
            cmd.linear.x = params.turn_linear_speed
            cmd.angular.z = params.turn_angular_speed

        #Case 2: Only the right sensor is over the line
        elif right_on and not left_on:
            self.last_direction = -1
            self.line_lost_time = None
            cmd.linear.x = params.turn_linear_speed
            cmd.angular.z = -params.turn_angular_speed

        #Case 3: Both sensors are over the line
        elif left_on and right_on:
            self.line_lost_time = None
            cmd.linear.x = params.forward_speed
            cmd.angular.z = 0.0

        #Case 4: No sensor is over the line
        elif not left_on and not right_on:
            now = self.get_clock().now()

            if self.line_lost_time is None:
                self.line_lost_time = now

            elapsed_time = (now - self.line_lost_time).nanoseconds / 1e9

            #Case 4.1: Time limit not reached yet -> keeps moving forward
            if elapsed_time < params.search_delay:
                cmd.linear.x = params.forward_speed
                cmd.angular.z = 0.0

            #Case 4.2: Time limit exceeded -> performs the search
            else:
                if self.last_direction == 1:
                    cmd.linear.x = 0.0
                    cmd.angular.z = params.search_speed

                elif self.last_direction == -1:
                    cmd.linear.x = 0.0
                    cmd.angular.z = -params.search_speed

                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

        #Publishes the message to /cmd_vel
        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def is_on_line(value: float, params: ControlParams) -> bool:
        return params.line_min <= value <= params.line_max

    def stop_robot(self):

        self.cmd_vel_pub.publish(Twist())


#MAIN

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()

    # Multithreaded Executor to manage the parallel callbacks and the Mutex
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
