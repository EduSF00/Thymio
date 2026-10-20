import threading
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from asebaros_msgs.msg import Event
from geometry_msgs.msg import Twist


# Parameters that can be changed while the robot is running
PARAM_INFO = {
    'kp':                (0.8,   'Proportional gain (rad/s of turn per unit of error)', 0.0, 5.0),
    'forward_speed':     (0.10,  'Linear speed with the line centered (m/s)', 0.0, 0.5),
    'turn_linear_speed': (0.06,  'Linear speed when the error is maximum (m/s)', 0.0, 0.5),
    'line_value':        (250.0, 'Raw sensor reading over the line', 0.0, 1023.0),
    'floor_value':       (950.0, 'Raw sensor reading over the floor', 0.0, 1023.0),
    'lost_level':        (0.3,   'Darkness (0 to 1) below which the line is considered lost', 0.0, 1.0),
}

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # Inicializa o Mutex (Lock) para proteger o acesso a variáveis compartilhadas
        self.lock = threading.Lock()

        # Declares the parameters with description and limits
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

        # Last raw readings from the ground sensors (None until the first message arrives)
        self.left_raw = None
        self.right_raw = None

        # Stores the last direction in which the line was detected
        self.last_direction = 0

        # Errors smaller than this are ignored when remembering the last direction
        self.direction_deadband = 0.1

        # Stores how long both sensors have been off the line
        self.line_lost_time = None

        # Maximum time without detecting the line before starting the search
        self.search_delay = 0.5

        # Rotation speed during the search
        self.search_speed = 0.8

        # Subscription to the raw ground sensors (data[0] = left, data[1] = right)
        self.ground_sub = self.create_subscription(
            Event, '/aseba/events/ground_reflected', self.ground_callback, 10)

        # Timer running the control loop 10 times per second (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def param(self, name):
        """Returns the current value of a parameter (always up to date)."""
        return self.get_parameter(name).value

    def parameters_callback(self, changes):
        """Called by ROS BEFORE a parameter changes. Returning False rejects the change."""
        values = {c.name: c.value for c in changes}
        line = values.get('line_value', self.param('line_value'))
        floor = values.get('floor_value', self.param('floor_value'))

        if line == floor:
            reason = 'line_value and floor_value cannot be equal'
            self.get_logger().warn(f'Parameter rejected: {reason}')
            return SetParametersResult(successful=False, reason=reason)

        return SetParametersResult(successful=True)

    def ground_callback(self, msg):
        """Atualiza os valores dos sensores protegidos por mutex."""
        if len(msg.data) >= 2:
            with self.lock:
                self.left_raw = msg.data[0]
                self.right_raw = msg.data[1]

    def darkness(self, raw):
        """Converts a raw reading to 0.0 (floor) ... 1.0 (line)."""
        line = self.param('line_value')
        floor = self.param('floor_value')

        value = (floor - raw) / (floor - line)
        return max(0.0, min(1.0, value))

    def control_loop(self):
        # Lê os dados de forma segura com o mutex
        with self.lock:
            left_val = self.left_raw
            right_val = self.right_raw

        # No sensor data yet: stay stopped
        if left_val is None or right_val is None:
            self.cmd_vel_pub.publish(Twist())
            return

        left_dark = self.darkness(left_val)
        right_dark = self.darkness(right_val)

        # Error: positive = line on the left | negative = line on the right | 0 = centered
        error = left_dark - right_dark

        # At least one sensor sees the line -> proportional control
        if max(left_dark, right_dark) >= self.param('lost_level'):
            cmd = self.follow_line(error)
        else:
            cmd = self.search_line()

        # Publishes the command
        self.cmd_vel_pub.publish(cmd)

    def follow_line(self, error):
        """Proportional (P) control: the bigger the error, the stronger the turn."""
        cmd = Twist()

        with self.lock:
            # The line was detected, so reset the timer
            self.line_lost_time = None

            # Remembers on which side the line is (ignores very small errors)
            if abs(error) > self.direction_deadband:
                self.last_direction = 1 if error > 0 else -1

        forward = self.param('forward_speed')
        turn = self.param('turn_linear_speed')

        cmd.angular.z = self.param('kp') * error
        cmd.linear.x = forward - (forward - turn) * abs(error)

        return cmd

    def search_line(self):
        """No sensor over the line: keeps moving forward for a while, then searches."""
        cmd = Twist()

        with self.lock:
            # First moment when the line is lost
            if self.line_lost_time is None:
                self.line_lost_time = self.get_clock().now()

            # Calculate how long we have been without detecting the line
            elapsed_time = (self.get_clock().now() - self.line_lost_time).nanoseconds / 1e9

            # Captura a última direção de forma segura
            direction = self.last_direction

        # Not enough time has passed: continue moving forward
        if elapsed_time < self.search_delay:
            cmd.linear.x = self.param('forward_speed')
        else:
            cmd.angular.z = self.search_speed * direction

        return cmd


def main(args=None):
    rclpy.init(args=args)

    node = LineFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()