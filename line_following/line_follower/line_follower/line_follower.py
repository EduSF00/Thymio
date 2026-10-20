import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import (
    FloatingPointRange, Parameter as ParameterMsg, ParameterDescriptor,
    ParameterType, ParameterValue, SetParametersResult)
from rcl_interfaces.srv import GetParameters, SetParameters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


# Thymio driver node and the parameter that defines the ground threshold
DRIVER_NODE = '/thymio_driver'
DRIVER_THRESHOLD_PARAM = 'ground.threshold'
DRIVER_TIMEOUT = 2.0               # Seconds to wait for the driver when starting

# Used only if the driver is not running when this node starts
FALLBACK_THRESHOLD = 200.0


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # ------------------------------------------------------------------
        # PARAMETER: ground_threshold
        # (declared first, before the control timer exists, so the robot
        #  does not move while the node waits for the driver to answer)
        # ------------------------------------------------------------------

        # Clients used to read and change parameters of the Thymio driver node
        self.driver_get_client = self.create_client(
            GetParameters, f'{DRIVER_NODE}/get_parameters')
        self.driver_set_client = self.create_client(
            SetParameters, f'{DRIVER_NODE}/set_parameters')

        # Type used by the driver parameter (integer or double)
        self.driver_threshold_is_double = False

        # Starts with the value the driver is using right now, so starting
        # this node never changes the driver by itself
        self.declare_parameter(
            'ground_threshold',
            self.read_driver_threshold(),
            ParameterDescriptor(
                description='Ground threshold of thymio_driver (forwarded to ground.threshold)',
                # Thymio raw ground values go from 0 to 1023
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1023.0, step=0.0)]
            )
        )

        # Called every time someone runs: ros2 param set /line_follower ground_threshold ...
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Publishes to the robot velocity topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Last values received from the sensors
        self.left_sensor = 0.0
        self.right_sensor = 0.0

        # Stores the last direction in which the line was detected
        # 1  = left
        # -1 = right
        # 0  = unknown
        self.last_direction = 0

        # Stores how long both sensors have been off the line
        self.line_lost_time = None

        # Maximum time without detecting the line before starting the search
        self.search_delay = 0.5

        # Rotation speed during the search
        self.search_speed = 0.8

        # Subscription to the left ground sensor
        self.left_sub = self.create_subscription(Range, '/ground/left', self.left_callback, 10)

        # Subscription to the right ground sensor
        self.right_sub = self.create_subscription(Range, '/ground/right', self.right_callback, 10)

        # Timer running the control loop 10 times per second (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    # ----------------------------------------------------------------------
    # GROUND THRESHOLD PARAMETER
    # ----------------------------------------------------------------------
    def read_driver_threshold(self):
        """Reads ground.threshold from thymio_driver (falls back if the driver is off)."""

        if not self.driver_get_client.wait_for_service(timeout_sec=DRIVER_TIMEOUT):
            self.get_logger().warn(
                f'{DRIVER_NODE} not found. ground_threshold starts at {FALLBACK_THRESHOLD} '
                'and is only sent to the driver when you change it.')
            return FALLBACK_THRESHOLD

        request = GetParameters.Request()
        request.names = [DRIVER_THRESHOLD_PARAM]
        future = self.driver_get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=DRIVER_TIMEOUT)

        if not future.done() or not future.result().values:
            self.get_logger().warn('Could not read ground.threshold from the driver.')
            return FALLBACK_THRESHOLD

        value = future.result().values[0]

        # The driver may store the threshold as double or as integer
        if value.type == ParameterType.PARAMETER_DOUBLE:
            self.driver_threshold_is_double = True
            return float(value.double_value)

        if value.type == ParameterType.PARAMETER_INTEGER:
            self.driver_threshold_is_double = False
            return float(value.integer_value)

        self.get_logger().warn(f'{DRIVER_THRESHOLD_PARAM} is not set in the driver.')
        return FALLBACK_THRESHOLD

    def parameters_callback(self, changes):
        """Called by ROS BEFORE a parameter changes. Returning False rejects the change."""

        for change in changes:
            if change.name == 'ground_threshold':

                # Rejects the change if the driver is not running
                if not self.driver_set_client.service_is_ready():
                    reason = f'{DRIVER_NODE} is not available'
                    self.get_logger().warn(f'Parameter rejected: {reason}')
                    return SetParametersResult(successful=False, reason=reason)

                self.send_threshold_to_driver(change.value)

        return SetParametersResult(successful=True)

    def send_threshold_to_driver(self, threshold):
        """Sends the new threshold to thymio_driver (does not block the node)."""

        # Builds the value using the same type the driver uses
        value = ParameterValue()
        if self.driver_threshold_is_double:
            value.type = ParameterType.PARAMETER_DOUBLE
            value.double_value = float(threshold)
        else:
            value.type = ParameterType.PARAMETER_INTEGER
            value.integer_value = int(round(threshold))

        request = SetParameters.Request()
        request.parameters = [ParameterMsg(name=DRIVER_THRESHOLD_PARAM, value=value)]

        # Sends the request; driver_answer_callback runs when the driver answers
        future = self.driver_set_client.call_async(request)
        future.add_done_callback(self.driver_answer_callback)

    def driver_answer_callback(self, future):
        """Shows in the terminal whether the driver accepted the new threshold."""

        result = future.result().results[0]

        if result.successful:
            self.get_logger().info(f'Driver {DRIVER_THRESHOLD_PARAM} updated.')
        else:
            self.get_logger().warn(f'Driver refused the threshold: {result.reason}')

    # ----------------------------------------------------------------------
    # SENSORS AND CONTROL
    # ----------------------------------------------------------------------
    def left_callback(self, msg):
        self.left_sensor = msg.range

    def right_callback(self, msg):
        self.right_sensor = msg.range

    def control_loop(self):
        cmd = Twist()

        left = self.left_sensor
        right = self.right_sensor


        #Case 1. LEFT SENSOR OVER THE LINE
        if left == 1.0 and right == 0.0:

            # The line is on the left
            self.last_direction = 1

            # The line was detected, so reset the timer
            self.line_lost_time = None

            cmd.linear.x = 0.06
            cmd.angular.z = 0.8

        #Case 2. RIGHT SENSOR OVER THE LINE
        elif left == 0.0 and right == 1.0:

            # The line is on the right
            self.last_direction = -1

            # The line was detected, so reset the timer
            self.line_lost_time = None

            cmd.linear.x = 0.06
            cmd.angular.z = -0.8

        #Case 3. BOTH SENSORS OVER THE LINE
        elif left == 1.0 and right == 1.0:

            # The line was detected again
            self.line_lost_time = None

            cmd.linear.x = 0.10
            cmd.angular.z = 0.0

        #Case 4. NO SENSOR OVER THE LINE
        elif left == 0.0 and right == 0.0:

            # First moment when the line is lost
            if self.line_lost_time is None:
                self.line_lost_time = self.get_clock().now()

            # Calculate how long we have been without detecting the line
            elapsed_time = (self.get_clock().now() - self.line_lost_time).nanoseconds / 1e9

            #Case 4.1. Not enough time has passed:
            # continue moving forward
            if elapsed_time < self.search_delay:

                cmd.linear.x = 0.10
                cmd.angular.z = 0.0

            #Case 4.2. Enough time has passed:
            # start searching for the line
            else:

                # IF the last time we saw the line was on the left
                if self.last_direction == 1:

                    cmd.linear.x = 0.0
                    cmd.angular.z = self.search_speed

                # IF the last time we saw the line was on the right
                elif self.last_direction == -1:

                    cmd.linear.x = 0.0
                    cmd.angular.z = -self.search_speed

                # IF we do not have a known direction yet
                else:

                    # remain stopped
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

        #Case 5. UNEXPECTED VALUE
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publishes the command
        self.cmd_vel_pub.publish(cmd)


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
