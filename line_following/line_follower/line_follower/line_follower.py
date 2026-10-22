import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class ThymioController(Node):

    def __init__(self):
        super().__init__('thymio_controller')
        
        # ROS Parameters configuration for speeds, distances, timeout, and PD gains
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('wall_lost_dist', 0.12)
        self.declare_parameter('front_stop_dist', 0.06)
        self.declare_parameter('sensor_timeout', 0.5)
        self.declare_parameter('target_wall_dist', 0.07)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('front_release_dist', 0.08)
        self.declare_parameter('kp', 8.0)
        self.declare_parameter('kd', 0.25)

        # Publisher for velocity commands and subscribers for proximity sensors
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Range, '/proximity/right', self.right_sensor_callback, 10)
        self.create_subscription(Range, '/proximity/center', self.center_sensor_callback, 10)

        # Possible States:
        # FOLLOW_WALL
        # TURN_LEFT
        # SEARCH_WALL

        # Initial State
        self.state = 'FOLLOW_WALL'

        # Control loop timer running at 10Hz (0.1s)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Internal sensor variables and timestamps for freshness check
        self.center_dist = None # Distance received from the center sensor
        self.right_dist = None  # Distance received from the right sensor
        self.center_stamp = None
        self.right_stamp = None

        # PD controller variables
        self.previous_error = 0.0
        self.derivative_filtered = 0.0

        # SEARCH_WALL parameters: move forward 15 cm then turn 90 degrees to the right
        self.search_phase = 'ADVANCE'   # ADVANCE -> TURN
        self.search_phase_start = None
        self.search_finished = False    # True when the complete search maneuver has finished

        self.search_advance_dist = 0.15              # m
        self.search_advance_speed = 0.10             # m/s
        self.search_advance_duration = (
            self.search_advance_dist / self.search_advance_speed)  # 1.5 s

        self.search_turn_angle = math.pi / 2         # rad (90 degrees)
        self.search_turn_speed = -0.8                # rad/s (negative = right turn)
        self.search_turn_duration = (
            self.search_turn_angle / abs(self.search_turn_speed))  # ~1.96 s

        self.last_control_time = self.get_clock().now()


    def _read_range(self, msg: Range):
        """Convert missing or invalid range readings into None."""
        value = msg.range
        if not math.isfinite(value) or value <= 0.0:
            return None
        return value        

    def center_sensor_callback(self, msg: Range):
        """Store center sensor reading and the timestamp when it occurred."""
        self.center_dist = self._read_range(msg)
        self.center_stamp = self.get_clock().now()

    def right_sensor_callback(self, msg: Range):
        """Store right sensor reading and the timestamp when it occurred."""
        self.right_dist = self._read_range(msg)
        self.right_stamp = self.get_clock().now()

    def reset_search(self):
        """Reset the SEARCH_WALL maneuver (move 15 cm + turn 90 degrees)."""
        self.search_phase = 'ADVANCE'
        self.search_phase_start = None
        self.search_finished = False

    def change_state(self, new_state):
        """Handle state transitions and log changes."""
        if new_state != self.state:
            self.state = new_state
            if new_state == 'SEARCH_WALL':
                self.reset_search()
            self.get_logger().info(f'State: {self.state}')

    def _sensor_is_fresh(self, stamp, timeout):
        """Verify if the last sensor reading is within the acceptable timeout limit."""
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= timeout


    @staticmethod
    def clamp(value, low, high):
        """Clamp a value between a lower and upper bound."""
        return max(low, min(high, value))

    def control_loop(self):
        """Main control loop handling FSM transitions and velocity calculations."""
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        if dt <= 0.0:
            dt = 0.1

        timeout = float(self.get_parameter('sensor_timeout').value)

        center_fresh = self._sensor_is_fresh(self.center_stamp, timeout)
        right_fresh = self._sensor_is_fresh(self.right_stamp, timeout)

        front = self.center_dist if center_fresh else None
        right = self.right_dist if right_fresh else None
        
        front_stop = float(self.get_parameter('front_stop_dist').value)
        front_release = float(self.get_parameter('front_release_dist').value)
        wall_lost = float(self.get_parameter('wall_lost_dist').value)
        target_great = float(self.get_parameter('target_wall_dist').value)
        
        front_blocked = front is not None and front <= front_stop
        front_clear = front is None or front >= front_release
        wall_detected = right is not None and right < wall_lost

        # FINITE STATE MACHINE (FSM) LOGIC
        # Priority 1: Prevent frontal collisions by turning left if blocked.
        if self.state == 'TURN_LEFT':
            # Keep turning until the front is clear and right wall status is evaluated
            if front_clear:
                self.change_state(
                    'FOLLOW_WALL' if wall_detected else 'SEARCH_WALL')
        elif front_blocked:
            self.change_state('TURN_LEFT')
        elif self.state == 'SEARCH_WALL' and not self.search_finished:
            # Do not interrupt the search maneuver (move 15 cm + turn 90) halfway through.
            pass
        elif not wall_detected:
            if self.state == 'SEARCH_WALL':
                # Maneuver completed and wall still not found: repeat search.
                self.reset_search()
            self.change_state('SEARCH_WALL')
        else:
            self.change_state('FOLLOW_WALL')

        cmd = Twist()
        max_speed = float(self.get_parameter('linear_speed').value)
        max_angular = abs(float(
            self.get_parameter('max_angular_speed').value))

        # BEHAVIOR EXECUTION PER STATE
        if self.state == 'TURN_LEFT':
            # Rotate in place until the front obstacle is cleared.
            cmd.linear.x = 0.0
            cmd.angular.z = min(1.0, max_angular)
                                
        elif self.state == 'SEARCH_WALL':
            # Phase 1: Move forward 15 cm. Phase 2: Turn 90 degrees to the right.
            if self.search_phase_start is None:
                self.search_phase_start = now
            elapsed = (now - self.search_phase_start).nanoseconds / 1e9

            if self.search_phase == 'ADVANCE':
                cmd.linear.x = self.search_advance_speed
                cmd.angular.z = 0.0
                if elapsed >= self.search_advance_duration:
                    self.search_phase = 'TURN'
                    self.search_phase_start = now

            elif self.search_phase == 'TURN':
                cmd.linear.x = 0.0
                cmd.angular.z = self.search_turn_speed
                if elapsed >= self.search_turn_duration:
                    # Maneuver finished: let the FSM re-evaluate conditions.
                    self.search_finished = True
                    cmd.angular.z = 0.0

       
        elif self.state == 'FOLLOW_WALL':
            # Safety check: ensure right sensor data is available before running PD controller
            if right is None:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                return

            target = float(self.get_parameter('target_wall_dist').value)
            kp = float(self.get_parameter('kp').value)
            kd = float(self.get_parameter('kd').value)

            # Positive error = wall is too far; needs to steer to the right.
            error = right - target
            derivative = (error - self.previous_error) / dt
            
            # Simple low-pass filter to reduce sensor noise and oscillations.
            alpha = 0.25
            self.derivative_filtered = (
                alpha * derivative +
                (1.0 - alpha) * self.derivative_filtered
            )
            self.previous_error = error

            # ROS convention: positive angular velocity turns left, negative turns right.
            angular = -(kp * error + kd * self.derivative_filtered)
            cmd.angular.z = self.clamp(angular, -max_angular, max_angular)

            # Reduce linear speed based on proximity to front obstacles.
            if front is None:
                front_factor = 1.0
            else:
                slow_dist = max(front_release, front_stop + 0.01)
                front_factor = self.clamp(
                    (front - front_stop) / (slow_dist - front_stop), 0.0, 1.0)

            # Sharper turns also reduce linear speed for stability.
            turn_factor = 1.0 - 0.5 * min(1.0, abs(cmd.angular.z) / max(max_angular, 1e-6))
            cmd.linear.x = max_speed * min(front_factor, turn_factor)
            
        else:
            # Unexpected state: stop safely.
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ThymioController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Attempt to stop the robot safely upon node shutdown.
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()