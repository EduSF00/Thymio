import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class ThymioController(Node):

    def __init__(self):
        super().__init__('thymio_controller')

        # Parameters (can be changed at runtime through ROS 2)
        self.declare_parameter('linear_speed', 0.10)        # Max forward speed (m/s)
        self.declare_parameter('wall_lost_dist', 0.10)      # Right distance (m) from which the wall is considered lost
        self.declare_parameter('front_stop_dist', 0.06)     # Front distance (m) at or below which the front is blocked
        self.declare_parameter('sensor_timeout', 0.5)       # Max age (s) of a reading before it is considered stale
        self.declare_parameter('target_wall_dist', 0.05)    # Ideal distance (m) to the right wall
        self.declare_parameter('max_angular_speed', 1.0)    # Max angular speed (rad/s)
        self.declare_parameter('front_release_dist', 0.08)  # Front distance (m) at or above which the front is clear again
        self.declare_parameter('kp', 8.0)                   # Proportional gain: how strongly the turn reacts to the distance error
        self.declare_parameter('kd', 0.25)                  # Derivative gain: damps the turn using how fast the error changes
        self.declare_parameter('safe_wall_dist', 0.04)         # Below this: stop and move away from the wall
        self.declare_parameter('safe_wall_hysteresis', 0.01)   # Margin to leave the recovery mode
        self.declare_parameter('recover_angular_speed', 0.6)   # Turn speed (rad/s) to the left while recovering

        # Publishes to cmd_vel and subscribes to the center and right sensor topics
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Range, '/proximity/right', self.right_sensor_callback, 10)
        self.create_subscription(Range, '/proximity/center', self.center_sensor_callback, 10)

        # Possible states:
        #   FOLLOW_WALL
        #   TURN_LEFT
        #   SEARCH_WALL

        # Initial state
        self.state = 'FOLLOW_WALL'

        # Control loop runs at 10 Hz (every 0.1 s)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Initial variables
        self.center_dist = None   # Last distance received from the center sensor
        self.right_dist = None    # Last distance received from the right sensor
        self.center_stamp = None  # Time at which the center sensor was last read
        self.right_stamp = None   # Time at which the right sensor was last read
        self.wall_too_close = False  # True while in the "too close to the wall" recovery mode

        # PD controller memory
        self.previous_error = 0.0       # Error from the previous cycle (for the derivative)
        self.derivative_filtered = 0.0  # Low-pass filtered derivative

        # SEARCH_WALL parameters: drive 12 cm forward, then turn 90 degrees to the right
        self.search_phase = 'ADVANCE'   # ADVANCE -> TURN
        self.search_phase_start = None  # Time at which the current phase started
        self.search_finished = False    # True when the whole maneuver is done

        self.search_advance_dist = 0.12   # Distance driven during SEARCH_WALL (m)
        self.search_advance_speed = 0.10  # m/s
        self.search_advance_duration = (self.search_advance_dist / self.search_advance_speed)  # 1.2 s

        self.search_turn_angle = math.pi / 2  # rad (90 degrees)
        self.search_turn_speed = -0.8         # rad/s (negative = turn right)
        self.search_turn_duration = (self.search_turn_angle / abs(self.search_turn_speed))  # ~1.96 s

        self.last_control_time = self.get_clock().now()

    def _read_range(self, msg: Range):
        """Converts missing/invalid readings into None."""
        value = msg.range
        if not math.isfinite(value) or value <= 0.0:
            return None
        return value

    # Sensor callbacks: store the reading and the time it was received
    def center_sensor_callback(self, msg: Range):
        self.center_dist = self._read_range(msg)
        self.center_stamp = self.get_clock().now()

    def right_sensor_callback(self, msg: Range):
        self.right_dist = self._read_range(msg)
        self.right_stamp = self.get_clock().now()

    def reset_search(self):
        """Restarts the SEARCH_WALL maneuver (drive 12 cm + turn 90 degrees)."""
        self.search_phase = 'ADVANCE'
        self.search_phase_start = None
        self.search_finished = False

    def change_state(self, new_state):
        """Switches state (does nothing if it is already in that state)."""
        if new_state != self.state:
            self.state = new_state
            # Entering SEARCH_WALL always starts the maneuver from the beginning
            if new_state == 'SEARCH_WALL':
                self.reset_search()
            self.get_logger().info(f'State: {self.state}')

    def _sensor_is_fresh(self, stamp, timeout):
        """Checks that the last reading is not older than the allowed timeout."""
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= timeout

    @staticmethod
    def clamp(value, low, high):
        """Limits a value to the range [low, high]."""
        return max(low, min(high, value))

    def control_loop(self):

        # Time elapsed since the previous cycle (used by the PD derivative)
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        if dt <= 0.0:
            dt = 0.1

        timeout = float(self.get_parameter('sensor_timeout').value)

        center_fresh = self._sensor_is_fresh(self.center_stamp, timeout)
        right_fresh = self._sensor_is_fresh(self.right_stamp, timeout)

        # Stale or invalid readings become None so they never drive the robot
        # (the sensor may report -1.0 when nothing is detected)
        front = self.center_dist if center_fresh else None
        right = self.right_dist if right_fresh else None

        front_stop = float(self.get_parameter('front_stop_dist').value)
        front_release = float(self.get_parameter('front_release_dist').value)
        wall_lost = float(self.get_parameter('wall_lost_dist').value)
        target_great = float(self.get_parameter('target_wall_dist').value)

        # Conditions used by the state machine
        front_blocked = front is not None and front <= front_stop    # obstacle right in front
        front_clear = front is None or front >= front_release       # front is free (release > stop gives hysteresis)
        wall_detected = right is not None and right < wall_lost      # wall close enough on the right
        # Right wall is between the target distance and the "lost" distance
        target_great_dist = right is not None and (right < wall_lost and right > target_great)

        # ---------- STATE MACHINE ----------
        # Priority is to avoid a frontal collision.
        if self.state == 'TURN_LEFT':
            # Keeps turning until the front is clear and the right wall is at a good distance
            if front_clear and target_great_dist:
                self.change_state(
                    'FOLLOW_WALL' if wall_detected else 'SEARCH_WALL')
        elif front_blocked:
            self.change_state('TURN_LEFT')
        elif self.state == 'SEARCH_WALL' and not self.search_finished:
            # Does not interrupt the maneuver (drive 12 cm + turn 90) halfway.
            pass
        elif not wall_detected:
            if self.state == 'SEARCH_WALL':
                # Maneuver finished and no wall was found yet: repeat it.
                self.reset_search()
            self.change_state('SEARCH_WALL')
        else:
            self.change_state('FOLLOW_WALL')

        # ---------- COMMAND FOR THE CURRENT STATE ----------
        cmd = Twist()
        max_speed = float(self.get_parameter('linear_speed').value)
        max_angular = abs(float(
            self.get_parameter('max_angular_speed').value))

        if self.state == 'TURN_LEFT':
            # Spins in place until the front is clear.
            cmd.linear.x = 0.0
            cmd.angular.z = min(1.0, max_angular)

        elif self.state == 'SEARCH_WALL':
            # Phase 1: drive 12 cm forward. Phase 2: turn 90 degrees to the right.
            # The maneuver is timed (open loop): the elapsed time decides when each phase ends.
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
                    # Maneuver finished: the state machine reevaluates on the next cycle.
                    self.search_finished = True
                    cmd.angular.z = 0.0

        elif self.state == 'FOLLOW_WALL':
            target = float(self.get_parameter('target_wall_dist').value)
            kp = float(self.get_parameter('kp').value)
            kd = float(self.get_parameter('kd').value)
            safe = float(self.get_parameter('safe_wall_dist').value)
            hyst = float(self.get_parameter('safe_wall_hysteresis').value)
            recover_w = abs(float(self.get_parameter('recover_angular_speed').value))

            # Hysteresis: enters recovery below 'safe',
            # only leaves it once the distance exceeds 'safe + hyst'.
            if self.wall_too_close:
                if right >= safe + hyst:
                    self.wall_too_close = False
            elif right < safe:
                self.wall_too_close = True

            if self.wall_too_close:
                # Recovery: stopped, turning left (moving away from the wall).
                cmd.linear.x = 0.0
                cmd.angular.z = min(recover_w, max_angular)
                # Keeps the PD "clean" for when it resumes.
                self.previous_error = right - target
                self.derivative_filtered = 0.0
            else:
                # Normal PD control.
                # Positive error = wall is far away; the robot must turn right.
                error = right - target
                derivative = (error - self.previous_error) / dt

                # Simple low-pass filter to reduce sensor noise/oscillations.
                alpha = 0.25
                self.derivative_filtered = (
                    alpha * derivative +
                    (1.0 - alpha) * self.derivative_filtered
                )
                self.previous_error = error

                # ROS: positive angular turns left, negative turns right.
                # P term: reacts to the current error. D term: damps based on how fast the error changes.
                angular = -(kp * error + kd * self.derivative_filtered)
                cmd.angular.z = self.clamp(angular, -max_angular, max_angular)

                # Reduces the speed as the front obstacle gets closer
                # (factor goes from 0 at front_stop to 1 at front_release).
                if front is None:
                    front_factor = 1.0
                else:
                    slow_dist = max(front_release, front_stop + 0.01)
                    front_factor = self.clamp(
                        (front - front_stop) / (slow_dist - front_stop), 0.0, 1.0)

                # Sharper turns also reduce the linear speed (down to 50% at max angular speed).
                turn_factor = 1.0 - 0.5 * min(1.0, abs(cmd.angular.z) / max(max_angular, 1e-6))
                cmd.linear.x = max_speed * min(front_factor, turn_factor)

        else:
            # Unexpected state: stop for safety.
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
        # Tries to stop the robot when the node shuts down.
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()