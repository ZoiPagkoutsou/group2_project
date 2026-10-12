#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.action import RotateAngle


class WallFollowerFSM(Node):

    def __init__(self):
        super().__init__('wall_follower_fsm')

        # =====================================================
        # STATES
        # =====================================================

        self.FOLLOW_WALL = 'FOLLOW_WALL'
        self.CLEAR_CORNER = 'CLEAR_CORNER'
        self.TURN_RIGHT = 'TURN_RIGHT'
        self.TURN_LEFT = 'TURN_LEFT'
        self.STOP = 'STOP'

        self.current_state = self.FOLLOW_WALL

        # =====================================================
        # CONTROL LOOP
        # =====================================================

        self.control_frequency = 10.0

        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )

        # =====================================================
        # 18 VIRTUAL LIDAR SENSORS
        # =====================================================

        # 0°   = front
        # 90°  = left
        # 270° = right

        self.sensor_angles = [
            280, 290, 300, 310,
            320, 330, 340, 350,
            0,
            10, 20, 30, 40,
            50, 60, 70, 80, 90
        ]

        # Each virtual sensor represents ±3°
        self.sensor_half_width = 3.0

        # Median values:
        # useful for stable wall geometry
        self.wall_sensors = {
            angle: None
            for angle in self.sensor_angles
        }

        # Minimum values:
        # useful for obstacle safety
        self.obstacle_sensors = {
            angle: None
            for angle in self.sensor_angles
        }

        # =====================================================
        # WALL FOLLOWING
        # =====================================================

        # Desired LiDAR-to-wall distance
        self.desired_wall_distance = 0.24

        # Forward speed
        self.forward_speed = 0.06

        # Distance and orientation correction gains
        self.k_distance = 1.2
        self.k_angle = 1.0

        self.max_wall_turn = 0.35

        # Small deadbands reduce steering jitter
        self.distance_deadband = 0.01
        self.angle_deadband = math.radians(2.0)

        # =====================================================
        # FRONT OBSTACLE
        # =====================================================

        self.front_stop_distance = 0.40

        # Half-width of protected path in front
        self.path_half_width = 0.18

        # =====================================================
        # LEFT WALL DETECTION
        # =====================================================

        # Sensors used for left-wall fitting
        self.left_wall_angles = [
            60,
            70,
            80,
            90
        ]

        self.wall_detection_max = 0.70

        # Confirm wall loss over several scans
        self.wall_lost_required = 3
        self.wall_lost_counter = 0

        self.left_wall_seen = False

        # =====================================================
        # 90-DEGREE TURNS
        # =====================================================

        self.turn_angle = math.pi / 2.0
        self.turn_speed = 0.50

        self.turn_in_progress = False

        # =====================================================
        # FRESH LIDAR AFTER TURN
        # =====================================================

        # Counts complete LaserScan messages
        self.scan_number = 0

        # Wait for 2 NEW scans after a turn
        self.post_turn_scans_required = 2

        self.wait_until_scan = 0

        # =====================================================
        # LEFT CORNER CLEARANCE
        # =====================================================

        # Walls are around 10 cm thick.
        # Move about 12 cm past wall end before left turn.
        self.corner_clear_distance = 0.12

        self.corner_clear_cycles = round(
            (
                self.corner_clear_distance
                / self.forward_speed
            )
            * self.control_frequency
        )

        self.corner_counter = 0

        # =====================================================
        # PERCEPTION RESULTS
        # =====================================================

        self.front_clearance = None
        self.left_distance = None
        self.wall_angle = None

        self.last_scan_time = None
        self.last_log_time = 0.0

        # =====================================================
        # ROS INTERFACES
        # =====================================================

        # LiDAR subscriber
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot_8/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Velocity publisher
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/robot_8/cmd_vel',
            10
        )

        # 90° rotation action
        self.rotate_client = ActionClient(
            self,
            RotateAngle,
            '/robot_8/rotate_angle'
        )

        self.get_logger().info(
            'Wall-following FSM started'
        )

    # =========================================================
    # SEE
    # =========================================================

    def scan_callback(self, msg):

        # One more complete LiDAR scan received
        self.scan_number += 1

        # -----------------------------------------------------
        # Create 18 virtual sensors
        # -----------------------------------------------------

        for angle in self.sensor_angles:

            # Median:
            # stable measurement for wall geometry
            self.wall_sensors[angle] = (
                self.get_virtual_sensor(
                    msg,
                    angle,
                    use_min=False
                )
            )

            # Minimum:
            # nearest object for collision safety
            self.obstacle_sensors[angle] = (
                self.get_virtual_sensor(
                    msg,
                    angle,
                    use_min=True
                )
            )

        # -----------------------------------------------------
        # Calculate front obstacle clearance
        # -----------------------------------------------------

        self.front_clearance = (
            self.calculate_front_clearance()
        )

        # -----------------------------------------------------
        # Estimate left wall
        # -----------------------------------------------------

        wall = self.estimate_left_wall()

        if wall is None:

            self.left_distance = None
            self.wall_angle = None

        else:

            self.left_distance, self.wall_angle = wall

        # -----------------------------------------------------
        # Remember time of newest scan
        # -----------------------------------------------------

        self.last_scan_time = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

    # =========================================================
    # THINK + ACT
    # =========================================================

    def control_loop(self):

        # -----------------------------------------------------
        # No LiDAR yet
        # -----------------------------------------------------

        if self.last_scan_time is None:

            self.stop_robot()
            return

        now = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        # -----------------------------------------------------
        # LiDAR stopped updating
        # -----------------------------------------------------

        if now - self.last_scan_time > 0.5:

            if self.current_state != self.STOP:

                self.get_logger().error(
                    'LiDAR data lost -> STOP'
                )

            self.current_state = self.STOP

            self.stop_robot()
            return

        # -----------------------------------------------------
        # RotateAngle is currently controlling robot
        # -----------------------------------------------------

        if self.turn_in_progress:
            return

        # -----------------------------------------------------
        # CRITICAL:
        # wait for fresh LiDAR after every turn
        # -----------------------------------------------------

        if self.scan_number < self.wait_until_scan:

            self.stop_robot()
            return

        # -----------------------------------------------------
        # FRONT OBSTACLE EVENT
        # -----------------------------------------------------

        front_blocked = (
            self.front_clearance is not None
            and self.front_clearance
            < self.front_stop_distance
        )

        # =====================================================
        # STATE 1: FOLLOW WALL
        # =====================================================

        if self.current_state == self.FOLLOW_WALL:

            # -------------------------------------------------
            # Front obstacle
            #
            # FOLLOW_WALL -> TURN_RIGHT
            # -------------------------------------------------

            if front_blocked:

                self.get_logger().warn(
                    'Front blocked: '
                    'FOLLOW_WALL -> TURN_RIGHT'
                )

                self.start_turn('right')
                return

            # -------------------------------------------------
            # Is left wall present?
            # -------------------------------------------------

            wall_present = (
                self.left_distance is not None
                and self.left_distance
                <= self.wall_detection_max
            )

            # -------------------------------------------------
            # Left wall detected
            # -------------------------------------------------

            if wall_present:

                self.left_wall_seen = True
                self.wall_lost_counter = 0

                self.follow_left_wall()

            # -------------------------------------------------
            # Wall existed before, but now disappeared
            # -------------------------------------------------

            elif self.left_wall_seen:

                self.wall_lost_counter += 1

                if (
                    self.wall_lost_counter
                    >= self.wall_lost_required
                ):

                    self.get_logger().info(
                        'Left wall ended: '
                        'FOLLOW_WALL -> CLEAR_CORNER'
                    )

                    self.current_state = (
                        self.CLEAR_CORNER
                    )

                    self.corner_counter = 0
                    self.wall_lost_counter = 0

                else:

                    # Keep going while confirming
                    # that wall is really gone
                    self.move_forward()

            # -------------------------------------------------
            # No wall has been found yet
            # -------------------------------------------------

            else:

                self.move_forward()

        # =====================================================
        # STATE 2: CLEAR CORNER
        # =====================================================

        elif self.current_state == self.CLEAR_CORNER:

            # Front obstacle still has priority
            if front_blocked:

                self.get_logger().warn(
                    'Obstacle while clearing corner: '
                    'CLEAR_CORNER -> TURN_RIGHT'
                )

                self.start_turn('right')
                return

            # Move beyond wall end
            self.move_forward()

            self.corner_counter += 1

            if (
                self.corner_counter
                >= self.corner_clear_cycles
            ):

                self.get_logger().info(
                    'Corner cleared: '
                    'CLEAR_CORNER -> TURN_LEFT'
                )

                self.start_turn('left')
                return

        # =====================================================
        # TURN STATES
        # =====================================================

        elif self.current_state in (
            self.TURN_RIGHT,
            self.TURN_LEFT
        ):

            self.stop_robot()

        # =====================================================
        # STOP
        # =====================================================

        elif self.current_state == self.STOP:

            self.stop_robot()

        # -----------------------------------------------------
        # Print status twice per second
        # -----------------------------------------------------

        if now - self.last_log_time >= 0.5:

            self.print_status()

            self.last_log_time = now

    # =========================================================
    # FOLLOW LEFT WALL
    # =========================================================

    def follow_left_wall(self):

        # -----------------------------------------------------
        # DISTANCE ERROR
        # -----------------------------------------------------
        #
        # positive:
        # robot too far -> steer LEFT
        #
        # negative:
        # robot too close -> steer RIGHT

        distance_error = (
            self.left_distance
            - self.desired_wall_distance
        )

        if (
            abs(distance_error)
            < self.distance_deadband
        ):

            distance_error = 0.0

        # -----------------------------------------------------
        # WALL ANGLE ERROR
        # -----------------------------------------------------
        #
        # 0:
        # robot parallel to wall
        #
        # negative:
        # robot points toward wall
        #
        # positive:
        # robot points away from wall

        angle_error = self.wall_angle

        if (
            abs(angle_error)
            < self.angle_deadband
        ):

            angle_error = 0.0

        # -----------------------------------------------------
        # Combine distance + angle corrections
        # -----------------------------------------------------

        angular_speed = (
            self.k_distance * distance_error
            + self.k_angle * angle_error
        )

        angular_speed = self.limit(
            angular_speed,
            -self.max_wall_turn,
            self.max_wall_turn
        )

        # -----------------------------------------------------
        # ACT
        # -----------------------------------------------------

        cmd = Twist()

        cmd.linear.x = self.forward_speed
        cmd.angular.z = angular_speed

        self.cmd_publisher.publish(cmd)

    # =========================================================
    # FRONT OBSTACLE DETECTION
    # =========================================================

    def calculate_front_clearance(self):

        closest_x = None

        # Use all 18 virtual sensors
        for angle_deg, distance in (
            self.obstacle_sensors.items()
        ):

            if distance is None:
                continue

            # Convert 0...360 to signed angle
            if angle_deg > 180:

                signed_angle = (
                    angle_deg - 360
                )

            else:

                signed_angle = angle_deg

            angle_rad = math.radians(
                signed_angle
            )

            # Convert polar LiDAR measurement
            # to robot coordinates:
            #
            # x = forward
            # y = left

            x = (
                distance
                * math.cos(angle_rad)
            )

            y = (
                distance
                * math.sin(angle_rad)
            )

            # Ignore points behind robot
            if x <= 0.0:
                continue

            # Only obstacles inside future path
            if (
                abs(y)
                <= self.path_half_width
            ):

                if (
                    closest_x is None
                    or x < closest_x
                ):

                    closest_x = x

        return closest_x

    # =========================================================
    # LEFT WALL ESTIMATION
    # =========================================================

    def estimate_left_wall(self):

        points = []

        # Use several left-side sensors
        for angle_deg in self.left_wall_angles:

            distance = self.wall_sensors.get(
                angle_deg
            )

            if distance is None:
                continue

            if (
                distance
                > self.wall_detection_max
            ):

                continue

            angle_rad = math.radians(
                angle_deg
            )

            x = (
                distance
                * math.cos(angle_rad)
            )

            y = (
                distance
                * math.sin(angle_rad)
            )

            points.append(
                (x, y)
            )

        # Need at least 3 useful points
        if len(points) < 3:

            return None

        # -----------------------------------------------------
        # Fit a straight line:
        #
        # y = m*x + b
        # -----------------------------------------------------

        x_mean = (
            sum(
                x
                for x, y in points
            )
            / len(points)
        )

        y_mean = (
            sum(
                y
                for x, y in points
            )
            / len(points)
        )

        numerator = 0.0
        denominator = 0.0

        for x, y in points:

            numerator += (
                (x - x_mean)
                * (y - y_mean)
            )

            denominator += (
                (x - x_mean) ** 2
            )

        if denominator < 1e-6:

            return None

        slope = (
            numerator
            / denominator
        )

        intercept = (
            y_mean
            - slope * x_mean
        )

        # Wall must be on left side
        if intercept <= 0.0:

            return None

        # -----------------------------------------------------
        # Perpendicular distance from robot
        # to fitted wall
        # -----------------------------------------------------

        wall_distance = (
            intercept
            / math.sqrt(
                slope ** 2 + 1.0
            )
        )

        # -----------------------------------------------------
        # Wall orientation
        # -----------------------------------------------------

        wall_angle = math.atan(
            slope
        )

        return (
            wall_distance,
            wall_angle
        )

    # =========================================================
    # CREATE ONE VIRTUAL LIDAR SENSOR
    # =========================================================

    def get_virtual_sensor(
        self,
        msg,
        center_angle_deg,
        use_min=False
    ):

        values = []

        for i, distance in enumerate(
            msg.ranges
        ):

            # Ignore invalid LiDAR values
            if not math.isfinite(distance):
                continue

            if (
                distance < msg.range_min
                or distance > msg.range_max
            ):

                continue

            # Angle of this LiDAR ray
            angle_rad = (
                msg.angle_min
                + i * msg.angle_increment
            )

            angle_deg = (
                math.degrees(angle_rad)
                % 360.0
            )

            # Shortest difference between
            # LiDAR ray and virtual sensor angle
            difference = (
                (
                    angle_deg
                    - center_angle_deg
                    + 180.0
                )
                % 360.0
                - 180.0
            )

            if (
                abs(difference)
                <= self.sensor_half_width
            ):

                values.append(distance)

        if not values:

            return None

        # Obstacle:
        # closest point matters
        if use_min:

            return min(values)

        # Wall:
        # median is more stable
        return statistics.median(values)

    # =========================================================
    # START 90-DEGREE TURN
    # =========================================================

    def start_turn(
        self,
        direction
    ):

        self.stop_robot()

        if not self.rotate_client.wait_for_server(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                'RotateAngle server unavailable'
            )

            self.current_state = self.STOP
            return

        goal = RotateAngle.Goal()

        if direction == 'left':

            self.current_state = (
                self.TURN_LEFT
            )

            goal.angle = (
                self.turn_angle
            )

        else:

            self.current_state = (
                self.TURN_RIGHT
            )

            goal.angle = (
                -self.turn_angle
            )

        goal.max_rotation_speed = (
            self.turn_speed
        )

        self.turn_in_progress = True

        future = (
            self.rotate_client.send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.turn_goal_response
        )

    # =========================================================
    # TURN GOAL RESPONSE
    # =========================================================

    def turn_goal_response(
        self,
        future
    ):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Turn request failed: {error}'
            )

            self.turn_in_progress = False
            self.current_state = self.STOP
            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'RotateAngle goal rejected'
            )

            self.turn_in_progress = False
            self.current_state = self.STOP
            return

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.turn_finished
        )

    # =========================================================
    # TURN FINISHED
    # =========================================================

    def turn_finished(
        self,
        future
    ):

        try:

            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Turn failed: {error}'
            )

            self.turn_in_progress = False
            self.current_state = self.STOP
            return

        self.turn_in_progress = False

        # -----------------------------------------------------
        # Check turn succeeded
        # -----------------------------------------------------

        if (
            result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().error(
                '90-degree turn did not succeed '
                '-> STOP'
            )

            self.current_state = self.STOP
            return

        # -----------------------------------------------------
        # Turn succeeded
        # -----------------------------------------------------

        self.current_state = self.FOLLOW_WALL

        # Reset wall tracking
        self.left_wall_seen = False
        self.wall_lost_counter = 0
        self.corner_counter = 0

        # -----------------------------------------------------
        # Remove perception from old orientation
        # -----------------------------------------------------

        self.front_clearance = None
        self.left_distance = None
        self.wall_angle = None

        # -----------------------------------------------------
        # IMPORTANT FIX
        #
        # Wait for completely fresh LiDAR data.
        #
        # Two scans are used because the first scan
        # after rotation may have started while the
        # robot was still moving.
        # -----------------------------------------------------

        self.wait_until_scan = (
            self.scan_number
            + self.post_turn_scans_required
        )

        self.get_logger().info(
            '90-degree turn finished -> '
            'waiting for fresh LiDAR'
        )

    # =========================================================
    # BASIC MOTION
    # =========================================================

    def move_forward(self):

        cmd = Twist()

        cmd.linear.x = self.forward_speed
        cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

    # =========================================================
    # STATUS
    # =========================================================

    def print_status(self):

        front_text = (
            f'{self.front_clearance:.2f}'
            if self.front_clearance is not None
            else 'N/A'
        )

        left_text = (
            f'{self.left_distance:.2f}'
            if self.left_distance is not None
            else 'N/A'
        )

        angle_text = (
            f'{math.degrees(self.wall_angle):.1f}'
            if self.wall_angle is not None
            else 'N/A'
        )

        self.get_logger().info(
            f'STATE={self.current_state} | '
            f'FRONT={front_text} m | '
            f'LEFT={left_text} m | '
            f'WALL_ANGLE={angle_text} deg'
        )

    # =========================================================
    # LIMIT VALUE
    # =========================================================

    def limit(
        self,
        value,
        minimum,
        maximum
    ):

        return max(
            minimum,
            min(
                maximum,
                value
            )
        )


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = WallFollowerFSM()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        if rclpy.ok():

            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()