#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # -------------------------------------------------
        # SETTINGS
        # -------------------------------------------------

        # Desired distance from the left wall
        self.desired_left_distance = 0.30   # meters

        # Forward speeds
        self.forward_speed = 0.08           # normal speed
        self.slow_speed = 0.04              # when front gets close

        # Safety distances
        self.front_stop_distance = 0.25     # emergency turn
        self.front_slow_distance = 0.45     # start slowing down

        # P-controller gain
        self.kp = 1.0

        # Maximum turning speed
        self.max_turn_speed = 0.45

        # Turning speed when front is blocked
        self.front_turn_speed = 0.50

        # Previous filtered values
        self.filtered_left = None
        self.filtered_front = None

        # Smoothing factor
        # Larger alpha = more weight on old value
        self.alpha = 0.7

        # -------------------------------------------------
        # LIDAR SUBSCRIBER
        # -------------------------------------------------

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot_8/scan',
            self.scan_callback,
            10
        )

        # -------------------------------------------------
        # CMD_VEL PUBLISHER
        # -------------------------------------------------

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot_8/cmd_vel',
            10
        )

        self.get_logger().info(
            'LiDAR wall follower started'
        )


    # =====================================================
    # LIDAR CALLBACK
    # =====================================================

    def scan_callback(self, msg):

        # Get distances from useful sectors
        left_distance = self.get_sector_distance(
            msg,
            75,
            105,
            use_min=False
        )

        front_distance = self.get_sector_distance(
            msg,
            -15,
            15,
            use_min=True
        )

        right_distance = self.get_sector_distance(
            msg,
            -105,
            -75,
            use_min=False
        )

        # If important data is missing, stop
        if left_distance is None or front_distance is None:

            self.stop_robot()

            self.get_logger().warn(
                'Invalid LiDAR data -> stopping'
            )

            return

        # -------------------------------------------------
        # FILTER SENSOR VALUES
        # -------------------------------------------------

        self.filtered_left = self.low_pass_filter(
            self.filtered_left,
            left_distance
        )

        self.filtered_front = self.low_pass_filter(
            self.filtered_front,
            front_distance
        )

        left_distance = self.filtered_left
        front_distance = self.filtered_front

        # -------------------------------------------------
        # PRINT PERCEPTION
        # -------------------------------------------------

        right_text = (
            f'{right_distance:.2f}'
            if right_distance is not None
            else 'N/A'
        )

        self.get_logger().info(
            f'LEFT={left_distance:.2f} m | '
            f'FRONT={front_distance:.2f} m | '
            f'RIGHT={right_text} m'
        )

        # -------------------------------------------------
        # CONTROL
        # -------------------------------------------------

        cmd = Twist()

        # CASE 1:
        # Front is dangerously close
        if front_distance < self.front_stop_distance:

            self.get_logger().warn(
                'Front blocked -> turning right'
            )

            cmd.linear.x = 0.0
            cmd.angular.z = -self.front_turn_speed

        # CASE 2:
        # Front is getting close
        elif front_distance < self.front_slow_distance:

            error = (
                self.desired_left_distance
                - left_distance
            )

            angular_correction = -self.kp * error

            angular_correction = self.limit(
                angular_correction,
                -self.max_turn_speed,
                self.max_turn_speed
            )

            self.get_logger().info(
                'Front close -> slowing down'
            )

            cmd.linear.x = self.slow_speed
            cmd.angular.z = angular_correction

        # CASE 3:
        # Normal wall following
        else:

            error = (
                self.desired_left_distance
                - left_distance
            )

            angular_correction = -self.kp * error

            angular_correction = self.limit(
                angular_correction,
                -self.max_turn_speed,
                self.max_turn_speed
            )

            cmd.linear.x = self.forward_speed
            cmd.angular.z = angular_correction

            self.get_logger().info(
                f'Wall follow | '
                f'error={error:.3f} | '
                f'turn={angular_correction:.3f}'
            )

        self.cmd_vel_publisher.publish(cmd)


    # =====================================================
    # GET DISTANCE FROM A LIDAR SECTOR
    # =====================================================

    def get_sector_distance(
        self,
        msg,
        start_angle_deg,
        end_angle_deg,
        use_min=False
    ):

        valid_ranges = []

        for i, distance in enumerate(msg.ranges):

            angle_rad = (
                msg.angle_min
                + i * msg.angle_increment
            )

            angle_deg = math.degrees(angle_rad)

            if (
                start_angle_deg
                <= angle_deg
                <= end_angle_deg
            ):

                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                ):

                    valid_ranges.append(distance)

        if len(valid_ranges) == 0:
            return None

        # Front:
        # nearest obstacle is important
        if use_min:
            return min(valid_ranges)

        # Side:
        # median is more stable than one ray
        return statistics.median(valid_ranges)


    # =====================================================
    # LOW-PASS FILTER
    # =====================================================

    def low_pass_filter(
        self,
        previous_value,
        new_value
    ):

        if previous_value is None:
            return new_value

        return (
            self.alpha * previous_value
            + (1.0 - self.alpha) * new_value
        )


    # =====================================================
    # LIMIT VALUE
    # =====================================================

    def limit(
        self,
        value,
        minimum,
        maximum
    ):

        return max(
            minimum,
            min(maximum, value)
        )


    # =====================================================
    # STOP ROBOT
    # =====================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = WallFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.stop_robot()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()