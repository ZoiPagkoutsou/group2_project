#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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
        self.forward_speed = 0.06           # normal speed
        self.slow_speed = 0.03              # near obstacle / wall lost

        # Safety distances
        self.front_stop_distance = 0.45
        self.front_slow_distance = 0.70

        # If left wall is farther than this,
        # we consider the wall lost
        self.left_wall_lost_distance = 0.60

        # P-controller
        self.kp = 0.5

        # Maximum normal steering
        self.max_turn_speed = 0.20

        # Emergency front turn
        self.front_turn_speed = 0.30

        # Turn left while searching for lost wall
        self.reacquire_turn_speed = 0.30

        # Previous filtered values
        self.filtered_left = None
        self.filtered_front = None

        # Low-pass filter
        self.alpha = 0.7

        # -------------------------------------------------
        # LIDAR SUBSCRIBER
        # -------------------------------------------------

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot_8/scan',
            self.scan_callback,
            qos_profile_sensor_data
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
            'LiDAR left-wall follower started'
        )


    # =====================================================
    # LIDAR CALLBACK
    # =====================================================

    def scan_callback(self, msg):

        # -------------------------------------------------
        # READ LIDAR SECTORS
        # -------------------------------------------------

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

        # Front information is required for safety
        if front_distance is None:

            self.get_logger().warn(
                'No valid front LiDAR data -> STOP'
            )

            self.stop_robot()
            return

        # -------------------------------------------------
        # FILTER SENSOR VALUES
        # -------------------------------------------------

        if left_distance is not None:

            self.filtered_left = self.low_pass_filter(
                self.filtered_left,
                left_distance
            )

            left_distance = self.filtered_left

        else:

            self.filtered_left = None

        self.filtered_front = self.low_pass_filter(
            self.filtered_front,
            front_distance
        )

        # Use current value if it detects something closer
        # than the filtered value
        front_distance = min(
            front_distance,
            self.filtered_front
        )

        # -------------------------------------------------
        # DISPLAY PERCEPTION
        # -------------------------------------------------

        left_text = (
            f'{left_distance:.2f}'
            if left_distance is not None
            else 'N/A'
        )

        right_text = (
            f'{right_distance:.2f}'
            if right_distance is not None
            else 'N/A'
        )

        self.get_logger().info(
            f'LEFT={left_text} m | '
            f'FRONT={front_distance:.2f} m | '
            f'RIGHT={right_text} m'
        )

        # -------------------------------------------------
        # WALL STATUS
        # -------------------------------------------------

        left_wall_lost = (
            left_distance is None
            or left_distance > self.left_wall_lost_distance
        )

        # -------------------------------------------------
        # CONTROL
        # -------------------------------------------------

        cmd = Twist()

        # =================================================
        # CASE 1:
        # FRONT DANGEROUSLY CLOSE
        # Highest priority
        # =================================================

        if front_distance < self.front_stop_distance:

            self.get_logger().warn(
                'Front blocked -> turning RIGHT'
            )

            cmd.linear.x = 0.0
            cmd.angular.z = -self.front_turn_speed


        # =================================================
        # CASE 2:
        # LEFT WALL LOST
        # Search for it again
        # =================================================

        elif left_wall_lost:

            self.get_logger().warn(
                'Left wall lost -> turning LEFT to reacquire'
            )

            cmd.linear.x = self.slow_speed
            cmd.angular.z = self.reacquire_turn_speed


        # =================================================
        # CASE 3:
        # FRONT GETTING CLOSE
        # Slow down but continue following wall
        # =================================================

        elif front_distance < self.front_slow_distance:

            error = (
                self.desired_left_distance
                - left_distance
            )

            angular_correction = (
                -self.kp * error
            )

            angular_correction = self.limit(
                angular_correction,
                -self.max_turn_speed,
                self.max_turn_speed
            )

            self.get_logger().info(
                f'Front close -> slow | '
                f'error={error:.3f} | '
                f'turn={angular_correction:.3f}'
            )

            cmd.linear.x = self.slow_speed
            cmd.angular.z = angular_correction


        # =================================================
        # CASE 4:
        # NORMAL LEFT-WALL FOLLOWING
        # =================================================

        else:

            error = (
                self.desired_left_distance
                - left_distance
            )

            angular_correction = (
                -self.kp * error
            )

            angular_correction = self.limit(
                angular_correction,
                -self.max_turn_speed,
                self.max_turn_speed
            )

            self.get_logger().info(
                f'Wall follow | '
                f'error={error:.3f} | '
                f'turn={angular_correction:.3f}'
            )

            cmd.linear.x = self.forward_speed
            cmd.angular.z = angular_correction

        # -------------------------------------------------
        # SEND COMMAND
        # -------------------------------------------------

        self.cmd_vel_publisher.publish(cmd)


    # =====================================================
    # GET DISTANCE FROM LIDAR SECTOR
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

            angle_deg = (
                math.degrees(angle_rad) + 180.0
            ) % 360.0 - 180.0

            if (
                start_angle_deg
                <= angle_deg
                <= end_angle_deg
            ):

                if (
                    math.isfinite(distance)
                    and msg.range_min
                    <= distance
                    <= msg.range_max
                ):

                    valid_ranges.append(distance)

        if len(valid_ranges) == 0:
            return None

        # Front:
        # nearest obstacle matters most
        if use_min:
            return min(valid_ranges)

        # Side:
        # median gives more stable wall measurement
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

    finally:

        if rclpy.ok():
            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()