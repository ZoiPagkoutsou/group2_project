#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollowerV1(Node):

    def __init__(self):
        super().__init__('wall_follower_v1')

        # -------------------------------------------------
        # SETTINGS
        # -------------------------------------------------

        # Normal forward speed
        self.forward_speed = 0.05

        # Slower speed when an obstacle is close
        self.slow_speed = 0.03

        # Front obstacle thresholds
        self.front_warning_distance = 0.35
        self.front_danger_distance = 0.20

        # Front-left obstacle threshold
        self.front_left_warning_distance = 0.2

        # Left-wall distance limits
        self.left_too_close = 0.10
        self.left_too_far = 0.2

        # Steering speeds
        self.wall_steer_speed = 0.15
        self.obstacle_turn_speed = 0.25
        self.emergency_turn_speed = 0.35

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
            'Wall follower V1 started'
        )


    def scan_callback(self, msg):

        # -------------------------------------------------
        # READ LIDAR SECTORS
        # -------------------------------------------------

        front_distance = self.get_sector_distance(
            msg,
            -15,
            15,
            use_min=True
        )

        front_left_distance = self.get_sector_distance(
            msg,
            20,
            70,
            use_min=True
        )

        left_distance = self.get_sector_distance(
            msg,
            75,
            105,
            use_min=False
        )

        # -------------------------------------------------
        # SAFETY: FRONT DATA REQUIRED
        # -------------------------------------------------

        if front_distance is None:

            self.get_logger().warn(
                'No front LiDAR data -> STOP'
            )

            self.stop_robot()
            return

        # -------------------------------------------------
        # PRINT SENSOR VALUES
        # -------------------------------------------------

        left_text = (
            f'{left_distance:.2f}'
            if left_distance is not None
            else 'N/A'
        )

        front_left_text = (
            f'{front_left_distance:.2f}'
            if front_left_distance is not None
            else 'N/A'
        )

        self.get_logger().info(
            f'LEFT={left_text} m | '
            f'FRONT_LEFT={front_left_text} m | '
            f'FRONT={front_distance:.2f} m'
        )

        # -------------------------------------------------
        # CREATE COMMAND
        # -------------------------------------------------

        cmd = Twist()

        # -------------------------------------------------
        # CASE 1:
        # VERY CLOSE FRONT OBSTACLE
        # -------------------------------------------------

        if front_distance < self.front_danger_distance:

            self.get_logger().warn(
                'Danger in front -> STOP and turn RIGHT'
            )

            cmd.linear.x = 0.0
            cmd.angular.z = -self.emergency_turn_speed

        # -------------------------------------------------
        # CASE 2:
        # FRONT OBSTACLE APPROACHING
        # -------------------------------------------------

        elif front_distance < self.front_warning_distance:

            self.get_logger().info(
                'Obstacle ahead -> curve RIGHT'
            )

            cmd.linear.x = self.slow_speed
            cmd.angular.z = -self.obstacle_turn_speed

        # -------------------------------------------------
        # CASE 3:
        # FRONT-LEFT OBSTACLE
        # -------------------------------------------------

        elif (
            front_left_distance is not None
            and front_left_distance
            < self.front_left_warning_distance
        ):

            self.get_logger().info(
                'Obstacle front-left -> curve RIGHT'
            )

            cmd.linear.x = self.slow_speed
            cmd.angular.z = -self.obstacle_turn_speed

        # -------------------------------------------------
        # CASE 4:
        # NO LEFT WALL
        # -------------------------------------------------

        elif left_distance is None:

            self.get_logger().info(
                'No left wall -> go FORWARD'
            )

            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        # -------------------------------------------------
        # CASE 5:
        # TOO CLOSE TO LEFT WALL
        # -------------------------------------------------

        elif left_distance < self.left_too_close:

            self.get_logger().info(
                'Too close to left wall -> steer RIGHT'
            )

            cmd.linear.x = self.forward_speed
            cmd.angular.z = -self.wall_steer_speed

        # -------------------------------------------------
        # CASE 6:
        # TOO FAR FROM LEFT WALL
        # -------------------------------------------------

        elif left_distance > self.left_too_far:

            self.get_logger().info(
                'Too far from left wall -> steer LEFT'
            )

            cmd.linear.x = self.forward_speed
            cmd.angular.z = self.wall_steer_speed

        # -------------------------------------------------
        # CASE 7:
        # GOOD WALL DISTANCE
        # -------------------------------------------------

        else:

            self.get_logger().info(
                'Wall distance good -> go STRAIGHT'
            )

            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        # -------------------------------------------------
        # SEND COMMAND
        # -------------------------------------------------

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
                    and msg.range_min
                    <= distance
                    <= msg.range_max
                ):

                    valid_ranges.append(distance)

        if not valid_ranges:
            return None

        # For obstacle sectors:
        # use the nearest object
        if use_min:
            return min(valid_ranges)

        # For the wall:
        # median gives a more stable estimate
        return statistics.median(valid_ranges)


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

    node = WallFollowerV1()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        node.stop_robot()

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()