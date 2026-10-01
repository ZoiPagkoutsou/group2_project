#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # -----------------------------
        # SETTINGS
        # -----------------------------

        # These are starting values only.
        # You must tune them using real sensor readings.
        self.front_threshold = 700

        self.left_too_far = 250
        self.left_too_close = 900

        self.forward_speed = 0.10
        self.turn_speed = 0.35
        self.correction_speed = 0.18

        # -----------------------------
        # SENSOR SUBSCRIBER
        # -----------------------------

        self.ir_subscriber = self.create_subscription(
            IrIntensityVector,
            '/robot_8/ir_intensity',
            self.ir_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        # -----------------------------
        # VELOCITY PUBLISHER
        # -----------------------------

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot_8/cmd_vel',
            10
        )

        self.get_logger().info(
            'Left wall follower started'
        )


    def ir_callback(self, msg):

        values = [
            reading.value
            for reading in msg.readings
        ]

        if len(values) != 7:
            return

        # Sensor positions
        side_left = values[0]
        left = values[1]

        front_left = values[2]
        front_center_left = values[3]
        front_center_right = values[4]
        front_right = values[5]

        right = values[6]

        # -----------------------------
        # PERCEPTION
        # -----------------------------

        left_wall = max(
            side_left,
            left
        )

        front = max(
            front_left,
            front_center_left,
            front_center_right,
            front_right
        )

        right_wall = max(
            front_right,
            right
        )

        self.get_logger().info(
            f'LEFT={left_wall} | '
            f'FRONT={front} | '
            f'RIGHT={right_wall}'
        )

        # -----------------------------
        # DECISION
        # -----------------------------

        cmd = Twist()

        # CASE 1:
        # Obstacle in front
        if front > self.front_threshold:

            self.get_logger().info(
                'Front blocked -> turn right'
            )

            cmd.linear.x = 0.0
            cmd.angular.z = -self.turn_speed

        # CASE 2:
        # Left wall is too far away
        elif left_wall < self.left_too_far:

            self.get_logger().info(
                'Wall too far -> steer left'
            )

            cmd.linear.x = self.forward_speed
            cmd.angular.z = self.correction_speed

        # CASE 3:
        # Left wall is too close
        elif left_wall > self.left_too_close:

            self.get_logger().info(
                'Wall too close -> steer right'
            )

            cmd.linear.x = self.forward_speed
            cmd.angular.z = -self.correction_speed

        # CASE 4:
        # Good wall distance
        else:

            self.get_logger().info(
                'Wall distance good -> forward'
            )

            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = WallFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:

        # Stop robot when Ctrl+C is pressed
        stop_cmd = Twist()

        node.cmd_vel_publisher.publish(
            stop_cmd
        )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()