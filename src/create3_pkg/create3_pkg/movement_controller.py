#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from irobot_create_msgs.action import DriveDistance, RotateAngle


class MovementController(Node):

    def __init__(self):
        super().__init__('movement_controller')

        self.drive_client = ActionClient(
            self,
            DriveDistance,
            '/robot_8/drive_distance'
        )

        self.rotate_client = ActionClient(
            self,
            RotateAngle,
            '/robot_8/rotate_angle'
        )

        self.get_logger().info('Movement controller started')

    def drive_forward(self, distance=0.2, speed=0.1):
        goal = DriveDistance.Goal()

        goal.distance = distance
        goal.max_translation_speed = speed

        self.get_logger().info(
            f'Driving forward {distance} m'
        )

        self.drive_client.wait_for_server()

        return self.drive_client.send_goal_async(goal)

    def rotate_left(self, angle_deg=90, speed=0.5):
        goal = RotateAngle.Goal()

        goal.angle = math.radians(angle_deg)
        goal.max_rotation_speed = speed

        self.get_logger().info(
            f'Rotating left {angle_deg} degrees'
        )

        self.rotate_client.wait_for_server()

        return self.rotate_client.send_goal_async(goal)

    def rotate_right(self, angle_deg=90, speed=0.5):
        goal = RotateAngle.Goal()

        goal.angle = -math.radians(angle_deg)
        goal.max_rotation_speed = speed

        self.get_logger().info(
            f'Rotating right {angle_deg} degrees'
        )

        self.rotate_client.wait_for_server()

        return self.rotate_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)

    node = MovementController()

    # First test: move forward 20 cm
    future = node.drive_forward(distance=0.2, speed=0.1)

    rclpy.spin_until_future_complete(node, future)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()