#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import IrIntensityVector


class WallPerception(Node):

    def __init__(self):
        super().__init__('wall_perception')

        self.subscription = self.create_subscription(
            IrIntensityVector,
            '/robot_8/ir_intensity',
            self.ir_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.get_logger().info('Wall perception started')


    def ir_callback(self, msg):

        values = [reading.value for reading in msg.readings]

        if len(values) != 7:
            return

        side_left = values[0]
        left = values[1]
        front_left = values[2]
        front_center_left = values[3]
        front_center_right = values[4]
        front_right = values[5]
        right = values[6]

        left_wall = max(side_left, left)

        front = max(
            front_left,
            front_center_left,
            front_center_right,
            front_right
        )

        right_wall = max(front_right, right)

        self.get_logger().info(
            f'LEFT={left_wall:4d} | '
            f'FRONT={front:4d} | '
            f'RIGHT={right_wall:4d}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = WallPerception()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()