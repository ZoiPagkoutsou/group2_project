#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SensorReader(Node):

    def __init__(self):
        super().__init__('sensor_reader')

        self.ir_subscriber = self.create_subscription(
            IrIntensityVector,
            '/robot_8/ir_intensity',
            self.ir_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.get_logger().info('Sensor reader started')


    def ir_callback(self, msg):

        values = [reading.value for reading in msg.readings]

        self.get_logger().info(
            f'IR readings: {values}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SensorReader()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()