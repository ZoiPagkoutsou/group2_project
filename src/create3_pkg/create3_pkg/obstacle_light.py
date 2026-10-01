#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import IrIntensityVector, LightringLeds, LedColor
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ObstacleLight(Node):

    def __init__(self):
        super().__init__('obstacle_light')

        self.ir_readings = []

        self.ir_subscriber = self.create_subscription(
            IrIntensityVector,
            '/robot_8/ir_intensity',
            self.ir_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.light_publisher = self.create_publisher(
            LightringLeds,
            '/robot_8/cmd_lightring',
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.timer_callback
        )

        self.get_logger().info('Obstacle light node started')

    def ir_callback(self, msg):
        self.ir_readings = msg.readings

    def timer_callback(self):

        if not self.ir_readings:
            return

        msg = LightringLeds()
        msg.override_system = True

        green_led = LedColor(red=0, green=255, blue=0)
        red_led = LedColor(red=255, green=0, blue=0)

        values = [reading.value for reading in self.ir_readings]

        max_value = max(values)

        if max_value > 500:
            msg.leds = [red_led] * 6
        else:
            msg.leds = [green_led] * 6

        self.light_publisher.publish(msg)

        self.get_logger().info(
            f'IR: {values} | max={max_value}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleLight()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()