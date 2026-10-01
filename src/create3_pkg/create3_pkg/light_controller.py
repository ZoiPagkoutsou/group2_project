#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import LightringLeds, LedColor


class LightController(Node):

    def __init__(self):
        super().__init__('light_controller')

        self.publisher = self.create_publisher(
            LightringLeds,
            '/robot_8/cmd_lightring',
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.timer_callback
        )

        self.get_logger().info('Light controller started')


    def timer_callback(self):
        msg = LightringLeds()

        msg.override_system = True

        blue_led = LedColor(
            red=0,
            green=0,
            blue=255
        )

        msg.leds = [
            blue_led,
            blue_led,
            blue_led,
            blue_led,
            blue_led,
            blue_led
        ]

        self.publisher.publish(msg)

        self.get_logger().info('Publishing blue light')


def main(args=None):
    rclpy.init(args=args)

    node = LightController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()