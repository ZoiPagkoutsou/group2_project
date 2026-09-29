#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import LightringLeds


class LightSubscriber(Node):

    def __init__(self):
        super().__init__('lightSubscriber')

        # Subscribe to the cmd_lightring topic
        self.subscription = self.create_subscription(
            LightringLeds,
            'cmd_lightring',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received lightring message: override_system={msg.override_system}'
        )

        # Print information about the LEDs
        for i, led in enumerate(msg.leds):
            self.get_logger().info(
                f'LED {i}: R={led.red}, G={led.green}, B={led.blue}'
            )


def main(args=None):
    rclpy.init(args=args)

    subscriber = LightSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()