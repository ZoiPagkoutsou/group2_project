#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import IrIntensityVector, LightringLeds, LedColor
from rclpy.qos import ReliabilityPolicy, QoSProfile

class lightController(Node):

    def __init__(self):
        super().__init__("lightController")

        self.irSubscriber = self.create_subscription(
            IrIntensityVector,
            "/robot_8/ir_intensity",
            self.ir_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.lightringPublisher = self.create_publisher(
            LightringLeds,
            "/robot_8/cmd_lightring",
            10
        )

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ir_readings = []

    def timer_callback(self):
        msg = LightringLeds()
        msg.override_system = True

        blueLed = LedColor(red=0, green=0, blue=255)

        if self.ir_readings:
            if self.ir_readings[1].value > 100:
                msg.leds = [blueLed, blueLed, blueLed,
                            blueLed, blueLed, blueLed]

        self.lightringPublisher.publish(msg)
        print("Publishing...")

    def ir_callback(self, msg):
        self.ir_readings = msg.readings

def main():
    rclpy.init()
    controller = lightController()
    rclpy.spin(controller)

if __name__ == '__main__':
    main()
