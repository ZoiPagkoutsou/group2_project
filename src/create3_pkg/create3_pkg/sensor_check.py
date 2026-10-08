#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector
from sensor_msgs.msg import LaserScan


class SensorCheck(Node):

    def __init__(self):
        super().__init__('sensor_check')

        self.ir_values = []
        self.hazard_detections = []
        self.scan_msg = None

        self.ir_subscriber = self.create_subscription(
            IrIntensityVector,
            '/robot_8/ir_intensity',
            self.ir_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.hazard_subscriber = self.create_subscription(
            HazardDetectionVector,
            '/robot_8/hazard_detection',
            self.hazard_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot_8/scan',
            self.scan_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.report_status)

        self.get_logger().info(
            'Sensor check started. Waiting for sensor data...'
        )

    def ir_callback(self, msg):
        self.ir_values = [
            reading.value for reading in msg.readings
        ]

    def hazard_callback(self, msg):
        self.hazard_detections = msg.detections

    def scan_callback(self, msg):
        self.scan_msg = msg

    def report_status(self):
        ir_text = str(self.ir_values) if self.ir_values else 'NO DATA'

        hazards = []
        for detection in self.hazard_detections:
            frame = getattr(detection.header, 'frame_id', 'unknown')
            hazards.append(f'type={detection.type}, frame={frame}')

        hazard_text = str(hazards) if hazards else 'NO DATA'

        if self.scan_msg is not None:
            valid_ranges = [
                distance
                for distance in self.scan_msg.ranges
                if (
                    self.scan_msg.range_min <= distance <= self.scan_msg.range_max
                    and distance != float('inf')
                )
            ]

            if valid_ranges:
                closest = min(valid_ranges)
                scan_text = f'closest={closest:.3f} m | readings={len(valid_ranges)}'
            else:
                scan_text = 'NO VALID RANGE DATA'
        else:
            scan_text = 'NO DATA'

        self.get_logger().info(
            f'IR={ir_text} | HAZARD={hazard_text} | LASER={scan_text}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SensorCheck()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
