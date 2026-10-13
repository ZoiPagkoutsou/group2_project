#!/usr/bin/env python3
"""Helper to calibrate the LiDAR mounting angle.

Put a box ~30 cm directly IN FRONT of the robot (nothing else closer) and run:
    ros2 run create3_pkg lidar_check
It prints the angle of the closest object. If the box is in front but the
'robot-frame angle' is not ~0 deg, set lidar_angle_offset_deg to minus that
value. Then move the box to the robot's LEFT: it should read ~+90 deg. If it
reads ~-90 deg, set lidar_reversed: true.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarCheck(Node):

    def __init__(self):
        super().__init__('lidar_check')
        self.declare_parameter('scan_topic', '/robot_8/scan')
        self.declare_parameter('lidar_angle_offset_deg', 0.0)
        self.declare_parameter('lidar_reversed', False)
        topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'Listening on {topic}')

    def _on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        valid = np.isfinite(r) & (r > max(msg.range_min, 0.05))
        if not np.any(valid):
            return
        i = int(np.argmin(np.where(valid, r, np.inf)))
        raw = msg.angle_min + i * msg.angle_increment
        robot = -raw if self.get_parameter('lidar_reversed').value else raw
        robot += math.radians(self.get_parameter('lidar_angle_offset_deg').value)
        robot = (robot + math.pi) % (2 * math.pi) - math.pi
        self.get_logger().info(
            f'closest {r[i]:.2f} m | raw angle {math.degrees(raw):+7.1f} deg | '
            f'robot-frame angle {math.degrees(robot):+7.1f} deg '
            f'(0=front, +90=left, -90=right) | {r.size} points',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
