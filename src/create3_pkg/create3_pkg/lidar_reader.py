#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class LidarReader(Node):

    def __init__(self):
        super().__init__('lidar_reader')

        self.subscription = self.create_subscription(
            LaserScan,
            '/robot_8/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('LiDAR reader started')


    def scan_callback(self, msg):

        # -------------------------------------------------
        # BASIC RAW SCAN INFORMATION
        # -------------------------------------------------

        total_points = len(msg.ranges)

        self.get_logger().info(
            f'Points={total_points} | '
            f'Angle min={math.degrees(msg.angle_min):.1f} deg | '
            f'Angle max={math.degrees(msg.angle_max):.1f} deg'
        )

        # -------------------------------------------------
        # GET FRONT / LEFT / RIGHT DISTANCES
        # -------------------------------------------------

        front = self.get_sector_distance(
            msg,
            -10,
            10
        )

        left = self.get_sector_distance(
            msg,
            80,
            100
        )

        right = self.get_sector_distance(
            msg,
            -100,
            -80
        )

        front_text = (
            f'{front:.2f} m'
            if front is not None
            else 'N/A'
        )

        left_text = (
            f'{left:.2f} m'
            if left is not None
            else 'N/A'
        )

        right_text = (
            f'{right:.2f} m'
            if right is not None
            else 'N/A'
        )

        self.get_logger().info(
            f'LEFT={left_text} | '
            f'FRONT={front_text} | '
            f'RIGHT={right_text}'
        )

        # -------------------------------------------------
        # PRINT SOME RAW INDIVIDUAL MEASUREMENTS
        # -------------------------------------------------

        sample_angles = [
            -90,
            -45,
            0,
            45,
            90
        ]

        raw_text = []

        for angle in sample_angles:

            distance = self.get_distance_at_angle(
                msg,
                angle
            )

            if distance is None:
                text = f'{angle:+}°=N/A'
            else:
                text = f'{angle:+}°={distance:.2f}m'

            raw_text.append(text)

        self.get_logger().info(
            'RAW: ' + ' | '.join(raw_text)
        )


    # =====================================================
    # GET DISTANCE FROM A SMALL SECTOR
    # =====================================================

    def get_sector_distance(
        self,
        msg,
        start_angle_deg,
        end_angle_deg
    ):

        valid_ranges = []

        for i, distance in enumerate(msg.ranges):

            angle_rad = (
                msg.angle_min
                + i * msg.angle_increment
            )

            angle_deg = math.degrees(angle_rad)

            if (
                start_angle_deg
                <= angle_deg
                <= end_angle_deg
            ):

                if (
                    math.isfinite(distance)
                    and msg.range_min
                    <= distance
                    <= msg.range_max
                ):

                    valid_ranges.append(distance)

        if not valid_ranges:
            return None

        return statistics.median(valid_ranges)


    # =====================================================
    # GET RAW DISTANCE AT ONE ANGLE
    # =====================================================

    def get_distance_at_angle(
        self,
        msg,
        target_angle_deg
    ):

        target_angle_rad = math.radians(
            target_angle_deg
        )

        index = round(
            (
                target_angle_rad
                - msg.angle_min
            )
            / msg.angle_increment
        )

        if index < 0 or index >= len(msg.ranges):
            return None

        distance = msg.ranges[index]

        if (
            not math.isfinite(distance)
            or distance < msg.range_min
            or distance > msg.range_max
        ):
            return None

        return distance


def main(args=None):

    rclpy.init(args=args)

    node = LidarReader()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()