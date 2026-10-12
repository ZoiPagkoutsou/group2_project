#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.action import RotateAngle


class WallFollowerV2(Node):

    def __init__(self):
        super().__init__('wall_follower_v2')

        # =================================================
        # SETTINGS
        # =================================================

        # Forward motion
        self.forward_speed = 0.05

        # Front wall detection
        self.front_stop_distance = 0.45

        # Left wall:
        # farther than this means the wall probably ended
        self.left_wall_lost_distance = 0.60

        # Require several scans before deciding wall is lost
        self.wall_lost_required_scans = 3
        self.wall_lost_counter = 0

        # Rotation
        self.turn_angle_deg = 90.0
        self.rotation_speed = 0.40

        # True while RotateAngle action is running
        self.turning = False

        # We do not detect "lost wall" until a left wall
        # has actually been seen first.
        self.left_wall_seen = False

        # =================================================
        # LIDAR SUBSCRIBER
        # =================================================

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot_8/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # =================================================
        # CMD_VEL PUBLISHER
        # =================================================

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot_8/cmd_vel',
            10
        )

        # =================================================
        # ROTATE ACTION CLIENT
        # =================================================

        self.rotate_client = ActionClient(
            self,
            RotateAngle,
            '/robot_8/rotate_angle'
        )

        self.get_logger().info(
            'Wall follower V2 started'
        )


    # =====================================================
    # LIDAR CALLBACK
    # =====================================================

    def scan_callback(self, msg):

        # Do not send forward commands during a 90° turn
        if self.turning:
            return

        # -------------------------------------------------
        # FRONT
        # -20° to +20°
        # nearest obstacle matters
        # -------------------------------------------------

        front_distance = self.get_sector_distance(
            msg,
            -20,
            20,
            use_min=True
        )

        # -------------------------------------------------
        # LEFT
        # +75° to +105°
        # median gives stable wall distance
        # -------------------------------------------------

        left_distance = self.get_sector_distance(
            msg,
            75,
            105,
            use_min=False
        )

        # -------------------------------------------------
        # FRONT-LEFT
        # extra collision protection
        # -------------------------------------------------

        front_left_distance = self.get_sector_distance(
            msg,
            20,
            70,
            use_min=True
        )

        # Front data is mandatory
        if front_distance is None:
            self.get_logger().warn(
                'No valid front LiDAR data -> STOP'
            )
            self.stop_robot()
            return

        # -------------------------------------------------
        # PRINT VALUES
        # -------------------------------------------------

        left_text = (
            f'{left_distance:.2f}'
            if left_distance is not None
            else 'N/A'
        )

        front_left_text = (
            f'{front_left_distance:.2f}'
            if front_left_distance is not None
            else 'N/A'
        )

        self.get_logger().info(
            f'LEFT={left_text} m | '
            f'FRONT_LEFT={front_left_text} m | '
            f'FRONT={front_distance:.2f} m'
        )

        # =================================================
        # DECISION 1:
        # WALL / OBSTACLE DIRECTLY IN FRONT
        #
        # Stop and make exact 90° RIGHT turn
        # =================================================

        if front_distance < self.front_stop_distance:

            self.get_logger().warn(
                'Front wall detected -> RIGHT 90 degrees'
            )

            self.stop_robot()

            self.wall_lost_counter = 0

            self.rotate_right_90()

            return

        # =================================================
        # CHECK WHETHER LEFT WALL EXISTS
        # =================================================

        left_wall_present = (
            left_distance is not None
            and left_distance
            <= self.left_wall_lost_distance
        )

        # If we see a wall, remember that we have found it
        if left_wall_present:

            self.left_wall_seen = True
            self.wall_lost_counter = 0

        # =================================================
        # DECISION 2:
        # LEFT WALL HAS FINISHED
        #
        # Only do this if we have previously seen a wall.
        # =================================================

        elif self.left_wall_seen:

            self.wall_lost_counter += 1

            self.get_logger().info(
                f'Left wall missing: '
                f'{self.wall_lost_counter}/'
                f'{self.wall_lost_required_scans}'
            )

            if (
                self.wall_lost_counter
                >= self.wall_lost_required_scans
            ):

                self.get_logger().warn(
                    'Left wall ended -> LEFT 90 degrees'
                )

                self.stop_robot()

                self.wall_lost_counter = 0
                self.left_wall_seen = False

                self.rotate_left_90()

                return

        # =================================================
        # NORMAL STATE:
        # KEEP MOVING STRAIGHT
        # =================================================

        self.move_forward()


    # =====================================================
    # FORWARD MOVEMENT
    # =====================================================

    def move_forward(self):

        cmd = Twist()

        cmd.linear.x = self.forward_speed
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)


    # =====================================================
    # STOP
    # =====================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)


    # =====================================================
    # TURN LEFT 90°
    # =====================================================

    def rotate_left_90(self):

        goal = RotateAngle.Goal()

        goal.angle = math.radians(
            self.turn_angle_deg
        )

        goal.max_rotation_speed = (
            self.rotation_speed
        )

        self.send_rotation_goal(goal)


    # =====================================================
    # TURN RIGHT 90°
    # =====================================================

    def rotate_right_90(self):

        goal = RotateAngle.Goal()

        goal.angle = -math.radians(
            self.turn_angle_deg
        )

        goal.max_rotation_speed = (
            self.rotation_speed
        )

        self.send_rotation_goal(goal)


    # =====================================================
    # SEND ROTATION ACTION
    # =====================================================

    def send_rotation_goal(self, goal):

        self.turning = True

        if not self.rotate_client.wait_for_server(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                'RotateAngle action server not available'
            )

            self.turning = False
            return

        future = self.rotate_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.rotation_goal_response
        )


    # =====================================================
    # ROTATION GOAL ACCEPTED?
    # =====================================================

    def rotation_goal_response(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Rotation goal rejected'
            )

            self.turning = False
            return

        self.get_logger().info(
            'Rotation started'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.rotation_finished
        )


    # =====================================================
    # ROTATION FINISHED
    # =====================================================

    def rotation_finished(self, future):

        self.get_logger().info(
            '90 degree rotation finished -> forward'
        )

        self.turning = False

        # Immediately continue moving
        self.move_forward()


    # =====================================================
    # GET DISTANCE FROM LIDAR SECTOR
    # =====================================================

    def get_sector_distance(
        self,
        msg,
        start_angle_deg,
        end_angle_deg,
        use_min=False
    ):

        valid_ranges = []

        for i, distance in enumerate(msg.ranges):

            angle_rad = (
                msg.angle_min
                + i * msg.angle_increment
            )

            angle_deg = (
                math.degrees(angle_rad)
                + 180.0
            ) % 360.0 - 180.0

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

                    valid_ranges.append(
                        distance
                    )

        if not valid_ranges:
            return None

        # Obstacle detection
        if use_min:
            return min(valid_ranges)

        # Wall measurement
        return statistics.median(
            valid_ranges
        )


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = WallFollowerV2()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()