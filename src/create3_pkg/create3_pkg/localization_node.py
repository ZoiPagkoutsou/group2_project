#!/usr/bin/env python3
"""
ROS 2 node: localization for the iRobot Create 3 (robot_8).

Turns the robot's wheel odometry into the robot pose relative to where it
started, the 'start frame': x metres forward and y metres left of the start
position, yaw measured from the start heading (ROS REP-103). This is the pose
the go-to-goal navigation (wall_follower_node with use_goal:=true) drives
with.

Subscribes
    <odom_topic>   nav_msgs/Odometry          (default /robot_8/odom)
Publishes
    ~/pose         geometry_msgs/PoseStamped  pose in the start frame,
                   frame_id 'start', one message per odometry message,
                   stamped with the odometry time
    ~/heading      std_msgs/Float64           yaw in the start frame (rad)
    ~/distance     std_msgs/Float64           path length driven so far (m)

The start pose is fixed by the first odometry message, so start this node
with the robot at the start point (go_to_goal.launch.py does).
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    """Yaw (rotation about z, rad) of a geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    """(x, y, z, w) of a pure rotation about z."""
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class StartFrame:
    """Converts odometry poses into poses relative to the start pose."""

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self._c, self._s = math.cos(yaw), math.sin(yaw)

    def to_start(self, x, y, yaw):
        """Odometry pose -> (forward, left, yaw) relative to the start."""
        dx, dy = x - self.x, y - self.y
        return (self._c * dx + self._s * dy,
                -self._s * dx + self._c * dy,
                wrap_angle(yaw - self.yaw))


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization')
        self.declare_parameter('odom_topic', '/robot_8/odom')
        self.declare_parameter('frame_id', 'start')
        odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.start = None            # StartFrame, fixed by the first message
        self.pose = None             # latest (x, y, yaw) in the start frame
        self.last_xy = None
        self.distance = 0.0

        # Create 3 sensor topics are best effort; this QoS also works with a
        # reliable publisher.
        self.create_subscription(Odometry, odom_topic, self._on_odom,
                                 qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, '~/pose', 10)
        self.heading_pub = self.create_publisher(Float64, '~/heading', 10)
        self.distance_pub = self.create_publisher(Float64, '~/distance', 10)
        self.create_timer(1.0, self._log_status)

        self.get_logger().info(
            f'Localization ready: odometry from {odom_topic}, pose in the '
            f"'{self.frame_id}' frame (start pose = first odometry message)")

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation)
        if self.start is None:
            self.start = StartFrame(p.position.x, p.position.y, yaw)
            self.get_logger().info(
                f'Start pose fixed at odom ({p.position.x:.2f}, '
                f'{p.position.y:.2f}, {math.degrees(yaw):.0f} deg)')
        x, y, rel_yaw = self.start.to_start(p.position.x, p.position.y, yaw)
        if self.last_xy is not None:
            self.distance += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        self.pose = (x, y, rel_yaw)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.pose.position.x = x
        out.pose.position.y = y
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = quaternion_from_yaw(rel_yaw)
        self.pose_pub.publish(out)
        self.heading_pub.publish(Float64(data=rel_yaw))
        self.distance_pub.publish(Float64(data=self.distance))

    def _log_status(self):
        if self.pose is None:
            self.get_logger().warn('Waiting for odometry...',
                                   throttle_duration_sec=2.0)
            return
        x, y, yaw = self.pose
        self.get_logger().info(
            f'x {x:+.2f} m (forward)  y {y:+.2f} m (left)  '
            f'yaw {math.degrees(yaw):+5.0f} deg  driven {self.distance:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
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
