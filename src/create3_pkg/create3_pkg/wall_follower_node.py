#!/usr/bin/env python3
"""ROS 2 node: LiDAR-based wall following for the iRobot Create 3.

Subscribes
    <scan_topic>      sensor_msgs/LaserScan   (default /robot_8/scan)
Publishes
    <cmd_vel_topic>   geometry_msgs/Twist     (default /robot_8/cmd_vel)
    ~/distances       std_msgs/Float32MultiArray
                      [front, left, right, front_left, front_right] in metres
    ~/situation       std_msgs/String  corridor | corner_left | corner_right |
                                       junction | dead_end
    ~/state           std_msgs/String  controller state (FOLLOW_WALL, ...)

The distances/situation topics are the perception output for the
navigation node. Set `drive:=false` to run perception only (robot stays still).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String

from create3_pkg.controller import (Command, ControllerParams,
                                    WallFollowController)
from create3_pkg.lidar_processing import (LidarProcessor, ScanConfig,
                                          describe_surroundings)


class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # ---------------- parameters (override in config/wall_follower.yaml)
        self.declare_parameter('scan_topic', '/robot_8/scan')
        self.declare_parameter('cmd_vel_topic', '/robot_8/cmd_vel')
        self.declare_parameter('drive', True)          # False = perception only
        self.declare_parameter('control_rate', 20.0)   # Hz, cmd_vel publishing
        self.declare_parameter('scan_timeout', 0.5)    # s without scan -> stop
        self.declare_parameter('follow_side', 'right')  # 'right' or 'left'
        self.declare_parameter('lidar_angle_offset_deg', 0.0)
        self.declare_parameter('lidar_reversed', False)
        self.declare_parameter('lidar_median_filter', 3)
        self.declare_parameter('opening_distance', 0.50)

        defaults = ControllerParams()
        self.declare_parameter('desired_distance', defaults.desired_distance)
        self.declare_parameter('look_ahead', defaults.look_ahead)
        self.declare_parameter('beam_angle_deg', math.degrees(defaults.beam_angle))
        self.declare_parameter('kp', defaults.kp)
        self.declare_parameter('kd', defaults.kd)
        self.declare_parameter('max_linear', defaults.max_linear)
        self.declare_parameter('min_linear', defaults.min_linear)
        self.declare_parameter('max_angular', defaults.max_angular)
        self.declare_parameter('front_stop', defaults.front_stop)
        self.declare_parameter('front_clear', defaults.front_clear)
        self.declare_parameter('front_slow', defaults.front_slow)
        self.declare_parameter('wall_lost_distance', defaults.wall_lost_distance)
        self.declare_parameter('turn_angular', defaults.turn_angular)
        self.declare_parameter('corner_linear', defaults.corner_linear)
        self.declare_parameter('corner_angular', defaults.corner_angular)
        self.declare_parameter('corner_advance', defaults.corner_advance)
        self.declare_parameter('corner_timeout', defaults.corner_timeout)

        g = self._param
        side = g('follow_side').lower()
        if side not in ('left', 'right'):
            raise ValueError("follow_side must be 'left' or 'right'")

        self.lidar = LidarProcessor(ScanConfig(
            angle_offset=math.radians(g('lidar_angle_offset_deg')),
            reversed=g('lidar_reversed'),
            median_filter_size=g('lidar_median_filter'),
        ))
        self.controller = WallFollowController(ControllerParams(
            side=+1 if side == 'left' else -1,
            desired_distance=g('desired_distance'),
            look_ahead=g('look_ahead'),
            beam_angle=math.radians(g('beam_angle_deg')),
            kp=g('kp'), kd=g('kd'),
            max_linear=g('max_linear'), min_linear=g('min_linear'),
            max_angular=g('max_angular'),
            front_stop=g('front_stop'), front_clear=g('front_clear'),
            front_slow=g('front_slow'),
            wall_lost_distance=g('wall_lost_distance'),
            turn_angular=g('turn_angular'),
            corner_linear=g('corner_linear'),
            corner_angular=g('corner_angular'),
            corner_advance=g('corner_advance'),
            corner_timeout=g('corner_timeout'),
        ))
        self.drive = g('drive')
        self.opening_distance = g('opening_distance')
        self.scan_timeout = g('scan_timeout')

        # ---------------- ROS interfaces
        # Sensor-data QoS (best effort) works with both reliable and
        # best-effort LiDAR publishers.
        self.create_subscription(LaserScan, g('scan_topic'), self._on_scan,
                                 qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.dist_pub = self.create_publisher(Float32MultiArray, '~/distances', 10)
        self.situation_pub = self.create_publisher(String, '~/situation', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)

        # cmd_vel is re-sent at a fixed rate; the Create 3 stops if commands
        # stop arriving, so this also keeps motion smooth between scans.
        self.command = Command(0.0, 0.0)
        self.last_scan_time = None
        self.create_timer(1.0 / g('control_rate'), self._on_timer)
        self.create_timer(0.5, self._log_status)

        self.get_logger().info(
            f"Wall follower ready: following {side} wall, "
            f"scan={g('scan_topic')}, cmd_vel={g('cmd_vel_topic')}, "
            f"drive={self.drive}")

    def _param(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------- callbacks
    def _on_scan(self, msg: LaserScan):
        self.lidar.update(msg.ranges, msg.angle_min, msg.angle_increment,
                          msg.range_min, msg.range_max)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.last_scan_time = now
        if not self.lidar.has_data:
            self.command = Command(0.0, 0.0)
            return

        self.command = self.controller.compute(self.lidar, now)
        self._publish_perception()

    def _on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        scan_fresh = (self.last_scan_time is not None
                      and now - self.last_scan_time < self.scan_timeout)
        if not scan_fresh:
            self.command = Command(0.0, 0.0)  # safety: no LiDAR -> stop
        if self.drive:
            self._send(self.command)

    def _publish_perception(self):
        s = describe_surroundings(self.lidar, self.opening_distance)
        self.dist_pub.publish(Float32MultiArray(data=[
            float(v) for v in (s.front, s.left, s.right,
                               s.front_left, s.front_right)]))
        self.situation_pub.publish(String(data=s.situation))
        self.state_pub.publish(String(data=self.controller.state.value))

    def _log_status(self):
        if not self.lidar.has_data:
            self.get_logger().warn('Waiting for LiDAR data...',
                                   throttle_duration_sec=2.0)
            return
        s = describe_surroundings(self.lidar, self.opening_distance)
        self.get_logger().info(
            f'{self.controller.state.value:<11} | F {s.front:4.2f}  '
            f'L {s.left:4.2f}  R {s.right:4.2f} | {s.situation:<12} | '
            f'v {self.command.linear:+.2f}  w {self.command.angular:+.2f}')

    def _send(self, command: Command):
        twist = Twist()
        twist.linear.x = float(command.linear)
        twist.angular.z = float(command.angular)
        self.cmd_pub.publish(twist)

    def stop(self):
        self._send(Command(0.0, 0.0))


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()  # make sure the robot stops on Ctrl+C
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
