#!/usr/bin/env python3
"""
ROS 2 node: LiDAR-based wall following for the iRobot Create 3.

Subscribes
    <scan_topic>      sensor_msgs/LaserScan   (default /robot_8/scan)
    <pose_topic>      geometry_msgs/PoseStamped  robot pose from the
                      localization node (default /robot_8/localization/pose,
                      only with use_goal:=true)
Publishes
    <cmd_vel_topic>   geometry_msgs/Twist     (default /robot_8/cmd_vel)
    ~/distances       std_msgs/Float32MultiArray
                      [front, left, right, front_left, front_right] in metres,
                      measured from the robot centre
    ~/situation       std_msgs/String  corridor | corner_left | corner_right |
                                       junction | dead_end
    ~/state           std_msgs/String  controller state (FOLLOW_WALL, ...;
                                       GO_TO_GOAL, GOAL_REACHED, ... with
                                       use_goal:=true)

The distances/situation topics are the perception output for the
navigation node. Set `drive:=false` to run perception only (robot stays still).

Set `use_goal:=true` to drive to a goal with DistBug (see goal_controller)
instead of following the wall forever. The goal is given before the start,
relative to the start pose: `goal_x` metres forward, `goal_y` metres left.
With `auto_side:=true` (default) the side used to go round each obstacle is
chosen from the LiDAR scan; `follow_side` is then only the fallback.
`on_loop:=retry` (default): after a full lap round an obstacle the robot
tries the other way round, then explores; `on_loop:=stop` stops with
GOAL UNREACHABLE instead.
"""

import math

from create3_pkg.controller import (Command, ControllerParams,
                                    WallFollowController)
from create3_pkg.goal_controller import (GoalParams, GoalState,
                                         GoToGoalController)
from create3_pkg.lidar_processing import (describe_surroundings,
                                          LidarProcessor, ScanConfig)
from create3_pkg.localization_node import yaw_from_quaternion
from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String


class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # ---------------- parameters (override with --ros-args -p name:=value)
        self.declare_parameter('scan_topic', '/robot_8/scan')
        self.declare_parameter('cmd_vel_topic', '/robot_8/cmd_vel')
        self.declare_parameter('drive', True)          # False = perception only
        self.declare_parameter('control_rate', 20.0)   # Hz, cmd_vel publishing
        self.declare_parameter('scan_timeout', 0.5)    # s without scan -> stop
        self.declare_parameter('follow_side', 'right')  # 'right' or 'left'
        self.declare_parameter('lidar_angle_offset_deg', 0.0)
        self.declare_parameter('lidar_reversed', False)
        self.declare_parameter('lidar_median_filter', 3)
        # Ignore parts of the robot seen by the LiDAR: flat list of
        # [from_deg, to_deg, max_range_m, ...] in raw LiDAR angles
        # (the 'raw angle' printed by lidar_check). Empty = off.
        self.declare_parameter('lidar_ignore_sectors', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('opening_distance', 0.50)

        # Go-to-goal (DistBug). Goal relative to the start pose:
        # goal_x m forward, goal_y m left.
        goal_defaults = GoalParams()
        self.declare_parameter('use_goal', False)
        self.declare_parameter('pose_topic', '/robot_8/localization/pose')
        self.declare_parameter('goal_x', goal_defaults.goal_x)
        self.declare_parameter('goal_y', goal_defaults.goal_y)
        self.declare_parameter('goal_tolerance', goal_defaults.goal_tolerance)
        self.declare_parameter('auto_side', goal_defaults.auto_side)
        self.declare_parameter('on_loop', goal_defaults.on_loop)

        # Robot geometry: 35 cm round body, LiDAR 20 cm behind the front edge.
        defaults = ControllerParams()
        self.declare_parameter('robot_radius', defaults.robot_radius)
        self.declare_parameter('lidar_to_front', 0.20)

        self.declare_parameter('wall_clearance', defaults.wall_clearance)
        self.declare_parameter('follow_range', defaults.follow_range)
        self.declare_parameter('approach_angle_deg',
                               math.degrees(defaults.approach_angle))
        self.declare_parameter('k_distance', defaults.k_distance)
        self.declare_parameter('k_bearing', defaults.k_bearing)
        self.declare_parameter('max_linear', defaults.max_linear)
        self.declare_parameter('min_linear', defaults.min_linear)
        self.declare_parameter('corner_linear', defaults.corner_linear)
        self.declare_parameter('max_angular', defaults.max_angular)
        self.declare_parameter('turn_angular', defaults.turn_angular)
        self.declare_parameter('side_margin', defaults.side_margin)
        self.declare_parameter('front_stop', defaults.front_stop)
        self.declare_parameter('front_clear', defaults.front_clear)
        self.declare_parameter('front_slow', defaults.front_slow)

        g = self._param
        side = g('follow_side').lower()
        if side not in ('left', 'right'):
            raise ValueError("follow_side must be 'left' or 'right'")

        ignore = list(self.get_parameter_or(
            'lidar_ignore_sectors',
            Parameter('lidar_ignore_sectors', Parameter.Type.DOUBLE_ARRAY, [])).value)
        if len(ignore) % 3:
            raise ValueError('lidar_ignore_sectors needs triples: '
                             '[from_deg, to_deg, max_range_m, ...]')
        ignore_sectors = tuple(
            (math.radians(ignore[i]), math.radians(ignore[i + 1]), ignore[i + 2])
            for i in range(0, len(ignore), 3))

        self.lidar = LidarProcessor(ScanConfig(
            angle_offset=math.radians(g('lidar_angle_offset_deg')),
            scan_reversed=g('lidar_reversed'),
            lidar_x=g('robot_radius') - g('lidar_to_front'),
            ignore_sectors=ignore_sectors,
            median_filter_size=g('lidar_median_filter'),
        ))
        self.controller = WallFollowController(ControllerParams(
            side=+1 if side == 'left' else -1,
            robot_radius=g('robot_radius'),
            wall_clearance=g('wall_clearance'),
            follow_range=g('follow_range'),
            approach_angle=math.radians(g('approach_angle_deg')),
            k_distance=g('k_distance'), k_bearing=g('k_bearing'),
            max_linear=g('max_linear'), min_linear=g('min_linear'),
            corner_linear=g('corner_linear'),
            max_angular=g('max_angular'), turn_angular=g('turn_angular'),
            side_margin=g('side_margin'),
            front_stop=g('front_stop'), front_clear=g('front_clear'),
            front_slow=g('front_slow'),
        ))
        self.drive = g('drive')
        self.goal = None
        if g('use_goal'):
            if g('on_loop') not in ('retry', 'stop'):
                raise ValueError("on_loop must be 'retry' or 'stop'")
            self.goal = GoToGoalController(GoalParams(
                goal_x=g('goal_x'), goal_y=g('goal_y'),
                goal_tolerance=g('goal_tolerance'),
                auto_side=g('auto_side'),
                on_loop=g('on_loop'),
            ), self.controller)
        self.pose = None             # (x, y, yaw) from the localization node
        self.last_pose_time = None
        self.goal_logged = False     # 'goal reached' printed once
        self.laps_logged = 0         # laps already reported
        self.opening_distance = g('opening_distance')
        self.scan_timeout = g('scan_timeout')

        # ---------------- ROS interfaces
        # Sensor-data QoS (best effort) works with both reliable and
        # best-effort LiDAR publishers.
        self.create_subscription(LaserScan, g('scan_topic'), self._on_scan,
                                 qos_profile_sensor_data)
        if self.goal is not None:
            self.create_subscription(PoseStamped, g('pose_topic'),
                                     self._on_pose, 10)
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

        scan_topic, cmd_vel_topic = g('scan_topic'), g('cmd_vel_topic')
        self.get_logger().info(
            f'Wall follower ready: following {side} wall at '
            f'{self.controller.p.desired_distance:.3f} m from the centre, '
            f'scan={scan_topic}, cmd_vel={cmd_vel_topic}, '
            f'drive={self.drive}')
        if self.goal is not None:
            goal_x, goal_y, auto_side = g('goal_x'), g('goal_y'), g('auto_side')
            on_loop = g('on_loop')
            self.get_logger().info(
                f'Go-to-goal: goal {goal_x:.2f} m forward, {goal_y:.2f} m '
                f'left of the start pose (fixed at the first pose message), '
                f'auto_side={auto_side}, on_loop={on_loop}')

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

        if self.goal is None:
            self.command = self.controller.compute(self.lidar, now)
        elif self.pose is None:
            self.command = Command(0.0, 0.0)  # wait for the pose
        else:
            self.command = self.goal.compute(self.lidar, *self.pose, now)
            self._log_laps()
            self._log_goal_finished()
        self._publish_perception()

    def _on_pose(self, msg: PoseStamped):
        pose = msg.pose
        self.pose = (pose.position.x, pose.position.y,
                     yaw_from_quaternion(pose.orientation))
        self.last_pose_time = self.get_clock().now().nanoseconds * 1e-9
        if not self.goal.started:
            self.goal.set_start(*self.pose)
            gx, gy = self.goal.goal
            self.get_logger().info(
                f'Start pose fixed; goal at ({gx:.2f}, {gy:.2f}) in the '
                f"'{msg.header.frame_id}' frame")

    def _on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        scan_fresh = (self.last_scan_time is not None
                      and now - self.last_scan_time < self.scan_timeout)
        if not scan_fresh:
            self.command = Command(0.0, 0.0)  # safety: no LiDAR -> stop
        if self.goal is not None and (
                self.last_pose_time is None
                or now - self.last_pose_time >= self.scan_timeout):
            self.command = Command(0.0, 0.0)  # safety: no pose -> stop
        if self.drive:
            self._send(self.command)

    def _publish_perception(self):
        s = describe_surroundings(self.lidar, self.opening_distance)
        self.dist_pub.publish(Float32MultiArray(data=[
            float(v) for v in (s.front, s.left, s.right,
                               s.front_left, s.front_right)]))
        self.situation_pub.publish(String(data=s.situation))
        state = self.controller.state.value
        if self.goal is not None:
            state = self.goal.state.value
            if self.goal.following:
                state += f'({self._side_name()})/{self.controller.state.value}'
        self.state_pub.publish(String(data=state))

    def _side_name(self):
        return 'left' if self.controller.p.side > 0 else 'right'

    def _log_laps(self):
        laps = self.goal.laps
        if laps > self.laps_logged:
            if self.goal.state == GoalState.EXPLORE:
                self.get_logger().warn(
                    'Lap round the obstacle on both sides: exploring '
                    '(following walls until the goal is passed)')
            elif not self.goal.finished:
                self.get_logger().warn(
                    'Lap round the obstacle without a way to the goal: '
                    f'trying the other way ({self._side_name()} wall)')
        self.laps_logged = laps

    def _log_goal_finished(self):
        if self.goal.finished and not self.goal_logged:
            self.goal_logged = True
            if self.goal.state == GoalState.GOAL_REACHED:
                self.get_logger().info(
                    f'GOAL REACHED ({self.goal.distance_to_goal:.2f} m away)')
            else:
                self.get_logger().error(
                    'GOAL UNREACHABLE: went round the obstacle back to the '
                    'hit point, stopping')

    def _log_status(self):
        if not self.lidar.has_data:
            self.get_logger().warn('Waiting for LiDAR data...',
                                   throttle_duration_sec=2.0)
            return
        s = describe_surroundings(self.lidar, self.opening_distance)
        self.get_logger().info(
            f'{self.controller.state.value:<11} | F {s.front:4.2f}  '
            f'L {s.left:4.2f}  R {s.right:4.2f} | {s.situation:<12} | '
            f'v {self.command.linear:+.2f}  w {self.command.angular:+.2f}'
            + ('' if self.goal is None else
               f' | {self.goal.state.value}'
               + (f'({self._side_name()})' if self.goal.following else '')
               + f' goal {self.goal.distance_to_goal:.2f} m'))

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
