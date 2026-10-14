#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from action_msgs.msg import GoalStatus

from irobot_create_msgs.action import DriveDistance, RotateAngle


class MovementController(Node):

    def __init__(self):
        super().__init__('movement_controller')

        self.command_subscriber = self.create_subscription(
            String,
            '/movement_command',
            self.command_callback,
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            '/movement_status',
            10
        )

        self.drive_client = ActionClient(
            self,
            DriveDistance,
            '/robot_8/drive_distance'
        )

        self.rotate_client = ActionClient(
            self,
            RotateAngle,
            '/robot_8/rotate_angle'
        )

        self.command_queue = deque()

        self.action_active = False
        self.goal_handle = None
        self.stop_requested = False

        self.get_logger().info(
            'Movement controller started'
        )


    def publish_status(self, status):

        msg = String()
        msg.data = status

        self.status_publisher.publish(msg)

        self.get_logger().info(
            f'Status: {status}'
        )


    def command_callback(self, msg):

        command = msg.data.strip().upper()

        self.get_logger().info(
            f'Command received: {command}'
        )

        # STOP has highest priority
        if command == 'STOP':

            self.command_queue.clear()

            self.stop_robot()
            return

        # Store command in queue
        self.command_queue.append(command)

        self.get_logger().info(
            f'Command queued: {command}'
        )

        # If robot is free, start immediately
        if not self.action_active:
            self.execute_next_command()


    def execute_next_command(self):

        if self.action_active:
            return

        if len(self.command_queue) == 0:
            return

        command = self.command_queue.popleft()

        parts = command.split()

        if len(parts) != 2:
            self.get_logger().error(
                f'Invalid command: {command}'
            )

            self.publish_status('ERROR')
            self.command_queue.clear()
            return

        action = parts[0]

        try:
            value = float(parts[1])

        except ValueError:
            self.get_logger().error(
                'Command value must be a number'
            )

            self.publish_status('ERROR')
            self.command_queue.clear()
            return


        if action == 'FORWARD':

            future = self.drive_forward(
                distance=value,
                speed=0.1
            )

        elif action == 'LEFT':

            future = self.rotate_left(
                angle_deg=value,
                speed=0.5
            )

        elif action == 'RIGHT':

            future = self.rotate_right(
                angle_deg=value,
                speed=0.5
            )

        else:

            self.get_logger().error(
                f'Unknown command: {command}'
            )

            self.publish_status('ERROR')
            self.command_queue.clear()
            return


        if future is None:
            self.publish_status('ERROR')
            self.command_queue.clear()
            return


        self.action_active = True
        self.stop_requested = False

        self.publish_status('BUSY')

        future.add_done_callback(
            self.goal_response_callback
        )


    def drive_forward(self, distance, speed):

        goal = DriveDistance.Goal()

        goal.distance = distance
        goal.max_translation_speed = speed

        self.get_logger().info(
            f'Driving forward {distance} m'
        )

        if not self.drive_client.wait_for_server(
            timeout_sec=2.0
        ):
            return None

        return self.drive_client.send_goal_async(goal)


    def rotate_left(self, angle_deg, speed):

        goal = RotateAngle.Goal()

        goal.angle = math.radians(angle_deg)
        goal.max_rotation_speed = speed

        self.get_logger().info(
            f'Rotating LEFT {angle_deg} degrees'
        )

        if not self.rotate_client.wait_for_server(
            timeout_sec=2.0
        ):
            return None

        return self.rotate_client.send_goal_async(goal)


    def rotate_right(self, angle_deg, speed):

        goal = RotateAngle.Goal()

        goal.angle = -math.radians(angle_deg)
        goal.max_rotation_speed = speed

        self.get_logger().info(
            f'Rotating RIGHT {angle_deg} degrees'
        )

        if not self.rotate_client.wait_for_server(
            timeout_sec=2.0
        ):
            return None

        return self.rotate_client.send_goal_async(goal)


    def goal_response_callback(self, future):

        try:
            self.goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Goal error: {error}'
            )

            self.action_active = False
            self.goal_handle = None
            self.command_queue.clear()

            self.publish_status('ERROR')
            return


        if not self.goal_handle.accepted:

            self.action_active = False
            self.goal_handle = None
            self.command_queue.clear()

            self.publish_status('ERROR')
            return


        self.get_logger().info(
            'Goal accepted'
        )


        if self.stop_requested:

            cancel_future = (
                self.goal_handle.cancel_goal_async()
            )

            cancel_future.add_done_callback(
                self.cancel_response_callback
            )


        result_future = (
            self.goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.goal_result_callback
        )


    def goal_result_callback(self, future):

        try:
            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Result error: {error}'
            )

            self.action_active = False
            self.goal_handle = None
            self.stop_requested = False
            self.command_queue.clear()

            self.publish_status('ERROR')
            return


        self.action_active = False
        self.goal_handle = None


        if (
            self.stop_requested
            and result.status == GoalStatus.STATUS_CANCELED
        ):

            self.stop_requested = False

            self.publish_status('STOPPED')
            return


        self.stop_requested = False


        if result.status == GoalStatus.STATUS_SUCCEEDED:

            self.publish_status('DONE')

            # Automatically execute next queued command
            self.execute_next_command()

        else:

            self.command_queue.clear()
            self.publish_status('ERROR')


    def stop_robot(self):

        self.get_logger().info(
            'STOP command received'
        )

        self.stop_requested = True


        if self.goal_handle is None:

            if not self.action_active:

                self.stop_requested = False
                self.publish_status('STOPPED')

            return


        cancel_future = (
            self.goal_handle.cancel_goal_async()
        )

        cancel_future.add_done_callback(
            self.cancel_response_callback
        )


    def cancel_response_callback(self, future):

        try:
            cancel_response = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Cancel error: {error}'
            )

            self.stop_requested = False
            self.publish_status('ERROR')
            return


        if len(cancel_response.goals_canceling) > 0:

            self.get_logger().info(
                'Cancel request accepted'
            )

            return


        self.stop_requested = False

        self.publish_status('ERROR')


def main(args=None):

    rclpy.init(args=args)

    node = MovementController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()