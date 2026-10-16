#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from action_msgs.msg import GoalStatus

from irobot_create_msgs.action import (
    DriveDistance,
    RotateAngle,
    DriveArc
)


# ==========================================================
# SETTINGS
# ==========================================================

FORWARD_SPEED = 0.08
TURN_SPEED = 0.30
ARC_SPEED = 0.08

ACTION_TIMEOUT = 15.0


class MovementController(Node):

    def __init__(self):

        super().__init__('movement_controller')

        # --------------------------------------------------
        # Commands from Pledge / navigation node
        #
        # Examples:
        # FORWARD 0.30
        # LEFT 90
        # RIGHT 90
        # CURVE_LEFT 45 0.40
        # CURVE_RIGHT 45 0.40
        # STOP
        # --------------------------------------------------

        self.command_subscriber = self.create_subscription(
            String,
            '/movement_command',
            self.command_callback,
            10
        )

        # --------------------------------------------------
        # Status back to Pledge
        #
        # BUSY
        # DONE
        # STOPPED
        # BLOCKED
        # ERROR
        # --------------------------------------------------

        self.status_publisher = self.create_publisher(
            String,
            '/movement_status',
            10
        )

        # Straight movement
        self.drive_client = ActionClient(
            self,
            DriveDistance,
            '/robot_8/drive_distance'
        )

        # Rotation
        self.rotate_client = ActionClient(
            self,
            RotateAngle,
            '/robot_8/rotate_angle'
        )

        # Curved movement
        self.arc_client = ActionClient(
            self,
            DriveArc,
            '/robot_8/drive_arc'
        )

        # Commands waiting to run
        self.command_queue = deque()

        # Current movement state
        self.action_active = False
        self.goal_handle = None

        # Current command, for logging/debugging
        self.current_command = None

        # Time when current action started
        self.action_start_time = None

        # Reason for cancelling:
        # None / STOP / TIMEOUT
        self.cancel_reason = None

        # Prevent duplicate cancel requests
        self.cancel_in_progress = False

        # Check periodically for actions taking too long
        self.timeout_timer = self.create_timer(
            0.5,
            self.check_action_timeout
        )

        self.get_logger().info(
            'MovementController started'
        )


    # ======================================================
    # STATUS
    # ======================================================

    def publish_status(self, status):

        msg = String()
        msg.data = status

        self.status_publisher.publish(msg)

        self.get_logger().info(
            f'Status: {status}'
        )


    # ======================================================
    # RECEIVE COMMAND
    # ======================================================

    def command_callback(self, msg):

        command = msg.data.strip().upper()

        self.get_logger().info(
            f'Command received: {command}'
        )

        # STOP has highest priority
        if command == 'STOP':

            # Remove commands that have not started yet
            self.command_queue.clear()

            self.stop_robot()
            return

        # Store command
        self.command_queue.append(command)

        self.get_logger().info(
            f'Command queued: {command}'
        )

        # Start immediately if robot is free
        if not self.action_active:

            self.execute_next_command()


    # ======================================================
    # EXECUTE NEXT COMMAND
    # ======================================================

    def execute_next_command(self):

        if self.action_active:
            return

        if len(self.command_queue) == 0:
            return

        command = self.command_queue.popleft()

        parts = command.split()

        if len(parts) == 0:
            self.invalid_command(command)
            return

        action = parts[0]

        future = None


        # --------------------------------------------------
        # FORWARD
        #
        # Example:
        # FORWARD 0.30
        # --------------------------------------------------

        if action == 'FORWARD':

            if len(parts) != 2:
                self.invalid_command(command)
                return

            try:
                distance = float(parts[1])

            except ValueError:
                self.invalid_command(command)
                return

            if distance <= 0:
                self.invalid_command(command)
                return

            future = self.drive_forward(
                distance=distance,
                speed=FORWARD_SPEED
            )


        # --------------------------------------------------
        # LEFT
        #
        # Example:
        # LEFT 90
        # --------------------------------------------------

        elif action == 'LEFT':

            if len(parts) != 2:
                self.invalid_command(command)
                return

            try:
                angle = float(parts[1])

            except ValueError:
                self.invalid_command(command)
                return

            if angle <= 0:
                self.invalid_command(command)
                return

            future = self.rotate_left(
                angle_deg=angle,
                speed=TURN_SPEED
            )


        # --------------------------------------------------
        # RIGHT
        #
        # Example:
        # RIGHT 90
        # --------------------------------------------------

        elif action == 'RIGHT':

            if len(parts) != 2:
                self.invalid_command(command)
                return

            try:
                angle = float(parts[1])

            except ValueError:
                self.invalid_command(command)
                return

            if angle <= 0:
                self.invalid_command(command)
                return

            future = self.rotate_right(
                angle_deg=angle,
                speed=TURN_SPEED
            )


        # --------------------------------------------------
        # CURVE LEFT
        #
        # Example:
        # CURVE_LEFT 45 0.40
        #
        # 45   = angle in degrees
        # 0.40 = radius in meters
        # --------------------------------------------------

        elif action == 'CURVE_LEFT':

            if len(parts) != 3:
                self.invalid_command(command)
                return

            try:
                angle = float(parts[1])
                radius = float(parts[2])

            except ValueError:
                self.invalid_command(command)
                return

            if angle <= 0 or radius <= 0:
                self.invalid_command(command)
                return

            future = self.curve_left(
                angle_deg=angle,
                radius=radius,
                speed=ARC_SPEED
            )


        # --------------------------------------------------
        # CURVE RIGHT
        #
        # Example:
        # CURVE_RIGHT 45 0.40
        # --------------------------------------------------

        elif action == 'CURVE_RIGHT':

            if len(parts) != 3:
                self.invalid_command(command)
                return

            try:
                angle = float(parts[1])
                radius = float(parts[2])

            except ValueError:
                self.invalid_command(command)
                return

            if angle <= 0 or radius <= 0:
                self.invalid_command(command)
                return

            future = self.curve_right(
                angle_deg=angle,
                radius=radius,
                speed=ARC_SPEED
            )


        # --------------------------------------------------
        # UNKNOWN COMMAND
        # --------------------------------------------------

        else:

            self.invalid_command(command)
            return


        # Action server was unavailable
        if future is None:

            self.command_queue.clear()
            self.publish_status('ERROR')
            return


        # Movement has started
        self.action_active = True

        self.current_command = command

        self.action_start_time = time.monotonic()

        self.cancel_reason = None
        self.cancel_in_progress = False

        self.get_logger().info(
            f'Starting: {self.current_command}'
        )

        self.publish_status('BUSY')

        future.add_done_callback(
            self.goal_response_callback
        )


    # ======================================================
    # INVALID COMMAND
    # ======================================================

    def invalid_command(self, command):

        self.get_logger().error(
            f'Invalid command: {command}'
        )

        self.command_queue.clear()

        self.publish_status('ERROR')


    # ======================================================
    # FORWARD
    # ======================================================

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

            self.get_logger().error(
                'DriveDistance server not available'
            )

            return None

        return self.drive_client.send_goal_async(goal)


    # ======================================================
    # LEFT ROTATION
    # ======================================================

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

            self.get_logger().error(
                'RotateAngle server not available'
            )

            return None

        return self.rotate_client.send_goal_async(goal)


    # ======================================================
    # RIGHT ROTATION
    # ======================================================

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

            self.get_logger().error(
                'RotateAngle server not available'
            )

            return None

        return self.rotate_client.send_goal_async(goal)


    # ======================================================
    # CURVE LEFT
    # ======================================================

    def curve_left(
        self,
        angle_deg,
        radius,
        speed
    ):

        goal = DriveArc.Goal()

        # 1 = drive forward
        goal.translate_direction = 1

        # Positive angle = left
        goal.angle = math.radians(angle_deg)

        goal.radius = radius

        goal.max_translation_speed = speed

        self.get_logger().info(
            f'Curving LEFT {angle_deg} degrees '
            f'with radius {radius} m'
        )

        if not self.arc_client.wait_for_server(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                'DriveArc server not available'
            )

            return None

        return self.arc_client.send_goal_async(goal)


    # ======================================================
    # CURVE RIGHT
    # ======================================================

    def curve_right(
        self,
        angle_deg,
        radius,
        speed
    ):

        goal = DriveArc.Goal()

        # 1 = drive forward
        goal.translate_direction = 1

        # Negative angle = right
        goal.angle = -math.radians(angle_deg)

        goal.radius = radius

        goal.max_translation_speed = speed

        self.get_logger().info(
            f'Curving RIGHT {angle_deg} degrees '
            f'with radius {radius} m'
        )

        if not self.arc_client.wait_for_server(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                'DriveArc server not available'
            )

            return None

        return self.arc_client.send_goal_async(goal)


    # ======================================================
    # GOAL ACCEPTED / REJECTED
    # ======================================================

    def goal_response_callback(self, future):

        try:

            self.goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Goal error: {error}'
            )

            self.finish_with_error()
            return


        if not self.goal_handle.accepted:

            self.get_logger().error(
                'Goal rejected'
            )

            self.finish_with_error()
            return


        self.get_logger().info(
            'Goal accepted'
        )


        # STOP or timeout may have happened
        # before the server accepted the goal
        if self.cancel_reason is not None:

            self.send_cancel_request()


        result_future = (
            self.goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.goal_result_callback
        )


    # ======================================================
    # FINAL ACTION RESULT
    # ======================================================

    def goal_result_callback(self, future):

        try:

            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Result error: {error}'
            )

            self.finish_with_error()
            return


        finished_command = self.current_command

        cancel_reason = self.cancel_reason

        self.action_active = False
        self.goal_handle = None
        self.action_start_time = None
        self.current_command = None
        self.cancel_in_progress = False
        self.cancel_reason = None


        # --------------------------------------------------
        # Successful movement
        # --------------------------------------------------

        if result.status == GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info(
                f'Finished: {finished_command}'
            )

            self.publish_status('DONE')

            self.execute_next_command()

            return


        # --------------------------------------------------
        # Manual STOP
        # --------------------------------------------------

        if (
            result.status == GoalStatus.STATUS_CANCELED
            and cancel_reason == 'STOP'
        ):

            self.publish_status('STOPPED')
            return


        # --------------------------------------------------
        # Timeout / probably blocked
        # --------------------------------------------------

        if (
            result.status == GoalStatus.STATUS_CANCELED
            and cancel_reason == 'TIMEOUT'
        ):

            self.publish_status('BLOCKED')
            return


        # --------------------------------------------------
        # Other failure
        # --------------------------------------------------

        self.command_queue.clear()

        self.publish_status('ERROR')


    # ======================================================
    # STOP
    # ======================================================

    def stop_robot(self):

        self.get_logger().info(
            'STOP command received'
        )

        self.command_queue.clear()


        # Robot is already idle
        if not self.action_active:

            self.publish_status('STOPPED')
            return


        self.cancel_reason = 'STOP'

        self.send_cancel_request()


    # ======================================================
    # TIMEOUT CHECK
    # ======================================================

    def check_action_timeout(self):

        if not self.action_active:
            return

        if self.action_start_time is None:
            return

        # Already cancelling for STOP or timeout
        if self.cancel_reason is not None:
            return

        elapsed = (
            time.monotonic()
            - self.action_start_time
        )

        if elapsed < ACTION_TIMEOUT:
            return


        self.get_logger().warning(
            f'Action timeout after {elapsed:.1f}s: '
            f'{self.current_command}'
        )

        # Do not continue queued path after timeout
        self.command_queue.clear()

        self.cancel_reason = 'TIMEOUT'

        self.send_cancel_request()


    # ======================================================
    # SEND CANCEL REQUEST
    # ======================================================

    def send_cancel_request(self):

        # Goal has not been accepted yet.
        # goal_response_callback will try again.
        if self.goal_handle is None:
            return

        if self.cancel_in_progress:
            return

        self.cancel_in_progress = True

        cancel_future = (
            self.goal_handle.cancel_goal_async()
        )

        cancel_future.add_done_callback(
            self.cancel_response_callback
        )


    # ======================================================
    # CANCEL RESPONSE
    # ======================================================

    def cancel_response_callback(self, future):

        try:

            cancel_response = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Cancel error: {error}'
            )

            self.cancel_in_progress = False
            return


        if len(
            cancel_response.goals_canceling
        ) > 0:

            self.get_logger().info(
                'Cancel request accepted'
            )

            # Final STOPPED / BLOCKED status
            # will come from goal_result_callback
            return


        self.get_logger().warning(
            'Cancel request was not accepted'
        )

        self.cancel_in_progress = False


    # ======================================================
    # GENERAL ERROR CLEANUP
    # ======================================================

    def finish_with_error(self):

        self.action_active = False
        self.goal_handle = None
        self.current_command = None
        self.action_start_time = None

        self.cancel_reason = None
        self.cancel_in_progress = False

        self.command_queue.clear()

        self.publish_status('ERROR')


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = MovementController()

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