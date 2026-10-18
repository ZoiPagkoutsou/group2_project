#!/usr/bin/env python3

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from irobot_create_msgs.action import DriveDistance, RotateAngle
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, HazardDetection


# Movement values used when Navigation sends a command
FORWARD_DISTANCE = 0.20      # meters
FORWARD_SPEED = 0.08         # m/s
TURN_ANGLE = 90.0            # degrees
TURN_AROUND_ANGLE = 180.0    # degrees
TURN_SPEED = 0.30            # rad/s

# Default IR threshold. Tune on the real robot with:
#   ros2 run <pkg> <node> --ros-args -p ir_threshold:=500
DEFAULT_IR_THRESHOLD = 1000

# If no IR message arrives for this long, FORWARD is blocked.
IR_TIMEOUT_SEC = 1.0

# A bump message after this gap counts as a new collision.
BUMP_GAP_SEC = 0.5

FRONT_IR_SENSORS = {
    'ir_intensity_front_left',
    'ir_intensity_front_center_left',
    'ir_intensity_front_center_right',
    'ir_intensity_front_right',
}


class MovementController(Node):

    def __init__(self):
        super().__init__('movement_controller')

        self.declare_parameter('robot_namespace', '/robot_8')
        self.declare_parameter('ir_threshold', DEFAULT_IR_THRESHOLD)

        ns = str(self.get_parameter('robot_namespace').value).rstrip('/')
        self.ir_threshold = int(self.get_parameter('ir_threshold').value)

        # Commands from the Navigation node (same strings as its
        # pledge_decision): FORWARD, TURN_LEFT, TURN_RIGHT, TURN_AROUND, STOP
        self.command_subscriber = self.create_subscription(
            String, '/movement_command', self.command_callback, 10
        )

        # Status values for Navigation:
        # BUSY, DONE, STOPPED, SAFETY_BLOCKED, SAFETY_STOPPED, COLLISION, ERROR
        self.status_publisher = self.create_publisher(
            String, '/movement_status', 10
        )

        self.drive_client = ActionClient(
            self, DriveDistance, f'{ns}/drive_distance'
        )
        self.rotate_client = ActionClient(
            self, RotateAngle, f'{ns}/rotate_angle'
        )

        # Create 3 publishes sensors as BEST_EFFORT, so the sensor-data
        # QoS profile is required. With the default (RELIABLE) QoS no
        # messages arrive and the safety code never runs.
        self.ir_subscriber = self.create_subscription(
            IrIntensityVector,
            f'{ns}/ir_intensity',
            self.ir_safety_callback,
            qos_profile_sensor_data
        )
        self.hazard_subscriber = self.create_subscription(
            HazardDetectionVector,
            f'{ns}/hazard_detection',
            self.hazard_callback,
            qos_profile_sensor_data
        )

        # Action state
        self.action_active = False
        self.goal_handle = None
        self.current_command = None
        self.stop_requested = False
        self.safety_stop_requested = False
        self.safety_stop_reason = None

        # Sensor state
        self.front_ir_max = 0
        self.front_ir_sensor = None
        self.last_ir_time = None
        self.last_bump_time = None

        # After a bumper hit, FORWARD stays blocked until a turn succeeds.
        self.forward_blocked = False

        # Wait once at startup instead of blocking inside callbacks.
        if not self.drive_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning(f'{ns}/drive_distance not available yet')
        if not self.rotate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning(f'{ns}/rotate_angle not available yet')

        self.get_logger().info(
            f'MovementController started (namespace={ns}, '
            f'ir_threshold={self.ir_threshold})'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Status: {status}')

    def seconds_since(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def ir_data_is_fresh(self):
        return (
            self.last_ir_time is not None
            and self.seconds_since(self.last_ir_time) <= IR_TIMEOUT_SEC
        )

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def command_callback(self, msg):
        command = msg.data.strip().upper()
        self.get_logger().info(f'Command received: {command}')

        if command == 'STOP':
            self.stop_robot()
            return

        # Navigation may send a decision on every LiDAR update, so commands
        # often arrive while a move is running. Ignore them quietly.
        if self.action_active:
            self.get_logger().warning(
                f'Ignoring {command}: robot is BUSY',
                throttle_duration_sec=1.0)
            return

        if command == 'FORWARD':
            if self.forward_blocked:
                self.get_logger().warning(
                    'SAFETY: FORWARD blocked after bumper collision, '
                    'turn first')
                self.publish_status('SAFETY_BLOCKED')
                return

            # Safety check BEFORE moving. This prevents the loop where
            # Navigation keeps sending FORWARD into a wall.
            if not self.ir_data_is_fresh():
                self.get_logger().warning(
                    'SAFETY: no recent IR data, FORWARD blocked '
                    '(check topic name / QoS)'
                )
                self.publish_status('SAFETY_BLOCKED')
                return

            if self.front_ir_max >= self.ir_threshold:
                self.get_logger().warning(
                    f'SAFETY: FORWARD blocked, obstacle ahead '
                    f'({self.front_ir_sensor}: {self.front_ir_max})'
                )
                self.publish_status('SAFETY_BLOCKED')
                return

            future = self.drive_forward()

        elif command == 'TURN_LEFT':
            future = self.rotate_robot(TURN_ANGLE)

        elif command == 'TURN_RIGHT':
            future = self.rotate_robot(-TURN_ANGLE)

        elif command == 'TURN_AROUND':
            future = self.rotate_robot(TURN_AROUND_ANGLE)

        else:
            self.get_logger().error(f'Unknown command: {command}')
            self.publish_status('ERROR')
            return

        if future is None:
            self.publish_status('ERROR')
            return

        self.action_active = True
        self.current_command = command
        self.stop_requested = False
        self.safety_stop_requested = False
        self.safety_stop_reason = None

        self.publish_status('BUSY')
        future.add_done_callback(self.goal_response_callback)

    def drive_forward(self):
        if not self.drive_client.server_is_ready():
            self.get_logger().error('DriveDistance server not available')
            return None

        goal = DriveDistance.Goal()
        goal.distance = FORWARD_DISTANCE
        goal.max_translation_speed = FORWARD_SPEED

        self.get_logger().info(f'Driving forward {FORWARD_DISTANCE} m')
        return self.drive_client.send_goal_async(goal)

    # Positive angle = left, negative angle = right.
    def rotate_robot(self, angle_deg):
        if not self.rotate_client.server_is_ready():
            self.get_logger().error('RotateAngle server not available')
            return None

        goal = RotateAngle.Goal()
        goal.angle = math.radians(angle_deg)
        goal.max_rotation_speed = TURN_SPEED

        self.get_logger().info(f'Rotating {angle_deg} degrees')
        return self.rotate_client.send_goal_async(goal)

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def goal_response_callback(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Goal error: {error}')
            self.finish_with_error()
            return

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.finish_with_error()
            return

        self.get_logger().info('Goal accepted')

        # STOP or a safety stop may have arrived before acceptance.
        if self.stop_requested:
            self.cancel_current_goal()

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f'Result error: {error}')
            self.finish_with_error()
            return

        finished_command = self.current_command
        was_stopped = self.stop_requested
        safety_reason = self.safety_stop_reason
        status = result.status

        self.reset_state()

        if status == GoalStatus.STATUS_SUCCEEDED:
            if finished_command in ('TURN_LEFT', 'TURN_RIGHT', 'TURN_AROUND'):
                if self.forward_blocked:
                    self.get_logger().info(
                        'SAFETY: forward block cleared after successful turn')
                self.forward_blocked = False
            self.get_logger().info(f'Finished: {finished_command}')
            self.publish_status('DONE')
            return

        # Check the safety reason first: Create 3 may ABORT the goal by
        # itself on contact before our cancel arrives.
        if safety_reason == 'BUMPER':
            self.publish_status('COLLISION')
            return

        if safety_reason == 'IR':
            self.publish_status('SAFETY_STOPPED')
            return

        if status == GoalStatus.STATUS_CANCELED and was_stopped:
            self.publish_status('STOPPED')
            return

        self.get_logger().error(
            f'{finished_command} ended with status {status} '
            f'(6 = ABORTED by Create 3)'
        )
        self.publish_status('ERROR')

    # ------------------------------------------------------------------
    # Stopping
    # ------------------------------------------------------------------

    def stop_robot(self):
        if not self.action_active:
            self.publish_status('STOPPED')
            return

        self.stop_requested = True
        if self.goal_handle is not None:
            self.cancel_current_goal()

    def request_safety_stop(self, reason):
        self.safety_stop_requested = True
        self.safety_stop_reason = reason
        self.stop_requested = True
        if self.goal_handle is not None:
            self.cancel_current_goal()

    def cancel_current_goal(self):
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Cancel error: {error}')
            return

        if response.goals_canceling:
            self.get_logger().info('Cancel request accepted')
        else:
            self.get_logger().warning('Cancel request was not accepted')

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------

    # Emergency backup only: normal obstacle avoidance is the wall
    # follower's job. IR only stops FORWARD, never a turn, otherwise the
    # robot could not turn away from a wall.
    def ir_safety_callback(self, msg):
        found_front = False
        max_value = 0
        max_sensor = None

        for reading in msg.readings:
            name = reading.header.frame_id
            if name in FRONT_IR_SENSORS:
                if not found_front or reading.value > max_value:
                    max_value = reading.value
                    max_sensor = name
                found_front = True

        if not found_front:
            names = [r.header.frame_id for r in msg.readings]
            self.get_logger().warning(
                f'No front IR sensors matched. Names received: {names}',
                throttle_duration_sec=5.0
            )
            return

        self.front_ir_max = max_value
        self.front_ir_sensor = max_sensor
        self.last_ir_time = self.get_clock().now()

        # Useful while tuning the threshold.
        self.get_logger().info(
            f'IR front max: {max_value} ({max_sensor})',
            throttle_duration_sec=2.0
        )

        if max_value < self.ir_threshold:
            return

        if (
            self.action_active
            and self.current_command == 'FORWARD'
            and not self.safety_stop_requested
        ):
            self.get_logger().warning(
                f'SAFETY: obstacle very close, stopping '
                f'({max_sensor}: {max_value})'
            )
            self.request_safety_stop('IR')

    def hazard_callback(self, msg):
        bump_now = any(
            d.type == HazardDetection.BUMP for d in msg.detections
        )
        if not bump_now:
            return

        now = self.get_clock().now()
        new_bump = (
            self.last_bump_time is None
            or self.seconds_since(self.last_bump_time) > BUMP_GAP_SEC
        )
        self.last_bump_time = now

        if not new_bump:
            return

        self.get_logger().warning('SAFETY: bumper collision detected')
        self.forward_blocked = True

        if self.action_active and self.current_command == 'FORWARD':
            if not self.safety_stop_requested:
                self.request_safety_stop('BUMPER')
            else:
                # Bumper is more important than an IR stop already in progress.
                self.safety_stop_reason = 'BUMPER'
        elif not self.action_active:
            self.publish_status('COLLISION')

    def reset_state(self):
        self.action_active = False
        self.goal_handle = None
        self.current_command = None
        self.stop_requested = False
        self.safety_stop_requested = False
        self.safety_stop_reason = None
        # forward_blocked is NOT cleared here: it survives until a
        # successful recovery turn.

    def finish_with_error(self):
        self.reset_state()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
