#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from irobot_create_msgs.msg import (
    IrIntensityVector,
    HazardDetectionVector
)

from irobot_create_msgs.action import (
    DriveDistance,
    RotateAngle
)

class MazeController(Node):

    def __init__(self):
        super().__init__('maze_controller')

        # ---------------------------------------------------------
        # VARIABLES
        # ---------------------------------------------------------

        self.ir_readings = []

        # Prevent sending another movement command
        # while the current action is still running.
        self.busy = False

        # Hazard information
        self.bump_detected = False
        self.cliff_detected = False
        self.bump_side = None

        # Keep track of current action goal
        self.current_goal_handle = None

        # ---------------------------------------------------------
        # SETTINGS
        # ---------------------------------------------------------

        # Temporary IR threshold.
        # You should tune this experimentally.
        self.obstacle_threshold = 500

        # Forward movement
        self.forward_distance = 0.10      # meters
        self.forward_speed = 0.08         # m/s

        # Rotation
        self.turn_angle = 90              # degrees
        self.rotation_speed = 0.4         # rad/s

        # ---------------------------------------------------------
        # IR SENSOR SUBSCRIBER
        # ---------------------------------------------------------

        self.ir_subscriber = self.create_subscription(
            IrIntensityVector,
            '/robot_8/ir_intensity',
            self.ir_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        # ---------------------------------------------------------
        # HAZARD / BUMPER SUBSCRIBER
        # ---------------------------------------------------------

        self.hazard_subscriber = self.create_subscription(
            HazardDetectionVector,
            '/robot_8/hazard_detection',
            self.hazard_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        # ---------------------------------------------------------
        # ACTION CLIENTS
        # ---------------------------------------------------------

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

        # ---------------------------------------------------------
        # CONTROL LOOP
        # ---------------------------------------------------------

        self.timer = self.create_timer(
            0.5,
            self.control_loop
        )

        self.get_logger().info(
            'Maze controller started'
        )


    # =============================================================
    # IR SENSOR CALLBACK
    # =============================================================

    def ir_callback(self, msg):

        self.ir_readings = [
            reading.value
            for reading in msg.readings
        ]


    # =============================================================
    # HAZARD CALLBACK
    # =============================================================

    def hazard_callback(self, msg):

        for detection in msg.detections:

            # type 1 = BUMP
            if detection.type == 1:

                self.bump_detected = True
                self.bump_side = detection.header.frame_id

                self.get_logger().warn(
                    f'BUMPER HIT: {self.bump_side}'
                )

            # type 2 = CLIFF
            elif detection.type == 2:

                self.cliff_detected = True

                self.get_logger().error(
                    f'CLIFF DETECTED: '
                    f'{detection.header.frame_id}'
                )


    # =============================================================
    # MAIN CONTROL LOGIC
    # =============================================================

    def control_loop(self):

        # Do nothing while robot is completing an action
        if self.busy:
            return

        # Wait until all 7 IR readings are available
        if len(self.ir_readings) != 7:
            return

        # ---------------------------------------------------------
        # CLIFF SAFETY
        # ---------------------------------------------------------

        if self.cliff_detected:

            self.get_logger().error(
                'Cliff detected -> stopping autonomous movement'
            )

            return

        # ---------------------------------------------------------
        # BUMPER REACTION
        # ---------------------------------------------------------

        if self.bump_detected:

            if self.bump_side == 'bump_left':

                self.get_logger().warn(
                    'Left bumper -> turning right'
                )

                self.bump_detected = False
                self.rotate_right()
                return

            elif self.bump_side == 'bump_right':

                self.get_logger().warn(
                    'Right bumper -> turning left'
                )

                self.bump_detected = False
                self.rotate_left()
                return

            else:

                self.get_logger().warn(
                    'Bumper -> turning right'
                )

                self.bump_detected = False
                self.rotate_right()
                return

        # ---------------------------------------------------------
        # SENSOR VALUES
        # ---------------------------------------------------------

        side_left = self.ir_readings[0]
        left = self.ir_readings[1]
        front_left = self.ir_readings[2]
        front_center_left = self.ir_readings[3]
        front_center_right = self.ir_readings[4]
        front_right = self.ir_readings[5]
        right = self.ir_readings[6]

        # Combine sensors into useful regions
        front_value = max(
            front_left,
            front_center_left,
            front_center_right,
            front_right
        )

        left_value = max(
            side_left,
            left
        )

        right_value = max(
            front_right,
            right
        )

        self.get_logger().info(
            f'LEFT={left_value} | '
            f'FRONT={front_value} | '
            f'RIGHT={right_value}'
        )

        # ---------------------------------------------------------
        # MAZE DECISION
        # ---------------------------------------------------------

        # Front is clear
        if front_value < self.obstacle_threshold:

            self.get_logger().info(
                'Decision: FORWARD'
            )

            self.drive_forward()

        # Front blocked -> compare left and right
        elif left_value < right_value:

            self.get_logger().info(
                'Decision: TURN LEFT'
            )

            self.rotate_left()

        else:

            self.get_logger().info(
                'Decision: TURN RIGHT'
            )

            self.rotate_right()


    # =============================================================
    # DRIVE FORWARD
    # =============================================================

    def drive_forward(self):

        goal = DriveDistance.Goal()

        goal.distance = self.forward_distance
        goal.max_translation_speed = self.forward_speed

        self.busy = True

        self.get_logger().info(
            f'Driving {self.forward_distance} m forward'
        )

        self.drive_client.wait_for_server()

        future = self.drive_client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_sent_callback
        )


    # =============================================================
    # ROTATE LEFT
    # =============================================================

    def rotate_left(self):

        goal = RotateAngle.Goal()

        goal.angle = math.radians(
            self.turn_angle
        )

        goal.max_rotation_speed = (
            self.rotation_speed
        )

        self.busy = True

        self.get_logger().info(
            f'Turning LEFT {self.turn_angle} degrees'
        )

        self.rotate_client.wait_for_server()

        future = self.rotate_client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_sent_callback
        )


    # =============================================================
    # ROTATE RIGHT
    # =============================================================

    def rotate_right(self):

        goal = RotateAngle.Goal()

        goal.angle = -math.radians(
            self.turn_angle
        )

        goal.max_rotation_speed = (
            self.rotation_speed
        )

        self.busy = True

        self.get_logger().info(
            f'Turning RIGHT {self.turn_angle} degrees'
        )

        self.rotate_client.wait_for_server()

        future = self.rotate_client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_sent_callback
        )


    # =============================================================
    # ACTION ACCEPTED CALLBACK
    # =============================================================

    def goal_sent_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().warn(
                'Movement goal rejected'
            )

            self.busy = False
            return

        self.current_goal_handle = goal_handle

        self.get_logger().info(
            'Movement goal accepted'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.goal_done_callback
        )


    # =============================================================
    # ACTION FINISHED CALLBACK
    # =============================================================

    def goal_done_callback(self, future):

        self.get_logger().info(
            'Movement finished'
        )

        self.current_goal_handle = None
        self.busy = False


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = MazeController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()