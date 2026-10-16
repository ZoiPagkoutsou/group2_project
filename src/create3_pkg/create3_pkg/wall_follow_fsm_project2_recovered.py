#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.action import WallFollow, RotateAngle


IR_TOPIC = "/robot_8/ir_intensity"

FOLLOW_WALL = "FOLLOW_WALL"
TURN_RIGHT = "TURN_RIGHT"
CURVE_LEFT = "CURVE_LEFT"

FRONT_THRESHOLD = 50
LEFT_THRESHOLD = 50

WALL_FOLLOW_RUNTIME = 3

TURN_RIGHT_ANGLE = -0.7
TURN_RIGHT_SPEED = 0.40

CURVE_LEFT_ANGLE = 0.35
CURVE_LEFT_SPEED = 0.20


class WallFollowFSM(Node):

    def __init__(self):

        super().__init__("wall_follow_fsm_project2")

        self.create_subscription(
            IrIntensityVector,
            IR_TOPIC,
            self.ir_callback,
            qos_profile_sensor_data
        )

        self.wall_client = ActionClient(
            self,
            WallFollow,
            "/robot_8/wall_follow"
        )

        self.rotate_client = ActionClient(
            self,
            RotateAngle,
            "/robot_8/rotate_angle"
        )

        self.ir = {}

        self.state = FOLLOW_WALL

        self.action_active = False

        self.goal_handle = None

        self.create_timer(
            0.2,
            self.control_loop
        )

        self.get_logger().info(
            "Action-based Wall Follow FSM started"
        )

    def ir_callback(self, msg):

        self.ir = {
            reading.header.frame_id: int(reading.value)
            for reading in msg.readings
        }

    def value(self, name):

        return self.ir.get(
            f"ir_intensity_{name}",
            0
        )

    def front_obstacle(self):

        return (
            self.value("front_center_left") > FRONT_THRESHOLD
            and
            self.value("front_center_right") > FRONT_THRESHOLD
        )

    def left_wall_detected(self):

        return (
            self.value("side_left") > LEFT_THRESHOLD
            or
            self.value("left") > LEFT_THRESHOLD
        )

    def update_state(self):

        if self.front_obstacle():

            self.state = TURN_RIGHT

        elif not self.left_wall_detected():

            self.state = CURVE_LEFT

        else:

            self.state = FOLLOW_WALL

    def control_loop(self):

        if not self.ir:
            return

        if self.action_active:
            return

        self.update_state()

        if self.state == FOLLOW_WALL:

            self.send_wall_follow()

        elif self.state == TURN_RIGHT:

            self.send_turn_right()

        elif self.state == CURVE_LEFT:

            self.send_curve_left()

    def send_wall_follow(self):

        if not self.wall_client.server_is_ready():

            self.get_logger().warn(
                "wall_follow action server not ready"
            )

            return

        goal = WallFollow.Goal()

        goal.follow_side = 1

        goal.max_runtime.sec = WALL_FOLLOW_RUNTIME
        goal.max_runtime.nanosec = 0

        self.start_goal(
            self.wall_client,
            goal,
            FOLLOW_WALL
        )

    def send_turn_right(self):

        if not self.rotate_client.server_is_ready():

            self.get_logger().warn(
                "rotate_angle action server not ready"
            )

            return

        goal = RotateAngle.Goal()

        goal.angle = TURN_RIGHT_ANGLE
        goal.max_rotation_speed = TURN_RIGHT_SPEED

        self.start_goal(
            self.rotate_client,
            goal,
            TURN_RIGHT
        )

    def send_curve_left(self):

        if not self.rotate_client.server_is_ready():

            self.get_logger().warn(
                "rotate_angle action server not ready"
            )

            return

        goal = RotateAngle.Goal()

        goal.angle = CURVE_LEFT_ANGLE
        goal.max_rotation_speed = CURVE_LEFT_SPEED

        self.start_goal(
            self.rotate_client,
            goal,
            CURVE_LEFT
        )

    def start_goal(self, client, goal, state_name):

        self.action_active = True

        self.get_logger().info(
            f"State: {state_name}"
        )

        future = client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().warn(
                "Goal rejected"
            )

            self.action_active = False

            return

        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.goal_result_callback
        )

    def goal_result_callback(self, future):

        self.action_active = False

        self.goal_handle = None

    def cancel_active_goal(self):

        if self.goal_handle is None:
            return

        self.goal_handle.cancel_goal_async()


def main(args=None):

    rclpy.init(args=args)

    node = WallFollowFSM()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.cancel_active_goal()

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    main()