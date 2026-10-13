"""Exercise corner recovery without creating a ROS node or publishing motion."""

import math
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
from create3_pkg import wall_follower_simple as wf


class Controller(wf.WallFollower):
    """Use real controller logic with simulated sensor readings and commands."""

    def __init__(self):
        self.state = self.TURN_LEFT
        self.state_start_time = 10.0
        self.last_scan_time = 10.0
        self.last_odom_time = 10.0
        self.yaw = 0.0
        self.turn_start_yaw = None
        self.turn_progress = 0.0
        self.corner_clearance_time = wf.CORNER_CLEARANCE_TIME
        self.wall_aligned_since = None
        self.left_retry_after = 0.0
        self.turn_target_angle = math.radians(wf.TURN_ANGLE_DEGREES)
        self.left_wall_seen = True
        self.open_since = None
        self.wait_reason = None
        self.fault = None
        self.sectors = {name: (1.0, 1.0) for name in wf.DIRECTIONS}
        self.logger = Mock()
        self.command = (0.0, 0.0)

    def get_logger(self):
        return self.logger

    def act(self, linear, angular):
        self.command = linear, angular

    def step(self, now):
        self.last_scan_time = now
        self.last_odom_time = now
        self.think(now)


def test_clear_edge_turns_without_fixed_advance():
    robot = Controller()
    robot.step(10.0)
    assert robot.command == (0.0, wf.TURN_SPEED)


def test_uncleared_edge_advances_then_rotates_if_surroundings_clear():
    robot = Controller()
    robot.sectors['back_left'] = (0.25, 0.25)
    robot.step(10.0)
    assert robot.command == (wf.SLOW_SPEED, 0.0)
    robot.step(10.0 + wf.CORNER_CLEARANCE_TIME)
    assert robot.fault is None
    assert robot.command == (0.0, wf.TURN_SPEED)


def test_rotation_enters_active_wall_search():
    robot = Controller()
    robot.turn_start_yaw = 0.0
    robot.turn_progress = math.radians(90.0)
    robot.step(10.0)
    assert robot.state == robot.REACQUIRE_WALL
    robot.step(10.02)
    assert robot.command == (wf.REACQUIRE_SPEED, wf.REACQUIRE_TURN_SPEED)


def test_visible_side_wall_hands_over_to_tracking():
    robot = Controller()
    robot.state = robot.REACQUIRE_WALL
    robot.sectors['left'] = (wf.DESIRED_DISTANCE,) * 2
    diagonal = wf.DESIRED_DISTANCE * math.sqrt(2.0)
    robot.sectors['front_left'] = (diagonal,) * 2
    robot.step(10.0)
    assert robot.state == robot.FOLLOW_WALL
    assert robot.left_wall_seen
    robot.step(10.02)
    assert robot.command[0] > 0.0


@pytest.mark.parametrize('condition', ['timeout', 'rotation', 'unknown'])
def test_wall_search_stops_on_limits_or_unknown_data(condition):
    robot = Controller()
    robot.state = robot.REACQUIRE_WALL
    robot.turn_start_yaw = 0.0
    now = 10.0
    if condition == 'timeout':
        now += wf.REACQUIRE_TIMEOUT
    elif condition == 'rotation':
        robot.turn_progress = wf.REACQUIRE_MAX_ANGLE
    else:
        robot.sectors['left'] = (None, None)
    robot.step(now)
    assert robot.command == (0.0, 0.0)
    assert robot.fault is None
    if condition != 'unknown':
        assert robot.state == robot.TURN_RIGHT


def test_stale_scan_stops_wall_search():
    robot = Controller()
    robot.state = robot.REACQUIRE_WALL
    robot.think(11.0)
    assert robot.fault is not None
    assert robot.command == (0.0, 0.0)


def test_visible_aligned_wall_can_end_left_rotation_early():
    robot = Controller()
    robot.turn_start_yaw = 0.0
    robot.turn_progress = math.radians(70.0)
    robot.sectors['left'] = (wf.DESIRED_DISTANCE,) * 2
    robot.sectors['front_left'] = (wf.DESIRED_DISTANCE * math.sqrt(2.0),) * 2
    robot.step(10.0)
    assert robot.command == (0.0, wf.TURN_FINISH_SPEED)
    robot.step(10.0 + wf.WALL_CONFIRM_TIME + 0.01)
    assert robot.state == robot.REACQUIRE_WALL


def test_close_diagonal_prevents_forward_wall_search():
    robot = Controller()
    robot.state = robot.REACQUIRE_WALL
    robot.sectors['front_left'] = (0.21, 0.21)
    robot.step(10.0)
    assert robot.command == (0.0, -wf.MAX_CORRECTION)


def test_unknown_rear_sector_does_not_confirm_clearance():
    robot = Controller()
    robot.sectors['back_left'] = (None, None)
    assert not robot.corner_clear()


@pytest.mark.parametrize('front_distance', [0.1, wf.FRONT_BLOCKED - 0.01])
def test_blocked_wall_search_requests_right_turn(front_distance):
    robot = Controller()
    robot.state = robot.REACQUIRE_WALL
    robot.sectors['front'] = (front_distance,) * 2
    robot.step(10.0)
    assert robot.state == robot.TURN_RIGHT
    assert robot.fault is None
    assert robot.command == (0.0, 0.0)
    robot.step(10.02)
    assert robot.command == (0.0, -wf.TURN_SPEED)


def test_dead_end_two_right_turns_then_drive_out():
    robot = Controller()
    robot.state = robot.FOLLOW_WALL
    robot.sectors['front'] = (wf.FRONT_BLOCKED - 0.01,) * 2
    robot.sectors['left'] = (wf.DESIRED_DISTANCE,) * 2
    robot.step(10.0)
    assert robot.state == robot.TURN_RIGHT
    robot.step(10.02)
    assert robot.command == (0.0, -wf.TURN_SPEED)
    robot.turn_progress = -math.radians(90.0)
    robot.step(10.04)
    assert robot.state == robot.FOLLOW_WALL
    # After the first quarter-turn the side wall is still in front.
    robot.step(10.06)
    assert robot.state == robot.TURN_RIGHT
    robot.step(10.08)
    assert robot.command == (0.0, -wf.TURN_SPEED)
    robot.turn_progress = -math.radians(90.0)
    robot.sectors['front'] = (1.0, 1.0)
    robot.sectors['front_left'] = (wf.DESIRED_DISTANCE * math.sqrt(2.0),) * 2
    robot.step(10.10)
    assert robot.state == robot.FOLLOW_WALL
    robot.step(10.12)
    assert robot.command[0] > 0.0
    assert robot.fault is None


def test_single_right_turn_uses_open_exit_without_second_turn():
    robot = Controller()
    robot.state = robot.TURN_RIGHT
    robot.turn_start_yaw = 0.0
    robot.turn_progress = -math.radians(90.0)
    robot.step(10.0)
    assert robot.state == robot.FOLLOW_WALL
    robot.step(10.02)
    assert robot.command[0] > 0.0
    assert robot.state == robot.FOLLOW_WALL


def test_blocked_left_rotation_undoes_partial_turn():
    robot = Controller()
    robot.turn_start_yaw = 0.0
    robot.turn_progress = math.radians(35.0)
    robot.sectors['front_left'] = (0.21, 0.21)
    robot.step(10.0)
    assert robot.state == robot.TURN_RIGHT
    assert robot.fault is None
    assert math.isclose(robot.turn_target_angle, math.radians(35.0))
    robot.step(10.02)
    assert robot.command == (0.0, -wf.TURN_SPEED)
    robot.turn_progress = -math.radians(35.0)
    robot.step(10.04)
    assert robot.state == robot.FOLLOW_WALL


def test_uncleared_corner_with_close_rear_cancels_left_attempt():
    robot = Controller()
    robot.sectors['back_left'] = (0.21, 0.21)
    robot.step(10.0 + wf.CORNER_CLEARANCE_TIME)
    assert robot.state == robot.TURN_RIGHT
    assert robot.fault is None
    assert robot.command == (0.0, 0.0)


def test_cancelled_left_attempt_is_not_immediately_retried():
    robot = Controller()
    robot.cancel_left_attempt('Test blocked corner', 10.0)
    robot.state = robot.FOLLOW_WALL
    robot.left_wall_seen = True
    robot.step(10.02)
    robot.step(10.5)
    assert robot.state == robot.FOLLOW_WALL
    assert robot.open_since is None


def test_logged_front_left_wall_does_not_trigger_more_left_search():
    robot = Controller()
    robot.state = robot.REACQUIRE_WALL
    readings = {
        'front': (4.38, 1.83), 'front_left': (0.45, 0.41),
        'left': (1.17, 1.08), 'back_left': (0.35, 0.31),
        'back': (0.27, 0.27), 'back_right': (0.47, 0.39),
        'right': (1.75, 1.42), 'front_right': (1.71, 0.63),
    }
    robot.sectors.update(readings)
    robot.turn_start_yaw = 0.0
    robot.turn_progress = math.radians(9.6)
    robot.step(10.0)
    assert robot.command[0] == wf.REACQUIRE_SPEED
    assert 0.0 < robot.command[1] < wf.REACQUIRE_TURN_SPEED
    assert robot.fault is None


@pytest.mark.parametrize('angle', [-50.3, 50.5])
def test_logged_visible_side_wall_at_search_limit_resumes_tracking(angle):
    robot = Controller()
    robot.state = robot.REACQUIRE_WALL
    robot.turn_start_yaw = 0.0
    robot.turn_progress = math.radians(angle)
    robot.sectors['front'] = (0.39, 0.34)
    robot.sectors['front_left'] = (1.13, 1.13)
    robot.sectors['left'] = (0.52, 0.46)
    robot.step(10.0)
    assert robot.state == robot.FOLLOW_WALL
    assert robot.fault is None
    robot.step(10.02)
    assert robot.command[0] > 0.0


def test_short_wall_near_returns_override_distant_sector_median():
    robot = Controller()
    robot.state = robot.FOLLOW_WALL
    robot.sectors['left'] = (1.15, 0.31)
    robot.sectors['front_left'] = (1.37, 0.69)
    robot.sectors['back_left'] = (0.46, 0.39)
    robot.follow_left_wall()
    assert robot.command[1] != 0.0
    assert robot.command[0] <= wf.SLOW_SPEED


def test_small_wall_ahead_left_gets_clearance_correction_before_collision():
    robot = Controller()
    robot.state = robot.REACQUIRE_WALL
    robot.sectors['left'] = (3.23, 3.22)
    robot.sectors['front_left'] = (0.29, 0.26)
    robot.step(10.0)
    assert robot.command[0] == wf.REACQUIRE_SPEED
    assert -wf.REACQUIRE_TURN_SPEED < robot.command[1] < 0.0


def test_close_side_wall_in_following_prevents_forward_motion():
    robot = Controller()
    robot.state = robot.FOLLOW_WALL
    robot.sectors['left'] = (0.20, 0.19)
    robot.follow_left_wall()
    assert robot.command == (0.0, -wf.REACQUIRE_TURN_SPEED)
