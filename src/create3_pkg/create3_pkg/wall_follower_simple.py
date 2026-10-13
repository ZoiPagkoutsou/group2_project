#!/usr/bin/env python3
"""See -> Think -> Act wall follower with sensor-guided corner recovery."""

import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# =============================================================
# CALIBRATION SETTINGS (distances: m, speeds: m/s and rad/s)
# =============================================================
SCAN_TOPIC = '/robot_8/scan'
ODOM_TOPIC = '/robot_8/odom'
CMD_TOPIC = '/robot_8/cmd_vel'

# Physical setup. All three measurements can be changed independently.
ROBOT_DIAMETER = 0.35
LIDAR_TO_FRONT = 0.20    # Distance from LiDAR centre to the front body edge.
CORRIDOR_WIDTH = 1.5 * ROBOT_DIAMETER  # Clear space between wall surfaces.
ROBOT_RADIUS = ROBOT_DIAMETER / 2.0

# The LiDAR is centred left/right, so distance to the wall includes the radius.
# Default: centre the robot, leaving 0.0875 m beside each body edge.
DESIRED_SIDE_CLEARANCE = (CORRIDOR_WIDTH - ROBOT_DIAMETER) / 2.0
DESIRED_DISTANCE = ROBOT_RADIUS + DESIRED_SIDE_CLEARANCE
SAFETY_CLEARANCE = 0.03  # Starting body-to-wall margin; calibrate on the robot.

FORWARD_SPEED = 0.306    # Current /robot_8/motion_control max_speed.
SLOW_SPEED = 0.15
CURVE_SPEED = 0.0        # In-place left turn; increase only after calibration.
TURN_SPEED = 1.50
TURN_FINISH_SPEED = 0.50
TURN_SLOW_ANGLE_DEGREES = 15.0
# Heading error is measured in radians; avoid scaling gains with cruise speed.
K_DISTANCE = 1.0
K_ANGLE = 1.5
MAX_CORRECTION = 0.60
WALL_ALIGNED_ANGLE = math.radians(10.0)
WALL_DISTANCE_TOLERANCE = 0.05

SECTOR_HALF_WIDTH = 10.0  # Each conceptual direction covers +/-10 degrees.
MIN_VALID_READINGS = 3
NEAR_READING_COUNT = 3   # Median of the three closest valid readings.
# Turn before the front body reaches the same clearance as the side body.
# Separate this manoeuvre trigger from the closer stop threshold for left turns.
FRONT_SAFETY_DISTANCE = LIDAR_TO_FRONT + SAFETY_CLEARANCE
FRONT_BLOCKED = LIDAR_TO_FRONT + DESIRED_SIDE_CLEARANCE
# Conservative starting diagonal thresholds, not a footprint/geometry model.
FRONT_LEFT_BLOCKED = max(ROBOT_RADIUS, LIDAR_TO_FRONT) + SAFETY_CLEARANCE
FRONT_RIGHT_BLOCKED = FRONT_LEFT_BLOCKED
SLOW_FRONT_DISTANCE = FRONT_BLOCKED + 0.20
SLOW_DIAGONAL_DISTANCE = FRONT_LEFT_BLOCKED + 0.05
# The 90-degree sector must see farther than the normal nearby left wall.
# The 45-degree sector only needs enough space to enter the left corridor.
LEFT_OPEN_DISTANCE = DESIRED_DISTANCE + 0.10
FRONT_LEFT_OPEN_DISTANCE = SLOW_DIAGONAL_DISTANCE
LEFT_WALL_SEEN_DISTANCE = DESIRED_DISTANCE + 0.05
WALL_FOLLOW_MAX_DISTANCE = CORRIDOR_WIDTH + 0.10
LEFT_OPEN_PERSISTENCE = 0.30
CORNER_CLEARANCE_DISTANCE = 0.15  # Maximum advance, not a fixed turn delay.
CORNER_CLEARANCE_TIME = CORNER_CLEARANCE_DISTANCE / SLOW_SPEED
CORNER_BACK_CLEARANCE = math.hypot(
    DESIRED_DISTANCE, ROBOT_RADIUS + SAFETY_CLEARANCE)
MIN_LEFT_TURN_ANGLE = math.radians(60.0)
WALL_CONFIRM_TIME = 0.20
REACQUIRE_SPEED = 0.06
REACQUIRE_TURN_SPEED = 0.35
REACQUIRE_MAX_ANGLE = math.radians(45.0)
REACQUIRE_TIMEOUT = 8.0
LEFT_RETRY_DELAY = 3.0
CONTROLLER_REVISION = 'small-wall-recovery-3'

TURN_ANGLE_DEGREES = 90.0
TURN_TOLERANCE_DEGREES = 3.0
MANOEUVRE_TIMEOUT = 12.0  # Includes clearance when turning left.
SCAN_TIMEOUT = 0.25
ODOM_TIMEOUT = 0.25
CONTROL_PERIOD = 0.02
LOG_PERIOD = 1.0

DIRECTIONS = {
    'front': 0, 'front_left': 45, 'left': 90, 'back_left': 135,
    'back': 180, 'back_right': 225, 'right': 270, 'front_right': 315,
}


class WallFollower(Node):
    FOLLOW_WALL = 'FOLLOW_WALL'
    TURN_RIGHT = 'TURN_RIGHT'
    TURN_LEFT = 'TURN_LEFT'
    REACQUIRE_WALL = 'REACQUIRE_WALL'

    def __init__(self):
        super().__init__('wall_follower')
        # Catch settings that cannot provide the chosen clearances.
        if (ROBOT_DIAMETER <= 0.0 or LIDAR_TO_FRONT <= 0.0
                or LIDAR_TO_FRONT > ROBOT_DIAMETER
                or CORRIDOR_WIDTH <= ROBOT_DIAMETER
                or SAFETY_CLEARANCE <= 0.0
                or DESIRED_SIDE_CLEARANCE < SAFETY_CLEARANCE
                or CORRIDOR_WIDTH - DESIRED_DISTANCE - ROBOT_RADIUS
                < SAFETY_CLEARANCE):
            raise ValueError('Robot/corridor settings do not leave safe side clearance')
        if not FRONT_SAFETY_DISTANCE <= FRONT_BLOCKED < SLOW_FRONT_DISTANCE:
            raise ValueError('Expected front safety <= turn trigger < slow distance')
        self.state = self.FOLLOW_WALL
        self.state_start_time = self.now()
        self.left_wall_seen = False
        self.open_since = None
        self.corner_clearance_time = CORNER_CLEARANCE_TIME
        self.turn_start_yaw = None
        self.turn_progress = 0.0
        self.wall_aligned_since = None
        self.left_retry_after = 0.0
        self.turn_target_angle = math.radians(TURN_ANGLE_DEGREES)
        self.fault = None
        self.wait_reason = None

        # Each direction stores (median distance, near distance).
        # (None, None) means unknown, never confirmed open space.
        self.sectors = {name: (None, None) for name in DIRECTIONS}
        self.last_scan_time = None
        self.last_odom_time = None
        self.yaw = None
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.last_log_time = None

        self.cmd_publisher = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, SCAN_TOPIC, self.scan_callback, qos_profile_sensor_data)
        self.odom_subscriber = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, qos_profile_sensor_data)
        # A timer still stops the robot when scan callbacks stop arriving.
        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control_robot)
        self.get_logger().info(
            f'8-sector FSM started [{CONTROLLER_REVISION}]: '
            f'scan={SCAN_TOPIC}, odom={ODOM_TOPIC}; '
            f'sectors=+/-{SECTOR_HALF_WIDTH:.0f} deg, '
            f'wall distance={DESIRED_DISTANCE:.2f} m')
        self.get_logger().info(
            f'Setup: diameter={ROBOT_DIAMETER:.3f} m, '
            f'LiDAR-to-front={LIDAR_TO_FRONT:.3f} m, '
            f'corridor={CORRIDOR_WIDTH:.3f} m; '
            f'left/right body clearance='
            f'{DESIRED_SIDE_CLEARANCE:.3f}/'
            f'{CORRIDOR_WIDTH - DESIRED_DISTANCE - ROBOT_RADIUS:.3f} m; '
            f'front turn/stop={FRONT_BLOCKED:.3f}/{FRONT_SAFETY_DISTANCE:.3f} m')
        self.get_logger().info(
            f'Speeds: straight={FORWARD_SPEED:.3f} m/s, '
            f'approach/clearance={SLOW_SPEED:.3f} m/s, '
            f'turn={TURN_SPEED:.2f} rad/s, finish={TURN_FINISH_SPEED:.2f} rad/s')

    # =========================================================
    # SEE: eight LiDAR sectors and odometry yaw
    # =========================================================
    def scan_callback(self, msg):
        self.sectors = {
            name: self.get_sector(msg, angle)
            for name, angle in DIRECTIONS.items()
        }
        self.last_scan_time = self.now()

    def get_sector(self, msg, target_angle):
        values = []
        for i, distance in enumerate(msg.ranges):
            if (not math.isfinite(distance) or distance <= 0.0
                    or not msg.range_min <= distance <= msg.range_max):
                continue
            angle = math.degrees(msg.angle_min + i * msg.angle_increment)
            difference = (angle - target_angle + 180.0) % 360.0 - 180.0
            if abs(difference) <= SECTOR_HALF_WIDTH:
                values.append(distance)
        if len(values) < max(MIN_VALID_READINGS, NEAR_READING_COUNT):
            return None, None
        values.sort()
        return (statistics.median(values),
                statistics.median(values[:NEAR_READING_COUNT]))

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        components = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in components):
            return
        norm = math.sqrt(sum(value * value for value in components))
        if norm < 1e-6:
            return
        x, y, z, w = (value / norm for value in components)
        yaw = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
        # Accumulate small yaw changes, handling the +/-180 degree wrap.
        if self.turn_start_yaw is not None and self.yaw is not None:
            change = math.atan2(math.sin(yaw - self.yaw),
                                math.cos(yaw - self.yaw))
            self.turn_progress += change
        self.yaw = yaw
        self.last_odom_time = self.now()

    # =========================================================
    # THINK: follow, clear/turn, then reacquire the wall
    # =========================================================
    def control_robot(self):
        now = self.now()
        self.think(now)
        self.print_status(now)

    def think(self, now):
        if self.fault is not None:
            self.act(0.0, 0.0)
            return

        if not self.is_fresh(self.last_scan_time, SCAN_TIMEOUT, now):
            self.open_since = None
            if self.state != self.FOLLOW_WALL:
                self.fail('LiDAR stale during manoeuvre')
            else:
                self.wait('Waiting for fresh LiDAR')
            return

        # Unknown front sectors do not provide permission to move.
        if any(self.sectors[name][1] is None
               for name in ('front', 'front_left', 'front_right')):
            self.open_since = None
            if self.state != self.FOLLOW_WALL:
                self.fail('Front sector data missing during manoeuvre')
            else:
                self.wait('Front sector data missing/invalid')
            return
        if self.state == self.FOLLOW_WALL:
            # Very close frontal obstacles take priority over corner detection.
            if self.sectors['front'][1] < FRONT_SAFETY_DISTANCE:
                self.open_since = None
                self.start_manoeuvre(self.TURN_RIGHT, 'Front safety distance reached', now)
                return

            _, left_near = self.sectors['left']
            if left_near is not None and left_near < LEFT_WALL_SEEN_DISTANCE:
                self.left_wall_seen = True

            left_open = self.left_opening()
            if self.left_wall_seen and left_open and now >= self.left_retry_after:
                if self.open_since is None:
                    self.open_since = now
                if now - self.open_since >= LEFT_OPEN_PERSISTENCE:
                    self.start_manoeuvre(
                        self.TURN_LEFT, 'Persistent left opening; clear corner', now)
                    return
                # Give a possible left corner time to confirm before turning right.
                if self.right_turn_needed():
                    self.wait('Confirming left opening at front obstacle')
                else:
                    self.follow_left_wall(opening_pending=True)
                return
            else:
                self.open_since = None

            if self.right_turn_needed():
                self.start_manoeuvre(self.TURN_RIGHT, 'Front obstacle confirmed', now)
                return
            self.follow_left_wall()
            return

        # Recovery also needs odometry so searching cannot continue indefinitely.
        if not self.is_fresh(self.last_odom_time, ODOM_TIMEOUT, now):
            self.fail('Odometry stale during manoeuvre')
            return
        if self.state == self.REACQUIRE_WALL:
            self.reacquire_wall(now)
            return
        if now - self.state_start_time >= MANOEUVRE_TIMEOUT:
            self.fail('Manoeuvre timeout: required yaw change not reached')
            return

        if self.state == self.TURN_LEFT:
            # Check all three front sectors throughout clearance and rotation.
            if self.front_blocked(FRONT_SAFETY_DISTANCE):
                self.cancel_left_attempt('Front area blocked during left manoeuvre', now)
                return
            if self.turn_start_yaw is None and not self.corner_clear():
                if (now - self.state_start_time >= self.corner_clearance_time
                        or self.sectors['front'][1] <= FRONT_BLOCKED):
                    if not self.rotation_clear():
                        self.cancel_left_attempt('Wall edge clearance unavailable', now)
                        return
                    # A rear wall can remain visible in a narrow corridor even
                    # when there is space for an in-place rotation.
                else:
                    self.act(SLOW_SPEED, 0.0)
                    return

        if self.turn_start_yaw is None:
            self.turn_start_yaw = self.yaw
            self.turn_progress = 0.0
            self.get_logger().info(
                f'{self.state}: rotation starts at yaw='
                f'{math.degrees(self.yaw):.1f} deg')

        direction = 1.0 if self.state == self.TURN_LEFT else -1.0
        target = max(0.0, self.turn_target_angle - math.radians(TURN_TOLERANCE_DEGREES))
        wall_aligned = (direction > 0.0
                        and self.turn_progress >= MIN_LEFT_TURN_ANGLE
                        and self.wall_alignment_confirmed(now))
        if wall_aligned or direction * self.turn_progress >= target:
            self.act(0.0, 0.0)
            self.left_wall_seen = False
            self.turn_start_yaw = None
            self.open_since = None
            self.wall_aligned_since = None
            if direction < 0.0:
                # A dead end can require another right turn. Recheck the front
                # before trying to align with a wall or searching back left.
                self.change_state(self.FOLLOW_WALL, 'Right turn complete; recheck exit', now)
            else:
                self.change_state(self.REACQUIRE_WALL, 'Turn complete; find left wall', now)
            return

        linear = self.forward_speed(CURVE_SPEED) if direction > 0.0 else 0.0
        # Rotate quickly, then slow near the target to reduce overshoot.
        remaining = self.turn_target_angle - direction * self.turn_progress
        angular_speed = TURN_SPEED
        if (remaining <= math.radians(TURN_SLOW_ANGLE_DEGREES)
                or self.wall_aligned_since is not None):
            angular_speed = TURN_FINISH_SPEED
        self.act(linear, direction * angular_speed)

    def start_manoeuvre(self, state, reason, now):
        self.act(0.0, 0.0)
        if not self.is_fresh(self.last_odom_time, ODOM_TIMEOUT, now):
            self.wait('Waiting for fresh odometry before manoeuvre')
            return
        self.turn_start_yaw = None
        self.turn_progress = 0.0
        self.turn_target_angle = math.radians(TURN_ANGLE_DEGREES)
        self.wall_aligned_since = None
        self.open_since = None
        self.corner_clearance_time = CORNER_CLEARANCE_TIME
        if state == self.TURN_LEFT:
            # Shorten/skip the advance when a front wall leaves less room.
            # Keep the normal front-turn clearance, not just the safety margin.
            available_forward = max(0.0, self.sectors['front'][1] - FRONT_BLOCKED)
            self.corner_clearance_time = min(
                CORNER_CLEARANCE_TIME, available_forward / SLOW_SPEED)
            reason += f' (forward clearance {self.corner_clearance_time:.2f} s)'
        self.change_state(state, reason, now)

    def rotation_clear(self):
        clearance = max(ROBOT_RADIUS, LIDAR_TO_FRONT) + SAFETY_CLEARANCE
        return all(near is not None and near > clearance
                   for _, near in self.sectors.values())

    def cancel_left_attempt(self, reason, now):
        # Undo a partial turn; if rotation never began, choose a right exit.
        undo_angle = max(0.0, self.turn_progress)
        rotating = self.turn_start_yaw is not None
        self.left_retry_after = now + LEFT_RETRY_DELAY
        self.start_manoeuvre(self.TURN_RIGHT, reason + '; cancel left attempt', now)
        if rotating and undo_angle > math.radians(TURN_TOLERANCE_DEGREES):
            self.turn_target_angle = undo_angle

    def corner_clear(self):
        # At the normal side offset, a receding rear diagonal indicates that
        # the edge is behind the body. Unknown readings never confirm clearance.
        left_near = self.sectors['left'][1]
        back_near = self.sectors['back_left'][1]
        return (left_near is not None and back_near is not None
                and left_near > ROBOT_RADIUS + SAFETY_CLEARANCE
                and back_near > CORNER_BACK_CLEARANCE)

    def wall_geometry(self):
        # A short wall may occupy fewer than half the rays in a sector.
        # Its nearby returns still provide a wall reference.
        left = self.sectors['left'][1]
        front_left = self.sectors['front_left'][1]
        if (left is None or front_left is None
                or left >= WALL_FOLLOW_MAX_DISTANCE
                or front_left >= WALL_FOLLOW_MAX_DISTANCE):
            return None
        diagonal = front_left * math.sqrt(0.5)
        heading = math.atan2(diagonal - left, diagonal)
        return left * math.cos(heading), heading

    def wall_alignment_confirmed(self, now):
        geometry = self.wall_geometry()
        aligned = (geometry is not None
                   and abs(geometry[1]) <= WALL_ALIGNED_ANGLE
                   and abs(geometry[0] - DESIRED_DISTANCE)
                   <= WALL_DISTANCE_TOLERANCE)
        if not aligned:
            self.wall_aligned_since = None
            return False
        if self.wall_aligned_since is None:
            self.wall_aligned_since = now
        return now - self.wall_aligned_since >= WALL_CONFIRM_TIME

    def reacquire_wall(self, now):
        # A blocked passage is a navigation decision, not a sensor fault.
        # Handle it before search limits so a dead end can be exited normally.
        if self.right_turn_needed():
            self.start_manoeuvre(
                self.TURN_RIGHT, 'Front blocked during wall search; turn toward exit', now)
            return
        left_near = self.sectors['left'][1]
        if (left_near is not None and left_near < WALL_FOLLOW_MAX_DISTANCE
                and left_near > ROBOT_RADIUS + SAFETY_CLEARANCE):
            # Finding a wall is enough to hand over to the tracking controller.
            # Requiring perfect alignment here made short walls disappear again
            # during repeated in-place corrections.
            self.left_wall_seen = True
            self.turn_start_yaw = None
            self.open_since = None
            self.wall_aligned_since = None
            self.act(0.0, 0.0)
            self.change_state(self.FOLLOW_WALL, 'Nearby left wall acquired; track slowly', now)
            return
        if now - self.state_start_time >= REACQUIRE_TIMEOUT:
            self.left_retry_after = now + LEFT_RETRY_DELAY
            self.start_manoeuvre(self.TURN_RIGHT, 'Wall search timeout; recheck exit', now)
            return
        if self.turn_start_yaw is None:
            self.turn_start_yaw = self.yaw
            self.turn_progress = 0.0
        if abs(self.turn_progress) >= REACQUIRE_MAX_ANGLE:
            self.left_retry_after = now + LEFT_RETRY_DELAY
            self.start_manoeuvre(self.TURN_RIGHT, 'Wall search limit; recheck exit', now)
            return
        if self.wall_alignment_confirmed(now):
            self.left_wall_seen = True
            self.turn_start_yaw = None
            self.open_since = None
            self.act(0.0, 0.0)
            self.change_state(self.FOLLOW_WALL, 'Left wall reacquired and aligned', now)
            return
        if self.sectors['left'][1] is None:
            self.wait('Waiting for valid left sector during wall search')
            return
        if self.sectors['front_left'][1] < FRONT_LEFT_BLOCKED:
            self.act(0.0, -MAX_CORRECTION)
        elif self.sectors['front_right'][1] < FRONT_RIGHT_BLOCKED:
            self.act(0.0, MAX_CORRECTION)
        elif self.sectors['left'][1] < ROBOT_RADIUS + SAFETY_CLEARANCE:
            self.act(0.0, -MAX_CORRECTION)
        elif self.wall_geometry() is not None:
            self.follow_left_wall(reacquiring=True)
        elif (self.sectors['front_left'][1] < WALL_FOLLOW_MAX_DISTANCE
              and self.sectors['left'][1] >= WALL_FOLLOW_MAX_DISTANCE):
            # Keep clearance from a visible edge even if no wall spans the
            # left sector yet. A zero-steering approach runs into short walls.
            projected_clearance = self.sectors['front_left'][1] * math.sqrt(0.5)
            correction = K_DISTANCE * (projected_clearance - DESIRED_DISTANCE)
            correction = max(-REACQUIRE_TURN_SPEED,
                             min(REACQUIRE_TURN_SPEED, correction))
            self.act(REACQUIRE_SPEED, correction)
        else:
            # Actively look left instead of driving straight past a thin wall.
            speed = REACQUIRE_SPEED
            if self.sectors['front'][1] < FRONT_BLOCKED:
                speed = 0.0
            self.act(speed, REACQUIRE_TURN_SPEED)

    def right_turn_needed(self):
        # One close diagonal is often the side wall, not a dead end.
        front_close = self.sectors['front'][1] < FRONT_BLOCKED
        both_diagonals_close = (
            self.sectors['front_left'][1] < FRONT_LEFT_BLOCKED
            and self.sectors['front_right'][1] < FRONT_RIGHT_BLOCKED)
        return front_close or both_diagonals_close

    def front_blocked(self, front_threshold=FRONT_BLOCKED):
        return any(self.sectors[name][1] < threshold for name, threshold in (
            ('front', front_threshold),
            ('front_left', FRONT_LEFT_BLOCKED),
            ('front_right', FRONT_RIGHT_BLOCKED)))

    def is_open(self, name, threshold=LEFT_OPEN_DISTANCE):
        median, near = self.sectors[name]
        return (median is not None and near is not None
                and median > threshold and near > threshold)

    def left_opening(self):
        back_median, back_near = self.sectors['back_left']
        # Back-left must be valid and clear of the body. It may still see the
        # previous wall: waiting for that wall to disappear made turns too late.
        back_safe = (back_median is not None and back_near is not None
                     and back_near > ROBOT_RADIUS + SAFETY_CLEARANCE)
        return (self.is_open('front_left', FRONT_LEFT_OPEN_DISTANCE)
                and self.is_open('left') and back_safe)

    def forward_speed(self, normal_speed):
        approaching = (
            self.sectors['front'][1] < SLOW_FRONT_DISTANCE
            or self.sectors['front_left'][1] < SLOW_DIAGONAL_DISTANCE
            or self.sectors['front_right'][1] < SLOW_DIAGONAL_DISTANCE)
        return min(normal_speed, SLOW_SPEED) if approaching else normal_speed

    def follow_left_wall(self, opening_pending=False, reacquiring=False):
        _, left = self.sectors['left']
        _, front_left = self.sectors['front_left']
        _, back_left = self.sectors['back_left']
        steering = 0.0
        aligned = False
        # After a turn, gently approach a visible wall even if it is farther
        # than the opening threshold. Do not steer across a pending opening.
        if (not opening_pending and left is not None
                and left < WALL_FOLLOW_MAX_DISTANCE):
            angle_error = 0.0
            # Two rays estimate the wall heading even immediately after a turn.
            # Prefer the wall ahead; the rear ray may still see the old corridor.
            diagonal = math.sqrt(0.5)
            heading_known = False
            if (front_left is not None
                    and front_left < WALL_FOLLOW_MAX_DISTANCE):
                angle_error = math.atan2(
                    front_left * diagonal - left, front_left * diagonal)
                heading_known = True
            elif (back_left is not None
                    and back_left < WALL_FOLLOW_MAX_DISTANCE):
                angle_error = math.atan2(
                    left - back_left * diagonal, back_left * diagonal)
                heading_known = True
            distance_error = left * math.cos(angle_error) - DESIRED_DISTANCE
            steering = K_DISTANCE * distance_error + K_ANGLE * angle_error
            steering = max(-MAX_CORRECTION, min(MAX_CORRECTION, steering))
            aligned = (heading_known
                       and abs(angle_error) <= WALL_ALIGNED_ANGLE
                       and abs(distance_error) <= WALL_DISTANCE_TOLERANCE
                       and self.sectors['left'][0] is not None
                       and abs(self.sectors['left'][0] - left)
                       <= WALL_DISTANCE_TOLERANCE)
        normal_speed = FORWARD_SPEED if aligned and not opening_pending else SLOW_SPEED
        if reacquiring:
            normal_speed = REACQUIRE_SPEED
        speed = self.forward_speed(normal_speed)
        # Correct a close side wall within FOLLOW_WALL instead of turning 90 deg.
        # Pause forward movement while steering away from a very close diagonal.
        if self.sectors['front_left'][1] < FRONT_LEFT_BLOCKED:
            speed = 0.0
            steering = -MAX_CORRECTION
        elif self.sectors['front_right'][1] < FRONT_RIGHT_BLOCKED:
            speed = 0.0
            steering = MAX_CORRECTION
        elif left is not None and left < ROBOT_RADIUS + SAFETY_CLEARANCE:
            speed = 0.0
            steering = -REACQUIRE_TURN_SPEED
        self.wait_reason = None
        self.act(speed, steering)

    # =========================================================
    # ACT: publish a simple Twist
    # =========================================================
    def act(self, linear, angular):
        self.command_linear = linear
        self.command_angular = angular
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_publisher.publish(cmd)

    def wait(self, reason):
        self.act(0.0, 0.0)
        if reason != self.wait_reason:
            self.get_logger().warning(reason)
            self.wait_reason = reason

    def fail(self, reason):
        # A fault inhibits all motion until the node is restarted.
        # Restart after inspecting a manoeuvre fault, rather than retry blindly.
        self.fault = reason
        self.act(0.0, 0.0)
        self.get_logger().error(f'STOP: {reason}. Inspect robot, then restart node.')

    def change_state(self, new_state, reason, now):
        self.wait_reason = None
        self.get_logger().info(
            f'{self.state} -> {new_state}: {reason}; '
            f'near F/FL/FR={self.sectors["front"][1]:.3f}/'
            f'{self.sectors["front_left"][1]:.3f}/{self.sectors["front_right"][1]:.3f} m')
        self.state = new_state
        self.state_start_time = now

    def print_status(self, now):
        if self.last_log_time is not None and now - self.last_log_time < LOG_PERIOD:
            return
        self.last_log_time = now

        def value(distance):
            return 'unknown' if distance is None else f'{distance:.2f}'

        sectors = ' '.join(
            f'{angle}={value(self.sectors[name][0])}/{value(self.sectors[name][1])}'
            for name, angle in DIRECTIONS.items())
        yaw = 'unknown' if self.yaw is None else f'{math.degrees(self.yaw):.1f}'
        self.get_logger().info(
            f'STATE={self.state} | sector median/near [m]: {sectors} | '
            f'yaw={yaw} deg turn={math.degrees(self.turn_progress):.1f} deg | '
            f'cmd v={self.command_linear:.3f} m/s '
            f'w={self.command_angular:.3f} rad/s | '
            f'status={self.fault or self.wait_reason or "running"}')

    @staticmethod
    def is_fresh(received_at, timeout, now):
        return received_at is not None and 0.0 <= now - received_at <= timeout

    @staticmethod
    def now():
        # Monotonic time keeps watchdogs working even if simulation time pauses.
        return time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.act(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
