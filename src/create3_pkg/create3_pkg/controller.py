"""
Wall-following controller for a round robot (pure Python, no ROS).

All distances are from the robot centre (see lidar_processing).

The robot steers on the NEAREST obstacle point on the followed side:
keep it beside the robot (bearing -90 deg for a right wall) at
`robot_radius + wall_clearance`. This one rule handles
  * straight walls      - the nearest point is perpendicular to the wall
  * inner corners       - the wall ahead becomes the nearest point -> turn away
  * wall ends / thin walls - the nearest point is the wall tip; the robot
                          circles it at a safe radius, whatever its thickness

States (for logging)
--------------------
FIND_WALL   nothing on the followed side within follow_range: drive straight
FOLLOW_WALL nearest-point control
TURN_CORNER nearest point is behind the robot (rounding a wall end): slow
            down and add the turn rate needed to circle the tip
TURN_AWAY   footprint about to touch something in front: rotate in place
            away from the followed wall (dead ends, head-on walls)

Following one wall (right-hand or left-hand rule) solves any maze whose start
and finish are both on the outer boundary. Loops can trap a pure wall follower
only if the goal sits on an "island" wall that is not connected to the start.
"""

from dataclasses import dataclass
from enum import Enum
import math

from create3_pkg.lidar_processing import LidarProcessor, wrap_angle


class State(Enum):
    FIND_WALL = 'FIND_WALL'
    FOLLOW_WALL = 'FOLLOW_WALL'
    TURN_CORNER = 'TURN_CORNER'
    TURN_AWAY = 'TURN_AWAY'


@dataclass
class ControllerParams:
    """Tuning parameters. Distances in m, speeds in m/s and rad/s."""

    side: int = -1                     # -1 = follow right wall, +1 = left wall
    robot_radius: float = 0.175        # Create 3: 35 cm diameter
    wall_clearance: float = 0.09       # gap body edge <-> followed wall
    follow_range: float = 1.0          # look for a wall this far from the centre
    approach_angle: float = math.radians(35.0)  # max angle to steer at a wall
    k_distance: float = 6.0            # approach angle (rad) per m distance error
    k_bearing: float = 2.5             # turn rate (rad/s) per rad bearing error
    max_linear: float = 0.15           # forward speed on a straight wall
    min_linear: float = 0.04           # slowest forward speed while following
    corner_linear: float = 0.08        # forward speed while rounding a wall end
    max_angular: float = 1.2           # angular speed limit
    turn_angular: float = 1.0          # in-place turning speed in TURN_AWAY
    side_margin: float = 0.02          # extra width of the swept path checked ahead
    front_stop: float = 0.04           # free path shorter than this -> TURN_AWAY
    front_clear: float = 0.15          # leave TURN_AWAY when path is this free
    front_slow: float = 0.35           # start slowing down below this free path

    @property
    def desired_distance(self):
        """Target distance robot centre -> wall."""
        return self.robot_radius + self.wall_clearance


@dataclass
class Command:
    linear: float
    angular: float


def clamp(value, low, high):
    return max(low, min(high, value))


class WallFollowController:
    """Computes velocity commands from processed LiDAR data."""

    def __init__(self, params: ControllerParams):
        self.p = params
        self.state = State.FIND_WALL
        self.last_error = 0.0  # distance error, exposed for logging / plotting

    def compute(self, lidar: LidarProcessor, now: float) -> Command:
        """
        Return the velocity command for the current scan.

        `now` (s) is unused by this controller; kept for the node interface.
        """
        p = self.p
        s = p.side
        free = lidar.free_distance_ahead(p.robot_radius + p.side_margin)

        # ---- safety first: something in the swept path right in front ----
        if free < p.front_stop or (self.state == State.TURN_AWAY
                                   and free < p.front_clear):
            self.state = State.TURN_AWAY
            return Command(0.0, -s * p.turn_angular)

        # Followed side, plus a little of the front on the other side so a
        # wall straight ahead also counts (inner corners).
        nearest = lidar.nearest_point(*sorted((-s * math.radians(10.0),
                                               s * math.pi)),
                                      max_distance=p.follow_range)
        if nearest is None:
            self.state = State.FIND_WALL
            return Command(self._speed(free, 0.0), 0.0)

        distance, bearing = nearest
        error = distance - p.desired_distance  # > 0: too far from the wall
        self.last_error = error

        # Too far -> point the robot towards the wall (at most approach_angle),
        # too close -> away from it. Pointing towards a right wall by `alpha`
        # moves the wall's nearest point forward to bearing -(90 deg - alpha).
        alpha = clamp(p.k_distance * error, -p.approach_angle, p.approach_angle)
        bearing_target = s * (math.pi / 2.0 - alpha)
        bearing_error = float(wrap_angle(bearing - bearing_target))
        angular = p.k_bearing * bearing_error

        # Nearest point behind the robot = we are passing the end of a wall.
        # Add the turn rate that circles a fixed point at this distance
        # (w = v / d), so the robot follows the tip instead of lagging behind.
        # The 5 deg deadband ignores heading wobble on a straight wall.
        behind = clamp((s * bearing - math.radians(95.0)) / math.radians(10.0),
                       0.0, 1.0)
        if behind > 0.0:
            self.state = State.TURN_CORNER
            speed = min(self._speed(free, angular), p.corner_linear)
            angular += s * behind * speed / max(distance, 0.05)
        else:
            self.state = State.FOLLOW_WALL
            speed = None

        angular = clamp(angular, -p.max_angular, p.max_angular)
        if speed is None:
            speed = self._speed(free, angular)
        return Command(speed, angular)

    def _speed(self, free, angular):
        """Slow down near obstacles in front and during sharp turns."""
        p = self.p
        span = max(p.front_slow - p.front_stop, 1e-3)
        front_factor = clamp((free - p.front_stop) / span, 0.0, 1.0)
        turn_factor = 1.0 - 0.7 * abs(angular) / max(p.max_angular, 1e-3)
        speed = p.max_linear * front_factor * turn_factor
        return clamp(speed, p.min_linear, p.max_linear)
