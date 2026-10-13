"""Wall-following controller with a small state machine (pure Python, no ROS).

States
------
FIND_WALL   no wall on the followed side yet: drive straight until one appears
FOLLOW_WALL PD control on the (look-ahead) distance to the side wall
TURN_AWAY   wall/obstacle in front: rotate in place away from the followed wall
            (handles inner corners and dead ends)
TURN_CORNER followed wall disappeared (outer corner / opening): advance a bit so
            the robot body clears the corner, then arc around it towards the wall

Following one wall (right-hand or left-hand rule) solves any maze whose start
and finish are both on the outer boundary. Loops can trap a pure wall follower
only if the goal sits on an "island" wall that is not connected to the start.
"""

import math
from dataclasses import dataclass
from enum import Enum

from create3_pkg.lidar_processing import LidarProcessor, estimate_wall


class State(Enum):
    FIND_WALL = 'FIND_WALL'
    FOLLOW_WALL = 'FOLLOW_WALL'
    TURN_AWAY = 'TURN_AWAY'
    TURN_CORNER = 'TURN_CORNER'


@dataclass
class ControllerParams:
    """Tuning parameters. Distances in m, speeds in m/s and rad/s, times in s."""

    side: int = -1                     # -1 = follow right wall, +1 = left wall
    desired_distance: float = 0.25     # target distance to the wall
    look_ahead: float = 0.20           # predict distance this far ahead
    beam_angle: float = math.radians(50.0)  # angle between the two wall beams
    kp: float = 2.5                    # proportional gain on distance error
    kd: float = 0.6                    # derivative gain on distance error
    max_linear: float = 0.15           # forward speed on a straight wall
    min_linear: float = 0.04           # slowest forward speed while following
    max_angular: float = 1.0           # angular speed limit
    front_stop: float = 0.25           # obstacle closer than this -> TURN_AWAY
    front_clear: float = 0.40          # leave TURN_AWAY when front is this free
    front_slow: float = 0.60           # start slowing down below this distance
    wall_lost_distance: float = 0.50   # side wall farther than this = opening
    turn_angular: float = 0.9          # in-place turning speed in TURN_AWAY
    corner_linear: float = 0.10        # forward speed while rounding a corner
    corner_angular: float = 0.0        # 0 = auto: arc with radius desired_distance
    corner_advance: float = 0.03       # drive this far before arcing a corner
    corner_timeout: float = 0.0        # 0 = auto (one full circle) -> FIND_WALL


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
        self._state_start_time = 0.0
        self._prev_error = None
        self._prev_time = None
        self.last_error = 0.0  # exposed for logging / plotting

    # ------------------------------------------------------------------ helpers
    def _set_state(self, new_state, now):
        if new_state != self.state:
            self.state = new_state
            self._state_start_time = now
            self._prev_error = None  # reset derivative term after a switch

    def _time_in_state(self, now):
        return now - self._state_start_time

    def _advance_time(self):
        """Time to drive straight past a wall end before arcing."""
        return self.p.corner_advance / max(self.p.corner_linear, 1e-3)

    def _corner_angular(self):
        """Arc speed; auto (0) gives an arc of radius desired_distance."""
        p = self.p
        angular = p.corner_angular or p.corner_linear / max(p.desired_distance, 0.05)
        return min(angular, p.max_angular)

    def _corner_timeout(self):
        """Give up after ~one full circle (auto) so U-turns can finish."""
        if self.p.corner_timeout > 0:
            return self.p.corner_timeout
        return self._advance_time() + 2.0 * math.pi / self._corner_angular()

    def _front_distance(self, lidar: LidarProcessor):
        """Closest obstacle in a +-25 degree cone in front of the robot."""
        return lidar.min_distance_in(-math.radians(25.0), math.radians(25.0))

    # --------------------------------------------------------------- main step
    def compute(self, lidar: LidarProcessor, now: float) -> Command:
        """Return the velocity command for the current scan.

        `now` is a timestamp in seconds (used for the D-term and timers).
        """
        p = self.p
        front = self._front_distance(lidar)
        wall = estimate_wall(lidar, p.side, p.beam_angle, p.wall_lost_distance)
        wall_present = wall.valid
        # Hysteresis: re-acquiring a wall needs it clearly closer than the
        # "lost" threshold, so the state does not flicker at wall ends.
        wall_found = wall.valid and wall.distance < 0.8 * p.wall_lost_distance

        # ---- state transitions (front obstacle has priority) ----
        if front < p.front_stop:
            self._set_state(State.TURN_AWAY, now)
        elif self.state == State.TURN_AWAY:
            if front > p.front_clear:
                self._set_state(State.FOLLOW_WALL if wall_present
                                else State.FIND_WALL, now)
        elif self.state == State.FIND_WALL:
            if wall_found:
                self._set_state(State.FOLLOW_WALL, now)
        elif self.state == State.FOLLOW_WALL:
            if not wall_present:
                self._set_state(State.TURN_CORNER, now)
        elif self.state == State.TURN_CORNER:
            if wall_found and self._time_in_state(now) > self._advance_time():
                self._set_state(State.FOLLOW_WALL, now)
            elif self._time_in_state(now) > self._corner_timeout():
                self._set_state(State.FIND_WALL, now)

        # ---- actions ----
        if self.state == State.TURN_AWAY:
            # Rotate away from the followed wall (right-wall follower turns left).
            return Command(0.0, -p.side * p.turn_angular)

        if self.state == State.FIND_WALL:
            return Command(self._speed_for_front(front, 0.0), 0.0)

        if self.state == State.TURN_CORNER:
            if self._time_in_state(now) < self._advance_time():
                return Command(p.corner_linear, 0.0)
            # Arc around the wall end. The base turn rate gives an arc of
            # radius desired_distance; the P-term keeps the robot at
            # desired_distance from the wall tip measured by the LiDAR, which
            # compensates for detecting the corner a little late.
            tip = lidar.min_distance_in(*sorted((p.side * math.radians(20.0),
                                                 p.side * math.radians(160.0))))
            correction = p.kp * (tip - p.desired_distance) if math.isfinite(tip) else 0.0
            angular = clamp(self._corner_angular() + correction,
                            -0.3 * p.max_angular, p.max_angular)
            return Command(p.corner_linear, p.side * angular)

        # FOLLOW_WALL: PD on predicted distance to the wall.
        predicted = wall.distance - p.look_ahead * math.sin(wall.angle)
        error = predicted - p.desired_distance  # > 0: too far from the wall
        derivative = 0.0
        if self._prev_error is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if 1e-3 < dt < 1.0:
                derivative = (error - self._prev_error) / dt
        self._prev_error, self._prev_time = error, now
        self.last_error = error

        # Too far -> turn towards the wall (side sign gives the direction).
        angular = clamp(p.side * (p.kp * error + p.kd * derivative),
                        -p.max_angular, p.max_angular)
        speed = self._speed_for_front(front, angular)
        if wall.ends_ahead:
            # Corner coming: slow down so the turn starts on time.
            speed = min(speed, p.corner_linear)
        return Command(speed, angular)

    def _speed_for_front(self, front, angular):
        """Slow down near obstacles in front and during sharp turns."""
        p = self.p
        span = max(p.front_slow - p.front_stop, 1e-3)
        front_factor = clamp((front - p.front_stop) / span, 0.0, 1.0)
        turn_factor = 1.0 - 0.6 * abs(angular) / max(p.max_angular, 1e-3)
        speed = p.max_linear * front_factor * turn_factor
        return clamp(speed, p.min_linear, p.max_linear)
