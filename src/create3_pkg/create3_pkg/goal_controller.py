"""
Go-to-goal navigation with the DistBug algorithm (pure Python, no ROS).

The goal is fixed before the start, relative to the start pose:
goal_x metres forward and goal_y metres to the left (ROS REP-103).
Example: goal_x=2.0, goal_y=1.0 -> 2 m forward, 1 m left.

States
------
GO_TO_GOAL   turn towards the goal and drive straight at it
FOLLOW_WALL  path to the goal blocked: follow the obstacle with the
             wall-follow controller until the LiDAR shows a free straight
             path that ends closer to the goal than the robot has ever been
             (DistBug leave rule)
GOAL_REACHED within goal_tolerance of the goal: stop
UNREACHABLE  followed the obstacle all the way round, back to the hit point:
             the goal cannot be reached, stop

Side choice (auto_side): when the robot hits an obstacle it looks with the
LiDAR for both ends of it and goes round the end that gives the shorter way
(robot -> end -> goal). Going round the left end = obstacle on the right =
right-wall following, and the other way round. The side is kept until the
robot leaves that obstacle.

Every leave point is at least leave_step closer to the goal than any point
before it, so the robot cannot loop: it reaches the goal whenever it is
reachable.
"""

from dataclasses import dataclass
from enum import Enum
import math

from create3_pkg.controller import clamp, Command, State, WallFollowController
from create3_pkg.lidar_processing import LidarProcessor, wrap_angle


class GoalState(Enum):
    GO_TO_GOAL = 'GO_TO_GOAL'
    FOLLOW_WALL = 'FOLLOW_WALL'
    GOAL_REACHED = 'GOAL_REACHED'
    UNREACHABLE = 'UNREACHABLE'


@dataclass
class GoalParams:
    """Tuning parameters. Distances in m, speeds in m/s and rad/s."""

    goal_x: float = 0.0                # goal, metres forward of the start
    goal_y: float = 0.0                # goal, metres left of the start
    goal_tolerance: float = 0.1       # stop this close to the goal
    k_heading: float = 1.5             # turn rate (rad/s) per rad heading error
    turn_in_place: float = math.radians(30.0)  # larger error: rotate only
    max_linear: float = 0.15
    max_angular: float = 1.0
    blocked_distance: float = 0.20     # free path to goal shorter -> wall
    path_clearance: float = 0.03       # side margin of the path checked to goal
    leave_clearance: float = 0.40      # free path needed to leave the wall
    leave_step: float = 0.10           # leave only if this much closer than ever
    auto_side: bool = True             # choose the side at each obstacle
    edge_gap: float = 0.40             # wider gap between points = obstacle end
    leave_confirm: int = 2             # scans in a row the leave rule must hold
    same_obstacle: float = 0.50        # hit again this soon after leaving: keep side
    hit_return_radius: float = 0.20    # back at the hit point ...
    min_loop_length: float = 1.0       # ... after following this far: give up


class GoToGoalController:
    """DistBug on top of the LiDAR wall-follow controller."""

    def __init__(self, params: GoalParams, wall: WallFollowController):
        self.p = params
        self.wall = wall
        # Half width of the straight path towards the goal that must be free:
        # robot radius plus a margin, so it does not graze wall ends.
        self.half_width = wall.p.robot_radius + params.path_clearance
        self.state = GoalState.GO_TO_GOAL
        self.start = None            # (x, y) odom frame
        self.goal = None             # (x, y) odom frame
        self.hit_point = None        # where the current wall was hit
        self.hit_distance = None     # distance to goal at the hit point
        self.followed = 0.0          # path length along the current wall
        self.last_xy = None
        self.distance_to_goal = float('inf')
        self.d_min = float('inf')    # closest the robot has been to the goal
        self.default_side = wall.p.side  # follow_side, used if no end is seen
        self.leave_count = 0         # scans in a row the leave rule held
        self.since_leave = float('inf')  # path length since leaving a wall

    @property
    def started(self):
        return self.goal is not None

    @property
    def finished(self):
        return self.state in (GoalState.GOAL_REACHED, GoalState.UNREACHABLE)

    def set_start(self, x, y, yaw):
        """Fix the start pose (odom frame) and place the goal relative to it."""
        c, s = math.cos(yaw), math.sin(yaw)
        self.start = (x, y)
        self.goal = (x + c * self.p.goal_x - s * self.p.goal_y,
                     y + s * self.p.goal_x + c * self.p.goal_y)

    def compute(self, lidar: LidarProcessor, x, y, yaw, now) -> Command:
        """Return the velocity command for the current scan and pose."""
        p = self.p
        gx, gy = self.goal
        self.distance_to_goal = math.hypot(gx - x, gy - y)
        self.d_min = min(self.d_min, self.distance_to_goal)

        if self.finished:
            return Command(0.0, 0.0)
        if self.distance_to_goal < p.goal_tolerance:
            self.state = GoalState.GOAL_REACHED
            return Command(0.0, 0.0)

        # Goal direction in the robot frame and the free straight path
        # towards it (a wall beyond the goal does not block it).
        goal_bearing = float(wrap_angle(math.atan2(gy - y, gx - x) - yaw))
        free_to_goal = lidar.free_distance_along(goal_bearing, self.half_width)

        if self.state == GoalState.GO_TO_GOAL:
            if self.last_xy is not None:
                self.since_leave += math.hypot(x - self.last_xy[0],
                                               y - self.last_xy[1])
                self.last_xy = (x, y)
            if free_to_goal < min(p.blocked_distance, self.distance_to_goal):
                self._start_wall_follow(lidar, x, y, goal_bearing)
            else:
                return self._drive_to_goal(lidar, goal_bearing)

        # ---- FOLLOW_WALL
        self.followed += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

        self.leave_count = self.leave_count + 1 if self._can_leave(free_to_goal) else 0
        if self.leave_count >= p.leave_confirm:
            self.state = GoalState.GO_TO_GOAL
            self.since_leave = 0.0
            return self._drive_to_goal(lidar, goal_bearing)

        hx, hy = self.hit_point
        if (self.followed > p.min_loop_length
                and math.hypot(x - hx, y - hy) < p.hit_return_radius):
            self.state = GoalState.UNREACHABLE
            return Command(0.0, 0.0)

        return self.wall.compute(lidar, now)

    def _start_wall_follow(self, lidar, x, y, goal_bearing):
        self.state = GoalState.FOLLOW_WALL
        self.hit_point = (x, y)
        self.hit_distance = self.distance_to_goal
        self.followed = 0.0
        self.last_xy = (x, y)
        self.leave_count = 0
        # Hit again right after leaving = the same obstacle: keep the side
        # (and the wall-follow state), otherwise choose afresh.
        if self.since_leave > self.p.same_obstacle:
            self.wall.state = State.FIND_WALL  # clear TURN_AWAY from a past wall
            if self.p.auto_side:
                self.wall.p.side = self._choose_side(lidar, goal_bearing)

    def _choose_side(self, lidar, goal_bearing):
        """
        Wall-follow side for the obstacle just hit (-1 right, +1 left).

        Cost of going round an end = distance robot -> end + end -> goal.
        Left end (turn=+1): robot passes left of it, obstacle on its right.
        """
        gx = self.distance_to_goal * math.cos(goal_bearing)
        gy = self.distance_to_goal * math.sin(goal_bearing)
        best_side, best_cost = self.default_side, float('inf')
        for turn, side in ((+1, -1), (-1, +1)):
            end = lidar.obstacle_edge(goal_bearing, self.half_width, turn,
                                      self.p.edge_gap)
            if end is None:
                continue
            ex, ey = end
            cost = math.hypot(ex, ey) + math.hypot(gx - ex, gy - ey)
            if cost < best_cost:
                best_side, best_cost = side, cost
        return best_side

    def _can_leave(self, free_to_goal):
        """
        Leave rule of DistBug.

        Leave when the goal itself is reachable in a straight line, or when
        driving straight at it would end (blocked_distance before the next
        obstacle) at least leave_step closer than the robot has ever been.
        """
        p = self.p
        if free_to_goal >= self.distance_to_goal:
            return True
        end_distance = self.distance_to_goal - free_to_goal + p.blocked_distance
        return (free_to_goal > p.leave_clearance
                and end_distance <= self.d_min - p.leave_step)

    def _drive_to_goal(self, lidar, goal_bearing):
        """Turn towards the goal; drive forward once roughly facing it."""
        p, w = self.p, self.wall.p
        angular = clamp(p.k_heading * goal_bearing, -p.max_angular, p.max_angular)
        heading_factor = 1.0 - abs(goal_bearing) / p.turn_in_place
        if heading_factor <= 0.0:
            return Command(0.0, angular)  # rotate in place first
        # Slow down near the goal and near anything straight ahead.
        free = lidar.free_distance_ahead(self.half_width)
        span = max(w.front_slow - w.front_stop, 1e-3)
        front_factor = clamp((free - w.front_stop) / span, 0.0, 1.0)
        goal_factor = clamp(self.distance_to_goal / 0.3, 0.3, 1.0)
        return Command(p.max_linear * heading_factor * front_factor * goal_factor,
                       angular)
