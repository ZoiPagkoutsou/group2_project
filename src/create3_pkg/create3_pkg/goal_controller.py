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
EXPLORE      fallback after laps round the same obstacle on both sides:
             plain wall following (visits every corridor of a maze) until
             the robot passes the goal. A lap in EXPLORE means the robot is
             circling an island: it drives straight across to a different
             wall and explores along that one.
GOAL_REACHED within goal_tolerance of the goal, or as close as a wall lets
             the robot get to a goal right next to it: stop
UNREACHABLE  only with on_loop='stop': a lap round the obstacle, back at the
             hit point, means the goal is enclosed: stop

Lap = back at the hit point after turning about a full circle (turn
counting as in the Pledge algorithm). Driving into a dead end and back out
turns only ~180 deg, so it is not a lap. After a lap, on_loop='retry'
(default) goes round the obstacle the other way, and after a second lap
switches to EXPLORE.

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
    EXPLORE = 'EXPLORE'
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
    min_loop_length: float = 1.0       # ... after following this far ...
    lap_turn: float = math.radians(270.0)  # ... having turned this much = lap
    on_loop: str = 'retry'             # after a lap: 'retry' or 'stop'
    # Face the goal this well before calling the path blocked.
    hit_facing: float = math.radians(20.0)
    align_angular: float = 0.8         # rotation speed while lining up
    # Goal against a wall: reached this close if the way is blocked. Smaller
    # than robot radius + 10 cm wall, so it never counts through a wall.
    near_wall_radius: float = 0.25
    # Hit again without driving this far since leaving the wall ...
    min_progress: float = 0.05
    # ... then follow the wall at least this far before leaving again
    # (stops turning back and forth on the spot).
    commit_distance: float = 0.40
    # A leave after which the robot drove this far without getting closer
    # to the goal counts as failed (next leave must promise more progress).
    failed_leave_distance: float = 0.25
    # EXPLORE: drive at most this far to reach a different wall.
    cross_max: float = 3.0


class GoToGoalController:
    """DistBug on top of the LiDAR wall-follow controller."""

    def __init__(self, params: GoalParams, wall: WallFollowController):
        self.p = params
        self.wall = wall
        # Half width of the straight path towards the goal that must be free:
        # robot radius plus a margin, so it does not graze wall ends.
        self.half_width = wall.p.robot_radius + params.path_clearance
        self.state = GoalState.GO_TO_GOAL
        self.goal = None             # (x, y) in the pose frame
        self.hit_point = None        # where the current wall was hit
        self.followed = 0.0          # path length along the current wall
        self.last_xy = None
        self.distance_to_goal = float('inf')
        self.d_min = float('inf')    # closest the robot has been to the goal
        self.default_side = wall.p.side  # follow_side, used if no end is seen
        self.leave_count = 0         # scans in a row the leave rule held
        self.since_leave = float('inf')  # path length since leaving a wall
        self.turned = 0.0            # heading change along the current wall
        self.last_yaw = None
        self.laps = 0                # laps round the current obstacle
        self.aligning = False        # turning to put the obstacle on the side
        self.commit = 0.0            # wall length to follow before leaving
        self.crossing = None         # EXPLORE: heading (odom yaw) to a new wall
        self.cross_from = None       # where the crossing started
        self.crossings = 0           # how often the robot switched walls
        self.align_point = None      # odom (x, y) of the obstacle to line up with
        self.leave_d_min = None      # d_min when the robot last left a wall
        self.failed_leaves = 0       # leaves in a row that brought no progress

    @property
    def started(self):
        return self.goal is not None

    @property
    def following(self):
        return self.state in (GoalState.FOLLOW_WALL, GoalState.EXPLORE)

    @property
    def finished(self):
        return self.state in (GoalState.GOAL_REACHED, GoalState.UNREACHABLE)

    def set_start(self, x, y, yaw):
        """Fix the start pose and place the goal relative to it."""
        c, s = math.cos(yaw), math.sin(yaw)
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

        # Goal right next to a wall: the robot cannot get closer than the
        # wall allows, so being near it with the way blocked counts.
        if (self.distance_to_goal < p.near_wall_radius
                and free_to_goal < self.distance_to_goal):
            self.state = GoalState.GOAL_REACHED
            return Command(0.0, 0.0)

        # A straight drive gets the robot close enough to the goal (a goal
        # right next to a wall cannot be reached closer). Across a 10 cm wall
        # this is never true. Used both for 'blocked' and for leaving a wall,
        # so the two can never contradict each other.
        reach = p.near_wall_radius - self.wall.p.front_stop - 0.01
        goal_in_reach = self.distance_to_goal - free_to_goal <= reach

        if self.state == GoalState.GO_TO_GOAL:
            if self.last_xy is not None:
                self.since_leave += math.hypot(x - self.last_xy[0],
                                               y - self.last_xy[1])
                self.last_xy = (x, y)
            if (free_to_goal < min(p.blocked_distance, self.distance_to_goal)
                    and not goal_in_reach):
                if abs(goal_bearing) > p.hit_facing:
                    # Face the goal first (safe: the robot is round), so the
                    # obstacle is in front when the side is chosen.
                    return Command(0.0, clamp(p.k_heading * goal_bearing,
                                              -p.max_angular, p.max_angular))
                self._start_wall_follow(lidar, x, y, yaw, goal_bearing)
            else:
                return self._drive_to_goal(lidar, goal_bearing)

        # ---- FOLLOW_WALL / EXPLORE
        if self.crossing is not None:
            return self._cross(lidar, x, y, yaw)
        if self.aligning:
            if not self._obstacle_on_side(lidar, x, y, yaw):
                return Command(0.0, -self.wall.p.side * p.align_angular)
            self.aligning = False
            self.align_point = None
            self._reset_lap(x, y, yaw)

        self.followed += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        self.turned += float(wrap_angle(yaw - self.last_yaw))
        self.last_yaw = yaw

        if self.state == GoalState.FOLLOW_WALL:
            can_leave = (self.followed >= self.commit
                         and self._can_leave(lidar, goal_bearing, goal_in_reach))
            self.leave_count = self.leave_count + 1 if can_leave else 0
            if self.leave_count >= p.leave_confirm:
                self.state = GoalState.GO_TO_GOAL
                self.since_leave = 0.0
                self.leave_d_min = self.d_min
                return self._drive_to_goal(lidar, goal_bearing)

            if self._lap_closed(x, y):
                return self._lap_done(x, y, yaw)

        elif self._lap_closed(x, y):
            # EXPLORE went once round this wall: it is an island that does
            # not lead to the goal, so switch to a different wall.
            return self._start_crossing(lidar, x, y, yaw, goal_bearing)

        return self.wall.compute(lidar, now)

    def _start_wall_follow(self, lidar, x, y, yaw, goal_bearing):
        self.state = GoalState.FOLLOW_WALL
        self.leave_count = 0
        # Did the last leave get the robot closer to the goal? Only count it
        # as failed if the robot really drove (no-move re-hits are handled by
        # commit_distance below, and are often just LiDAR noise).
        if self.leave_d_min is not None:
            if self.distance_to_goal <= self.leave_d_min - 0.5 * self.p.leave_step:
                self.failed_leaves = 0
            elif self.since_leave >= self.p.failed_leave_distance:
                self.failed_leaves += 1
        # Back at a wall without having moved: commit to the wall for a while.
        self.commit = (self.p.commit_distance
                       if self.since_leave < self.p.min_progress else 0.0)
        # Hit again right after leaving = the same obstacle: keep the side
        # (and the wall-follow state), otherwise choose afresh.
        if self.since_leave > self.p.same_obstacle:
            self.wall.state = State.FIND_WALL  # clear TURN_AWAY from a past wall
            self.laps = 0
            if self.p.auto_side:
                self.wall.p.side = self._choose_side(lidar, goal_bearing)
        # Line up with the obstacle that blocked the way to the goal - not
        # with whatever wall happens to be nearest.
        block = lidar.blocking_point(goal_bearing, self.half_width)
        if block is not None:
            c, s = math.cos(yaw), math.sin(yaw)
            self.align_point = (x + c * block[0] - s * block[1],
                                y + s * block[0] + c * block[1])
        self.aligning = True
        self._reset_lap(x, y, yaw)

    def _reset_lap(self, x, y, yaw):
        """Start measuring a lap (hit point, path length, turning) here."""
        self.hit_point = (x, y)
        self.followed = 0.0
        self.turned = 0.0
        self.last_xy = (x, y)
        self.last_yaw = yaw

    def _lap_closed(self, x, y):
        """Back at the hit point after a full turn round the wall."""
        p = self.p
        hx, hy = self.hit_point
        return (self.followed > p.min_loop_length
                and math.hypot(x - hx, y - hy) < p.hit_return_radius
                and abs(self.turned) > p.lap_turn)

    def _start_crossing(self, lidar, x, y, yaw, goal_bearing):
        """Pick a direction away from the followed wall, towards open space."""
        s = self.wall.p.side
        best, best_score = None, 0.0
        for deg in range(-180, 180, 10):
            b = math.radians(deg)
            # Only directions on the other side from the followed wall.
            if not math.radians(15.0) <= -s * b <= math.radians(165.0):
                continue
            free = min(lidar.free_distance_along(b, self.half_width),
                       self.p.cross_max)
            # Prefer open space, then the direction of the goal.
            score = free - 0.3 * abs(float(wrap_angle(b - goal_bearing))) / math.pi
            if best is None or score > best_score:
                best, best_score = b, score
        self.crossings += 1
        if best is None or lidar.free_distance_along(best, self.half_width) < 0.3:
            # Nowhere to go: follow the other side of this wall instead.
            self.wall.p.side = -s
            self.aligning = True
            self._reset_lap(x, y, yaw)
            return Command(0.0, 0.0)
        self.crossing = float(wrap_angle(yaw + best))
        self.cross_from = (x, y)
        return Command(0.0, 0.0)

    def _cross(self, lidar, x, y, yaw):
        """Turn to the crossing heading, drive straight to the next wall."""
        p, w = self.p, self.wall.p
        error = float(wrap_angle(self.crossing - yaw))
        angular = clamp(p.k_heading * error, -p.max_angular, p.max_angular)
        if abs(error) > math.radians(10.0):
            return Command(0.0, angular)
        free = lidar.free_distance_ahead(w.robot_radius + w.side_margin)
        driven = math.hypot(x - self.cross_from[0], y - self.cross_from[1])
        if free < p.blocked_distance or driven > p.cross_max:
            # Arrived at a (new) wall: explore along it.
            self.crossing = None
            self.wall.state = State.FIND_WALL
            self.aligning = True
            self._reset_lap(x, y, yaw)
            return Command(0.0, 0.0)
        span = max(w.front_slow - w.front_stop, 1e-3)
        front_factor = clamp((free - w.front_stop) / span, 0.0, 1.0)
        return Command(max(p.max_linear * front_factor, w.min_linear), angular)

    def _lap_done(self, x, y, yaw):
        """Went once round the obstacle without finding a way to the goal."""
        self.laps += 1
        if self.p.on_loop == 'stop':
            self.state = GoalState.UNREACHABLE
            return Command(0.0, 0.0)
        if self.laps >= 2:
            # Both ways round failed (e.g. odometry drift hid the leave
            # point): follow walls everywhere until the goal is passed.
            self.state = GoalState.EXPLORE
            if self.wall.p.side != self.default_side:
                self.wall.p.side = self.default_side
                self.aligning = True
        else:
            # Try going round the other way.
            self.wall.p.side = -self.wall.p.side
            self.aligning = True
        self.wall.state = State.FIND_WALL
        self._reset_lap(x, y, yaw)
        return Command(0.0, 0.0)

    def _obstacle_on_side(self, lidar, x, y, yaw):
        """Check that the obstacle to follow is on the chosen side."""
        w = self.wall.p
        if self.align_point is not None:
            ax, ay = self.align_point
            bearing = float(wrap_angle(math.atan2(ay - y, ax - x) - yaw))
            return w.side * bearing >= -math.radians(10.0)
        nearest = lidar.nearest_point(-math.pi, math.pi, max_distance=w.follow_range)
        if nearest is None:
            return True  # nothing near: let the wall follower find a wall
        return w.side * nearest[1] >= -math.radians(10.0)

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

    def _can_leave(self, lidar, goal_bearing, goal_in_reach):
        """
        Leave rule of DistBug.

        Leave when the goal itself is reachable in a straight line, or when
        driving straight at it would end (blocked_distance before the next
        obstacle) at least leave_step closer than the robot has ever been.
        """
        p = self.p
        if goal_in_reach:
            return True
        free_to_goal = lidar.free_distance_along(goal_bearing, self.half_width)
        end_distance = self.distance_to_goal - free_to_goal + p.blocked_distance
        # Each failed leave (see _start_wall_follow) asks for one more step of
        # progress, so a misleading free path cannot trap the robot.
        step = p.leave_step * (1 + self.failed_leaves)
        return (free_to_goal > p.leave_clearance
                and end_distance <= self.d_min - step)

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
