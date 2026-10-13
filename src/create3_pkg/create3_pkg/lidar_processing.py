"""LiDAR processing for maze navigation (pure Python, no ROS dependency).

Keeping this module free of ROS makes it easy to unit-test offline with
recorded or synthetic scans.

Angle convention (ROS REP-103, robot frame):
    0 rad   = robot front
    +pi/2   = robot left
    -pi/2   = robot right
"""

import math
from dataclasses import dataclass

import numpy as np


def wrap_angle(angle):
    """Wrap an angle (scalar or array) to the range [-pi, pi)."""
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi


def circular_median_filter(values, size):
    """Median over `size` neighbouring beams (wraps around 360 deg).

    NaN marks an invalid beam and is ignored; a beam stays NaN only if more
    than half of its neighbourhood is invalid (so isolated dropouts are
    filled in, but real openings stay open).
    """
    if size <= 1 or values.size < size:
        return values
    half = size // 2
    padded = np.concatenate([values[-half:], values, values[:half]])
    windows = np.lib.stride_tricks.sliding_window_view(padded, size)
    n_valid = np.sum(np.isfinite(windows), axis=1)
    out = np.full(values.shape, np.nan)
    enough = n_valid > half
    if np.any(enough):
        out[enough] = np.nanmedian(windows[enough], axis=1)
    return out


@dataclass
class ScanConfig:
    """How the LiDAR is mounted relative to the robot."""

    # Rotation added to every scan angle so that 0 rad points to the robot front.
    # Find it once by putting an object in front of the robot (see README).
    angle_offset: float = 0.0
    # True if the LiDAR reports angles clockwise (e.g. mounted upside down).
    reversed: bool = False
    # Readings outside [min_valid_range, max_valid_range] are ignored (metres).
    min_valid_range: float = 0.05
    max_valid_range: float = 8.0
    # Half-width of the angular window used to compute a robust distance (rad).
    sector_half_width: float = math.radians(8.0)
    # Each beam is replaced by the median of this many neighbouring beams
    # (odd number, 1 = off). Removes single-beam spikes before any minimum is
    # taken. Keep it small: a thin wall seen end-on covers only a few beams
    # and a large filter would erase it.
    median_filter_size: int = 3


class LidarProcessor:
    """Converts a raw LaserScan into robust distances in robot-frame directions."""

    def __init__(self, config: ScanConfig):
        self.config = config
        self._angles = np.empty(0)  # robot-frame angle of each valid reading
        self._ranges = np.empty(0)  # range of each valid reading

    def update(self, ranges, angle_min, angle_increment,
               range_min=0.0, range_max=float('inf')):
        """Store a new scan. Arguments mirror sensor_msgs/LaserScan fields."""
        ranges = np.asarray(ranges, dtype=float)
        raw_angles = angle_min + angle_increment * np.arange(ranges.size)
        if self.config.reversed:
            raw_angles = -raw_angles
        angles = wrap_angle(raw_angles + self.config.angle_offset)

        lo = max(range_min, self.config.min_valid_range)
        hi = min(range_max, self.config.max_valid_range)
        valid = np.isfinite(ranges) & (ranges >= lo) & (ranges <= hi)
        ranges = np.where(valid, ranges, np.nan)
        ranges = circular_median_filter(ranges, self.config.median_filter_size)
        valid = np.isfinite(ranges)

        self._angles = angles[valid]
        self._ranges = ranges[valid]

    @property
    def has_data(self):
        return self._ranges.size > 0

    def distance_at(self, angle, half_width=None):
        """Median range inside a window around `angle` (robot frame).

        The median rejects single noisy readings. Returns inf when there is
        no valid reading in the window (open space or out of range).
        """
        if half_width is None:
            half_width = self.config.sector_half_width
        diff = np.abs(wrap_angle(self._angles - angle))
        in_window = self._ranges[diff <= half_width]
        if in_window.size == 0:
            return float('inf')
        return float(np.median(in_window))

    def min_distance_in(self, angle_from, angle_to):
        """Closest valid reading between two robot-frame angles (inclusive)."""
        lo, hi = sorted((angle_from, angle_to))
        mask = (self._angles >= lo) & (self._angles <= hi)
        if not np.any(mask):
            return float('inf')
        return float(np.min(self._ranges[mask]))


@dataclass
class WallEstimate:
    """Geometry of the wall on one side of the robot."""

    distance: float  # perpendicular distance robot -> wall (m)
    angle: float     # rad; > 0 means the robot is heading INTO the wall
    valid: bool
    ends_ahead: bool = False  # the wall stops a little ahead (corner coming)


def estimate_wall(processor: LidarProcessor, side: int, beam_angle: float,
                  max_distance: float = float('inf')):
    """Estimate distance and relative angle of the wall on one side.

    Uses two beams: one perpendicular to the robot (b) and one rotated
    `beam_angle` towards the front (a). With both points on the wall line:

        psi = atan2(b - a*cos(theta), a*sin(theta))
        D   = b * cos(psi)

    The wall counts as present while the perpendicular beam b sees it
    (b < max_distance). If beam a already looks past the end of the wall
    (an opening ahead), the angle is unknown and set to 0 so the robot keeps
    going straight instead of cutting the corner.

    side: +1 = left wall, -1 = right wall.
    """
    b = processor.distance_at(side * math.pi / 2.0)
    a = processor.distance_at(side * (math.pi / 2.0 - beam_angle))
    if not (math.isfinite(b) and b < max_distance):
        return WallEstimate(distance=b, angle=0.0, valid=False)
    a_on_wall = math.isfinite(a) and a * math.cos(beam_angle) < max_distance
    if not a_on_wall:
        return WallEstimate(distance=b, angle=0.0, valid=True, ends_ahead=True)
    psi = math.atan2(b - a * math.cos(beam_angle), a * math.sin(beam_angle))
    return WallEstimate(distance=b * math.cos(psi), angle=psi, valid=True)


@dataclass
class Surroundings:
    """Perception summary that the navigation node can use."""

    front: float
    left: float
    right: float
    front_left: float
    front_right: float
    front_open: bool
    left_open: bool
    right_open: bool

    @property
    def situation(self):
        """Classify the local maze structure from which directions are open."""
        n_open = self.front_open + self.left_open + self.right_open
        if n_open == 0:
            return 'dead_end'
        if n_open >= 2:
            return 'junction'
        if self.front_open:
            return 'corridor'
        return 'corner_left' if self.left_open else 'corner_right'


def describe_surroundings(processor: LidarProcessor, opening_distance: float):
    """Front/left/right distances plus which directions are open."""
    front = processor.distance_at(0.0)
    left = processor.distance_at(math.pi / 2.0)
    right = processor.distance_at(-math.pi / 2.0)
    return Surroundings(
        front=front,
        left=left,
        right=right,
        front_left=processor.distance_at(math.pi / 4.0),
        front_right=processor.distance_at(-math.pi / 4.0),
        front_open=front > opening_distance,
        left_open=left > opening_distance,
        right_open=right > opening_distance,
    )
