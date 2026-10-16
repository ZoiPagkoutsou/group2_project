"""
LiDAR processing for maze navigation (pure Python, no ROS dependency).

Keeping this module free of ROS makes it easy to unit-test offline with
recorded or synthetic scans.

All distances and angles are measured from the ROBOT CENTRE, not from the
LiDAR (the LiDAR position is given in ScanConfig). Angle convention
(ROS REP-103, robot frame):
    0 rad   = robot front
    +pi/2   = robot left
    -pi/2   = robot right
"""

from dataclasses import dataclass
import math

import numpy as np


def wrap_angle(angle):
    """Wrap an angle (scalar or array) to the range [-pi, pi)."""
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi


def circular_median_filter(values, size):
    """
    Median over `size` neighbouring beams (wraps around 360 deg).

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
    # Find it once by putting an object in front of the robot (see lidar_check).
    angle_offset: float = 0.0
    # True if the LiDAR reports angles clockwise (e.g. mounted upside down).
    scan_reversed: bool = False
    # LiDAR position relative to the robot centre (m, +x = forward).
    # Create 3: 35 cm body, LiDAR 20 cm behind the front edge -> 0.175 - 0.20.
    lidar_x: float = -0.025
    # Readings outside [min_valid_range, max_valid_range] are ignored (metres,
    # measured from the LiDAR).
    min_valid_range: float = 0.05
    max_valid_range: float = 8.0
    # Parts of the robot the LiDAR sees (cables, brackets): readings closer
    # than max_range between two RAW LiDAR angles (rad, as in the LaserScan,
    # before angle_offset/scan_reversed) are ignored. ((from, to, max_range), ...)
    ignore_sectors: tuple = ()
    # Half-width of the angular window used to compute a robust distance (rad).
    sector_half_width: float = math.radians(8.0)
    # Each beam is replaced by the median of this many neighbouring beams
    # (odd number, 1 = off). Removes single-beam spikes before any minimum is
    # taken. Keep it small: a thin wall seen end-on covers only a few beams
    # and a large filter would erase it.
    median_filter_size: int = 3


class LidarProcessor:
    """Converts a raw LaserScan into obstacle points around the robot centre."""

    def __init__(self, config: ScanConfig):
        self.config = config
        self._x = np.empty(0)       # robot-frame point coordinates (m)
        self._y = np.empty(0)
        self._angles = np.empty(0)  # bearing of each point from the centre
        self._ranges = np.empty(0)  # distance of each point from the centre

    def update(self, ranges, angle_min, angle_increment,
               range_min=0.0, range_max=float('inf')):
        """Store a new scan. Arguments mirror sensor_msgs/LaserScan fields."""
        ranges = np.asarray(ranges, dtype=float)
        raw_angles = wrap_angle(angle_min + angle_increment * np.arange(ranges.size))
        angles = -raw_angles if self.config.scan_reversed else raw_angles
        angles = wrap_angle(angles + self.config.angle_offset)

        lo = max(range_min, self.config.min_valid_range)
        hi = min(range_max, self.config.max_valid_range)
        valid = np.isfinite(ranges) & (ranges >= lo) & (ranges <= hi)
        for start, end, max_range in self.config.ignore_sectors:
            start, end = sorted((start, end))
            valid &= ~((raw_angles >= start) & (raw_angles <= end)
                       & (ranges < max_range))
        ranges = np.where(valid, ranges, np.nan)
        ranges = circular_median_filter(ranges, self.config.median_filter_size)
        valid = np.isfinite(ranges)

        # Shift the points from the LiDAR to the robot centre.
        self._x = self.config.lidar_x + ranges[valid] * np.cos(angles[valid])
        self._y = ranges[valid] * np.sin(angles[valid])
        self._angles = np.arctan2(self._y, self._x)
        self._ranges = np.hypot(self._x, self._y)

    @property
    def has_data(self):
        return self._ranges.size > 0

    def distance_at(self, angle, half_width=None):
        """
        Median distance inside a window around `angle` (robot frame).

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
        mask = self._sector_mask(angle_from, angle_to)
        if not np.any(mask):
            return float('inf')
        return float(np.min(self._ranges[mask]))

    def nearest_point(self, angle_from, angle_to, max_distance=float('inf')):
        """(distance, bearing) of the closest point in a sector, or None."""
        mask = self._sector_mask(angle_from, angle_to) & (self._ranges <= max_distance)
        if not np.any(mask):
            return None
        i = np.argmin(np.where(mask, self._ranges, np.inf))
        return float(self._ranges[i]), float(self._angles[i])

    def free_distance_ahead(self, half_width):
        """
        Return how far the robot can drive straight before touching.

        Touching = an obstacle enters a circle of radius `half_width`
        around the robot centre.
        This is the swept path of a round robot: only points ahead of the
        centre with |y| < half_width can be hit. Negative = already too close.
        """
        mask = (self._x > 0.0) & (np.abs(self._y) < half_width)
        if not np.any(mask):
            return float('inf')
        x, y = self._x[mask], self._y[mask]
        return float(np.min(x - np.sqrt(half_width ** 2 - y ** 2)))

    def _sector_mask(self, angle_from, angle_to):
        lo, hi = sorted((angle_from, angle_to))
        return (self._angles >= lo) & (self._angles <= hi)


@dataclass
class Surroundings:
    """
    Perception summary that the navigation node can use.

    Distances are from the robot centre.
    """

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
