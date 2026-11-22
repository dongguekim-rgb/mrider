"""Rear-axle Pure Pursuit and input watchdogs (SI units, positive left)."""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class Config:
    wheelbase: float = 0.65
    max_speed: float = 0.8
    max_steering: float = 0.3927
    lookahead: float = 0.7
    slowdown_distance: float = 1.5
    goal_tolerance: float = 0.12
    input_timeout: float = 0.5
    future_tolerance: float = 0.05
    control_rate: float = 50.0
    frame: str = 'map'
    rear_frame: str = 'base_link'

    def __post_init__(self):
        for f in fields(self):
            if f.name not in ('frame', 'rear_frame'):
                value = getattr(self, f.name)
                if not math.isfinite(value) or value <= 0:
                    raise ValueError('invalid parameter: ' + f.name)
        if self.max_steering >= math.pi / 2:
            raise ValueError('steering must be below pi/2')
        if self.slowdown_distance <= self.goal_tolerance:
            raise ValueError('slowdown_distance must exceed goal_tolerance')
        if not self.frame or not self.rear_frame or self.frame == self.rear_frame:
            raise ValueError('distinct nonempty frames required')


@dataclass(frozen=True)
class Command:
    speed: float = 0.0
    steering: float = 0.0
    reason: str = 'missing_input'


def finite(*values):
    return all(math.isfinite(v) for v in values)


class Controller:
    """Caller serializes access. now=ROS/source clock; wall=monotonic clock.

    Paths are ordered, non-self-intersecting forward polylines. No TF lookup.
    Invalid updates immediately invalidate the corresponding cached input.
    """
    def __init__(self, config=Config()):
        self.config = config
        self.path = None
        self.odom = None
        self.last_now = None
        self.path_stamp = None
        self.odom_stamp = None

    def invalidate_path(self):
        self.path = None

    def invalidate_odom(self):
        self.odom = None

    def _stamp_ok(self, stamp, now, wall, previous):
        return (finite(stamp, now, wall) and stamp > 0 and now > 0
                and -self.config.future_tolerance <= now - stamp <= self.config.input_timeout
                and (previous is None or stamp > previous))

    def set_path(self, points, frame, stamp, now, wall):
        previous = self.path_stamp
        self.path = None
        try:
            points = tuple((float(x), float(y)) for x, y in points)
            if (frame != self.config.frame or len(points) < 2
                    or not all(finite(*p) and max(abs(v) for v in p) <= 1e6 for p in points)
                    or not self._stamp_ok(stamp, now, wall, previous)):
                return False
            # Remove adjacent duplicate points; all-identical paths are invalid.
            points = tuple(p for i, p in enumerate(points) if i == 0 or p != points[i-1])
            if len(points) < 2 or any(math.hypot(b[0]-a[0], b[1]-a[1]) < 1e-9
                                      for a, b in zip(points, points[1:])):
                return False
            self.path_stamp = stamp
            self.path = (points, stamp, wall)
            return True
        except (ValueError, TypeError, OverflowError):
            return False

    def set_odometry(self, x, y, yaw, frame, child_frame, stamp, now, wall):
        previous = self.odom_stamp
        self.odom = None
        if (frame != self.config.frame or child_frame != self.config.rear_frame
                or not finite(x, y, yaw) or max(abs(x), abs(y), abs(yaw)) > 1e6
                or not self._stamp_ok(stamp, now, wall, previous)):
            return False
        self.odom_stamp = stamp
        self.odom = ((x, y, yaw), stamp, wall)
        return True

    def command(self, now, wall):
        if not finite(now, wall) or (self.last_now is not None and now < self.last_now):
            self.path = self.odom = None
            self.path_stamp = self.odom_stamp = None
            self.last_now = now if math.isfinite(now) else None
            return Command(reason='clock_reset')
        self.last_now = now
        if self.path is None or self.odom is None:
            return Command()
        for sample in (self.path, self.odom):
            age, wall_age = now - sample[1], wall - sample[2]
            if (not -self.config.future_tolerance <= age <= self.config.input_timeout
                    or not 0 <= wall_age <= self.config.input_timeout):
                return Command(reason='stale_input')
        points = self.path[0]
        x, y, yaw = self.odom[0]
        goal_distance = math.hypot(points[-1][0]-x, points[-1][1]-y)
        if goal_distance <= self.config.goal_tolerance:
            return Command(reason='goal_reached')
        # Project onto closest segment, then advance lookahead by arc length.
        best = None
        for i, (a, b) in enumerate(zip(points, points[1:])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            t = max(0.0, min(1.0, ((x-a[0])*dx+(y-a[1])*dy)/(length*length)))
            px, py = a[0]+t*dx, a[1]+t*dy
            distance = math.hypot(x-px, y-py)
            if best is None or distance < best[0]:
                best = (distance, i, (px, py))
        _, i, target = best
        remaining = self.config.lookahead
        for endpoint in points[i+1:]:
            length = math.hypot(endpoint[0]-target[0], endpoint[1]-target[1])
            if length >= remaining and length > 0:
                target = (target[0]+remaining*(endpoint[0]-target[0])/length,
                          target[1]+remaining*(endpoint[1]-target[1])/length)
                break
            remaining -= length
            target = endpoint
        dx, dy = target[0]-x, target[1]-y
        forward = math.cos(yaw)*dx + math.sin(yaw)*dy
        lateral = -math.sin(yaw)*dx + math.cos(yaw)*dy
        if forward <= 1e-9:
            return Command(reason='target_behind')
        steering = math.atan2(2*self.config.wheelbase*lateral, dx*dx+dy*dy)
        steering = max(-self.config.max_steering, min(self.config.max_steering, steering))
        speed = self.config.max_speed * min(1.0, goal_distance/self.config.slowdown_distance)
        if not finite(speed, steering):
            return Command(reason='invalid_geometry')
        return Command(speed, steering, 'tracking')
