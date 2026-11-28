"""Metric path checks before FollowPath; independent of ROS and simulator truth."""
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def require_matching_resolution(navigation, slam_resolution):
    if not math.isfinite(slam_resolution) or slam_resolution <= 0:
        raise ValueError('invalid_slam_resolution')
    for name in ('global_costmap', 'local_costmap'):
        value = navigation[name][name]['ros__parameters']['resolution']
        if not math.isclose(value, slam_resolution, rel_tol=0., abs_tol=1e-8):
            raise ValueError(f'{name}_resolution_must_match_slam: {value} != {slam_resolution}')


def validate_path(points, current, wheelbase, max_steering, lookahead, margin=1.05):
    """Reject tight, sparse, reverse or disconnected paths before execution.

    Three-point circumcircle curvature checks the *positions*, including the
    smoother output, rather than trusting pose yaw or the planner parameter.
    Also check the first interpolated Pure Pursuit circle from the current
    estimated pose. This is not a guarantee against subsequent tracking error;
    the independent live safety gate must retain the same physical limit.
    """
    values = (*current, wheelbase, max_steering, lookahead, margin)
    p = np.asarray(points, dtype=float)
    if (p.ndim != 2 or p.shape[1] != 3 or len(p) < 2 or not np.isfinite(p).all()
            or not all(math.isfinite(v) for v in values)
            or min(wheelbase, lookahead) <= 0 or not 0 < max_steering < math.pi/2 or margin < 1):
        raise ValueError('invalid_path_geometry')
    distance = np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)
    for i in np.flatnonzero(distance < 1e-6):
        if abs(wrap(p[i+1, 2]-p[i, 2])) > 1e-3:
            raise ValueError('path_rotation_in_place')
    p = p[np.r_[True, distance >= 1e-6]]
    if len(p) < 2:
        raise ValueError('empty_path_motion')
    segments = np.diff(p[:, :2], axis=0)
    distance = np.linalg.norm(segments, axis=1)
    if max(distance) > .5:
        raise ValueError('path_sampling_too_sparse')
    if math.dist(p[0, :2], current[:2]) > .35:
        raise ValueError('path_start_disconnected')
    tangents = np.column_stack((np.cos(p[:-1, 2]), np.sin(p[:-1, 2])))
    if np.any(np.sum(tangents*segments, axis=1) < -.01*distance):
        raise ValueError('path_reverse_segment')
    physical_curvature = math.tan(max_steering)/wheelbase
    allowed = physical_curvature/margin
    curvature = np.zeros(max(0, len(p)-2))
    if len(p) > 2:
        chords = np.linalg.norm(p[2:, :2]-p[:-2, :2], axis=1)
        if np.any(chords < 1e-6):
            raise ValueError('path_cusp')
        cross = segments[:-1, 0]*segments[1:, 1]-segments[:-1, 1]*segments[1:, 0]
        curvature = 2*np.abs(cross)/(distance[:-1]*distance[1:]*chords)
    maximum = float(max(curvature, default=0.))
    if maximum > allowed + 1e-4:
        raise ValueError(f'path_curvature: {maximum:.6f} > {allowed:.6f}')
    c, s = math.cos(current[2]), math.sin(current[2])
    relative = (p[:, :2]-np.asarray(current[:2])) @ np.array([[c, -s], [s, c]])
    # Match the initial RPP nearest-point search within the local costmap extent.
    prefix = np.r_[0., np.cumsum(distance)] <= 4.
    nearest = int(np.argmin(np.where(prefix, np.linalg.norm(relative, axis=1), np.inf)))
    relative = relative[nearest:]
    outside = np.flatnonzero(np.linalg.norm(relative, axis=1) >= lookahead)
    carrot = relative[-1]
    if len(outside):
        i = outside[0]; carrot = relative[i]
        if i:
            a = relative[i-1]; d = relative[i]-a
            aa = np.dot(d, d); bb = 2*np.dot(a, d); cc = np.dot(a, a)-lookahead**2
            t = (-bb+math.sqrt(max(0., bb*bb-4*aa*cc)))/(2*aa)
            carrot = a+min(1., max(0., t))*d
    length2 = float(np.dot(carrot, carrot))
    initial = 2*float(carrot[1])/length2 if length2 > .001 else 0.
    if carrot[0] < 0 or abs(initial) > allowed+1e-4:
        raise ValueError(f'initial_tracking_curvature: {initial:.6f}, limit {allowed:.6f}')
    return dict(max_path_curvature=maximum, initial_tracking_curvature=initial,
                max_path_steering=math.atan(wheelbase*maximum),
                physical_max_curvature=physical_curvature, allowed_curvature=allowed,
                length_m=float(sum(distance)), samples=len(p))
