"""Offline diagnosis of the first live-run ICP rejection; publishes nothing.

Usage: python3 diagnose_registration.py TRACE.json.gz REPORT.json.gz OUT.json
Use the recorded mrider_navigation_sim package on PYTHONPATH (numpy/scipy).
Only recorded scan and estimated odometry seed registration. Truth is read
separately to evaluate whether the simulated vehicle was physically stationary.
"""
import gzip
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from mrider_navigation_sim.sensor_core import register_scan


def read(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


trace, report = read(sys.argv[1]), read(sys.argv[2])
scans = trace['/scan']
odometry = trace['/mrider_sim/odometry']
estimates = trace['/mrider_sim/estimator_status']
failed = next(e for e in estimates if not e['valid'])
previous = next(e for e in reversed(estimates)
                if e['valid'] and e['stamp'] < failed['stamp'])
reference_stamp = previous['stamp'] - previous['reference_age_s']


def nearest(records, stamp):
    item = min(records, key=lambda e: abs(e['stamp'] - stamp))
    assert abs(item['stamp'] - stamp) < 1e-6, (item['stamp'], stamp)
    return item


def pose(stamp):
    x, y, z, w = nearest(odometry, stamp)['pose']
    return np.array([x, y, 2 * math.atan2(z, w)])


def points(scan):
    ranges = np.asarray(scan['ranges'])
    angles = scan['angle_min'] + np.arange(len(ranges)) * scan['angle_increment']
    valid = (np.isfinite(ranges) & (ranges > scan['range_min']) &
             (ranges < scan['range_max'] - .05))
    return np.column_stack((ranges[valid] * np.cos(angles[valid]),
                            ranges[valid] * np.sin(angles[valid])))


def rotation(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])


def initial_in(reference):
    last = pose(previous['stamp'])
    return np.r_[rotation(reference[2]).T @ (last[:2] - reference[:2]),
                 last[2] - reference[2]]


current = points(nearest(scans, failed['stamp']))
reference = points(nearest(scans, reference_stamp))
initial = initial_in(pose(reference_stamp))


def evaluate(a, b, guess):
    info = {}
    try:
        translation, angle, _ = register_scan(a, b, guess, .8, info)
        return dict(accepted=True, translation=translation.tolist(), angle=angle, **info)
    except ValueError as error:
        return dict(accepted=False, reason=str(error), **info)


exact = evaluate(current, reference, initial)
assert not exact['accepted'] and exact['reason'] == failed['reason'], exact
assert exact['supported_points'] == failed['supported_points'], exact


def candidate_fit(a, b, guess):
    """Instrument the existing optimization, WITHOUT treating it as accepted.

    This exposes point distances even when production rejects broad support.
    It reproduces the optimization in sensor_core.register_scan; the exact
    production call above is the authoritative rejection result.
    """
    tree = cKDTree(b)
    _, neighbors = tree.query(b, k=7)
    patches = b[neighbors]
    centered = patches - patches.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered)
    _, vectors = np.linalg.eigh(covariance)
    normals = vectors[:, :, 0]
    r, t = rotation(guess[2]), guess[:2].copy()
    for _ in range(30):
        moved = a @ r.T + t
        distances, indices = tree.query(moved)
        mask = distances <= min(.35, max(1e-6, float(np.quantile(distances, .9))))
        selected, target, n = moved[mask], b[indices[mask]], normals[indices[mask]]
        residual = np.sum(n * (target - selected), axis=1)
        design = np.column_stack((n[:, 0], n[:, 1],
                                  -n[:, 0] * selected[:, 1] + n[:, 1] * selected[:, 0]))
        assert mask.sum() >= 40 and np.linalg.cond(design) <= 1000
        delta = np.linalg.lstsq(design, residual, rcond=None)[0]
        dr = rotation(delta[2])
        r, t = dr @ r, dr @ t + delta[:2]
        if np.linalg.norm(delta) < 1e-6:
            break
    moved = a @ r.T + t
    distances, _ = tree.query(moved)
    accepted = distances <= min(.35, max(1e-6, float(np.quantile(distances, .9))))
    assert int(accepted.sum()) == failed['supported_points']
    return moved, distances, accepted


aligned, distances, supported = candidate_fit(current, reference, initial)
fresh = points(nearest(scans, previous['stamp']))
near_current = np.linalg.norm(current, axis=1) < 2.
near_reference = np.linalg.norm(reference, axis=1) < 2.
timeline = []
for e in estimates:
    if failed['stamp'] - 7 <= e['stamp'] <= failed['stamp'] + .25:
        timeline.append(e)

truth_tail = np.asarray(report['evaluation_truth_trace'][-200:])
intervals = np.diff([s['stamp'] for s in scans])
transitions, last = [], None
for wall, state in report['safety_history']:
    pair = state['state'], state['reason']
    if pair != last:
        transitions.append(dict(wall_since_first_s=wall-report['safety_history'][0][0],
                                state=pair[0], reason=pair[1],
                                camera_stamp=(state.get('camera') or {}).get('stamp')))
        last = pair

result = dict(
    reference_stamp=reference_stamp,
    failure_stamp=failed['stamp'],
    reference_age_at_failure_s=failed['stamp']-reference_stamp,
    reference_points=len(reference), current_points=len(current),
    initial_from_estimated_odometry=initial.tolist(),
    exact_pair_replay=exact,
    fresh_previous_scan_control=evaluate(current, fresh, np.zeros(3)),
    exclude_under_2m_diagnostic_only=evaluate(current[~near_current], reference[~near_reference], initial),
    near_current_count=int(near_current.sum()), near_reference_count=int(near_reference.sum()),
    unsupported_near_count=int((~supported & near_current).sum()),
    unsupported_far_count=int((~supported & ~near_current).sum()),
    scan_interval_s=dict(min=float(intervals.min()), max=float(intervals.max()),
                         mean=float(intervals.mean())),
    evaluation_only_last_200_truth_samples_span= np.ptp(truth_tail, axis=0).tolist(),
    safety_transitions=transitions,
    estimator_timeline=timeline,
    plot_data=dict(reference=reference.tolist(),current=current.tolist(),
                   aligned_current=aligned.tolist(),supported=supported.tolist(),
                   nearest_distance=distances.tolist()),
)
Path(sys.argv[3]).write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({k:v for k,v in result.items() if k not in
                 ('estimator_timeline','plot_data','safety_transitions')}, indent=2))
