"""Evaluation-only attribution of failed returns to simulated objects.

Fits the two known simulated boxes to recorded scans of a stationary car.
Simulator geometry and truth pose are used ONLY here to explain the data,
never as input to registration, localization, planning or the live vehicle.
Usage: python3 attribute_returns.py TRACE.gz REPORT.gz REGISTRATION.json OUT.json
"""
import gzip
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from mrider_navigation_sim.geometry import ray_distances
from mrider_navigation_sim.indoor import box
from mrider_navigation_sim.large_office import walls_and_furniture


with gzip.open(sys.argv[1], 'rt') as f:
    trace = json.load(f)
with gzip.open(sys.argv[2], 'rt') as f:
    report = json.load(f)
analysis = json.loads(Path(sys.argv[3]).read_text())
pose = np.asarray(report['evaluation_truth_trace'][-1])
truth = np.asarray(report['evaluation_truth_trace'][-200:])
assert np.all(np.ptp(truth, axis=0) == 0), 'Requires stationary evaluation window'
walls = walls_and_furniture()


def scan_at(stamp):
    s = min(trace['/scan'], key=lambda e: abs(e['stamp']-stamp))
    assert abs(s['stamp']-stamp) < 1e-6
    return s


def fit(scan):
    ranges = np.asarray(scan['ranges'])
    angles = pose[2] + scan['angle_min'] + np.arange(len(ranges))*scan['angle_increment']
    world = pose[:2] + np.column_stack((ranges*np.cos(angles), ranges*np.sin(angles)))
    first = world[(ranges < 2) & (np.abs(world[:, 0]-24.) < .35)]
    second = world[(ranges < 4) & (np.abs(world[:, 1]-16.) < .35)]
    guess = [float(np.median(first[:, 1])), float(np.median(second[:, 0]))]

    def ranges_for(values):
        y, x = values
        layers = np.column_stack([
            ray_distances(*pose[:2], angles, walls),
            ray_distances(*pose[:2], angles, [box(24., y)]),
            ray_distances(*pose[:2], angles, [box(x, 16.)])])
        return layers

    opt = least_squares(lambda values: ranges_for(values).min(axis=1)-ranges,
                        guess, bounds=([0., 0.], [32., 52.]), loss='soft_l1',
                        f_scale=.01, max_nfev=500)
    layers = ranges_for(opt.x)
    labels = np.argmin(layers, axis=1)
    return dict(values=opt.x.tolist(), initial=guess, success=bool(opt.success),
                range_rmse_m=float(np.sqrt(np.mean((layers.min(axis=1)-ranges)**2))),
                ray_labels=labels.tolist())


reference = scan_at(analysis['reference_stamp'])
current = scan_at(analysis['failure_stamp'])
ref_fit, cur_fit = fit(reference), fit(current)
assert ref_fit['success'] and cur_fit['success']
assert max(ref_fit['range_rmse_m'], cur_fit['range_rmse_m']) < .01, 'Geometry attribution is not a good fit'
ranges = np.asarray(current['ranges'])
valid = np.isfinite(ranges) & (ranges > current['range_min']) & (ranges < current['range_max']-.05)
beam_indices = np.flatnonzero(valid)
bad = beam_indices[~np.asarray(analysis['plot_data']['supported'])]
ref_labels = np.asarray(ref_fit['ray_labels'])
cur_labels = np.asarray(cur_fit['ray_labels'])
names = ['static_background', 'pedestrian_1', 'pedestrian_2']
counts = {}
for i in bad:
    key = names[ref_labels[i]] + ' -> ' + names[cur_labels[i]]
    counts[key] = counts.get(key, 0)+1
dt = analysis['failure_stamp']-analysis['reference_stamp']
delta = (np.asarray(cur_fit['values'])-ref_fit['values']).tolist()
assert np.allclose(np.abs(delta), [.5*dt, .6*dt], atol=.01), 'Does not match simulated actor motion'
result = dict(
    scope='Offline evaluation only. Truth pose and known synthetic boxes explain observations; '
          'they are NOT registration inputs or a production dynamic-object classifier.',
    evaluation_pose=pose.tolist(), labels=names,
    reference=ref_fit, current=cur_fit,
    pair_dt_s=dt, fitted_pedestrian_displacement_m=delta,
    expected_absolute_displacement_from_world_speed_m=[.5*dt, .6*dt],
    unmatched_points=len(bad), unmatched_attribution=counts,
    unsupported_beam_indices=bad.tolist(),
)
Path(sys.argv[4]).write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in
                 ('reference','current','unsupported_beam_indices')}, indent=2))
print('fit residuals', ref_fit['range_rmse_m'],cur_fit['range_rmse_m'])
