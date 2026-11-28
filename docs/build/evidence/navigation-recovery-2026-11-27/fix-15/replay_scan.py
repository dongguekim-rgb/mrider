"""Read-only scan replay. Run with the recorded mrider_navigation_sim source on PYTHONPATH."""
import gzip
import json
import sys

import numpy as np
from mrider_navigation_sim.sensor_core import KeyframeOdometry

with gzip.open(sys.argv[1], 'rt') as stream:
    scans = json.load(stream)['/scan']
estimator = KeyframeOdometry()
failures, residuals = [], []
for scan in scans:
    ranges = np.asarray(scan['ranges'])
    angles = scan['angle_min'] + np.arange(len(ranges)) * scan['angle_increment']
    valid = (np.isfinite(ranges) & (ranges > scan['range_min']) &
             (ranges < scan['range_max'] - .05))
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]),
                              ranges[valid] * np.sin(angles[valid])))
    try:
        _, residual = estimator.update(points)
        residuals.append(residual)
    except ValueError as error:
        failures.append({'stamp': scan['stamp'], 'reason': str(error)})
print(json.dumps({'scans': len(scans), 'failures': failures,
                  'max_trimmed_rmse_m': max(residuals),
                  'last_relative_pose': estimator.pose.tolist()}, indent=2))
