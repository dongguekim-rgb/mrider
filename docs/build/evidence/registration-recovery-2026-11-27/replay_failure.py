"""Replay the saved laser-frame observations; no ROS, commands or GT input.

Run in the built Humble workspace:
python3 replay_failure.py <parked_dynamic_scans.json.gz> <output.json>
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
from mrider_navigation_sim.sensor_core import KeyframeOdometry

parser=argparse.ArgumentParser()
parser.add_argument('fixture',type=Path)
parser.add_argument('output',type=Path)
args=parser.parse_args()
with gzip.open(args.fixture,'rt') as stream:fixture=json.load(stream)
estimator=KeyframeOdometry();results=[]
for scan in fixture['scans']:
    ranges=np.asarray(scan['ranges'])
    angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    valid=np.isfinite(ranges)&(ranges>scan['range_min'])&(ranges<scan['range_max']-.05)
    observed=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])))
    pose,rmse=estimator.update(observed,scan['stamp'])
    results.append(dict(stamp=scan['stamp'],pose=pose.tolist(),rmse=rmse,diagnostics=dict(estimator.diagnostics)))
report=dict(fixture_sha256=hashlib.sha256(args.fixture.read_bytes()).hexdigest(),
    original_first_rejected_stamp=fixture['first_rejected_stamp'],scan_count=len(results),
    accepted_count=len(results),
    max_relative_translation_m=max(float(np.linalg.norm(r['pose'][:2])) for r in results),
    max_relative_yaw_rad=max(abs(r['pose'][2]) for r in results),
    recovery_count=sum(r['diagnostics']['registration_mode']=='recent_consensus' for r in results),results=results)
args.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='results'}))
