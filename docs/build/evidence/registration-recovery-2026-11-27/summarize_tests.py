"""Collect only this run's selected XMLs, rejecting stale test results."""
import json
import shutil
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

out=Path(sys.argv[1]);start=json.loads((out/'start.json').read_text())
source=Path('/root/mrider-team/ros2_ws/build-docker/mrider_navigation_sim/test_results/mrider_navigation_sim')
names=['camera_contract','segmentation','sensor_core','registration_recovery','moving_obstacles','motion_gate','free_evidence',
       'sensor_messages','safety','local_goal','observed_map','synchronization','path_feasibility',
       'path_messages','path_recovery','vehicle_overlay','large_office','indoor_geometry']
names+=['test_test_'+name+'_launch.py' for name in ['sensor_only','sensor_tf','sensor_dropout','large_office',
        'large_office_turns','large_office_dynamic','sensor_turns','navigation','blocked','recovery','faults']]
(out/'xml').mkdir(exist_ok=True);results=[]
for name in names:
    path=source/(name+('.gtest.xml' if name=='free_evidence' else '.xunit.xml'))
    assert path.stat().st_mtime>=start['epoch']-1.,('stale XML',str(path))
    cases=ET.parse(path).getroot().findall('.//testcase')
    assert cases,('no cases',name)
    failures=sum(c.find('failure') is not None for c in cases)
    errors=sum(c.find('error') is not None for c in cases)
    skips=sum(c.find('skipped') is not None for c in cases)
    results.append(dict(name=name,tests=len(cases),passed=len(cases)-failures-errors-skips,
        failures=failures,errors=errors,skipped=skips))
    shutil.copy2(path,out/'xml'/path.name)
totals={key:sum(r[key] for r in results) for key in ['tests','passed','failures','errors','skipped']}
summary=dict(date_kst='2026-11-27',environment=start['environment'],vehicle=1,
    scope='Selected navigation unit/ROS driving/fault/shutdown tests; not whole upstream workspace CI',
    suites=results,total=totals)
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(totals))
raise SystemExit(0 if totals['tests']==totals['passed'] else 1)
