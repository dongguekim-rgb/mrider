"""Combine the full regression and the test-loader-only safety rerun.

Run from any directory with Python 3. Original failures remain in regression-08.
This does not launch ROS or operate the vehicle.
"""
import json
from pathlib import Path
from xml.etree import ElementTree as ET


root = Path(__file__).parent


def read(run, name):
    return json.loads((root / run / name).read_text())


original = read('regression-08', 'summary.json')
rerun = read('fix-20', 'summary.json')
before = read('regression-08', 'source-sha256.json')
after = read('fix-20', 'source-sha256.json')
assert before.keys() == after.keys()
changed = [name for name in before if before[name] != after[name]]
assert changed == ['ros2_ws/src/mrider_navigation_sim/test/test_safety.py'], changed


def cases(run):
    tree = ET.parse(root / run / 'xml/safety.xunit.xml')
    return {(case.get('classname'), case.get('name')) for case in tree.findall('.//testcase')}


assert cases('regression-08') == cases('fix-20')
assert len(cases('fix-20')) == 46
assert len(rerun['suites']) == 1 and rerun['suites'][0]['name'] == 'safety'
suites = []
for suite in original['suites']:
    source = 'fix-20' if suite['name'] == 'safety' else 'regression-08'
    result = rerun['suites'][0] if source == 'fix-20' else suite
    suites.append({**result, 'source_run': source})

result = {
    'environment': original['environment'],
    'aggregation': 'Latest result per test, not one all-pass full run. '
                   'Full run: 211/212; test-loader-only change; safety rerun: 46/46.',
    'runtime_source_unchanged': True,
    'changed_files_between_runs': changed,
    'full_run': 'regression-08/summary.json',
    'targeted_rerun': 'fix-20/summary.json',
    'final_source_manifest': 'fix-20/source-sha256.json',
    'executions_in_these_two_runs': original['total']['tests'] + rerun['total']['tests'],
    'suites': suites,
    'total': {key: sum(s[key] for s in suites)
              for key in ('tests', 'passed', 'failures', 'errors', 'skipped')},
}
(root / 'final-summary.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result['total']))
