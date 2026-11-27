"""Summarize one complete regression and the separate live RViz validation.

Run only after both report directories have been archived. Earlier failed
runs remain separate and are never replaced in this aggregate.
"""
import gzip
import json
from pathlib import Path

root = Path(__file__).resolve().parent


def read(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as stream:
        return json.load(stream)


regression = root / 'regression-03'
live = root / 'live-rviz'
tests = read(regression / 'summary.json')
assert tests['total']['tests'] == tests['total']['passed'], tests['total']
assert read(regression / 'source-sha256.json') == read(live / 'source-sha256.json')
reports = {}
for name in ('sensor_only', 'large_office', 'large_office_turns',
             'large_office_dynamic', 'sensor_turns', 'sensor_dropout'):
    report = read(regression / (name + '.json.gz'))
    assert report['result'] == 'PASS', (name, report.get('error'))
    fields = ('result', 'goal_reached_error_m', 'legs', 'automatic_resumes',
              'collision', 'min_clearance_m', 'max_candidate_steering_rad',
              'estimator_counts', 'elapsed_s', 'stop_to_resume_s',
              'steering_min_rad', 'steering_max_rad', 'camera_dropout_latched',
              'stop_detection_s', 'action_terminal_s')
    reports[name] = {k: report[k] for k in fields if k in report}
gui = read(live / 'live_large_office_dynamic.json.gz')
assert gui['result'] == 'PASS' and not gui['collision']
assert len(gui['legs']) == 3 and gui['automatic_resumes']
reports['live_rviz'] = {k: gui[k] for k in fields if k in gui}
gui_environment = read(live / 'environment.json.gz')
cleanup = read(live / 'cleanup.json.gz')
assert gui_environment['rviz_pid_before'] == gui_environment['rviz_pid_after']
assert not gui_environment['rviz_restarted'] and cleanup['rviz_alive']
assert not cleanup['remaining']
summary = dict(date_kst='2026-11-27', environment='ros2_humble:/root/mrider-team',
               vehicle=1, source_manifest='regression-03/source-sha256.json',
               regression=tests['total'], suites=len(tests['suites']), reports=reports,
               live_scope=read(live / 'scope.json'),
               remaining_validation=['Load-dependent startup scan gaps',
                   'Prolonged parked dynamic-scene registration',
                   'Background runner lifecycle (manual cleanup was required)',
                   'Gazebo physics and physical camera/LiDAR/Teensy integration'],
               scope='Synthetic camera/LiDAR, real SLAM/Nav2, 2D kinematic sensor simulation. '
                     'Ground truth is evaluation-only. Not Gazebo or physical-vehicle acceptance.')
(root / 'final-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(dict(tests=summary['regression'],
                     gui_errors_m=[x['goal_error_m'] for x in gui['legs']],
                     gui_resumes=len(gui['automatic_resumes']))))
