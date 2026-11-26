"""Render recorded diagnosis; no ROS connection or control outputs."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


root = Path(__file__).parent
d = json.loads((root/'registration.json').read_text())
plt.rcParams.update({'font.family':'Noto Sans CJK JP', 'axes.unicode_minus':False})
fig, axes = plt.subplots(1, 3, figsize=(16, 5.2), constrained_layout=True)
reference = np.asarray(d['plot_data']['reference'])
current = np.asarray(d['plot_data']['aligned_current'])
supported = np.asarray(d['plot_data']['supported'])
ax = axes[0]
ax.scatter(reference[:, 0], reference[:, 1], s=12, color='#89949e', label='1.70초 전 기준 scan')
ax.scatter(current[supported, 0], current[supported, 1], s=16, color='#23865b', label='현재 일치점')
ax.scatter(current[~supported, 0], current[~supported, 1], s=19, color='#c64335', label='현재 불일치점')
ax.scatter([0], [0], marker='>', s=90, color='#1c64a4', label='정지한 차량의 LiDAR')
ax.set(xlim=(-2, 4), ylim=(-.5, 2.5), xlabel='전방 x (m)', ylabel='좌측 y (m)',
       title='가까운 물체의 관측 변화\n회색: 이전 위치 / 빨강: 이동 후 위치')
ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(fontsize=8, loc='lower right')

ax = axes[1]
events = [e for e in d['estimator_timeline'] if 'support_fraction' in e]
ax.plot([e['stamp']-d['failure_stamp'] for e in events],
        [100*e['support_fraction'] for e in events], color='#1c64a4', linewidth=2)
ax.axhline(70, color='#c64335', linestyle='--', label='유효 정합 기준 70%')
ax.axvline(-d['reference_age_at_failure_s'], color='#89949e', linestyle=':', label='마지막 기준 scan 갱신')
stopped = next(e for e in d['safety_transitions'] if e['reason']=='camera_obstacle')
ax.axvline(stopped['camera_stamp']-d['failure_stamp'], color='#b5872a', linestyle=':', label='보행자 정지 판정')
ax.scatter([0], [100*d['exact_pair_replay']['support_fraction']], color='#c64335', zorder=5)
ax.set(xlim=(-7, .3), ylim=(60, 100), xlabel='첫 정합 실패 기준 시간 (s)', ylabel='대응점 비율 (%)',
       title='정지 후에도 보행자는 계속 이동\n147 / 217점 = 67.7%로 거부')
ax.grid(alpha=.2); ax.legend(fontsize=8, loc='upper right')

ax = axes[2]
values = [100*d['exact_pair_replay']['support_fraction'],
          100*d['fresh_previous_scan_control']['support_fraction']]
ax.barh([1, 0], values, color=['#c64335', '#23865b'], height=.5)
ax.axvline(70, color='#59636b', linestyle='--')
for y, v in zip([1, 0], values):
    ax.text(v+1, y, f'{v:.1f}%', va='center')
ax.set(yticks=[1, 0], yticklabels=['실패 당시 기준 scan', '직전 scan 대조'], xlim=(0, 105),
       xlabel='같은 현재 scan의 대응점 비율 (%)',
       title='GUI 없는 오프라인 재생\n다른 검사 기준은 동일')
ax.text(1, -.7, '직전 scan 통과는 원인 분리 결과이며\n장거리 주행 수정 검증이 아님', fontsize=9, color='#48535b')
fig.suptitle('단일 기준 scan의 동적 관측 변화 취약성 — 저장된 센서 데이터 분석', fontsize=16)
fig.savefig(root/'diagnosis.png', dpi=160)
plt.close(fig)
