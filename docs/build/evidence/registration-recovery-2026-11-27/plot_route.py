"""Plot recorded trajectories for review; never publishes driving inputs.

Usage: plot_route.py REPORT.json[.gz] MAP.json[.gz] OUTPUT.png
"""
import gzip
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.font_manager import FontProperties
import numpy as np


def read(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as stream:
        return json.load(stream)


report, grid = read(sys.argv[1]), read(sys.argv[2])
font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family'] = font.get_name()
plt.rcParams['axes.unicode_minus'] = False
fig, ax = plt.subplots(figsize=(12, 7), constrained_layout=True)
cells = np.asarray(grid['data']).reshape(grid['height'], grid['width'])
display = np.where(cells < 0, 0, np.where(cells > 50, 2, 1))
ox, oy = grid['origin'][:2]
resolution = grid['resolution']
ax.imshow(display, origin='lower', interpolation='nearest',
          extent=(ox, ox + grid['width']*resolution, oy, oy + grid['height']*resolution),
          cmap=ListedColormap(['#dce1e5', '#ffffff', '#3e4854']), vmin=0, vmax=2)
estimated = np.asarray(report['trajectory'])
truth = np.asarray(report['evaluation_truth_trace'])
ax.plot(truth[:, 0], truth[:, 1], color='#17212b', linewidth=1.2,
        linestyle='--', label='평가용 실제 궤적 (주행 입력 아님)')
ax.plot(estimated[:, 0], estimated[:, 1], color='#067d92', linewidth=2,
        label='LiDAR 추정 궤적')
for index, leg in enumerate(report['legs'], 1):
    gx, gy, yaw = leg['goal']
    ax.scatter(gx, gy, s=110, marker='*', color='#d12252', zorder=5)
    ax.annotate(f"목표 {index}: {leg['goal_error_m']*100:.1f} cm", (gx, gy),
                xytext=(10, -23 if index == 1 else 12), textcoords='offset points',
                fontsize=10, weight='bold', color='#902045')
for index, stop in enumerate(report['automatic_resumes']):
    ax.scatter(*stop['stop_pose'], s=55, marker='o', color='#e48b12', zorder=6,
               label='장애물 정지 → 자동 재개' if index == 0 else None)
ax.scatter(0, 0, s=55, marker='s', color='#067d92')
ax.set(xlabel='x [m]', ylabel='y [m]', aspect='equal')
ax.set_title(f"큰 실내 맵: 직진 → 좌회전 → 우회전 · {report['result']}\n"
             f"충돌 {int(report['collision'])}회 · 최소 여유 {report['min_clearance_m']:.3f} m · "
             f"자동 재개 {len(report['automatic_resumes'])}회", loc='left', fontsize=15)
ax.legend(loc='upper left', framealpha=.95, fontsize=9)
ax.grid(alpha=.16)
fig.savefig(Path(sys.argv[3]), dpi=170)
plt.close(fig)
