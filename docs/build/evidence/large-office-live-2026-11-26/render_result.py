"""Render this recorded experiment; never used as navigation input."""
import bisect
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Rectangle
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/mrider_navigation_sim'))
from mrider_navigation_sim.large_office import walls_and_furniture

font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family'] = font.get_name()
plt.rcParams['axes.unicode_minus'] = False
report = json.loads((HERE / 'drive.json').read_text())
grid = json.loads((HERE / 'map.json').read_text())
truth = np.array(report['evaluation_truth_trace'])
transforms = report['evaluation_tf_trace']
map_tf = sorted([t for t in transforms if t[1:3] == ['map', 'odom']])
times = [t[0] for t in map_tf]
estimated = []
for t in transforms:
    if t[1:3] != ['odom', 'base_link']:
        continue
    index = bisect.bisect_right(times, t[0]) - 1
    if index < 0:
        continue
    _, _, _, x, y, angle = map_tf[index]
    c, s = math.cos(angle), math.sin(angle)
    estimated.append([x + c * t[3] - s * t[4], y + s * t[3] + c * t[4]])
estimated = np.array(estimated)

fig, axes = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
for a, b, c, d in walls_and_furniture():
    axes[0].add_patch(Rectangle((a, b), c-a, d-b, color='#596575'))
axes[0].plot(truth[:, 1], truth[:, 2], color='#d64b35', lw=2.3, label='실제 궤적 — 평가용 GT')
axes[0].set_title('60×40 m 정적 환경 구조와 실제 주행\n설계도·GT는 평가에만 사용')

values = np.array(grid['data']).reshape(grid['height'], grid['width'])
display = np.where(values < 0, 0, np.where(values < 50, 1, 2))
x, y = grid['origin']; resolution = grid['resolution']
axes[1].imshow(display, origin='lower', interpolation='nearest',
               extent=[x, x + grid['width']*resolution, y, y + grid['height']*resolution],
               cmap=ListedColormap(['#cbd0d5', '#ffffff', '#394552']), vmin=0, vmax=2)
axes[1].plot(estimated[:, 0], estimated[:, 1], color='#156fa7', lw=2.0,
             label='추정 궤적 — LiDAR odometry + map TF')
axes[1].set_facecolor('#cbd0d5')
axes[1].set_title('센서로 만든 SLAM 지도\n회색: 미확인·지도 밖 / 흰색: 자유 / 검정: 점유')
for ax in axes:
    ax.plot(0, 0, 'o', color='#177448', label='시작')
    ax.plot(24, 0, '*', ms=13, color='#177448', label='도착: (24, 0), 오차 0.146 m')
    ax.plot(24, 16, 'x', ms=10, mew=2, color='#d64b35', label='요청 후 실패: (24, 16)')
    ax.set(xlim=(-5, 57), ylim=(-5, 37), xlabel='x [m]', ylabel='y [m]')
    ax.set_aspect('equal'); ax.grid(alpha=.18); ax.legend(loc='upper right', fontsize=8)
fig.suptitle('큰 맵 실험 · 2026-11-26 | 24 m 구간 통과 / 다음 90° 회전은 조향 한계로 정지', fontsize=14)
fig.savefig(HERE / 'result.png', dpi=150)
print(HERE / 'result.png')
