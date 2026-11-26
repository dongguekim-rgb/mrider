"""Render recorded evaluation data; never connects to ROS or commands a car.

Usage: python3 plot_regression.py regression-03
Requires numpy, matplotlib and Noto Sans CJK (for Korean labels).
"""
import gzip
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np


directory = Path(__file__).parent / (sys.argv[1] if len(sys.argv) > 1 else 'regression-03')


def read(name):
    with gzip.open(directory / (name + '.json.gz'), 'rt') as source:
        return json.load(source)


plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
large = read('large_office_turns')
small = read('sensor_turns')
grid = read('map')
fig, axes = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
for ax, report in zip(axes, (large, small)):
    for path in report['planned_paths']:
        points = np.asarray(path)
        ax.plot(points[:, 0], points[:, 1], color='#ddac65', alpha=.5, linewidth=1)
    estimate = np.asarray(report['trajectory'])
    truth = np.asarray(report['evaluation_truth_trace'])
    if len(estimate):
        ax.plot(estimate[:, 0], estimate[:, 1], color='#126cbd', label='LiDAR 추정 궤적', linewidth=2)
    if len(truth):
        ax.plot(truth[:, 0], truth[:, 1], '--', color='#20252d', label='평가용 실제 궤적', linewidth=1.2)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(alpha=.2)
    ax.legend(loc='lower right')

raw = np.asarray(grid['data']).reshape(grid['height'], grid['width'])
display = np.where(raw < 0, 0, np.where(raw > 20, 2, 1))
ox, oy = grid['origin']
res = grid['resolution']
axes[0].imshow(display, origin='lower', zorder=-1,
               extent=(ox, ox + raw.shape[1]*res, oy, oy + raw.shape[0]*res),
               cmap=ListedColormap(['#e8ecf0', 'white', '#747b84']), vmin=0, vmax=2)
for i, leg in enumerate(large['legs'], 1):
    x, y = leg['goal'][:2]
    axes[0].plot(x, y, 'o', color='#ac2341')
    axes[0].annotate(f"목표 {i}: {leg['goal_error_m']:.3f} m", (x, y),
                     xytext=(5, 12), textcoords='offset points', fontsize=9)
axes[0].set(xlim=(-2, 45), ylim=(-4, 22),
            title=f"큰 맵 직진 → 좌회전 → 우회전: {large['result']}")
axes[1].plot(12, 0, 'o', color='#ac2341')
resume = f"{small['stop_to_resume_s']:.2f} s" if 'stop_to_resume_s' in small else '미완료'
arrival = f"{small['goal_reached_error_m']:.3f} m" if 'goal_reached_error_m' in small else '미도착'
axes[1].set(xlim=(-.5, 13), ylim=(-2, 3),
            title=f"장애물 우회·정지·재개: {small['result']}\n"
                  f"재개 {resume} / 도착 오차 {arrival}")
fig.suptitle('카메라·LiDAR 기반 2D 시뮬레이션 — GT는 평가에만 사용', fontsize=15)
fig.savefig(directory / 'trajectories.png', dpi=160)
plt.close(fig)
