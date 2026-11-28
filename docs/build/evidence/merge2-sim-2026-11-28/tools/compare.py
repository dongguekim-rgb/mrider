import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from render import load, draw_map, footprint, goals, view, NAVY, CYAN, ORANGE, RED, GREY


def plan_radius(pts):
    """Smallest radius of curvature along a planned path (circle through 3 points 0.5 m apart)."""
    if len(pts) < 20:
        return np.inf
    s = np.r_[0, np.cumsum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])))]
    best = np.inf
    for i in range(len(pts)):
        j = np.searchsorted(s, s[i] + 0.5)
        k = np.searchsorted(s, s[i] + 1.0)
        if k >= len(pts):
            break
        a, b, c = pts[i], pts[j], pts[k]
        A, B, C = np.linalg.norm(b - c), np.linalg.norm(a - c), np.linalg.norm(a - b)
        area = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2
        if area > 1e-6:
            best = min(best, A * B * C / (4 * area))
    return best


def driven_radius(d, wheelbase):
    """Tightest radius actually driven: wheelbase / tan(max mean steer) while moving."""
    S = np.array(d['steer'])
    inner = np.abs(S[:, 1:]).max(axis=1)
    # bicycle-equivalent angle from the two Ackermann angles
    cot = (1 / np.tan(np.clip(np.abs(S[:, 1]), 1e-4, None)) + 1 / np.tan(np.clip(np.abs(S[:, 2]), 1e-4, None))) / 2
    centre = np.arctan(1 / cot)
    return wheelbase / np.tan(centre.max()), float(np.degrees(inner.max())), float(np.degrees(centre.max()))


runs = [('estimates (vehicle:=0)', sys.argv[1], [[0.81, 0.28], [0.81, -0.28], [-0.18, -0.28], [-0.18, 0.28]], 0.63, 1.60),
        ('vehicle 1, measured (vehicle:=1)', sys.argv[2], [[0.76, 0.28], [0.76, -0.28], [-0.23, -0.28], [-0.23, 0.28]], 0.533, 2.21)]
fig, axes = plt.subplots(1, 2, figsize=(13, 5.4), dpi=160, sharey=True)
for ax, (label, path, fp, wb, planner_r) in zip(axes, runs):
    d = load(path)
    P = d['P']
    draw_map(ax, d)
    for _, pts in d['plan'][::5]:
        if len(pts):
            ax.plot(pts[:, 0], pts[:, 1], color=CYAN, lw=1.0, alpha=0.5, zorder=2)
    ax.plot(P[:, 1], P[:, 2], color=ORANGE, lw=2.4, zorder=3)
    for i in np.linspace(0, len(P) - 1, 8).astype(int):
        footprint(ax, P[i, 1:4], fp, fill=False, ec=NAVY, lw=1.0, zorder=4)
    for k, (gx, gy) in enumerate(goals(d), 1):
        ax.plot(gx, gy, marker='*', ms=16, color=RED, zorder=5, ls='none')
        ax.annotate(f'goal {k}', (gx, gy), textcoords='offset points', xytext=(8, 8), color=RED, fontweight='bold')
    ax.set_xlim(-1.5, 9.5); ax.set_ylim(-1.6, 6.0); ax.set_aspect('equal')
    ax.set_xlabel('map x [m]')
    r_drv, inner, centre = driven_radius(d, wb)
    length = float(np.sum(np.hypot(np.diff(P[:, 1]), np.diff(P[:, 2]))))
    ax.set_title(f'{label}\nNav2 minimum_turning_radius = {planner_r:.2f} m', color=NAVY, fontweight='bold', fontsize=11)
    print(f'{label}: planner R {planner_r} m | max steer {centre:.1f} deg -> tightest drivable R {r_drv:.2f} m | driven {length:.1f} m | plans {len(d["plan"])}')
axes[0].set_ylabel('map y [m]')
fig.suptitle('Same world, same two goals: only the geometry file differs', color=NAVY, fontweight='bold', fontsize=13)
fig.tight_layout()
fig.savefig(sys.argv[3])
