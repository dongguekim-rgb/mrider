import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from render import load, NAVY, BLUE, ORANGE, RED, GREY

d = load(sys.argv[1]); wb, r_meas, limit = 0.533, 2.10, 22.5
S = np.array(d['steer']); t = S[:, 0] - d['P'][0, 0]
keep = t >= 0
S, t = S[keep], t[keep]
a = np.clip(np.abs(S[:, 1:]), 1e-5, None)
bic = np.degrees(np.arctan(1 / ((1 / np.tan(a[:, 0]) + 1 / np.tan(a[:, 1])) / 2))) * np.sign(S[:, 1])
implied = np.degrees(np.arctan(wb / r_meas))
moving = np.abs(bic) > 0.5
over = np.abs(bic) > implied
dt = np.gradient(t)
print(f'implied steer angle from measured R: {implied:.2f} deg')
print(f'max bicycle steer in twin: {np.abs(bic).max():.2f} deg -> radius {wb / np.tan(np.radians(np.abs(bic).max())):.2f} m')
print(f'time steering: {dt[moving].sum():.1f} s, of which beyond {implied:.1f} deg: {dt[over].sum():.1f} s ({100 * dt[over].sum() / dt[moving].sum():.0f}%)')
fig, ax = plt.subplots(figsize=(9, 3.9), dpi=160)
ax.axhspan(implied, limit, color=RED, alpha=0.10, lw=0)
ax.axhspan(-limit, -implied, color=RED, alpha=0.10, lw=0)
ax.plot(t, bic, color=BLUE, lw=1.3, label='twin steering')
for s in (1, -1):
    ax.axhline(s * limit, color=RED, ls='--', lw=1.2, label=f'twin limit ±{limit}° (R = 1.29 m)' if s == 1 else None)
    ax.axhline(s * implied, color=ORANGE, ls='-', lw=1.4, label=f'±{implied:.1f}° from measured R = {r_meas:.2f} m' if s == 1 else None)
ax.set_xlabel('sim time [s]'); ax.set_ylabel('steer angle [deg]')
ax.set_ylim(-29, 36); ax.set_xlim(0, t[-1])
ax.set_title('Twin of vehicle 1 steers tighter than the measured car can', color=NAVY, fontweight='bold')
ax.legend(loc='upper center', ncol=3, fontsize=9, frameon=False, columnspacing=1.6)
ax.grid(alpha=0.25); fig.tight_layout(); fig.savefig(sys.argv[2])
