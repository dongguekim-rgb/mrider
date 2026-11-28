"""Turn a recorded twin run (recorder.py pickle) into figures and a video."""
import pickle
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

LASER_X = 0.24  # laser_link ahead of base_link, vehicle 1 URDF
NAVY, BLUE, CYAN, ORANGE, RED, GREY = '#002060', '#005195', '#01A0E9', '#EA7125', '#DC291E', '#616265'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                     'axes.edgecolor': GREY, 'axes.labelcolor': '#222222'})


def load(path):
    with open(path, 'rb') as f:
        d = pickle.load(f)
    d['P'] = np.array(d['pose'])
    return d


def draw_map(ax, d):
    t, res, ox, oy, w, h, data = d['map']
    g = data.reshape(h, w).astype(float)
    img = np.full((h, w, 3), 0.80)            # unknown
    img[g == 0] = 1.0                          # free
    img[g > 50] = (0.0, 0.125, 0.376)          # occupied (navy)
    ax.imshow(img, origin='lower', extent=(ox, ox + w * res, oy, oy + h * res), zorder=0)


def footprint(ax, pose, fp, **kw):
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    pts = [(x + c * px - s * py, y + s * px + c * py) for px, py in fp]
    ax.add_patch(Polygon(pts, closed=True, **kw))


def view(ax, d, pad=1.5):
    P = d['P']
    xs = [P[:, 1].min(), P[:, 1].max()] + [g[1] for g in d['goal']]
    ys = [P[:, 2].min(), P[:, 2].max()] + [g[2] for g in d['goal']]
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect('equal')
    ax.set_xlabel('map x [m]')
    ax.set_ylabel('map y [m]')


def goals(d):
    seen = []
    for _, x, y in d['goal']:
        if (x, y) not in seen:
            seen.append((x, y))
    return seen


def fig_trajectory(d, fp, title, out):
    fig, ax = plt.subplots(figsize=(9, 6.2), dpi=160)
    draw_map(ax, d)
    P = d['P']
    for _, pts in d['plan'][::6]:
        if len(pts):
            ax.plot(pts[:, 0], pts[:, 1], color=CYAN, lw=1.0, alpha=0.45, zorder=2)
    ax.plot([], [], color=CYAN, lw=1.5, label='Nav2 planned paths (/plan)')
    ax.plot(P[:, 1], P[:, 2], color=ORANGE, lw=2.4, zorder=3, label='driven path (map -> base_link)')
    for i in np.linspace(0, len(P) - 1, 9).astype(int):
        footprint(ax, P[i, 1:4], fp, fill=False, ec=NAVY, lw=1.0, zorder=4)
    for k, (gx, gy) in enumerate(goals(d), 1):
        ax.plot(gx, gy, marker='*', ms=18, color=RED, zorder=5, ls='none',
                label='goal' if k == 1 else None)
        ax.annotate(f'goal {k}', (gx, gy), textcoords='offset points', xytext=(10, 8),
                    color=RED, fontweight='bold')
    ax.plot(P[0, 1], P[0, 2], marker='o', ms=9, color=NAVY, zorder=5, ls='none', label='start')
    view(ax, d)
    ax.set_title(title, color=NAVY, fontweight='bold')
    ax.legend(loc='lower right', framealpha=0.95, fontsize=9)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def fig_steer(d, limit_deg, title, out):
    S = np.array(d['steer'])
    t = S[:, 0] - d['P'][0, 0]
    fig, ax = plt.subplots(figsize=(9, 3.6), dpi=160)
    ax.plot(t, np.degrees(S[:, 1]), color=BLUE, lw=1.2, label='front left steer joint')
    ax.plot(t, np.degrees(S[:, 2]), color=ORANGE, lw=1.2, label='front right steer joint')
    for s in (1, -1):
        ax.axhline(s * limit_deg, color=RED, ls='--', lw=1.2,
                   label=f'URDF limit ±{limit_deg}°' if s == 1 else None)
    ax.set_xlabel('sim time [s]')
    ax.set_ylabel('steer angle [deg]')
    ax.set_ylim(-limit_deg * 1.25, limit_deg * 1.25)
    ax.set_title(title, color=NAVY, fontweight='bold')
    ax.legend(loc='upper right', ncol=3, fontsize=9)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    return float(np.degrees(np.abs(S[:, 1:]).max()))


def video(d, fp, title, out, fps=10, speed=3.0):
    """Top-down replay next to the simulated camera, piped to ffmpeg."""
    P = d['P']
    t0, t1 = P[0, 0], P[-1, 0]
    times = np.arange(t0, t1, speed / fps)
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.05, 0.09, 0.56, 0.80])
    axc = fig.add_axes([0.64, 0.42, 0.34, 0.47])
    axt = fig.add_axes([0.64, 0.09, 0.34, 0.27])
    axt.axis('off')
    fig.suptitle(title, color=NAVY, fontweight='bold', fontsize=15, x=0.05, ha='left')
    # The recording holds only the FINAL /map, so drawing it under every frame
    # would show walls the LiDAR had not seen yet. Build the background from
    # the scans instead: a cell turns navy the first time a beam ends in it.
    view(ax, d)
    (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
    res = 0.05
    seen = np.zeros((int((y1 - y0) / res) + 1, int((x1 - x0) / res) + 1), dtype=bool)
    rgb = np.ones(seen.shape + (3,))
    bg = ax.imshow(rgb, origin='lower', extent=(x0, x1, y0, y1), zorder=0, interpolation='nearest')
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_title('navy = LiDAR returns accumulated so far (/scan), red = current scan',
                 fontsize=9, color=GREY)
    next_scan = 0
    for k, (gx, gy) in enumerate(goals(d), 1):
        ax.plot(gx, gy, marker='*', ms=18, color=RED, ls='none', zorder=5)
        ax.annotate(f'goal {k}', (gx, gy), textcoords='offset points', xytext=(10, 8),
                    color=RED, fontweight='bold')
    trail, = ax.plot([], [], color=ORANGE, lw=2.4, zorder=3)
    plan, = ax.plot([], [], color=CYAN, lw=2.0, zorder=2)
    scan = ax.scatter([], [], s=2, color=RED, zorder=3)
    body = Polygon(fp, closed=True, fc=NAVY, ec='white', lw=0.8, alpha=0.9, zorder=6)
    ax.add_patch(body)
    axc.set_xticks([])
    axc.set_yticks([])
    axc.set_title('simulated camera  /camera/image_raw', fontsize=10, color=GREY)
    im = axc.imshow(d['cam'][0][1])
    txt = axt.text(0, 1, '', va='top', fontsize=12, family='DejaVu Sans Mono', color='#222222')
    cam_t = np.array([c[0] for c in d['cam']])
    scan_t = np.array([s[0] for s in d['scan']])
    plan_t = np.array([p[0] for p in d['plan']]) if d['plan'] else np.array([])
    cmd = np.array(d['cmd']) if d['cmd'] else np.zeros((1, 3))
    steer = np.array(d['steer'])
    w, h = fig.canvas.get_width_height()
    ff = subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgba',
         '-s', f'{w}x{h}', '-r', str(fps), '-i', '-', '-pix_fmt', 'yuv420p',
         '-vcodec', 'libx264', '-crf', '23', out], stdin=subprocess.PIPE)
    for t in times:
        i = int(np.searchsorted(P[:, 0], t))
        i = min(i, len(P) - 1)
        x, y, yaw = P[i, 1:4]
        trail.set_data(P[:i + 1, 1], P[:i + 1, 2])
        c, s = np.cos(yaw), np.sin(yaw)
        body.set_xy([(x + c * px - s * py, y + s * px + c * py) for px, py in fp])
        while next_scan < len(scan_t) and scan_t[next_scan] <= t:
            _, qp, qa0, qda, qr = d['scan'][next_scan]
            good = np.isfinite(qr) & (qr > 0.05)
            qang = qp[2] + qa0 + qda * np.arange(len(qr))
            hx = qp[0] + LASER_X * np.cos(qp[2]) + qr[good] * np.cos(qang[good])
            hy = qp[1] + LASER_X * np.sin(qp[2]) + qr[good] * np.sin(qang[good])
            cx = ((hx - x0) / res).astype(int)
            cy = ((hy - y0) / res).astype(int)
            inside = (cx >= 0) & (cx < seen.shape[1]) & (cy >= 0) & (cy < seen.shape[0])
            seen[cy[inside], cx[inside]] = True
            next_scan += 1
        rgb[:] = 1.0
        rgb[seen] = (0.0, 0.125, 0.376)
        bg.set_data(rgb)
        j = min(int(np.searchsorted(scan_t, t)), len(scan_t) - 1)
        _, sp, a0, da, r = d['scan'][j]
        ok = np.isfinite(r) & (r > 0.05)
        ang = sp[2] + a0 + da * np.arange(len(r))
        lx, ly = sp[0] + LASER_X * np.cos(sp[2]), sp[1] + LASER_X * np.sin(sp[2])
        scan.set_offsets(np.c_[lx + r[ok] * np.cos(ang[ok]), ly + r[ok] * np.sin(ang[ok])])
        if len(plan_t) and t >= plan_t[0]:
            k = max(int(np.searchsorted(plan_t, t)) - 1, 0)
            pts = d['plan'][k][1]
            if len(pts) and t - plan_t[k] < 3.0:
                plan.set_data(pts[:, 0], pts[:, 1])
            else:
                plan.set_data([], [])
        im.set_data(d['cam'][min(int(np.searchsorted(cam_t, t)), len(cam_t) - 1)][1])
        ci = min(int(np.searchsorted(cmd[:, 0], t)), len(cmd) - 1)
        vx = cmd[ci, 1] if abs(cmd[ci, 0] - t) < 0.5 else 0.0
        si = min(int(np.searchsorted(steer[:, 0], t)), len(steer) - 1)
        txt.set_text(f'sim time   {t - t0:6.1f} s   (x{speed:g})\n'
                     f'pose       x {x:5.2f}  y {y:5.2f} m\n'
                     f'heading    {np.degrees(yaw):6.1f} deg\n'
                     f'/cmd_vel   {vx:5.2f} m/s\n'
                     f'steer L/R  {np.degrees(steer[si, 1]):5.1f} / {np.degrees(steer[si, 2]):5.1f} deg')
        fig.canvas.draw()
        ff.stdin.write(fig.canvas.buffer_rgba().tobytes())
    ff.stdin.close()
    ff.wait()
    plt.close(fig)


if __name__ == '__main__':
    pkl, vehicle, outdir, fp = sys.argv[1], sys.argv[2], sys.argv[3], eval(sys.argv[4])
    label = sys.argv[5]
    d = load(pkl)
    fig_trajectory(d, fp, f'Twin of {label}: SLAM map, Nav2 plans and driven path',
                   f'{outdir}/twin_v{vehicle}_trajectory.png')
    mx = fig_steer(d, 22.5, f'Twin of {label}: steering joints stay inside the URDF limit',
                   f'{outdir}/twin_v{vehicle}_steering.png')
    print('max |steer| deg', round(mx, 2))
    from PIL import Image
    cams = d['cam']
    Image.fromarray(cams[len(cams) // 3][1]).save(f'{outdir}/twin_v{vehicle}_camera.png')
    if len(sys.argv) > 6 and sys.argv[6] == 'video':
        video(d, fp, f'Twin of {label}  -  sim.launch.py vehicle:={vehicle} + nav2.launch.py vehicle:={vehicle}',
              f'{outdir}/twin_v{vehicle}_nav2.mp4')
