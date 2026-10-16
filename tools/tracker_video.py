"""Renders what the drone's tracker "sees" during a run, as an mp4 to edit next to the
camera videos (docker/film.sh, docker/replay.sh).

The drone does not track the truck with its camera. It gets the bed position over the
simulated link (target_tracker: noise, latency, dropouts), filters it, and flies to the
estimate. This video shows exactly that, from the run's CSV log (logs/run_*.csv):

  left   top-down view (North up) around the drone: the real truck and bed, the link
         measurements, the filtered estimate the controller flies to, and the drone
  right  sim time, state machine, errors, and the last 30 s of tracking error and height

    python3 tools/tracker_video.py                      # newest logs/run_*.csv
    python3 tools/tracker_video.py logs/run_20261016_101415.csv --fps 20 --out tracker.mp4

Needs matplotlib and ffmpeg (or `python3 -m pip install --user matplotlib imageio-ffmpeg`).
The clock in the corner is sim time, the same time base as the camera videos.
"""
import argparse
import csv
import glob
import math
import os
import shutil
import sys

BED_OFFSET = (-1.15, 0.0)          # falcon.yaml bed_offset (forward, left)
TRUCK_LEN, TRUCK_W = 5.2, 1.9      # truck body outline, generate_hive_base.py
BED_LEN, BED_W = 2.5, 2.0
HISTORY = 30.0                     # seconds shown in the time plots
STATE_COLORS = {'IDLE': '#888888', 'TAKEOFF': '#8a6d3b', 'APPROACH': '#d58512', 'TRACK': '#2c7fb8',
                'DESCEND': '#6a3d9a', 'TOUCHDOWN': '#e31a1c', 'LANDED': '#33a02c', 'ABORT': '#b10026'}


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def load(path):
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            row = {k: (v if k in ('state', 'request') else num(v)) for k, v in r.items()}
            if math.isnan(row['t']) or math.isnan(row['truck_x']):
                continue
            rows.append(row)
    return rows


def to_en(x_ned, y_ned):
    """PX4 local NED (spawn at the world origin, as in every start script) -> plot East, North."""
    return y_ned, x_ned


def truck_shape(row):
    """Truck outline and bed outline corners in East/North, from the truck's ENU pose."""
    x, y, yaw = row['truck_x'], row['truck_y'], row['truck_yaw']
    c, s = math.cos(yaw), math.sin(yaw)

    def box(cx, cy, length, width):
        pts = [(cx + dx * length / 2, cy + dy * width / 2) for dx, dy in ((1, 1), (-1, 1), (-1, -1), (1, -1), (1, 1))]
        return [(x + c * px - s * py, y + s * px + c * py) for px, py in pts]

    return box(0.1, 0, TRUCK_LEN, TRUCK_W), box(BED_OFFSET[0], 0, BED_LEN, BED_W)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('csv', nargs='?', help='run log (default: newest logs/run_*.csv)')
    ap.add_argument('--out', help='output mp4 (default: videos/tracker_<run>.mp4)')
    ap.add_argument('--fps', type=float, default=20.0, help='video frame rate (the log is 20 Hz)')
    ap.add_argument('--res', default='1280x720', help='video size, e.g. 1920x1080')
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.animation import FFMpegWriter
    except ImportError:
        sys.exit('matplotlib is missing: python3 -m pip install --user matplotlib imageio-ffmpeg')
    if not shutil.which('ffmpeg'):
        try:
            import imageio_ffmpeg
            matplotlib.rcParams['animation.ffmpeg_path'] = imageio_ffmpeg.get_ffmpeg_exe()
        except ImportError:
            sys.exit('ffmpeg is missing: python3 -m pip install --user imageio-ffmpeg')

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = args.csv or max(glob.glob(os.path.join(root, 'logs', 'run_*.csv')), key=os.path.getmtime, default=None)
    if not path:
        sys.exit('No logs/run_*.csv found.')
    rows = load(path)
    if len(rows) < 2:
        sys.exit(f'{path} has no usable rows.')
    out = args.out or os.path.join(root, 'videos', 'tracker_' + os.path.basename(path).replace('.csv', '.mp4'))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    w, h = (int(v) for v in args.res.split('x'))

    # Resample the log on a fixed sim-time grid, so the video plays at real (sim) speed.
    t0, t1 = rows[0]['t'], rows[-1]['t']
    frames, i = [], 0
    for k in range(int((t1 - t0) * args.fps) + 1):
        t = t0 + k / args.fps
        while i + 1 < len(rows) and rows[i + 1]['t'] <= t:
            i += 1
        frames.append(i)

    plt.rcParams.update({'font.size': 13 * h / 720, 'axes.titlesize': 14 * h / 720})
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100)
    ax = fig.add_axes([0.05, 0.17, 0.50, 0.76])
    hud = fig.add_axes([0.60, 0.55, 0.38, 0.38]); hud.axis('off')
    ax_e = fig.add_axes([0.62, 0.30, 0.35, 0.18])
    ax_h = fig.add_axes([0.62, 0.07, 0.35, 0.18])

    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_xlabel('East [m]'); ax.set_ylabel('North [m]')
    ax.set_title("What the tracker sees (top-down, around the drone)")
    truck_ln, = ax.plot([], [], color='#7f2704', lw=2, label='truck (true)')
    bed_ln, = ax.plot([], [], color='#33a02c', lw=2, label='bed (true)')
    meas_sc, = ax.plot([], [], 'o', color='#fd8d3c', ms=4, alpha=0.6, label='link measurements (last 1 s)')
    est_pt, = ax.plot([], [], 'x', color='#2c7fb8', ms=14, mew=3, label='filtered estimate (aim point)')
    est_v, = ax.plot([], [], color='#2c7fb8', lw=1.5)
    drone_tr, = ax.plot([], [], color='black', lw=1, alpha=0.4)
    drone_pt, = ax.plot([], [], '^', color='black', ms=11, label='drone')
    err_ln, = ax.plot([], [], '--', color='#e31a1c', lw=1.2, label='tracking error')
    fig.legend(handles=[truck_ln, bed_ln, meas_sc, est_pt, drone_pt, err_ln], loc='lower left',
               bbox_to_anchor=(0.03, 0.0), ncol=3, fontsize='small', frameon=False)

    for a, label, ylim in ((ax_e, 'tracking error [m]', None), (ax_h, 'height above bed [m]', None)):
        a.grid(alpha=0.3); a.set_ylabel(label, fontsize='small')
    ax_h.set_xlabel('sim time [s]', fontsize='small')
    e_ln, = ax_e.plot([], [], color='#e31a1c')
    h_ln, = ax_h.plot([], [], color='#6a3d9a')
    hud_txt = hud.text(0, 1, '', va='top', family='monospace', fontsize='large')
    state_txt = hud.text(0, 0.05, '', va='bottom', fontsize=26 * h / 720, weight='bold')

    ts = [r['t'] for r in rows]
    exy = [r['e_xy'] for r in rows]
    hh = [r['h'] for r in rows]

    writer = FFMpegWriter(fps=args.fps, bitrate=6000, codec='libx264', extra_args=['-pix_fmt', 'yuv420p'])
    print(f'Rendering {len(frames)} frames ({t1 - t0:.0f} s of sim time) from {os.path.basename(path)}...')
    with writer.saving(fig, out, dpi=100):
        for n, i in enumerate(frames):
            r = rows[i]
            t = t0 + n / args.fps
            truck, bed = truck_shape(r)
            truck_ln.set_data(*zip(*truck)); bed_ln.set_data(*zip(*bed))
            recent = [rows[j] for j in range(max(0, i - 20), i + 1)]
            meas_sc.set_data([to_en(q['meas_x'], q['meas_y'])[0] for q in recent],
                             [to_en(q['meas_x'], q['meas_y'])[1] for q in recent])
            ex, en = to_en(r['target_x'], r['target_y'])
            vx, vn = to_en(r['target_vx'], r['target_vy'])
            est_pt.set_data([ex], [en]); est_v.set_data([ex, ex + vx], [en, en + vn])
            trail = [to_en(q['drone_x'], q['drone_y']) for q in rows[max(0, i - 200):i + 1]]
            drone_tr.set_data([p[0] for p in trail], [p[1] for p in trail])
            dx, dn = to_en(r['drone_x'], r['drone_y'])
            have_drone = not math.isnan(dx)
            drone_pt.set_data([dx] if have_drone else [], [dn] if have_drone else [])
            err_ln.set_data([dx, ex] if have_drone else [], [dn, en] if have_drone else [])

            # Keep the drone and the bed in view, at least 30 m across.
            cx, cn = ((dx + ex) / 2, (dn + en) / 2) if have_drone else (ex, en)
            half = max(15.0, 0.65 * math.hypot(dx - ex, dn - en) if have_drone else 15.0)
            ax.set_xlim(cx - half * 1.0, cx + half * 1.0); ax.set_ylim(cn - half, cn + half)

            state = r['state'] or '-'
            fmt = lambda v, u='': '   -' if math.isnan(v) else f'{v:6.2f}{u}'
            speed = math.hypot(r['drone_vx'], r['drone_vy'])
            hud_txt.set_text(
                f'sim time    {t:8.2f} s\n'
                f'error xy    {fmt(r["e_xy"], " m")}\n'
                f'error vel   {fmt(r["e_v"], " m/s")}\n'
                f'height      {fmt(r["h"], " m")}\n'
                f'drone speed {fmt(speed, " m/s")}\n'
                f'truck speed {fmt(r["truck_v"], " m/s")}\n'
                f'request     {r["request"] or "-"}\n'
                f'landing ok  {"yes" if r["landing_ok"] == 1 else "no"}'
                f'   contact {"yes" if r["contact"] == 1 else "no"}')
            state_txt.set_text(state); state_txt.set_color(STATE_COLORS.get(state, 'black'))

            lo = max(0, next(k for k in range(i + 1) if ts[k] >= t - HISTORY))
            e_ln.set_data(ts[lo:i + 1], exy[lo:i + 1]); h_ln.set_data(ts[lo:i + 1], hh[lo:i + 1])
            for a, ys in ((ax_e, exy[lo:i + 1]), (ax_h, hh[lo:i + 1])):
                a.set_xlim(t - HISTORY, t + 0.5)
                good = [v for v in ys if not math.isnan(v)]
                if good:
                    a.set_ylim(min(0.0, min(good)) - 0.5, max(good) * 1.1 + 0.5)
            writer.grab_frame()
            if n % 200 == 0:
                print(f'  {n}/{len(frames)}', flush=True)
    print(f'Wrote {out}')


if __name__ == '__main__':
    main()
