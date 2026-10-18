"""Renders how several controllers follow the truck along the road, as a 1920x1080 video for a
presentation. The runs are aligned on the moment they start tracking (state TRACK).

  left    phase portrait: along-track error (drone ahead of the bed +, behind -) against its
          rate of change, with a trail. A controller without integral action settles at a
          point behind the bed; one with integral action pulls in towards the origin.
  right   the along-track error over time, and the truck's speed below it: the error spikes
          when the truck brakes or speeds up (only the centripetal acceleration is fed forward)

    .venv/bin/python tools/dynamics_video.py "PID=logs/run_A.csv" "LQR=logs/run_B.csv" "LQI=logs/run_C.csv" \\
        --out videos/clips/controller_dynamics.mp4

Needs Pillow and ffmpeg.
"""
import argparse
import bisect
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from overlay_video import GATE_ON, PANEL, STATE_COLORS, fonts, load_rows, ok  # noqa: E402

W, H = 1920, 1080
BG = (22, 26, 33)
COLORS = [(80, 170, 255), (255, 150, 60), (90, 210, 110), (230, 90, 200)]
E_RANGE = (-2.0, 1.5)     # along-track error shown [m]
R_RANGE = (-2.0, 2.0)     # its rate of change shown [m/s]
TRAIL = 4.0               # seconds of trail in the phase portrait
AFTER_LAST = 8.0          # seconds after the last landing


class Run:
    def __init__(self, label, path, color):
        self.label, self.color = label, color
        rows = load_rows(path)
        start = next(r['t'] for r in rows if r['state'] == 'TRACK')
        self.rows = []
        for r in rows:
            if r['t'] < start or not ok(r['drone_x']):
                continue
            c, s = math.cos(r['truck_yaw']), math.sin(r['truck_yaw'])
            de, dn = r['drone_y'] - r['bed_e'], r['drone_x'] - r['bed_n']
            # Drone velocity relative to the truck, along the truck's heading (ENU: east = vy, north = vx).
            rel_e, rel_n = r['drone_vy'] - r['truck_v'] * c, r['drone_vx'] - r['truck_v'] * s
            self.rows.append(dict(r, tr=r['t'] - start, e=de * c + dn * s, edot=rel_e * c + rel_n * s))
        # Smooth the rate over 0.45 s (9 log rows): the logged velocities are sampled at 20 Hz.
        raw = [r['edot'] for r in self.rows]
        for n, r in enumerate(self.rows):
            win = raw[max(0, n - 4):n + 5]
            r['edot'] = sum(win) / len(win)
        landed = next((r for r in self.rows if r['state'] == 'LANDED'), None)
        self.landed = landed['tr'] if landed else None
        if landed:
            # Freeze where touchdown starts: the impact jolts PX4's velocity estimate.
            touch = next(r['tr'] for r in self.rows if r['state'] in ('TOUCHDOWN', 'LANDED'))
            self.rows = [r for r in self.rows if r['tr'] <= touch]
        self.ts = [r['tr'] for r in self.rows]

    def at(self, t):
        i = bisect.bisect_right(self.ts, t) - 1
        return i, (self.rows[i] if i >= 0 else None)


def phase(d, runs, t, f, x, y, size):
    d.rounded_rectangle((x, y, x + size, y + size), radius=12, fill=PANEL)
    d.text((x + 20, y + 14), 'phase portrait (along the road)', font=f['title'], fill=(220, 225, 235))
    gx0, gx1, gy0, gy1 = x + 90, x + size - 30, y + 60, y + size - 110

    def sx(e):
        return gx0 + (min(max(e, E_RANGE[0]), E_RANGE[1]) - E_RANGE[0]) / (E_RANGE[1] - E_RANGE[0]) * (gx1 - gx0)

    def sy(v):
        return gy1 - (min(max(v, R_RANGE[0]), R_RANGE[1]) - R_RANGE[0]) / (R_RANGE[1] - R_RANGE[0]) * (gy1 - gy0)

    for e in (-2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5):
        d.line((sx(e), gy0, sx(e), gy1), fill=(255, 255, 255, 22 if e else 70))
        d.text((sx(e), gy1 + 10), f'{e:g}', font=f['small'], fill=(150, 155, 165), anchor='ma')
    for v in (-2, -1, 0, 1, 2):
        d.line((gx0, sy(v), gx1, sy(v)), fill=(255, 255, 255, 22 if v else 70))
        d.text((gx0 - 12, sy(v)), f'{v:g}', font=f['small'], fill=(150, 155, 165), anchor='rm')
    # The gate: within 0.3 m and 0.3 m/s (the gate measures the full horizontal error, this is one axis).
    d.rectangle((sx(-0.3), sy(0.3), sx(0.3), sy(-0.3)), fill=GATE_ON + (60,), outline=GATE_ON, width=2)
    d.text((sx(0.3) + 8, sy(0.3)), 'gate', font=f['small'], fill=GATE_ON)
    d.text(((gx0 + gx1) / 2, gy1 + 40), 'horizontal: error along the road [m]   (−: behind the bed, +: ahead)',
           font=f['small'], fill=(200, 205, 215), anchor='ma')
    d.text(((gx0 + gx1) / 2, gy1 + 68), 'vertical: its rate [m/s]   (up: catching up, down: falling back)',
           font=f['small'], fill=(200, 205, 215), anchor='ma')
    for run in runs:
        i, r = run.at(t)
        if r is None:
            continue
        lo = bisect.bisect_left(run.ts, t - TRAIL)
        pts = [(sx(q['e']), sy(q['edot'])) for q in run.rows[min(lo, i):i + 1]]
        if len(pts) > 1:
            d.line(pts, fill=run.color + (170,), width=3)
        px, py = pts[-1]  # a landed run stays where it landed
        d.ellipse((px - 11, py - 11, px + 11, py + 11), fill=run.color, outline=(255, 255, 255), width=3)


def timeline(d, runs, t, t_end, f, x, y, w, h, title, key, lo, hi, unit_ticks, band=None):
    d.rounded_rectangle((x, y, x + w, y + h), radius=12, fill=PANEL)
    d.text((x + 20, y + 14), title, font=f['title'], fill=(220, 225, 235))
    gx0, gx1, gy0, gy1 = x + 80, x + w - 24, y + 60, y + h - 50

    def sx(tt):
        return gx0 + tt / t_end * (gx1 - gx0)

    def sy(v):
        return gy1 - (min(max(v, lo), hi) - lo) / (hi - lo) * (gy1 - gy0)

    for v in unit_ticks:
        d.line((gx0, sy(v), gx1, sy(v)), fill=(255, 255, 255, 70 if v == 0 else 22))
        d.text((gx0 - 12, sy(v)), f'{v:g}', font=f['small'], fill=(150, 155, 165), anchor='rm')
    for k in range(0, int(t_end) + 1, 10):
        d.text((sx(k), gy1 + 10), f'{k}', font=f['small'], fill=(150, 155, 165), anchor='ma')
    d.text((x + w - 24, y + 18), 'seconds after TRACK', font=f['small'], fill=(150, 155, 165), anchor='ra')
    if band:
        d.rectangle((gx0, sy(band[1]), gx1, sy(band[0])), fill=GATE_ON + (50,))
    d.line((sx(t), gy0, sx(t), gy1), fill=(255, 255, 255, 80), width=2)
    for run in runs:
        i, _ = run.at(t)
        pts = [(sx(q['tr']), sy(q[key])) for q in run.rows[:i + 1:2] if ok(q[key])]
        if len(pts) > 1:
            d.line(pts, fill=run.color, width=3)


def legend(d, runs, t, f, x, y):
    for n, run in enumerate(runs):
        yy = y + n * 52
        _, r = run.at(t)
        d.ellipse((x, yy + 6, x + 26, yy + 32), fill=run.color)
        d.text((x + 40, yy), run.label, font=f['title'], fill=(230, 235, 245))
        if run.landed is not None and t >= run.landed:
            state, col = f'LANDED at {run.landed:.0f} s', STATE_COLORS['LANDED']
        else:
            state = (r['state'] if r else '') or '-'
            col = STATE_COLORS.get(state, (200, 205, 215))
        d.text((x + 640, yy), state, font=f['title'], fill=col)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='+', help='LABEL=path/to/run.csv')
    ap.add_argument('--title', default='How the controllers follow the truck')
    ap.add_argument('--fps', type=float, default=25.0)
    ap.add_argument('--max-time', type=float, help='seconds after TRACK to render (default: last landing + 8 s)')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        sys.exit('Pillow is missing: python3 -m venv .venv && .venv/bin/pip install pillow')
    runs = []
    for n, spec in enumerate(args.runs):
        label, _, path = spec.partition('=')
        # The same colour per controller as in the other videos.
        named = {'PID': COLORS[0], 'LQR': COLORS[1], 'LQI': COLORS[2]}
        runs.append(Run(label, path, named.get(label.split()[0], COLORS[n % len(COLORS)])))
    landings = [r.landed for r in runs if r.landed is not None]
    t_end = args.max_time or (max(landings) + AFTER_LAST if landings else max(r.ts[-1] for r in runs))
    f = fonts(1.6)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    frames = int(t_end * args.fps)
    print(f'Rendering {frames} frames ({t_end:.0f} s after TRACK) to {args.out}')
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                            '-framerate', f'{args.fps:g}', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                            '-crf', '18', args.out], stdin=subprocess.PIPE)
    for k in range(frames):
        t = k / args.fps
        img = Image.new('RGB', (W, H), BG)
        d = ImageDraw.Draw(img, 'RGBA')
        d.text((40, 26), args.title, font=f['banner'], fill=(235, 240, 250))
        d.text((40, 76), 'same truck, same radio link, Kalman filter on; aligned at the start of tracking',
               font=f['small'], fill=(170, 175, 185))
        d.text((W - 40, 34), f't = {t:5.1f} s after TRACK', font=f['mono'], fill=(200, 205, 215), anchor='ra')
        phase(d, runs, t, f, 40, 120, 800)
        timeline(d, runs, t, t_end, f, 880, 120, 1000, 400, 'error along the road [m]', 'e', -2.0, 2.0,
                 (-2, -1, 0, 1, 2), band=(-0.3, 0.3))
        timeline(d, runs, t, t_end, f, 880, 540, 1000, 300, 'truck speed [m/s]', 'truck_v', 0.0, 9.0, (0, 4, 8))
        legend(d, runs, t, f, 900, 870)
        enc.stdin.write(img.tobytes())
        if k % 250 == 0:
            print(f'  {k}/{frames}', flush=True)
    enc.stdin.close()
    if enc.wait() != 0:
        sys.exit('ffmpeg failed while writing the video.')
    print(f'Wrote {args.out}')


if __name__ == '__main__':
    main()
