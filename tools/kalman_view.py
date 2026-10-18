"""Renders "what the drone knows about the bed" for two runs side by side (e.g. Kalman filter
on and off), synced on sim time, as a 1920x1080 video for a presentation.

Per run:
  top     the bed from above in the truck's frame, zoomed in: the link measurements of the
          last second (behind the bed: latency), the aim point the controller flies to with
          its trail (jitter), and the drone
  middle  the bed height: raw measurements, the aim point and the true height
  bottom  the drone's height above the bed with the descent gate's window

    .venv/bin/python tools/kalman_view.py "Kalman filter=logs/run_20261018_120347.csv" \\
        "No filter (extrapolate)=logs/run_20261018_124456.csv" --start 33 --end 82 --out videos/kalman_view.mp4

Needs Pillow and ffmpeg.
"""
import argparse
import bisect
import math
import os
import statistics
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from overlay_video import (BED, BED_LEN, BED_W, FILT, GATE_H, GATE_OFF, GATE_ON, H_TRACK, PANEL, RAW,  # noqa: E402
                           STATE_COLORS, add_gate, fonts, load_rows, ok)

W, H = 1920, 1080
BG = (22, 26, 33)
VIEW = 2.0          # metres from the bed centre to the edge of the bed view
TRAIL = 1.0         # seconds of measurements and aim point trail
HISTORY = 10.0      # seconds in the plots


class Run:
    def __init__(self, label, path):
        self.label = label
        self.rows = add_gate(load_rows(path))
        landed = next((r for r in self.rows if r['state'] == 'LANDED'), None)
        if landed:
            self.rows = [r for r in self.rows if r['t'] <= landed['t']]
        self.ts = [r['t'] for r in self.rows]
        # The road is flat, so the true bed height is constant: the median of the (unbiased) measurements.
        self.bed_z = statistics.median(r['meas_z'] for r in self.rows if ok(r['meas_z']))

    def at(self, t):
        i = bisect.bisect_right(self.ts, t) - 1
        return i, (self.rows[i] if i >= 0 else None)

    def frame(self, r, x_ned, y_ned):
        """A point (PX4 local NED) relative to the true bed in the truck frame: (forward, left)."""
        c, s = math.cos(r['truck_yaw']), math.sin(r['truck_yaw'])
        de, dn = y_ned - r['bed_e'], x_ned - r['bed_n']
        return de * c + dn * s, -de * s + dn * c


def bed_view(d, run, i, r, f, x, y, size):
    d.rounded_rectangle((x, y, x + size, y + size), radius=12, fill=PANEL)
    d.text((x + 16, y + 12), 'bed from above (truck frame)', font=f['small'], fill=(200, 205, 215))
    cx, cy = x + size / 2, y + size / 2 + 14
    scale = (size / 2 - 30) / VIEW

    def px(fwd, left):
        return cx - max(-VIEW, min(VIEW, left)) * scale, cy - max(-VIEW, min(VIEW, fwd)) * scale

    for m in range(-int(VIEW), int(VIEW) + 1):
        d.line((cx + m * scale, cy - VIEW * scale, cx + m * scale, cy + VIEW * scale), fill=(255, 255, 255, 16))
        d.line((cx - VIEW * scale, cy + m * scale, cx + VIEW * scale, cy + m * scale), fill=(255, 255, 255, 16))
    hl, hw = BED_LEN / 2 * scale, BED_W / 2 * scale
    d.rectangle((cx - hw, cy - hl, cx + hw, cy + hl), outline=BED, width=4)
    d.polygon([(cx, cy - hl - 24), (cx - 12, cy - hl - 6), (cx + 12, cy - hl - 6)], fill=BED)
    d.text((cx, cy - hl - 28), 'driving direction', font=f['small'], fill=(150, 200, 160), anchor='md')
    if r is None:
        return
    lo = bisect.bisect_left(run.ts, r['t'] - TRAIL)
    seen = set()
    for q in run.rows[lo:i + 1]:
        if ok(q['meas_x']) and (q['meas_x'], q['meas_y']) not in seen:
            seen.add((q['meas_x'], q['meas_y']))
            mx, my = px(*run.frame(q, q['meas_x'], q['meas_y']))
            d.ellipse((mx - 6, my - 6, mx + 6, my + 6), fill=RAW + (210,))
    trail = [px(*run.frame(q, q['target_x'], q['target_y'])) for q in run.rows[lo:i + 1] if ok(q['target_x'])]
    if len(trail) > 1:
        d.line(trail, fill=FILT + (170,), width=3)
    if trail:
        ax, ay = trail[-1]
        d.line((ax - 14, ay - 14, ax + 14, ay + 14), fill=FILT, width=5)
        d.line((ax - 14, ay + 14, ax + 14, ay - 14), fill=FILT, width=5)
    if ok(r['drone_x']):
        dx, dy = px(*run.frame(r, r['drone_x'], r['drone_y']))
        d.ellipse((dx - 13, dy - 13, dx + 13, dy + 13), outline=(255, 255, 255), width=4)
    d.text((x + 16, y + size - 34), f'{2 * VIEW:g} m across, 1 m grid', font=f['small'], fill=(150, 155, 165))


def plot(d, x, y, w, h, t, f, title, series, y_lo, y_hi, band=None, ref=None, legend=()):
    d.rounded_rectangle((x, y, x + w, y + h), radius=12, fill=PANEL)
    d.text((x + 16, y + 10), title, font=f['small'], fill=(200, 205, 215))
    lx = x + w - 16
    for label, color in reversed(legend):
        d.text((lx, y + 10), label, font=f['small'], fill=color, anchor='ra')
        lx -= d.textlength(label, font=f['small']) + 22
    gx0, gx1, gy0, gy1 = x + 60, x + w - 16, y + 44, y + h - 16

    def sx(tt):
        return gx0 + (tt - (t - HISTORY)) / HISTORY * (gx1 - gx0)

    def sy(v):
        return gy1 - (min(max(v, y_lo), y_hi) - y_lo) / (y_hi - y_lo) * (gy1 - gy0)

    for k in range(3):
        v = y_lo + (y_hi - y_lo) * k / 2
        d.line((gx0, sy(v), gx1, sy(v)), fill=(255, 255, 255, 30))
        d.text((gx0 - 10, sy(v)), f'{v:.1f}', font=f['small'], fill=(150, 155, 165), anchor='rm')
    if band:
        d.rectangle((gx0, sy(band[1]), gx1, sy(band[0])), fill=GATE_ON + (55,))
    if ref is not None:
        d.line((gx0, sy(ref), gx1, sy(ref)), fill=BED, width=3)
    for pts_rows, key_fn, color, dots in series:
        pts = [(sx(tt), sy(v)) for tt, v in ((q['t'], key_fn(q)) for q in pts_rows) if ok(v)]
        if dots:
            for px_, py_ in pts[::2]:
                d.ellipse((px_ - 3, py_ - 3, px_ + 3, py_ + 3), fill=color)
        elif len(pts) > 1:
            d.line(pts, fill=color, width=4)


def height_color(r):
    """Green inside the gate's height window (or once past the gate), red outside it while tracking."""
    if r is None:
        return (200, 205, 215)
    if r['g_passed'] or r['g_h']:
        return GATE_ON
    return GATE_OFF if r['state'] == 'TRACK' else (200, 205, 215)


def column(d, run, t, f, x0, w):
    i, r = run.at(t)
    d.text((x0 + w / 2, 128), run.label, font=f['banner'], fill=(235, 240, 250), anchor='ma')
    state = (r['state'] if r else '') or '-'
    d.text((x0 + w / 2, 176), state, font=f['state'], fill=STATE_COLORS.get(state, (220, 225, 235)), anchor='ma')
    size = 520
    bx = x0 + 30
    bed_view(d, run, i, r, f, bx, 230, size)
    # Numbers next to the bed view.
    nx, ny = bx + size + 30, 250
    rows = [('link measurement', r['err_raw'] if r else math.nan, RAW, 'off the true bed'),
            ('aim point', r['err_filt'] if r else math.nan, FILT, 'off the true bed'),
            ('drone height', r['h'] if r else math.nan, height_color(r), 'above bed')]
    for n, (label, v, col, sub) in enumerate(rows):
        yy = ny + n * 165
        d.text((nx, yy), label, font=f['title'], fill=(200, 205, 215))
        d.text((nx, yy + 30), '-' if math.isnan(v) else f'{v:.2f} m', font=f['big'], fill=col)
        d.text((nx, yy + 100), sub, font=f['small'], fill=(150, 155, 165))
    lo = bisect.bisect_left(run.ts, t - HISTORY)
    seg = run.rows[lo:i + 1]
    pw = w - 60
    plot(d, x0 + 30, 770, pw, 140, t, f, 'bed height [m]',
         [(seg, lambda q: -q['meas_z'], RAW, True), (seg, lambda q: -q['target_z'], FILT, False)],
         -run.bed_z - 0.5, -run.bed_z + 0.5, ref=-run.bed_z,
         legend=(('measured', RAW), ('aim point', FILT), ('true', BED)))
    plot(d, x0 + 30, 922, pw, 140, t, f, 'drone height above bed [m]',
         [(seg, lambda q: q['h'], (190, 140, 255), False)], 0.0, 5.0,
         band=(H_TRACK - GATE_H, H_TRACK + GATE_H), legend=(('gate window 4 ± 0.3 m', GATE_ON),))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs=2, help='LABEL=path/to/run.csv, left then right')
    ap.add_argument('--start', type=float, default=33.0, help='sim time to start at')
    ap.add_argument('--end', type=float, default=82.0, help='sim time to end at')
    ap.add_argument('--title', default='What the drone knows about the bed')
    ap.add_argument('--subtitle', default='same radio link for both: 0.1 m noise, 0.1 s latency, 5 % dropouts, '
                                          'same random seed')
    ap.add_argument('--fps', type=float, default=25.0)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        sys.exit('Pillow is missing: python3 -m venv .venv && .venv/bin/pip install pillow')
    runs = []
    for spec in args.runs:
        label, _, path = spec.partition('=')
        runs.append(Run(label, path))
    f = fonts(1.6)
    f['big'] = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf', 58)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    frames = int((args.end - args.start) * args.fps)
    print(f'Rendering {frames} frames (sim time {args.start:g} to {args.end:g} s) to {args.out}')
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                            '-framerate', f'{args.fps:g}', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                            '-crf', '18', args.out], stdin=subprocess.PIPE)
    for k in range(frames):
        t = args.start + k / args.fps
        img = Image.new('RGB', (W, H), BG)
        d = ImageDraw.Draw(img, 'RGBA')
        d.text((40, 26), args.title, font=f['banner'], fill=(235, 240, 250))
        d.text((40, 76), args.subtitle, font=f['small'], fill=(170, 175, 185))
        d.text((W - 40, 34), f'sim time {t:6.1f} s', font=f['mono'], fill=(200, 205, 215), anchor='ra')
        d.line((W / 2, 120, W / 2, H - 16), fill=(60, 66, 78), width=2)
        for n, run in enumerate(runs):
            column(d, run, t, f, n * W // 2, W // 2)
        enc.stdin.write(img.tobytes())
        if k % 250 == 0:
            print(f'  {k}/{frames}', flush=True)
    enc.stdin.close()
    if enc.wait() != 0:
        sys.exit('ffmpeg failed while writing the video.')
    print(f'Wrote {args.out}')


if __name__ == '__main__':
    main()
