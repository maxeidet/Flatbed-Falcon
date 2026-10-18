"""Puts two replay recordings of different runs next to each other, synced on sim time, with
a shared panel underneath that compares them (e.g. one controller against another).

Each recording needs its Gazebo log name (for the run's CSV and the log's start time) and
the sim time at which the recording started (--offset, as for overlay_video.py):

    .venv/bin/python tools/side_by_side.py \\
        "LQI (kp 1.88 kd 2.01 ki 0.58)=LQI1.mp4@20261018-120332+3" \\
        "LQR (kp 1.07 kd 1.58)=LQR1.mp4@20261018-120619+3" --out videos/lqi_vs_lqr.mp4

  top      the two recordings, each with its controller, state, height and true error
  bottom   both runs' error to the true bed and height over the last 30 s, and the bed
           from above (truck frame) with both drones

Both runs start the same way (same truck, same spawn), so the same sim time shows the same
moment of the task. Needs Pillow and ffmpeg.
"""
import argparse
import bisect
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from overlay_video import (BED, BED_LEN, BED_W, GATE_H, GATE_IDLE, GATE_OFF, GATE_ON, GATE_TIME,  # noqa: E402
                           H_TRACK, PANEL, STATE_COLORS, add_gate, fmt, fonts, load_rows, log_t0, matching_csv,
                           ok, probe)

COLORS = [(90, 210, 110), (255, 150, 60)]
STRIP = 300        # height of the shared panel under the videos
HISTORY = 30.0     # seconds shown in the plots
VIEW_RANGE = 3.0   # metres from the bed centre to the edge of the bed view


class Side:
    def __init__(self, spec, root, color):
        self.label, _, rest = spec.partition('=')
        video, _, rest = rest.partition('@')
        log, _, offset = rest.partition('+')
        if not (video and log):
            sys.exit(f'Expected "LABEL=video.mp4@<gazebo log>+<offset>", got {spec!r}')
        self.video, self.color = video, color
        csv_path = matching_csv(root, log)
        if not csv_path:
            sys.exit(f'No logs/run_*.csv from after {log}.')
        self.rows = add_gate(load_rows(csv_path))
        self.ts = [r['t'] for r in self.rows]
        self.t0 = log_t0(os.path.join(root, 'logs', 'gz', log)) + float(offset or 0)
        self.w, self.h, self.fps, self.frames = probe(video)
        landed = next((r for r in self.rows if r['state'] == 'LANDED'), None)
        self.landed = (landed['t'], landed['err_drone']) if landed else None
        if landed:
            # After touchdown the drone is disarmed and its logged position drifts.
            self.rows = [r for r in self.rows if r['t'] <= landed['t']]
            self.ts = self.ts[:len(self.rows)]
        print(f'{self.label}: {video} ({self.w}x{self.h}, {self.frames} frames) + {os.path.basename(csv_path)}, '
              f'starts at sim time {self.t0:.2f} s')

    def at(self, t):
        i = bisect.bisect_right(self.ts, t) - 1
        return i, (self.rows[i] if i >= 0 else None)

    def rel(self, r):
        c, s = math.cos(r['truck_yaw']), math.sin(r['truck_yaw'])
        de, dn = r['drone_y'] - r['bed_e'], r['drone_x'] - r['bed_n']
        return de * c + dn * s, -de * s + dn * c


def side_hud(d, side, t, f, x0, w):
    _, r = side.at(t)
    d.rounded_rectangle((x0 + 12, 12, x0 + 380, 112), radius=8, fill=PANEL)
    d.ellipse((x0 + 24, 24, x0 + 40, 40), fill=side.color)
    d.text((x0 + 50, 20), side.label, font=f['title'], fill=(235, 240, 250))
    state = (r['state'] if r else '') or 'WAITING'
    if side.landed and t >= side.landed[0]:
        state = 'LANDED'
    d.text((x0 + 24, 46), state, font=f['state'], fill=STATE_COLORS.get(state, (230, 230, 230)))
    d.text((x0 + 24, 86), f'sim time {t:6.2f} s', font=f['mono'], fill=(200, 205, 215))
    d.rounded_rectangle((x0 + w - 300, 12, x0 + w - 12, 88), radius=8, fill=PANEL)
    landed = side.landed and t >= side.landed[0]
    h = math.nan if landed or not r else r['h']
    err = side.landed[1] if landed else (r['err_drone'] if r else math.nan)
    d.text((x0 + w - 288, 22), 'height above bed', font=f['mono'], fill=(200, 205, 215))
    d.text((x0 + w - 24, 22), fmt(h, 'm'), font=f['mono'], fill=(190, 140, 255), anchor='ra')
    d.text((x0 + w - 288, 50), 'touchdown error' if landed else 'drone -> bed', font=f['mono'], fill=(200, 205, 215))
    d.text((x0 + w - 24, 50), fmt(err, 'm'), font=f['mono'], fill=(90, 210, 110) if landed else (235, 240, 250),
           anchor='ra')
    gate_row(d, r, f, x0 + 12, 120)


def gate_row(d, r, f, x, y):
    """Compact descent gate: one lamp per condition and the hold time towards 1.5 s."""
    d.rounded_rectangle((x, y, x + 368, y + 62), radius=8, fill=PANEL)
    tracking = r is not None and r['state'] == 'TRACK'
    passed = r is not None and r['g_passed']
    d.text((x + 12, y + 6), 'descent gate: passed' if passed else 'descent gate', font=f['small'],
           fill=GATE_ON if passed else (200, 205, 215))
    if tracking and r['request'] == 'READY_TO_LAND':
        d.text((x + 356, y + 6), 'asked to land', font=f['small'], fill=GATE_ON, anchor='ra')
    for n, (label, key) in enumerate((('position', 'g_pos'), ('speed', 'g_vel'), ('height', 'g_h'))):
        col = GATE_ON if passed else ((GATE_ON if r[key] else GATE_OFF) if tracking else GATE_IDLE)
        lx = x + 12 + n * 92
        d.ellipse((lx, y + 31, lx + 14, y + 45), fill=col)
        d.text((lx + 20, y + 29), label, font=f['small'], fill=(220, 225, 235))
    held = GATE_TIME if passed else (r['g_held'] if tracking else 0.0)
    bx0, bx1 = x + 290, x + 356
    d.rounded_rectangle((bx0, y + 33, bx1, y + 43), radius=3, fill=(255, 255, 255, 30))
    if held > 0:
        d.rounded_rectangle((bx0, y + 33, bx0 + (bx1 - bx0) * min(1.0, held / GATE_TIME), y + 43), radius=3,
                            fill=GATE_ON)


def plot(d, sides, t, f, x, y, w, h, title, key, y_lo, y_hi, ref=None):
    d.rounded_rectangle((x, y, x + w, y + h), radius=10, fill=PANEL)
    d.text((x + 12, y + 8), title, font=f['small'], fill=(220, 225, 235))
    gx0, gx1, gy0, gy1 = x + 40, x + w - 12, y + 32, y + h - 24

    def sx(tt):
        return gx0 + (tt - (t - HISTORY)) / HISTORY * (gx1 - gx0)

    def sy(v):
        return gy1 - (min(max(v, y_lo), y_hi) - y_lo) / (y_hi - y_lo) * (gy1 - gy0)

    for k in range(5):
        v = y_lo + (y_hi - y_lo) * k / 4
        d.line((gx0, sy(v), gx1, sy(v)), fill=(255, 255, 255, 30))
        d.text((gx0 - 6, sy(v)), f'{v:g}', font=f['small'], fill=(160, 165, 175), anchor='rm')
    d.text((gx1, gy1 + 4), 'now', font=f['small'], fill=(160, 165, 175), anchor='ra')
    d.text((gx0, gy1 + 4), f'-{HISTORY:g} s', font=f['small'], fill=(160, 165, 175), anchor='la')
    if ref is not None:
        d.rectangle((gx0, sy(ref + GATE_H), gx1, sy(ref - GATE_H)), fill=GATE_ON + (50,))
        d.line((gx0, sy(ref), gx1, sy(ref)), fill=BED + (120,), width=1)
        d.text((gx1 - 4, sy(ref + GATE_H) - 3), 'gate 4 ± 0.3 m', font=f['small'], fill=(200, 205, 215), anchor='rd')
    if key == 'err_drone':
        d.rectangle((gx0, sy(0.3), gx1, sy(0)), fill=GATE_ON + (50,))
        d.line((gx0, sy(0.3), gx1, sy(0.3)), fill=(255, 255, 255, 90), width=1)
        d.text((gx1 - 4, sy(0.3) - 3), 'gate 0.3 m', font=f['small'], fill=(200, 205, 215), anchor='rd')
    for side in sides:
        i, _ = side.at(t)
        lo = bisect.bisect_left(side.ts, t - HISTORY)
        pts = [(sx(r['t']), sy(r[key])) for r in side.rows[lo:i + 1] if ok(r[key])]
        if len(pts) > 1:
            d.line(pts, fill=side.color, width=2)


def bed_view(d, sides, t, f, x, y, size):
    d.rounded_rectangle((x, y, x + size, y + size), radius=10, fill=PANEL)
    d.text((x + 12, y + 8), 'bed from above', font=f['small'], fill=(220, 225, 235))
    cx, cy = x + size / 2, y + size / 2 + 10
    scale = (size / 2 - 22) / VIEW_RANGE
    hl, hw = BED_LEN / 2 * scale, BED_W / 2 * scale
    d.rectangle((cx - hw, cy - hl, cx + hw, cy + hl), outline=BED, width=2)
    d.polygon([(cx, cy - hl - 14), (cx - 7, cy - hl - 4), (cx + 7, cy - hl - 4)], fill=BED)
    for side in sides:
        i, r = side.at(t)
        if r is None or not ok(r['drone_x']):
            continue
        lo = bisect.bisect_left(side.ts, t - 3.0)
        pts = []
        for q in side.rows[lo:i + 1]:
            if ok(q['drone_x']):
                fwd, left = side.rel(q)
                pts.append((cx - max(-VIEW_RANGE, min(VIEW_RANGE, left)) * scale,
                            cy - max(-VIEW_RANGE, min(VIEW_RANGE, fwd)) * scale))
        if len(pts) > 1:
            d.line(pts, fill=side.color + (140,), width=2)
        px, py = pts[-1] if pts else (cx, cy)
        d.ellipse((px - 7, py - 7, px + 7, py + 7), fill=side.color, outline=(255, 255, 255), width=2)
    d.text((x + 12, y + size - 22), f'{2 * VIEW_RANGE:g} m across, forward up', font=f['small'], fill=(160, 165, 175))


def reader(side):
    return subprocess.Popen(['ffmpeg', '-v', 'error', '-i', side.video, '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'],
                            stdout=subprocess.PIPE)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('sides', nargs=2, help='"LABEL=video.mp4@<gazebo log>+<offset>", left then right')
    ap.add_argument('--out', required=True, help='output mp4')
    args = ap.parse_args()
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        sys.exit('Pillow is missing: python3 -m venv .venv && .venv/bin/pip install pillow')

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sides = [Side(spec, root, COLORS[n]) for n, spec in enumerate(args.sides)]
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fps = sides[0].fps
    # Same height for both, so they sit next to each other; the shorter one is scaled.
    vh = max(s.h for s in sides) // 2 * 2
    widths = [round(s.w * vh / s.h) // 2 * 2 for s in sides]
    W, H = sum(widths), vh + STRIP
    t_start = max(s.t0 for s in sides)
    t_stop = min(s.t0 + s.frames / s.fps for s in sides)
    frames = int((t_stop - t_start) * fps)
    f = fonts(1.0)
    print(f'Rendering {frames} frames ({W}x{H}), sim time {t_start:.2f} to {t_stop:.2f} s')

    readers = [reader(s) for s in sides]
    skip = [round((t_start - s.t0) * s.fps) for s in sides]
    for rd, s, n in zip(readers, sides, skip):
        for _ in range(n):  # start both at the same sim time
            rd.stdout.read(s.w * s.h * 3)
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                            '-framerate', f'{fps:g}', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                            '-crf', '18', args.out], stdin=subprocess.PIPE)
    for k in range(frames):
        t = t_start + k / fps
        canvas = Image.new('RGB', (W, H), (22, 26, 33))
        x = 0
        imgs = []
        for rd, s, w in zip(readers, sides, widths):
            buf = rd.stdout.read(s.w * s.h * 3)
            if len(buf) < s.w * s.h * 3:
                break
            img = Image.frombytes('RGB', (s.w, s.h), buf)
            imgs.append(img if (s.w, s.h) == (w, vh) else img.resize((w, vh)))
        if len(imgs) < 2:
            frames = k
            break
        for img in imgs:
            canvas.paste(img, (x, 0))
            x += img.width
        d = ImageDraw.Draw(canvas, 'RGBA')
        d.line((widths[0], 0, widths[0], vh), fill=(22, 26, 33), width=4)
        x = 0
        for s, w in zip(sides, widths):
            side_hud(d, s, t, f, x, w)
            x += w
        y, gap, bed = vh + 12, 12, STRIP - 24
        pw = (W - 24 - bed - 2 * gap) // 2
        plot(d, sides, t, f, 12, y, pw, bed, 'horizontal error drone -> true bed [m]', 'err_drone', 0, 2.0)
        plot(d, sides, t, f, 12 + pw + gap, y, pw, bed, 'height above bed [m]', 'h', 0, 8.0, ref=H_TRACK)
        bed_view(d, sides, t, f, W - 12 - bed, y, bed)
        enc.stdin.write(canvas.tobytes())
        if k % 500 == 0:
            print(f'  {k}/{frames}', flush=True)
    for rd in readers:
        rd.kill()
    enc.stdin.close()
    if enc.wait() != 0:
        sys.exit('ffmpeg failed while writing the video.')
    print(f'Wrote {args.out} ({frames} frames)')


if __name__ == '__main__':
    main()
