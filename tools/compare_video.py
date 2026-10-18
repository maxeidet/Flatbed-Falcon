"""Renders a video comparing several runs (e.g. one per controller) from their CSV logs.

Each run is aligned on the moment it starts tracking the truck (state TRACK), so the
controllers are compared on the same task even if takeoff and approach took different
times:

  left    the bed from above in the truck's frame (forward up): each drone with a trail,
          so a controller that lags behind the bed is easy to see
  right   the true horizontal error drone -> bed and the height above the bed over time,
          and per run its state, current error and touchdown error

    .venv/bin/python tools/compare_video.py "PID=logs/run_a.csv" "LQR=logs/run_b.csv" "LQI=logs/run_c.csv"
    .venv/bin/python tools/compare_video.py ... --title "Same truck, noisy link + Kalman" --out videos/compare.mp4

Needs Pillow and ffmpeg (python3 -m venv .venv && .venv/bin/pip install pillow).
"""
import argparse
import bisect
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from overlay_video import BED_LEN, BED_W, H_TRACK, PANEL, STATE_COLORS, fonts, load_rows, ok  # noqa: E402

W, H = 1280, 720
COLORS = [(80, 170, 255), (255, 150, 60), (90, 210, 110), (230, 90, 200), (240, 220, 80)]
BG = (22, 26, 33)
TRAIL = 3.0        # seconds of drone trail in the bed view
VIEW_RANGE = 3.0   # metres from the bed centre to the edge of the bed view
AFTER_LAST = 15.0  # seconds to keep rendering after the last landing (runs that never land are cut there)


class Run:
    def __init__(self, label, path, color):
        self.label, self.color = label, color
        rows = load_rows(path)
        start = next((r['t'] for r in rows if r['state'] == 'TRACK'), None)
        if start is None:
            sys.exit(f'{path} never reaches TRACK.')
        self.rows = [dict(r, tr=r['t'] - start) for r in rows if r['t'] >= start]
        landed = next((r for r in self.rows if r['state'] == 'LANDED'), None)
        self.landed = (landed['tr'], landed['err_drone']) if landed else None
        if landed:
            # After touchdown the drone is disarmed and its logged position drifts: freeze the
            # run at the landing so the plots and the bed view show where it landed.
            self.rows = [r for r in self.rows if r['tr'] <= landed['tr']]
        self.ts = [r['tr'] for r in self.rows]

    def at(self, t):
        i = bisect.bisect_right(self.ts, t) - 1
        return i, (self.rows[i] if i >= 0 else None)

    def rel(self, r):
        """Drone relative to the true bed in the truck frame: (forward, left) in metres."""
        c, s = math.cos(r['truck_yaw']), math.sin(r['truck_yaw'])
        de, dn = r['drone_y'] - r['bed_e'], r['drone_x'] - r['bed_n']
        return de * c + dn * s, -de * s + dn * c


def draw_frame(runs, t, t_end, f, title):
    from PIL import Image, ImageDraw
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img, 'RGBA')
    d.text((24, 16), title, font=f['title'], fill=(230, 235, 245))
    d.text((W - 24, 16), f't = {t:6.2f} s after TRACK', font=f['mono'], fill=(200, 205, 215), anchor='ra')
    bed_view(d, runs, t, f, 24, 52, 470)
    plot(d, runs, t, t_end, f, 520, 52, 736, 200, 'horizontal error drone -> true bed [m]', 'err_drone', 0, 2.0)
    plot(d, runs, t, t_end, f, 520, 266, 736, 200, 'height above bed [m]', 'h', 0, 8.0, ref=H_TRACK)
    scoreboard(d, runs, t, t_end, f, 520, 480, 736, 216)
    return img


def bed_view(d, runs, t, f, x, y, size):
    d.rounded_rectangle((x, y, x + size, y + size + 170), radius=10, fill=PANEL)
    d.text((x + 12, y + 8), 'bed from above (truck frame)', font=f['title'], fill=(220, 225, 235))
    cx, cy = x + size / 2, y + size / 2 + 20
    scale = (size / 2 - 20) / VIEW_RANGE
    for m in range(-int(VIEW_RANGE), int(VIEW_RANGE) + 1):
        d.line((cx + m * scale, cy - VIEW_RANGE * scale, cx + m * scale, cy + VIEW_RANGE * scale), fill=(255, 255, 255, 18))
        d.line((cx - VIEW_RANGE * scale, cy + m * scale, cx + VIEW_RANGE * scale, cy + m * scale), fill=(255, 255, 255, 18))
    hl, hw = BED_LEN / 2 * scale, BED_W / 2 * scale
    d.rectangle((cx - hw, cy - hl, cx + hw, cy + hl), outline=(90, 210, 110), width=3)
    d.polygon([(cx, cy - hl - 18), (cx - 9, cy - hl - 5), (cx + 9, cy - hl - 5)], fill=(90, 210, 110))
    d.text((cx, cy - hl - 22), 'driving direction', font=f['small'], fill=(150, 200, 160), anchor='md')

    def to_px(fwd, left):
        fwd = max(-VIEW_RANGE, min(VIEW_RANGE, fwd))
        left = max(-VIEW_RANGE, min(VIEW_RANGE, left))
        return cx - left * scale, cy - fwd * scale

    for run in runs:
        i, r = run.at(t)
        if r is None:
            continue
        lo = bisect.bisect_left(run.ts, t - TRAIL)
        pts = [to_px(*run.rel(q)) for q in run.rows[lo:i + 1] if ok(q['drone_x'])]
        if len(pts) > 1:
            d.line(pts, fill=run.color + (140,), width=2)
        if ok(r['drone_x']):
            px, py = to_px(*run.rel(r))
            d.ellipse((px - 9, py - 9, px + 9, py + 9), fill=run.color, outline=(255, 255, 255), width=2)
    ly = y + size + 40
    d.text((x + 12, ly), f'{2 * VIEW_RANGE:g} m across, 1 m grid. Dots: each drone, trail: last {TRAIL:g} s.',
           font=f['small'], fill=(160, 165, 175))
    for n, run in enumerate(runs):
        yy = ly + 28 + n * 26
        d.ellipse((x + 14, yy + 3, x + 28, yy + 17), fill=run.color)
        d.text((x + 38, yy), run.label, font=f['mono'], fill=(220, 225, 235))


def plot(d, runs, t, t_end, f, x, y, w, h, title, key, y_lo, y_hi, ref=None):
    d.rounded_rectangle((x, y, x + w, y + h), radius=10, fill=PANEL)
    d.text((x + 12, y + 8), title, font=f['small'], fill=(220, 225, 235))
    gx0, gx1, gy0, gy1 = x + 44, x + w - 14, y + 32, y + h - 26

    def sx(tt):
        return gx0 + tt / t_end * (gx1 - gx0)

    def sy(v):
        return gy1 - (min(max(v, y_lo), y_hi) - y_lo) / (y_hi - y_lo) * (gy1 - gy0)

    for k in range(5):
        v = y_lo + (y_hi - y_lo) * k / 4
        d.line((gx0, sy(v), gx1, sy(v)), fill=(255, 255, 255, 30))
        d.text((gx0 - 8, sy(v)), f'{v:g}', font=f['small'], fill=(160, 165, 175), anchor='rm')
    step = 10 if t_end <= 120 else 20
    for k in range(0, int(t_end) + 1, step):
        d.text((sx(k), gy1 + 6), f'{k}', font=f['small'], fill=(160, 165, 175), anchor='ma')
    if ref is not None:
        d.line((gx0, sy(ref), gx1, sy(ref)), fill=(90, 210, 110, 120), width=1)
    if key == 'err_drone':
        d.line((gx0, sy(0.3), gx1, sy(0.3)), fill=(255, 255, 255, 90), width=1)
        d.text((gx1 - 4, sy(0.3) - 4), 'gate 0.3 m', font=f['small'], fill=(200, 205, 215), anchor='rd')
    d.line((sx(t), gy0, sx(t), gy1), fill=(255, 255, 255, 60))
    for run in runs:
        i, _ = run.at(t)
        pts = [(sx(r['tr']), sy(r[key])) for r in run.rows[:i + 1:2]
               if ok(r[key]) and not (key == 'h' and r['state'] == 'LANDED')]
        if len(pts) > 1:
            d.line(pts, fill=run.color, width=2)


def scoreboard(d, runs, t, t_end, f, x, y, w, h):
    d.rounded_rectangle((x, y, x + w, y + h), radius=10, fill=PANEL)
    cols = [x + 14, x + 260, x + 420, x + 560]
    for c, head in zip(cols, ('controller', 'state', 'error now', 'touchdown')):
        d.text((c, y + 10), head, font=f['small'], fill=(160, 165, 175))
    for n, run in enumerate(runs):
        yy = y + 38 + n * 34
        _, r = run.at(t)
        d.ellipse((cols[0], yy + 4, cols[0] + 14, yy + 18), fill=run.color)
        d.text((cols[0] + 22, yy), run.label.split(' (')[0], font=f['mono'], fill=(230, 235, 245))
        state = r['state'] if r else '-'
        d.text((cols[1], yy), state, font=f['mono'], fill=STATE_COLORS.get(state, (220, 225, 235)))
        err = r['err_drone'] if r and state != 'LANDED' else math.nan
        d.text((cols[2], yy), '   -' if math.isnan(err) else f'{err:5.2f} m', font=f['mono'], fill=(230, 235, 245))
        if run.landed and t >= run.landed[0]:
            txt, col = f'{run.landed[1]:.2f} m at {run.landed[0]:.0f} s', (90, 210, 110)
        elif t >= min(run.ts[-1], t_end - 1.0):
            txt, col = 'not landed', (230, 90, 90)
        else:
            txt, col = '-', (160, 165, 175)
        d.text((cols[3], yy), txt, font=f['mono'], fill=col)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='+', help='LABEL=path/to/run.csv, one per run')
    ap.add_argument('--title', default='Controller comparison: same truck, same noisy link, Kalman filter on')
    ap.add_argument('--out', default=None, help='output mp4 (default: videos/compare_<labels>.mp4)')
    ap.add_argument('--fps', type=float, default=25.0)
    ap.add_argument('--max-time', type=float, help='seconds after TRACK to render (default: last landing + 15 s)')
    args = ap.parse_args()
    try:
        import PIL  # noqa: F401
    except ImportError:
        sys.exit('Pillow is missing: python3 -m venv .venv && .venv/bin/pip install pillow')

    runs = []
    for n, spec in enumerate(args.runs):
        label, _, path = spec.partition('=')
        if not path:
            sys.exit(f'Expected LABEL=path, got {spec!r}')
        runs.append(Run(label, path, COLORS[n % len(COLORS)]))
    landings = [run.landed[0] for run in runs if run.landed]
    t_end = args.max_time or (max(landings) + AFTER_LAST if landings else max(run.ts[-1] for run in runs))
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    labels = '_'.join(run.label.split(' (')[0].lower() for run in runs)
    out = args.out or os.path.join(root, 'videos', f'compare_{labels}.mp4')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    f = fonts(1.0)

    frames = int(t_end * args.fps) + 1
    print(f'Rendering {frames} frames ({t_end:.0f} s from TRACK) to {out}')
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                            '-framerate', f'{args.fps:g}', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                            '-crf', '18', out], stdin=subprocess.PIPE)
    for k in range(frames):
        enc.stdin.write(draw_frame(runs, k / args.fps, t_end, f, args.title).tobytes())
        if k % 500 == 0:
            print(f'  {k}/{frames}', flush=True)
    enc.stdin.close()
    if enc.wait() != 0:
        sys.exit('ffmpeg failed while writing the video.')
    print(f'Wrote {out}')


if __name__ == '__main__':
    main()
