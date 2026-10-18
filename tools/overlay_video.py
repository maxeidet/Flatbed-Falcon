"""Draws the run's measurements on top of a replay video (docker/replay.sh --auto).

The replay video and the run's CSV log (logs/run_*.csv) are both on sim time: frame i of
videos/<log>/<angle>.mp4 shows sim time t0 + i / fps, where t0 is the first message in
the Gazebo log (the same t0 docker/replay_frames.py seeks from). So every frame gets the
CSV row from the moment it shows:

  top left      state machine, sim time, landing handshake
  top right     tracking: height above the bed, drone -> aim point, speeds
  right         Kalman filter: raw link error and filtered error against the true bed
  bottom left   the last 20 s of those errors, and of the height above the bed
  bottom right  the bed seen from above in the truck's frame: link measurements (last 1 s),
                the filtered aim point and the drone
  centre        a banner with the touchdown error once the drone has landed

    .venv/bin/python tools/overlay_video.py                       # newest logs/gz/<log>, drone_chase
    .venv/bin/python tools/overlay_video.py 20261018-105132 -a overhead --offset 0.1
    .venv/bin/python tools/overlay_video.py 20261018-105132 --video my.mp4 --offset 8 --scale 0.6

The panels fit themselves to the video's resolution; --scale makes them smaller or larger.

Needs Pillow and ffmpeg (python3 -m venv .venv && .venv/bin/pip install pillow).
"""
import argparse
import bisect
import csv
import glob
import json
import math
import os
import sqlite3
import subprocess
import sys

BED_OFFSET = (-1.15, 0.0)     # falcon.yaml bed_offset (forward, left)
BED_LEN, BED_W = 2.5, 2.0     # generate_hive_base.py
H_TRACK = 4.0                 # falcon.yaml h_track
HISTORY = 20.0                # seconds shown in the plots
INSET_RANGE = 3.0             # metres from the bed centre to the inset's edge
TRAIL = 1.0                   # seconds of link measurements in the inset
STATE_COLORS = {'IDLE': (136, 136, 136), 'TAKEOFF': (190, 150, 80), 'APPROACH': (230, 150, 40),
                'TRACK': (60, 150, 220), 'DESCEND': (150, 100, 210), 'TOUCHDOWN': (230, 50, 50),
                'LANDED': (60, 190, 80), 'ABORT': (200, 0, 40)}
RAW, FILT, DRONE, BED, HEIGHT = (255, 150, 60), (80, 170, 255), (255, 255, 255), (90, 210, 110), (190, 140, 255)
PANEL = (15, 18, 24, 170)
FONT_DIR = '/usr/share/fonts/truetype/dejavu'


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def ok(*vals):
    return all(not math.isnan(v) for v in vals)


def load_rows(path):
    """CSV rows with the true bed (world ENU) and the errors the video shows."""
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            row = {k: (v if k in ('state', 'request') else num(v)) for k, v in r.items()}
            if math.isnan(row['t']):
                continue
            c, s = math.cos(row['truck_yaw']), math.sin(row['truck_yaw'])
            row['bed_e'] = row['truck_x'] + c * BED_OFFSET[0] - s * BED_OFFSET[1]
            row['bed_n'] = row['truck_y'] + s * BED_OFFSET[0] + c * BED_OFFSET[1]
            # PX4 local NED has the spawn at the world origin (every start script): east = y, north = x.
            row['err_raw'] = math.hypot(row['meas_y'] - row['bed_e'], row['meas_x'] - row['bed_n'])
            row['err_filt'] = math.hypot(row['target_y'] - row['bed_e'], row['target_x'] - row['bed_n'])
            row['err_drone'] = math.hypot(row['drone_y'] - row['bed_e'], row['drone_x'] - row['bed_n'])
            rows.append(row)
    return rows


def log_t0(log_dir):
    db = sqlite3.connect(f'file:{os.path.join(log_dir, "state.tlog")}?mode=ro', uri=True)
    t0 = db.execute('SELECT MIN(time_recv) FROM messages').fetchone()[0]
    db.close()
    return int(t0) * 1e-9


def matching_csv(root, log_name):
    """The run CSV started closest after the Gazebo log (logger starts a few seconds later)."""
    stamp = log_name.replace('-', '_')
    runs = sorted(glob.glob(os.path.join(root, 'logs', 'run_*.csv')))
    later = [p for p in runs if os.path.basename(p)[4:19] >= stamp]
    return later[0] if later else None


def probe(path):
    out = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-count_packets',
                          '-show_entries', 'stream=width,height,r_frame_rate,nb_read_packets',
                          '-of', 'json', path], capture_output=True, text=True, check=True).stdout
    s = json.loads(out)['streams'][0]
    a, b = s['r_frame_rate'].split('/')
    return int(s['width']), int(s['height']), float(a) / float(b), int(s.get('nb_read_packets', 0))


def fonts(scale):
    from PIL import ImageFont

    def load(name, size):
        try:
            return ImageFont.truetype(os.path.join(FONT_DIR, name), size)
        except OSError:
            return ImageFont.load_default(size)

    return {'mono': load('DejaVuSansMono.ttf', round(17 * scale)),
            'small': load('DejaVuSans.ttf', round(13 * scale)),
            'title': load('DejaVuSans-Bold.ttf', round(15 * scale)),
            'state': load('DejaVuSans-Bold.ttf', round(30 * scale)),
            'banner': load('DejaVuSans-Bold.ttf', round(24 * scale))}


def fmt(v, unit, digits=2):
    return '  -' if math.isnan(v) else f'{v:5.{digits}f} {unit}'


# The descent gate (landing_logic.py, LandingParams): in TRACK, all three held for GATE_TIME
# before the drone asks the truck for permission to land.
GATE_POS, GATE_VEL, GATE_H, GATE_TIME = 0.3, 0.3, 0.3, 1.5
GATE_ON, GATE_OFF, GATE_IDLE = (70, 200, 90), (220, 60, 60), (90, 95, 105)


def add_gate(rows):
    """Marks each row with the gate's three conditions and how long all three have held."""
    since = None
    for r in rows:
        r['g_pos'] = ok(r['e_xy']) and r['e_xy'] < GATE_POS
        r['g_vel'] = ok(r['e_v']) and r['e_v'] < GATE_VEL
        r['g_h'] = ok(r['h']) and abs(r['h'] - H_TRACK) < GATE_H
        tracking = r['state'] == 'TRACK'
        if tracking and r['g_pos'] and r['g_vel'] and r['g_h']:
            since = r['t'] if since is None else since
        else:
            since = None
        r['g_held'] = 0.0 if since is None else r['t'] - since
        r['g_passed'] = r['state'] in ('DESCEND', 'TOUCHDOWN', 'LANDED')
    return rows


def draw_gate(d, r, f, x, y, px, width=300):
    """The gate panel: a lamp per condition with its value and limit, and the hold time."""
    d.rounded_rectangle((x, y, x + px(width), y + px(150)), radius=px(8), fill=PANEL)
    tracking = r is not None and r['state'] == 'TRACK'
    passed = r is not None and r['g_passed']
    title = 'Descent gate: passed' if passed else 'Descent gate'
    d.text((x + px(10), y + px(7)), title, font=f['title'], fill=GATE_ON if passed else (220, 225, 235))
    if tracking and r['request'] == 'READY_TO_LAND':
        # Asked once already; the gate has to hold again when the truck gives LANDING_OK.
        note = 'asked, truck OK' if r['landing_ok'] == 1 else 'asked to land'
        d.text((x + px(width - 10), y + px(9)), note, font=f['small'], fill=GATE_ON, anchor='ra')
    items = [('position', 'g_pos', 'e_xy', 'm', f'< {GATE_POS:g}'),
             ('speed', 'g_vel', 'e_v', 'm/s', f'< {GATE_VEL:g}'),
             ('height', 'g_h', 'h', 'm', f'4 ± {GATE_H:g}')]
    for n, (label, key, val, unit, limit) in enumerate(items):
        yy = y + px(36 + n * 25)
        if passed:
            col = GATE_ON
        elif tracking:
            col = GATE_ON if r[key] else GATE_OFF
        else:
            col = GATE_IDLE
        rad = px(7)
        d.ellipse((x + px(14), yy + px(3), x + px(14) + 2 * rad, yy + px(3) + 2 * rad), fill=col)
        d.text((x + px(36), yy), label, font=f['mono'], fill=(220, 225, 235))
        v = r[val] if r is not None and not passed else math.nan
        d.text((x + px(132), yy), fmt(v, unit), font=f['mono'], fill=(220, 225, 235))
        d.text((x + px(width - 10), yy), limit, font=f['small'], fill=(160, 165, 175), anchor='ra')
    # Hold time towards 1.5 s.
    by = y + px(116)
    held = GATE_TIME if passed else (r['g_held'] if tracking else 0.0)
    frac = min(1.0, held / GATE_TIME)
    bx0, bx1 = x + px(14), x + px(width - 70)
    d.rounded_rectangle((bx0, by, bx1, by + px(14)), radius=px(4), fill=(255, 255, 255, 30))
    if frac > 0:
        d.rounded_rectangle((bx0, by, bx0 + (bx1 - bx0) * frac, by + px(14)), radius=px(4), fill=GATE_ON)
    d.text((x + px(width - 10), by - px(2)), f'{held:.1f}/{GATE_TIME:g} s', font=f['small'],
           fill=(200, 205, 215), anchor='ra')


class Overlay:
    def __init__(self, rows, w, h, scale=0.7):
        self.rows, self.w, self.h = add_gate(rows), w, h
        self.ts = [r['t'] for r in rows]
        # Sizes are in pixels of a 720 px high video. The panels side by side need about
        # 760 px of width, so a narrow window is limited by its width, any other by its height.
        self.k = min(w / 760, h / 720) * scale
        self.f = fonts(self.k)
        self.touchdown = next(((r['t'], r['err_drone']) for r in rows if r['state'] == 'LANDED'), None)

    def px(self, v):
        return round(v * self.k)

    def row_at(self, t):
        i = bisect.bisect_right(self.ts, t) - 1
        return (i, self.rows[i]) if i >= 0 and t - self.ts[i] < 0.5 else (i, None)

    def panel(self, d, x, y, w, h, title=None):
        d.rounded_rectangle((x, y, x + w, y + h), radius=self.px(8), fill=PANEL)
        if title:
            d.text((x + self.px(10), y + self.px(7)), title, font=self.f['title'], fill=(220, 225, 235))

    def lines(self, d, x, y, items):
        for n, (label, value, color) in enumerate(items):
            yy = y + n * self.px(23)
            d.text((x, yy), label, font=self.f['mono'], fill=(200, 205, 215))
            d.text((x + self.px(182), yy), value, font=self.f['mono'], fill=color)

    def draw(self, img, t):
        from PIL import Image, ImageDraw
        layer = Image.new('RGBA', img.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(layer)
        i, r = self.row_at(t)
        self.status(d, t, r)
        draw_gate(d, r, self.f, self.px(16), self.px(146), self.px)
        if r is not None:
            self.tracking(d, r)
            self.filter_panel(d, r)
            self.plots(d, i, t)
            self.inset(d, i, r)
        if self.touchdown and t >= self.touchdown[0]:
            self.banner(d)
        return Image.alpha_composite(img.convert('RGBA'), layer).convert('RGB')

    def status(self, d, t, r):
        x, y, p = self.px(16), self.px(16), self.px(12)
        self.panel(d, x, y, self.px(300), self.px(118))
        state = r['state'] if r and r['state'] else 'WAITING'
        d.text((x + p, y + self.px(8)), state, font=self.f['state'], fill=STATE_COLORS.get(state, (230, 230, 230)))
        d.text((x + p, y + self.px(50)), f'sim time {t:7.2f} s', font=self.f['mono'], fill=(220, 225, 235))
        if r:
            req = r['request'] or 'NONE'
            ok_txt = 'LANDING_OK' if r['landing_ok'] == 1 else 'no permission'
            d.text((x + p, y + self.px(74)), f'drone: {req}', font=self.f['small'], fill=(200, 205, 215))
            d.text((x + p, y + self.px(93)), f'truck: {ok_txt}', font=self.f['small'],
                   fill=BED if r['landing_ok'] == 1 else (200, 205, 215))

    def tracking(self, d, r):
        w = self.px(350)
        x, y = self.w - w - self.px(16), self.px(16)
        self.panel(d, x, y, w, self.px(132), 'Tracking')
        speed = math.hypot(r['drone_vx'], r['drone_vy'])
        # On the bed the height is meaningless (the target sits on the bed surface), so hide it.
        height = math.nan if r['state'] == 'LANDED' else r['h']
        h_col = BED if ok(height) and abs(r['h'] - H_TRACK) < 0.3 and r['state'] == 'TRACK' else HEIGHT
        self.lines(d, x + self.px(12), y + self.px(32), [
            ('height above bed', fmt(height, 'm'), h_col),
            ('drone -> aim', fmt(r['e_xy'], 'm'), DRONE),
            ('drone speed', fmt(speed, 'm/s'), DRONE),
            ('truck speed', fmt(r['truck_v'], 'm/s'), DRONE)])

    def filter_panel(self, d, r):
        w = self.px(350)
        x, y = self.w - w - self.px(16), self.px(160)
        self.panel(d, x, y, w, self.px(110), 'Kalman filter vs true bed')
        self.lines(d, x + self.px(12), y + self.px(32), [
            ('raw link error', fmt(r['err_raw'], 'm'), RAW),
            ('filtered error', fmt(r['err_filt'], 'm'), FILT),
            ('drone -> bed', fmt(r['err_drone'], 'm'), DRONE)])

    def plots(self, d, i, t):
        lo = bisect.bisect_left(self.ts, t - HISTORY)
        seg = self.rows[lo:i + 1]
        w, h, gap = self.px(420), self.px(110), self.px(12)
        x, y = self.px(16), self.h - 2 * h - gap - self.px(16)
        self.plot(d, x, y, w, h, t, seg, 'error [m]',
                  [('err_raw', RAW, 'link'), ('err_filt', FILT, 'filtered'), ('e_xy', DRONE, 'drone→aim')],
                  0.0, 3.0, band=(0.0, GATE_POS))
        self.plot(d, x, y + h + gap, w, h, t, seg, 'height above bed [m]',
                  [('h', HEIGHT, None)], 0.0, 8.0, ref=H_TRACK, band=(H_TRACK - GATE_H, H_TRACK + GATE_H))

    def plot(self, d, x, y, w, h, t, seg, title, series, y_lo, y_hi, ref=None, band=None):
        self.panel(d, x, y, w, h)
        d.text((x + self.px(10), y + self.px(5)), title, font=self.f['small'], fill=(220, 225, 235))
        lx = x + w - self.px(10)
        for key, color, label in reversed(series):
            if label:
                tw = d.textlength(label, font=self.f['small'])
                d.text((lx - tw, y + self.px(5)), label, font=self.f['small'], fill=color)
                lx -= tw + self.px(12)
        gx0, gx1 = x + self.px(34), x + w - self.px(10)
        gy0, gy1 = y + self.px(26), y + h - self.px(10)

        def sx(tt):
            return gx0 + (tt - (t - HISTORY)) / HISTORY * (gx1 - gx0)

        def sy(v):
            return gy1 - (min(max(v, y_lo), y_hi) - y_lo) / (y_hi - y_lo) * (gy1 - gy0)

        for v in (y_lo, (y_lo + y_hi) / 2, y_hi):
            d.line((gx0, sy(v), gx1, sy(v)), fill=(255, 255, 255, 40))
            d.text((x + self.px(6), sy(v) - self.px(8)), f'{v:g}', font=self.f['small'], fill=(160, 165, 175))
        if band is not None:  # the gate's window for this quantity
            d.rectangle((gx0, sy(band[1]), gx1, sy(band[0])), fill=GATE_ON + (55,))
        if ref is not None:
            d.line((gx0, sy(ref), gx1, sy(ref)), fill=BED + (160,), width=max(1, self.px(1)))
        for key, color, _ in series:
            pts = [(sx(r['t']), sy(r[key])) for r in seg
                   if ok(r[key]) and not (key == 'h' and r['state'] == 'LANDED')]
            if len(pts) > 1:
                d.line(pts, fill=color, width=max(2, self.px(2)))

    def inset(self, d, i, r):
        size = self.px(230)
        x, y = self.w - size - self.px(16), self.h - size - self.px(16)
        self.panel(d, x, y, size, size, 'bed from above')
        cx, cy, scale = x + size / 2, y + size / 2 + self.px(10), (size / 2 - self.px(14)) / INSET_RANGE
        c, s = math.cos(r['truck_yaw']), math.sin(r['truck_yaw'])

        def to_px(east, north):
            # Truck frame, forward up: the bed stays put and everything else moves around it.
            de, dn = east - r['bed_e'], north - r['bed_n']
            fwd, left = de * c + dn * s, -de * s + dn * c
            return cx - left * scale, cy - fwd * scale

        hl, hw = BED_LEN / 2 * scale, BED_W / 2 * scale
        d.rectangle((cx - hw, cy - hl, cx + hw, cy + hl), outline=BED, width=max(2, self.px(2)))
        d.polygon([(cx, cy - hl - self.px(14)), (cx - self.px(7), cy - hl - self.px(4)),
                   (cx + self.px(7), cy - hl - self.px(4))], fill=BED)
        d.text((x + self.px(10), y + size - self.px(22)), f'{2 * INSET_RANGE:g} m across, forward up',
               font=self.f['small'], fill=(160, 165, 175))

        lo = bisect.bisect_left(self.ts, r['t'] - TRAIL)
        seen = set()
        for q in self.rows[lo:i + 1]:
            if not ok(q['meas_x']) or (q['meas_x'], q['meas_y']) in seen:
                continue
            seen.add((q['meas_x'], q['meas_y']))
            # Each measurement is drawn where it was relative to the bed at its own time.
            de, dn = q['meas_y'] - q['bed_e'], q['meas_x'] - q['bed_n']
            px, py = to_px(r['bed_e'] + de, r['bed_n'] + dn)
            rad = self.px(3)
            d.ellipse((px - rad, py - rad, px + rad, py + rad), fill=RAW + (200,))
        if ok(r['target_x']):
            px, py, a = *to_px(r['target_y'], r['target_x']), self.px(8)
            d.line((px - a, py - a, px + a, py + a), fill=FILT, width=self.px(3))
            d.line((px - a, py + a, px + a, py - a), fill=FILT, width=self.px(3))
        if ok(r['drone_x']):
            px, py, a = *to_px(r['drone_y'], r['drone_x']), self.px(7)
            d.ellipse((px - a, py - a, px + a, py + a), outline=DRONE, width=self.px(2))

    def banner(self, d):
        text = f'LANDED  {self.touchdown[1]:.2f} m from the bed centre'
        tw = d.textlength(text, font=self.f['banner'])
        # Top centre, between the status and tracking panels, clear of the drone; below the
        # panels when the window is too narrow for that.
        x, y = (self.w - tw) / 2, self.px(28)
        if x - self.px(16) < self.px(332) or x + tw + self.px(16) > self.w - self.px(366):
            y = self.px(290)
        d.rounded_rectangle((x - self.px(16), y - self.px(12), x + tw + self.px(16), y + self.px(40)),
                            radius=self.px(10), fill=(20, 60, 30, 200))
        d.text((x, y), text, font=self.f['banner'], fill=(235, 255, 235))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('log', nargs='?', help='Gazebo log name under logs/gz/ (default: newest)')
    ap.add_argument('-a', '--angle', default='drone_chase', help='which replay video (default drone_chase)')
    ap.add_argument('--video', help='input mp4 (default: videos/<log>/<angle>.mp4)')
    ap.add_argument('--csv', help='run log (default: the run_*.csv started right after the Gazebo log)')
    ap.add_argument('--out', help='output mp4 (default: videos/<log>/<angle>_overlay.mp4)')
    ap.add_argument('--offset', type=float, default=0.0, help='shift the data by this many seconds')
    ap.add_argument('--scale', type=float, default=0.7, help='panel size relative to the video (0.7)')
    args = ap.parse_args()

    try:
        from PIL import Image
    except ImportError:
        sys.exit('Pillow is missing: python3 -m venv .venv && .venv/bin/pip install pillow')

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    logs = sorted(glob.glob(os.path.join(root, 'logs', 'gz', '*', 'state.tlog')))
    log_dir = os.path.join(root, 'logs', 'gz', args.log) if args.log else (os.path.dirname(logs[-1]) if logs else None)
    if not log_dir or not os.path.exists(os.path.join(log_dir, 'state.tlog')):
        sys.exit('No Gazebo log found in logs/gz/. Record a run with start-stellar.sh --record.')
    name = os.path.basename(log_dir)
    video = args.video or os.path.join(root, 'videos', name, f'{args.angle}.mp4')
    if not os.path.exists(video):
        sys.exit(f'{video} not found. Film it first: ./docker/replay.sh {name} -a {args.angle} --auto')
    csv_path = args.csv or matching_csv(root, name)
    if not csv_path:
        sys.exit(f'No logs/run_*.csv from after {name}. Pass one with --csv.')
    out = args.out or os.path.join(os.path.dirname(video), f'{args.angle}_overlay.mp4')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    rows = load_rows(csv_path)
    t0 = log_t0(log_dir) + args.offset
    w, h, fps, frames = probe(video)
    print(f'{os.path.basename(video)} ({w}x{h}, {fps:g} fps, {frames} frames) + {os.path.basename(csv_path)}, '
          f'video starts at sim time {t0:.2f} s')
    overlay = Overlay(rows, w, h, args.scale)

    dec = subprocess.Popen(['ffmpeg', '-v', 'error', '-i', video, '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'],
                           stdout=subprocess.PIPE)
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                            '-s', f'{w}x{h}', '-framerate', f'{fps:g}', '-i', '-',
                            '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18', out], stdin=subprocess.PIPE)
    size, n = w * h * 3, 0
    while True:
        buf = dec.stdout.read(size)
        if len(buf) < size:
            break
        img = overlay.draw(Image.frombytes('RGB', (w, h), buf), t0 + n / fps)
        enc.stdin.write(img.tobytes())
        n += 1
        if n % 500 == 0:
            print(f'  {n}/{frames}', flush=True)
    enc.stdin.close()
    dec.wait()
    if enc.wait() != 0:
        sys.exit('ffmpeg failed while writing the video.')
    print(f'Wrote {out} ({n} frames)')


if __name__ == '__main__':
    main()
