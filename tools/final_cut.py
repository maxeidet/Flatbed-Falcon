"""Cuts the final presentation video (about 3 min, 1920x1080) from the clips, overlays and
still images: a landing, LQR vs LQI, and the Kalman filter on vs off.

Every part is scaled into the upper 1920x960 with a caption bar underneath, fades in and out,
and the parts are joined with title cards between the sections. The source videos must exist
(see VIDEOS.md); the sim-time windows below assume the offsets listed there.

    .venv/bin/python tools/final_cut.py --out videos/flatbed_falcon_final.mp4

Needs Pillow and ffmpeg.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

W, H, AREA, FPS = 1920, 1080, 960, 25
BG = (22, 26, 33)
TEXT, MUTED, ACCENT = (235, 240, 250), (160, 166, 178), (90, 210, 110)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FADE = 0.4

# (kind, source or card text, start [s into the source], length [s], speed, caption)
SEGMENTS = [
    ('card', ('Flatbed Falcon', 'Landing a drone on a moving truck',
              'PX4 + Gazebo + ROS 2  ·  LQR vs LQI  ·  Kalman filter on vs off'), 0, 5, 1, None),

    ('card', ('1 · A landing', 'LQI controller with Kalman filter',
              'The drone tracks the bed 4 m above it. When position, speed and height\n'
              'are inside the descent gate for 1.5 s, it asks the truck for permission and descends.'), 0, 6, 1, None),
    ('video', 'videos/20261018-120332/lqi_overlay.mp4', 56, 24, 1,
     'Gate lamps top left: all three green for 1.5 s → the truck holds 4 m/s on a straight → descent → '
     'touchdown 0.09 m from the bed centre'),

    ('card', ('2 · LQR vs LQI', 'Same truck, same radio link, same Kalman filter',
              'Both get their gains from the same Riccati equation.\n'
              'LQI adds one state: the integral of the position error.'), 0, 6, 1, None),
    ('video', 'videos/lqi_vs_lqr.mp4', 52, 28.6, 1,
     'LQI (left) passes the gate and lands. LQR (right) hangs ~0.9 m behind the bed: '
     'its position lamp never turns green'),
    ('video', 'videos/clips/controller_dynamics_lqr_lqi.mp4', 0, 53, 2,
     'Rotor drag pushes the drone back. Without an integral term LQR can only hold it with a steady error '
     '(drag · v / kp); LQI integrates it away. 2× speed'),
    ('still', 'videos/figures/no_pid/along_track.png', 0, 7, 1,
     'Over the whole run: LQR sits 0.84 m behind the bed centre, LQI 0.01 m'),

    ('card', ('3 · Kalman filter on vs off', 'Same LQI controller, same noisy radio link',
              'The truck sends its position over a simulated radio:\n'
              '0.1 m noise, 0.1 s latency, 5 % dropouts.'), 0, 6, 1, None),
    ('video', 'videos/clips/kalman_vs_extrapolation.mp4', 17, 28, 1,
     'The measurements arrive 0.1 s late, so they trail behind the bed. The filter predicts them to "now" '
     'and smooths them, the height too'),
    ('video', 'videos/lqi_kalman_vs_none.mp4', 59, 20, 1,
     'Without the filter the noisy bed height makes the drone hover 0.8 m too low: '
     'the height lamp stays red and it never asks to land'),
    ('still', 'videos/figures/no_pid/height.png', 0, 7, 1,
     'Height while tracking: 4.00 m with the filter, 3.17 m without, outside the gate window'),
    ('still', 'videos/figures/no_pid/error_chain.png', 0, 7, 1,
     'The radio error is mostly latency (0.1 s × ~6 m/s). The filter brings it down to 0.02 m'),

    ('still', 'videos/figures/no_pid/results_overview.png', 0, 9, 1, None),
    ('card', ('LQI + Kalman filter', 'lands 0.09 m from the bed centre',
              'Integral action against drag  ·  filtered, latency-compensated target  ·  a strict descent gate'),
     0, 6, 1, None),
]


def font(path, size):
    from PIL import ImageFont
    return ImageFont.truetype(path, size)


def card_png(lines, path):
    from PIL import Image, ImageDraw
    title, sub, body = lines
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rectangle((0, H // 2 - 190, 14, H // 2 + 190), fill=ACCENT)
    d.text((120, H // 2 - 170), title, font=font(BOLD, 96), fill=TEXT)
    d.text((124, H // 2 - 40), sub, font=font(FONT, 46), fill=ACCENT)
    d.multiline_text((124, H // 2 + 50), body, font=font(FONT, 32), fill=MUTED, spacing=16)
    img.save(path)


def caption_png(text, path, speed):
    from PIL import Image, ImageDraw
    img = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rectangle((0, AREA, W, H), fill=BG + (255,))
    d.rectangle((0, AREA, W, AREA + 3), fill=(50, 56, 68, 255))
    if text:
        f = font(FONT, 30)
        # Wrap to at most two lines.
        words, lines, line = text.split(), [], ''
        for w in words:
            if d.textlength(f'{line} {w}'.strip(), font=f) > W - 360:
                lines.append(line)
                line = w
            else:
                line = f'{line} {w}'.strip()
        lines.append(line)
        y = AREA + (H - AREA) / 2 - (len(lines) * 42) / 2
        for ln in lines:
            d.text((W / 2, y), ln, font=f, fill=TEXT, anchor='ma')
            y += 42
    if speed != 1:
        # In the caption bar, so it never covers the clip's own labels.
        cy = AREA + (H - AREA) / 2
        d.rounded_rectangle((W - 130, cy - 26, W - 24, cy + 26), radius=10, fill=(60, 66, 78, 255))
        d.text((W - 77, cy), f'{speed:g}×', font=font(BOLD, 32), fill=TEXT, anchor='mm')
    img.save(path)


def render(seg, n, tmp, root):
    kind, src, start, length, speed, caption = seg
    out = os.path.join(tmp, f'seg{n:02d}.mp4')
    dur = length / speed
    fade = f'fade=t=in:st=0:d={FADE},fade=t=out:st={dur - FADE:.3f}:d={FADE}'
    enc = ['-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', '-r', str(FPS), '-an']
    if kind == 'card':
        png = os.path.join(tmp, f'card{n:02d}.png')
        card_png(src, png)
        cmd = ['-loop', '1', '-framerate', str(FPS), '-t', f'{dur}', '-i', png, '-vf', f'format=rgb24,{fade}']
    else:
        path = os.path.join(root, src)
        if not os.path.exists(path):
            sys.exit(f'Missing {src}: render it first (see VIDEOS.md).')
        cap = os.path.join(tmp, f'cap{n:02d}.png')
        caption_png(caption, cap, speed)
        if kind == 'still':
            inp = ['-loop', '1', '-framerate', str(FPS), '-t', f'{dur}', '-i', path]
            pre = '[0]'
        else:
            inp = ['-ss', f'{start}', '-t', f'{length}', '-i', path]
            pre = f'[0]setpts=PTS/{speed},'
        graph = (f'{pre}fps={FPS},scale={W}:{AREA}:force_original_aspect_ratio=decrease,'
                 f'pad={W}:{H}:(ow-iw)/2:({AREA}-ih)/2:color=0x161a21[v];'
                 f'[v][1]overlay=0:0,{fade}')
        cmd = inp + ['-loop', '1', '-framerate', str(FPS), '-t', f'{dur}', '-i', cap,
                     '-filter_complex', graph, '-t', f'{dur}']
    subprocess.run(['ffmpeg', '-v', 'error', '-y'] + cmd + enc + [out], check=True)
    return out, dur


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default='videos/flatbed_falcon_final.mp4')
    args = ap.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    tmp = tempfile.mkdtemp(prefix='final_cut_')
    try:
        parts, total = [], 0.0
        for n, seg in enumerate(SEGMENTS):
            out, dur = render(seg, n, tmp, root)
            parts.append(out)
            total += dur
            name = seg[1][0] if seg[0] == 'card' else os.path.basename(seg[1])
            print(f'  {n:2d}  {dur:5.1f} s  {name}', flush=True)
        listing = os.path.join(tmp, 'parts.txt')
        with open(listing, 'w') as fh:
            fh.writelines(f"file '{p}'\n" for p in parts)
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', listing,
                        '-c', 'copy', '-movflags', '+faststart', args.out], check=True)
        print(f'Wrote {args.out} ({total:.0f} s, {int(total // 60)}:{int(total % 60):02d})')
    finally:
        shutil.rmtree(tmp)


if __name__ == '__main__':
    main()
