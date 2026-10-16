"""Turns numbered PNG frames (00000.png, 00001.png, ...) into an mp4, then deletes the frames.
Used by docker/replay_all.sh. Needs ffmpeg (or `python3 -m pip install --user imageio-ffmpeg`).

    python3 tools/frames_to_mp4.py <frames dir> <out.mp4> [--fps 25] [--keep]
"""
import argparse
import os
import shutil
import subprocess
import sys


def ffmpeg_exe():
    exe = shutil.which('ffmpeg')
    if exe:
        return exe
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        sys.exit('ffmpeg is missing: python3 -m pip install --user imageio-ffmpeg')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frames')
    ap.add_argument('out')
    ap.add_argument('--fps', type=float, default=25.0)
    ap.add_argument('--keep', action='store_true', help='keep the PNG frames')
    args = ap.parse_args()

    if not os.path.exists(os.path.join(args.frames, '00000.png')):
        sys.exit(f'No frames in {args.frames}')
    cmd = [ffmpeg_exe(), '-y', '-loglevel', 'error', '-framerate', f'{args.fps:g}',
           '-i', os.path.join(args.frames, '%05d.png'),
           '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
           '-crf', '18', args.out]
    subprocess.run(cmd, check=True)
    print(f'Saved {args.out}')
    if not args.keep:
        shutil.rmtree(args.frames)


if __name__ == '__main__':
    main()
