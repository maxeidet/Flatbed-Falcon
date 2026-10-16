"""Films one replay angle with no clicking (docker/replay.sh --auto, used by replay_all.sh).

Runs INSIDE the sim container next to `gz sim --playback`, after replay_camera.py has set
the angle. Keeps the replay paused and, for every video frame, seeks to that frame's sim
time and saves a screenshot of the GUI camera. The frames are numbered PNGs that
tools/frames_to_mp4.py turns into an mp4, so the video has every frame at the run's real
speed however slow the computer is.

    python3 replay_frames.py <log dir> <frames dir> [<fps>]
"""
import os
import sqlite3
import subprocess
import sys
import time

SETTLE = 0.1  # s between a seek and its screenshot, so the GUI shows the new state


def gz(*args, timeout=10):
    try:
        return subprocess.run(['gz', *args], capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ''


class Services:
    """Calls GUI and server services through the Python bindings (fast), or the gz CLI."""

    def __init__(self):
        self.node = None
        for transport, msgs in (('transport13', 'msgs10'), ('transport14', 'msgs11'), ('transport12', 'msgs9')):
            try:
                node_mod = __import__(f'gz.{transport}', fromlist=['Node'])
                pb = lambda name: __import__(f'gz.{msgs}.{name}_pb2', fromlist=['x'])  # noqa: E731
                self.LogPlaybackControl = pb('log_playback_control').LogPlaybackControl
                self.StringMsg = pb('stringmsg').StringMsg
                self.Boolean = pb('boolean').Boolean
                self.node = node_mod.Node()
                break
            except ImportError:
                continue
        if self.node is None:
            print('No gz Python bindings found: using the gz command, which is a lot slower.')

    def seek(self, service, t_ns):
        sec, nsec = divmod(t_ns, 1_000_000_000)
        if self.node:
            req = self.LogPlaybackControl(pause=True)
            req.seek.sec, req.seek.nsec = sec, nsec
            return self.node.request(service, req, self.LogPlaybackControl, self.Boolean, 3000)[0]
        gz('service', '-s', service, '--reqtype', 'gz.msgs.LogPlaybackControl', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '3000', '--req', f'pause: true seek {{ sec: {sec} nsec: {nsec} }}')
        return True

    def screenshot(self, directory):
        if self.node:
            req = self.StringMsg(data=directory)
            return self.node.request('/gui/screenshot', req, self.StringMsg, self.Boolean, 3000)[0]
        gz('service', '-s', '/gui/screenshot', '--reqtype', 'gz.msgs.StringMsg', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '3000', '--req', f'data: "{directory}"')
        return True


def log_span(log_dir):
    """First and last sim time (ns) in the recorded log."""
    db = sqlite3.connect(f'file:{os.path.join(log_dir, "state.tlog")}?mode=ro', uri=True)
    t0, t1 = db.execute('SELECT MIN(time_recv), MAX(time_recv) FROM messages').fetchone()
    db.close()
    return int(t0), int(t1)


def pngs(directory):
    return {f for f in os.listdir(directory) if f.endswith('.png') and not f[:5].isdigit()}


def wait_for_new(directory, before, timeout=5.0):
    """Waits for the screenshot the GUI saves on its next frame, and for it to be fully written."""
    end = time.time() + timeout
    while time.time() < end:
        new = pngs(directory) - before
        if new:
            path = os.path.join(directory, new.pop())
            size = -1
            while size != os.path.getsize(path):
                size = os.path.getsize(path)
                time.sleep(0.02)
            return path
        time.sleep(0.01)
    return None


def main(log_dir, frames_dir, fps=25.0):
    os.makedirs(frames_dir, exist_ok=True)
    services = Services()

    control = next((s for s in gz('service', '-l').split() if s.endswith('/playback/control')), None)
    if not control:
        sys.exit('The replay has no playback control service, so it cannot be filmed automatically.')
    for _ in range(60):
        if '/gui/screenshot' in gz('service', '-l'):
            break
        time.sleep(1)
    else:
        sys.exit('The replay window has no screenshot service, so it cannot be filmed automatically.')

    t0, t1 = log_span(log_dir)
    step = int(1e9 / fps)
    count = (t1 - t0) // step + 1
    print(f'Filming {count} frames ({(t1 - t0) / 1e9:.0f} s at {fps:g} fps) into {frames_dir}')

    services.seek(control, t0)
    time.sleep(3)  # let the camera settle on its target before the first frame
    started = time.time()
    for i in range(count):
        services.seek(control, t0 + i * step)
        time.sleep(SETTLE)
        before = pngs(frames_dir)
        services.screenshot(frames_dir)
        path = wait_for_new(frames_dir, before)
        if not path:
            sys.exit(f'No screenshot arrived for frame {i}: is the replay window still open?')
        os.replace(path, os.path.join(frames_dir, f'{i:05d}.png'))
        if i % 100 == 0 or i == count - 1:
            done = (i + 1) / count
            left = (time.time() - started) / done * (1 - done)
            print(f'  frame {i + 1}/{count}, about {left / 60:.0f} min left', flush=True)
    print('All frames saved.')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 25.0)
