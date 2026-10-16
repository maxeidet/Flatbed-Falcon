"""Records the film cameras added by `start-dev.sh --film` (see film_cameras.py).

Runs INSIDE the sim container; docker/film.sh starts it. All shots record at the same
time from the same run, so the videos line up for editing. Stops on Enter, or by itself
a while after the drone touches down on the bed.

    python3 film.py <out dir> <shots, comma separated or "all"> <seconds after touchdown>
"""
import os
import re
import subprocess
import sys
import threading
import time

WORLD = 'hive_base'


def gz(*args, timeout=10):
    try:
        return subprocess.run(['gz', *args], capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ''


def record(shot, start, path=''):
    req = f'start: {str(start).lower()}, stop: {str(not start).lower()}, format: "mp4", save_filename: "{path}"'
    out = gz('service', '-s', f'/film/{shot}/record_video', '--reqtype', 'gz.msgs.VideoRecord',
             '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req)
    return 'data: true' in out


def sim_time():
    out = gz('topic', '-e', '-n', '1', '-t', f'/world/{WORLD}/stats', timeout=20)
    m = re.search(r'sim_time\s*\{\s*sec:\s*(\d+)(?:\s*nsec:\s*(\d+))?', out)
    return int(m.group(1)) + int(m.group(2) or 0) * 1e-9 if m else None


def main(out_dir, wanted, after_touchdown):
    services = gz('service', '-l')
    available = re.findall(r'^/film/([^/]+)/record_video$', services, flags=re.M)
    if not available:
        sys.exit('No film cameras found. Start the sim with --film (./docker/start-dev.sh --film) '
                 'and wait until the drone has spawned.')
    shots = available if wanted == 'all' else wanted.split(',')
    missing = [s for s in shots if s not in available]
    if missing:
        sys.exit(f'Unknown shot(s): {", ".join(missing)}. Available: {", ".join(sorted(available))}')

    os.makedirs(out_dir, exist_ok=True)
    started = [s for s in shots if record(s, True, os.path.join(out_dir, f'{s}.mp4'))]
    if not started:
        sys.exit('No camera started recording.')
    print(f'Recording {", ".join(started)}')
    print('Press Enter to stop' + (f', or it stops {after_touchdown:g} s (sim time) after touchdown.'
                                   if after_touchdown > 0 else '.'))

    done = threading.Event()
    threading.Thread(target=lambda: (sys.stdin.readline(), done.set()), daemon=True).start()

    def touchdown():
        # Wait for a bed contact that involves the drone (as landing_controller does).
        while not done.is_set():
            if 'x500' in gz('topic', '-e', '-n', '1', '-t', '/truck/bed_contact', timeout=30):
                t0 = sim_time()
                print(f'Touchdown. Filming {after_touchdown:g} s more.')
                while not done.is_set():
                    t = sim_time()
                    if t0 is None or t is None or t - t0 >= after_touchdown:
                        break
                    time.sleep(1)
                done.set()
                return

    if after_touchdown > 0:
        threading.Thread(target=touchdown, daemon=True).start()
    done.wait()

    # A camera only finishes its file on its next render, and nothing renders once every
    # recorder has unsubscribed. Keep one camera subscribed until all files are written.
    keep = subprocess.Popen(['gz', 'topic', '-e', '-t', f'/film/{started[0]}/image'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for s in started:
        record(s, False)
    print('Stopping and writing the videos...')
    files = {s: os.path.join(out_dir, f'{s}.mp4') for s in started}
    deadline = time.time() + 300
    while time.time() < deadline and not all(os.path.exists(f) for f in files.values()):
        time.sleep(1)
    keep.terminate()
    for s, f in files.items():
        if os.path.exists(f):
            print(f'  {s}: {f.replace("/workspace/", "")} ({os.path.getsize(f) / 1e6:.1f} MB)')
        else:
            print(f'  {s}: not written (check the PX4 terminal for errors)')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]))
