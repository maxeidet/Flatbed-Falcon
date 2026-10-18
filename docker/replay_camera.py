"""Points the replay GUI's camera at one of the film angles (docker/replay.sh -a ...).

Runs INSIDE the sim container next to `gz sim --playback`. The angles match the film
cameras in film_cameras.py, so clips from replays and from film.sh cut together:
  drone_chase  behind and above the drone (default)
  truck_chase, truck_side, overhead   ride along with the truck, looking at it
  bed_closeup  on the cab roof, turned towards the drone as it comes down
  tripod_z0 / tripod_z1 / tripod_z2   fixed beside a landing zone, panning after the drone
The gimbal camera cannot be replayed (sensors do not run in playback): film it live
with `film.sh -s gimbal`.

    python3 replay_camera.py <angle> [<pgain>]
"""
import csv
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import film_cameras as fc  # noqa: E402

DRONE, TRUCK = 'x500_gimbal_0', 'truck'
CHASSIS_Z = fc.CHASSIS_Z
# Offsets in the followed model's frame (x forward, y left, z up from its origin on the ground).
FOLLOW = {
    'drone_chase': (DRONE, (-8.0, 0.0, 4.0), None),
    'truck_chase': (TRUCK, (-16.0, 0.0, 6.0 + CHASSIS_Z), None),
    'truck_side': (TRUCK, (fc.BED_X, 14.0, 2.5 + CHASSIS_Z), None),
    'overhead': (TRUCK, (fc.BED_X - 3.0, 0.0, 35.0), None),
    'bed_closeup': (TRUCK, (1.0, 0.0, 2.4 + CHASSIS_Z), DRONE),
}


def gz(*args, timeout=10):
    try:
        return subprocess.run(['gz', *args], capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ''


def vec(v):
    return f'{{ x: {v[0]} y: {v[1]} z: {v[2]} }}'


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def track(msg):
    # Published a few times: the GUI may not have discovered this publisher yet.
    for _ in range(3):
        gz('topic', '-t', '/gui/track', '-m', 'gz.msgs.CameraTrack', '-p', msg)
        time.sleep(0.5)


def follow_old(target, offset):
    """Older GUIs without /gui/track: plain follow (no smoothing gain, no look-at)."""
    gz('service', '-s', '/gui/follow', '--reqtype', 'gz.msgs.StringMsg', '--reptype', 'gz.msgs.Boolean',
       '--timeout', '3000', '--req', f'data: "{target}"')
    gz('service', '-s', '/gui/follow/offset', '--reqtype', 'gz.msgs.Vector3d', '--reptype', 'gz.msgs.Boolean',
       '--timeout', '3000', '--req', vec(offset).strip('{} ').replace(' y', ', y').replace(' z', ', z'))


def wait_for_model(name, max_s=30):
    """PX4 spawns the drone a few seconds into the log, so at the start of a replay it does
    not exist yet and a camera following it gets an empty target. Steps the paused replay
    forward one second at a time until the model is there. Returns the sim time it stopped
    at, or None if the model was there already."""
    def present():
        return f'- {name}' in gz('model', '--list')

    if present():
        return None
    control = next((s for s in gz('service', '-l').split() if s.endswith('/playback/control')), None)
    if not control:
        return None
    for sec in range(1, max_s + 1):
        gz('service', '-s', control, '--reqtype', 'gz.msgs.LogPlaybackControl', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '3000', '--req', f'pause: true seek {{ sec: {sec} }}')
        time.sleep(0.5)
        if present():
            return sec
    print(f'{name} did not appear in the first {max_s} s of the log.')
    return None


def main(angle, pgain=0.1):
    tripods = {}
    if angle.startswith('tripod_'):
        rows = list(csv.DictReader(open('/workspace/worlds/road_centerline.csv')))
        tripods = fc.tripod_poses(open('/workspace/worlds/hive_base.sdf').read(), rows)
    if angle not in FOLLOW and angle not in tripods:
        names = list(FOLLOW) + ['tripod_z0', 'tripod_z1', 'tripod_z2']
        sys.exit(f'Unknown angle {angle!r}. Angles: {", ".join(names)}')

    # Wait for the GUI's camera tracking plugin.
    for _ in range(240):
        if '/gui/move_to/pose' in gz('service', '-l'):
            break
        time.sleep(1)
    else:
        sys.exit('The Gazebo GUI did not come up, so the camera angle was not set.')
    time.sleep(2)

    needed = {DRONE} if angle in tripods else {n for n in FOLLOW[angle][::2] if n}
    if DRONE in needed:
        at = wait_for_model(DRONE)
        if at is not None:
            print(f'The drone appears at sim time {at} s: the replay is paused there.')

    gains = f'follow_pgain: {pgain} track_pgain: {pgain}'
    has_track = '/gui/track' in gz('topic', '-l')
    if not has_track:
        print('This Gazebo GUI has no /gui/track: using plain follow (bed_closeup looks at the truck, '
              'tripods stay fixed).')
    if angle in FOLLOW and not has_track:
        follow_old(*FOLLOW[angle][:2])
    elif angle in FOLLOW:
        target, offset, look_at = FOLLOW[angle]
        if look_at:
            track(f'track_mode: FOLLOW_LOOK_AT follow_target {{ name: "{target}" }} '
                  f'track_target {{ name: "{look_at}" }} follow_offset {vec(offset)} {gains}')
        else:
            track(f'track_mode: FOLLOW follow_target {{ name: "{target}" }} follow_offset {vec(offset)} {gains}')
    else:
        p, target = tripods[angle]
        q = quat(*fc.look_at(p, target))
        gz('service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '3000', '--req',
           f'pose {{ position {vec(p)} orientation {{ x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]} }} }}')
        time.sleep(2)
        if has_track:
            track(f'track_mode: TRACK track_target {{ name: "{DRONE}" }} track_offset {{ x: 0 y: 0 z: 0 }} {gains}')
    print(f'Camera set to {angle}. Start the video recorder (top left), then press play.')


if __name__ == '__main__':
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.1)
