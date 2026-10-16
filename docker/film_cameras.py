"""Adds the film cameras to the container's copy of the world (start-dev.sh --film).

Runs INSIDE the sim container before PX4 starts Gazebo, like enable-log-record.sh, so
worlds/hive_base.sdf itself is never changed and normal runs have no extra cameras.

Every camera is a Gazebo camera sensor with the CameraVideoRecorder system. They only
render while docker/film.sh is recording, and they record on sim time, so the videos
are smooth and real speed however slow the computer is (the sim just runs slower).

Shots on the truck move with it (they are sensors on the chassis link, no extra mass):
  truck_chase   behind and above the truck, the drone chasing it in frame
  truck_side    alongside the truck, a tracking shot from the left
  bed_closeup   on the cab roof looking back at the bed, for the touchdown
  overhead      straight down from 35 m, the drone over the bed
Fixed tripods beside the three landing zones, placed where no tree blocks the view:
  tripod_z0, tripod_z1, tripod_z2
And the drone's own gimbal camera (PX4's model, so it is patched in the models folder):
  gimbal

    python3 film_cameras.py <world.sdf> <road_centerline.csv> [<PX4 models dir>]
"""
import csv
import math
import os
import re
import sys

# FILM_RES=960x540 FILM_FPS=15 ./docker/start-stellar.sh --film: lower resolution and frame
# rate, so each camera costs less.
_fps = os.environ.get('FILM_FPS', '25')
if not _fps.isdigit() or int(_fps) == 0:
    sys.exit(f'film_cameras: FILM_FPS must be a whole number like 15, got {_fps!r}')
FPS = int(_fps)
_res = re.fullmatch(r'(\d+)x(\d+)', os.environ.get('FILM_RES', '1280x720'))
if not _res:
    sys.exit(f"film_cameras: FILM_RES must look like 960x540, got {os.environ.get('FILM_RES')!r}")
WIDTH, HEIGHT = int(_res.group(1)), int(_res.group(2))
BED_X, BED_TOP, CHASSIS_Z = -1.15, 1.0, 0.4   # generate_hive_base.py (BED_X, BED_TOP, wheel radius)
H_TRACK = 4.0                                  # falcon.yaml: drone height above the bed while tracking

# name: (camera position, point it looks at, horizontal fov) in the truck's chassis frame
# (x forward, y left, z up, origin at the chassis link 0.4 m above the ground).
TRUCK_SHOTS = {
    'truck_chase': ((-16.0, 0.0, 6.0), (BED_X, 0.0, 2.5), 1.05),
    'truck_side': ((BED_X, 14.0, 2.5), (BED_X, 0.0, 2.0), 1.05),
    'bed_closeup': ((1.0, 0.0, 2.4), (BED_X - 0.5, 0.0, BED_TOP + 1.0 - CHASSIS_Z), 1.5),
    'overhead': ((BED_X, 0.0, 35.0), (BED_X + 0.001, 0.0, 0.0), 1.05),
}


def look_at(p, a):
    """Roll, pitch, yaw that point a camera (looking along +x) from p at a."""
    dx, dy, dz = a[0] - p[0], a[1] - p[1], a[2] - p[2]
    return 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)


def fmt(*v):
    return ' '.join(f'{x:.4f}' for x in v)


def sensor(name, p, a, fov):
    return (
        f'<sensor name="{name}" type="camera"><pose>{fmt(*p, *look_at(p, a))}</pose>'
        f'<camera><horizontal_fov>{fov}</horizontal_fov>'
        f'<image><width>{WIDTH}</width><height>{HEIGHT}</height></image>'
        '<clip><near>0.1</near><far>1500</far></clip></camera>'
        f'<always_on>true</always_on><update_rate>{FPS}</update_rate><topic>/film/{name}/image</topic>'
        '<plugin filename="gz-sim-camera-video-recorder-system" name="gz::sim::systems::CameraVideoRecorder">'
        f'<service>/film/{name}/record_video</service><use_sim_time>true</use_sim_time>'
        f'<fps>{FPS}</fps><bitrate>8000000</bitrate></plugin></sensor>'
    )


def trees(sdf):
    """(x, y, crown radius) of every tree, from its collision cylinder (radius = 0.35 * crown)."""
    out = []
    for m in re.finditer(r'<collision name="c\d+"><pose>([^<]+)</pose><geometry><cylinder>'
                         r'<radius>([^<]+)</radius>', sdf):
        x, y = (float(v) for v in m.group(1).split()[:2])
        out.append((x, y, float(m.group(2)) / 0.35))
    return out


def seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)))
    return math.hypot(px - ax - t * vx, py - ay - t * vy)


def tripod_pose(zone_rows, tree_list):
    """Position beside the middle of a landing zone with a clear view, and the point it looks at."""
    mid = zone_rows[len(zone_rows) // 2]
    x, y, h = float(mid['x']), float(mid['y']), float(mid['heading'])
    target = (x, y, BED_TOP + 1.5)
    best = None
    for dist in (12.0, 9.0, 15.0, 18.0, 7.0):
        for side in (1, -1):
            for along in (0.0, -10.0, 10.0):
                cx = x - side * dist * math.sin(h) + along * math.cos(h)
                cy = y + side * dist * math.cos(h) + along * math.sin(h)
                clear = min((seg_dist(tx, ty, cx, cy, x, y) - r for tx, ty, r in tree_list),
                            default=99.0)
                if best is None or clear > best[0]:
                    best = (clear, cx, cy)
                if clear > 1.0:
                    break
            if best[0] > 1.0:
                break
        if best[0] > 1.0:
            break
    _, cx, cy = best
    return (cx, cy, 4.0), target


def tripod_poses(sdf, rows):
    """{'tripod_z0': (position, look-at point), ...} for every landing zone in the road CSV."""
    tree_list = trees(sdf)
    zones = sorted({r['landing_zone'] for r in rows} - {'-1'})
    return {f'tripod_z{z}': tripod_pose([r for r in rows if r['landing_zone'] == z], tree_list)
            for z in zones}


def tripod(name, p, target):
    """A static camera model at p looking at target."""
    return (
        f'<model name="film_{name}"><static>true</static><pose>{fmt(*p, *look_at(p, target))}</pose>'
        f'<link name="link">{sensor(name, (0, 0, 0), (1, 0, 0), 1.2)}</link></model>'
    )


def patch_gimbal(models_dir):
    """Adds a recorder to the drone's own gimbal camera (PX4's gimbal model), as shot 'gimbal'.
    It records exactly what the drone's camera sees, at the camera's own resolution."""
    path = os.path.join(models_dir, 'gimbal', 'model.sdf')
    if not os.path.exists(path):
        print(f'film_cameras: {path} not found, no gimbal shot')
        return False
    text = open(path).read()
    if '/film/gimbal/record_video' in text:
        return True
    i = text.find('<sensor name="camera" type="camera">')
    if i < 0:
        print(f'film_cameras: no camera sensor in {path}, no gimbal shot')
        return False
    k = text.index('</sensor>', i)
    plugin = ('<plugin filename="gz-sim-camera-video-recorder-system" name="gz::sim::systems::CameraVideoRecorder">'
              '<service>/film/gimbal/record_video</service><use_sim_time>true</use_sim_time>'
              f'<fps>{FPS}</fps><bitrate>8000000</bitrate></plugin>')
    open(path, 'w').write(text[:k] + plugin + text[k:])
    return True


def main(sdf_path, csv_path, models_dir=None):
    sdf = open(sdf_path).read()
    shots = list(TRUCK_SHOTS)
    rows = list(csv.DictReader(open(csv_path)))
    if 'name="truck_chase"' not in sdf:
        i = sdf.index('<model name="truck">')
        j = sdf.index('<link name="chassis">', i)
        k = sdf.index('</link>', j)
        sdf = sdf[:k] + ''.join(sensor(n, *s) for n, s in TRUCK_SHOTS.items()) + sdf[k:]
        poses = tripod_poses(sdf, rows)
        sdf = sdf.replace('</world>', ''.join(tripod(n, *pt) + '\n    ' for n, pt in poses.items())
                          + '</world>', 1)
        open(sdf_path, 'w').write(sdf)
    shots += list(tripod_poses(sdf, rows))
    if models_dir and patch_gimbal(models_dir):
        shots.append('gimbal')
    print(f'Film cameras added at {WIDTH}x{HEIGHT}, {FPS} fps: ' + ', '.join(shots)
          + ' (record with docker/film.sh)')


if __name__ == '__main__':
    main(*sys.argv[1:4])
