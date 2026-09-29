# PX4 / Gazebo / ROS 2 dev container

## Starting local dev on macOS (`start-dev.sh`)

`start-dev.sh` only works on macOS (it opens Terminal.app with AppleScript). On the Stellar (Ubuntu) computers use `start-stellar.sh`, see below.

```bash
./start-dev.sh                        # default drone (gz_x500) in baylands
./start-dev.sh -m gz_x500_depth        # pick a different sensor loadout
./start-dev.sh --model gz_x500_lidar_front
./start-dev.sh -w forest               # pick a different world
./start-dev.sh -m gz_x500_depth -w ridge
./start-dev.sh -h                      # show usage
```

The `-m`/`--model` flag sets which drone variant gets spawned — any value from the `PX4_SIM_MODEL` table below. Defaults to `gz_x500` (no extra sensors) if omitted.

The `-w`/`--world` flag sets which Gazebo world gets loaded — any value from the world table below. **Defaults to `baylands`** if omitted.

**Available worlds** (all bundled in the image already, no download needed):

| `-w` value                      | Description                                                                                                                                                                             |
| ------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `baylands` (default)            | Terrain + water, more realistic environment. Takes noticeably longer to load than the others — the "Waiting for Gazebo world..." line can repeat for 30–60s, that's normal, not a hang. |
| `default`                       | Flat empty ground plane                                                                                                                                                                 |
| `forest`                        | Trees / vegetation                                                                                                                                                                      |
| `ridge`                         | Hilly / elevated terrain                                                                                                                                                                |
| `walls`                         | Enclosed walled area                                                                                                                                                                    |
| `lawn`                          | Grass field                                                                                                                                                                             |
| `windy`                         | Wind-affected environment                                                                                                                                                               |
| `aruco`                         | Contains ArUco markers (for vision/tag detection)                                                                                                                                       |
| `frictionless`                  | Frictionless ground (rover/testing edge cases)                                                                                                                                          |
| `underwater`                    | Underwater environment                                                                                                                                                                  |
| `rover`                         | Rover-oriented terrain                                                                                                                                                                  |
| `kthspacelab`, `kth_marinarium` | KTH-specific scenario worlds                                                                                                                                                            |
| `moving_platform`               | Includes a moving landing platform                                                                                                                                                      |

**What the script actually starts:**

1. Builds the `flatbed-falcon-px4` image from `docker/Dockerfile` (the official `px4io/px4-sitl-gazebo-ros2` base image, plus `Xvfb` + `fluxbox` + `x11vnc` + `noVNC`/`websockify` for the GUI layer). The Dockerfile also sets PX4's `NAV_DLL_ACT` to 0 in the x500 airframe (simulation only), so the drone can be armed by the backend without QGroundControl connected.
2. Runs that image as a container named `flatbed-falcon-sim`, publishing:
   - port `6080` → noVNC in the browser (`http://localhost:6080/vnc.html`)
   - port `5900` → native VNC client (`vnc://127.0.0.1:5900`)
   - both bound to `127.0.0.1` only, and protected by a **random password generated on every run** (printed by the script, nothing hardcoded in the repo)
   - the repo root mounted at `/workspace` inside the container
   - the container's default command starts the DDS agent (`MicroXRCEAgent udp4 -p 8888`), bridging PX4 ↔ ROS 2
3. Opens a **second terminal window** automatically, which waits for the container to be running, then starts PX4 + Gazebo (drone instance `0`, GUI mode — not headless) with the chosen `PX4_SIM_MODEL`.
4. Connect QGroundControl separately (it auto-detects the running PX4 instance on macOS via Docker Desktop's gateway IP).

Stop everything with `Ctrl+C` in the first terminal (the container runs with `--rm`, so it's removed on exit).

## Running on Stellar (LiU lab computers)

Use `start-stellar.sh` there (one terminal, no AppleScript). The default is just `./start-stellar.sh`, which means `--native-gui -w hive_base`. The full steps, the rules that affect us, and how to clean up after a session are in [README-stellar.md](README-stellar.md).

---

# Drone sensors (PX4 SITL / Gazebo)

Verified by inspecting the model files directly inside the `px4io/px4-sitl-gazebo-ros2` image (`/opt/px4-gazebo/share/gz/models/`).

## Default: every drone always has this

No matter which `PX4_SIM_MODEL` variant we pick, they all build on the base model `x500_base`, which always includes:

1. Every drone (base `x500`) already has, regardless of variant — confirmed from `x500_base/model.sdf`:
   - IMU (`imu_sensor`)
   - Magnetometer (`magnetometer_sensor`)
   - Barometer (`air_pressure_sensor`)
   - GPS (`navsat_sensor`)

No camera, LiDAR, or radar is included in the base model.

## Extra sensors: pick via `PX4_SIM_MODEL`

We switch the sensor loadout by setting this environment variable before starting `px4-gazebo`, e.g:

```bash
export PX4_SIM_MODEL=gz_x500_depth
```

| `PX4_SIM_MODEL` value | Extra sensor(s) on top of the default                             |
| --------------------- | ----------------------------------------------------------------- |
| `gz_x500`             | None — just IMU/magnetometer/barometer/GPS                        |
| `gz_x500_mono_cam`    | Regular camera (`mono_cam`)                                       |
| `gz_x500_depth`       | RGB camera + depth camera (OAK-D Lite: `camera` + `depth_camera`) |
| `gz_x500_lidar_2d`    | 2D LiDAR                                                          |
| `gz_x500_lidar_front` | 3D LiDAR, forward-facing (`gpu_lidar`)                            |
| `gz_x500_lidar_down`  | 3D LiDAR, downward-facing (`gpu_lidar`)                           |
| `gz_x500_flow`        | Optical flow sensor + LiDAR (altitude hold without GPS)           |
| `gz_x500_vision`      | No extra sensor — for external vision/mocap input instead         |
| `gz_x500_gimbal`      | Gimbal mount (pair with a camera model)                           |

**Radar doesn't exist** as a built-in sensor in PX4/Gazebo. For Counter-UAS detection, we'll either need to fake it with a scripted "detection radius" node, or build a custom Gazebo sensor plugin.

## Example: starting a drone with a depth camera

If the container is running via `./start-dev.sh` (container name `flatbed-falcon-sim`), the simplest way to get a depth-camera drone as drone 0 is to just pass the model to the script itself:

```bash
./start-dev.sh -m gz_x500_depth
```

## Example: adding a second drone with a depth camera

To add an _extra_ drone with a depth camera alongside whatever `start-dev.sh` already started (drone 0), run this in a new terminal — note the different container name, instance number, and spawn pose so it doesn't collide with drone 0:

```bash
docker exec -it flatbed-falcon-sim bash -c "
  export PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE='5,0,0,0,0,0'
  /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 1
"
```

It'll spawn into the same running Gazebo world and show up namespaced as `/px4_1/fmu/...`. Bump `-i` and the pose further for a third, fourth, etc. drone.

Sensor data from camera/LiDAR variants gets bridged to ROS 2 via `ros_gz` — check `ros2 topic list` after startup to see which topics are actually published for the chosen model.
