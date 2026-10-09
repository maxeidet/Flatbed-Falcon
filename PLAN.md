# Flatbed Falcon: project plan

Goal: a PX4 drone autonomously lands on the flatbed of an autonomously driven pickup truck in Gazebo. Both vehicles are independent agents that only share information over ROS 2 topics, as two real vehicles would over a radio link.

![Road loop](assets/road_map.png)

## System overview

```
 TRUCK AGENT                                    DRONE AGENT
 ┌──────────────────────────┐                   ┌──────────────────────────────┐
 │ road_centerline.csv      │                   │ target_tracker               │
 │   ↓                      │  /truck/odom      │  truck state → NED, KF,      │
 │ truck_driver             │ ────────────────► │  prediction  → /falcon/target│
 │  SplinePath + PurePursuit│  /truck/status    │   ↓                          │
 │  + speed profile         │ ◄──────────────── │ landing_controller           │
 │   ↓ /truck/cmd_vel       │  /falcon/request  │  state machine + tracking    │
 │ Gazebo Ackermann plugin  │                   │   ↓ /fmu/in/*                │
 └──────────────────────────┘                   │ PX4 offboard (x500)          │
                                                └──────────────────────────────┘
                    logger: CSV of all of the above → plots
```

## The world (done)

`worlds/generate_hive_base.py` now builds a **closed 1129 m road loop** around the camp, made of straights joined by circular arcs (radius 15–25 m):

- **Three landing zones**: straights with exactly zero curvature, marked in the world with white/orange posts at both ends.
  - Zone 0: south straight, 160 m
  - Zone 1: inner straight past the access road, 109 m
  - Zone 2: middle straight of the serpentine, 131 m
- **Hard parts for path following**: a serpentine with two 180° hairpins, an S-curve on the east side, and a tight 15 m crest curve in the north-west.
- **Clear corridor**: tree crowns, boulders and fallen logs are kept clear of the road, so nothing within 7 m of the centerline has collision geometry and the drone has open air above the truck.
- **Centerline export**: the generator also writes `worlds/road_centerline.csv`, with one point per meter: `s, x, y, heading, curvature, landing_zone` (world ENU, `landing_zone = -1` outside the zones). This is the single source of truth for the truck's path and for the cooperative landing logic.

To change the road, edit `ROAD_CORNERS` and rerun `python3 worlds/generate_hive_base.py`.

## Phases

Each phase ends in something that can be demonstrated in the simulator.

### Phase 1: Truck in the world
- [x] Add a pickup model to the generator. It needs a dynamic model, unlike the static models already there:
  - chassis, cab, and a flatbed about 2.0 × 2.5 m at about 1.0 m height, with a high-friction landing pad and a visual marker
  - 4 wheels driven by `gz-sim-ackermann-steering-system`, with wheelbase L ≈ 3.0 m
  - `gz-sim-odometry-publisher-system` for ground-truth pose and twist
  - a contact sensor on the bed, used later for touchdown detection
- [x] Spawn the truck on the road, at the start of zone 1 near the access road.
- [x] Add a `ros_gz_bridge` config to the launch file: `/truck/cmd_vel` (Twist), `/truck/odom` (Odometry), `/truck/bed_contact`.
- [x] Use `use_sim_time` in all nodes.

**Done when** `ros2 topic pub /truck/cmd_vel ...` drives the truck and `/truck/odom` reports its motion.

### Phase 2: Truck autonomy (`truck_driver`), reusing Lab 3
- [x] Load `road_centerline.csv`, build a `SplinePath` (copy `HI3/splinepath.py` into the package), and handle the wrap-around at `s = 1129 m`.
- [x] Lateral control: adapt `PurePursuitController` from `HI3/main.py`, using lookahead `l = l0 + k·v` and turning steering into a yaw rate with `ω = v·tan(δ)/L` for the Ackermann plugin.
- [x] Longitudinal control: build a speed profile from curvature, `v_ref(s) = min(v_max, sqrt(a_lat,max / |κ(s)|))`, look ahead so the truck brakes before curves, and track it with a PI speed controller.
- [x] Publish `/truck/status`: current `s`, `v`, whether the truck is in a landing zone, and the remaining distance in that zone.
- [ ] Optional: compare with Lab 3's LQR `StateFeedbackController`, which is a good report section.

**Done when** the truck drives laps without leaving the road, with a lateral error plot. Target: under 0.5 m on straights and under 1.0 m in hairpins.

### Phase 3: Target tracking (`target_tracker`)
- [x] Convert `/truck/odom` (world ENU) into PX4 local NED with `frames.world_enu_to_px4_local()` and the drone spawn pose. Also convert velocity and yaw.
- [x] Publish `/falcon/target`: bed position (body offset from the truck origin applied), velocity, and yaw.
- [x] Add configurable Gaussian noise, latency, and dropout parameters to simulate a realistic link.

**Done when** the target's NED position matches the drone's NED position when the drone sits on the bed. This is the frame sanity check.

### Phase 4: Tracking flight (`landing_controller`, first part)
- [x] PX4 offboard boilerplate: stream `OffboardControlMode` at ≥ 10 Hz, switch to offboard mode, arm.
- [x] States `IDLE → TAKEOFF → APPROACH → TRACK`.
- [x] TRACK sends `TrajectorySetpoint` with **position = predicted bed position + (0, 0, −h_track)** and **velocity = truck velocity** as feedforward, and the yaw follows the truck heading. PX4's inner position and velocity loops do the rest.
- [x] Outer correction loop on the relative error: PD first, then LQR on a double integrator using `solve_discrete_are` as in Lab 3.

**Done when** the drone follows the truck at 4 m above the bed for a full lap, with horizontal error under 0.3 m on straights.

### Phase 5: Landing
- [x] States `DESCEND → TOUCHDOWN → LANDED`, plus `ABORT → TRACK`.
- [x] DESCEND is only allowed inside a landing zone, with enough zone left for the descent, after `|e_xy| < 0.3 m` and `|e_v| < 0.3 m/s` have held for 1.5 s.
- [x] Descend at about 0.5 m/s inside a glide cone where the allowed error shrinks with height. Leaving the cone or reaching the end of the zone triggers ABORT and the drone climbs back to `h_track`.
- [x] Touchdown at about 0.2 m above the bed: fast final descent, contact detected through the bed contact sensor or `e_z ≈ 0` with `v_z ≈ 0`, then force-disarm with `VehicleCommand` disarm `param2 = 21196`. PX4's own land detector is unreliable on a moving platform.
- [x] Failsafes: target stale for more than 0.5 s means hover and climb; offboard setpoints lost means PX4 failsafe.

**Done when** the drone lands on the bed at 3 m/s and stays on through the following curves.

### Phase 6: Cooperation (Lab 4 idea)
- [x] Handshake: the drone publishes `/falcon/request = READY_TO_LAND`. The truck replies through `/truck/status = LANDING_OK` when it enters a landing zone with enough length left, and it then **commits** to a constant speed until the zone ends or the drone reports `LANDED` or `ABORT`.
- [x] The truck may slow down for the landing, as a cooperation knob to evaluate.
- [x] After `LANDED`, the truck continues driving laps with the drone on the bed.

### Phase 7: Estimation and prediction (Labs 4 and 5)
- [x] Kalman filter in `target_tracker` with a constant-velocity or constant-turn-rate model, fed by noisy, delayed measurements.
- [ ] Predict the bed position τ seconds ahead to compensate latency. The truck's known path (the CSV) can be used as a prior, so the prediction follows the road instead of a straight line.
- [ ] Stretch goal: a downward camera with an ArUco/AprilTag marker on the bed, fused into the filter during the final meters.

### Phase 8: Logging and evaluation (`logger`)
- [x] CSV per run: time, truck state, drone state, target (raw, filtered, predicted), controller state, errors.
- [ ] Plot script for trajectories on the map, tracking error vs. time, a state-machine timeline, and touchdown position on the bed.
- [ ] Batch experiments (seeded): truck speed 2–8 m/s × noise × latency, measuring success rate, touchdown error, time to land, and number of aborts.

## Status (2026-10-09)

Phases 1–6 are implemented, and the first part of 7 and 8. Verified in a headless Gazebo run: the truck drives the loop with at most 0.43 m lateral error, the drone takes off from the camp, chases the truck, tracks it, gets LANDING_OK in zone 0 and lands 0.02 m from the bed centre at 4 m/s (bed contact sensor fired). The drone stays on the bed through the following curves at 8 m/s, although it slides about 0.8 m sideways over a few curves.

Implementation notes:

- **Horizontal control is Lab 4's double-integrator PD** (`u = kv(v* − v) + kp(p* − p)`) with the truck's turning acceleration as feed-forward, sent to PX4 as an acceleration setpoint for x and y. The height is a separate profile run by the state machine. `control_mode` in `config/falcon.yaml` switches between `pd`, `pid`, `lqr` (gains from `solve_discrete_are`) and `px4` (position + velocity feed-forward, PX4's own loops).
- **PID is the default, because the logs needed it.** With the PD, the drone trailed the bed by a steady 0.56 m along the track (0.03 m across it) while commanding a constant 0.65 m/s² forward. That is the x500's rotor drag in Gazebo, a constant force, which a PD can only hold with an error of drag / kp. That is above the 0.3 m descent gate, so the PD never landed. The I term (anti-windup: integration window, clamp, kept through TRACK → DESCEND → TOUCHDOWN) removes it.
- **The truck commits only once it is at v_land.** Committing while it was still braking made the PD lag a/kp ≈ 1.25 m behind and leave the glide cone.
- **PX4's estimate is useless after disarm on the truck.** PX4 thinks it is standing still, so its local position drifts tens of metres while the truck carries it. The logs after LANDED therefore do not show where the drone really is; Gazebo's ground truth does.
- **The drone is `gz_x500_gimbal`** (PX4's x500 with a CGO3 3-axis gimbal camera), the default in every start script. `landing_controller` takes control of PX4's gimbal manager and keeps the camera on the bed at 5 Hz, pointing straight down without a target (`gimbal_mode` in `falcon.yaml`). In Gazebo the median pointing error was 5.5° while tracking and 2.9° while descending. The image is bridged lazily to `/falcon/camera/image` (1280×720), ready for the marker detection in phase 7.
- **Offline tests.** `ros2_ws/src/flatbed_falcon/test/` checks the road, a kinematic truck lap and the full landing with a PX4-like drone model (with drag and PX4's velocity integrator), without Gazebo.

Not done yet: LQR for the truck (phase 2, optional), road-shaped prediction and camera/marker (phase 7), plot script and batch experiments (phase 8), and a run with link noise, latency and the Kalman filter in Gazebo (only tested offline).

## Lab code reuse

| Lab | What we reuse | Where |
|---|---|---|
| Lab 3 (path following) | `SplinePath`, `PurePursuitController`, LQR state feedback, `solve_discrete_are` | `truck_driver`, outer loop in `landing_controller` |
| Lab 4 (collaborative control) | Cooperative agents, relative measurements, consensus thinking | landing handshake, relative-state filter |
| Lab 5 (trajectory prediction) | Predicting a vehicle's future position | `target_tracker` prediction step |
| Lab 1/2 (planning) | Graph search / lattice planner | Optional: plan the truck's exit from the parking area to the loop |

## Interfaces

| Topic | Type | From → to | Frame |
|---|---|---|---|
| `/truck/odom` | `nav_msgs/Odometry` | Gazebo → `truck_driver`, `target_tracker`, `logger` | world ENU |
| `/truck/cmd_vel` | `geometry_msgs/Twist` | `truck_driver` → Gazebo | truck body |
| `/truck/status` | custom or JSON `std_msgs/String` | `truck_driver` → drone | — |
| `/truck/bed_contact` | `ros_gz_interfaces/Contacts` | Gazebo → `landing_controller` | — |
| `/falcon/target` | `nav_msgs/Odometry` | `target_tracker` → `landing_controller`, `logger` | PX4 local NED |
| `/falcon/request` | custom or `std_msgs/String` | `landing_controller` → `truck_driver` | — |
| `/falcon/state` | `std_msgs/String` | `landing_controller` → `logger` | — |
| `/fmu/in/*`, `/fmu/out/*` | `px4_msgs` | PX4 bridge | PX4 local NED |

All frame conversions stay in `frames.py`.

## Starting parameters (tune in phases 2, 4 and 5)

| Parameter | Value |
|---|---|
| Truck max speed / lateral acceleration | 8 m/s / 2 m/s² (≈ 5.5 m/s in 15 m curves) |
| Pure Pursuit lookahead | `l = 3 m + 0.8 s · v` |
| Track height above bed | 4 m |
| Descent rate | 0.5 m/s, final 1.0 m/s below 0.2 m |
| Descent gate | `|e_xy| < 0.3 m`, `|e_v| < 0.3 m/s` for 1.5 s |
| Landing zone budget | ≥ 15 s of zone remaining at current speed |
| Setpoint rate | 20 Hz |

## Risks

| Risk | Mitigation |
|---|---|
| PX4 land detector fails on a moving platform | Own touchdown detection plus force-disarm |
| Drone slides off the bed in curves after landing | High-friction pad; stretch: detachable joint to latch the drone |
| Frame or offset bugs (ENU/NED, spawn offset) | Phase 3 sanity check, all conversions in `frames.py` |
| Sim slower than real time on a laptop | Run headless on Stellar for batch runs; all nodes on `use_sim_time` |
| Rotor downwash or ground effect near the bed | Accept in sim; keep final descent short |
| A landing zone is too short at high speed | Speed-dependent zone budget, ABORT at zone end, truck slows when committed |

## Open questions

1. Ground truth plus simulated noise, or camera and marker detection as the main target measurement? The current plan uses ground truth as the baseline and the camera as a stretch goal.
2. Should the drone take off from the camp (origin) and chase the truck, or start on the truck bed?
