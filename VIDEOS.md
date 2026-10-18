# Videos and the configuration they were made with

Every video made on 2026-10-18: which run it shows, how that run was configured, and how the video was made and synced. How the controllers, the Kalman filter and the error measurements work is in [CONTROLLERS.md](CONTROLLERS.md).

## Download

All the videos and still images are in one zip, `flatbed_falcon_videos_2026-10-18.zip` (1.3 GB), in this **[Google Drive folder](https://drive.google.com/drive/folders/1Q1wMDsK5-q4DVrhlGSGVsWqEiBOJH4Jf?usp=sharing)**.

| In the zip | Contents |
|---|---|
| `videos/flatbed_falcon_final.mp4` | **The final presentation video** (3:06). Narration script: [PRESENTATION_SCRIPT.md](PRESENTATION_SCRIPT.md) |
| `videos/clips/`, `videos/figures/` | Presentation clips and still images (section 3.5–3.6) |
| `videos/<log>/`, `videos/*.mp4` | Recordings with overlays, side by side, and the comparison from the CSV logs (section 3.2–3.4) |
| `recordings/` | The raw Gazebo recordings. In this file they're named after where they lie in the repo root (section 3.1) |
| `VIDEOS.md` | This file |

The videos themselves are not in Git: `videos/` is in `.gitignore`. The recordings in the repo root (`*.mp4`) aren't ignored, but they aren't committed either. The run logs are in `logs/` (also ignored). Everything lives only on this computer, with no backup.

## 1. Shared configuration (the same for every run)

Unless a run is listed differently in section 2, it used `ros2_ws/src/flatbed_falcon/config/falcon.yaml` with these values.

**Simulation:** started with `./docker/start-stellar.sh --record` (native GUI, GPU). World `hive_base`, drone `gz_x500_gimbal` spawned at the world origin. A Gazebo log of each run is in `logs/gz/<log>/`, and the CSV log in `logs/run_<time>.csv`. The CSV name is 15–20 s later than the Gazebo log name, since the logger starts after the sim.

**Truck** (`truck_driver`): road loop 1129 m with three landing zones (straights of 160, 109 and 131 m), Pure Pursuit (`lookahead` 3 m + 0.8·v), `v_max` 8 m/s, speed profile with `a_lat` 2 m/s², `a_brake` 1.5 m/s², `a_accel` 1 m/s². During a landing it holds `v_land` 4 m/s, and it only gives LANDING_OK with at least `zone_budget_s` 15 s of landing zone left.

**Landing logic** (`landing_controller`): `cruise_alt` 25 m, `h_track` 4 m, `descend_rate` 0.5 m/s, gate 0.3 m / 0.3 m/s / 1.5 s, `require_handshake` true, glide cone 0.2 + 0.1·height, `control_rate_hz` 20. The gimbal camera points at the bed (`gimbal_mode: track`).

**Controller gains** (as used by each mode):

| Mode | kp | kd | ki | |
|---|---|---|---|---|
| `pid` | 1.200 | 2.200 | 0.300 | `control_kp/kd/ki` in `falcon.yaml` |
| `lqr` | 1.074 | 1.579 | – | LQR, Q = diag(1.0, 0.3), R = 0.8 |
| `lqi` | 1.875 | 2.008 | 0.582 | LQI, Q = diag(0.3, 1.0, 0.3), R = 0.8 |

**Kalman filter** (`target_tracker`): `filter_accel_std` 1.0 m/s², `predict_ahead` 0, `rate_hz` 50, bed offset (−1.15, 0, 1.0) m.

**Simulated link** (`target_tracker`) in every run from run 2 onward: `link_pos_noise` 0.1 m, `link_vel_noise` 0.05 m/s, `link_latency` 0.1 s, `link_dropout` 0.05, `link_seed` 0.

## 2. The runs

| Run | Gazebo log | CSV log | Controller | Kalman filter | Link | Result |
|---|---|---|---|---|---|---|
| R0 | `20261018-102023` | `run_20261018_102049.csv` | PID | off | **ground truth** (all 0) | Landed. Made before this session, as the undisturbed baseline |
| R1 | `20261018-103902` | `run_20261018_103921.csv` | PID | on, **2D** (height unfiltered) | **0.3 m / 0.2 m/s / 0.2 s / 10%** | Never landed; stopped at 140 s |
| R2 | `20261018-104305` | `run_20261018_104325.csv` | PID | on, **2D** (height unfiltered) | 0.1 / 0.05 / 0.1 / 5% | Never landed (hovered 0.85 m too low); stopped at 281 s |
| **R3** | `20261018-105132` | `run_20261018_105148.csv` | **PID** | on, 3D | 0.1 / 0.05 / 0.1 / 5% | **Landed** at 82.95 s, 0.04 m from the bed centre |
| **R4** | `20261018-120332` | `run_20261018_120347.csv` | **LQI** | on, 3D | 0.1 / 0.05 / 0.1 / 5% | **Landed** at 78.85 s, 0.09 m (true) / 0.02 m (vs. estimate) |
| **R5** | `20261018-120619` | `run_20261018_120635.csv` | **LQR** | on, 3D | 0.1 / 0.05 / 0.1 / 5% | Never landed (lags 0.84 m behind); closed by us at ~210 s |
| **R6** | `20261018-124440` | `run_20261018_124456.csv` | **LQI** | **off** | 0.1 / 0.05 / 0.1 / 5% | Never landed (hovered 0.83 m too low); stopped at 221 s |
| R7 | `20261018-125011` | `run_20261018_125027.csv` | LQR | off | 0.1 / 0.05 / 0.1 / 5% | Stopped after 89 s on request. No videos |
| R8 | `20261018-131748` | `run_20261018_131803.csv` | LQI | on, 3D | 0.1 / 0.05 / 0.1 / 5%, **seed 1** | First of the planned repeat runs; stopped after 45 s on request. No videos |

The code changed between some of the runs:
- **R1–R2:** the Kalman filter estimated only x/y, and the height was copied raw from the link.
- **From R3:** the filter estimates x, y **and z** (`target_filter.py`). That's the change that made the landings work with link noise on.
- **From R4:** the `lqi` mode exists. It doesn't change the PID mode.

R3–R7 differ **only** in `control_mode` and `use_filter`. Every other setting, the seed included, is the same.

## 3. The videos

### 3.1 Gazebo recordings (repo root)

Made with the replay `./docker/replay.sh <log> -a drone_chase`: the camera follows the drone 8 m behind and 4 m above, with smoothing `pgain` 0.1. They were recorded with the Gazebo window's video button, on sim time. The **offset** is the sim time the recording starts at. It's what you pass as `--offset` to the overlay.

| File | Run | Resolution, length | Offset | How the offset was determined | Notes |
|---|---|---|---|---|---|
| `test.mp4` | ? | 582×798, 101.6 s | – | – | From 2026-10-16, before this session. Not used |
| `test2.mp4` | R3 (PID) | 2076×1324, 81.6 s | 6.0 | camera set by hand with the replay paused at 6 s | Jerky at the start (large window). Its overlay was overwritten later |
| `test3.mp4` | R3 (PID) | 1146×1192, 66.0 s | not determined | – | The camera follows the drone. No overlay made |
| `test4.mp4` | R3 (PID) | 1424×1166, 79.8 s | **11.92** | the touchdown you pointed out (71 s into the video = 82.92 s) | First assumed 8, which was wrong (see 3.8) |
| `test5.mp4` | R3 (PID) | 772×484, 78.4 s | **13.4** | the touchdown found in the video (69.5 s into the video) | Small window |
| `LQI1.mp4` | R4 (LQI) | 972×956, 82.6 s | **3.0** | recording started while paused; checked against the touchdown (75.6 s into the video = 78.60 s) | |
| `LQR1.mp4` | R5 (LQR) | 972×954, 80.7 s | **3.0** | recording started while paused; checked against the lift-off (0.25 s after LQI's in the video, 0.20 s in the data) | Doesn't land |
| `LQI_no_kalman.mp4` | R6 (LQI, no Kalman) | 1030×964, 86.5 s | **3.0** | recording started while paused; lift-off frame for frame the same as in `LQI1.mp4` (14.05 s in both runs) | Doesn't land |

### 3.2 Recordings with the measurement overlay (`tools/overlay_video.py`)

Each shows one run with: state, sim time, handshake, the **descent gate** (a lamp each for position < 0.3 m, speed < 0.3 m/s and height 4 ± 0.3 m, a bar filling toward 1.5 s, and "asked to land" once READY_TO_LAND has been sent), height above bed, drone → aim, speeds, raw link error, filtered error, drone → true bed, 20 s plots with the gate windows shaded green, and the bed from above. The definitions are in CONTROLLERS.md, section 4.

| File | Recording | Run | Offset | Notes |
|---|---|---|---|---|
| `videos/20261018-105132/drone_chase_overlay_test4_synced.mp4` | `test4.mp4` | R3 (PID) | 11.92 | **The correct one for test4** |
| `videos/20261018-105132/drone_chase_overlay_test5.mp4` | `test5.mp4` | R3 (PID) | 13.4 | |
| `videos/20261018-120332/lqi_overlay.mp4` | `LQI1.mp4` | R4 (LQI) | 3.0 | |
| `videos/20261018-120619/lqr_overlay.mp4` | `LQR1.mp4` | R5 (LQR) | 3.0 | |
| `videos/20261018-124440/lqi_no_kalman_overlay.mp4` | `LQI_no_kalman.mp4` | R6 (LQI, no Kalman) | 3.0 | The panel still says "Kalman filter", but "filtered error" here is the plain **extrapolation** of the latest measurement (about 0.12 m), since the filter is off |

### 3.3 Side by side (`tools/side_by_side.py`)

Two recordings next to each other on the **same sim time** (both offset 3.0). Each has its controller, state, height, true error and the gate lamps. Underneath is a shared strip (gate windows shaded green) with both runs' error to the true bed and height over the last 30 s, plus the bed from above with both drones. A run is frozen at its touchdown.

| File | Left | Right | Shows |
|---|---|---|---|
| `videos/lqi_vs_lqr.mp4` | `LQI1.mp4` (R4, LQI) | `LQR1.mp4` (R5, LQR) | LQI lands and LQR lags ~0.9 m behind the bed. The trucks are in different places at the end, since the LQI truck holds 4 m/s after LANDING_OK |
| `videos/lqi_kalman_vs_none.mp4` | `LQI1.mp4` (R4, LQI + Kalman) | `LQI_no_kalman.mp4` (R6, LQI without Kalman) | Same controller, filter on vs off: with the filter the height holds 4 m and it lands; without it, it hovers 0.8 m too low and never asks to land. The horizontal error is about the same |

### 3.4 Rendered from the CSV logs only (`tools/compare_video.py`)

| File | Runs | Notes |
|---|---|---|
| `videos/compare_pid_lqr_lqi.mp4` | R3 (PID), R5 (LQR), R4 (LQI) | 1280×720, 64 s. **Aligned on when each run enters TRACK** (not sim time). The bed from above in the truck's frame, error to the true bed, height, and a scoreboard with touchdown error. Each run is frozen at its touchdown |

### 3.5 Presentation clips (`videos/clips/`, 1920×1080, rendered from the CSV logs)

| File | Tool | Runs | Shows |
|---|---|---|---|
| `videos/clips/kalman_vs_extrapolation.mp4` | `tools/kalman_view.py` | R4 (LQI + Kalman) left, R6 (LQI, no filter) right, sim time 33–82 s (49 s) | "What the drone knows": the bed from above, zoomed in, with the radio measurements trailing behind the bed (latency), the aim point and its jitter, the drone; the bed height as measured vs. aim point; the drone height with the gate window. Same link seed, so the noise is the same in both |
| `videos/clips/controller_dynamics.mp4` | `tools/dynamics_video.py` | R3 (PID), R5 (LQR), R4 (LQI), aligned on TRACK (57 s) | Phase portrait of the along-track error against its rate, with the gate box; the along-track error over time; the truck's speed below it. LQR swings around ~1 m behind the bed; the error spikes come when the truck changes speed. A landed run is frozen where touchdown starts |

### 3.6 Still images (`videos/figures/`, 1920×1080 PNG, `tools/summary_figures.py`)

Statistics over TRACK at the 4 m tracking height, from 20 s after TRACK (the same selection as in CONTROLLERS.md).

| File | Runs | Shows |
|---|---|---|
| `results_overview.png` | R3, R5, R4, R6 | One card per run: landed or not, why, touchdown error, typical tracking error. R6 has no tracking number, since it isn't comparable (its truck never slowed for a landing) |
| `tracking_error.png` | R3, R5, R4 | Distribution of the distance drone → true bed per controller, with the gate (0.3 m) and the bed edge (1 m) |
| `along_track.png` | R3, R5, R4 | Where along the road each drone sits relative to the bed: LQR's distribution is shifted 0.84 m back |
| `height.png` | R4, R6 | The drone height with and without the Kalman filter, with the gate window 4 ± 0.3 m |
| `error_chain.png` | R4, R6 | The radio measurement's error (0.61 / 0.73 m, mostly latency) and the aim point's after processing (0.02 m with Kalman, 0.12 m extrapolation) |

### 3.7 The final presentation video (`tools/final_cut.py`)

`videos/flatbed_falcon_final.mp4`: 1920×1080, 3:06, no sound. Two stories, **LQR vs LQI** and **Kalman filter on vs off**; PID is left out. Every part has a caption bar underneath, and section cards come between the parts. The parts and the source windows:

| Time | Part | Source | Window |
|---|---|---|---|
| 0:00 | Title card | | 5 s |
| 0:05 | Section 1: A landing | | 6 s |
| 0:11 | LQI landing with the gate lamps | `videos/20261018-120332/lqi_overlay.mp4` (R4) | sim 59–83 s |
| 0:35 | Section 2: LQR vs LQI | | 6 s |
| 0:41 | LQI vs LQR side by side | `videos/lqi_vs_lqr.mp4` (R4, R5) | sim 55–83.6 s |
| 1:10 | Phase portrait, 2× speed | `videos/clips/controller_dynamics_lqr_lqi.mp4` (R5, R4) | 0–53 s after TRACK |
| 1:36 | Along-track still | `videos/figures/no_pid/along_track.png` | 7 s |
| 1:43 | Section 3: Kalman filter on vs off | | 6 s |
| 1:49 | "What the drone knows" | `videos/clips/kalman_vs_extrapolation.mp4` (R4, R6) | sim 50–78 s |
| 2:17 | LQI with vs without Kalman side by side | `videos/lqi_kalman_vs_none.mp4` (R4, R6) | sim 62–82 s |
| 2:37 | Height still | `videos/figures/no_pid/height.png` | 7 s |
| 2:44 | Error chain still | `videos/figures/no_pid/error_chain.png` | 7 s |
| 2:51 | Results (LQR, LQI, LQI without Kalman) | `videos/figures/no_pid/results_overview.png` | 9 s |
| 3:00 | End card | | 6 s |

The versions without PID: `videos/clips/controller_dynamics_lqr_lqi.mp4` and `videos/figures/no_pid/` (`summary_figures.py --skip PID`). The parts, captions and windows are listed in `SEGMENTS` at the top of `tools/final_cut.py`. Change them there and run the script again (it takes about 3 minutes).

### 3.8 Deleted videos

- `videos/20261018-105132/drone_chase_overlay.mp4`: `test4.mp4` with offset 8, out of sync (the data ran 3.9 s late). Replaced by `drone_chase_overlay_test4_synced.mp4`.
- `videos/20261018-105132/drone_chase.mp4` (R3) and `videos/20261018-102023/drone_chase.mp4` (R0): made with the frame-by-frame method (`replay.sh --auto`), which doesn't work. The replay never moved forward, so the picture was static.

## 4. Remaking a video

```bash
# overlay on a recording (offset = the sim time where the recording starts)
.venv/bin/python tools/overlay_video.py 20261018-120332 --video LQI1.mp4 --offset 3 --out videos/20261018-120332/lqi_overlay.mp4

# presentation clips and still images (from the CSV logs only)
.venv/bin/python tools/kalman_view.py "Kalman filter=logs/run_20261018_120347.csv" "No filter (extrapolation)=logs/run_20261018_124456.csv" --start 33 --end 82 --out videos/clips/kalman_vs_extrapolation.mp4
.venv/bin/python tools/dynamics_video.py "PID (kp 1.20 kd 2.20 ki 0.30)=logs/run_20261018_105148.csv" "LQR (kp 1.07 kd 1.58)=logs/run_20261018_120635.csv" "LQI (kp 1.88 kd 2.01 ki 0.58)=logs/run_20261018_120347.csv" --out videos/clips/controller_dynamics.mp4
.venv/bin/python tools/summary_figures.py --out videos/figures      # needs matplotlib in .venv
.venv/bin/python tools/summary_figures.py --skip PID --out videos/figures/no_pid
.venv/bin/python tools/dynamics_video.py "LQR (kp 1.07 kd 1.58)=logs/run_20261018_120635.csv" "LQI (kp 1.88 kd 2.01 ki 0.58)=logs/run_20261018_120347.csv" --title "Why LQR never lands" --out videos/clips/controller_dynamics_lqr_lqi.mp4
.venv/bin/python tools/final_cut.py --out videos/flatbed_falcon_final.mp4   # the final video, from everything above

# side by side
.venv/bin/python tools/side_by_side.py "LQI + Kalman=LQI1.mp4@20261018-120332+3" "LQI, no Kalman=LQI_no_kalman.mp4@20261018-124440+3" --out videos/lqi_kalman_vs_none.mp4

# comparison from the CSV logs only
.venv/bin/python tools/compare_video.py "PID (kp 1.20 kd 2.20 ki 0.30)=logs/run_20261018_105148.csv" "LQR (kp 1.07 kd 1.58)=logs/run_20261018_120635.csv" "LQI (kp 1.88 kd 2.01 ki 0.58)=logs/run_20261018_120347.csv" --out videos/compare_pid_lqr_lqi.mp4
```

To redo a run with exactly the same configuration: set `control_mode` and `use_filter` in `falcon.yaml` as in the table in section 2 (the rest is already as listed in section 1), then `./docker/start-stellar.sh --record` and `./ros2_ws/run.sh`. `falcon.yaml` is now back to `pid` with the filter on.
