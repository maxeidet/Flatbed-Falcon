# Making videos with `tools/final_cut.py`

`tools/final_cut.py` edits a presentation video together from existing clips, overlays and still images: no video editor needed, and the result can be reproduced exactly. It produced `videos/flatbed_falcon_final.mp4`. This file explains how it works and how to change the video or make a new one.

## 1. What the script does

For each part in the `SEGMENTS` list at the top of the file:

1. **Cards** (title/section cards) are drawn as an image with Pillow: a large title, a green subtitle and a grey body text.
2. **Videos** are cut out (`start` + `length` seconds into the source file), optionally sped up, and scaled to fit the **upper 1920×960** while keeping their aspect ratio. Any empty space becomes the dark background colour.
3. **Still images** (PNG) are shown for `length` seconds, scaled the same way.
4. A **caption bar** (1920×120) at the bottom carries the part's caption. Long text wraps onto two lines. A sped-up part gets a badge ("2×") at the right of the bar.
5. Every part fades in and out over 0.4 s and is encoded identically (H.264, 25 fps, 1920×1080, no sound).
6. All parts are joined in order into one file (`ffmpeg -f concat`, without re-encoding).

The temporary files live in a temp folder that is removed afterwards. It takes about 3 minutes for the 3-minute video.

```text
┌──────────────────────────────────────────┐
│                                          │
│     the clip or image, 1920×960,          │
│     aspect ratio kept, centred           │
│                                          │
├──────────────────────────────────────────┤
│  caption (max 2 lines)            [2×]   │  1920×120
└──────────────────────────────────────────┘
```

## 2. Requirements

- **ffmpeg** on the computer (the Stellar computers have it).
- **Pillow** in the project's venv: `python3 -m venv .venv && .venv/bin/pip install pillow`.
- **The source files** have to exist. They're made with the other tools (see [VIDEOS.md](VIDEOS.md), section 4). The script stops with `Missing <file>` if one is missing.

Run it from the repo root:

```bash
.venv/bin/python tools/final_cut.py                          # -> videos/flatbed_falcon_final.mp4
.venv/bin/python tools/final_cut.py --out videos/test_cut.mp4  # another output file
```

It prints one line per part with its length, and the total length at the end:

```text
   2   24.0 s  lqi_overlay.mp4
   ...
Wrote videos/flatbed_falcon_final.mp4 (186 s, 3:06)
```

## 3. The `SEGMENTS` list

Every part is a tuple of six fields:

```python
(kind, source, start, length, speed, caption)
```

| Field | Meaning |
|---|---|
| `kind` | `'card'`, `'video'` or `'still'` |
| `source` | For `card`: a tuple `(title, subtitle, body)`, where the body may contain `\n` for line breaks. For `video`/`still`: a path relative to the repo root |
| `start` | Seconds into the source video to start at. Ignored for `card` and `still` (set 0) |
| `length` | How many seconds **of the source** to use. For `card` and `still`, the time on screen |
| `speed` | 1 = normal, 2 = double speed (time on screen = `length / speed`). A badge appears when it isn't 1 |
| `caption` | The text in the caption bar, or `None` for an empty bar. Cards have no caption bar |

Examples from the current list:

```python
# section card, 6 s
('card', ('2 · LQR vs LQI', 'Same truck, same radio link, same Kalman filter',
          'Both get their gains from the same Riccati equation.\n'
          'LQI adds one state: the integral of the position error.'), 0, 6, 1, None),

# 28.6 s of a video, from 52 s in
('video', 'videos/lqi_vs_lqr.mp4', 52, 28.6, 1,
 'LQI (left) passes the gate and lands. LQR (right) hangs ~0.9 m behind the bed: ...'),

# 53 s of a video at double speed -> 26.5 s on screen
('video', 'videos/clips/controller_dynamics_lqr_lqi.mp4', 0, 53, 2, 'Rotor drag pushes ... 2× speed'),

# still image, 7 s
('still', 'videos/figures/no_pid/height.png', 0, 7, 1, 'Height while tracking: ...'),
```

## 4. Picking the right window in a video

The overlays and side-by-sides start at a known **sim time** (the offset in VIDEOS.md). To show a particular moment, convert sim time to "seconds into the video":

```text
start = sim time you want to start at − the sim time the video starts at
```

| Source | Video time 0 is | Example |
|---|---|---|
| `lqi_overlay.mp4`, `lqr_overlay.mp4`, `lqi_no_kalman_overlay.mp4`, `lqi_vs_lqr.mp4`, `lqi_kalman_vs_none.mp4` | sim time 3.0 s | from sim 59 s: `start = 56` |
| `drone_chase_overlay_test4_synced.mp4` | sim time 11.92 s | from sim 70 s: `start = 58.08` |
| `drone_chase_overlay_test5.mp4` | sim time 13.4 s | |
| `videos/clips/kalman_vs_extrapolation.mp4` | sim time 33 s | from sim 50 s: `start = 17` |
| `videos/clips/controller_dynamics*.mp4`, `videos/compare_pid_lqr_lqi.mp4` | **0 s after TRACK** (not sim time) | LQI's TRACK starts at sim 33.4 s |

Useful moments in the LQI run (R4), in sim time:

| Event | Sim time |
|---|---|
| TRACK starts | 33.40 s |
| READY_TO_LAND (the gate held) | 63.30 s |
| The truck gives LANDING_OK | 65.80 s |
| DESCEND | 70.95 s |
| TOUCHDOWN / LANDED | 78.60 / 78.85 s |

Other runs: list the state changes from the CSV log, or look at the clip.

**Check a window before rendering.** Pull out one frame at a given point in the source:

```bash
ffmpeg -v error -ss 75.6 -i videos/20261018-120332/lqi_overlay.mp4 -frames:v 1 /tmp/check.png
```

## 5. Common changes

**Change a caption or card text:** edit the string in `SEGMENTS` and run the script again.

**Shorten or move a part:** change `start` and/or `length`. Make sure `start + length` doesn't run past the end of the source (check its length with `ffprobe -v error -show_entries format=duration -of csv=p=0 <file>`).

**Speed up a slow part:** set `speed` to 2 (or 1.5). The badge appears by itself. It's good to mention it in the caption too.

**Reorder or remove parts:** move or delete lines in the list.

**Add a new part:** for example, the PID comparison from the CSV logs:

```python
('video', 'videos/compare_pid_lqr_lqi.mp4', 0, 64, 2, 'PID, LQR and LQI aligned on TRACK. 2× speed'),
```

**Make a separate, short version** (e.g. for a 1-minute pitch): copy `final_cut.py`, keep the important lines in `SEGMENTS`, and run it with another `--out`. Or keep one list and comment out lines.

**Change the timing between the narration and the video:** the times in [PRESENTATION_SCRIPT.md](PRESENTATION_SCRIPT.md) follow the current list. If you change lengths, shift the times there too. The script's own output lists each part's length, so you can add them up from there.

## 6. Settings at the top of the script

| Name | Value | |
|---|---|---|
| `W, H` | 1920, 1080 | the video's size |
| `AREA` | 960 | height of the image area; the rest is the caption bar |
| `FPS` | 25 | frame rate; every source is converted to it |
| `FADE` | 0.4 s | fade in/out per part |
| `BG`, `TEXT`, `MUTED`, `ACCENT` | colours | the same dark style as the other videos |
| `FONT`, `BOLD` | DejaVu Sans | must exist on the computer (`/usr/share/fonts/truetype/dejavu/`) |

## 7. From a new recording to the final video

The whole chain, e.g. for a new run:

1. **Run** with `--record`: `cd docker && ./start-stellar.sh --record`, then `./ros2_ws/run.sh`. Note the Gazebo log's name (`logs/gz/<log>`).
2. **Replay and record** in Gazebo: `./docker/replay.sh <log> -a drone_chase`. Start recording while it's paused. The terminal prints the sim time (the offset).
3. **Add the overlay:** `.venv/bin/python tools/overlay_video.py <log> --video rec.mp4 --offset <N> --out videos/<log>/overlay.mp4`.
4. **Optionally, side by side** with another run: `tools/side_by_side.py "A=a.mp4@<log A>+N" "B=b.mp4@<log B>+N" --out videos/a_vs_b.mp4`.
5. **Add a line** to `SEGMENTS` (with `start` calculated as in section 4) and run `tools/final_cut.py`.

## 8. Troubleshooting

| Problem | Cause / fix |
|---|---|
| `Missing videos/...` | The source file doesn't exist. Make it with the matching tool (VIDEOS.md, section 4) |
| A part is shorter than expected | `start + length` runs past the end of the source. Check with `ffprobe` |
| The caption is cut off | It wraps onto at most two lines and is centred. With very long text, shorten it |
| The text in an image is too small | The source is scaled to fit 1920×960. A narrow or small source (e.g. a recording from a small Gazebo window) gets small text, so record with a larger window |
| `ffmpeg` error on concat | Every part is encoded the same way, so it shouldn't happen. If you've changed `FPS` or the size for a single part, all parts have to match |
