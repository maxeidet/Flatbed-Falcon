# Running on Stellar (LiU lab computers)

How we run the simulation on the Stellar computers: Ubuntu, each user gets their own rootless Docker daemon, we sit at the machine (course students have no remote access). Based on the [Stellar rules](https://gitlab.liu.se/stellar-support/rules) and the rootless Docker guide in the [Stellar guides](https://gitlab.liu.se/stellar-support/guides), last read 2026-09-25. The rules change and keeping up is our responsibility, so re-read them now and then.

The script for all of this is `docker/start-stellar.sh`. It only works on Linux. On a Mac use `start-dev.sh` (see `README.md` in this folder).

## 1. Rules that matter for us

- **No root, nothing to install.** Docker is already set up per user. Never run `dockerd-rootless-setuptool.sh` (out of date, it's done for us).
- **Course students:** no remote access, use the rooms only when we need a GPU, normally one computer per group, leave at once when someone with higher priority needs it, no reservation notes.
- **Don't hog.** Use the computer our work needs, not the best free one. `--gpu` gives the container **all** GPUs in the computer, so check `nvidia-smi` first that nobody is using it.
- **Nothing reachable from outside the university network.** No reverse tunnels, ngrok or Tailscale. Our script binds ports to `127.0.0.1` only. Never change that to `0.0.0.0`.
- **Shared disk, no backup.** Images live in `~/.local/share/docker` on a disk shared with everyone on that computer (1-2 TB, the login screen shows how much is free). Files stay on the computer they were made on and are never backed up, so anything that matters goes to Git. Disks that fill up get cleared by stellar-support, and people who repeatedly fill them can get a notice. If we ever need to keep a lot of data, write to stellar-support before the disk fills. Clean up after every session (section 6), and when the project ends move off or delete what we no longer use.
- **Log out when leaving.** Never switch a computer off. The last one out of the room arms the alarm.

## 2. First time on a computer

Each computer has its own Docker daemon and its own images, so do this once per computer.

Check that Docker works. The first time it may say it is setting up your daemon, wait a few seconds:
```bash
docker run --rm hello-world
```

Check the CPU architecture (we expect `x86_64`):
```bash
uname -m
```

Check the GPU and who is using it:
```bash
nvidia-smi
```

Clone the repo:
```bash
git clone https://github.com/maxeidet/Flatbed-Falcon.git
```

## 3. Start a session

Get the latest code:
```bash
cd ~/flatbed-falcon && git pull
```

Start the simulation (default is `--native-gui -w hive_base`, drone `gz_x500`):
```bash
cd ~/flatbed-falcon/docker && ./start-stellar.sh
```

What happens:

1. The script checks that Docker is reachable and builds the image. **The first time on a computer it downloads the multi-GB base image, which takes several minutes.**
2. It starts the container in the background (DDS agent, repo mounted at `/workspace`).
3. It runs PX4 + Gazebo in **this** terminal. You get the `pxh>` prompt, and the Gazebo window opens on the computer's own screen.

`hive_base` is a military tent camp inside a dense forest. Its tree meshes (from the CC0 Fuel models `Pine Tree` and `Oak tree`) are stored in `worlds/trees/` and read from `/workspace`, so no internet is needed. Regenerate it with `python3 worlds/generate_hive_base.py`.

### Flags

| Flag | What it does |
|---|---|
| *(none)* | Same as `--native-gui -w hive_base`, drone `gz_x500` |
| `-m MODEL` | Drone variant, e.g. `gz_x500_depth` (all values in `README.md`) |
| `-w WORLD` | World, e.g. `baylands`, `forest`, `default` (all values in `README.md`) |
| `--native-gui` | Gazebo window on this computer's own screen (default) |
| `--vnc` | GUI in a browser on this computer instead: `http://localhost:6080/vnc.html`, with a random password printed by the script |
| `--headless` | No GUI at all |
| `--gpu` | Give the container the GPU(s). Prints `nvidia-smi` first |
| `-h` | Show usage |

The last of `--native-gui`, `--vnc` and `--headless` wins. If another user already holds the VNC ports, override them:
```bash
HOST_NOVNC_PORT=6081 HOST_VNC_PORT=5901 ./start-stellar.sh --vnc
```

## 4. Run the backend

Open a second terminal (Ctrl+Alt+T):
```bash
cd ~/flatbed-falcon && ./ros2_ws/run.sh
```

It builds the ROS 2 workspace inside the container and starts the backend, which flies the drone to 5 m above its start point and hovers there. In the log you should see, in this order:

1. `PX4 pre-flight checks now PASS` (the backend waits for this, and prints a warning every few seconds while it is still failing. This can take 10-20 seconds after PX4 starts.)
2. `PX4 accepted the offboard mode command` and `PX4 accepted the arm command`
3. `PX4 is now ARMED` and `PX4 is now in OFFBOARD mode`
4. `position x=... altitude=...` once per second, then `Reached 5.0 m, hovering`

If PX4 refuses a command the backend keeps retrying once per second and prints `PX4 denied the ... command` (the reason is in the PX4 terminal). **Stopping the backend (Ctrl+C) makes PX4 land the drone**, because it treats the lost offboard stream as a failsafe.

## 5. Looking at the world, and the GPU

- **Default (`--native-gui`):** the window opens on the screen we're sitting at. If no window appears or it errors, use `--vnc` instead.
- **GPU:** `./start-stellar.sh --gpu` passes `--device nvidia.com/gpu=all` to the container. That alone does not make Gazebo render on the GPU. The VNC display is software-rendered, so only `--native-gui --gpu` can use it. To check, run `nvidia-smi` in another terminal while the GUI is up: `gz` should be listed as a graphics (G) process. If it isn't, rendering is still on the CPU.
- **Sensors:** camera and lidar sensors render inside the simulation, so they only benefit from the GPU in `--native-gui` mode.
- **ML training** is not covered yet: the image has no PyTorch or CUDA.

## 6. Clean up after a session

Do this every time. The disk is shared with everyone on the computer and disks that fill up get cleared.

### A. Stop everything

1. Backend terminal: `Ctrl+C`.
2. PX4 terminal: `Ctrl+C`, or type `shutdown` at `pxh>`. The script prints `Stopping container...` and removes the container itself.
3. Close any leftover windows.

Check that nothing of ours is still running:
```bash
docker ps
```
The list should be empty. If `flatbed-falcon-sim` is still there:
```bash
docker rm -f flatbed-falcon-sim
```
If we used the GPU, check that no `gz` or `px4` process of ours is left in the process table:
```bash
nvidia-smi
```

### B. Free disk (quick, every session)

See what Docker takes:
```bash
docker system df
```

Remove stopped containers, unused networks, dangling images and build cache. It keeps our images (`flatbed-falcon-px4`, `px4io/px4-sitl-gazebo-ros2`), so the next start is fast:
```bash
docker system prune
```

### C. When we won't be back soon (end of a work block, end of a phase)

Remove the multi-GB images too. The Stellar cleaning guide says images we remove come back quickly from the lab's own cache when we need them again, so this is cheap. Our Docker daemon is our own, so this only affects our own images:
```bash
docker system prune -a
```

Check the result (the guide says Docker's data lives here):
```bash
du -sh ~/.local/share/docker
```

### D. Find what else fills our home directory

Old checkpoints, logs and caches grow unnoticed. To see what takes space (in `ncdu`, `d` deletes and `q` quits):
```bash
cd ~ && ncdu
```

The same without `ncdu`:
```bash
du -h -d1 ~ | sort -h
```

Caches such as pip and Hugging Face:
```bash
du -sh ~/.cache/*
```

### E. Files in our clone

ROS 2 build output from the backend is regenerated on the next build and is not in Git. Remove it:
```bash
cd ~/flatbed-falcon && rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
```
Under rootless Docker these files belong to our own user, so no root is needed.

### F. Keep what matters

Nothing on Stellar is backed up. Check for changes made on this computer and push what's worth keeping:
```bash
cd ~/flatbed-falcon && git status
```

PX4 flight logs (`.ulg`) live inside the container and disappear when it stops. To keep one, copy it out **before** stopping (the path comes from PX4's startup output):
```bash
docker cp flatbed-falcon-sim:/root/.local/share/px4/rootfs/0/fs/log ./px4-logs
```

### G. Leave the computer

Log out. Don't shut it down, and only restart it if nobody else is using it. If we're the last ones out of the room, arm the alarm. When the whole project ends, delete the clone too.

## 7. Troubleshooting

- **`Cannot connect to the Docker daemon`:** run `systemctl --user start docker`, or log out and in again. If the setup failed: `journalctl -t stellar-rootless-docker`.
- **Build fails while pulling:** the computer may not reach Docker Hub or the internet. Check with `docker run --rm hello-world`.
- **`port is already allocated`** (with `--vnc`): another user holds the port. Use the `HOST_NOVNC_PORT` / `HOST_VNC_PORT` overrides in section 3.
- **`--native-gui ... DISPLAY is not set`:** we're not in a graphical session. Use `--vnc` or `--headless`.
- **No Gazebo window with `--native-gui`:** use `--vnc`, and tell the others so we can fix it.
- **Stuck on `Waiting for Gazebo world...`:** `baylands` is slow (30-60 s or more). Try `-w default` to tell which.
- **The backend keeps printing `pre-flight checks are failing`:** read the `Preflight Fail` lines in the PX4 terminal. `no valid barometer data` is normal for the first seconds after PX4 starts. Anything that stays is a real problem, paste it to us.

## 8. Not verified yet

Checked on a Stellar computer so far: Docker works, `x86_64`, an NVIDIA GPU (RTX 3090 Ti) is visible, the image builds, PX4 + Gazebo start, and the backend arms the drone without QGroundControl and it hovers at 5 m.

Checked only on a Mac: the image build, the container starting, VNC password login, and the script's flag handling.

Not checked anywhere yet, so update this list as we learn:

- `--native-gui`: whether the window opens through the container's X11 access.
- `--gpu`: whether `--device nvidia.com/gpu=all` works for us, and whether Gazebo really renders on the GPU.
- **QGroundControl:** the image sends MAVLink to `192.168.65.254`, which is Docker Desktop for macOS's address and doesn't exist on Linux, so QGC on a Stellar computer will not auto-connect. We don't need it for flying: the image now disables PX4's "return home on ground-station link loss" setting (`NAV_DLL_ACT=0`, simulation only), and with no QGC running the backend armed and hovered at 5 m, verified on a Mac and on a Stellar computer. You get the change by running `start-stellar.sh`, which rebuilds the image (`git pull` first).
