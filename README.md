# Flatbed Falcon

![Flatbed Falcon](assets/flatbed-falcon.jpg)

A drone (PX4, x500) that autonomously tracks and lands on the flatbed of a moving pickup truck in Gazebo. Project for the Autonomous Vehicles course.

The truck drives a perimeter with Pure Pursuit, shares its position over ROS 2, and the drone follows it with PX4 Offboard velocity setpoints before descending onto the flatbed.

## Running

```bash
cd docker && ./start-dev.sh        # macOS: builds the image, starts PX4 + Gazebo (hive_base world)
./ros2_ws/run.sh                   # second terminal: builds and launches all nodes
```

On the Stellar computers use `docker/start-stellar.sh` instead (see [docker/README-stellar.md](docker/README-stellar.md)).

## Layout

| Path | Contents |
|---|---|
| `docker/` | Simulation container (PX4 + Gazebo + ROS 2 Jazzy, pinned image) |
| `worlds/` | Gazebo world (`hive_base.sdf`) and tree models |
| `ros2_ws/src/flatbed_falcon/` | ROS 2 package with all nodes |

| Node | Role |
|---|---|
| `truck_driver` | Pure Pursuit around the perimeter |
| `target_tracker` | Truck pose (world ENU) → landing target (PX4 local NED) |
| `landing_controller` | PID tracking + landing state machine → PX4 Offboard |
| `logger` | CSV logs for plots |

All frame conversions (ENU ↔ NED, spawn offset) live in `flatbed_falcon/frames.py`.

The simulation environment and world are reused from our Hive project (Projectcourse in AI); everything under `ros2_ws/` is new for this course.
