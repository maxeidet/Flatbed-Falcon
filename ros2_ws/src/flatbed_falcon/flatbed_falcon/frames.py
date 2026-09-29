"""Coordinate frame conversions shared by every node.

Gazebo and ROS use ENU (x=East, y=North, z=Up, yaw 0 = East, counter-clockwise).
PX4 uses NED (x=North, y=East, z=Down, yaw 0 = North, clockwise), relative to
where the drone spawned. Convert here and nowhere else.
"""
import math


def enu_to_ned(x, y, z):
    """Position or velocity from ENU to NED. The same swap works both ways."""
    return y, x, -z


def ned_to_enu(x, y, z):
    return y, x, -z


def wrap_pi(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_enu_to_ned(yaw):
    return wrap_pi(math.pi / 2.0 - yaw)


def yaw_ned_to_enu(yaw):
    return wrap_pi(math.pi / 2.0 - yaw)


def world_enu_to_px4_local(x, y, z, spawn_enu):
    """Gazebo world ENU position -> PX4 local NED position.

    PX4's local origin is the drone's spawn point (PX4_GZ_MODEL_POSE), not the
    world origin, so subtract it before swapping axes.
    """
    sx, sy, sz = spawn_enu
    return enu_to_ned(x - sx, y - sy, z - sz)
