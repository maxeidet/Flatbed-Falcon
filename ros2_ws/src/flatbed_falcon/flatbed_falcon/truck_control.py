"""Truck autonomy without ROS: Pure Pursuit (Lab 3), a curvature speed profile with a
PI speed loop, and the truck's side of the landing handshake.

All positions are world ENU (metres), headings in radians from east, counter-clockwise.
"""
import math
from dataclasses import dataclass

import numpy as np


def pure_pursuit_control(dp, theta, L):
    """Steering angle towards the pursuit point (TSFS12 Lab 3, PurePursuitController).

    dp: vector from the rear axle to the pursuit point, theta: heading, L: wheelbase.
    """
    heading = np.array([np.cos(theta), np.sin(theta)])
    cross_track = heading[0] * dp[1] - heading[1] * dp[0]
    return float(np.arctan2(2 * L * cross_track, dp.dot(dp)))


class SpeedProfile:
    """v_ref(s) = min(v_max, sqrt(a_lat / |kappa(s)|)), lowered before each curve so
    the truck can brake down to it at a_brake (a backward pass around the loop)."""

    def __init__(self, road, v_max, a_lat, a_brake, ds=1.0):
        self.road = road
        self.ds = ds
        s = np.arange(0.0, road.length, ds)
        kappa = np.abs(road.curvature(s))
        v = np.full_like(s, v_max)
        curved = kappa > 1e-6
        v[curved] = np.minimum(v_max, np.sqrt(a_lat / kappa[curved]))
        # Twice around the loop so the braking before s = 0 sees the curves after it.
        for _ in range(2):
            for i in range(len(v) - 1, -1, -1):
                v[i] = min(v[i], math.sqrt(v[(i + 1) % len(v)] ** 2 + 2 * a_brake * ds))
        self.v = v

    def __call__(self, s):
        return float(self.v[int(self.road.wrap(s) / self.ds) % len(self.v)])


@dataclass
class TruckParams:
    wheelbase: float = 3.0
    rear_axle_offset: float = 1.5     # odometry point (between the axles) -> rear axle
    steer_limit: float = 0.6
    lookahead_min: float = 3.0        # l = l0 + k v
    lookahead_gain: float = 0.8
    v_max: float = 8.0
    a_lat: float = 2.0
    a_brake: float = 1.5
    a_accel: float = 1.0              # how fast v_ref may rise
    kp_speed: float = 0.5             # PI on the speed error, added to v_ref
    ki_speed: float = 0.2
    trim_limit: float = 1.0           # max |PI correction|, m/s


class TruckController:
    """One step per control tick: pose and speed in, (speed, yaw rate) command out."""

    def __init__(self, road, p: TruckParams):
        self.road = road
        self.p = p
        self.profile = SpeedProfile(road, p.v_max, p.a_lat, p.a_brake)
        self.s = None
        self.v_target = 0.0
        self._integral = 0.0

    def step(self, x, y, yaw, v, dt, v_cap=None):
        """Returns (v_cmd, yaw_rate_cmd, info). v_cap limits the speed (landing)."""
        p = self.p
        rear = np.array([x, y]) - p.rear_axle_offset * np.array([math.cos(yaw), math.sin(yaw)])
        if self.s is None:
            self.s, d = self.road.locate(rear)
        else:
            self.s, d = self.road.project(rear, self.s)

        # Lateral: Pure Pursuit towards the point l metres further along the road.
        lookahead = p.lookahead_min + p.lookahead_gain * max(v, 0.0)
        pursuit = self.road.point(self.s + lookahead)
        delta = pure_pursuit_control(pursuit - rear, yaw, p.wheelbase)
        delta = max(-p.steer_limit, min(p.steer_limit, delta))

        # Longitudinal: speed profile (looked up at the front axle), rate limited,
        # plus a PI trim on the measured speed error.
        v_ref = self.profile(self.s + p.wheelbase)
        if v_cap is not None:
            v_ref = min(v_ref, v_cap)
        if v_ref > self.v_target:
            self.v_target = min(v_ref, self.v_target + p.a_accel * dt)
        else:
            self.v_target = max(v_ref, self.v_target - p.a_brake * dt)
        error = self.v_target - v
        trim = p.kp_speed * error + p.ki_speed * self._integral
        if abs(trim) < p.trim_limit:  # conditional integration = anti-windup
            self._integral += error * dt
        trim = max(-p.trim_limit, min(p.trim_limit, trim))
        v_cmd = max(0.0, self.v_target + trim) if self.v_target > 0.05 else 0.0

        # The Ackermann plugin turns (v, yaw rate) back into a steering angle with
        # delta = atan(L * yaw_rate / v), so send the yaw rate that gives our delta.
        yaw_rate = v_cmd * math.tan(delta) / p.wheelbase
        info = {'s': self.s, 'd': d, 'delta': delta, 'v_ref': v_ref, 'v_target': self.v_target,
                'lookahead': lookahead}
        return v_cmd, yaw_rate, info


@dataclass
class HandshakeParams:
    v_land: float = 4.0         # constant speed the truck holds while the drone lands
    v_tolerance: float = 0.3    # commit only once the speed is this close to v_land
    zone_budget_s: float = 15.0  # commit only with this much landing zone left


class LandingCoordinator:
    """The truck's side of the landing handshake (PLAN.md phase 6).

    The drone sends READY_TO_LAND while it tracks well. From then on the truck caps its
    speed at v_land, and when it is in a landing zone with at least zone_budget_s left
    at that speed it answers LANDING_OK and commits: constant v_land until the zone ends
    or the drone reports LANDED or ABORT. It only commits once it has slowed down to
    v_land: the drone's PD has no feed-forward for braking and would lag a/kp behind.
    """

    def __init__(self, road, p: HandshakeParams):
        self.road = road
        self.p = p
        self.request = 'NONE'
        self.committed = False
        self.landed = False
        self.commit_zone = -1

    def on_request(self, request):
        self.request = request
        if request == 'LANDED':
            self.landed = True
        if request in ('ABORT', 'LANDED', 'NONE'):
            self.committed = False

    def update(self, s, v):
        """s: position on the loop, v: measured speed. Returns (v_cap or None, status dict)."""
        zone, left = self.road.zone_at(s)
        if self.committed and zone != self.commit_zone:
            self.committed = False  # the zone ended under us
        ready = self.request == 'READY_TO_LAND' and not self.landed
        steady = abs(v - self.p.v_land) < self.p.v_tolerance
        if ready and steady and not self.committed and zone >= 0 and left >= self.p.v_land * self.p.zone_budget_s:
            self.committed = True
            self.commit_zone = zone
        v_cap = self.p.v_land if (ready or self.committed) else None
        status = {
            'zone': int(zone),
            'zone_left_m': round(left, 2),
            'zone_left_s': round(left / self.p.v_land, 2) if zone >= 0 else 0.0,
            'landing_ok': self.committed,
            'drone_request': self.request,
        }
        return v_cap, status
