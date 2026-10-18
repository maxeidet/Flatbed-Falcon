"""Landing-target estimation without ROS: a simulated radio link and a
constant-velocity Kalman filter that predicts the bed position ahead in time.

Everything is in PX4 local NED. A measurement is (t, position[3], velocity[3], yaw,
yaw_rate) where t is when the truck measured it.
"""
import collections
import math
import random
from dataclasses import dataclass

import numpy as np


@dataclass
class Measurement:
    t: float
    position: np.ndarray
    velocity: np.ndarray
    yaw: float
    yaw_rate: float


@dataclass
class LinkParams:
    pos_noise: float = 0.0     # std, metres (horizontal and vertical)
    vel_noise: float = 0.0     # std, m/s
    latency: float = 0.0       # seconds until a measurement arrives
    dropout: float = 0.0       # probability that a measurement is lost
    seed: int = 0


class SimulatedLink:
    """Turns ground truth into what the drone would receive over a radio link."""

    def __init__(self, p: LinkParams):
        self.p = p
        self._rng = random.Random(p.seed)
        self._queue = collections.deque()

    def send(self, m: Measurement):
        if self._rng.random() < self.p.dropout:
            return
        noisy = Measurement(
            m.t,
            m.position + np.array([self._rng.gauss(0, self.p.pos_noise) for _ in range(3)]),
            m.velocity + np.array([self._rng.gauss(0, self.p.vel_noise) for _ in range(3)]),
            m.yaw, m.yaw_rate)
        self._queue.append(noisy)

    def receive(self, now):
        """All measurements whose latency has passed by `now`, oldest first."""
        out = []
        while self._queue and self._queue[0].t + self.p.latency <= now:
            out.append(self._queue.popleft())
        return out


class TargetEstimator:
    """Constant-velocity Kalman filter in 3D (state x, y, z, vx, vy, vz) fed with delayed
    position and velocity measurements. The yaw is taken from the latest measurement.
    The height is filtered too: the drone's height ramp turns raw height noise into
    vertical velocity commands, and that kept it outside the descent gate.

    With filter=False it just extrapolates the latest measurement with its velocity:
    the plain latency compensation to compare the filter against.
    """

    def __init__(self, use_filter=True, accel_std=1.0, pos_std=0.1, vel_std=0.1):
        self.use_filter = use_filter
        self.q = accel_std ** 2
        self.R = np.diag([pos_std ** 2] * 3 + [vel_std ** 2] * 3)
        self.x = None
        self.P = None
        self.t = None
        self.last = None  # latest Measurement

    def _predict(self, x, P, dt):
        I = np.eye(3)
        F = np.block([[I, dt * I], [np.zeros((3, 3)), I]])
        G = np.vstack((dt * dt / 2 * I, dt * I))
        return F @ x, F @ P @ F.T + self.q * G @ G.T

    def update(self, m: Measurement):
        if self.last is not None and m.t <= self.last.t:
            return  # out of order or duplicate
        self.last = m
        z = np.hstack((m.position, m.velocity))
        if self.x is None or not self.use_filter:
            self.x = z.copy()
            self.P = self.R.copy()
            self.t = m.t
            return
        x, P = self._predict(self.x, self.P, m.t - self.t)
        S = P + self.R
        K = P @ np.linalg.inv(S)
        self.x = x + K @ (z - x)
        self.P = (np.eye(6) - K) @ P
        self.t = m.t

    def age(self, now):
        return math.inf if self.last is None else now - self.last.t

    def estimate(self, t_pred):
        """(position, velocity, yaw, yaw_rate) predicted to time t_pred."""
        dt = t_pred - self.t
        if self.use_filter:
            x, _ = self._predict(self.x, self.P, dt)
        else:
            x = self.x.copy()
            x[:3] += x[3:] * dt
        position, velocity = x[:3], x[3:]
        yaw = self.last.yaw + self.last.yaw_rate * (t_pred - self.last.t)
        return position, velocity, yaw, self.last.yaw_rate
