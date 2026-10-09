"""Offline checks of the ROS-free modules: no Gazebo, no PX4.

Run inside the sim container (it has numpy, scipy and pytest):
    cd /workspace/ros2_ws/src/flatbed_falcon && python3 -m pytest -q test
"""
import math
from pathlib import Path

import numpy as np
import pytest

from flatbed_falcon import frames
from flatbed_falcon import landing_logic as logic
from flatbed_falcon.road import RoadLoop
from flatbed_falcon.target_filter import LinkParams, Measurement, SimulatedLink, TargetEstimator
from flatbed_falcon.truck_control import (HandshakeParams, LandingCoordinator, TruckController,
                                          TruckParams)

CSV = Path(__file__).resolve().parents[4] / 'worlds' / 'road_centerline.csv'
BED_OFFSET = (-1.15, 0.0, 1.0)


@pytest.fixture(scope='module')
def road():
    return RoadLoop(CSV)


def test_road_loop(road):
    assert road.length == pytest.approx(1129, abs=2)
    assert sorted(road.zones) == [0, 1, 2]
    for s in (0.0, 5.0, 300.0, road.length - 3.0):
        p = road.point(s)
        s_back, d = road.project(p + 0.5 * np.array([-road.heading(s)[1], road.heading(s)[0]]), s + 2.0)
        assert road.distance_ahead(s, s_back) == pytest.approx(0.0, abs=0.05) or \
            road.distance_ahead(s_back, s) == pytest.approx(0.0, abs=0.05)
        assert d == pytest.approx(0.5, abs=0.05)  # left of the road is positive
    # Across the seam the road continues smoothly
    assert np.linalg.norm(road.point(road.length - 0.5) - road.point(0.5)) == pytest.approx(1.0, abs=0.05)
    zid, left = road.zone_at(road.zones[1][0] + 10.0)
    assert zid == 1 and left == pytest.approx(road.zones[1][1] - road.zones[1][0] - 10.0)


def test_body_point_velocity():
    # Truck heading north (ENU yaw 90 deg) at 5 m/s turning left at 0.2 rad/s: the bed
    # 1.15 m behind the centre moves at 5 m/s north plus 0.23 m/s towards east.
    p, v = frames.body_point_enu(0, 0, 0, math.pi / 2, 0, 5, 0, 0.2, BED_OFFSET)
    assert p == pytest.approx((0, -1.15, 1.0))
    assert v == pytest.approx((0.23, 5, 0))


class Bicycle:
    """Kinematic bicycle (rear-axle reference) with a first-order steering and speed
    response, standing in for the Gazebo truck. Reports the odometry point (between the axles)."""

    def __init__(self, road, p: TruckParams, s0):
        self.p = p
        rear = road.point(s0) - 0.0
        h = road.heading(s0)
        self.yaw = math.atan2(h[1], h[0])
        self.rear = rear
        self.v = 0.0
        self.delta = 0.0

    @property
    def center(self):
        return self.rear + self.p.rear_axle_offset * np.array([math.cos(self.yaw), math.sin(self.yaw)])

    def step(self, v_cmd, yaw_rate_cmd, dt):
        delta_cmd = math.atan(self.p.wheelbase * yaw_rate_cmd / v_cmd) if v_cmd > 0.01 else 0.0
        self.delta += (delta_cmd - self.delta) * min(1.0, dt / 0.15)
        self.v += max(-4 * dt, min(2.5 * dt, v_cmd - self.v))
        self.yaw += self.v * math.tan(self.delta) / self.p.wheelbase * dt
        self.rear = self.rear + self.v * dt * np.array([math.cos(self.yaw), math.sin(self.yaw)])

    @property
    def yaw_rate(self):
        return self.v * math.tan(self.delta) / self.p.wheelbase


def test_truck_laps(road):
    p = TruckParams()
    ctrl = TruckController(road, p)
    truck = Bicycle(road, p, road.zones[1][0])
    dt, err_straight, err_curve, driven = 0.05, [], [], 0.0
    while driven < 2 * road.length:
        x, y = truck.center
        v_cmd, w_cmd, info = ctrl.step(x, y, truck.yaw, truck.v, dt)
        truck.step(v_cmd, w_cmd, dt)
        driven += truck.v * dt
        if driven > 30:  # after the start-up
            (err_curve if abs(road.curvature(info['s'])) > 0 else err_straight).append(abs(info['d']))
    assert max(err_straight) < 0.5
    assert max(err_curve) < 1.0


class PX4Like:
    """Point-mass drone that follows a TrajectorySetpoint roughly like PX4's position
    controller: P on position -> velocity, PI on velocity -> acceleration, NaN = not
    controlled on that axis, plus a 0.1 s lag on the acceleration. Gains are PX4's
    defaults (MPC_XY_P, MPC_XY_VEL_P_ACC, MPC_XY_VEL_I_ACC, MPC_Z_*)."""

    KP = np.array([0.95, 0.95, 1.0])
    KV = np.array([1.8, 1.8, 4.0])
    KI = np.array([0.4, 0.4, 2.0])

    def __init__(self, drag=0.0):
        self.drag = drag  # linear horizontal drag (1/s), like the x500's rotor drag in Gazebo
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.v_int = np.zeros(3)
        self.armed = self.offboard = False

    def step(self, sp: logic.Setpoint, dt):
        pos, vel, acc = (np.array(x, dtype=float) for x in (sp.position, sp.velocity, sp.acceleration))
        v_sp = np.where(np.isnan(vel), 0.0, vel) + np.where(np.isnan(pos), 0.0, self.KP * (pos - self.p))
        has_v = ~np.isnan(pos) | ~np.isnan(vel)
        a_sp = np.where(np.isnan(acc), 0.0, acc) + np.where(has_v, self.KV * (v_sp - self.v) + self.v_int, 0.0)
        saturated = np.linalg.norm(a_sp[:2]) > 5.0
        a_sp[:2] = logic.clip_norm(a_sp[:2], 5.0)
        if self.armed and not saturated:  # anti-windup, like PX4
            self.v_int += np.where(has_v, self.KI * (v_sp - self.v), 0.0) * dt
            self.v_int[:2] = logic.clip_norm(self.v_int[:2], 1.5)
        if not self.armed:
            a_sp[:] = 0.0
        self.a += (a_sp - self.a) * min(1.0, dt / 0.1)
        self.v += (self.a - np.array([self.drag, self.drag, 0.0]) * self.v) * dt
        self.p += self.v * dt
        if self.p[2] > 0 and not self.armed:
            self.p[2] = 0.0


def run_landing(road, control: logic.ControlParams, link=LinkParams(), use_filter=False, t_max=400.0,
                drag=0.0):
    tp = TruckParams()
    truck_ctrl = TruckController(road, tp)
    coordinator = LandingCoordinator(road, HandshakeParams())
    truck = Bicycle(road, tp, road.zones[1][0])
    machine = logic.LandingStateMachine(control, logic.LandingParams())
    drone = PX4Like(drag)
    radio = SimulatedLink(link)
    estimator = TargetEstimator(use_filter, 1.0, max(link.pos_noise, 0.02), max(link.vel_noise, 0.02))
    dt, t, sp = 0.01, 0.0, logic.Setpoint(position=[0.0, 0.0, 0.0])
    request, landed_at, states, zone_at_landing = 'NONE', None, [], None
    while t < t_max:
        k = round(t / dt)
        if k % 5 == 0:  # truck at 20 Hz, odometry/target at 20 Hz here
            if truck_ctrl.s is not None:
                coordinator.on_request(request)
            v_cap, status = coordinator.update(truck_ctrl.s, truck.v) if truck_ctrl.s is not None else (None, {})
            v_cmd, w_cmd, _ = truck_ctrl.step(*truck.center, truck.yaw, truck.v, 0.05, v_cap)
            c, s = math.cos(truck.yaw), math.sin(truck.yaw)
            pos, vel = frames.body_point_enu(*truck.center, 0.0, truck.yaw, truck.v * c, truck.v * s, 0.0,
                                             truck.yaw_rate, BED_OFFSET)
            radio.send(Measurement(t, np.array(frames.world_enu_to_px4_local(*pos, (0, 0, 0))),
                                   np.array(frames.enu_to_ned(*vel)), frames.yaw_enu_to_ned(truck.yaw),
                                   -truck.yaw_rate))
            for m in radio.receive(t):
                estimator.update(m)
            truck_status = logic.TruckStatus(status.get('landing_ok', False), status.get('zone', -1),
                                             status.get('zone_left_s', 0.0), 0.0)
            target = None
            if estimator.age(t) < 1.0:
                tp_, tv, tyaw, trate = estimator.estimate(t)
                target = logic.Target(tp_, tv, tyaw, trate, 0.0)
            ds = logic.DroneState(drone.p.copy(), drone.v.copy(), 0.0, drone.armed, drone.offboard)
            out = machine.step(t, ds, target, truck_status, bed_contact=False)
            if out.want_armed and out.want_offboard:
                drone.armed = drone.offboard = True
            if out.force_disarm:
                drone.armed = False
            sp, request = out.setpoint, out.request
            if not states or states[-1] != out.info['state']:
                states.append(out.info['state'])
            if machine.state == logic.LANDED:
                landed_at = out.info.get('touchdown_e_xy', landed_at)
                zone_at_landing = status.get('zone')
                break
        drone.step(sp, dt)
        truck.step(v_cmd, w_cmd, dt)
        t += dt
    return states, landed_at, zone_at_landing, t


@pytest.mark.parametrize('mode', ['pd', 'pid', 'lqr', 'px4'])
def test_landing_ground_truth(road, mode):
    states, e_touchdown, zone, t = run_landing(road, logic.ControlParams(mode=mode))
    assert states[:4] == ['IDLE', 'TAKEOFF', 'APPROACH', 'TRACK'], states
    assert states[-1] == 'LANDED', states
    assert e_touchdown < 0.3
    assert zone >= 0


# Gazebo's x500 needed ~0.65 m/s^2 forward at 4 m/s to hold the truck's speed.
X500_DRAG = 0.16


@pytest.mark.parametrize('mode', ['pid', 'px4'])
def test_landing_with_drag(road, mode):
    states, e_touchdown, zone, t = run_landing(road, logic.ControlParams(mode=mode), drag=X500_DRAG)
    assert states[-1] == 'LANDED', states
    assert e_touchdown < 0.3


def test_pd_lags_behind_with_drag(road):
    """A PD holds the drag with a steady error drag * v / kp > the 0.3 m gate: never lands."""
    states, _, _, _ = run_landing(road, logic.ControlParams(mode='pd'), drag=X500_DRAG, t_max=150)
    assert 'DESCEND' not in states


def test_landing_noisy_link_with_filter(road):
    link = LinkParams(pos_noise=0.1, vel_noise=0.1, latency=0.1, dropout=0.1, seed=1)
    states, e_touchdown, zone, t = run_landing(road, logic.ControlParams(), link, use_filter=True,
                                               drag=X500_DRAG)
    assert states[-1] == 'LANDED', states
    assert e_touchdown < 0.4


def test_gimbal_angles():
    # Drone heading north (yaw 0), 4 m above a target 4 m to the east: 45 deg down, 90 deg right.
    pitch, yaw = logic.gimbal_angles([0, 0, -5], 0.0, [0, 4, -1])
    assert (pitch, yaw) == pytest.approx((-45.0, 90.0))
    # Heading east, target straight ahead and below.
    assert logic.gimbal_angles([0, 0, -5], math.pi / 2, [0, 10, -5]) == pytest.approx((0.0, 0.0))
    # Straight above: nadir, yaw stays on the nose.
    assert logic.gimbal_angles([1, 1, -5], 1.0, [1, 1, -1]) == pytest.approx((-90.0, 0.0))
    # A target above the drone is clamped to level.
    assert logic.gimbal_angles([0, 0, -1], 0.0, [5, 0, -5])[0] == 0.0
