"""Tracking control law and landing state machine, without ROS.

Everything is PX4 local NED: x north, y east, z down, yaw from north, clockwise.

Horizontal tracking (TSFS12 Lab 4, double integrator with a PD law):
    a = a_truck + kp (p_bed - p) + kd (v_truck - v) [+ ki * integral of (p_bed - p)]
It is computed as a saturated cascade that equals the PD for small errors:
    v_ref = v_truck + clip(kp/kd * e_p, v_rel_max),   a = a_truck + kd (v_ref - v)
so a far-away truck (APPROACH) gives a speed-limited chase instead of a huge
acceleration. PX4 gets the result as an acceleration setpoint for x and y.
mode 'pid' adds the integral term (the default): the x500's rotor drag in Gazebo is a
constant force at constant speed, and a PD can only hold it with a constant error
(drag / kp, about 0.5 m behind the bed at 4 m/s). mode 'lqr' takes kp, kd from a
discrete LQR on the same double integrator, and mode 'px4' sends position + velocity
feed-forward and lets PX4's own loops (which have an integrator) do the work.

The vertical axis is a separate height profile driven by the state machine.
"""
import math
from dataclasses import dataclass, field

import numpy as np

NAN = float('nan')

IDLE, TAKEOFF, APPROACH, TRACK, DESCEND, TOUCHDOWN, LANDED, ABORT = (
    'IDLE', 'TAKEOFF', 'APPROACH', 'TRACK', 'DESCEND', 'TOUCHDOWN', 'LANDED', 'ABORT')
TRACKING_STATES = (TRACK, DESCEND, TOUCHDOWN, ABORT)


@dataclass
class ControlParams:
    mode: str = 'pid'          # 'pd' | 'pid' | 'lqr' | 'px4'
    kp: float = 1.2            # 1/s^2
    kd: float = 2.2            # 1/s  (zeta = kd / (2 sqrt(kp)) = 1.0)
    ki: float = 0.3            # 1/s^3, only used in mode 'pid' (stable while ki < kp kd)
    i_window: float = 1.0      # integrate only while |e_p| is below this (m)
    i_acc_limit: float = 1.5   # |ki * integral| at most this (m/s^2)
    lqr_q_pos: float = 1.0     # mode 'lqr': Q = diag(q_pos, q_vel), R = r
    lqr_q_vel: float = 0.3
    lqr_r: float = 0.8
    rate_hz: float = 20.0
    v_rel_max: float = 10.0    # chase speed relative to the truck (m/s)
    v_max: float = 12.0        # absolute horizontal speed limit (PX4 default MPC_XY_VEL_MAX)
    a_max: float = 4.0         # horizontal acceleration limit (m/s^2), about 22 degrees tilt

    def gains(self):
        """(kp, kd) actually used: the LQR gains in mode 'lqr'."""
        if self.mode != 'lqr':
            return self.kp, self.kd
        from scipy.linalg import solve_discrete_are
        dt = 1.0 / self.rate_hz
        A = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([[dt * dt / 2], [dt]])
        Q = np.diag([self.lqr_q_pos, self.lqr_q_vel])
        R = np.array([[self.lqr_r]])
        P = solve_discrete_are(A, B, Q, R)
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        return float(K[0, 0]), float(K[0, 1])


@dataclass
class LandingParams:
    autostart: bool = True
    cruise_alt: float = 25.0       # m above the drone's start, clears the tallest trees (~20 m)
    capture_dist: float = 8.0      # APPROACH -> TRACK when this close horizontally
    h_track: float = 4.0           # tracking height above the bed
    climb_rate: float = 2.0        # m/s, height profile going up
    sink_rate: float = 1.0         # m/s, height profile going down to h_track
    descend_rate: float = 0.5      # m/s in DESCEND
    touchdown_h: float = 0.2       # DESCEND -> TOUCHDOWN at this height above the bed
    final_rate: float = 1.0        # m/s pushed down in TOUCHDOWN
    touchdown_timeout: float = 4.0
    gate_pos: float = 0.3          # descent gate: |e_xy| below this ...
    gate_vel: float = 0.3          # ... and |e_v| below this ...
    gate_time: float = 1.5         # ... for this long
    gate_height: float = 0.3       # ... at h_track within this
    ready_drop: float = 3.0        # withdraw READY_TO_LAND only if |e_xy| grows past this
                                   # (the truck braking to v_land makes the PD lag ~a/kp)
    cone_e0: float = 0.2           # glide cone: allowed |e_xy| = e0 + slope * height
    cone_slope: float = 0.1
    touchdown_max_e: float = 0.5   # |e_xy| that still lands on the 2.5 x 2.0 m bed
    zone_margin_s: float = 2.0     # abort DESCEND with less landing zone than this left
    require_handshake: bool = True  # wait for the truck's LANDING_OK (phase 6)
    zone_budget_s: float = 15.0    # without the handshake: zone time needed to start
    target_timeout: float = 0.5    # target older than this = stale
    status_timeout: float = 1.0    # truck status older than this = no permission
    landed_height: float = 0.08    # touchdown without contact sensor: this close ...
    landed_speed: float = 0.15     # ... this still (relative vertical speed) ...
    landed_time: float = 0.3       # ... for this long


@dataclass
class DroneState:
    position: np.ndarray            # NED
    velocity: np.ndarray
    yaw: float
    armed: bool = False
    offboard: bool = False


@dataclass
class Target:
    position: np.ndarray            # bed surface centre, NED
    velocity: np.ndarray
    yaw: float
    yaw_rate: float
    age: float = 0.0                # seconds since it was received

    @property
    def acceleration(self):
        """Centripetal acceleration from the yaw rate (the truck keeps its speed while
        we land): a = r x v with r along NED down."""
        return np.array([-self.yaw_rate * self.velocity[1], self.yaw_rate * self.velocity[0], 0.0])


@dataclass
class TruckStatus:
    landing_ok: bool = False
    zone: int = -1
    zone_left_s: float = 0.0
    age: float = math.inf


@dataclass
class Setpoint:
    position: list = field(default_factory=lambda: [NAN] * 3)
    velocity: list = field(default_factory=lambda: [NAN] * 3)
    acceleration: list = field(default_factory=lambda: [NAN] * 3)
    yaw: float = NAN


@dataclass
class Output:
    setpoint: Setpoint
    want_offboard: bool
    want_armed: bool
    force_disarm: bool
    request: str                    # to the truck: NONE | READY_TO_LAND | ABORT | LANDED
    info: dict


def clip_norm(v, limit):
    n = float(np.linalg.norm(v))
    return v if n <= limit else v * (limit / n)


class TrackingLaw:
    """Horizontal control law; see the module docstring."""

    def __init__(self, p: ControlParams):
        self.p = p
        self.kp, self.kd = p.gains()
        self.integral = np.zeros(2)

    def reset(self):
        self.integral = np.zeros(2)

    def accel(self, drone, target, dt, integrate):
        p = self.p
        e_p = target.position[:2] - drone.position[:2]
        v_ref = target.velocity[:2] + clip_norm(self.kp / self.kd * e_p, p.v_rel_max)
        v_ref = clip_norm(v_ref, p.v_max)
        a = target.acceleration[:2] + self.kd * (v_ref - drone.velocity[:2])
        if p.mode == 'pid':
            if integrate and np.linalg.norm(e_p) < p.i_window:
                self.integral += e_p * dt
            if p.ki > 0:
                self.integral = clip_norm(self.integral, p.i_acc_limit / p.ki)
            a = a + p.ki * self.integral
        return clip_norm(a, p.a_max)

    def setpoint_xy(self, sp, drone, target, dt, integrate):
        if self.p.mode == 'px4':
            sp.position[:2] = [float(v) for v in target.position[:2]]
            sp.velocity[:2] = [float(v) for v in target.velocity[:2]]
            sp.acceleration[:2] = [float(v) for v in target.acceleration[:2]]
        else:
            sp.acceleration[:2] = [float(v) for v in self.accel(drone, target, dt, integrate)]


class LandingStateMachine:
    """Call step() at the setpoint rate. It never talks to PX4 itself: the returned
    Output says what to stream and whether to be armed / in offboard."""

    def __init__(self, control: ControlParams, p: LandingParams):
        self.p = p
        self.law = TrackingLaw(control)
        self.state = IDLE
        self.state_since = None
        self.request = 'NONE'
        self._z_sp = None           # vertical profile
        self._hold_xy = None
        self._gate_since = None
        self._landed_since = None
        self._h_cmd = None
        self._last_t = None
        self._last_target_z = None
        self.abort_reason = ''

    def _enter(self, state, now):
        if state != self.state:
            self.state = state
            self.state_since = now
            # The integral holds the drag, which is still there in DESCEND, TOUCHDOWN and
            # ABORT; start it afresh only for a new chase.
            if state == APPROACH:
                self.law.reset()
            self._gate_since = None
            self._landed_since = None

    def _ramp_z(self, goal, dt):
        """Move the vertical setpoint towards goal at the climb/sink rate; returns the
        setpoint's vertical speed (NED, + down) for feed-forward."""
        if self._z_sp is None:
            self._z_sp = goal
            return 0.0
        dz = goal - self._z_sp
        rate = self.p.sink_rate if dz > 0 else self.p.climb_rate
        step = max(-rate * dt, min(rate * dt, dz))
        self._z_sp += step
        return step / dt if dt > 0 else 0.0

    def step(self, now, drone: DroneState, target: Target | None, truck: TruckStatus,
             bed_contact: bool) -> Output:
        p = self.p
        dt = 1.0 / self.law.p.rate_hz if self._last_t is None else max(1e-3, now - self._last_t)
        self._last_t = now
        if self.state_since is None:
            self.state_since = now
        fresh = target is not None and target.age <= p.target_timeout
        if fresh:
            self._last_target_z = float(target.position[2])
        sp = Setpoint()
        want_armed = drone.armed
        want_offboard = drone.offboard
        force_disarm = False
        info = {}

        if self._hold_xy is None:
            self._hold_xy = drone.position[:2].copy()

        e_xy = e_v = h = math.nan
        if fresh:
            e_xy = float(np.linalg.norm(target.position[:2] - drone.position[:2]))
            e_v = float(np.linalg.norm(target.velocity[:2] - drone.velocity[:2]))
            h = float(target.position[2] - drone.position[2])  # height above the bed
        info.update(e_xy=e_xy, e_v=e_v, h=h)

        # Lost target while flying near the truck: hold position and climb (failsafe).
        if self.state in (APPROACH, TRACK, DESCEND, TOUCHDOWN) and not fresh:
            self.abort_reason = 'target stale'
            self._enter(ABORT, now)

        if self.state == IDLE:
            sp.position = [float(drone.position[0]), float(drone.position[1]), float(drone.position[2])]
            self._z_sp = float(drone.position[2])
            self._hold_xy = drone.position[:2].copy()
            if p.autostart and fresh and now - self.state_since > 2.0:
                self._enter(TAKEOFF, now)

        if self.state == TAKEOFF:
            want_armed = want_offboard = True
            vz = 0.0
            if drone.armed and drone.offboard:
                vz = self._ramp_z(-p.cruise_alt, dt)
            sp.position = [float(self._hold_xy[0]), float(self._hold_xy[1]), self._z_sp]
            sp.velocity = [0.0, 0.0, vz]
            if drone.position[2] < -(p.cruise_alt - 1.0):
                self._enter(APPROACH, now)

        if self.state == APPROACH:
            vz = self._ramp_z(-p.cruise_alt, dt)
            self.law.setpoint_xy(sp, drone, target, dt, integrate=False)
            sp.position[2], sp.velocity[2] = self._z_sp, vz
            sp.yaw = target.yaw
            if e_xy < p.capture_dist:
                self._enter(TRACK, now)

        if self.state == TRACK:
            goal = target.position[2] - p.h_track
            vz = self._ramp_z(goal, dt) + float(target.velocity[2])
            self.law.setpoint_xy(sp, drone, target, dt, integrate=True)
            sp.position[2], sp.velocity[2] = self._z_sp, vz
            sp.yaw = target.yaw
            gate = (e_xy < p.gate_pos and e_v < p.gate_vel
                    and abs(drone.position[2] - goal) < p.gate_height)
            if gate:
                self._gate_since = now if self._gate_since is None else self._gate_since
            else:
                self._gate_since = None
            gated = self._gate_since is not None and now - self._gate_since >= p.gate_time
            if gated:
                self.request = 'READY_TO_LAND'
            elif e_xy > p.ready_drop:
                self.request = 'NONE'
            descent_time = p.h_track / p.descend_rate + p.zone_margin_s
            status_ok = truck.age <= p.status_timeout
            if p.require_handshake:
                allowed = status_ok and truck.landing_ok and truck.zone_left_s >= descent_time
            else:
                allowed = status_ok and truck.zone >= 0 and truck.zone_left_s >= max(descent_time, p.zone_budget_s)
            info.update(gated=gated, allowed=allowed)
            if gated and allowed:
                self._h_cmd = p.h_track
                self._enter(DESCEND, now)

        if self.state == DESCEND:
            self._h_cmd = max(0.0, self._h_cmd - p.descend_rate * dt)
            sp.position[2] = float(target.position[2] - self._h_cmd)
            sp.velocity[2] = p.descend_rate + float(target.velocity[2])
            self._z_sp = sp.position[2]
            self.law.setpoint_xy(sp, drone, target, dt, integrate=True)
            sp.yaw = target.yaw
            cone = p.cone_e0 + p.cone_slope * max(h, 0.0)
            info.update(cone=cone)
            if e_xy > cone:
                self.abort_reason = f'left the glide cone ({e_xy:.2f} > {cone:.2f} m)'
                self._enter(ABORT, now)
            elif truck.age > p.status_timeout or (p.require_handshake and not truck.landing_ok):
                self.abort_reason = 'truck withdrew LANDING_OK'
                self._enter(ABORT, now)
            elif truck.zone_left_s < p.zone_margin_s:
                self.abort_reason = 'landing zone ends'
                self._enter(ABORT, now)
            elif h <= p.touchdown_h:
                self._enter(TOUCHDOWN, now)

        if self.state == TOUCHDOWN:
            # Push gently below the bed surface so the drone settles on it.
            sp.position[2] = float(target.position[2] + 0.3)
            sp.velocity[2] = p.final_rate + float(target.velocity[2])
            self._z_sp = sp.position[2]
            self.law.setpoint_xy(sp, drone, target, dt, integrate=True)
            sp.yaw = target.yaw
            still = h < p.landed_height and abs(drone.velocity[2] - target.velocity[2]) < p.landed_speed
            if still:
                self._landed_since = now if self._landed_since is None else self._landed_since
            else:
                self._landed_since = None
            if bed_contact or (self._landed_since is not None and now - self._landed_since >= p.landed_time):
                info.update(touchdown_e_xy=e_xy)
                self._enter(LANDED, now)
            elif e_xy > p.touchdown_max_e:
                self.abort_reason = f'off the bed at touchdown ({e_xy:.2f} m)'
                self._enter(ABORT, now)
            elif now - self.state_since > p.touchdown_timeout:
                self.abort_reason = 'no touchdown detected'
                self._enter(ABORT, now)

        if self.state == ABORT:
            self.request = 'ABORT'
            if fresh:
                goal = target.position[2] - p.h_track
                vz = self._ramp_z(goal, dt) + float(target.velocity[2])
                self.law.setpoint_xy(sp, drone, target, dt, integrate=False)
                sp.yaw = target.yaw
                self._hold_xy = drone.position[:2].copy()
                if abs(drone.position[2] - goal) < p.gate_height:
                    self.request = 'NONE'
                    self._enter(TRACK, now)
            else:
                ref_z = self._last_target_z if self._last_target_z is not None else 0.0
                vz = self._ramp_z(min(self._z_sp if self._z_sp is not None else ref_z,
                                      ref_z - p.h_track), dt)
                sp.position[:2] = [float(v) for v in self._hold_xy]
                sp.velocity[:2] = [0.0, 0.0]
                sp.yaw = drone.yaw
            sp.position[2], sp.velocity[2] = self._z_sp, vz

        if self.state == LANDED:
            self.request = 'LANDED'
            want_armed = False
            force_disarm = drone.armed
            sp.position = [float(drone.position[0]), float(drone.position[1]), float(drone.position[2])]

        info.update(state=self.state, request=self.request)
        return Output(sp, want_offboard, want_armed, force_disarm, self.request, info)


def wrap_deg(angle):
    """Angle in [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def gimbal_angles(drone_position, drone_yaw, target_position):
    """(pitch, yaw) in degrees that point the gimbal camera at the target, in the form
    PX4's DO_GIMBAL_MANAGER_PITCHYAW takes: pitch from the horizon (-90 straight down,
    kept in [-90, 0]), yaw relative to the drone's nose. NED positions, yaw in radians."""
    d = np.asarray(target_position, dtype=float) - np.asarray(drone_position, dtype=float)
    horizontal = math.hypot(d[0], d[1])
    pitch = -math.degrees(math.atan2(d[2], horizontal))
    yaw = 0.0 if horizontal < 1e-3 else wrap_deg(math.degrees(math.atan2(d[1], d[0]) - drone_yaw))
    return max(-90.0, min(0.0, pitch)), yaw


def pitch_yaw_deg(q):
    """Pitch and yaw (degrees, ZYX Euler) of a quaternion (w, x, y, z)."""
    w, x, y, z = q
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(pitch), math.degrees(yaw)
