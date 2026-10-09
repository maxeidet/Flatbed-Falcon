"""Tracking controller and landing state machine on PX4 offboard (PLAN.md phases 4-6).

States: IDLE -> TAKEOFF -> APPROACH -> TRACK -> DESCEND -> TOUCHDOWN -> LANDED,
        ABORT -> TRACK. The control law and the states live in landing_logic.py;
this node feeds them and talks to PX4.

With a gimbal (gz_x500_gimbal, the default drone) it also keeps the camera on the
bed: it takes control of PX4's gimbal manager and streams pitch/yaw at gimbal_rate_hz.
gimbal_mode: track (point at the bed, straight down without a target) | down | off.

Subscribes: /falcon/target, /truck/status, /truck/bed_contact,
            /fmu/out/vehicle_odometry, /fmu/out/vehicle_status_v4,
            /fmu/out/vehicle_command_ack_v1, /fmu/out/gimbal_device_attitude_status
Publishes:  /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint,
            /fmu/in/vehicle_command, /falcon/request, /falcon/state (JSON)
"""
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import GimbalDeviceAttitudeStatus, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleOdometry, VehicleStatus
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String

from flatbed_falcon import landing_logic as logic
from flatbed_falcon.frames import yaw_from_quaternion
from flatbed_falcon.params import declare_dataclass

# PX4's output topics (/fmu/out/...) publish BEST_EFFORT/TRANSIENT_LOCAL, not the
# default RELIABLE. A mismatched QoS connects but silently never receives anything.
PX4_OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
# PX4's input topics (/fmu/in/...) expect BEST_EFFORT/VOLATILE.
PX4_IN_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=1)

RETRY_PERIOD_S = 1.0           # between repeated arm/offboard/disarm commands
STREAM_BEFORE_OFFBOARD_S = 1.0  # PX4 rejects offboard without a live setpoint stream
CONTACT_HOLD_S = 0.2           # a bed contact counts for this long
FORCE_DISARM = 21196.0         # VehicleCommand param2: disarm even if PX4 thinks it flies
GIMBAL_TRIES = 10              # take-control attempts before deciding there is no gimbal


class Quat:
    def __init__(self, q):
        self.w, self.x, self.y, self.z = (float(v) for v in q)


class LandingController(Node):
    def __init__(self):
        super().__init__('landing_controller')
        control = declare_dataclass(self, logic.ControlParams(), prefix='control_')
        landing = declare_dataclass(self, logic.LandingParams())
        self.drone_name = self.declare_parameter('drone_collision_name', 'x500').value
        self.gimbal_mode = self.declare_parameter('gimbal_mode', 'track').value
        self.gimbal_period = 1.0 / self.declare_parameter('gimbal_rate_hz', 5.0).value
        self.machine = logic.LandingStateMachine(control, landing)
        kp, kd = self.machine.law.kp, self.machine.law.kd
        self.get_logger().info(f'Control mode {control.mode}: kp {kp:.3f}, kd {kd:.3f}'
                               + (f', ki {control.ki}' if control.mode == 'pid' else ''))
        # PX4 timestamps are on the system clock (the DDS agent's time sync), not sim time.
        self.px4_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_IN_QOS)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_IN_QOS)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_IN_QOS)
        self.request_pub = self.create_publisher(String, '/falcon/request', 10)
        self.state_pub = self.create_publisher(String, '/falcon/state', 10)

        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odometry, PX4_OUT_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.on_status, PX4_OUT_QOS)
        self.create_subscription(Odometry, '/falcon/target', self.on_target, 10)
        self.create_subscription(String, '/truck/status', self.on_truck_status, 10)
        self.create_subscription(Contacts, '/truck/bed_contact', self.on_contact, 10)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack_v1', self.on_ack, PX4_OUT_QOS)
        self.create_subscription(GimbalDeviceAttitudeStatus, '/fmu/out/gimbal_device_attitude_status',
                                 self.on_gimbal, PX4_OUT_QOS)
        self.create_timer(1.0 / control.rate_hz, self.on_timer)

        self.drone = None
        self.status = None
        self.target = None
        self.target_time = None
        self.truck = None
        self.truck_time = None
        self.contact_time = None
        self.streamed = 0
        self.last_command = {}
        self.rate_hz = control.rate_hz
        self.last_state = None
        self.gimbal_controlled = False  # PX4 acked us as the gimbal's primary controller
        self.gimbal_tries = 0
        self.gimbal_last_sent = -math.inf
        self.gimbal_cmd = (math.nan, math.nan)
        self.gimbal = (math.nan, math.nan)  # measured (pitch, yaw relative to the nose), degrees

    # --- inputs ---------------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odometry(self, msg):
        if any(math.isnan(v) for v in msg.position):
            return
        if msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            self.get_logger().error('vehicle_odometry velocity is not NED', throttle_duration_sec=5.0)
            return
        self.drone = logic.DroneState(np.array(msg.position, dtype=float), np.array(msg.velocity, dtype=float),
                                      yaw_from_quaternion(Quat(msg.q)))

    def on_status(self, msg):
        if self.status is None:
            self.get_logger().info('Connected to PX4')
        self.status = msg

    def on_target(self, msg):
        self.target = msg
        self.target_time = self.now()

    def on_truck_status(self, msg):
        self.truck = json.loads(msg.data)
        self.truck_time = self.now()

    def on_ack(self, msg):
        if msg.command != VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE or self.gimbal_controlled:
            return
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.gimbal_controlled = True
            self.get_logger().info('In control of the gimbal')
        else:
            self.get_logger().warning(f'PX4 refused gimbal control (result {msg.result})',
                                      throttle_duration_sec=5.0)

    def on_gimbal(self, msg):
        q = [float(v) for v in msg.q]
        if any(math.isnan(v) for v in q) or self.drone is None:
            return
        pitch, yaw = logic.pitch_yaw_deg(q)
        # PX4's sim gimbal reports yaw in the earth frame (NED, from north); make it
        # relative to the nose like the command.
        if msg.device_flags & GimbalDeviceAttitudeStatus.DEVICE_FLAGS_YAW_IN_EARTH_FRAME:
            yaw -= math.degrees(self.drone.yaw)
        self.gimbal = (pitch, logic.wrap_deg(yaw))

    def on_contact(self, msg):
        for c in msg.contacts:
            if self.drone_name in c.collision1.name or self.drone_name in c.collision2.name:
                self.contact_time = self.now()
                return

    # --- the loop -------------------------------------------------------

    def on_timer(self):
        if self.drone is None:
            return
        now = self.now()
        drone = self.drone
        if self.status is not None:
            drone.armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            drone.offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        target = None
        if self.target is not None:
            pose, twist = self.target.pose.pose, self.target.twist.twist
            target = logic.Target(
                np.array([pose.position.x, pose.position.y, pose.position.z]),
                np.array([twist.linear.x, twist.linear.y, twist.linear.z]),
                yaw_from_quaternion(pose.orientation), twist.angular.z, now - self.target_time)
        truck = logic.TruckStatus()
        if self.truck is not None:
            truck = logic.TruckStatus(bool(self.truck.get('landing_ok', False)), int(self.truck.get('zone', -1)),
                                      float(self.truck.get('zone_left_s', 0.0)), now - self.truck_time)
        contact = self.contact_time is not None and now - self.contact_time < CONTACT_HOLD_S

        out = self.machine.step(now, drone, target, truck, contact)
        self.stream(out.setpoint)
        self.command(drone, out)
        fresh = target is not None and target.age <= self.machine.p.target_timeout
        self.aim_gimbal(now, drone, target if fresh and out.info['state'] != logic.LANDED else None)
        self.request_pub.publish(String(data=out.request))

        info = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in out.info.items()}
        info.update(t=round(now, 3), armed=drone.armed, offboard=drone.offboard, contact=contact,
                    acc_sp=[round(v, 3) for v in out.setpoint.acceleration],
                    gimbal_cmd=[round(v, 2) for v in self.gimbal_cmd],
                    gimbal=[round(v, 2) for v in self.gimbal])
        self.state_pub.publish(String(data=json.dumps(info).replace('NaN', 'null')))
        if out.info['state'] != self.last_state:
            reason = f' ({self.machine.abort_reason})' if out.info['state'] == logic.ABORT else ''
            self.get_logger().info(f'State {self.last_state} -> {out.info["state"]}{reason}')
            self.last_state = out.info['state']
            if out.info['state'] == logic.LANDED:
                self.get_logger().info(f'Touchdown {out.info.get("touchdown_e_xy", float("nan")):.2f} m '
                                       'from the bed centre')

    def aim_gimbal(self, now, drone, target):
        """Keep the camera on the bed (straight down without a target)."""
        if self.gimbal_mode == 'off' or self.status is None or self.gimbal_tries > GIMBAL_TRIES:
            return
        if now - self.gimbal_last_sent < self.gimbal_period:
            return
        self.gimbal_last_sent = now
        if not self.gimbal_controlled:
            # PX4's gimbal manager ignores pitch/yaw from anyone but its primary controller.
            # Our ids are given explicitly: PX4 resolves "-2 = the sender" to its own id
            # for commands arriving over DDS. param3/4 = -1 leave the secondary unchanged.
            self.gimbal_tries += 1
            if self.gimbal_tries > GIMBAL_TRIES:
                self.get_logger().warning('No gimbal answered; is the drone gz_x500_gimbal?')
                return
            self.publish_command(VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE, 1.0, 1.0, -1.0, -1.0)
            return
        if self.gimbal_mode == 'track' and target is not None:
            self.gimbal_cmd = logic.gimbal_angles(drone.position, drone.yaw, target.position)
        else:
            self.gimbal_cmd = (-90.0, 0.0)
        # Pitch from the horizon, yaw from the nose; rates NaN = as fast as it can;
        # flags 0 = yaw follows the drone; param7 0 = every gimbal on the drone.
        self.publish_command(VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                             *self.gimbal_cmd, math.nan, math.nan, 0.0, 0.0, 0.0)

    def px4_us(self):
        return int(self.px4_clock.now().nanoseconds / 1000)

    def stream(self, sp):
        mode = OffboardControlMode()
        mode.timestamp = self.px4_us()
        mode.position = True
        mode.velocity = True
        mode.acceleration = True
        self.offboard_pub.publish(mode)
        msg = TrajectorySetpoint()
        msg.timestamp = mode.timestamp
        msg.position = [float(v) for v in sp.position]
        msg.velocity = [float(v) for v in sp.velocity]
        msg.acceleration = [float(v) for v in sp.acceleration]
        msg.jerk = [math.nan] * 3
        msg.yaw = float(sp.yaw)
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)
        self.streamed += 1

    def command(self, drone, out):
        if self.status is None:
            return
        if out.force_disarm:
            self.send_command('disarm', VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, FORCE_DISARM)
            return
        if out.want_offboard and not drone.offboard and self.streamed >= STREAM_BEFORE_OFFBOARD_S * self.rate_hz:
            self.send_command('offboard', VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if out.want_armed and not drone.armed:
            self.send_command('arm', VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def send_command(self, name, command, param1=0.0, param2=0.0):
        """Send at most once per RETRY_PERIOD_S (on the ROS clock); repeated until the
        status shows it took effect, because single commands can get lost."""
        now = self.now()
        if now - self.last_command.get(name, -math.inf) < RETRY_PERIOD_S:
            return
        self.last_command[name] = now
        self.get_logger().info(f'Sending {name}')
        self.publish_command(command, param1, param2)

    def publish_command(self, command, *params):
        """params = param1, param2, ... (the rest stay 0)."""
        msg = VehicleCommand()
        msg.timestamp = self.px4_us()
        msg.command = command
        for i, value in enumerate(params, start=1):
            setattr(msg, f'param{i}', float(value))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LandingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
