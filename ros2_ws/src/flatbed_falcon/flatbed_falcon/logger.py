"""Records truck, target, drone and controller state to one CSV per run (PLAN.md phase 8).

Writes <log_dir>/run_<date>_<time>.csv at rate_hz with the latest value of each topic.
Positions: truck in world ENU, target and drone in PX4 local NED.
"""
import csv
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from std_msgs.msg import String

from flatbed_falcon.frames import yaw_from_quaternion
from flatbed_falcon.landing_controller import PX4_OUT_QOS

COLUMNS = [
    't',
    'truck_x', 'truck_y', 'truck_yaw', 'truck_v', 'truck_cmd_v', 'truck_cmd_yaw_rate',
    'truck_s', 'truck_d', 'truck_v_ref', 'truck_zone', 'truck_zone_left_s', 'landing_ok', 'lap',
    'meas_x', 'meas_y', 'meas_z', 'meas_vx', 'meas_vy',
    'target_x', 'target_y', 'target_z', 'target_vx', 'target_vy', 'target_yaw',
    'drone_x', 'drone_y', 'drone_z', 'drone_vx', 'drone_vy', 'drone_vz',
    'state', 'request', 'e_xy', 'e_v', 'h', 'armed', 'offboard', 'contact', 'acc_sp_x', 'acc_sp_y',
    'gimbal_pitch_cmd', 'gimbal_yaw_cmd', 'gimbal_pitch', 'gimbal_yaw',
]


class Logger(Node):
    def __init__(self):
        super().__init__('falcon_logger')
        log_dir = self.declare_parameter('log_dir', '/workspace/logs').value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        os.makedirs(log_dir, exist_ok=True)
        self.path = os.path.join(log_dir, time.strftime('run_%Y%m%d_%H%M%S.csv'))
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.DictWriter(self.file, COLUMNS, restval='')
        self.writer.writeheader()
        self.row = {}

        self.create_subscription(Odometry, '/truck/odom', self.on_truck, 10)
        self.create_subscription(Twist, '/truck/cmd_vel', self.on_cmd, 10)
        self.create_subscription(String, '/truck/status', self.on_truck_status, 10)
        self.create_subscription(Odometry, '/falcon/target_measured', self.on_measured, 10)
        self.create_subscription(Odometry, '/falcon/target', self.on_target, 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_drone, PX4_OUT_QOS)
        self.create_subscription(String, '/falcon/state', self.on_state, 10)
        self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(f'Logging to {self.path}')

    def on_truck(self, msg):
        p, t = msg.pose.pose, msg.twist.twist
        self.row.update(truck_x=p.position.x, truck_y=p.position.y,
                        truck_yaw=yaw_from_quaternion(p.orientation), truck_v=t.linear.x)

    def on_cmd(self, msg):
        self.row.update(truck_cmd_v=msg.linear.x, truck_cmd_yaw_rate=msg.angular.z)

    def on_truck_status(self, msg):
        s = json.loads(msg.data)
        self.row.update(truck_s=s.get('s'), truck_d=s.get('d'), truck_v_ref=s.get('v_ref'),
                        truck_zone=s.get('zone'), truck_zone_left_s=s.get('zone_left_s'),
                        landing_ok=int(bool(s.get('landing_ok'))), lap=s.get('lap'))

    def on_measured(self, msg):
        p, t = msg.pose.pose.position, msg.twist.twist.linear
        self.row.update(meas_x=p.x, meas_y=p.y, meas_z=p.z, meas_vx=t.x, meas_vy=t.y)

    def on_target(self, msg):
        p, t = msg.pose.pose, msg.twist.twist.linear
        self.row.update(target_x=p.position.x, target_y=p.position.y, target_z=p.position.z,
                        target_vx=t.x, target_vy=t.y, target_yaw=yaw_from_quaternion(p.orientation))

    def on_drone(self, msg):
        (x, y, z), (vx, vy, vz) = msg.position, msg.velocity
        self.row.update(drone_x=x, drone_y=y, drone_z=z, drone_vx=vx, drone_vy=vy, drone_vz=vz)

    def on_state(self, msg):
        s = json.loads(msg.data)
        acc = s.get('acc_sp') or [None, None]
        gimbal_cmd = s.get('gimbal_cmd') or [None, None]
        gimbal = s.get('gimbal') or [None, None]
        self.row.update(state=s.get('state'), request=s.get('request'), e_xy=s.get('e_xy'),
                        e_v=s.get('e_v'), h=s.get('h'), armed=int(bool(s.get('armed'))),
                        offboard=int(bool(s.get('offboard'))), contact=int(bool(s.get('contact'))),
                        acc_sp_x=acc[0], acc_sp_y=acc[1],
                        gimbal_pitch_cmd=gimbal_cmd[0], gimbal_yaw_cmd=gimbal_cmd[1],
                        gimbal_pitch=gimbal[0], gimbal_yaw=gimbal[1])

    def on_timer(self):
        if 'truck_x' not in self.row:
            return
        row = {k: (round(v, 4) if isinstance(v, float) and math.isfinite(v) else v)
               for k, v in self.row.items()}
        row['t'] = round(self.get_clock().now().nanoseconds * 1e-9, 3)
        self.writer.writerow(row)
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
