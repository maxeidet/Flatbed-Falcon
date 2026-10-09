"""Drives the truck laps around the road loop (PLAN.md phases 2 and 6).

Lateral: Pure Pursuit (Lab 3) on the road_centerline.csv SplinePath.
Longitudinal: curvature speed profile + PI speed trim. Landing handshake with the drone.

Subscribes: /truck/odom (nav_msgs/Odometry, world ENU, twist in the truck frame)
            /falcon/request (std_msgs/String: NONE | READY_TO_LAND | ABORT | LANDED)
Publishes:  /truck/cmd_vel (geometry_msgs/Twist: speed, yaw rate for the Ackermann plugin)
            /truck/status (std_msgs/String, JSON: s, d, v, zone, landing_ok, ...)
"""
import json

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from flatbed_falcon.frames import yaw_from_quaternion
from flatbed_falcon.params import declare_dataclass
from flatbed_falcon.road import RoadLoop
from flatbed_falcon.truck_control import (HandshakeParams, LandingCoordinator, TruckController,
                                          TruckParams)


class TruckDriver(Node):
    def __init__(self):
        super().__init__('truck_driver')
        road = RoadLoop(self.declare_parameter('road_csv', '/workspace/worlds/road_centerline.csv').value)
        self.rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.odom_timeout = self.declare_parameter('odom_timeout', 0.5).value
        self.controller = TruckController(road, declare_dataclass(self, TruckParams()))
        self.coordinator = LandingCoordinator(road, declare_dataclass(self, HandshakeParams()))
        self.road = road

        self.cmd_pub = self.create_publisher(Twist, '/truck/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/truck/status', 10)
        self.create_subscription(Odometry, '/truck/odom', self.on_odom, 10)
        self.create_subscription(String, '/falcon/request', self.on_request, 10)
        self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.odom = None
        self.odom_time = None
        self.lap = 0
        self.last_s = None
        self.was_committed = False
        self.get_logger().info(
            f'Road loop {road.length:.0f} m, landing zones '
            + ', '.join(f'{z}: {e - s:.0f} m' for z, (s, e) in road.zones.items()))

    def on_odom(self, msg):
        self.odom = msg
        self.odom_time = self.get_clock().now()

    def on_request(self, msg):
        if msg.data != self.coordinator.request:
            self.get_logger().info(f'Drone request: {msg.data}')
        self.coordinator.on_request(msg.data)

    def on_timer(self):
        now = self.get_clock().now()
        cmd = Twist()
        if self.odom is None or (now - self.odom_time).nanoseconds * 1e-9 > self.odom_timeout:
            self.cmd_pub.publish(cmd)  # no odometry: stand still
            return
        pose, twist = self.odom.pose.pose, self.odom.twist.twist
        yaw = yaw_from_quaternion(pose.orientation)
        v = twist.linear.x

        s_now = self.controller.s
        v_cap, status = self.coordinator.update(s_now, v) if s_now is not None else (None, {})
        committed = status.get('landing_ok', False)
        v_cmd, yaw_rate, info = self.controller.step(
            pose.position.x, pose.position.y, yaw, v, 1.0 / self.rate_hz, v_cap)
        cmd.linear.x = v_cmd
        cmd.angular.z = yaw_rate
        self.cmd_pub.publish(cmd)

        if self.last_s is not None and info['s'] < self.last_s - self.road.length / 2:
            self.lap += 1
            self.get_logger().info(f'Lap {self.lap} done')
        self.last_s = info['s']
        if committed != self.was_committed:
            self.get_logger().info(f'LANDING_OK: holding {v_cmd:.1f} m/s' if committed
                                   else 'Landing commitment released')
        self.was_committed = committed

        status.update(s=round(info['s'], 2), d=round(info['d'], 3), v=round(v, 3),
                      v_ref=round(info['v_ref'], 2), delta=round(info['delta'], 4), lap=self.lap)
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = TruckDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
