"""Turns the truck's odometry into the landing target in PX4's frame (PLAN.md phases 3, 7).

The target is the centre of the bed surface. Its world ENU pose and velocity are
converted to PX4 local NED (origin = the drone's spawn point), sent through a
simulated radio link (noise, latency, dropout; all off by default), filtered and
predicted to the current time (+ predict_ahead).

Subscribes: /truck/odom (nav_msgs/Odometry, world ENU, twist in the truck frame)
Publishes:  /falcon/target (nav_msgs/Odometry, PX4 local NED, see below)
            /falcon/target_measured (same, the latest measurement as received, for logs)

In both outputs the twist is NOT in the child frame as ROS usually has it:
twist.linear is the NED world velocity and twist.angular.z the NED yaw rate
(positive clockwise seen from above). Orientation is the NED yaw.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from flatbed_falcon import frames
from flatbed_falcon.params import declare_dataclass
from flatbed_falcon.target_filter import LinkParams, Measurement, SimulatedLink, TargetEstimator


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')
        self.spawn_enu = list(self.declare_parameter('spawn_enu', [0.0, 0.0, 0.0]).value)
        self.bed_offset = list(self.declare_parameter('bed_offset', [-1.15, 0.0, 1.0]).value)
        self.rate_hz = self.declare_parameter('rate_hz', 50.0).value
        self.predict_ahead = self.declare_parameter('predict_ahead', 0.0).value
        self.timeout = self.declare_parameter('timeout', 1.0).value
        use_filter = self.declare_parameter('use_filter', False).value
        accel_std = self.declare_parameter('filter_accel_std', 1.0).value
        link = declare_dataclass(self, LinkParams(), prefix='link_')
        self.link = SimulatedLink(link)
        # The filter's measurement noise follows the link's, with a floor so it stays sane.
        self.estimator = TargetEstimator(use_filter, accel_std,
                                         max(link.pos_noise, 0.02), max(link.vel_noise, 0.02))

        self.pub = self.create_publisher(Odometry, '/falcon/target', 10)
        self.measured_pub = self.create_publisher(Odometry, '/falcon/target_measured', 10)
        self.create_subscription(Odometry, '/truck/odom', self.on_odom, 10)
        self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().info(
            f'Bed offset {self.bed_offset}, spawn {self.spawn_enu}, filter {"on" if use_filter else "off"}, '
            f'link noise {link.pos_noise} m / {link.vel_noise} m/s, latency {link.latency} s, '
            f'dropout {link.dropout}')

    def on_odom(self, msg):
        pose, twist = msg.pose.pose, msg.twist.twist
        yaw = frames.yaw_from_quaternion(pose.orientation)
        # Gazebo's odometry twist is in the truck frame; rotate it into the world.
        c, s = math.cos(yaw), math.sin(yaw)
        vx = c * twist.linear.x - s * twist.linear.y
        vy = s * twist.linear.x + c * twist.linear.y
        p, v = frames.body_point_enu(pose.position.x, pose.position.y, pose.position.z, yaw,
                                     vx, vy, twist.linear.z, twist.angular.z, self.bed_offset)
        self.link.send(Measurement(
            stamp_seconds(msg.header.stamp),
            np.array(frames.world_enu_to_px4_local(*p, self.spawn_enu)),
            np.array(frames.enu_to_ned(*v)),
            frames.yaw_enu_to_ned(yaw),
            -twist.angular.z))

    def on_timer(self):
        now_msg = self.get_clock().now().to_msg()
        now = stamp_seconds(now_msg)
        for m in self.link.receive(now):
            self.estimator.update(m)
            self.measured_pub.publish(self.to_msg(now_msg, m.position, m.velocity, m.yaw, m.yaw_rate))
        if self.estimator.age(now) > self.timeout:
            return  # stale: publish nothing, the drone treats silence as a lost target
        self.pub.publish(self.to_msg(now_msg, *self.estimator.estimate(now + self.predict_ahead)))

    def to_msg(self, stamp, position, velocity, yaw, yaw_rate):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'px4_local_ned'
        msg.child_frame_id = 'truck_bed'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = (
            float(v) for v in position)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = (
            float(v) for v in velocity)
        msg.twist.twist.angular.z = float(yaw_rate)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = TargetTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
