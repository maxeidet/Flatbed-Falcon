"""Turns the truck's pose into a landing target in PX4's frame.

Subscribes: /truck/odom (nav_msgs/Odometry, world ENU)
Publishes:  /falcon/target (nav_msgs/Odometry, PX4 local NED)
Uses frames.world_enu_to_px4_local() with the drone's spawn position.
"""
import rclpy
from rclpy.node import Node


class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')
        # TODO: convert truck position, velocity and yaw to NED; optional noise/latency.
        self.get_logger().info('target_tracker started (not implemented yet)')


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
