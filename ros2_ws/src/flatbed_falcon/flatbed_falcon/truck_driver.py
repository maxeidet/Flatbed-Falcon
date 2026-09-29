"""Drives the truck around a perimeter with Pure Pursuit.

Subscribes: /truck/odom (nav_msgs/Odometry, world ENU)
Publishes:  /truck/cmd_vel (geometry_msgs/Twist)
"""
import rclpy
from rclpy.node import Node


class TruckDriver(Node):
    def __init__(self):
        super().__init__('truck_driver')
        # TODO: waypoint list, Pure Pursuit (lookahead distance, target speed).
        self.get_logger().info('truck_driver started (not implemented yet)')


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
