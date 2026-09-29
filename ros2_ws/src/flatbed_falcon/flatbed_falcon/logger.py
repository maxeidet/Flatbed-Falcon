"""Records drone, truck and controller state to CSV for the presentation plots."""
import rclpy
from rclpy.node import Node


class Logger(Node):
    def __init__(self):
        super().__init__('falcon_logger')
        # TODO: subscribe to target + drone odometry + state, write CSV rows.
        self.get_logger().info('logger started (not implemented yet)')


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
