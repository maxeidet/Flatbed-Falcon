"""Tracking controller and landing state machine.

States: IDLE -> TAKEOFF -> APPROACH -> TRACK -> DESCEND -> LANDED (ABORT on large error).

Subscribes: /falcon/target, /fmu/out/vehicle_odometry, /fmu/out/vehicle_status_v4
Publishes:  /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint,
            /fmu/in/vehicle_command
"""
import rclpy
from rclpy.node import Node


class LandingController(Node):
    def __init__(self):
        super().__init__('landing_controller')
        # TODO: PID on relative position + truck velocity feedforward, state machine.
        self.get_logger().info('landing_controller started (not implemented yet)')


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
