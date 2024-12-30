import sys

from control_msgs.srv import ControlCommand
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('command_call')
        self.cli = self.create_client(ControlCommand, 'control_node/control_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ControlCommand.Request()

    def send_request(self, a, b):
        self.req.cmd_name = a
        self.req.cmd_params = b
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request("activate", "RobotController")
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of call: %d' % response.result)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()