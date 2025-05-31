import sys

from interfaces.srv import GenerateBEV
import rclpy
from rclpy.node import Node


class BEVclient(Node):

    def __init__(self):
        super().__init__('bev_client_async')
        self.cli = self.create_client(GenerateBEV, 'generate_bev')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GenerateBEV.Request()

    def send_request(self, source, output):
        self.req.source = source
        self.req.output = output
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    bev_client = BEVclient()
    future = bev_client.send_request(sys.argv[1],sys.argv[2])
    rclpy.spin_until_future_complete(bev_client, future)
    response = future.result()
    bev_client.get_logger().info(
        'Converting %s to %s.avi: %s' %
        (sys.argv[1], sys.argv[2], response.status))

    bev_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()