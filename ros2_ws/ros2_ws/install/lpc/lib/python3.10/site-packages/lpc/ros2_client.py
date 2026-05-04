import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.cli = self.create_client(AddTwoInts, "add_two_ints")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()
    a, b = int(sys.argv[1]), int(sys.argv[2])
    future = client.send_request(a, b)
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    client.get_logger().info(f"[Client] {a} + {b} = {response.sum}")
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
