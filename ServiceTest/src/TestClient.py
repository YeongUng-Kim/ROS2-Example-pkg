#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class TestClient(Node):
    def __init__(self):
        super().__init__("TestClient")
        self.cli = self.create_client(Trigger, '/Trigger')
        self.req = Trigger.Request()
        self.timer1 = self.create_timer(4, self.timer1_callback)
        self.timer2 = self.create_timer(2, self.timer2_callback)
        self.future = None

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.receive_response)

    def timer1_callback(self):
        if self.future is not None:
            self.cli.remove_pending_request(self.future)
        self.send_request()

    def timer2_callback(self):
        self.get_logger().info('Alive')
        pass

    def receive_response(self, future):
        response = future.result()
        self.get_logger().info(f"Receive: {response.message}")

def main(args=None):
    rclpy.init(args=args)
    client = TestClient()
    # while rclpy.ok():
    #     rclpy.spin_once(client)
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == "__main__":
    main()