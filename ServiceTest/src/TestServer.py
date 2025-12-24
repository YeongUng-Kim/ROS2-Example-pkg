#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

import time

class TestServer(Node):
    def __init__(self, cb_group):
        super().__init__('TestServer')
        self.declare_parameter('name', 'default')
        self.srv_name = self.get_parameter('name').get_parameter_value().string_value
        self.srv = self.create_service(srv_type=Trigger, srv_name='/Trigger', callback=self.serve_trigger, callback_group=ReentrantCallbackGroup())
        self.cnt = 0
        self.timer = self.create_timer(2, self.timer_callback, ReentrantCallbackGroup())

    def timer_callback(self):
        self.get_logger().info('Alive')

    async def serve_trigger(self, request, response):
        response.success = True
        response.message = self.srv_name + "_" + str(self.cnt)
        self.get_logger().info('Received request')
        self.get_logger().info(f'Sleep in {self.cnt} sec')
        # now = time.time()
        # while (time.time() - now) < self.cnt:
        #     continue
        time.sleep(self.cnt)
        self.get_logger().info(f'Send response: {response.message}')
        self.cnt += 1
        return response

def main(args=None):
    rclpy.init(args=args)
    cb_group = ReentrantCallbackGroup()
    server = TestServer(cb_group=cb_group)
    executor = MultiThreadedExecutor(4)
    executor.add_node(server)
    try:
        executor.spin()
    finally:
        executor.remove_node(server)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
