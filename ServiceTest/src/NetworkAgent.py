#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger

import random
import time

class NeighborDiscover(Node):
    def __init__(self):
        super().__init__("ND")
        self.srv = self.create_service(
            srv_type=Trigger, srv_name='Trigger', callback=self.serve_trigger,
            callback_group=ReentrantCallbackGroup())
        self.timer1 = self.create_timer(5.0, self.discover)
        self.timer2 = self.create_timer(1.0, self.timer_cb)
        self.futures = []
        self.neighbors = set()
        self.cnt = 0

    def timer_cb(self):
        if len(self.neighbors) > 0:
            self.get_logger().info('='*10)
            for nei in self.neighbors:
                self.get_logger().info(f"{nei}")
            self.get_logger().info('='*10)


    def discover(self):
        if self.cnt % 2 == 0:
            for client, future in self.futures:
                client.remove_pending_request(future)
            self.futures.clear()
            self.neighbors.clear()
        self.cnt += 1
        services = self.get_service_names_and_types()
        my_name = self.get_namespace()
        for srv_name, srv_type in services:
            if 'Trigger' in srv_name and my_name not in srv_name:
                self.get_logger().info(f'Request to {srv_name}')
                client = self.create_client(Trigger, srv_name)
                future = client.call_async(Trigger.Request())
                future.add_done_callback(self.receive_response)
                self.futures.append((client,future))

    def serve_trigger(self, request, response):
        response.success = True
        response.message = self.get_namespace()
        # self.get_logger().info('Received request')
        duration = random.randint(1,5)
        # self.get_logger().info(f"Sleeping for {duration} seconds...")
        time.sleep(duration)
        # self.get_logger().info('Send response')
        return response

    def receive_response(self, future):
        response = future.result()
        self.get_logger().info(f"Receive: {response.message}")
        self.neighbors.add(response.message)

def main(args=None):
    rclpy.init(args=args)
    nd = NeighborDiscover()
    executor = MultiThreadedExecutor(4)
    executor.add_node(nd)
    executor.spin()
    executor.remove_node(nd)
    rclpy.shutdown()

if __name__ == "__main__":
    main()