#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import LEDStates
from turtlesim.srv import Spawn
import random


class TurtleGeneratorNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("generate_turtle") # MODIFY NAME
        self.spawn_client_= self.create_client(Spawn, "spawn")

        self.timer_ = self.create_timer(1.0, self.call_turtle_spawn)
        self.get_logger().info("turtle generator node has been started")

    def call_turtle_spawn(self):
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("waiting for turtlesim node...")

        request = Spawn.Request()
        request.x = random.uniform(0.0, 11.0)
        request.y = random.uniform(0.0, 11.0)
        request.theta = random.uniform(0.0, 6.28)

        future = self.spawn_client_.call_async(request)
        future.add_done_callback(self.callback_call_turtle_spawn)

    def callback_call_turtle_spawn(self, future):
        response = future.result()
        self.get_logger().info("got response, turtle: " + response.name + " has been created")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleGeneratorNode() # MODIFY NAME
    #node.call_turtle_spawn()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()