#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import LEDStateCommands



def main(args=None):
    rclpy.init(args=args)
    node = Node("led_client_no_oop")

    client = node.create_client(LEDStateCommands, "set_led")
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for light server ...")

    request = LEDStateCommands.Request()
    request.data = [0, 1, 0]
    
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node,future)
    
    response = future.result()

    node.get_logger().info(str(response.response))

    rclpy.shutdown()

if __name__ == "__main__":
    main()