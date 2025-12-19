#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from robot_interfaces.srv import ComputeRectangleArea

class ComputeRectangleAreaNode(Node): 
    def __init__(self):
        super().__init__("add_two_ints_server") 
        self.server_ = self.create_service(ComputeRectangleArea, "compute_rectangle_area", self.callback_compute_area) #server type, server name (pick one) - start it with a verb (action)
        self.get_logger().info("Compute Rectangle Area Server has been started")

    def callback_compute_area(self, request: ComputeRectangleArea.Request, response: ComputeRectangleArea.Response):
        response.area = request.length*request.width
        self.get_logger().info("Area is equal to " + str(response.area))
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ComputeRectangleAreaNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()