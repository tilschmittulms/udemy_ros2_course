#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("number_counter") # MODIFY NAME
        self.subscriber_ = self.create_subscription(
            Int64, "number", self.callback_number, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.counter_ = 0
        self.timer_ = self.create_timer(1, self. publish_count)
        self.server_ = self.create_service(SetBool,"reset_counter", self.callback_reset_counter)
        self.get_logger().info("number counter and reset counter service have been started")


    def callback_reset_counter(self, request: SetBool.Request, response: SetBool.Response):
        if request.data == True:
            response.success = True
            response.message = "Successfully reset number counter"
            self.counter_ = 0
        elif request.data == False:
            response.success = False
            response.message = "Did not reset counter"
        self.get_logger().info("Counter reset check done")
        return response 

    def publish_count(self):
        number = Int64()
        number.data = self.counter_
        self.publisher_.publish(number)
        self.counter_ += 1
        
    def callback_number(self, msg: Int64):
        self.get_logger().info(str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()