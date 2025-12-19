#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import LEDStateCommands

class BatteryNode(Node): 
    def __init__(self):
        super().__init__("battery_node")
        self.battery_state_ = "full"
        self.last_time_battery_state_changed_ = self.get_current_time_seconds()
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)
        self.set_led_client_ = self.create_client(LEDStateCommands, "set_led")
        self.get_logger().info("battery node started")

    def get_current_time_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1000000000.0
    
    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state_ == "full":
            if time_now - self.last_time_battery_state_changed_ > 4:
                self.battery_state_ = "empty"
                self.get_logger().info("battery is empty! charging...")
                self.last_time_battery_state_changed_ = time_now
                self.call_set_led([0, 0, 1])
        elif self.battery_state_ == "empty":
            if time_now - self.last_time_battery_state_changed_ > 6.0:
                self.battery_state_ = "full"
                self.get_logger().info("battery is now full.")
                self.call_set_led([0, 0, 0])
                self.last_time_battery_state_changed_ = time_now

    def call_set_led(self, led_state):
        while not self.set_led_client_.wait_for_service(1.0):
            self.get_logger().warn("waiting for set led service")

        request = LEDStateCommands.Request()
        request.data = led_state

        future = self.set_led_client_.call_async(request)
        future.add_done_callback(self.callback_call_set_led)


    def callback_call_set_led(self, future):
        response: LEDStateCommands.Response = future.result()
        if response.response:
            self.get_logger().info("LED state was changed")
        else:
            self.get_logger().info("LED not changed")


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()