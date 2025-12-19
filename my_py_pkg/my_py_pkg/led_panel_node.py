#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import LEDStates
from robot_interfaces.srv import LEDStateCommands

class LEDPanelNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("led_panel") # MODIFY NAME
        self.declare_parameter("led_states", [0, 0, 0])
        self.led_states_internal = self.get_parameter("led_states").value
        self.publisher_ = self.create_publisher(LEDStates, "light_state", 10)
        self.timer_ = self.create_timer(1.0, self.callback_publish_light_states)
        self.get_logger().info("LED panel initiated")
        self.server_ = self.create_service(LEDStateCommands, "set_led", self.callback_control_lights)
        

    def callback_control_lights(self, request: LEDStateCommands.Request, response: LEDStateCommands.Response):
        self.led_states_internal = request.data
        response.response = True
        self.get_logger().info("light states updated")
        return response

    def callback_publish_light_states(self):
        msg = LEDStates()
        msg.data = self.led_states_internal
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LEDPanelNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()