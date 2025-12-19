#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import LEDStates

class LEDPanelNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("led_panel") # MODIFY NAME


def main(args=None):
    rclpy.init(args=args)
    node = LEDPanelNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()