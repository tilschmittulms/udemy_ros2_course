#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import numpy as np
from turtlesim.srv import Kill
import math
from geometry_msgs.msg import Twist

class TurtleController(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_controller") # MODIFY NAME
        self.kill_client_= self.create_client(Kill, "kill")

        self.pose1 = None
        self.posex = None
        self.angle = None
        self.turtle = 2

        self.is_killing = False

        self.pose1_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_turtle1_pose, 10)
        self.posex_subscriber_ = None
        self.subscribe_to_next_turtle()
        self.vel = None
        self.vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.get_logger().info("turtle controller has been started")

    def subscribe_to_next_turtle(self):
        self.posex = None
        if self.posex_subscriber_:
            self.destroy_subscription(self.posex_subscriber_)
        self.posex_subscriber_ = self.create_subscription(Pose, "/turtle" + str(self.turtle) + "/pose", self.callback_turtlex_pose, 10)

    def callback_turtlex_pose(self, msg: Pose):
        self.posex = np.array([msg.x, msg.y])

    def callback_turtle1_pose(self, msg: Pose):
        self.pose1 = np.array([msg.x, msg.y])
        self.angle = msg.theta
        self.compare_turtles()

    def compare_turtles(self):
        if self.pose1 is None or self.posex is None or self.is_killing:
            return

        distance = np.linalg.norm(self.pose1 - self.posex)
        if distance < 0.5:
            self.posex = None
            self.is_killing = True
            self.call_turtle_kill()

        else:
            magnitude = np.linalg.norm(self.pose1 - self.posex)
            diff_x = self.posex[0] - self.pose1[0]
            diff_y = self.posex[1] - self.pose1[1]
            steering_error = math.atan2(diff_y, diff_x) - self.angle
            if steering_error > math.pi:
                steering_error -= 2*math.pi
            elif steering_error < -math.pi:
                steering_error += 2*math.pi

            if abs(steering_error) > 0.1:
                #self.get_logger().info("it is at angle: " + str(steering_error) + "and distance: " + str(magnitude))
                self.vel = Twist()
                #self.vel.linear.x = magnitude/5.0
                self.vel.angular.z = 2*steering_error
                self.vel_publisher_.publish(self.vel)
            else:
                self.vel = Twist()
                self.vel.linear.x = 10.0
                self.vel_publisher_.publish(self.vel)

    def call_turtle_kill(self):
        if not self.kill_client_.wait_for_service(1.0):
            return

        request = Kill.Request()
        request.name = "turtle" + str(self.turtle)

        future = self.kill_client_.call_async(request)
        future.add_done_callback(self.callback_finished_kill)

    def callback_finished_kill(self, future):
        try:
            future.result()
            self.turtle += 1
            self.get_logger().info("moving on to turtle " + str(self.turtle))
            self.subscribe_to_next_turtle()
        except Exception as e:
            self.get_logger().error("Kill failed")
        finally:
            self.is_killing = False



def main(args=None):
    rclpy.init(args=args)
    node = TurtleController() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()