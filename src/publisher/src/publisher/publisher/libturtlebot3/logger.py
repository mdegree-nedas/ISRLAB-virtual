"""libturtlebot3 logger module"""
import rclpy
from rclpy.node import Node


class Logger(Node):
    def __init__(self):
        """constructor"""
        super().__init__('turtlebot3_logger')

    def log(self, cmd, *argv):
        """general log method"""
        self.get_logger().info(cmd)
        for msg in argv:
            self.get_logger().info(str(msg))
