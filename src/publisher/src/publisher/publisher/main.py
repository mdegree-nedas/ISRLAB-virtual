import rclpy

from publisher.libplanner.planner import Planner
from publisher.libturtlebot3.turtlebot3 import TurtleBot3


def main(args=None):
    """main"""
    rclpy.init(args=args)
    turtlebot3 = TurtleBot3()
    planner = Planner(turtlebot3)
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
