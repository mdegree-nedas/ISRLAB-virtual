"""libturtlebot3 movement module"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Movement(Node):
    def __init__(self, logger, max_lin_vel, max_ang_vel):
        """constructor"""
        self.logger = logger
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        super().__init__('turtlebot3_movement')
        #  QoS setting or history depth (3rd param, self.create_publisher func)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    @staticmethod
    def __mov_init():
        """twist msg initializer"""
        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        return msg

    @staticmethod
    def __constr_vel(vel, lim):
        """speed constraints checker"""
        if vel < -lim:
            vel = -lim
        if vel > lim:
            vel = lim
        return vel

    def __go_init(self, vel):
        """move initializer"""
        if vel is None:
            vel = self.max_lin_vel
        vel = self.__constr_vel(vel, self.max_lin_vel)
        msg = self.__mov_init()
        return (msg, vel)

    def go_forward(self, vel=None):
        """move forward"""
        msg, vel = self.__go_init(vel)
        msg.angular.z = 0.0
        msg.linear.x = vel
        self.publisher_.publish(msg)
        self.logger.log('go_forward', msg.linear, msg.angular)

    def go_back(self, vel=None):
        """move back"""
        msg, vel = self.__go_init(vel)
        msg.angular.z = 0.0
        msg.linear.x = -vel
        self.publisher_.publish(msg)
        self.logger.log('go_back', msg.linear, msg.angular)

    def turn_right(self, vel=None):
        """turn right"""
        msg, vel = self.__go_init(vel)
        msg.linear.x = 0.0
        msg.angular.z = -vel
        self.publisher_.publish(msg)
        self.logger.log('turn_right', msg.linear, msg.angular)

    def turn_left(self, vel=None):
        """turn left"""
        msg, vel = self.__go_init(vel)
        msg.linear.x = 0.0
        msg.angular.z = vel
        self.publisher_.publish(msg)
        self.logger.log('turn_left', msg.linear, msg.angular)

    def stop(self):
        """stop"""
        msg = self.__mov_init()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.logger.log('stop', msg.linear, msg.angular)
