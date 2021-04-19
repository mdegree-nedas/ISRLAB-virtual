"""libturtlebot3 sensor module"""
import rclpy
from publisher.libturtlebot3.logger import Logger
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Sensor(Node):
    def __init__(self, logger):
        """constructor"""
        super().__init__('turtlebot3_sensor')
        self.logger = logger
        self.subscriber_ = self.create_subscription(
                LaserScan,
                'turtlebot3_laserscan/out',
                self.__subscribe_callback,
                QoSProfile(depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT))

    def __subscribe_callback(self, msg):
        """subscribe callback"""
        self._laser_data = msg

    def laser_scan(self):
        """get laser scan data"""
        rclpy.spin_once(self)
        #  self.logger.log('laser_scan', self._laser_data)
        return self._laser_data

    def laser_scan_ranges_front(self):
        """get laser scan data ranges front"""
        rclpy.spin_once(self)
        # return self._laser_data.ranges[-44:] + self._laser_data.ranges[:44]
        ranges_front_left = self._laser_data.ranges[:45]
        ranges_front_left.reverse()
        ranges_front_right = self._laser_data.ranges[-45:]
        ranges_front_right.reverse()
        return ranges_front_left + ranges_front_right

    def laser_scan_ranges_back(self):
        """get laser scan data ranges back"""
        rclpy.spin_once(self)
        return self._laser_data.ranges[134:222]

    def laser_scan_ranges_left(self):
        """get laser scan data ranges left"""
        rclpy.spin_once(self)
        return self._laser_data.ranges[44:132]

    def laser_scan_ranges_right(self):
        """get laser scan data ranges right"""
        rclpy.spin_once(self)
        return self._laser_data.ranges[-132:-44]

    def laser_scan_ranges_front_left(self):
        """get laser scan data ranges front left"""
        rclpy.spin_once(self)
        return self._laser_data.ranges[0:89]

    def laser_scan_ranges_front_right(self):
        """get laser scan data ranges front right"""
        rclpy.spin_once(self)
        return self._laser_data.ranges[270:359]

    def laser_scan_ranges_back_left(self):
        """get laser scan data ranges back left"""
        rclpy.spin_once(self)
        return self._laser_data.ranges[90:179]

    def laser_scan_ranges_back_right(self):
        """get laser scan data ranges back right"""
        rclpy.spin_once(self)
        return self._laser_data.ranges[180:269]

