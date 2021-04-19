"""libturtlebot3 main module"""
import publisher.libturtlebot3.common as Common
from publisher.libturtlebot3.logger import Logger
from publisher.libturtlebot3.movement import Movement
from publisher.libturtlebot3.sensor import Sensor


class TurtleBot3:
    def __init__(self, max_lin_vel=Common.MAX_LIN_VEL,
            max_ang_vel=Common.MAX_ANG_VEL):
        """constructor"""
        self.logger = Logger()
        self.movement = Movement(self.logger, max_lin_vel, max_ang_vel)
        self.sensor = Sensor(self.logger)
