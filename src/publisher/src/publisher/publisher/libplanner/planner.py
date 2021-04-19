"""libplanner main module"""

import sys
import time

import rclpy
from rclpy.node import Node

TIMER = 0.10

FORWARD_CLOCK = 1.0
STOP_CLOCK = 3.0

NEAR_THRESHOLD = 0.50
SEC_THRESHOLD = 1.0

class Planner(Node):
    def __init__(self, node, timer=TIMER):
        """constructor"""
        super().__init__('planner')
        self.node_ = node
        self.timer_ = timer
        self.clock_ = self.timer_
        self.prev_mode_, self.mode_ = 'stop', 'idle'
        self.collision_array_ = None
        self.collision_array_max_index_, self.collision_array_min_index_ = None, None
        self.laser_data_front_ = None
        self.laser_data_left_ = None
        self.laser_data_right_ = None
        self.laser_data_view_left_ = None
        self.laser_data_view_front_ = None
        self.laser_data_view_right_ = None
        self.loop_ = self.create_timer(self.timer_, self.__loop_callback_)

    def __get_clock(self):
        return self.clock_

    def __set_clock(self, value):
        self.clock_ = value

    def __clock_sleep(self):
        time.sleep(self.__get_clock())

    def __loop_callback_(self):
        """planner loop"""
        self.__clock_sleep()

        self.laser_data_front_, \
        self.laser_data_left_, \
        self.laser_data_right_, \
        self.laser_data_view_left_, \
        self.laser_data_view_front_, \
        self.laser_data_view_right_ = self.__laser_matrix_processing()

        self.__mode_supervisor()
        self.__mov_supervisor()
        self.__set_prev_mode()
        self.__log()

    def __laser_matrix_processing(self):
        """get laser data from node"""
        return [self.node_.sensor.laser_scan_ranges_front(),
                self.node_.sensor.laser_scan_ranges_left(),
                self.node_.sensor.laser_scan_ranges_right(),
                self.node_.sensor.laser_scan_ranges_front()[:30],
                self.node_.sensor.laser_scan_ranges_front()[31:59],
                self.node_.sensor.laser_scan_ranges_front()[60:]]

    def __mode_supervisor(self):
        self.__view_processing()
        self.mode_ = self.__mode_selector()

    def __view_processing(self):
        self.collision_array_ = [min(self.laser_data_view_left_),
                                 min(self.laser_data_view_front_),
                                 min(self.laser_data_view_right_)]
        self.collision_array_min_index_ = \
            self.collision_array_.index(min(self.collision_array_))
        self.collision_array_max_index_ = \
            self.collision_array_.index(max(self.collision_array_))

    def __mode_selector(self):
        if self.prev_mode_ != 'forward' and \
                self.collision_array_[1] > SEC_THRESHOLD and \
                self.collision_array_[0] > NEAR_THRESHOLD and \
                self.collision_array_[2] > NEAR_THRESHOLD:
            return 'forward'
        if self.prev_mode_ == 'forward' and \
                self.collision_array_[1] > SEC_THRESHOLD and \
                self.collision_array_[0] > NEAR_THRESHOLD and \
                self.collision_array_[2] > NEAR_THRESHOLD:
            return 'forward'
        if self.prev_mode_ == 'forward' and \
                (self.collision_array_[1] <= SEC_THRESHOLD or \
                self.collision_array_[0] <= NEAR_THRESHOLD or \
                self.collision_array_[2] <= NEAR_THRESHOLD):
            return 'stop'
        if self.prev_mode_ == 'stop':
            if self.collision_array_min_index_ == 0:
                return 'turn_right'
            if self.collision_array_min_index_ == 2:
                return 'turn_left'
            if self.collision_array_min_index_ == 1:
                if self.collision_array_max_index_ == 0:
                    return 'turn_left'
                if self.collision_array_max_index_ == 2:
                    return 'turn_right'
        if self.prev_mode_ == 'turn_left' or self.prev_mode_ == 'turn_right':
            if self.collision_array_[1] > SEC_THRESHOLD:
                return 'forward'
            return self.prev_mode_
        sys.exit('unknown dir')

    def __mov_supervisor(self):
        if self.mode_ == 'forward' and self.prev_mode_ != 'forward':
            self.__forward_mode()
        if self.mode_ == 'stop' and self.prev_mode_ != 'stop':
            self.__stop_mode()
        if self.mode_ == 'turn_left' and self.prev_mode_ != 'turn_left':
            self.__turn_left_routine()
        if self.mode_ == 'turn_right' and self.prev_mode_ != 'turn_right':
            self.__turn_right_routine()

    def __set_prev_mode(self):
        self.prev_mode_ = self.mode_

    def __forward_mode(self):
        self.__set_clock(FORWARD_CLOCK)
        self.__forward_routine()

    def __forward_routine(self,):
        self.node_.movement.go_forward()

    def __stop_mode(self):
        self.__set_clock(STOP_CLOCK)
        self.__stop_routine()

    def __stop_routine(self):
        self.node_.movement.stop()

    def __turn_left_routine(self):
        self.node_.movement.turn_left()

    def __turn_right_routine(self):
        self.node_.movement.turn_right()

    def __log(self):
        """logger"""
        self.get_logger().info('--------------------')
        self.get_logger().info(' prev mode: ' + str(self.prev_mode_))
        self.get_logger().info('      mode: ' + str(self.mode_))
        self.get_logger().info('      left: ' + str(self.collision_array_[0]) + ' ' +
                str(self.collision_array_[0] > NEAR_THRESHOLD))
        self.get_logger().info('     front: ' + str(self.collision_array_[1]) + ' ' +
                str(self.collision_array_[1] > SEC_THRESHOLD))
        self.get_logger().info('     right: ' + str(self.collision_array_[2]) + ' ' +
                str(self.collision_array_[2] > NEAR_THRESHOLD))
