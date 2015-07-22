#!/usr/bin/env python
# -*- coding: utf-8 -*-

from abc import ABCMeta, abstractmethod

import rospy
from sensor_msgs.msg import Joy
from jaco_msgs.srv import Start, Stop, HomeArm
from mico_js_controller.srv import InputMethodSwitchService

from src.robot_controller.mico_controller.src.joy_data import MyJoyData as JoyData


class ServiceCaller(object):
    __metaclass__ = ABCMeta
    start_arm_servicename = 'start_arm'
    stop_arm_servicename = 'stop_arm'
    home_arm_servicename = 'home_arm'
    switch_mode_servicename = 'switch_mode'

    def __init__(self, data_from_device_topic_name, data_from_device_topic_class):
        self._topic_name = data_from_device_topic_name
        self._topic_class = data_from_device_topic_class

        rospy.loginfo('start to wait for services')
        rospy.wait_for_service(self.start_arm_servicename)
        rospy.wait_for_service(self.stop_arm_servicename)
        rospy.wait_for_service(self.home_arm_servicename)
        rospy.wait_for_service(self.switch_mode_servicename)

        self.start_arm_srv = rospy.ServiceProxy(self.start_arm_servicename, Start)
        self.stop_arm_srv = rospy.ServiceProxy(self.stop_arm_servicename, Stop)
        self.home_arm_srv = rospy.ServiceProxy(self.home_arm_servicename, HomeArm)
        self.switch_mode_srv = rospy.ServiceProxy(self.switch_mode_servicename, InputMethodSwitchService)

    def activate(self):
        self.sub_data_from_device = rospy.Subscriber(self._topic_name, self._topic_class,
                                                     self._data_from_device_callback)

    def _data_from_device_callback(self, data):
        rospy.logdebug('subscribe')
        return self._call_service(data)

    @abstractmethod
    def _call_service(self, data):
        pass


class JoyStickSrvCaller(ServiceCaller):
    def __init__(self):
        super(JoyStickSrvCaller, self).__init__('joystick', Joy)
        self._switched_mode_time = rospy.Time.now()
        self._called_mico_service_time = rospy.Time.now()
        self._is_moving = True
        self._joy_data = JoyData()

        self.twist_pos_switch_cnt = 0
        self.pos_ori_switch_cnt = 0

    def _call_service(self, joy_data):
        self._joy_data.set_data(joy_data.axes, joy_data.buttons)
        rospy.logdebug(self._joy_data.is_pressed('select'), self._joy_data.is_pressed('start'))

        current_time = rospy.Time.now()
        if (current_time.secs - self._switched_mode_time.secs > 3.0) \
                and (self._joy_data.is_pressed('R3') is True):
            self.pos_ori_switch_cnt += 1
            self.switch_mode_srv((self.twist_pos_switch_cnt % 2) + 1, (self.pos_ori_switch_cnt % 2) + 1)
            self._switched_mode_time = current_time

        if current_time.secs - self._called_mico_service_time.secs > 3.0:
            if self._joy_data.is_pressed('start', is_only=True) and self._is_moving is False:
                rospy.loginfo(self.start_arm_srv())
                self._is_moving = True
                self._called_mico_service_time = current_time
            elif self._joy_data.is_pressed('select', is_only=True) is True:
                rospy.loginfo('calling HomeArm service...')
                rospy.loginfo(self.home_arm_srv())
                self._called_mico_service_time = current_time

        if self._joy_data.is_pressed(['select', 'start'], is_only=True):
            rospy.loginfo(self.stop_arm_srv())
            self._is_moving = False

        return None


# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node('service_caller')
    js_srv_caller = JoyStickSrvCaller()

    try:
        js_srv_caller.activate()
        rospy.loginfo('Run service_caller')
        rospy.spin()
    except rospy.ServiceException, e:
        js_srv_caller.stop_arm_srv()
        rospy.logwarn(str(e))
















