#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from sensor_msgs.msg import Joy

from joy_data import MyJoyData as JoyData
from src.robot_controller.mico_controller.src.mico_input_gen_abstract import MicoInputGeneratorAbstract


# remap-able
DEFAULT_NODE_NAME = 'js_input_gen'


class JoyStickInputGenerator(MicoInputGeneratorAbstract):
    def __init__(self):
        self._joy_data = JoyData()
        self._mico_input_data = TwistStamped()
        self._is_linear_control = True
        self._is_able_to_change_mode = False

        self._scale_twist = Twist(linear=Vector3(x=1.0, y=1.0, z=1.0),
                                  angular=Vector3(x=1.0, y=1.0, z=1.0))
        super(JoyStickInputGenerator, self).__init__('joy', Joy,
                                                     self._mico_input_data)

    def temp(self):
        pos = self._current_ee_pose.pose.position
        print(str(pow(pow(pos.x, 2.0) + pow(pos.y, 2.0), 0.5))
              + ' ' + str(pow(pow(pos.x, 2.0) + pow(pos.y, 2.0) + pow(pos.z-0.25, 2.0), 0.5)))

    def _device_data_callback(self, joy_data):
        self.temp()
        self._joy_data.set_data(joy_data.axes, joy_data.buttons)
        self._mico_input_data.header.stamp = rospy.Time.now()

        if self._joy_data.is_pressed('R3', is_only=True):
            self._is_able_to_change_mode = True
        elif self._is_able_to_change_mode:
            self._is_able_to_change_mode = False
            self._is_linear_control = not self._is_linear_control
            mode_str = 'position' if self._is_linear_control else 'orientation'
            rospy.loginfo(mode_str + ' mode now')

        if self._is_linear_control:
            self._mico_input_data.twist.linear.x = self._scale_twist.linear.x * self._joy_data.rstick.x
            self._mico_input_data.twist.linear.y = self._scale_twist.linear.y * self._joy_data.rstick.y
            self._mico_input_data.twist.linear.z = self._scale_twist.linear.z * self._joy_data.dpad.x
            self._mico_input_data.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        else:
            self._mico_input_data.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
            self._mico_input_data.twist.angular.x = self._scale_twist.angular.x * self._joy_data.rstick.y
            self._mico_input_data.twist.angular.y = self._scale_twist.angular.y * self._joy_data.dpad.x
            self._mico_input_data.twist.angular.z = self._scale_twist.angular.z * self._joy_data.rstick.x

# -------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('joystick_input_generator', anonymous=True)
    rate_mgr = rospy.Rate(10)  # [Hz]

    js_input_gen = JoyStickInputGenerator()
    try:
        js_input_gen.activate()

        rospy.loginfo('Run input_generator')
        while not rospy.is_shutdown():
            js_input_gen.publish_input_data()
            rate_mgr.sleep()

        rospy.loginfo('Close input_generator')

    except rospy.ROSInterruptException, e:
        rospy.logwarn(str(e))