#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import os, time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QSize
from python_qt_binding.QtGui import QTableWidget, QTableWidgetItem, QWidget, QPalette, QBrush, QAbstractItemView, QFont
import rospkg, rospy
from jaco_msgs.srv import Start as MicoStart, Stop as MicoStop, HomeArm as MicoHome


class MicoButtonsWidget(QWidget):
    def __init__(self, widget):
        super(MicoButtonsWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('mico_controller'), 'resource', 'MicoButtons.ui')
        loadUi(ui_file, self)

        self.start_arm_srv = rospy.ServiceProxy('mico_arm_driver/in/start', MicoStart)
        self.stop_arm_srv = rospy.ServiceProxy('mico_arm_driver/in/stop', MicoStop)
        self.home_arm_srv = rospy.ServiceProxy('mico_arm_driver/in/home_arm', MicoHome)

        self._updateTimer = QTimer(self)
        self._updateTimer.timeout.connect(self.timeout_callback)

    def start(self):
        self._updateTimer.start(1000)  # loop rate[ms]

    def stop(self):
        self._updateTimer.stop()

    def timeout_callback(self):
        pass

    # rqt override
    def save_settings(self, plugin_settings, instance_settings):
        pass
        # instance_settings.set_value('topic_name', self._topic_name)

    def restore_settings(self, plugin_settings, instance_settings):
        pass
        # topic_name = instance_settings.value('topic_name')
        # try:
        # self._topic_name = eval(topic_name)
        # except Exception:
        # self._topic_name = self.TOPIC_NAME

    def shutdown_plugin(self):
        self.stop()

    @Slot()
    def on_qt_start_btn_clicked(self):
        rospy.loginfo(self.start_arm_srv())

    @Slot()
    def on_qt_stop_btn_clicked(self):
        rospy.loginfo(self.stop_arm_srv())

    @Slot()
    def on_qt_home_btn_clicked(self):
        rospy.loginfo(self.home_arm_srv())


