#!/usr/bin/env python
# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod

import rospy
import tf2_ros
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
import jaco_msgs.msg as jaco_msgs


class MicoInputGeneratorAbstract(object):
    __metaclass__ = ABCMeta
    _cartesian, _angular, _position, _velocity = 'cartesian', 'angular', 'position', 'velocity'

    def __init__(self, device_data_topic_name, device_data_topic_class, mico_input_data,
                 input_enabled=True, is_tf_listener=False):
        self._current_ee_pose = PoseStamped()
        self._topic_name = device_data_topic_name
        self._topic_class = device_data_topic_class
        self._mico_input_data = mico_input_data

        self.is_activated = False
        self._input_enabled = input_enabled
        self._is_tf_listener = is_tf_listener
        if self._is_tf_listener:
            self._tfBuffer = tf2_ros.Buffer()

        if self._mico_input_data.__class__ is PoseStamped:
            self._ctrl_frame, self._ctrl_method = self._cartesian, self._position
            self._data_publisher = actionlib.SimpleActionClient('mico_arm_driver/arm_pose/arm_pose',
                                                                jaco_msgs.ArmPoseAction)
            rospy.loginfo('waiting for ArmPoseAction server...')
            self._data_publisher.wait_for_server()
            rospy.loginfo('server connected')
            self._action_return_data = PoseStamped()

        elif self._mico_input_data.__class__ is jaco_msgs.JointAngles:
            self._ctrl_frame, self._ctrl_method = self._angular, self._position
            self._data_publisher = actionlib.SimpleActionClient('mico_arm_driver/joint_angles/arm_joint_angles',
                                                                jaco_msgs.ArmJointAnglesAction)
            rospy.loginfo('waiting for ArmJointAnglesAction server...')
            self._data_publisher.wait_for_server()
            rospy.loginfo('server connected')
            self._action_return_data = jaco_msgs.JointAngles()

        elif self._mico_input_data.__class__ is TwistStamped:
            self._ctrl_frame, self._ctrl_method = self._cartesian, self._velocity
            self._data_publisher = rospy.Publisher('mico_arm_driver/in/cartesian_velocity',
                                                   TwistStamped, queue_size=1)

        elif self._mico_input_data.__class__ is jaco_msgs.JointVelocity:
            self._ctrl_frame, self._ctrl_method = self._angular, self._velocity
            self._data_publisher = rospy.Publisher('mico_arm_driver/in/joint_velocity',
                                                   jaco_msgs.JointVelocity, queue_size=1)
        else:
            self._ctrl_frame, self._ctrl_method = None, None
            rospy.logerr('Invalid input class.')
            rospy.logerr('"mico_input_data" should be '
                         'PoseStamped, JointAngles(jaco_msg), TwistStamped or JointVelocity(jaco_msg) ')

    def activate(self):
        rospy.Subscriber("mico_arm_driver/out/tool_position", PoseStamped, self._ee_pose_callback)

        if self._is_tf_listener:
            tf2_ros.TransformListener(self._tfBuffer)
        else:
            rospy.Subscriber(self._topic_name, self._topic_class, self._device_data_callback)
        self.is_activated = True

    def _ee_pose_callback(self, ee_pose):
        self._current_ee_pose = ee_pose

    def _device_data_callback(self, device_data):
        self._generate_input_from_source_data(device_data)

    @abstractmethod
    def _generate_input_from_source_data(self, source_data):
        pass

    def publish_input_data(self):
        if self.is_activated is False:
            self.activate()

        if self._input_enabled is False:
            rospy.loginfo('input is not enabled')
            return False

        if self._is_tf_listener:
            self._generate_input_from_source_data(self._tfBuffer)

        if self._ctrl_method == self._position:
            if self._ctrl_frame == self._cartesian:
                goal = jaco_msgs.ArmPoseGoal(pose=self._mico_input_data)
            elif self._ctrl_frame == self._angular:
                goal = jaco_msgs.ArmJointAnglesGoal(angles=self._mico_input_data)
            else:
                return False

            print('send_goal')
            self._data_publisher.send_goal(goal)

            if self._data_publisher.wait_for_result(rospy.Duration(10.0)):
                rospy.loginfo('action achieved')
                self._action_return_data = self._data_publisher.get_result()
                return True
            else:
                rospy.loginfo('action timed-out')
                self._data_publisher.cancel_all_goals()
                return False

        elif self._ctrl_method == self._velocity:
            self._data_publisher.publish(self._mico_input_data)
            return True
        else:
            return False


class InputGenSample(MicoInputGeneratorAbstract):
    def __init__(self):
        self._mico_input_data = PoseStamped()
        self._input_enabled = False
        super(InputGenSample, self).__init__('pose_from_rqt', Pose, self._mico_input_data,
                                             input_enabled=self._input_enabled)

    def _generate_input_from_source_data(self, pose_data):
        def length_bw_poses(pose1, pose2):
            pos1, pos2, length = pose1.position, pose2.position, 0.0
            for axis in ['x', 'y', 'z']:
                length += pow(getattr(pos1, axis) - getattr(pos2, axis), 2.0)
            return pow(length, 0.5)

        if length_bw_poses(self._mico_input_data.pose, pose_data) > 0.5:
            self._input_enabled = True
            print(self._input_enabled)
            self._mico_input_data.header.frame_id = 'mico_api_origin'
            self._mico_input_data.header.stamp = rospy.Time.now()
            self._mico_input_data.pose = pose_data


class InputGenSample2(MicoInputGeneratorAbstract):
    def __init__(self):
        self._mico_input_data = PoseStamped()
        self._input_enabled = False
        super(InputGenSample2, self).__init__(None, None, self._mico_input_data,
                                              input_enabled=self._input_enabled, is_tf_listener=True)

    def _generate_input_from_source_data(self, tf_buffer):
        pass

#-------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('sample_input_generator', anonymous=True)
    rate_mgr = rospy.Rate(1)

    sample_input_gen = InputGenSample()
    # sample_input_gen = InputGenSample2()
    try:
        sample_input_gen.activate()

        rospy.loginfo('Run input_generator')
        while not rospy.is_shutdown():
            sample_input_gen.publish_input_data()
            rate_mgr.sleep()

        rospy.loginfo('Close input_generator')

    except rospy.ROSInterruptException, e:
        rospy.logwarn(str(e))
