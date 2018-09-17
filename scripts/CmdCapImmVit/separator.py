#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
05/09/2018
@author Simon CHANU
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Twist, TwistStamped, Vector3
from std_msgs.msg import Float64
import dynamic_reconfigure.client
import tf

class PIDHandler:
    def __init__(self, ns):
        self.pub_setpoint = rospy.Publisher("{}/setpoint".format(ns), Float64, queue_size=1)
        self.pub_data = rospy.Publisher("{}/state".format(ns), Float64, queue_size=1)
        self.data = 0
        self.setpoint = 0
        self.dynconf_client = dynamic_reconfigure.client.Client("{}/pid".format(ns), timeout=1, config_callback=self.config_callback)

    def publish_data(self):
        self.pub_data.publish(self.data)

    def publish_setpoint(self):
        self.pub_setpoint.publish(self.setpoint)

    def publish(self):
        self.publish_setpoint()
        self.publish_data()

    def config_callback(self, config):
        rospy.loginfo("Config set to Kp={Kp}".format(**config))

class Separator():
    def __init__(self):

        self.pid = {
            'vel': PIDHandler('pid_vel'),
            'pitch': PIDHandler('pid_pitch'),
            'yaw': PIDHandler('pid_yaw')
        }

    def updateCmd(self, msg):
        self.pid['vel'].setpoint = msg.x
        self.pid['pitch'].setpoint = msg.y
        self.pid['yaw'].setpoint = msg.z

        # self.pid['pitch'].dynconf_client.update_configuration({"Kp":(msg.y*10)})

        for i in self.pid:
            self.pid[i].publish_setpoint()

    def updatePose(self, msg):

        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.pid['pitch'].data = euler[1]
        self.pid['yaw'].data = euler[2]


    def updateTwist(self, msg):
        self.pid['vel'].data = msg.twist.linear.x

    def publish_data(self):
        for i in self.pid:
            self.pid[i].publish_data()

    def spin(self):
        while not rospy.is_shutdown():
            self.publish_data()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('separator')
    rate = rospy.Rate(50)

    algo = Separator()

    sub_pose = rospy.Subscriber("real_pose", PoseStamped, algo.updatePose)
    sub_twist = rospy.Subscriber("real_twist", TwistStamped, algo.updateTwist)
    sub_cmd = rospy.Subscriber("cmd_autopilot", Vector3, algo.updateCmd)

    algo.spin()
