#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
05/09/2018
@author Simon CHANU
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3, Pose
from std_msgs.msg import Float32MultiArray, Bool, Float64, MultiArrayDimension
import tf

class PIDHandler:
    def __init__(self, ns):
        self.sub_output = rospy.Subscriber("{}/control_effort".format(ns), Float64, self.update_output)
        self.pub_enable = rospy.Publisher("{}/pid_enable".format(ns), Bool, queue_size=1)
        self.output = 0
        self._enable = True

    def enable(self, enable):
        self._enable = enable
        self.pub_enable.publish(self._enable)

    def update_output(self, msg):
        self.output = msg.data

class Autopilot():
    def __init__(self):

        self.pid = {
            'vel': PIDHandler('pid_vel'),
            'pitch': PIDHandler('pid_pitch'),
            'yaw': PIDHandler('pid_yaw')
        }

        self.pose = Pose()
        self.cmd = Float32MultiArray()
        self.cmd.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._enable = True

    def enable(self, msg):
        self._enable = msg.data
        for i in self.pid:
            if not self._enable:
                self.pid[i].output = 0.0
            self.pid[i].enable(self._enable)

    def updatePose(self, msg):
        self.pose = msg.pose
        # quaternion = (
        #     msg.pose.orientation.x,
        #     msg.pose.orientation.y,
        #     msg.pose.orientation.z,
        #     msg.pose.orientation.w)
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # self.pid_pitch.data = euler[1]
        # self.pid_yaw.data = euler[2]

    def computeCmd(self):
        rospy.loginfo("___")

        # Application du logarithme sur la commande de pitch
        # pitch_cmd = min(max(np.exp(self.pid['pitch'].output)-1, -1), 1)
        pitch_cmd = self.pid['pitch'].output

        cmd = np.array([self.pid['vel'].output, pitch_cmd, self.pid['yaw'].output])
        rospy.loginfo("cmds : {}".format(cmd))

        r3s2 = 3**0.5/2.0
        # matrice de commande
        m5y = 0.5*np.sign(cmd[2])
        #                vel | pitch | yaw
        M = np.array([[  1.0 ,  0.0 ,  0.0 ],  # mot 1
                      [  1.0 ,  0.0 ,  0.0 ],  # mot 2
                      [  1.0 ,  0.0 ,  0.0 ],  # mot 3
                      [  0.0 ,  0.5 , -1.0 ],  # mot 4
                      [  0.0 , -1.0 ,  m5y ],  # mot 5
                      [  0.0 ,  0.5 ,  1.0 ]]) # mot 6

        # Application de la matrice sur la commande
        unnormed_cmd = np.dot(M, cmd)

        unnormed_cmd[3:] = np.maximum(np.zeros(3), unnormed_cmd[3:])  # secu anti-reverse pour les tuyères

        if np.max(unnormed_cmd)!=0.0 and np.max(np.abs(unnormed_cmd))>1.0:
            # self.cmd.data = unnormed_cmd.tolist()
            self.cmd.data = (unnormed_cmd/np.max(np.abs(unnormed_cmd))).tolist()
        else:
            self.cmd.data = unnormed_cmd.tolist()

        # debug
        for i in range(len(self.cmd.data)):
            rospy.loginfo("mot {} : {}".format(i+1, self.cmd.data[i]))
        pub_cmd_mot.publish(self.cmd)

    def spin(self):
        while not rospy.is_shutdown():
            if self._enable:
                self.computeCmd()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('autopilot')
    rate = rospy.Rate(10)

    algo = Autopilot()

    sub_pose = rospy.Subscriber("real_pose", PoseStamped, algo.updatePose)
    pub_cmd_mot = rospy.Publisher("cmd_mot", Float32MultiArray, queue_size=1)
    pub_wrench = rospy.Publisher("wrench", WrenchStamped, queue_size=1)
    sub_cmd = rospy.Subscriber("autopilot/enable", Bool, algo.enable)

    algo.spin()
