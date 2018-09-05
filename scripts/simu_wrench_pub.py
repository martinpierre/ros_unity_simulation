#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy

class Motor:
    def __init__(self, x=0.0, y=0.0, z=0.0, ax=0.0, ay=0.0, az=0.0):
        self.pos = np.array([x, y, z])
        self.for_dir = np.array([ax, ay, az])

class Translator:
    def __init__(self):

        self.mot = [
            Motor(x=-0.6, y= 0   , z=-0.0925  , ax= 1.0, ay= 0.0       , az= 0.0),
            Motor(x=-0.6, y= 0.08, z= 0.04625 , ax= 1.0, ay= 0.0       , az= 0.0),
            Motor(x=-0.6, y=-0.08, z= 0.04625 , ax= 1.0, ay= 0.0       , az= 0.0),
            Motor(x=-0.5, y= 0.08, z=-0.04625 , ax= 0.0, ay= 3**0.5/2.0, az=-0.5),
            Motor(x=-0.5, y= 0   , z= 0.04625 , ax= 0.0, ay= 0.0       , az= 1.0),
            Motor(x=-0.5, y=-0.08, z=-0.04625 , ax= 0.0, ay= 3**0.5/2.0, az=-0.5)
        ]
        self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.wrench = Wrench()

    def updateCmd(self, msg):
        self.cmd = msg.data
        self.compute_wrench()

    def compute_wrench(self):

        # init
        self.wrench = Wrench()

        # forces
        for i in self.mot:
            # for each axis
            cmd = self.mot[i].for_dir*self.cmd[i]
            self.wrench.force.x += cmd[0]
            self.wrench.force.y += cmd[1]
            self.wrench.force.z += cmd[2]

        # torques
        for i in self.mot:
            force = self.mot[i].for_dir*self.cmd[i]
            torque = self.pos.cross(force)
            self.wrench.force.x += torque[0]
            self.wrench.force.y += torque[1]
            self.wrench.force.z += torque[2]

if __name__ == '__main__':

    rospy.init_node('wrench_pub')

    translator = Translator()

    sub_mot = rospy.Subscriber("cmd_mot", Float32MultiArray, translator.compute_wrench)
