#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Wrench
# from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Joy


class Motor:
    def __init__(self, x=0.0, y=0.0, z=0.0, ax=0.0, ay=0.0, az=0.0, gain=20):
        self.pos = np.array([x, y, z])
        self.for_dir = np.array([ax, ay, az])
        self.gain = gain  # a 100% le moteur envoie 20N


class Translator:
    def __init__(self):

        self.mot = [
            Motor(x=-0.6, y= 0   , z=-0.0925  , ax= 1.0, ay= 0.0       , az= 0.0),
            Motor(x=-0.6, y= 0.08, z= 0.04625 , ax= 1.0, ay= 0.0       , az= 0.0),
            Motor(x=-0.6, y=-0.08, z= 0.04625 , ax= 1.0, ay= 0.0       , az= 0.0),
            Motor(x=-0.5, y= 0.08, z=-0.04625 , ax= 0.0, ay=-3**0.5/2.0, az= 0.5),
            Motor(x=-0.5, y= 0   , z= 0.04625 , ax= 0.0, ay= 0.0       , az=-1.0),
            Motor(x=-0.5, y=-0.08, z=-0.04625 , ax= 0.0, ay= 3**0.5/2.0, az= 0.5)
        ]
        self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.wrench = Wrench()

    def updateCmd(self, msg):
        self.cmd = msg.data
        self.compute_wrench()
        pub.publish(self.wrench)

    def compute_wrench(self):

        # init
        self.wrench = Wrench()

        # forces
        for i in range(len(self.mot)):
            # for each axis
            cmd = self.mot[i].for_dir*self.cmd[i]*self.mot[i].gain
            self.wrench.force.x += cmd[0]
            self.wrench.force.y += cmd[1]
            self.wrench.force.z += cmd[2]

        # torques
        for i in range(len(self.mot)):
            force = self.mot[i].for_dir*self.cmd[i]*self.mot[i].gain
            torque = np.cross(self.mot[i].pos, force)
            self.wrench.torque.x += torque[0]
            self.wrench.torque.y += torque[1]
            self.wrench.torque.z += torque[2]

if __name__ == '__main__':

    rospy.init_node('wrench_pub')

    translator = Translator()

    sub_mot = rospy.Subscriber("cmd_mot", Float32MultiArray, translator.updateCmd)
    pub = rospy.Publisher("wrench", Wrench, queue_size=1)

    rospy.spin()
