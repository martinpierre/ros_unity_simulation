#!/usr/bin/env python
"""
05/09/2018
@author Simon CHANU
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Bool, Float32, MultiArrayDimension
import tf

class ImmFollow():
    def __init__(self):
        self.z = 0.0  # axe vers le bas
        self.pitch = 0.0

        self.enable = False

        self.target = 0.0
        self.cmd = Float32MultiArray()
        # self.cmd.layout.append(MultiArrayDimension())
        # self.cmd.layout.dim[0].label = "force"
        # self.cmd.layout.dim[0].size = 6
        # self.cmd.layout.dim[0].stride = 6
        self.cmd.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def switch(self, msg):
        self.enable = msg.data

    def updateTarget(self, msg):
        self.target = msg.data

    def updatePose(self, msg):
        # update values
        self.z = msg.pose.position.z
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.pitch = euler[1]

        if self.enable:
            # compute command
            self.compute_cmd()

            # publish command
            pub_wrench.publish(self.cmd)

    def compute_wanted_pitch(self):
        dist_react = 10.0
        e = (self.z - self.target)/2.0
        target_pitch = np.arctan(e)

        return target_pitch

    def compute_cmd(self):
        target_pitch = self.compute_wanted_pitch()

        rospy.loginfo('target pitch : {}'.format(target_pitch))

        # Test up/down
        # up   = [-1,  0.5,  0.5, 0.0, 1.0, 0.0]  # dec z
        # down = [ 1, -0.5, -0.5, 0.5, 0.0, 0.5]  # inc z

        # Test dive/surface
        dive    = [0.5, 0.5, 0.5, 0, 1, 0]
        surface = [0.5, 0.5, 0.5, 1, 0, 1]

        cmd = []
        if target_pitch>0:
            cmd = surface
        elif target_pitch<0: 
            cmd = dive
        else:
            cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        gain = np.abs(2*target_pitch/np.pi)
        gain_display= gain*(180*np.pi/np.pi)
        rospy.loginfo('gain : {}'.format(gain_display))
        self.cmd.data = (gain*np.array(cmd)).tolist()

if __name__ == '__main__':
    rospy.init_node('follow_immersion_algo')

    algo = ImmFollow()

    pub_wrench = rospy.Publisher("cmd_mot", Float32MultiArray, queue_size=1)
    sub_pose = rospy.Subscriber("test", PoseStamped, algo.updatePose)
    sub_target = rospy.Subscriber("imm_target", Float32, algo.updateTarget)
    sub_enable = rospy.Subscriber("enable", Bool, algo.switch)

    rospy.spin()
