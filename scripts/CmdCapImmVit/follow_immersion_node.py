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
        dist_react = 1.0
        e = (self.z - self.target)/dist_react
        target_pitch = np.arctan(e)
        print(e)
        return target_pitch

    def compute_cmd(self):
        target_pitch = self.compute_wanted_pitch()
        pitch_display = target_pitch * 180/(np.pi)
        rospy.loginfo('target pitch (deg) : {}'.format(pitch_display))

        # Test up/down
        # up   = [-1,  0.5,  0.5, 0.0, 1.0, 0.0]  # dec z
        # down = [ 1, -0.5, -0.5, 0.5, 0.0, 0.5]  # inc z

        # Test dive/surface
        # dive    = [1, 1, 1, 0, 0.5, 0]
        # surface = [1, 1, 1, 0.5, 0, 0.5]

        # gere un gain en fonction de l'erreur d'assiette
        rel_target_pitch = target_pitch - self.pitch
        rospy.loginfo('rel_target_pitch (deg) : {}'.format(rel_target_pitch * 180/(np.pi)))
        target_gain = np.abs(np.arctan(rel_target_pitch*2.0)/np.pi*2.0)
        rospy.loginfo('target_gain : {}'.format(targimmet_gain))

        # Applique le gain a la commande
        cmd_gain = target_gain*0.2
        dive_cmd = max(np.sign(-rel_target_pitch)*cmd_gain, 0)
        surface_cmd = max(np.sign(rel_target_pitch)*cmd_gain, 0)
        cmd = [0.5, 0.5, 0.5,
               surface_cmd, dive_cmd, surface_cmd]
        rospy.loginfo('cmd : {}'.format(cmd))

        # gere un gain en fonction de l'assiette nulle
        gain = np.abs(2*(target_pitch)/np.pi)

        rospy.loginfo('gain : {}'.format(gain))
        self.cmd.data = (gain*np.array(cmd)).tolist()

if __name__ == '__main__':
    rospy.init_node('follow_immersion_algo')

    algo = ImmFollow()

    pub_wrench = rospy.Publisher("cmd_mot", Float32MultiArray, queue_size=1)
    sub_pose = rospy.Subscriber("real_pose", PoseStamped, algo.updatePose)
    sub_target = rospy.Subscriber("imm_target", Float32, algo.updateTarget)
    sub_enable = rospy.Subscriber("enable", Bool, algo.switch)

    rospy.spin()
