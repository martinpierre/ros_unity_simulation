#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
05/09/2018
@author Simon CHANU
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Twist, TwistStamped, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import Float64
import tf


def compute_pose_distance(pose_obj, pose_origin):
    dist = Pose()
    dist.position.x = pose_obj.position.x - pose_origin.position.x
    dist.position.y = pose_obj.position.y - pose_origin.position.y
    dist.position.z = pose_obj.position.z - pose_origin.position.z
    return dist


def compute_pose_norm(pose):
    return np.linalg.norm([pose.position.x, pose.position.y, pose.position.z])


def angle_rad(a1, a2):
    return np.mod(a1 + a2 + 3*np.pi, 2*np.pi) - np.pi


class Guide():
    def __init__(self):
        self.path = [Pose()]
        self.wp_nb = 0
        self.twist = Twist()
        self.pose = Pose()
        self.cmd = Vector3()

        self.pitch = 0.0
        self.yaw = 0.0

        self.dist = compute_pose_distance(self.path[self.wp_nb], self.pose)
        self.dist_norm = compute_pose_norm(self.dist)
        self.dist_gnd = np.linalg.norm([self.dist.position.x, self.dist.position.y])

    def updateDist(self):
        self.dist = compute_pose_distance(self.path[self.wp_nb], self.pose)
        self.dist_norm = compute_pose_norm(self.dist)
        self.dist_gnd = np.linalg.norm([self.dist.position.x, self.dist.position.y])

    def updateWps(self, msg):
        self.path = msg.poses
        for i in range(len(self.path)):
            self.path[i] = self.path[i].pose

        self.wp_nb = 0

        self.updateDist()
        rospy.loginfo("Received new path")

    def updatePose(self, msg):
        self.pose = msg.pose
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.pitch = euler[1]
        self.yaw = euler[2]

        self.updateDist()

    def updateTwist(self, msg):
        self.twist = msg.twist

    def check_harvest_wp(self):
        return (self.dist_norm < 3.0)

    def harvest_wp(self):
        self.wp_nb = (self.wp_nb + 1) % len(self.path)
        self.updateDist()

    def follow_wp(self):
        wp = self.path[self.wp_nb].position
        x_vec = wp.x - self.pose.position.x
        y_vec = wp.y - self.pose.position.y
        z_vec = wp.z - self.pose.position.z

        rospy.loginfo("x_vec = {}".format(x_vec))
        rospy.loginfo("y_vec = {}".format(y_vec))
        rospy.loginfo("dist_gnd = {}".format(y_vec))

        obj_yaw = -np.arctan2(self.dist.position.y, self.dist.position.x)
        obj_pitch = -np.arctan2(self.dist.position.z, self.dist_gnd)
        vel = self.compute_vel(obj_yaw, obj_pitch)

        # Commande en coordonnées globales
        self.cmd.y = obj_pitch
        self.cmd.z = obj_yaw
        self.cmd.x = vel


    def compute_vel(self, obj_yaw, obj_pitch):
        d_max = 4.0  # m
        err_angle_max = 1.57  # rad
        cmd_max_speed = 1.5  # m/s

        # get angle error
        err_yaw = angle_rad(obj_yaw, -self.yaw)
        err_pitch = angle_rad(obj_pitch, -self.pitch)
        errorAngle = (err_pitch**2 + err_yaw**2)**0.5
        # rospy.loginfo("self.yaw = {}".format(self.yaw))
        # rospy.loginfo("self.pitch = {}".format(self.pitch))
        # rospy.loginfo("err_yaw = {}".format(errorAngle))
        # rospy.loginfo("err_pitch = {}".format(err_pitch))
        # rospy.loginfo("errorAngle = {}".format(errorAngle))

        errorAngle = min(abs(errorAngle), err_angle_max)
        distance = min(self.dist_norm, d_max)

        wantedVel = distance/d_max * (err_angle_max - errorAngle) * cmd_max_speed / err_angle_max
        rospy.loginfo("wantedVel = {}".format(wantedVel))
        return wantedVel


    def follow_line(self):
        if self.wp_nb > 0:
            wp_init = self.path[self.wp_nb - 1]
            wp_fin = self.path[self.wp_nb]
        else:
            rospy.loginfo("First wp, switching to follow_wp")
            self.follow_wp()
            return

        # Si le bateau dépasse la ligne
        if compute_pose_norm(compute_pose_distance(wp_init, wp_fin)) < compute_pose_norm(compute_pose_distance(wp_init, self.pose)):
            rospy.loginfo("Line overreached, switching to follow_wp")
            self.follow_wp()
            return

        wantedHead = self.follow_line_plane(wp_init, wp_fin, 'xy')
        # wantedPitch = self.follow_line_plane(wp_init, wp_fin, 'xz')
        wantedPitch = 0.0
        wantedVel = self.compute_vel(wantedHead, wantedPitch)


    def follow_line_plane(self, wp_init, wp_fin, plane):

        # Redefinition des entrees
        if plane == 'xy':
            ax = wp_init.position.x
            ay = wp_init.position.y
            bx = wp_fin.position.x
            by = wp_fin.position.y
        elif plane == 'xz':
            ax = wp_init.position.x
            ay = wp_init.position.z
            bx = wp_fin.position.x
            by = wp_fin.position.z
        else:
            rospy.logerr("[FollowLine] Plane not found, exiting follow")
            return
        pos = self.pose.position
        rospy.loginfo("ax = [{}]".format(ax))
        rospy.loginfo("ay = [{}]".format(ay))
        rospy.loginfo("bx = [{}]".format(bx))
        rospy.loginfo("by = [{}]".format(by))

        headLine = -np.arctan2(by - ay, bx - ax) # ENU convention, angle from east
        rospy.loginfo("headLine = [{}]".format(headLine))

        # Calcul de la distance à la ligne, si c'est un point alors une valeur est définie
        if np.sqrt( (bx-ax)**2 + (by-ay)**2 ) != 0:
            dist2Line = ((bx-ax)*(pos.y-ay) - (by-ay)*(pos.x-ax)) / np.sqrt( (bx-ax)**2 + (by-ay)**2)
        else:
            dist2Line = 100
        rospy.loginfo("dist2Line = [{}]".format(dist2Line))

        # Calcul de l'erreur
        distAtanPt = 3
        err = -np.arctan(dist2Line / distAtanPt)
        rospy.loginfo("self.yaw = [{}]".format(self.yaw))
        rospy.loginfo("headLine = [{}]".format(headLine))
        rospy.loginfo("err = [{}]".format(err))
        rospy.loginfo("headLine - err = [{}]".format( headLine - err))

        # Bornage entre -pi et pi
        wantedAngle = angle_rad(headLine,- err)
        rospy.loginfo("wantedAngle = [{}]".format( wantedAngle)) # Le cap voulu par rapport au repere global

        return wantedAngle

    def computeCmd(self):
        rospy.loginfo("wp_nb = {}".format(self.wp_nb))

        # self.follow_wp()
        self.follow_line()

        # rospy.loginfo(self.cmd)

        rospy.loginfo("dist_norm = {}".format(self.dist_norm))
        if self.check_harvest_wp():
            rospy.loginfo("Waypoint harvested")
            self.harvest_wp()


    def spin(self):
        while not rospy.is_shutdown():
            self.computeCmd()
            pub_cmd.publish(self.cmd)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('guidance')
    rate = rospy.Rate(50)

    algo = Guide()
    pub_cmd = rospy.Publisher("cmd_autopilot", Vector3, queue_size=1)
    sub_pose = rospy.Subscriber("real_pose", PoseStamped, algo.updatePose)
    sub_twist = rospy.Subscriber("real_twist", TwistStamped, algo.updateTwist)
    sub_cmd = rospy.Subscriber("path", Path, algo.updateWps)

    algo.spin()
