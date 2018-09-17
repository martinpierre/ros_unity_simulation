#!/usr/bin/env python

# Siemens AG, 2018
# Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import numpy as np
from numpy.linalg import inv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Quaternion
from Xlib import display
from Xlib.ext import randr
import tf


def frange(start,stop,step):
	i=start
	while i< stop:
		yield i
		i+=step

def xdot(x,u):
	x=x.flatten()
	u=u.flatten()
	theta=x[2]
	v=x[3]
	return np.array([[v*np.cos(theta)],[v*np.sin(theta)],[u[0]],[u[1]]])

def control(x,w,dw,ddw):
	x=x.flatten()
	y=np.array([[x[0]],[x[1]]])
	dy=np.array([[x[3]*np.cos(x[2])],[x[3]*np.sin(x[2])]])
	ddy=(w-y)+2*(dw-dy)+ddw
	A=np.array([[-x[3]*np.sin(x[2]),np.cos(x[2])],[x[3]*np.cos(x[2]),np.sin(x[2])]])
	return np.dot(inv(A),ddy)

    
def Trajectory(x):
	
	# initialize node
	rospy.init_node('Trajectory', anonymous = True)
	#### Setup MouseToJoy Publisher 
	TrajPublisher = rospy.Publisher("test", Pose, queue_size = 1)
	rate = rospy.Rate(25) # 25hz
	msg = Pose()

	t=0
	dt=1/float(25)
	
	
	
	while not rospy.is_shutdown():


		w=np.array([[5*np.cos(t)],[5*np.sin(t)]])
		dw=np.array([[-5*np.sin(t)],[5*np.cos(t)]])
		ddw=np.array([[-5*np.cos(t)],[-5*np.sin(t)]])
		u=control(x,w,dw,ddw)
		x=xdot(x,u)*dt+x
		
			#### Initialize joy msg every loop		
		msg.position.x=x.item(0)
		msg.position.y=x.item(1)
		Q=tf.transformations.quaternion_from_euler(0, 0, x.item(2))
		msg.orientation.x=Q[0]
		msg.orientation.y=Q[1]
		msg.orientation.z=Q[2]
		msg.orientation.w=Q[3]
			
		euler=tf.transformations.euler_from_quaternion(Q)
		yaw=euler[2]*(180/np.pi)
			#### Publish msg
			#rospy.loginfo([pos_x, pos_y])

		# rospy.loginfo(x.item(0))
		# rospy.loginfo(x.item(1))
		rospy.loginfo(u.item(1))
		rospy.loginfo(t)
		t=t+dt
		
			#rospy.loginfo(Q)
		TrajPublisher.publish(msg)
		rate.sleep()

	



if __name__ == '__main__':
	x=np.array([[10.],[10.],[1],[1.]])
	Trajectory(x)
	
