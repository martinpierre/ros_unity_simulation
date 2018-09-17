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
	rospy.init_node('speed', anonymous = True)
	#### Setup MouseToJoy Publisher 
	TrajPublisher = rospy.Publisher("speed_publish" ,Twist, queue_size = 1)
	rate = rospy.Rate(25) # 25hz
	msg = Twist()

	t=0
	dt=1/float(25)
	
	
	
	while not rospy.is_shutdown():


		w=np.array([[5*np.cos(t)],[5*np.sin(t)]])
		dw=np.array([[-5*np.sin(t)],[5*np.cos(t)]])
		ddw=np.array([[-5*np.cos(t)],[-5*np.sin(t)]])
		u=control(x,w,dw,ddw)
		speed=xdot(x,u)
		x=xdot(x,u)*dt+x
		
		
			#### Initialize joy msg every loop		
		msg.linear.x=speed.item(0)
		msg.linear.y=speed.item(1)
		msg.angular.z=np.sign(np.arctan2(speed.item(1),speed.item(0)))*u.item(0)
		t=t+dt
		
		rospy.loginfo(msg)
		TrajPublisher.publish(msg)
		rate.sleep()

	



if __name__ == '__main__':
	x=np.array([[0.],[0.],[0],[1]])
	Trajectory(x)
	
