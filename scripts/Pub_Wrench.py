#!/usr/bin/env python

import rospy
import numpy as np

from numpy.linalg import inv
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from Xlib import display
from Xlib.ext import randr
import tf


def VectProduct(a,b):
	return Vector3((a.y * b.z) - (a.z * b.y),
				   (a.z * b.x) - (a.x * b.z),
				   (a.x * b.y) - (a.y * b.x))	

def VectScaling(a,b):
	x=a.x*b
	y=a.y*b
	z=a.z*b
	return Vector3(x,y,z)

def VectSum6(a,b,c,d,e,f):
	return Vector3(a.x+ b.x+ c.x+ d.x+ e.x+ f.x, 
				   a.y+ b.y+ c.y+ d.y+ e.y+ f.y,
				   a.z+ b.z+ c.z+ d.z+ e.z+ f.z)

def VectSum2(a,b):
	return Vector3(a.x+ b.x, 
				   a.y+ b.y,
				   a.z+ b.z)

def SumForce():
	
	# initialize node
	rospy.init_node('PFD', anonymous = True)
	#### Setup MouseToJoy Publisher 
	PFDPublisher = rospy.Publisher("PFD", Wrench, queue_size = 1)
	rate = rospy.Rate(25) # 25hz
	msg = Wrench()

	t=0
	dt=1/float(25)



	while not rospy.is_shutdown() :
		angle=30 * np.pi/180
		# Definition des vecteurs locaux comme sous Unity
		Vback=Vector3(0,0,-1)
		Vforward=Vector3(0,0,1)

		Vdown=Vector3(0,-1,0)
		Vup=Vector3(0,1,0)

		Vright=Vector3(1,0,0)
		Vleft=Vector3(-1,0,0)

		###Position CG###
		M1=Vector3(-0.6,0,-0.0925)
		M2=Vector3(-0.6,0.08,0.04625)
		M3=Vector3(-0.6,-0.08,0.04625)
		M4=Vector3(-0.5,0.08,-0.04625)
		M5=Vector3(-0.5,0,0.04625)
		M6=Vector3(-0.5,-0.08,-0.04625)

		

		
		#Moteur 1
		Fmot1=Vector3(10,0,0)
		#force1=VectScaling(Vleft,Fmot1)
		#torque1=VectProduct(Vector3(0.6,0.0925,0), Fmot1)
		torque1=VectProduct(M1, Fmot1)
		
		#Moteur 2
		Fmot2=Vector3(10,0,0)
		#force2=VectScaling(Vleft,Fmot2)
		#torque2=VectProduct(Vector3(0.6, -0.04625, 0.08), Fmot2)
		torque2=VectProduct(M2, Fmot2)

		#Moteur 3
		Fmot3=Vector3(10,0,0)
		#force3=VectScaling(Vleft,Fmot3);
		#torque3=VectProduct(Vector3(0.6, -0.04625, -0.08), Fmot3)
		torque3=VectProduct(M3, Fmot3)
		
		#Moteur 4
		Fmot4=Vector3(0,-10,10)
		#F4 = VectSum2(VectScaling(VectScaling(Vback,Fmot4),np.cos(angle)),VectScaling(VectScaling(Vdown,Fmot4),np.sin(angle)))
		#force4=F4;
		#torque4=VectProduct(Vector3(0.6, 0.04625, 0.08), Fmot4)
		torque4=VectProduct(M4, Fmot4)

		#Moteur 5
		Fmot5=Vector3(0,0,-10)
		#force5=VectScaling(Vup,Fmot5)
		#torque5=VectProduct(Vector3(0.6, -0.0925, 0), VectScaling(Vup,Fmot5))
		torque5=VectProduct(M5, Fmot5)

		#Moteur 6
		Fmot6=Vector3(0,10,10)
		#F6 =VectSum2(VectScaling(VectScaling(Vforward,Fmot6),np.cos(angle)),VectScaling(VectScaling(Vdown,Fmot6),np.sin(angle)))
		#force6=F6
		#torque6=VectProduct(Vector3(0.6, 0.04625, -0.08), Fmot6)  
		torque6=VectProduct(M6, Fmot6)
		
		#msg.force=VectSum6(force1,force2,force3,force4,force5,force6)
		#msg.torque=VectSum6(torque1,torque2,torque3,torque4,torque5,torque6)   
		
		rospy.loginfo("F1")
		rospy.loginfo(Fmot1)
		rospy.loginfo("M1")
		rospy.loginfo(torque1)
		rospy.loginfo("F2")
		rospy.loginfo(Fmot2)
		rospy.loginfo("M2")
		rospy.loginfo(torque2)
		rospy.loginfo("F3")
		rospy.loginfo(Fmot3)
		rospy.loginfo("M3")
		rospy.loginfo(torque3)
		rospy.loginfo("F4")
		rospy.loginfo(Fmot4)
		rospy.loginfo("M4")
		rospy.loginfo(torque4)
		rospy.loginfo("F5")
		rospy.loginfo(Fmot5)
		rospy.loginfo("M5")
		rospy.loginfo(torque5)
		rospy.loginfo("F6")
		rospy.loginfo(Fmot6)
		rospy.loginfo("M6")
		rospy.loginfo(torque6)

		

		PFDPublisher.publish(msg)
		rate.sleep()


if __name__ == '__main__':
	SumForce()