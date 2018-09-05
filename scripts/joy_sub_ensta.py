#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy


def VectProduct(a,b):
    return Vector3((a.y * b.z) - (a.z * b.y),
                   (a.z * b.x) - (a.x * b.z),
                   (a.x * b.y) - (a.y * b.x))   

# def VectScaling3(a,b,c):
#     x=a.x*b*c
#     y=a.y*b*c
#     z=a.z*b*c
#     return Vector3(x,y,z)

def VectScaling(a,b):
    x=a.x*b
    y=a.y*b
    z=a.z*b
    return Vector3(x,y,z)

def VectSum6(a,b,c,d,e,f):
    return Vector3(a.x+ b.x+ c.x+ d.x+ e.x+ f.x, 
                   a.y+ b.y+ c.y+ d.y+ e.y+ f.y,
                   a.z+ b.z+ c.z+ d.z+ e.z+ f.z)

# def VectSum3(a,b,c):
#     return Vector3(a.x+ b.x + c.x, 
#                    a.y+ b.y + c.y,
#                    a.z+ b.z + c.z)

def VectSum2(a,b):
    return Vector3(a.x+ b.x , 
                   a.y+ b.y ,
                   a.z+ b.z )

def SumForce(msg1):

    
    b=msg1.axes

    # initialize node
    rospy.init_node('PFD', anonymous = True)
    #### Setup MouseToJoy Publisher 
    PFDPublisher = rospy.Publisher("PFD", Wrench, queue_size = 1)
    rate = rospy.Rate(25) # 25hz
    msg = Wrench()

    dt=1/float(25)
    angle=30 * np.pi/180

    # Definition des vecteurs locaux comme sous Unity
    Vback    = Vector3( 0, 0,-1)
    Vforward = Vector3( 0, 0, 1)
    Vdown    = Vector3( 0,-1, 0)
    Vup      = Vector3( 0, 1, 0)
    Vright   = Vector3( 1, 0, 0)
    Vleft    = Vector3(-1, 0, 0)

    ###Position CG###
    M1 = Vector3(-0.6,    0, -0.0925 )
    M2 = Vector3(-0.6, 0.08,  0.04625)
    M3 = Vector3(-0.6,-0.08,  0.04625)
    M4 = Vector3(-0.5, 0.08, -0.04625)
    M5 = Vector3(-0.5,    0,  0.04625)
    M6 = Vector3(-0.5,-0.08, -0.04625)
        
    #Moteur 1
    Fmot1   = Vector3(25,0,0)
    force1  = VectScaling(Fmot1, b[3])           
    torque1 = VectProduct(M1, force1)
        
    #Moteur 2
    Fmot2   = Vector3(25,0,0)
    force2  = VectScaling(Fmot2, b[3])
    torque2 = VectProduct(M2, force2)

    #Moteur 3
    Fmot3   = Vector3(25,0,0)
    force3  = VectScaling(Fmot3, b[3]);
    torque3 = VectProduct(M3, force3)

    #Moteur 4
    Fmot4   = VectScaling(Vector3( 0, -np.cos(angle), np.sin(angle)),0)
    force4  = VectScaling(Fmot4, b[2]) 
    torque4 = VectProduct(M4, force4)

    #Moteur 5
    Fmot5   = Vector3(0,0,-10)
    force5  = VectScaling(Fmot5,b[1])
    torque5 = VectProduct(M5, force5)

    Fmot6   = VectScaling(Vector3( 0,  np.cos(angle), np.sin(angle)),0)
    force6  = VectScaling(Fmot6, b[2]) 
    torque6 = VectProduct(M6, force6)
    
    if(b[2]<0):
        #Moteur 4
        Fmot4   = VectScaling(Vector3( 0, -np.cos(angle), np.sin(angle)),10)
        force4  = VectScaling(Fmot4, -b[2]) 
        torque4 = VectProduct(M4, force4)

        Fmot5   = Vector3(0,0,-5)
        force5  = VectScaling(Fmot5,-b[2])
        torque5 = VectProduct(M5, force5)

    if (b[2]>0):
        #Moteur 6
        Fmot6   = VectScaling(Vector3( 0,  np.cos(angle), np.sin(angle)),10)
        force6  = VectScaling(Fmot6, b[2]) 
        torque6 = VectProduct(M6, force6)

        Fmot5   = Vector3(0,0,-5)
        force5  = VectScaling(Fmot5,b[2])
        torque5 = VectProduct(M5, force5)

    if(b[1]<0):
        #Moteur 4
        Fmot4   = VectScaling(Vector3( 0, -np.cos(angle), np.sin(angle)),10)
        force4  = VectScaling(Fmot4, -b[1]) 
        torque4 = VectProduct(M4, force4)

        Fmot6   = VectScaling(Vector3( 0,  np.cos(angle), np.sin(angle)),10)
        force6  = VectScaling(Fmot6, -b[1]) 
        torque6 = VectProduct(M6, force6)


    
    msg.force=VectSum6(force1,force2,force3,force4,force5,force6)
    msg.torque=VectSum6(torque1,torque2,torque3,torque4,torque5,torque6)   
    
    #msg.force=VectSum3(force1,force2,force3)
    #msg.torque=VectSum3(torque1,torque2,torque3)
    '''
    rospy.loginfo("F1")
    rospy.loginfo(force1)
    rospy.loginfo("M1")
    rospy.loginfo(torque1)
    rospy.loginfo("F2")
    rospy.loginfo(force2)
    rospy.loginfo("M2")
    rospy.loginfo(torque2)
    rospy.loginfo("F3")
    rospy.loginfo(force3)
    rospy.loginfo("M3")
    rospy.loginfo(torque3)

    rospy.loginfo("F4")
    rospy.loginfo(force4)
    rospy.loginfo("M4")
    rospy.loginfo(torque4)
    
    rospy.loginfo("F5")
    rospy.loginfo(force5)
    rospy.loginfo("M5")
    rospy.loginfo(torque5)
    rospy.loginfo("F6")
    rospy.loginfo(force6)
    rospy.loginfo("M6")
    rospy.loginfo(torque6)
    '''
    rospy.loginfo("Force")
    rospy.loginfo(msg.force)
    rospy.loginfo("Moment")
    rospy.loginfo(msg.torque)
    
    PFDPublisher.publish(msg)
    rate.sleep()

def callback(msg):
    SumForce(msg)


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('PFD', anonymous=True)

    rospy.Subscriber("joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()