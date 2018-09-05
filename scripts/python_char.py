# -*- coding: utf-8 -*-
"""
Created on Tue Jul  3 13:30:47 2018

@author: martinpi2
"""
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

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
	
    
   
if __name__=="__main__":
    x=np.array([[0],[0],[1],[1]])
#    u=np.array([[0],[2]])
    dt=1/float(25) 
    x0=[]
    y0=[]
    th=[]
    time=[]
    file=open("position.txt",'w')
    for t in frange(0,100,dt):
        time.append(t)
        w=np.array([[5*np.cos(t)],[5*np.sin(t)]])
        dw=np.array([[-5*np.sin(t)],[5*np.cos(t)]])
        ddw=np.array([[-5*np.cos(t)],[-5*np.sin(t)]])
        u=control(x,w,dw,ddw)
        x=xdot(x,u)*dt+x
        x0.append(x[0])
        y0.append(x[1])
        th.append((x.item(2)*(180/np.pi)))
        file.write("x = " +str(x.item(0))+"\t"+"y =" + str(x.item(1))+"\t"+"th = " +str(x.item(2)*(180/np.pi))+"\n")
   
#plt.plot(x0,y0)
#plt.show()    
file.close()
    
   