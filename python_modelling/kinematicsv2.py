from numpy import*

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import linalg
from matplotlib.pylab import plot, figure, axis

# This class does forward and inverse kinematics for 
# the RRP arm designed by Team Maize for Project 2 in EECS 498.
# Team members;
# Ying Wang
# Eddie Chakmakian
# Matthew Cornett
# Jakob Hoellerbauer


# [0,0,d] configuration of robot is the following:
#       d
#   < ------>
#   __________
#   |
#   |
#   |
#   |
#   |
#   |


def kinematics(t1,t2,t3,h,A):

    para = 0.08889*25.4*360/(2*pi);
    T01 = array([[cos(t1+pi/2),0, sin(t1+pi/2), 0],
           [sin(t1+pi/2), 0, -cos(t1+pi/2) , 0],
           [0,      1,        0, h],
           [0,       0,        0, 1]])
    T12 = array([[ cos(t2-pi/2),0,sin(t2-pi/2),(4*A)*cos(t2-pi/2)],
            [sin(t2-pi/2),0,-cos(t2-pi/2),(4*A)*sin(t2-pi/2)],
            [0,1,0,0],
            [0,0,0,1]])

    T23 = array([[0, 0, 0,0],
            [0, 0, 0, 0],
            [0, 0, 0, para*t3],
            [0,0,0,1]])
    T02 = dot(T01,T12)
    T03 = dot(T02,T23)

    O0 = array([0, 0,0])
    O1 = T01[0:3,3]
    O2 = T02[0:3,3]
    O3 = T03[0:3,3]

    Z0 = array([0,0,1]);
    Z1 = T01[0:3,2]
    Z2 = T02[0:3,2]
    Z3 = T03[0:3,2]

    Jv1 = cross(Z0,O3-O0)
    Jv2 = cross(Z1,O3-O1)
    Jv3 = cross(Z2,O3-O2)
    JV = hstack([Jv1, Jv2, Jv3])
    JV = (JV)

    return [T03,JV]

def rrpkkk():
    N = 20
    t1=linspace(0,0,N)
    t2=linspace(pi/2,pi/2,N)
    t3=linspace(-pi,pi,N)
    A = 2*25.4
    d = 0.01
    h = 4*A

    gst = zeros(shape=(4,N))
    gst[:,1] = array([0,d+4*A,h-4*A,1])

    for i in range(1,N):
        [T,J] = kinematics(t1[i],t2[i],t3[i],h,A)
        gst[:,i] = dot(T,gst[:,1])

    X = gst[1,:]
    Y = gst[2,:]
    Z = gst[3,:]


    fig = plt.figure()
    ax = Axes3D(fig)
    #axis([-500,500,-500,500,-500,500])
    ax.plot(X,Y,Z,'p')
