from numpy import*
from scipy import linalg

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

class kinematics ( object ):

    def __init__(self,h, *arg, **kw):
        self.h = h
        self.cur_config = [0,0,0];
        self.zeta1_h = array([[0, -1, 0, 0],
                                [1, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0]])
        self.zeta2_h = array([[0, 0, 0, 0],
                                [0, 0, 1, 0-h],
                                [0, -1, 0, 0],
                                [0, 0, 0, 0]])
        self.zeta3_h = array([[0, 0, 0, 0],
                                [0, 0, 0, 1],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0]])
        self.Jacobian = zeros(shape=(3,3))

    # returns a RBT that can then be used to transform any desired number of points 
    # into the base frame. TO get the end effector position, just multply this matrix
    # with [0;0;0;1] (matlab-style syntax).
    def forward_kinematics_transform(self,theta1,theta2,d):
        return self.get_RBT_subset(3)

    # can be used to apply only a subset of g1,g2,g3 to
    # the provided point. Useful for animating links in an arm
    # config is 3 element array ==> [theta1, theta2, d]
    def get_RBT_subset(self,config,num):
        g1 = linalg.expm(self.zeta1_h*config[0])
        g2 = linalg.expm(self.zeta2_h*config[1])
        g3 = linalg.expm(self.zeta2_h*config[2])
        if num == 1:
            return g1
        elif num == 2:
            return g1*g2
        elif num == 3:
            return g1*g2*g3
        else:
            print "Error: Only three links in robotic arm"

    #configuration is an 3 element array ==> [theta1,theta2,d]
    def end_effector_position(self,configuration):
        return dot(self.forward_kinematics_transform(configuration[0],configuration[1],
        configuration[2]),array([[0],
                                [0],
                                [0],
                                [1]]))

    # new config is a 3 element array ==> [theta1,theta2,d]
    def set_cur_config(self,new_config):
        self.cur_config = new_config

    # ee_pos is a 3 element array ==> [x y z]
    # step_length should be small, on the order of 0.05 - 0.5
    def inverse_kinematics(self,desired_ee_pos,step_length):

        step_config = self.cur_config
        cur_ee_pos = self.end_effector_position(step_config)

        # iterate until we have achieved our goal
        while sqrt(sum(square(subtract(cur_ee_pos,desired_ee_pos)))) > 0.5 :
            self.computeJacobian(step_config)
            # step in the direction of the gradient that will minimize the
            # error between the desired and current end effector position
            step_config = step_config + step_length*dot(Jacobian,subtract(desired_ee_pos,cur_ee_pos))
            cur_ee_pos = self.end_effector_position

        self.cur_config = step_config
        return step_config

    def computeJacobian(self):

        for i in range(1,3): # each of the three joint variables
            delta = zeros(shape=(1,3))
            delta[i] = 0.1
            self.Jacobian[i,:] = calcInducedVelocity(delta)

        print self.Jacobian

    # delta is an 1 by 3 array
    # technically, delta should be all zero except for 1 element
    # that makes velocity calcuation easier
    # calculates slope using symmetric differnce
    def calcInducedVelocity(self,delta): 

        # check if delta is the right shape
        if sum(q!=0) > 1 :
            print "delta passed to calcInducedVelocities has the incorrect form!"
            return asanarray([0,0,0])

        c1 = self.cur_config - delta
        c2 = self.cur_config + delta

        p1 = self.end_effector_position(c1)
        p2 = self.end_effector_position(c2)

        # is correct only if delta has only 1 nonzero #!!
        return subtract(p2,p1)/(2*sum(delta)) 
