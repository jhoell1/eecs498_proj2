from numpy import *
from numpy.linalg import svd
from numpy.random import randn
from scipy import pi
from joy import *
import time
from pdb import set_trace as DEBUG
import ckbot.logical as L


# enum values for app state
manual_angle = 0
pos_set = 1
automatic = 2
angle_set = 3
teach_and_repeat = 4


def printState(state):
    if state == manual_angle:
        s = " Manual Angle "
    elif state == pos_set:
        s = " Synchronous Position Movement (sliders control x,y,z)"
    elif state == automatic:
        s = " autonomous control"
    elif state == angle_set:
        s = " Synchronous Angle Movement (sliders control th0,th1,th2)"
    elif state == teach_and_repeat:
        s = " Teach and Repeat "

    print "State: " + str(s)

class joint( object ):

    # Just a wrapper object for a generic joint
    # Has configurable limits

    def __init__(self,servoID,limits,*arg, **kw):
        self.limits = limits
        self.s = servoID
        self.s.mem[self.s.mcu.punch] = 15
        self.s.mem[self.s.mcu.cw_compliance_slope] = 128
        self.s.mem[self.s.mcu.ccw_compliance_slope] = 128


    # returns 1 if the servo movement command was sent, 0 otherwise
    def move(self,angle,UnitRadians):
        if angle < self.limits[0] or angle > self.limits[1]:
            # illegal command
            print "Error: Angle command outside valid range"
            return 0
        else:
            if UnitRadians is True:
                angle = angle * 180.0/pi
            self.s.set_pos(int(angle*100)) # unit of set_pos argument should be centidegrees
            return 1

    def get_angle(self):
        return self.s.get_pos()/100.0 # unit of return of get_pos is decidegrees
    def go_slack(self):
        self.s.go_slack()

class robotArmDriver ( JoyApp ):

    def __init__(self,height,A,*arg, **kw):
        JoyApp.__init__(self, *arg,**kw)
        C = L.Cluster(count=3)
        #self.servo = [joint(C.at.Nx0E,[-3.0,90.0]),joint(C.at.Nx1C,[-30.0,45.0]),joint(C.at.Nx07,[-90.0,90.0])]
        self.servo = [joint(C.at.Nx0E,[-90.0,90.0]),joint(C.at.Nx1C,[-90.0,90.0]),joint(C.at.Nx07,[-90.0,90.0])]
        self.mode = manual_angle
        self.angleCommand = [0,0,0]
        self.positionCommand = [0,0,0]
        self.curConfig = [0,0,0]
        self.followTrajectory = False
        self.trajectories = [[],[],[]]
        self.trajectoryIndex = 0
        self.trajectoryMax = 0
        self.h = height
        self.A = A

        self.teachPoints = []
        self.numTeachPoints = 0
        self.teachRepeatOn = False
        self.teachRepeatPlayback = False

        self.printState = False

        self.positionLimits = [ [3.5*2.54, (12+3.5)*2.54], [-6*2.54,6*2.54], [3.5*2.54, 20*2.54]]

    def onStart(self):
        self.controlUpdate = self.onceEvery(1.0/15.0)

    # start is a 2 x 1 array: [time, pos]
    # final is a 2 x 1 array: [time, pos]
    def linear_interp(self,start,final,timestep):
        slope = (final[1]-start[1])/(final[0]-start[0])
        arr_size = (final[0]-start[0])/timestep + 1
        trajectory = []
        pos = start[1]
        trajectory.append(pos)
        for i in range(1,arr_size):
            pos = pos + slope*timestep
            trajectory.append(pos)
        print trajectory
        return trajectory

    def collectPoint(self):
        if self.mode == teach_and_repeat:
            print "capturing configuration point: " + str(self.curConfig)
            print self.teachPoints
            self.teachPoints.append(list(self.curConfig))
            print self.teachPoints
            self.numTeachPoints = self.numTeachPoints + 1

    def clearPoints(self):
        self.teachPoints = []
        self.numTeachPoints = 0

    def withinRange(self,configuration,threshold):
        sum = 0
        for i in range(0,3):
            sum = sum + pow(self.curConfig[i]-configuration[i],2)

        #return sum < threshold
        return True

    def onEvent(self,evt):

        if evt.type == KEYDOWN :
            print describeEvt(evt)

            if evt.key == K_p:
                self.collectPoint()
            if evt.key == K_d:
                self.clearPoints()


        if evt.type == MIDIEVENT :
            if evt.kind == 'slider':
                if evt.index <= 3:

                    if self.mode is 1:
                        r = self.positionLimits[evt.index-1][1] - self.positionLimits[evt.index-1][0] 
                        self.positionCommand[evt.index-1] = self.positionLimits[evt.index-1][0] + r/127.0
                    else:
                        r = self.servo[evt.index-1].limits[1]- self.servo[evt.index-1].limits[0]
                        angle = self.servo[evt.index-1].limits[0] + evt.value*r/127.0
                        self.angleCommand[evt.index-1] =  angle
                   #print "setting angle " + str(evt.index)
            elif evt.kind == 'btnL' and evt.index==1 and evt.value == 127 :
                self.mode = (self.mode + 1)%5
                printState(self.mode)
                if self.mode == teach_and_repeat:
                    print "setting servos to slack"
                    for i in range(0,3):
                        self.servo[i].go_slack()
                    print "finished slacking servos!"


                print self.mode
            elif evt.kind == 'btnL' and evt.index==3 and evt.value == 127 :
                self.printState = not self.printState

            elif evt.kind == 'btnL' and evt.index==2 and evt.value == 127 : 
                # start following trajectory
                self.followTrajectory = True
                self.trajectoryIndex = 0

                if self.mode == angle_set:
                    desiredAngle = self.angleCommand
                elif self.mode == pos_set:
                    desiredAngle = self.inverse_kinematics(self.angleCommand,0.1)

                for i in range(0,3):
                    start = [0,self.curConfig[i]]
                    end = [5000,desiredAngle[i]]
                    self.trajectories[i] = self.linear_interp(start,end,100)
                    self.trajectoryMax = len(self.trajectories[i])


            elif evt.kind == 'btnU' and evt.index==1 and evt.value ==127 :
                print "Following trajectory!"
                self.followTrajectory = True
                self.trajectoryIndex = 0
                teachPointsWStart  = list(self.teachPoints)
                teachPointsWStart.insert(0, self.curConfig)
                for i in range(0,self.numTeachPoints-1):
                    for j in range(0,3):
                        start = [0, teachPointsWStart[i][j]]
                        end = [5000, teachPointsWStart[i+1][j]]
                        trajInterm = self.linear_interp(start,end,500)
                        self.trajectories[j] = self.trajectories[j] + trajInterm
                        self.trajectoryMax = len(self.trajectories[j])

        if self.controlUpdate():

            if self.printState is True:
                print "current config" + str(self.curConfig) 
                print "current position: " + str(self.end_effector_position(self.curConfig))

            for i in range(0,3):
                self.curConfig[i] = self.servo[i].get_angle()
                #print(self.angleCommand)
                if self.mode == manual_angle:
                    self.servo[i].move(self.angleCommand[i],False)
                elif self.mode == angle_set or self.mode == pos_set or self.mode == teach_and_repeat:
                    if self.followTrajectory == True:
                        if(i==1):
                            self.servo[i].move(self.trajectories[i][self.trajectoryIndex],False)
                        else:
                            self.servo[i].move(self.trajectories[i][self.trajectoryIndex],False)

            if self.mode == angle_set or self.mode == teach_and_repeat:
                if self.followTrajectory == True:
                    #curTrajPoint = [self.trajectories[0][self.trajectoryIndex],self.trajectories[1][self.trajectoryIndex],self.trajectories[2][self.trajectoryIndex]]
                    #if(self.withinRange(curTrajPoint,5)):
                    self.trajectoryIndex = self.trajectoryIndex + 1
                    if self.trajectoryIndex == self.trajectoryMax:
                        self.followTrajectory = False

     # ee_pos is a 3 element array ==> [x y z]
    # step_length should be small, on the order of 0.05 - 0.5
    def inverse_kinematics(self,desired_ee_pos,step_length):

        step_config = self.curConfig
        cur_ee_pos = self.end_effector_position(step_config)

        # iterate until we have achieved our goal
        while sqrt(sum(square(subtract(cur_ee_pos,desired_ee_pos)))) > 0.5 :
            self.computeNumericJacobian(step_config,step_config)
            # step in the direction of the gradient that will minimize the
            # error between the desired and current end effector position
            step_config = step_config + step_length*dot(self.NumericJacobian,subtract(desired_ee_pos,cur_ee_pos))
            cur_ee_pos = self.end_effector_position

        self.cur_config = step_config
        return step_config

    #configuration is an 3 element array ==> [theta1,theta2,theta3]
    def end_effector_position(self,configuration):
        [T,J] = self.kinematics(self.curConfig)
        return dot(T,array([[0],[0],[0],[1]]))[0:3]

    def computeNumericJacobian(self,config):
        for i in range(1,3): # each of the three joint variables
            delta = zeros(shape=(1,3))
            delta[i] = 0.1
            self.NumericJacobian[i,:] = calcInducedVelocity(delta,config)

        print self.NumericJacobian
    # delta is an 1 by 3 array
    # technically, delta should be all zero except for 1 element
    # that makes velocity calcuation easier
    # calculates slope using symmetric differnce
    def calcInducedVelocity(self,delta,config): 

        # check if delta is the right shape
        if sum(delta!=0) > 1 :
            print "delta passed to calcInducedVelocities has the incorrect form!"
            return asanarray([0,0,0])

        c1 = config - delta
        c2 = config + delta

        p1 = self.end_effector_position(c1)
        p2 = self.end_effector_position(c2)

        # is correct only if delta has only 1 nonzero #!!
        return subtract(p2,p1)/(2*sum(delta)) 

    def kinematics(self,config):
        A = self.A
        h = self.h
        t1 = config[0]
        t2 = config[1]
        t3 = config[2]

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


if __name__ == "__main__":
    A = 4*2.54 # inches
    height = 4*A
    app = robotArmDriver(height,A)
    app.run()