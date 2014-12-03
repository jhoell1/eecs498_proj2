from numpy import cos,sin,asfarray, dot, c_, newaxis, mean, exp, sum, sqrt, sign, pi, radians
from numpy.linalg import svd
from numpy.random import randn
from scipy import pi
from joy import *
import time
from pdb import set_trace as DEBUG
import ckbot.logical as L


# enum values for app state
manual_angle = 0
manual_position = 1
automatic = 2
angle_set = 3


class joint( object ):

    # Just a wrapper object for a generic joint
    # Has configurable limits

    def __init__(self,servoID,limits,*arg, **kw):
        self.limits = limits
        self.s = servoID

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

class robotArmDriver ( JoyApp ):

    def __init__(self,*arg, **kw):
        JoyApp.__init__(self, *arg,**kw)
        C = L.Cluster(count=3)
        self.servo = [joint(C.at.Nx0E,[-150.0,150.0]),joint(C.at.Nx1C,[-150.0,150.0]),joint(C.at.Nx07,[-150.0,150.0])]
        self.mode = manual_angle
        self.angleCommand = [0,0,0]

    def onStart(self):
        self.controlUpdate = self.onceEvery(1.0/20.0)

    # start is a 2 x 1 array: [time, pos]
    # final is a 2 x 1 array: [time, pos]
    def linear_interp(start,final,timestep):
        slope = (final[1]-start[1])/(final[0]-start[0])
        arr_size = (final[0]-start[0])/timestep + 1
        trajectory = 

        (int)(gsl_vector_get(end,0)-gsl_vector_get(start,0))/time_step + 1; //+1 for the final time step



    def onEvent(self,evt):

        if evt.type == MIDIEVENT :
            if evt.kind == 'slider':
                angle = -180 + evt.value*360.0/127.0
                if evt.index <= 3:
                    #print "setting angle " + str(evt.index)
                    self.angleCommand[evt.index-1] =  angle
            elif evt.kind == 'btnL':
                self.mode = (self.mode + 1)%3


        if self.controlUpdate():
            for i in range(0,3):
                #print(self.angleCommand)
                if self.mode == manual_angle:
                    self.servo[i].move(self.angleCommand[i],False)
                else:
                # do inverse kinematics..........
                    print "blah"


if __name__ == "__main__":
    app = robotArmDriver()
    app.run()