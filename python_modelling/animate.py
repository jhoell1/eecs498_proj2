from numpy import *
from matplotlib.pylab import plot, figure, axis
import kinematics as K


# This class contains functions to do the animation of the 
# the RRP arm designed by Team Maize for Project 2 in EECS 498.
# Team members;
# Ying Wang
# Eddie Chakmakian
# Matthew Cornett
# Jakob Hoellerbauer

# really really really simply animation right now....somebody 
# make it better please? haha

class Animation (object) :

	def __init__(self,robotKinematics):
		self.K = robotKinematics

	# config is 3 element array ==> [theta1, theta2, d]
	# the robot has 3 points that define it. The connecting point
	# between the first and second links, and then the end effector
	def draw(self,config):
		points = zeros(shape=(3,3))
		# end of link 1 is affected by first link only....which now
		# that I think about it, doesn't actually change anything 
		# because its only a rotation about z but i leave it in for
		# completeness sake
		points[1,:] = dot(self.K.get_RBT_subset(1),array([[0],[0],[0]]))
		# The end effector position is affected by everything
		points[2,:] = self.K.end_effector_position(config)

		plot(points[:,1],points[:,2])

	# desired position is 3 element array ==> [x y z]
	def animate(self,desiredPosition,numIterations,holdOn):

		cur_config = self.K.cur_config
		desired_config = self.K.inverse_kinematics(desiredPosition,0.1)
		i = linspace(cur_config[0],desired_config[0],numIterations)
		j = linspace(cur_config[1],desired_config[1],numIterations)
		k = linspace(cur_config[2],desired_config[2],numIterations)        
		#axis('equal')
		for t in range(0,numIterations):
			self.fig.set_visible(0)
			if holdOn is 0:
				clf()
			#axis('equal')
			#axis([-20, 20, -20, 20])
			temp_config = array([i[t],j[t],k[t]])
			self.draw()
			self.fig.set_visible(1)        
			draw()