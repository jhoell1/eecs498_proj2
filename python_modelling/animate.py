from numpy import *
from matplotlib.pylab import plot, figure, axis
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from kinematics import *
from scipy import linalg


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
		#self.fig = plt.figure()
		self.fig = figure()
		# NOTE!!!!!!!!!!!!!
		# if scipy version is 1.0.0 use the following lines:

		# self.ax = self.fig.add_subplot(111,projection='3d')
		# else use this:
		#self.ax = Axes3D(self.fig)

		self.fig.set_visible(1)


	# config is 3 element array ==> [theta1, theta2, d]
	# the robot has 3 points that define it. The connecting point
	# between the first and second links, and then the end effector
	def drawConfig(self,config):
		points = zeros(shape=(3,4))
		# end of link 1 is affected by first link only....which now
		# that I think about it, doesn't actually change anything 
		# because its only a rotation about z but i leave it in for
		# completeness sake
		points[:,1] = reshape(dot(self.K.get_RBT_subset(config,1),array([[0],[0],[self.K.h],[1]]))[0:3],3)
		# The 3rd point is the end of link2 where the hoeckens linkage is attached
		points[:,2] = reshape(dot(self.K.get_RBT_subset(config,2),array([[0],[self.K.d],[0],[1]]))[0:3],3)
		# The end effector position is affected by everything
		points[:,3] = reshape(self.K.end_effector_position(config),3)

		# NOTE!!!!!!!!!!!!!
		# if scipy version is 1.0.0 use the following lines:

		# Axes3D.plot(points[0,:],points[1,:],points[2,:])

		#self.ax.plot(points[0,:],points[1,:],points[2,:])

		# plot XY
		plt.subplot(221)
		axis([-5, 5, -5, 5])
		plt.plot(points[0,:],points[1,:],'p-')
		plt.title("XY")

		# plot YZ
		plt.subplot(222)
		axis([-5, 5, -5, 5])
		plt.plot(points[1,:],points[2,:],'p-')
		plt.title("YZ")

		#plot XZ
		plt.subplot(223)
		axis([-5, 5, -5, 5])
		plt.plot(points[0,:],points[2,:],'p-')
		plt.title("XZ")


	# if taskSpaceCoords is true
	# desired position is 3 element array ==> [x y z]
	# else it is 3 element array ==> [theta1 theta2 theta3]
	def animate(self,desiredPosition,numIterations,holdOn,taskSpaceCoords=True):

		cur_config = self.K.cur_config
		if taskSpaceCoords is True:
			desired_config = self.K.inverse_kinematics(desiredPosition,0.1)
		else : # desiredPostion holds joint values
			desired_config = desiredPosition
		i = linspace(cur_config[0],desired_config[0],numIterations)
		j = linspace(cur_config[1],desired_config[1],numIterations)
		k = linspace(cur_config[2],desired_config[2],numIterations)        
		#axis('equal')
		for t in range(0,numIterations):
			self.fig.set_visible(0)
			if holdOn is 0:
				self.fig.clear()
			#axis('equal')
			#axis([-20, 20, -20, 20])
			temp_config = array([i[t],j[t],k[t]])
			self.drawConfig(temp_config)
			self.fig.set_visible(1)        
			#self.fig.draw()
			draw()