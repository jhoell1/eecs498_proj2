from numpy import*
# The main program is a 



class kinematics ( Object ):

	def __init__(h, *arg, **kw):
		self.h = h
		self.zeta1_h = array([[0, -1, 0, 0],
							 [1, 0, 0, 0],
							 [0, 0, 0, 0],
							 [0, 0, 0, 0]]);
		self.zeta2_h = array([[0, 0, 0, 0],
							 [0, 0, 1, 0-h],
							 [0, -1, 0, 0],
							 [0, 0, 0, 0]]);
		self.zeta3_h = array([[0, 0, 0, 0],
							 [0, 0, 0, 1],
							 [0, 0, 0, 0],
							 [0, 0, 0, 0]]);

	# returns a RBT that can then be used to transform any desired number of points 
	# into the base frame. TO get the end effector position, just multply this matrix
	# with [0;0;0;1] (matlab-style syntax).
	def forward_kinematics_transform(theta1,theta2,d):
		#g1 = 
		print "Awesome"
