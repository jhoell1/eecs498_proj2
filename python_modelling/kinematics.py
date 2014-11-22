from numpy import*
# The main program is a 



class kinematics ( object ):

	def __init__(self,h, *arg, **kw):
		self.h = h
		self.cur_config = [0,0,0];
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
	def forward_kinematics_transform(self,theta1,theta2,d):
		g1 = exp(self.zeta1_h*theta1)
		g2 = exp(self.zeta2_h*theta2)
		g3 = exp(self.zeta2_h*d)
		return g1*g2*g3

	#configuration is an 3 element array ==> [theta1,theta2,d]
	def end_effector_position(self,configuration):
		return dot(self.forward_kinematics_transform(configuration[0],configuration[1],
			configuration[2]),array([[0],
									 [0],
									 [0],
									 [1]]))

	# ee_pos is a 3 element array ==> [x y z]
	def inverse_kinematics(ee_pos):




    # delta is an 1 by self.numJoints array
    def calcInducedVelocity(self,delta):        

        z1 = self.cur_config[0]
        z2 = self.cur_config[1]
        z3 = self.cur_config[2]
        z4 = self.cur_config[3]

        e_x_start = self.d*cos(z1) + z3*cos(z1+z2) + self.l3*cos(z1+z2+z4)
        e_y_start = self.d*sin(z1) + z3*sin(z1+z2) + self.l3*sin(z1+z2+z4)
        e_alpha = z1+z2+z4

        z_new = add(self.cur_config, delta)

        z1 = z_new[0]
        z2 = z_new[1]
        z3 = z_new[2]
        z4 = z_new[3]

        e_x_perturbed = self.d*cos(z1) + z3*cos(z1+z2) + self.l3*cos(z1+z2+z4)
        e_y_perturbed = self.d*sin(z1) + z3*sin(z1+z2) + self.l3*sin(z1+z2+z4)
        e_alpha_perturbed = z1+z2+z4

        res = zeros(3);
        res[0] = e_x_perturbed
        res[1] = e_y_perturbed
        res[2] = e_alpha_perturbed
		
    def computeJacobian(self):

        z1 = self.cur_config[0]
        z2 = self.cur_config[1]
        z3 = self.cur_config[2]
        z4 = self.cur_config[3]

        e_x_start = self.d*cos(z1) + z3*cos(z1+z2) + self.l3*cos(z1+z2+z4)
        e_y_start = self.d*sin(z1) + z3*sin(z1+z2) + self.l3*sin(z1+z2+z4)
        e_alpha = z1+z2+z4
        start = [e_x_start,e_y_start,e_alpha]

        self.Jacobian = zeros(shape=(4,3))

        delta = [0.2,0,0,0]
        res1 = p.calcInducedVelocity(delta)
        diff1 = subtract(res1,start)
        diff1 /= 0.2

        delta = [0,0.2,0,0]
        res2 = p.calcInducedVelocity(delta)
        diff2 = subtract(res2,start)
        diff2 /= 0.2

        delta = [0,0,0.2,0]
        res3 = p.calcInducedVelocity(delta)
        diff3 = subtract(res3,start)
        diff3 /= 0.2

        delta = [0,0,0,0.2]
        res4 = p.calcInducedVelocity(delta)
        diff4 = subtract(res4,start)
        diff4 /= 0.2

        self.Jacobian[0,:] = diff1
        self.Jacobian[1,:] = diff2
        self.Jacobian[2,:] = diff3
        self.Jacobian[3,:] = diff4

        print self.Jacobian
