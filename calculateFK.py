import numpy as np
import math
from math import pi
from math import cos
from math import sin

def dh_mat(a, alpha, d, theta):
	a = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
		[sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
		[0, sin(alpha), cos(alpha), d],
		[0, 0, 0, 1]])
	return a

def translate_z(dist):
	rtn = np.identity(4)
	rtn[2][3] = dist
	return rtn

class FK():
	def forward(self, q):
		"""
		INPUT:
		q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

		OUTPUTS:
		jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
				  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
				  The base of the robot is located at [0,0,0].
		T0e       - a 4 x 4 homogeneous transformation matrix,
				  representing the end effector frame expressed in the
				  world frame
		"""
		A0 = dh_mat(0, 0, 0.141, 0)
		A1 = dh_mat(0, -pi/2, 0.192, q[0])
		A2 = dh_mat(0, pi/2, 0, q[1])
		A3 = dh_mat(0.0825, pi/2, 0.316, q[2])
		A4 = dh_mat(0.0825, pi/2, 0, pi/2 + (q[3] + pi/2))
		A5 = dh_mat(0, -pi/2, 0.384, q[4])
		A6 = dh_mat(0.088, pi/2, 0, -pi/2 + (q[5] - pi/2))
		Ae = dh_mat(0, 0, 0.21, q[6] - pi/4)

		Tw0 = A0
		Tw1 = Tw0 @ A1
		Tw2 = Tw1 @ A2
		Tw3 = Tw2 @ A3
		Tw4 = Tw3 @ A4
		Tw5 = Tw4 @ A5
		Tw6 = Tw5 @ A6
		Twe = Tw6 @ Ae

		#since frame 2 and joint 2, frame 4 and joint 4, frame 5 and joint 5, frame 6 and joint 6 are not at the same points, so offsets ought to be implemented
		Tw2_prime = Tw2@translate_z(0.195)
		Tw4_prime = Tw4@translate_z(0.125)
		Tw5_prime = Tw5@translate_z(-0.015)
		Tw6_prime = Tw6@translate_z(0.051)

		# calculate the transformation matrix between the world frame and the end effector
		jointPositions = np.zeros((8,3))
		jointPositions[0, :] = Tw0[0:3, 3]
		jointPositions[1, :] = Tw1[0:3, 3]
		jointPositions[2, :] = Tw2_prime[0:3, 3]
		jointPositions[3, :] = Tw3[0:3, 3]
		jointPositions[4, :] = Tw4_prime[0:3, 3]
		jointPositions[5, :] = Tw5_prime[0:3, 3]
		jointPositions[6, :] = Tw6_prime[0:3, 3]
		jointPositions[7, :] = Twe[0:3, 3]

		return(jointPositions, Twe)

	def get_axis_of_rotation(self,q):
		"""
		INPUT:
		q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

		OUTPUTS:
		axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
								 world frame

		"""
		Ai = FK().compute_Ai(q)
		axes = np.zeros((7,3))
		for i in range(np.shape(FK().compute_Ai(q))[0]-1):
			zi = Ai[i] @ np.transpose([0,0,1,0])
			zi = zi[0:3]
			axes[i,:] = zi

		return(axes)

	def compute_Ai(sefl, q):
		"""
		INPUT:
		q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

		OUTPUTS:
		Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
			  necessarily located at the joint locations
		"""
		A0 = dh_mat(0, 0, 0.141, 0)
		A1 = dh_mat(0, -pi/2, 0.192, q[0])
		A2 = dh_mat(0, pi/2, 0, q[1])
		A3 = dh_mat(0.0825, pi/2, 0.316, q[2])
		A4 = dh_mat(0.0825, pi/2, 0, pi/2 + (q[3] + pi/2))
		A5 = dh_mat(0, -pi/2, 0.384, q[4])
		A6 = dh_mat(0.088, pi/2, 0, -pi/2 + (q[5] - pi/2))
		Ae = dh_mat(0, 0, 0.21, q[6] - pi/4)

		Tw0 = A0
		Tw1 = Tw0 @ A1
		Tw2 = Tw1 @ A2
		Tw3 = Tw2 @ A3
		Tw4 = Tw3 @ A4
		Tw5 = Tw4 @ A5
		Tw6 = Tw5 @ A6
		Twe = Tw6 @ Ae

		return((Tw0,Tw1,Tw2,Tw3,Tw4,Tw5,Tw6,Twe))

if __name__ == "__main__":

	fk = FK()
	q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])


	joint_positions, T0e = fk.forward(q)

	z_axes = fk.get_axis_of_rotation(q)

	print("Joint Positions:\n",joint_positions)
	print("End Effector Pose:\n",T0e)
	print("Z Axes:\n",z_axes)
