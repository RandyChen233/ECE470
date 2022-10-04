#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""


q1 = np.array([-150, 150 ,10])
q2 = np.array([-150 ,270 ,162])
q3 = np.array([94 ,270,162])
q4 = np.array([307, 177 , 162])
q5 = np.array([307, 260, 162])
q6 = np.array([390,260,162])

w1 = np.array([0, 0, 1])
w2 = np.array([ 0, 1,0 ])
w3 = np.array([0, 1, 0])
w4 = np.array([0, 1, 0])
w5 = np.array([1, 0 ,0])
w6 = np.array([0, 1, 0])

v1 = np.cross(-w1,q1)
v2 = np.cross(-w2,q2)
v3= np.cross(-w3,q3)
v4 = np.cross(-w4,q4)
v5 = np.cross(-w5,q5)
v6 = np.cross(-w6,q6)

q_m = np.array([390,401,215.5])


def hat(w): #Skew-symmetric representation of a vector

	return np.array([[0, -w[2], w[1]],[w[2], 0 ,-w[0]], [ -w[1], w[0], 0]])


def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	
	s1 = np.array([w1,v1]).flatten()
	s2 = np.array([w2,v2]).flatten()
	s3 = np.array([w3,v3]).flatten()
	s4 = np.array([w4,v4]).flatten()
	s5 = np.array([w5,v5]).flatten()
	s6 = np.array([w6,v6]).flatten()

	S = np.vstack([s1,s2,s3,s4,s5,s6]) #s1-s6 stacked in row by row fashion
	M = np.array([[0, -1., 0., 390.],[0., 0., -1., 401.],[1., 0., 0., 215.5],[0., 0., 0., 1.]]) 
	# ==============================================================#
	return M, S

S1 = np.block([[hat(w1),v1.reshape(-1,1)],[np.zeros((1,4))]]) #4 by 4 matrix for [S]
S2 = np.block([[hat(w2),v2.reshape(-1,1)],[np.zeros((1,4))]])
S3 = np.block([[hat(w3),v3.reshape(-1,1)],[np.zeros((1,4))]])
S4 = np.block([[hat(w4),v4.reshape(-1,1)],[np.zeros((1,4))]])
S5 = np.block([[hat(w5),v5.reshape(-1,1)],[np.zeros((1,4))]])
S6 = np.block([[hat(w6),v6.reshape(-1,1)],[np.zeros((1,4))]])


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	
	T1 = expm(S1*theta1)
	T2 = expm(S2*theta2)
	T3 = expm(S3*theta3)
	T4 = expm(S4*theta4)
	T5 = expm(S5*theta5)
	T6 = expm(S6*theta6)

	M,S = Get_MS()
	T = T1@T2@T3@T4@T5@T6@M

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
