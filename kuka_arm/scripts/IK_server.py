#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
	#
	q1,q2,q3,q4,q5,q6,q7,q8 = symbols('q1:9')
	a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
	d1,d2,d3,d4,d5,d6,d7,d8 = symbols('d1:9')
	alpha0,alpha1, alpha2, alpha3, alpha4, alpha5, alpha6  = symbols('alpha0:7') 
	 
	# Create Modified DH parameters
	#
	s= {alpha0: 0 ,a0: 0 , d1 : 0.75,q1 : q1,
	    alpha1: -pi/2 , a1:0.35 , d2: 0 ,q2:q2-pi/2,
	    alpha2: 0  , a2:1.25, d3:0 ,q3:q3,
	    alpha3: -pi/2  , a3: -0.054,  d4: 1.50 ,q4: q4,
	    alpha4: pi/2 , a4: 0, d5: 0,q5: q5,
	    alpha5:  -pi/2 , a5: 0 ,d6: 0 ,q6: q6,
	    alpha6:  0, a6:0, d7:0.303,q7:0 }	
	# Define Modified DH Transformation matrix
	T_0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
	T_0_1 = T_0_1.subs(s)
	T_1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
	T_1_2 = T_1_2.subs(s)

	T_2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
	T_2_3 = T_2_3.subs(s)

	T_3_4 =Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
	T_3_4 = T_3_4.subs(s)

	T_4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
	T_4_5 = T_4_5.subs(s)

	T_5_6 =  Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
	T_5_6 = T_5_6.subs(s)
	T_6_G =  Matrix([[             cos(q7),            -sin(q7),            0,              a5],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
        T_6_G = T_6_G.subs(s)
	# Create individual transformation matrices
	T_0_2 = simplify(T_0_1*T_1_2)
	T_0_3 = simplify(T_0_2*T_2_3)
	T_0_4 = simplify(T_0_2*T_3_4)
	T_0_5 = simplify(T_0_3*T_4_5)
	T_0_6 = simplify(T_0_4*T_5_6)
	T_0_7 = simplify(T_0_5*T_6_G)

	
	# Extract rotation matrices from the transformation matrices
	ang1,ang2 = symbols('ang1:3')
	R_z = Matrix([[             cos(ang1),            -sin(ang1),            0,              0],
               [ sin(ang1), cos(ang1), 0, 0],
               [ 0, 0,  1,  0],
               [                   0,                   0,            0,               1]])

     	R_y = Matrix([[             cos(ang2),            sin(ang2),            0,              0],
               [ 0, 1, 0, 0],
               [  -sin(ang2),0,  cos(ang2),  0],
               [                   0,                   0,            0,               1]])
	   
	R_corr = simplify(R_z*R_y)
        R_corr = R_corr.evalf(subs={'ang1': radians(180),' ang2': radians(-90) }, chop = True)
	phi1,phi2,phi3 = symbols('phi1:4')

        R_zz = Matrix([[             cos(phi3),            -sin(phi3),            0,              0],
               [ sin(phi3), cos(phi3), 0, 0],
               [ 0, 0,  1,  0],
               [                   0,                   0,            0,               1]])

        R_yy = Matrix([[             cos(phi2),            sin(phi2),            0,              0],
               [ 0, 1, 0, 0],
               [  -sin(phi2),0,  cos(phi2),  0],
               [                   0,                   0,            0,               1]])
        R_xx=  Matrix([[ 1.,     0.,  0.,0],
              [       0,        cos(phi1),        -sin(phi1),0],
              [0,        sin(phi1),  cos(phi1),1],
              [0,0,0,1]]) 
        R_rpy = simplify(R_zz*R_yy*R_xx*R_corr.T)
        R_0_3 = simplify(T_0_1[0:3,0:3]*T_1_2[0:3,0:3]*T_2_3[0:3,0:3])
          # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
 
	   
	    R_rpy= R_rpy.evalf(subs={'phi1': roll, 'phi2': pitch, 'phi3': yaw }, chop = True)
	    
            # Calulcate WC coordinates
	    d= 0.303
	    w_x = px - (d*R_rpy.col(2)[0])
	    w_y = py - (d*R_rpy.col(2)[1])
	    w_z = pz - (d*R_rpy.col(2)[2])

	    # Calculate joint angles using Geometric IK method
	    A= 1.501
            C= 1.25
            B = sqrt(pow(w_z - 0.75,2) + pow(sqrt(w_x*w_x+w_y*w_y) -0.35,2))
            ang = atan2(w_z - 0.75,sqrt(w_x*w_x+w_y*w_y) -0.35)
	    a = acos((pow(B,2) + pow(C,2) - pow(A,2))/(2*B*C))
	    b = acos((pow(C,2) + pow(A,2) - pow(B,2))/(2*C*A))
	    x = atan2(0.054, 1.5)
            theta2 = pi/2 -a -ang
	    theta3 = pi/2 -(b+x)
	    theta1 = atan2(w_y,w_x)
		    
            ###Compute R0_3
	    
	   	      
            R_0_33 = R_0_3.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3}, chop = True)
            # Compute R_3_6
            R_3_6 = simplify(R_0_33.T*R_rpy[0:3,0:3])
	    theta4 = atan2(R_3_6[2,2], -R_3_6[0,2])
            theta5 = atan2( sqrt(pow(R_3_6[2,2],2) + pow(R_3_6[0,2],2)) , R_3_6[1,2])
            theta6 =  atan2(-R_3_6[1,1], R_3_6[1,0])

	    	    # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
