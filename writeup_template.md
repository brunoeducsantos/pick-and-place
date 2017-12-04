# Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
  


### Kinematic Analysis
#### 1. Evaluation kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
From URDF file it is obtained the following offsets between joints:

Joint Name | Parent Link |Child Link | x(m) | y(m) | z(m) | roll | pitch | yaw
--- | --- | --- | --- | ---| ---|---|---|---
Joint_1| base_link |link_1  | 0     |0  |0.33   |0 |0 |0  
Joint_2| link_1    | link_2 |  0.35 | 0 |  0.42 |0 |0 |0 
Joint_3| link_2    | link_3 | 0     | 0 | 1.25  |0 |0 |0 
Joint_4|  link_3   | link_4 |0.96   | 0 | -0.054|0 |0 |0 
Joint_5| link_4    | link_5 | 0.54  |0  | 0     |0 |0 |0 
Joint_6| link_5    | link_6 | 0.193 | 0 | 0     |0 |0 |0 
gripper_joint |link_6| gripper_link|0.11|0 | 0     |0 |0 |0   

Using the previous URDF parameters it is possible to obtain DH parameter table:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | pi/2 | 0 | 0.303 | 0


#### 2. Derivation of individual transformation matrices about each joint using DH parameters
T_0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
[ 0, 0, 0, 1]])
\\begin{array}{cc}
  a & b \\\\
  c & c
\\end{array}

#### 3 Generation of generalized homogeneous transforme between base_link and gripper_link using only end-effector

 

#### 4. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


