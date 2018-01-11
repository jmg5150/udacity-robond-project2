#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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

# custom class for kinematics operations
class KR210Kinematics:

    def TF_Matrix(self, alpha, a, d, q):
        TF = Matrix([[             cos(q),           -sin(q),            0,             a],
                    [   sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha), -sin(alpha)*d],
                    [   sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),  cos(alpha)*d],
                    [                   0,                 0,            0,            1]])
        return TF


    # This simple function returns the computed wrist center in the format desired by the IK_debug.py script
    def wrist_center(self):
        return [self.WC[0], self.WC[1], self.WC[2]]

    # This function computes the end effector position based on the joint angles (Forward Kinematics)
    def end_effector(self):
        
        # define the symbols needed for evaluating the equation
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        
        # evalute the (already built) transformation matrix based on the currently-computed joint angles
        FK = self.T0_EE.evalf(subs={q1: self.theta1, q2: self.theta2, q3: self.theta3, q4: self.theta4, q5: self.theta5, q6: self.theta6})
        
        # return the result in the format desired by the IK_debug.py script
        return [FK[0,3], FK[1,3], FK[2,3]]

    def slowcompute(self, px, py, pz, roll, pitch, yaw):

        # Define the symbols to be used in the kinematics equations
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')    #link offsets
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')    #link lengths
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')    #rotation angles

        # Define the symbols for the joint angles
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Define the symbols used in the rotation matrices
        r, p, y = symbols('r p y')

        # Construct the DH Table, these values come from kr210.urdf.xacro file  
        DH_Table = {    alpha0:         0,  a0:      0,   d1:     0.75,   q1:           q1,
                        alpha1:   -pi/2.0,  a1:   0.35,   d2:        0,   q2: -pi/2.0 + q2,
                        alpha2:         0,  a2:   1.25,   d3:        0,   q3:           q3,
                        alpha3:   -pi/2.0,  a3: -0.054,   d4:      1.5,   q4:           q4,
                        alpha4:    pi/2.0,  a4:      0,   d5:        0,   q5:           q5,
                        alpha5:   -pi/2.0,  a5:      0,   d6:        0,   q6:           q6,
                        alpha6:         0,  a6:      0,   d7:    0.303,   q7:            0}

        
        # Build the individual transformation matrices using the TF_Matrix function (defined above)
        self.T0_1 = self.TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
        self.T1_2 = self.TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
        self.T2_3 = self.TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
        self.T3_4 = self.TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
        self.T4_5 = self.TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
        self.T5_6 = self.TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
        self.T6_EE = self.TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

        # Build the complete transformation matrix used for Forward Kinematics calculation
        self.T0_EE = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_EE

        # Define the Roll, Pitch, and Yaw rotation matrices
        self.ROT_roll =     Matrix([[1,      0,       0],
                                    [0, cos(r), -sin(r)],
                                    [0, sin(r),  cos(r)]])

        self.ROT_pitch =    Matrix([[ cos(p), 0,  sin(p)],
                                    [      0, 1,       0],
                                    [-sin(p), 0,  cos(p)]])

        self.ROT_yaw =      Matrix([[cos(y), -sin(y), 0],
                                    [sin(y),  cos(y), 0],
                                    [     0,       0, 1]])

        # Build the complete rotation matrix for the end effector
        self.ROT_EE = self.ROT_yaw * self.ROT_pitch * self.ROT_roll

        # compute the error correction needed for the end effector rotation caused by the difference
        # between the definition in the URDF versus the DH convention
        self.ROT_error = self.ROT_yaw.subs(y, radians(180)) * self.ROT_pitch.subs(p, radians(-90))

        self.ROT_EE = self.ROT_EE * self.ROT_error
        
        # start by substituing the roll, pitch, and yaw parameters into the previously-computed end effector rotation matrix
        self.ROT_EE = self.ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

        # now create the end effect position matrix using the position information from the parameters
        self.EE = Matrix([[px], [py], [pz]])


        # create the WristCenter matrix based on the end effector position and rotation matrices
        self.WC = self.EE - (0.303) * self.ROT_EE[:,2]

        self.theta1 = atan2(self.WC[1], self.WC[0]);
        
        # SSS triangle for computing joint angles
        self.side_a = 1.501
        self.side_b = sqrt(pow((sqrt(self.WC[0] * self.WC[0] + self.WC[1] * self.WC[1]) - 0.35), 2) + pow((self.WC[2] - 0.75), 2))
        self.side_c = 1.25

        # compute the squared value of each side to further optomize the algorithm computation time
        sidea_squared = self.side_a * self.side_a
        sideb_squared = self.side_b * self.side_b
        sidec_squared = self.side_c * self.side_c

        self.angle_a = acos((sideb_squared + sidec_squared - sidea_squared) / (2 * self.side_b * self.side_c))
        self.angle_b = acos((sidea_squared + sidec_squared - sideb_squared) / (2 * self.side_a * self.side_c))
        self.angle_c = acos((sidea_squared + sideb_squared - sidec_squared) / (2 * self.side_a * self.side_b))


        self.theta2 = pi / 2 - self.angle_a - atan2(self.WC[2] - 0.75, sqrt(self.WC[0] * self.WC[0] + self.WC[1] * self.WC[1]) - 0.35)
        self.theta3 = pi / 2 - (self.angle_b + 0.036)

        self.R0_3 = self.T0_1[0:3, 0:3] * self.T1_2[0:3, 0:3] * self.T2_3[0:3, 0:3]
        self.R0_3 = self.R0_3.evalf(subs={q1: self.theta1, q2: self.theta2, q3: self.theta3})

        #self.R3_6 = self.R0_3.inv("LU") * self.ROT_EE
        self.R3_6 = self.R0_3.transpose() * self.ROT_EE

        self.theta4 = atan2(self.R3_6[2,2], -self.R3_6[0,2])
        self.theta5 = atan2(sqrt(self.R3_6[0,2] * self.R3_6[0,2] + self.R3_6[2,2]*self.R3_6[2,2]), self.R3_6[1,2])
        self.theta6 = atan2(-self.R3_6[1,1], self.R3_6[1,0])     


    # This function takes the position and orientation as parameters and computes the joint angles (Inverse Kinematics) 
    def compute(self, px, py, pz, roll, pitch, yaw):

        # define the symbols needed for evaluating the equations
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # start by substituing the roll, pitch, and yaw parameters into the previously-computed end effector rotation matrix
        self.ROT_EE = self.ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

        # now create the end effect position matrix using the position information from the parameters
        self.EE = Matrix([[px], [py], [pz]])

        # create the WristCenter matrix based on the end effector position and rotation matrices
        self.WC = self.EE - (0.303) * self.ROT_EE[:,2]

        
        # SSS triangle for computing joint angles
        #self.side_a is static, already computed
        self.side_b = sqrt(pow((sqrt(self.WC[0] * self.WC[0] + self.WC[1] * self.WC[1]) - 0.35), 2) + pow((self.WC[2] - 0.75), 2))
        #self.side_c is static, already computed

        # compute the squared value of each side to further optomize the algorithm computation time
        sidea_squared = self.side_a * self.side_a
        sideb_squared = self.side_b * self.side_b
        sidec_squared = self.side_c * self.side_c

        self.angle_a = acos((sideb_squared + sidec_squared - sidea_squared) / (2 * self.side_b * self.side_c))
        self.angle_b = acos((sidea_squared + sidec_squared - sideb_squared) / (2 * self.side_a * self.side_c))
        self.angle_c = acos((sidea_squared + sideb_squared - sidec_squared) / (2 * self.side_a * self.side_b))


        self.theta1 = atan2(self.WC[1], self.WC[0]);
        self.theta2 = pi / 2 - self.angle_a - atan2(self.WC[2] - 0.75, sqrt(self.WC[0] * self.WC[0] + self.WC[1] * self.WC[1]) - 0.35)
        self.theta3 = pi / 2 - (self.angle_b + 0.036)


        
        self.R0_3 = self.R0_3.evalf(subs={q1: self.theta1, q2: self.theta2, q3: self.theta3})

        # something about this didn't work. WC location was always fine, but in several cases 
        # there would be a problem with theta5 joint angle
        #self.R3_6 = self.R0_3.inv("LU") * self.ROT_EE

        # after pulling my hair out for a few hours, I then reached out to my mentor for help
        # and after trying a few of his suggestions, he recommend to try this line of code.
        # This resolved the problem with theta5
        self.R3_6 = self.R0_3.inv("LU") * self.ROT_EE
        

        self.theta4 = atan2(self.R3_6[2,2], -self.R3_6[0,2])
        self.theta5 = atan2(sqrt(self.R3_6[0,2] * self.R3_6[0,2] + self.R3_6[2,2]*self.R3_6[2,2]), self.R3_6[1,2])
        self.theta6 = atan2(-self.R3_6[1,1], self.R3_6[1,0])


    def setup(self):

        # Define the symbols to be used in the kinematics equations
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')    #link offsets
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')    #link lengths
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')    #rotation angles

        # Define the symbols for the joint angles
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Define the symbols used in the rotation matrices
        r, p, y = symbols('r p y')

        # Construct the DH Table, these values come from kr210.urdf.xacro file  
        DH_Table = {    alpha0:         0,  a0:      0,   d1:     0.75,   q1:           q1,
                        alpha1:   -pi/2.0,  a1:   0.35,   d2:        0,   q2: -pi/2.0 + q2,
                        alpha2:         0,  a2:   1.25,   d3:        0,   q3:           q3,
                        alpha3:   -pi/2.0,  a3: -0.054,   d4:      1.5,   q4:           q4,
                        alpha4:    pi/2.0,  a4:      0,   d5:        0,   q5:           q5,
                        alpha5:   -pi/2.0,  a5:      0,   d6:        0,   q6:           q6,
                        alpha6:         0,  a6:      0,   d7:    0.303,   q7:            0}

        
        # Build the individual transformation matrices using the TF_Matrix function (defined above)
        self.T0_1 = self.TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
        self.T1_2 = self.TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
        self.T2_3 = self.TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
        self.T3_4 = self.TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
        self.T4_5 = self.TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
        self.T5_6 = self.TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
        self.T6_EE = self.TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

        # Build the complete transformation matrix used for Forward Kinematics calculation
        self.T0_EE = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_EE

        # Define the Roll, Pitch, and Yaw rotation matrices
        self.ROT_roll =     Matrix([[1,      0,       0],
                                    [0, cos(r), -sin(r)],
                                    [0, sin(r),  cos(r)]])

        self.ROT_pitch =    Matrix([[ cos(p), 0,  sin(p)],
                                    [      0, 1,       0],
                                    [-sin(p), 0,  cos(p)]])

        self.ROT_yaw =      Matrix([[cos(y), -sin(y), 0],
                                    [sin(y),  cos(y), 0],
                                    [     0,       0, 1]])

        # Build the complete rotation matrix for the end effector
        self.ROT_EE = self.ROT_yaw * self.ROT_pitch * self.ROT_roll

        # compute the error correction needed for the end effector rotation caused by the difference
        # between the definition in the URDF versus the DH convention
        self.ROT_error = self.ROT_yaw.subs(y, radians(180)) * self.ROT_pitch.subs(p, radians(-90))

        self.ROT_EE = self.ROT_EE * self.ROT_error

        # these triangle side lengths are used to calculate theta2 and theta3 values, however only side_b isn't a fixed value
        # as such, side_a and side_c are set now
        self.side_a = 1.501
        #side_b is dynamic, will be computed in function
        self.side_c = 1.25

        # assemble the full rotational transformation matrix
        self.R0_3 = self.T0_1[0:3, 0:3] * self.T1_2[0:3, 0:3] * self.T2_3[0:3, 0:3]

# initialize the class that handles the kinematics calculations for the KR210
kr210 = KR210Kinematics()


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # All the matrices are built when the class initialized for time savings
        # you can see the expected functions above in the KR210Kinematics class

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

            # pass the position and orientation info into our computations
            kr210.slowcompute(px, py, pz, roll, pitch, yaw)
            #kr210.compute(px, py, pz, roll, pitch, yaw)

            # get the computed joint angles
            theta1 = kr210.theta1
            theta2 = kr210.theta2
            theta3 = kr210.theta3
            theta4 = kr210.theta4
            theta5 = kr210.theta5
            theta6 = kr210.theta6

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

    # build up the transformation matrices to make calculations during operation execute faster
    #kr210.setup()
    
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
