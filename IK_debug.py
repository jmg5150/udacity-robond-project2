from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[2.15286, 0, 1.94653],
                  [0,-0.00014835,0,1]],
                  [1.8499, 0, 1.94645],
                  [0,0,0,0,0,0]],
              5:[[[1.8509, -0.00703284, 1.64353],
                  [0,0.705731,-0.0163874,0.708291]],
                  [1.848986, 0, 1.94645],
                  [0,0,0,0,1.57,0]]}


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
        #self.ROT_error = self.ROT_yaw.subs(y, radians(180))

        self.ROT_EE = self.ROT_EE * self.ROT_error

        self.side_a = 1.501
        #side_b is dynamic, will be computed in function
        self.side_c = 1.25

        self.R0_3 = self.T0_1[0:3, 0:3] * self.T1_2[0:3, 0:3] * self.T2_3[0:3, 0:3]

        

def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)

    kr210 = KR210Kinematics()
    kr210.setup()

    start_time = time()
    
    ########################################################################################
    ## 

    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y, req.poses[x].orientation.z, req.poses[x].orientation.w])
    
    kr210.compute(px, py, pz, roll, pitch, yaw)

    theta1 = kr210.theta1
    theta2 = kr210.theta2
    theta3 = kr210.theta3
    theta4 = kr210.theta4
    theta5 = kr210.theta5
    theta6 = kr210.theta6

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    #FK = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    #your_wc = [WC[0], WC[1], WC[2]] # <--- Load your calculated WC values in this array
    your_wc = kr210.wrist_center()

    #your_ee = [FK[0,3], FK[1,3], FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    your_ee = kr210.end_effector()
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    print ("theta1: %04.8f" % theta1)
    print ("theta2: %04.8f" % theta2)
    print ("theta3: %04.8f" % theta3)
    print ("theta4: %04.8f" % theta4)
    print ("theta5: %04.8f" % theta5)
    print ("theta6: %04.8f" % theta6)

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 4

    print ("---BEGIN---\n")
    print ("Test Case 1:\n")
    test_code(test_cases[1])

    print ("Test Case 2:\n")
    test_code(test_cases[2])

    print ("Test Case 3:\n")
    test_code(test_cases[3])

    print ("Test Case 4:\n")
    test_code(test_cases[4])

    print ("Test Case 5:\n")
    test_code(test_cases[5])