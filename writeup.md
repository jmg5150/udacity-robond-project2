## Project 2: Kinematics Pick & Place
Project submission for the Udacity Robotics Software Engineer Nanodegree

Jonathan Georgino
January 11 2018

---

### This project writeup is based on the sample template provided in the base repo provided by Udacity.

---

[//]: # (Image References)
[figure1]: ./figures/figure1.png
[figure2]: ./figures/figure2.png 
[figure3]: ./figures/figure3.png 
[figure4]: ./figures/figure4.png 
[figure5]: ./figures/figure5.png 
[figure6]: ./figures/figure6.png 
[figure7]: ./figures/figure7.png 
[figure8]: ./figures/figure8.png 
[projectsummary]: ./figures/projectsummary.png
[tfmatrix]: ./figures/TFMatrix.png 
[4inbin]: ./figures/stacked4inbin.png
[6inbin]: ./figures/4stacked+2inbin.png
[7inbin]: ./figures/5stacked+2inbin.png
[8inbin]: ./figures/8of9inbin.png
[theta1given]: ./figures/theta1given.gif
[theta2given]: ./figures/theta2given.gif

![projectsummary]

You can watch a successful pick and place cycle of the simulated KR210 pick and place robot running my Inverse Kinematics code [here](https://youtu.be/YELNEP8r350). Please note that due to the rather low resources available on the lubuntu VM running on my laptop, it is painfully slow. The video is 4:00 minutes in length and shows a single cycle. I am sure that no one wants to watch more than that :-P

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This document is intended to fullfil the requirement of the Writeup / Readme.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The following series of figures demonstrates the steps I took to derive the DH parameters of the Kuka KR210 robot

![figure1]

![figure2]

![figure3]

![figure4]

![figure5]

![figure6]

I originally thought about drawing them by hand, but my artistic skills are rather poor. As such, I thought it would a worthwhile pursuit to try to easily create them on the computer. I was pleasantly surprised how simple it was to create the above figures in Google Slides using the predefined shapes.

Finally, here is the DH Table that I arrived at:

![figure7]

and the following table replaces the symbols with numeric values:

![figure8]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

![tfmatrix]

The following function is used to assist in building the homogeneous transforms for each of the links:
```python
def TF_Matrix(self, alpha, a, d, q):
    TF = Matrix([[             cos(q),           -sin(q),            0,             a],
                [   sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha), -sin(alpha)*d],
                [   sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),  cos(alpha)*d],
                [                   0,                 0,            0,            1]])
    return TF
```

I used the following python code which calls the `TF_Matrix()` function above to create the individual transformation matrices for each joint.
```python

# Define the symbols to be used in the kinematics equations
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')    #link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')    #link lengths
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')    #rotation angles

# Define the symbols for the joint angles
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

# Define the symbols used in the rotation matrices
r, p, y = symbols('r p y')

# Construct the DH Table  
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

```

The above code can be found in `IK_server.py`, lines 50 to 83. Note that I am using `self.xxxx` because of my attempt to abstract this into a separate class called KR210Kinematics discussed at the bottom of this writeup.

From there, the code in lines 85 to 115 applies the rotational transformations such that the joint angles can be derived from end-effector pose.

```python
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
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The following code was written to compute the joint angles, theta1 through theta6. This code was developed by following the lecture material presented in the "Inverse Kinematics with Kuka KR210" lesson. This can be found from lines 117 through 145 in `IK_server.py`.


```python
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

```

Of particular note, you can see that I've commented out the use of Sympy's `inv("LU")` function and have replaced with the `transpose()` function. This was done during debugging per the suggestion of my course mentor. In the final version of my code which is working at the time of submission, the `transpose()` function was still being called, and if there's one thing I've learned in my engineering career: if something is not broke, don't fix it. If I had more time available for this project, I would dig into this a bit more, but in an effor to move forward with the course, I will leave it in it's current working form. 

**UPDATE:**

Additional details were requested regarding how to come up with the theta1 - theta6 joint angles:

I followed the `Inverse Kinematics Example` to understand how to compute theta1, theta2, and theta3. Theta1 is given as
![theta1given]

where the first parameter y~c~ is the y coordinate of the wrist center, and x~c~ is the x coordinate of the wrist center, so in my code, it's written as shown below.
```python
self.theta1 = atan2(self.WC[1], self.WC[0]);
```

Finding theta2 is done by projecting links 2 and 3 onto the x-z plane, and again uses the `atan2` function.

![theta2given]

The calculations for parameters r and s are also given, making it easy to implement the solution for theta2 in code:
```python
self.theta2 = pi / 2 - self.angle_a - atan2(self.WC[2] - 0.75, sqrt(self.WC[0] * self.WC[0] + self.WC[1] * self.WC[1]) - 0.35)
```

Calculating theta4, theta5, and theta6 comes from the guidance shared in the `Euler Angles from a Rotation Matrix` lesson. The suggestions in this lesson indicate the reason why it's advantageous to use the `atan2` function to avoid ambiguities which arise when using inverse of sine and cosine. Following the example given in this lesson, I was able to construct the equations for these joint angles in my code. Originally given as just:
```python
self.theta4 = atan2(self.R3_6[2,2], -self.R3_6[0,2])
self.theta5 = atan2(sqrt(self.R3_6[0,2] * self.R3_6[0,2] + self.R3_6[2,2]*self.R3_6[2,2]), self.R3_6[1,2])
self.theta6 = atan2(-self.R3_6[1,1], self.R3_6[1,0])    
```

after the feedback I received from my first submission of this project, I updated theta4 and theta6 to be computed conditionally based on the positive/negative value of theta5 joint angle, as shown below:

```python
self.theta5 = atan2(sqrt(self.R3_6[0,2] * self.R3_6[0,2] + self.R3_6[2,2] * self.R3_6[2,2]), self.R3_6[1,2])

if sin(self.theta5) < 0:

    self.theta4 = atan2(-self.R3_6[2,2], self.R3_6[0,2])
    self.theta6 = atan2(self.R3_6[1,1], -self.R3_6[1,0])

else:

    self.theta4 = atan2(self.R3_6[2,2], - self.R3_6[0,2])
    self.theta6 = atan2(-self.R3_6[1,1], self.R3_6[1,0])
```


Note that I did watch the project walkthrough while I was working on debugging my code, for reasons described in further detail below.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

**Please see the update below for details regarding the changes that have been implemented for the second submission.**

My approach to completing the IK_server.py script was to first start to develop the code in the IK_debug.py script and run it against the three provided test cases. For this reason, I have included the `IK_debug.py` and `IK_debug2.py` files as part of this repository. More discussion on why there are two files to follow.

My initial thoughts were that a lot of the computation of the matrices only needed to be done once, and that the resultant matrices could then be reused for each inverse kinematics state to solve for. This would optimize the amount of time needed for computation. As such, my approach was to make a class called KR210Kinematics which has a `setup()` function and a `compute()` function, so that any static computations could be done just once upon initialization in the `setup()` function, and that only the dynamic computations would occur for each state in the `compute()` function.

Regrettably, in my attempt to split up the math between these two functions, I created a bit of a mess for myself which was a debugging nightmare and filled with red herrings. At one point, it appeared to be a mathematical error with the theta5 calculation, as it always appeared to be offset by 90degrees during the trial runs in gazebo. [This is why I ultimately created `IK_debug2.py` and used the joint_state_publisher sliders to collect two additional test cases from Rviz - these cases were specifically designed to look at theta5, further increasing the amount of red herrings I was encountering.] After two days of debugging my approach, and after having watched the project walk-through video several times to try to find my error, I simply gave up on my optimization and got my code working in a non-optimized way. Now, the computation time for computing the Inverse Kinematics is about 3 to 4 x more than I was targetting with my original code -- but at least I have achieved a working solution for submission so that I can continue to the next material covered in the course.

So that brings me to the function added to the `KR210Kinematics` class titled `slowcompute()` -- this is the code that is actually being used to generate the Inverse Kinematics used in my working solution. This code begins on line 50 and ends at line 145 in the `IK_server.py` file. The code was developed from reviewing the material and examples discussed in KR210 Forward Kinematics parts 1, 2, and 3, of the course content, as well as the following lectures covering the Inverse Kinematics.


**UPDATE:**

Per feedback on the initial submission of this project, I've gone ahead and incorporated the feedback into my code. In order to clearly demonstrate the changes, I've created an second file titled `IK_server_updated.py` which is the the next revision of my code. There are two main changes that I incorporated into this revision. The first is to use Pickle files to optimize code performance. This resulted in ~50% faster execution than my previous `slowcompute()` function discussed above, and is actually in the spirit of my initial attempts of achieving a faster performing function. Using pickle files, I store the `T0_EE`, `R0_3`, and `ROT_EE` matrices such that they only need to be computed once.

 The other change that I implemented is to compute theta5 joint angle first, and then based on the result, compute theta4 and theta6. This implementation can be found on lines 184 to 194 in `IK_server_updated.py`:


```python
self.theta5 = atan2(sqrt(self.R3_6[0,2] * self.R3_6[0,2] + self.R3_6[2,2] * self.R3_6[2,2]), self.R3_6[1,2])

if sin(self.theta5) < 0:

    self.theta4 = atan2(-self.R3_6[2,2], self.R3_6[0,2])
    self.theta6 = atan2(self.R3_6[1,1], -self.R3_6[1,0])

else:

    self.theta4 = atan2(self.R3_6[2,2], - self.R3_6[0,2])
    self.theta6 = atan2(-self.R3_6[1,1], self.R3_6[1,0])
```


#### Results:

In order to demontrate that the code meets the performance criteria for a passing project, I ran a trial run in hopes to successfully place the item in the bin at least 8 times out of 10 attempts. The first screenshot I captured was after 4 items were placed into the bin.

![4inbin]

Note that they are stacked one on top of the other, which I knew would be interesting to what happens when the balance is lost.

![6inbin]

I took another screenshot after the 6th successful pick and place completed.

![7inbin]

With 7 in the bin, I decided I would then turn on screen capturing for what I hoped to be the final pick and place operation. However, with all the excitment of completing the project with a satisfactory implementation, I apparently clicked the "Next" button in Rviz too soon, which caused the the robot to begin the extraction maneuver before the grip operation had completed. This resulted in a failed attempt due to human error.

You can view the failed attempt [here](https://youtu.be/Rsum34j34q8) if you want to share in the heartbreak and disappointment.

However, I was not discouraged and begin the next cycle. The free screen capture software I am using has a 5 minute max length limitation, and when I realized there was a chance that there would not be enough time to see the end, I restarted a new capture beginning at the grip step of the last run. You can see this successful operation [here](https://youtu.be/b5hzOuW6PPM)

![8inbin]

There are 8 items in the bin, with a constant reminder of my mistake sitting up on the shelf :-P