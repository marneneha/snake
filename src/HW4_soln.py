# Title: HW-04 Panda Robot Inverse Velocity Kinematics + Plotting Circle
# Course: ENPM662-2022
# Author: Adarsh Malapaka (amalapak@terpmail.umd.edu)
# Date: 26th Oct, 2022

from sympy import Matrix, symbols, cos, sin, simplify, pi, pprint, diff
from numpy import linspace, meshgrid, ones_like
import matplotlib.pyplot as plt
import time

# Simulation parameters 
tool_length = 10     # (cm)
N = 40               # no. of data points
delta_time = 5.0/N     # time between successive data points

# Panda Link offsets (cm)
a3 = 8.8
d1 = 33.3
d3 = 31.6
d5 = 38.4
d7 = 10.7 + tool_length  # Including pen length in d7 parameter

# Defining the symbolic joint angle symbolic variables 
q1, q2, q4, q5, q6, q7 = symbols('q1, q2, q4, q5, q6, q7')

# Function to obtain Transformation matrix between consecutive links 
def get_tf(q,d,a,alpha):
    T = Matrix([[cos(q),-sin(q)*cos(alpha),sin(q)*sin(alpha),a*cos(q)], [sin(q),cos(q)*cos(alpha),-cos(q)*sin(alpha),a*sin(q)], [0,sin(alpha),cos(alpha),d], [0,0,0,1]])    
    return T

# D-H table for the Panda robot as given in the question.
T_01 = get_tf(q1,  d1,   0,  pi/2) # A_1
print("transformation matrix T01 here is")
pprint(T_01)
T_12 = get_tf(q2,   0,   0, -pi/2)
T_23 = get_tf(0,   d3,  a3, -pi/2)
T_34 = get_tf(q4,   0, -a3,  pi/2)
T_45 = get_tf(q5,  d5,   0,  pi/2)
T_56 = get_tf(q6,   0,  a3, -pi/2)
T_6n = get_tf(q7, -d7,   0,     0)

T_02 = T_01 * T_12 # A_2
pprint("T02 is")
pprint(T_02)
T_04 = T_02 * T_23 * T_34 # A_4
pprint("T04 is")
pprint(T_04)
T_05 = T_04 * T_45 # A_5
pprint("T05 is")
pprint(T_05)
T_06 = T_05 * T_56 # A_6
pprint("T06 is")
pprint(T_06)
T_0n = T_06 * T_6n # A_7
pprint("T07 is")
pprint(T_0n)
print("\n****************************************************************************")
print("       Transformation Matrix (T_0n) between end-effector & base frames:       ")
print("****************************************************************************\n")
pprint(T_0n)

# Setting up the Jacobian Matrix 

Z_0 = Matrix([0, 0, 1])
Z_1 = simplify(T_01[0:3,2])
Z_2 = simplify(T_02[0:3,2])
Z_4 = simplify(T_04[0:3,2])
Z_5 = simplify(T_05[0:3,2])
Z_6 = simplify(T_06[0:3,2])

#######################################
# Using Method-2 to compute Jacobian
#######################################
x_p = T_0n[0:3,3]

h_1 = diff(x_p,q1)
h_2 = diff(x_p,q2)
h_3 = diff(x_p,q4)
h_4 = diff(x_p,q5)
h_5 = diff(x_p,q6)
h_6 = diff(x_p,q7)
J_v1 = simplify(h_1)
J_v2 = simplify(h_2)
J_v3 = simplify(h_3)
J_v4 = simplify(h_4)
J_v5 = simplify(h_5)
J_v6 = simplify(h_6)

#######################################
# Using Method-1 to compute Jacobian
#######################################
# O_0 = Matrix([0, 0, 0])
# O_1 = T_01[0:3,3]
# O_2 = T_02[0:3,3]
# O_4 = T_04[0:3,3]
# O_5 = T_05[0:3,3]
# O_6 = T_06[0:3,3]
# O_n = T_0n[0:3,3]
# J_v1 = simplify(Z_0.cross((O_n-O_0)))
# J_v2 = simplify(Z_1.cross((O_n-O_1)))
# J_v3 = simplify(Z_2.cross((O_n-O_2)))
# J_v4 = simplify(Z_4.cross((O_n-O_4)))
# J_v5 = simplify(Z_5.cross((O_n-O_5)))
# J_v6 = simplify(Z_6.cross((O_n-O_6)))


J_v = Matrix().col_insert(0,J_v1).col_insert(1,J_v2).col_insert(2,J_v3).col_insert(3,J_v4).col_insert(4,J_v5).col_insert(5,J_v6)
J_w = Matrix().col_insert(0,Z_0).col_insert(1,Z_1).col_insert(2,Z_2).col_insert(3,Z_4).col_insert(4,Z_5).col_insert(5,Z_6) 
J = Matrix().row_insert(0,J_v).row_insert(3,J_w)

print("\n***************************************************************************")
print("                           Jacobian Matrix (J):                              ")
print("****************************************************************************\n")
pprint(J)


# Generating circle velocity trajectory using cylindrical coordinate equations of circle points
def generate_circle_velocity(th):
    y_dot = -4.0*pi*sin(th)
    z_dot = 4.0*pi*cos(th)
    X_dot = Matrix([0.0, y_dot, z_dot, 0.0, 0.0, 0.0]) # Generalized end_effector cartesian velocity components
    return X_dot


# Function to compute the inverse kinematics q_dot values from end-effector velocities.
def inverse_velocity_kinematics(X_dot, q_joint):
    J_inv = J.evalf(3,subs={q1: q_joint[0],q2: q_joint[1], q4: q_joint[2], q5: q_joint[3], q6: q_joint[4], q7: q_joint[5]}).inv()
    q_dot = J_inv * X_dot # Generalized joint velocity components from Jacobian inverse
    return q_dot


# Function to compute new value of joint angle based on q_dot and previous angle
def update_joint_angle(q, q_dot):
    q = q + q_dot*delta_time
    return q


# Function to compute forward kinematics (position) to obtain the end-effector (Pen) (x,y,z) co-ords wrt base frame
def forward_position_kinematics(q):
    T = T_0n.evalf(subs={q1: q[0], q2: q[1], q4: q[2], q5: q[3], q6: q[4], q7: q[5]})
    return (T[0,3].round(4),T[1,3].round(4),T[2,3].round(4))

if __name__ == '__main__':

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.axes.set_xlim(left=-10, right=90) 
    ax.axes.set_ylim(bottom=-30, top=55) 
    ax.axes.set_zlim(bottom=0, top=85) 
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    q_joint = Matrix([0.0, 0.0, float(pi/2), 0.0, float(pi), 0.0]) # initial joint angles of the robot
    
    # Looping cylindrical coordinate theta from pi/2 to 5pi/2 for full circle
    for theta in linspace(float(pi/2), float((5*pi)/2), num=N):
        circle_vel_traj = generate_circle_velocity(theta)

        q_dot_joint = inverse_velocity_kinematics(circle_vel_traj, q_joint)

        q_joint = update_joint_angle(q_joint, q_dot_joint)

        (x_0p, y_0p, z_0p) = forward_position_kinematics(q_joint)

        # plot on circle as live points from forward kinematics for every delta_time seconds
        ax.scatter(x_0p,y_0p,z_0p)

        plt.pause(delta_time)

    plt.show()



    
