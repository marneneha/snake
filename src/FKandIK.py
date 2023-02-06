from sympy import Matrix, symbols, cos, sin, simplify, pi, pprint, diff, shape, transpose
from numpy import linspace, meshgrid, ones_like
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from std_msgs.msg import Float64
import rospy
if __name__ == '__main__':
    pub_joint2 = rospy.Publisher('/snake/joint2Controller/command', Float64, queue_size=10) 
    pub_joint3 = rospy.Publisher('/snake/joint3Controller/command', Float64, queue_size=10) 
    pub_joint4 = rospy.Publisher('/snake/joint4Controller/command', Float64, queue_size=10) 
    pub_joint5 = rospy.Publisher('/snake/joint5Controller/command', Float64, queue_size=10) 
    pub_joint6 = rospy.Publisher('/snake/joint6Controller/command', Float64, queue_size=10) 
    pub_joint7 = rospy.Publisher('/snake/joint7Controller/command', Float64, queue_size=10) 
    rospy.init_node('snake_node', anonymous=True)

def transform(a, theta, alfa, d):
    Ai = Matrix([[cos(theta), -sin(theta)*cos(alfa), sin(theta)*sin(alfa), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alfa), -cos(theta)*sin(alfa), a*sin(theta)],
        [0, sin(alfa), cos(alfa), d],
        [0, 0, 0, 1]])
    return Ai

q1, q2, q3, q4, q5, q6, q7 = symbols('q1, q2, q3, q4, q5, q6, q7')

# DH paramter constants 
# inches
ai = 2.85
di = 0
# radians
alfa1 = pi/2
alfa2 = -pi/2
alfa3 = pi/2
alfa4 = -pi/2
alfa5 = pi/2
alfa6 = -pi/2
alfa7 = pi/2


T_01 = transform(ai, q1, alfa1, di)
T_12 = transform(ai, q2, alfa2, di)
T_23 = transform(ai, q3, alfa3, di)
T_34 = transform(ai, q4, alfa4, di)
T_45 = transform(ai, q5, alfa5, di)
T_56 = transform(ai, q6, alfa6, di)
T_67 = transform(ai, q7, alfa7, di)

T_02 = T_01 * T_12
T_03 = T_02 * T_23
T_04 = T_03 * T_34
T_05 = T_04 * T_45
T_06 = T_05 * T_56
T_07 = T_06 * T_67

print("\n****************************************************************************")
print("       Transformation Matrix (T_0n) between end-effector & base frames:       ")
print("****************************************************************************\n")
pprint(T_07)

# q1 = 0
# q2 = 0
# q3 = 0
# q4 = 0
# q5 = 0
# q6 = 0
# q7 = 0
# # T_home = T_07
# T_home = T_07.evalf(subs={ q1: 0.0, q2: 0.0, q3: 0.0, q4: 0.0, q5: 0.0, q6: 0.0, q7: 0.0})
# print("\n****************************************************************************")
# print("       Transformation Matrix (T_home) between end-effector & base frames:       ")
# print("****************************************************************************\n")
# pprint(simplify(T_home))



# # in set 2, q2 = q3 = -pi/2, q6 = pi/2, all other q's = 0
# T_set2 = T_07.evalf(subs={ q1: 0.0, q2: float(-pi/2), q3: float(-pi/2), q4: 0.0, q5: 0.0, q6: float(pi/2), q7: 0.0})
# print("\n****************************************************************************")
# print("T_0n, set 2 (q2 = q3 = -pi/2, q6 = pi/2, all other q's = 0)      ")
# print("****************************************************************************\n")
# pprint(simplify(T_set2))

# # in set 3, q2 = pi/3, q1 = 2pi/3, q5 = pi/6, all other q's = 0
# T_set3 = T_07.evalf(subs={ q1: float(2*pi/3), q2: float(-pi/3), q3: 0.0, q4: 0.0, q5: float(pi/6), q6: 0.0, q7: 0.0})
# print("\n****************************************************************************")
# print("T_0n, set 3 (q2 = -pi/3, q1 = 2pi/3, q5 = pi/6, all other q's = 0)      ")
# print("****************************************************************************\n")
# pprint(simplify(T_set3))
print("i am here 1")
Z_0 = Matrix([0, 0, 1]) # double check, not sure if this is right
Z_2 = simplify(T_02[0:3,2])
Z_3 = simplify(T_03[0:3,2])
Z_4 = simplify(T_04[0:3,2])
Z_5 = simplify(T_05[0:3,2])
Z_6 = simplify(T_06[0:3,2])
print("i am here 2")

# Z_7 = simplify(T_07[0:3,2])
# print("z_7: ")
# pprint(Z_7)

#####################
# Method 2 to compute the jacobian
#####################
x_p = simplify(T_07[0:3,3])

# h_1 = diff(x_p,q1)
# print("h_1: ")
# pprint(h_1)
print("i am here 3")
h_2 = simplify(diff(x_p,q2))
print("i am here 3.1")
h_3 = simplify(diff(x_p,q4))
print("i am here 3.2")
h_4 = simplify(diff(x_p,q5))
print("i am here 3.3")
h_5 = simplify(diff(x_p,q6))
print("i am here 3.4")
h_6 = simplify(diff(x_p,q7))
print("i am here 3.5")
h_7 = simplify(diff(x_p,q7))
print("i am here 4")

J = Matrix([[h_2, h_3, h_4, h_5, h_6, h_7],
    [Z_0, Z_2, Z_3, Z_4, Z_5, Z_6]]) # q3 is fixed
print("\n***************************************************************************")
print("                           Jacobian Matrix (J):                              ")
print("****************************************************************************\n")
# pprint(J)
print(shape(J))

qJoint = Matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # only 6 values because we are fixing q1
x_dot = 
z_dot = 
J = J.evalf(3,subs={ q1: qJoint[0], q2: qJoint[1], q4: qJoint[2], q5: qJoint[3], q6: qJoint[4], q7: qJoint[5]})
Jinv = J.inv()
X_dot = Matrix([0.0, 15, 0.0, 0.0, 0.0, 0.0, 0.0]) # Generalized end_effector cartesian velocity components
q = Jinv*X_dot
pprint(q)



# def inverse_velocity_kinematics(X_dot, q_joint):
#     # Jt = transpose(J)*(J*transpose(J)**-1) # lecture 9 slide 33, right pseudo inverse bc J is 6x7
#     # J_inv = J * Jt
#     J_inv = J_inv.evalf(3,subs={ q1: q_joint[0], q2: q_joint[1], q4: q_joint[2], q5: q_joint[3], q6: q_joint[4], q7: q_joint[5]}).inv()
#     # J_inv = J.evalf(3,subs={ q2: q_joint[0], q3: q_joint[1],  q4: q_joint[2], q5: q_joint[3], q6: q_joint[4], q7: q_joint[5]}).inv() # no q1 bc its fixed
#     q_dot = J_inv * X_dot # Generalized joint velocity components from Jacobian inverse
#     return q_dot

# N = 40
# T = 10
# delta_time = T/N

# def update_joint_angle(q, q_dot):
#     q = q + q_dot*delta_time
#     return q

# def forward_position_kinematics(q):
#     T = T_07.evalf(subs={ q2: q[0], q3: q[1], q4: q[2], q5: q[3], q6: q[4], q7: q[5]}) # no q1 bc its fixed
#     return (T[0,3].round(4),T[1,3].round(4),T[2,3].round(4))

# def generate_circle_velocity(th):
#     y_dot = -6.0*pi*sin(th) # units are in inches
#     x_dot = 6.0*pi*cos(th)
#     #               x dot, y dot, zdot,wx , wy , wz , ??
#     return X_dot

# i = 0
# jointAngles = []
# jointVelocities = []

# if __name__ == '__main__':
#     # fig = plt.figure()
#     # ax = fig.add_subplot(111, projection='3d')
#     # ax.axes.set_xlim(left=-10, right=90) 
#     # ax.axes.set_ylim(bottom=-30, top=55) 
#     # ax.axes.set_zlim(bottom=0, top=85) 
#     # ax.set_xlabel('X')
#     # ax.set_ylabel('Y')
#     # ax.set_zlabel('Z')
    
#     for theta in linspace(0, float(2*pi), num = N ):
#         i = i +1
#         circleVelocityTrajectory = generate_circle_velocity(theta)
#         qdotJoint = inverse_velocity_kinematics(circleVelocityTrajectory, qJoint)
#         jointVelocities.append(qdotJoint)
#         qJoint = update_joint_angle(qJoint, qdotJoint)
#         jointAngles.append(jointAngles)

#         (x_0p, y_0p, z_0p) = forward_position_kinematics(qJoint)
#     plt.show()
