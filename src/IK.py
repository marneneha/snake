from sympy import zeros, BlockMatrix, Matrix, symbols, cos, sin, diff, pprint, simplify, pi
import numpy as np
# from math import 

# theta = [0]*6
theta1, theta2, theta4, theta5, theta6, theta7 = symbols('theta1, theta2, theta4, theta5, theta6, theta7')
deltaT = 0.1
d1 = 33.3
d3 = 31.6
d5 = 38.4
d7 = 10.7
a3 = 8.8
def transformation_calc(a, theta, alpha, d):
    # pprint("inside transformation_matrix_calculator")
    T = Matrix([[cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
    [sin(theta), cos(theta)*cos(alpha), -1*cos(theta)*sin(alpha), a*sin(theta)],
    [0, sin(alpha), cos(alpha), d],
    [0,0,0,1]])
    # pprint(T)
    return T

DH_Table = np.array([[ 0,theta1, pi/2, d1], 
[0,theta2, -1*pi/2, 0],
[a3,0, -1*pi/2, d3],
[-1*a3,theta4, pi/2, 0],
[0,theta5, pi/2, d5], 
[a3,theta6, -1*pi/2, 0], 
[0,theta7, 0, -1*d7]])
pprint("DH Parameters are")
pprint(DH_Table)
T01 = transformation_calc(DH_Table[0][0],DH_Table[0][1], DH_Table[0][2], DH_Table[0][3])
T12 = transformation_calc(DH_Table[1][0],DH_Table[1][1], DH_Table[1][2], DH_Table[1][3])
T23 = transformation_calc(DH_Table[2][0],DH_Table[2][1], DH_Table[2][2], DH_Table[2][3])
T34 = transformation_calc(DH_Table[3][0],DH_Table[3][1], DH_Table[3][2], DH_Table[3][3])
T45 = transformation_calc(DH_Table[4][0],DH_Table[4][1], DH_Table[4][2], DH_Table[4][3])
T56 = transformation_calc(DH_Table[5][0],DH_Table[5][1], DH_Table[5][2], DH_Table[5][3])
T67 = transformation_calc(DH_Table[6][0],DH_Table[6][1], DH_Table[6][2], DH_Table[6][3])

T02 = T01*T12
T04 = T02*T23*T34
T05 = T04*T45
T06 = T05*T56
T07 = T06*T67

Z01 = simplify(T01[:,2])
Z02 = simplify(T02[:,2])
Z04 = simplify(T04[:,2])
Z05 = simplify(T05[:,2])
Z06 = simplify(T06[:,2])
Z07 = simplify(T07[:,2])
X0p = simplify(T07[:,3])
q1 = simplify(diff(X0p, theta1))
q2 = simplify(diff(X0p, theta2))
q4 = simplify(diff(X0p, theta4))
q5 = simplify(diff(X0p, theta5))
q6 = simplify(diff(X0p, theta6))
q7 = simplify(diff(X0p, theta7))
upper_jacobian_matrix = Z01.row_join(Z02).row_join(Z04).row_join(Z05).row_join(Z06).row_join(Z07)
upper_jacobian_matrix = upper_jacobian_matrix.row_del(3)
pprint("upper_jacobian_matrix is")
pprint(upper_jacobian_matrix)

lower_jacobian_matrix = q1.row_join(q2).row_join(q4).row_join(q5).row_join(q6).row_join(q7)
lower_jacobian_matrix = lower_jacobian_matrix.row_del(3)
pprint("lower_jacobian_matrix is")
pprint(lower_jacobian_matrix)
jacobian = upper_jacobian_matrix.col_join(lower_jacobian_matrix)

theta = 0
Vx = 0
Vy = 4*pi*cos(theta)
Vz = -1*4*pi*sin(theta)
Wx = 0
Wy = 0
Wz = 0
vel_vec = Matrix([Vx, Vy, Vz, Wx, Wy, Wz])
joint_angle = Matrix([0.0, 0.0, pi/2, 0.0, pi, 0.0])
t = 0
while t<=5.0:
    jacobian = jacobian.evalf(subs={theta1:joint_angle[0],theta2:joint_angle[1],theta4:joint_angle[2],theta5:joint_angle[3],theta6:joint_angle[4],theta7:joint_angle[5]})
    # pprint("jacobian is")
    # pprint(jacobian)
    Inv_Jacob = jacobian.inv()
    Vy = 4*pi*cos(theta)
    Vz = -1*4*pi*sin(theta)

    vel_vec = Matrix([ Vx, Vy, Vz, Wx, Wy, Wz])
    joint_angular_vel = Inv_Jacob*vel_vec
    joint_angle = simplify(joint_angle+joint_angular_vel*deltaT)
    # pprint("joint angle is")
    # pprint(joint_angle)
    # pprint("T07 is")
    # pprint(T07)
    state_vec = T07.evalf(subs={theta1:joint_angle[0],theta2:joint_angle[1],theta4:joint_angle[2],theta5:joint_angle[3],theta6:joint_angle[4],theta7:joint_angle[5]})
    print(state_vec[0,3], state_vec[1,3], state_vec[2,3])
    theta = theta+0.05
    t=t+deltaT
