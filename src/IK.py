from sympy import BlockMatrix, Matrix, symbols, cos, sin, diff, pprint, simplify, pi
import numpy as np
# from math import 

# theta = [0]*6
theta1, theta2, theta4, theta5, theta6, theta7 = symbols('theta1, theta2, theta4, theta5, theta6, theta7')
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
pprint("T01 is")
pprint(T01)
T12 = transformation_calc(DH_Table[1][0],DH_Table[1][1], DH_Table[1][2], DH_Table[1][3])
T23 = transformation_calc(DH_Table[2][0],DH_Table[2][1], DH_Table[2][2], DH_Table[2][3])
T34 = transformation_calc(DH_Table[3][0],DH_Table[3][1], DH_Table[3][2], DH_Table[3][3])
T45 = transformation_calc(DH_Table[4][0],DH_Table[4][1], DH_Table[4][2], DH_Table[4][3])
T56 = transformation_calc(DH_Table[5][0],DH_Table[5][1], DH_Table[5][2], DH_Table[5][3])
T67 = transformation_calc(DH_Table[6][0],DH_Table[6][1], DH_Table[6][2], DH_Table[6][3])

T02 = T01*T12
# pprint("T02 is")
# pprint(T02)
T04 = T02*T23*T34
# pprint("T04 is")
# pprint(T04)
T05 = T04*T45
# pprint("T05 is")
# pprint(T05)
T06 = T05*T56
# pprint("T06 is")
# pprint(T06)
T07 = T06*T67
# pprint("T07 is")
# pprint(T07)

Z01 = T01[:,2]
Z02 = T02[:,2]
Z04 = T04[:,2]
Z05 = T05[:,2]
Z06 = T06[:,2]
Z07 = T07[:,2]
X0p = T06[:,3]
q_dot1 = simplify(diff(X0p, theta1))
q_dot2 = simplify(diff(X0p, theta2))
q_dot4 = simplify(diff(X0p, theta4))
q_dot5 = simplify(diff(X0p, theta5))
q_dot6 = simplify(diff(X0p, theta6))
q_dot7 = simplify(diff(X0p, theta7))
upper_jacobian_matrix = BlockMatrix([q_dot1,q_dot2,q_dot4,q_dot5,q_dot6,q_dot7])
upper_jacobian_matrix = upper_jacobian_matrix.row_del(3)
pprint("upper_jacobian_matrix is")
pprint(upper_jacobian_matrix)

Z01.row_join(Z02)
Z01.row_join(Z04)
Z01.row_join(Z05)
Z01.row_join(Z06)
Z01.row_join(Z07)
lower_jacobian_matrix = BlockMatrix([Z01, Z02, Z04, Z05, Z06, Z07])

pprint("upper_jacobian_matrix is")
pprint(lower_jacobian_matrix)
jacobian = BlockMatrix([[upper_jacobian_matrix],[lower_jacobian_matrix]])
pprint("jacobian is")
pprint(jacobian)
# q_dot1.col_join(Z01)
# pprint("jacobian is")
# pprint(q_dot1)
# pprint(q_dot1(0,1))
# Inv_Jacob = q_dot1.inv()