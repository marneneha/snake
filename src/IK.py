from sympy import symbols, cos, sin
import numpy as np
# from math import 

# theta = [0]*6
theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1, theta2, theta3, theta4, theta5, theta6')
d1 = 0.333
d3 = 0.316
d5 = 0.384
d7 = 0.107
a3 = 0.088
pi = 3.14
def transformation_calc(a, theta, alpha, d):
    # print("inside transformation_matrix_calculator")
    T = np.array([[cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
    [sin(theta), cos(theta)*cos(alpha), -1*cos(theta)*sin(alpha), a*sin(theta)],
    [0, sin(alpha), cos(alpha), d],
    [0,0,0,1]])
    # print(T)
    return T

DH_Table = np.array([[ 0,theta1, pi/2, d1], 
[0,theta2, -1*pi/2, 0],
[-1*a3,theta3, pi/2, 0],
[0,theta4, pi/2, d5], 
[a3,theta5, -1*pi/2, 0], 
[0,theta6, 0, -1*d7]])
print("DH Parameters are")
print(DH_Table)
T01 = transformation_calc(DH_Table[0][0],DH_Table[0][1], DH_Table[0][2], DH_Table[0][3])
T12 = transformation_calc(DH_Table[1][0],DH_Table[1][1], DH_Table[1][2], DH_Table[1][3])
T23 = transformation_calc(DH_Table[2][0],DH_Table[2][1], DH_Table[2][2], DH_Table[2][3])
T34 = transformation_calc(DH_Table[3][0],DH_Table[3][1], DH_Table[3][2], DH_Table[3][3])
T45 = transformation_calc(DH_Table[4][0],DH_Table[4][1], DH_Table[4][2], DH_Table[4][3])
T56 = transformation_calc(DH_Table[5][0],DH_Table[5][1], DH_Table[5][2], DH_Table[5][3])

T02 = T01*T12
T03 = T02*T23
T04 = T03*T34
T05 = T04*T45
T06 = T05*T56
print("T06 is")
print(T06)
