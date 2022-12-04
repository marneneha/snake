from sympy import Matrix, symbols, cos, sin, simplify, pi, pprint, diff, shape, transpose
from numpy import linspace, meshgrid, ones_like
import matplotlib.pyplot as plt
import time

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
alfa1 = -pi/2
alfa2 = pi/2
alfa3 = -pi/2
alfa4 = pi/2
alfa5 = -pi/2
alfa6 = pi/2
alfa7 = -pi/2


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
pprint(simplify(T_07))

