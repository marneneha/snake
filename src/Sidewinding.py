from sympy import Matrix, symbols, cos, sin, simplify, pi, pprint, diff, shape, transpose
from numpy import linspace, meshgrid, ones_like
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from std_msgs.msg import Float64
import rospy
if __name__ == '__main__':
    rospy.init_node('snake_node', anonymous=True)
    pub_joint1 = rospy.Publisher('/snake/joint1Controller/command', Float64, queue_size=10) 
    pub_joint2 = rospy.Publisher('/snake/joint2Controller/command', Float64, queue_size=10) 
    pub_joint3 = rospy.Publisher('/snake/joint3Controller/command', Float64, queue_size=10) 
    pub_joint4 = rospy.Publisher('/snake/joint4Controller/command', Float64, queue_size=10) 
    pub_joint5 = rospy.Publisher('/snake/joint5Controller/command', Float64, queue_size=10) 
    pub_joint6 = rospy.Publisher('/snake/joint6Controller/command', Float64, queue_size=10) 
    pub_joint7 = rospy.Publisher('/snake/joint7Controller/command', Float64, queue_size=10) 
    rate = rospy.Rate(5) # 5hz
    Ax = pi/6
    Ay = pi/6
    Wx = 5*pi/6
    Wy = 5*pi/6
    deltaX = 2*pi/3
    deltaY = 2*pi/3
    t = 0
    while not rospy.is_shutdown():
        for i in range(1, 7):
            if i == 1:
                q = Ax*sin(Wx*t+i*deltaX)
                pub_joint1.publish(q)
            elif i == 2:
                q = Ay*sin(Wy*t+i*deltaY)
                pub_joint2.publish(q) 
            elif i==3:
                q = Ax*sin(Wx*t+i*deltaX)
                pub_joint3.publish(q)
            elif i==4:
                q = Ay*sin(Wy*t+i*deltaY)
                pub_joint4.publish(q) 
            elif i==5:
                q = Ax*sin(Wx*t+i*deltaX)
                pub_joint5.publish(q)
            elif i==6:
                q = Ay*sin(Wy*t+i*deltaY)
                pub_joint6.publish(q) 
            elif i==7:
                q = Ax*sin(Wx*t+i*deltaX)
                pub_joint7.publish(q)
        rate.sleep()
        t = t+2.0