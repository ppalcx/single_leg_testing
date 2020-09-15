from numpy import *
import math
import matplotlib.pyplot as plt


a=3
b=1.5
l1 = 6.2  # length of link a1 in cm
l2 = 5.2  # length of link a2 in cm
th1=[]
th2=[]
x_trac=[]
y_trac=[]

for i in range(0,361,5):
    #Ellipse
    x=a*cos(math.radians(i))
    y=b*sin(math.radians(i))
    #Invese kinematics
    r1 = sqrt(x**2+y**2)
    phi_1 = arccos((l1**2+r1**2-l2**2)/(2*l1*r1))
    phi_2 = arctan2(y, x)
    theta_1 = rad2deg(phi_2-phi_1)

    phi_3 = arccos((l1**2+l2**2-r1**2)/(2*l1*l2))
    theta_2 = 180-rad2deg(phi_3)

    # print('theta one: ', theta_1)
    # print('theta two: ', theta_2)
    th1.append(theta_1)
    th2.append(theta_2)
    x_trac.append(x)
    y_trac.append(y)
# print(x)
# plt.plot(x,y)
# plt.show()
# theta=(th1,th2)
# print(theta)
# print(x_trac)
# print(y_trac)
plt.plot(x_trac,y_trac)
plt.show()
