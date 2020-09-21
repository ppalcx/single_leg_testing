import numpy as np
import math as m
import matplotlib.pyplot as plt

def trajectory_ellipse():
    a = .06
    b = .03
    l1 = .137  # length of link a1 in cm
    l2 = .10  # length of link a2 in cm
    th1 = []
    th2 = []
    x_trac = []
    y_trac = []

    for i in range(0, 361, 5):
        #Ellipse
        x=-0.2+(a*m.cos(m.radians(i)))
        y=b*m.sin(m.radians(i))
        # x = 0.1
        # y = -0.2
        # Invese kinematics
        r1 = m.sqrt(x ** 2 + y ** 2)
        phi_1 = m.acos((l1 ** 2 + r1 ** 2 - l2 ** 2) / (2 * l1 * r1))
        phi_2 = m.atan2(y, x)
        theta_deg1 = np.rad2deg(phi_2 - phi_1)
        theta_1 = np.deg2rad(theta_deg1)

        phi_3 = m.acos((l1 ** 2 + l2 ** 2 - r1 ** 2) / (2 * l1 * l2))
        theta_deg2 = 180 - np.rad2deg(phi_3)
        theta_2 = np.deg2rad(theta_deg2)

        # print('theta one: ', theta_1)
        # print('theta two: ', theta_2)
        th1.append(theta_1)
        th2.append(theta_2)
        x_trac.append(x)
        y_trac.append(y)
    return [th1, th2]
# print(x)
# plt.plot(x,y)
# plt.show()
# theta=(th1,th2)
# print(theta)
# print(x_trac)
# print(y_trac)
# plt.plot(x_trac,y_trac)
# plt.show()
