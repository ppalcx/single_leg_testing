from numpy import *
import math
import matplotlib.pyplot as plt
a =1
b= 1
hip_position=[]
x_knee= []
y_knee = []
for theta in range(0,100,1):
    theta_1 = 30 + 30*math.sin(2*math.pi*theta * 0.01)
    theta_h = math.radians(theta_1)
    theta_k = -math.radians(theta_1 - 90) + theta_h

    x_h = a * math.cos(theta_h)
    x_K = x_h + b * math .cos(theta_k)
    y_h = a * math.sin(theta_h)
    y_K = y_h + b * math.sin(theta_k)

    x_knee.append(x_K)
    y_knee.append(y_K)
    # hip_position.append(hip_pos)

plt.plot(x_knee, y_knee)
plt.show()
