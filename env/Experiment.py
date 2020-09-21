import pybullet as p
import time
import pybullet_data
import numpy as np
import math
import matplotlib.pyplot as plt
from Inverse_kinematics import trajectory_ellipse
from ellipse_trajectory import trajectory
hip_motor_id=0
knee_motor_id=1
spring_motor_id=2
'''import joblib
theta_knee = joblib.load('theta_knee.obj')
theta_hip = joblib.load('theta_hip.obj')
print(theta_knee)
print(theta_hip)'''
theta_ellipse = trajectory_ellipse()
theta_pair = list(zip(*theta_ellipse))
print(theta_pair)
# thetaS = trajectory()
# print(thetaS[0])

def ResetLeg(standstilltorque=0):
    p.resetJointState(robot,hip_motor_id,targetValue=0, targetVelocity=0)
    p.resetJointState(robot,knee_motor_id, targetValue=0, targetVelocity=0)
    p.setJointMotorControl2(
        bodyIndex=robot,
        jointIndex=hip_motor_id,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=standstilltorque)
    p.setJointMotorControl2(
        bodyIndex=robot,
        jointIndex=knee_motor_id,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=standstilltorque)
    p.setJointMotorControl2(
        bodyIndex=robot,
        jointIndex=spring_motor_id,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=standstilltorque)


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
robotStartPos = [0,0,0.20]
robotStartOrientation = p.getQuaternionFromEuler([np.pi/2,0,0])
robot = p.loadURDF("/home/pramod/Single_leg_test/Urdf/Test_1/urdf/Test_1.urdf",robotStartPos, robotStartOrientation,useFixedBase=1)
ResetLeg()
for i in range (10000):

    for theta in theta_pair:

        p.setJointMotorControl2(
                bodyIndex=robot,
                jointIndex=hip_motor_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=theta[0],
                force=10)
        p.setJointMotorControl2(
                bodyIndex=robot,
                jointIndex=knee_motor_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=theta[1],
                force=10)
        p.stepSimulation()
        time.sleep(1. / 240.)
#     for j in range(len(spring_deflection)):
#
#         p.setJointMotorControl2(
#             bodyIndex=robot,
#             jointIndex=spring_motor_id,
#             controlMode=p.VELOCITY_CONTROL,
#             targetVelocity=0,
#             force=10)
#         time.sleep(1./240.)
#     spring_deflection=spring_def()
#     z_f=-k*spring_deflection+c*spring velocity
#     apply_external_force(z_f,spring_motor_id)
#
# def spring_def():
#     spring_deflection=p.getJointState(robot,spring_motor_id)[0]
#     return spring_deflection
# def apply_external_force(z_f,spring_motor_id):
#     p.applyExternalForces(robot,spring_motor_id,forceObj=[z_f,0,0],posObj=[0,0,0],flags=p.LINK_FRAME)
robotPos, robotOrn = p.getBasePositionAndOrientation(robot)
print(theta_pair)
# plt.plot(hip_pos)
# plt.show()
# print(robotPos,robotOrn)
p.disconnect()
