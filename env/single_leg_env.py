import pybullet as p
import time
import pybullet_data
import numpy as np
import math
hip_motor_id=0
knee_motor_id=1


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


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
robotStartPos = [0,0,.25]
robotStartOrientation = p.getQuaternionFromEuler([np.pi/2,0,0])
robot = p.loadURDF("/home/pramod/Single_leg_test/Urdf/Test_1/urdf/Test_1.urdf",robotStartPos, robotStartOrientation,useFixedBase=1)
ResetLeg()
for i in range (10000):
    p.stepSimulation()
    for theta in range(0,360,10):
        hip_pos=math.sin(math.radians(theta))*math.radians(30)
        print(hip_pos)
        p.setJointMotorControl2(
                bodyIndex=robot,
                jointIndex=hip_motor_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=hip_pos,
                force=10)
        p.setJointMotorControl2(
                bodyIndex=robot,
                jointIndex=knee_motor_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0,
                force=10)
        time.sleep(1./240.)
robotPos, robotOrn = p.getBasePositionAndOrientation(robot)
print(robotPos,robotOrn)
p.disconnect()
