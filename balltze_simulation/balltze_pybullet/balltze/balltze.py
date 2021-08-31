import pybullet as p
import time
import numpy as np
import pybullet_data
from balltze_description import Balltze, BalltzeKinematics
import math

if __name__ == '__main__':
    time_step = 1./240.
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.setTimeStep(time_step)
    planeId = p.loadURDF('plane.urdf')

    robot = Balltze('../../../balltze_description/balltze_description/urdf/balltze.urdf', p, position=[0,0,0.11])
    kinematics = BalltzeKinematics(None)

    i = 0.0
    dir = 1
    while True:
        try:
            ends = kinematics.body_inverse([0.0,0.0,i], [0.0,i/10,0.02], [[0.1, -0.1, -0.06],[0.1, 0.06, -0.02],[-0.1, -0.06, -0.06],[-0.1, 0.06, -0.06]])
            joints = kinematics.inverse(ends)
            robot.set_joint_arr(np.array(joints.T).reshape(1,12)[0])
            # print((kinematics.forward_leg(joints)*1000).astype(np.int64)/1000)
            # print(joints)
            # print(ends)
        except Exception as e:
            print(e)
        i += dir*0.0007
        if i >= np.pi/10:
            dir = -1
        if i <= -np.pi/10:
            dir = 1

        # robot.set_joint_arr([0, -np.pi/2, np.pi/2]*4)
        p.stepSimulation()
        time.sleep(time_step)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(robot)
    print(cubePos,cubeOrn)
    p.disconnect()
