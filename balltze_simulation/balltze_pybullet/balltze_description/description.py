import pybullet as p
from typing import Optional
import math

class Balltze:
    def __init__(self,
                 path,
                 client,
                 position: Optional[list] = [0,0,0],
                 orientation: Optional[list] = [0.0, 0.0, 0.0, 0.06*math.sqrt(2)]):
        self._p = client
        self._id = self._p.loadURDF(
                path,position,
                orientation,
                useFixedBase=False,
                flags=self._p.URDF_MERGE_FIXED_LINKS | \
                      self._p.URDF_USE_INERTIA_FROM_FILE)
        # self._p.enableJointForceTorqueSensor(self._id, True)
        self._joint_names = {
            'fr_coxa': 1,
            'fr_femur': 2,
            'fr_tibia': 3,
            'fl_coxa': 4,
            'fl_femur': 5,
            'fl_tibia': 6,
            'rr_coxa': 7,
            'rr_femur': 8,
            'rr_tibia': 9,
            'rl_coxa': 10,
            'rl_femur': 11,
            'rl_tibia': 12
        }

        self._lower_limit = [p.getJointInfo(self._id, i)[8] for i in range(12)]
        self._upper_limit = [p.getJointInfo(self._id, i)[9] for i in range(12)]
        self._max_effort = [p.getJointInfo(self._id, i)[10] for i in range(12)]
        self._max_velocity = [p.getJointInfo(self._id, i)[11] for i in range(12)]

        self.set_joint_arr(
            [-math.pi/16, -math.pi/2, math.pi/2] + \
            [+math.pi/16, -math.pi/2, math.pi/2] + \
            [-math.pi/16, -math.pi/2, math.pi/2] + \
            [+math.pi/16, -math.pi/2, math.pi/2],
            [0]*12,
            self._max_effort
            )


    def set_joint(self, idx, pos, vel=None, eff=None):
        '''set joint by id'''
        if vel:
            vel = min(vel, self._max_velocity[idx])
        else:
            vel = self._max_velocity[idx]

        if eff:
            eff = min(eff, self._max_effort[idx])
        else:
            eff = self._max_effort[idx]

        p.setJointMotorControl2(
            bodyUniqueId=self._id, 
            jointIndex=idx, 
            controlMode=self._p.POSITION_CONTROL,
            targetPosition = pos,
            targetVelocity = vel,
            force = eff,
            maxVelocity = 3.5
        )


    def set_joint_by_name(self, name, pos, vel=None, eff=None):
        '''set joint by name'''
        idx = self._joint_names[name]
        self.set_joint(idx, pos, vel, eff)


    def set_joint_arr(self, pos, vel=None, eff=None):
        '''set all joints at once'''
        if not vel:
            vel = self._max_velocity
        if not eff:
            eff = self._max_effort

        for i in range(12):
            self.set_joint(
                i, 
                pos[i],
                vel[i],
                eff[i]
            )

    def joint_state(self, idx):
        '''return state of given joint'''
        state = self._p.getJointState(self._id, idx)
        return [state[0], state[1], state[3]]


    @property
    def num_joint(self):
        '''return number of robot's joints'''
        return self._p.getNumJoints(self._id)


    def reset(self, position, orientation):
        '''resets robot to initial state'''
        self._p.resetBasePositionAndOrientation(self._id, position, orientation)
        for i in range(12):
            self._p.resetJointStat(self._id, i)


    @property
    def position(self):
        '''return current position of robot'''
        return self._p.getBasePositionAndOrientation(self._id)[0]


    @property
    def orientation(self):
        '''returns current orientation of robot'''
        return self._p.getBasePositionAndOrientation(self._id)[1]


    @property
    def id(self):
        '''return robot's model id given by connected client'''
        return self._id


    @property
    def client(self):
        '''return client to which robot is connected'''
        return self._p
