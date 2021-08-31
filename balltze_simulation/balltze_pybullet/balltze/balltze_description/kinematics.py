import numpy as np

class BalltzeKinematics:
    def __init__(self, description_yaml=None):
        self._coxa_len = 0.01
        self._femur_len = 0.06
        self._tibia_len = 0.06
        self._length = 0.2
        self._width = 0.1

        self._Tfr = np.array([[ 1.0, 0.0, 0.0, self._length/2],
                              [ 0.0, 1.0, 0.0, -self._width/2],
                              [ 0.0, 0.0, 1.0,            0.0],
                              [ 0.0, 0.0, 0.0,            1.0]])

        self._Tfl = np.array([[ 1.0, 0.0, 0.0, self._length/2],
                              [ 0.0, 1.0, 0.0,  self._width/2],
                              [ 0.0, 0.0, 1.0,            0.0],
                              [ 0.0, 0.0, 0.0,            1.0]])

        self._Trr = np.array([[ 1.0, 0.0, 0.0, -self._length/2],
                              [ 0.0, 1.0, 0.0,  -self._width/2],
                              [ 0.0, 0.0, 1.0,             0.0],
                              [ 0.0, 0.0, 0.0,             1.0]])

        self._Trl = np.array([[ 1.0, 0.0, 0.0, -self._length/2],
                              [ 0.0, 1.0, 0.0,   self._width/2],
                              [ 0.0, 0.0, 1.0,             0.0],
                              [ 0.0, 0.0, 0.0,             1.0]])



    def inverse(self, pos):
        '''returns inverse kinematics of leg, can be vectorized'''
        axis_mirror = np.array([-1, 1, -1, 1])
        x = np.array(pos[:,0])
        y = np.array(pos[:,1]) * axis_mirror
        z = np.array(pos[:,2])

        theta_1 = np.arctan2(z, y) - np.arctan2(np.sqrt(y**2 + z**2 - self._coxa_len**2), self._coxa_len) + np.pi

        D = (y**2 + z**2 + x**2 - self._coxa_len**2 - self._femur_len**2 - self._tibia_len**2) / (2 * self._femur_len * self._tibia_len)
        D2 = D**2
        if (D2 > 1).any():
            raise ValueError('Desired point is out of reachable space')

        theta_3 = np.arctan2(np.sqrt(np.abs(1-D2)), D)

        theta_2 = np.arctan2(x, np.sqrt(y**2 + z**2 - self._coxa_len**2)) \
                    - np.arctan2(self._tibia_len * np.sin(theta_3), self._femur_len + self._tibia_len * np.cos(theta_3))


        return np.array([theta_1 * axis_mirror, theta_2, theta_3])


    def body_transform_matrix(self, rot, pos):
        '''returns inverse kinematics with respect to body rotation'''
        # roll
        rs = np.sin(rot[0])
        rc = np.cos(rot[0])
        # pitch
        ps = np.sin(rot[1])
        pc = np.cos(rot[1])
        # yaw
        ys = np.sin(rot[2])
        yc = np.cos(rot[2])

        # roll rotation matrix
        Rx = np.array([[1.0, 0.0, 0.0, 0.0],
                       [0.0,  rc, -rs, 0.0],
                       [0.0,  rs,  rc, 0.0],
                       [0.0, 0.0, 0.0, 1.0]])

        # pitch rotation matrix
        Ry = np.array([[ pc, -ps, 0.0, 0.0],
                       [ ps,  pc, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]])

        # yaw rotation matrix
        Rz = np.array([[ yc, 0.0,  ys, 0.0],
                       [0.0, 1.0, 0.0, 0.0],
                       [-ys, 0.0,  yc, 0.0],
                       [0.0, 0.0, 0.0, 1.0]])

        # body position transformation matrix
        Tb = np.array([[1.0, 0.0, 0.0, pos[0]],
                       [0.0, 1.0, 0.0, pos[1]],
                       [0.0, 0.0, 1.0, pos[2]],
                       [0.0, 0.0, 0.0,    1.0]])


        return Rx @ Ry @ Rz @ Tb



    def body_inverse(self, body_rot, body_pos, legs):
        Tb = self.body_transform_matrix(body_rot, body_pos)
        fr = ((Tb @ self._Tfr)[:3,-1] - legs[0]) * np.array([1, 1, -1])
        fl = ((Tb @ self._Tfl)[:3,-1] - legs[1]) * np.array([1, 1, -1])
        rr = ((Tb @ self._Trr)[:3,-1] - legs[2]) * np.array([1, 1, -1])
        rl = ((Tb @ self._Trl)[:3,-1] - legs[3]) * np.array([1, 1, -1])
        return np.array([fr, fl, rr, rl])

    
        

    def forward_leg(self, theta):
        '''returns forward kinematics for single leg'''
        st1 = np.sin(theta[0])
        ct1 = np.cos(theta[0])
        st2 = np.sin(theta[1])
        ct2 = np.cos(theta[1])
        st3 = np.sin(theta[2])
        ct3 = np.cos(theta[2])
        L1 = self._coxa_len
        L2 = self._femur_len
        L3 = self._tibia_len


        x = L2*st2 + L3*ct2*st3 + L3*ct3*st2
        y = L2*ct2*st1 - L1*ct1 + L2*ct2*ct3*st1 - L3*st1*st2*st3
        z = L3*ct1*st2*st3 - L2*ct1*ct2 - L3*ct1*ct2*ct3 - L1*st1

        return np.array([x, y, z])

if __name__ == '__main__':
    bk = BalltzeKinematics(None)
    # angles = np.array([0.5, -np.pi/4, np.pi/2])
    # x, y, z = bk.forward_leg(angles)
    # print((angles/np.pi*180).astype(np.int64))
    # print([x, y, z])
    # print((bk.inverse(x, y, z)/np.pi*180).astype(np.int64))

    # x, y, z = -0.02, -0.01, 0.06*np.sqrt(2)
    # angles = bk.inverse(x, y, z)
    # print(x, y, z)
    # print(bk.forward_leg(angles))
    a = bk.body_inverse([0,0.0,0.0], [0.0,0.0,0.0], [[0.1, -0.06, -0.06],[0.1, 0.06, -0.06],[-0.1, -0.06, -0.06],[-0.1, 0.06, -0.06]])
    print(a[:-1,0])