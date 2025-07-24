import numpy as np
import math as m


class RobotTransformation:

    def __init__(self):
        self.RefPose = [0,-34.5,-397.5,0,0,0]


    def rpy2rotm(self, roll, pitch, yaw):
        """
        Roll, Pitch, Yaw (in radians) -> Rotation Matrix (3x3)
        """
        roll = roll * m.pi / 180.0
        pitch = pitch * m.pi / 180.0
        yaw = yaw * m.pi / 180.0

        # Rotation about X axis (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll),  np.cos(roll)]
        ])
        # Rotation about Y axis (pitch)
        Ry = np.array([
            [ np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        # Rotation about Z axis (yaw)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0, 0, 1]
        ])
        # ZYX 순서로 회전: R = Rz * Ry * Rx
        R = Rz @ Ry @ Rx
        return R

    def rotm2rpy(self,R):
        """
        Rotation Matrix (3x3) -> Roll, Pitch, Yaw (in radians)
        Returns tuple (roll, pitch, yaw)
        """
        # pitch 계산 (singularity 체크)
        if abs(R[2,0]) < 1:
            pitch = -np.arcsin(R[2,0])
            roll = np.arctan2(R[2,1]/np.cos(pitch), R[2,2]/np.cos(pitch))
            yaw  = np.arctan2(R[1,0]/np.cos(pitch), R[0,0]/np.cos(pitch))
        else:
            # Gimbal lock 상태 (pitch == +-90도)
            pitch = np.pi/2 if R[2,0] <= -1 else -np.pi/2
            roll = 0
            yaw = np.arctan2(-R[0,1], R[1,1])

            roll = roll * 180.0 / m.pi
            pitch = pitch * 180.0 / m.pi
            yaw = yaw * 180.0 / m.pi

        return roll, pitch, yaw

    def Pose2Tmatrix(self, Pose):
        rotm = self.rpy2rotm(Pose[3], Pose[4],Pose[5])
        xyz = np.array([Pose[0], Pose[1], Pose[2]])
        T = np.eye(4)
        T[:3,:3] = rotm
        T[:3,3] = xyz
        return T
