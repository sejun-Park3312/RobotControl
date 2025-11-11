import time
import math
import numpy as np
from ProjectPath import PROJECT_PATH
from simple_pid import PID
from ClassFiles.BasicMagnetFuns import BasicMagnetFuns

class Control_SiliconOil:
    def __init__(self):
        print("Controller Initializing...")
        # Basic Magnet Functions
        self.BF = BasicMagnetFuns()

        # Distance Offsets
        self.Z_Reference = 90 / 1000 # system(센터 코일 높이)과 Target 사이 Reference 거리
        # self.SystemPose = [0,0,85/1000] # system 높이(World 좌표계 기준)
        # self.TargetPose = [0,0,0] # Target 높이(World 좌표계 기준)

        # Array
        self.C_Points, self.C_Angles, self.M_Points, self.M_Angles = self.Array()

        # DipoleMoment Magnitude
        self.Ms = 2
        self.Mc = 0.92 # NA
        self.Mt = 0.024

        # Mechanical Properties
        self.F_Buoyance = 0.005146777750500
        self.Weight = 0.006776951342543
        self.I_Max = 1.5
        self.alpha = 0.55
        self.beta = 1
        self.theta = 0
        self.F_pid = 0
        self.a = 1
        self.b = 1


        # PID
        self.SamplingTime = 25 / 1000
        self.Kp = 1e-1/3
        self.Kd = 1e-2
        self.Ki = 0
        self.pid = None
        self.setPID()

        print("Controller Ready!")
        print("")
        print("")


    def setPID(self):
        self.pid = PID(Kp=self.Kp, Kd=self.Kd, Ki=self.Ki, setpoint=0)
        self.pid.sample_time = self.SamplingTime



    def Array(self):
        # Coil/Magent Array
        Data = np.load("../Data/Array_Data/Data.npz")
        C_Points = Data['C_Points']
        C_Angles = Data['C_Angles']
        M_Points = Data['M_Points']
        M_Angles = Data['M_Angles']

        C_Points[:,2] = 0
        M_Points[:,2] = 40/1000
        return C_Points, C_Angles, M_Points, M_Angles



    def Angle2Direction(self, Angle):
        Direction = np.array([math.sin(Angle[1]) * math.cos(Angle[0]), math.sin(Angle[1]) * math.sin(Angle[0]), math.cos(Angle[1])])
        return Direction



    def MagnetArray_Force(self, SystemPose, TargetPose):
        m_target = np.array([1, 0, 0]) * self.Mt
        F = np.array([[0], [0], [0]])
        for i in range(self.M_Points.shape[0]):
            m_source = self.Angle2Direction(self.M_Angles[i, :]) * self.Ms
            # World 좌표계 기준
            r_target = np.array([TargetPose])
            r_source = self.M_Points[i, :] + np.array([SystemPose])
            r_source2target = r_target - r_source
            F = F + self.BF.Cal_MagnetForce(r_source2target, m_source, m_target)

        Fz = F[2]
        return Fz


    # def CoilArray_ACoeff(self, SystemPose, TargetPose):
    #     m_target = np.array([1, 0, 0]) * self.Mt
    #     A_vec = np.array([[0], [0], [0]])
    #     for i in range(self.C_Points.shape[0]):
    #         m_source_i = self.Angle2Direction(self.C_Angles[i, :]) * self.Mc
    #         # World 좌표계 기준
    #         r_target = np.array([TargetPose])
    #         r_source = self.C_Points[i, :] + np.array([SystemPose])
    #         r_source2target = r_target - r_source
    #         A_vec = A_vec + self.BF.Cal_MagnetForce(r_source2target, m_source_i, m_target)
    #
    #     Az_Coeff = A_vec[2]
    #     return Az_Coeff


    def CoilArray_ACoeff(self, SystemPose, TargetPose):
        m_target = np.array([1, 0, 0]) * self.Mt
        A_vec_px = np.array([[0], [0], [0]])

        r_target = np.array([TargetPose])

        for i in [0,3,4]:
            m_source_i = self.Angle2Direction(self.C_Angles[i, :]) * self.Mc
            # World 좌표계 기준
            r_source = self.C_Points[i, :] + np.array([SystemPose])
            r_source2target = r_target - r_source
            A_vec_px = A_vec_px + self.BF.Cal_MagnetForce(r_source2target, m_source_i, m_target)

        A_vec_nx = np.array([[0], [0], [0]])
        for i in [1,2,5]:
            m_source_i = self.Angle2Direction(self.C_Angles[i, :]) * self.Mc
            # World 좌표계 기준
            r_source = self.C_Points[i, :] + np.array([SystemPose])
            r_source2target = r_target - r_source
            A_vec_nx = A_vec_nx + self.BF.Cal_MagnetForce(r_source2target, m_source_i, m_target)

        A_vec_center = np.array([[0], [0], [0]])
        for i in [6,7,8]:
            m_source_i = self.Angle2Direction(self.C_Angles[i, :]) * self.Mc
            # World 좌표계 기준
            r_source = self.C_Points[i, :] + np.array([SystemPose])
            r_source2target = r_target - r_source
            A_vec_center = A_vec_center + self.BF.Cal_MagnetForce(r_source2target, m_source_i, m_target)

        theta = np.clip(self.theta, -1, 1)
        Az_Coeff = (1-theta)*A_vec_px[2] + (1+theta)*A_vec_nx[2] + A_vec_center[2]
        return Az_Coeff


    def Get_PWM(self, SystemPose, TargetPose):
        TargetPose = [TargetPose[0] * self.beta, TargetPose[1] * self.beta, TargetPose[2]]
        SystemPose = [SystemPose[0] * self.beta, SystemPose[1] * self.beta, SystemPose[2]]

        Z_Error = self.Z_Reference/1000 - (SystemPose[2] - TargetPose[2])
        F_pid = self.pid(Z_Error, dt = self.SamplingTime)
        I = ((F_pid -self.MagnetArray_Force(SystemPose, TargetPose) + self.alpha * (-self.F_Buoyance + self.Weight))
             /self.CoilArray_ACoeff(SystemPose, TargetPose))

        PWM = round(float(np.clip(I, 0, self.I_Max)*255/self.I_Max))

        theta = np.clip(self.theta, -1, 1)
        PWM_List = [round((1-theta)*PWM),round((1+theta)*PWM),round(PWM)]

        return PWM_List
