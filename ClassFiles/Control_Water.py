import time
import math
import numpy as np
from ProjectPath import PROJECT_PATH
from simple_pid import PID

class Control_Water:
    def __init__(self):
        print("Controller Initializing...")

        # Distance Offsets
        self.Z_Reference = 103 / 1000 # system(센터 코일 높이)과 Target 사이 Reference 거리
        self.P_World2VisionHomePose = [450, 25, 270] # Vision에서 측정한 값의 Zero Position이 World 좌표계 기준일 때의 위치
        self.SystemPose = [450, 25, 400 + 83.5] # System 위치 (중앙 코일 중심점 위치)
        self.TargetPose = [0,0,0] # Target 높이(World 좌표계 기준)

        # Control Parameters
        self.a = 10.7091
        self.b = -933.3455
        self.alpha = 0
        self.beta = 0

        # PID
        self.SamplingTime = 25 / 1000
        self.Kp = 1e-1/2
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


    def Get_PWM(self, z_System, z_Target):
        z = z_System - z_Target
        z_err = self.Z_Reference - z
        x = self.pid(z_err, dt = self.SamplingTime)
        y = self.a * z + self.b - x

        PWM = round(float(np.clip(y, 0, 255)))
        return PWM
