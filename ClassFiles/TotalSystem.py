import time
import cv2
import threading
import code
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.Control import Control
from ClassFiles.Arduino import Arduino
from ClassFiles.RealTimeData_Recorder import RealTimeData_Recorder
from ClassFiles.RobotController import RobotController



class TotalSystem:
    def __init__(self):
        self.VS = Vision()
        self.CT = Control()
        self.AD = Arduino()
        self.RC = RobotController()

        self.VS_lock = self.VS.lock
        self.RC_lock = self.RC.lock
        self.StartTime = None
        self.Running = True

        self.ControlData = RealTimeData_Recorder()
        self.DataName_1 = "ControlData"
        self.ControlData.DefineData(self.DataName_1, ["Z_Error", "Z_System", "Z_Target", "PWM"])

        self.VisionData = RealTimeData_Recorder()
        self.DataName_2 = "VisionData"
        self.VisionData.DefineData(self.DataName_2, ["x_target", "y_target", "z_target"])

        self.RobotData = RealTimeData_Recorder()
        self.DataName_3 = "RobotData"
        self.RobotData.DefineData(self.DataName_3, ["x_robot", "y_robot", "z_robot"])



    def VisionControl(self):

        print("Waiting VisionControl...")
        while self.VS.Running == False:
            time.sleep(1)

        print("Start VisionControl!")
        while self.Running:
            self.StartTime = time.time()
            while self.Running:

                time.sleep(self.CT.SamplingTime)
                with self.VS_lock:
                    self.CT.Z_Target = self.VS.Z
                    self.VisionData.AppendData(self.DataName_2,[self.VS.Position[0], self.VS.Position[1], self.VS.Position[2]])

                with self.RC_lock:
                    self.CT.Z_System = (self.RC.EE_Position[2] - (self.RC.TCP_Offset[2] + self.RC.P_base2world[2])) / 1000
                    self.RobotData.AppendData(self.DataName_3,[self.RC.EE_Position[0], self.RC.EE_Position[1], self.RC.EE_Position[2]])

                PWM = self.CT.Get_PWM()

                self.ControlData.AppendData(self.DataName_1,[(self.CT.Z_Reference - (self.CT.Z_System - self.CT.Z_Target)), self.CT.Z_System, self.CT.Z_Target, PWM])



    def Print_ZSystem(self):
        with self.VS_lock:
            Z_System = (self.RC.EE_Position[2] - (self.RC.TCP_Offset[2] + self.RC.P_base2world[2])) / 1000
        print(f"Z_System: {round(Z_System*1000,2)}mm")
        print("")


    def Print_ZTarget(self):
        with self.VS_lock, self.RC_lock:
            Z_target = self.VS.Z
        print(f"Z_Target: {round(Z_target*1000,2)}mm")
        print("")


    def Print_PWM(self):
        with self.VS_lock, self.RC_lock:
            PWM = self.CT.Get_PWM()
        print(f"PWM: {round(PWM)}")
        print("")


    def End_Vision(self):
        self.VS.EndVision()


    def TotalSystemController(self):
        banner = "\n Waiting Your Order..."
        locals_dict = {'MoveJoint': self.RC.Move_Joint,
                       'MoveRel': self.RC.Move_Rel,
                       'MoveAbs': self.RC.Move_Abs,
                       'GetPose': self.RC.Get_Pose,
                       'GetJoint': self.RC.Get_Joint,
                       'HomePose': self.RC.Move_Home,
                       'InitPose': self.RC.Init_Pose,

                       'Z_Target': self.Print_ZTarget,
                       'Z_System': self.Print_ZSystem,
                       'PWM': self.Print_PWM,
                       'End_Vision': self.End_Vision}

        code.interact(banner=banner, local=locals_dict)

        print("End TotalSystemController")


    def SaveResults(self):
        print("Saving Results...")
        # Align Vision Timestamp to Reference
        self.ControlData.Data[self.DataName_1]["Time"]["StartTime"] = self.StartTime
        self.VisionData.Data[self.DataName_2]["Time"]["StartTime"] = self.StartTime
        self.RobotData.Data[self.DataName_3]["Time"]["StartTime"] = self.StartTime

        # Save Data
        self.ControlData.SaveData(self.DataName_1, self.DataName_1, SavePath='../Data/Results')
        self.VisionData.SaveData(self.DataName_2, self.DataName_2, SavePath='../Data/Results')
        self.RobotData.SaveData(self.DataName_3, self.DataName_3, SavePath='../Data/Results')
