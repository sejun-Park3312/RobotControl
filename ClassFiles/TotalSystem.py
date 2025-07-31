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
        print("TotalSystem Initializing...")
        print("-------------------------")
        print("")

        self.VS = Vision()
        self.CT = Control()
        self.AD = Arduino()
        self.RC = RobotController()

        self.VS_lock = self.VS.lock
        self.RC_lock = self.RC.lock
        self.StartTime = None
        self.Running = True
        self.PWM_ONOFF = False

        self.ControlData = RealTimeData_Recorder()
        self.DataName_1 = "ControlData"
        self.ControlData.DefineData(self.DataName_1, ["Z_Error", "Z_System", "Z_Target", "PWM"])

        self.VisionData = RealTimeData_Recorder()
        self.DataName_2 = "VisionData"
        self.VisionData.DefineData(self.DataName_2, ["x_target", "y_target", "z_target"])

        self.RobotData = RealTimeData_Recorder()
        self.DataName_3 = "RobotData"
        self.RobotData.DefineData(self.DataName_3, ["x_robot", "y_robot", "z_robot", 'rx', 'ry', "rz"])

        print("-------------------------")
        print("TotalSystem Ready!")
        print("")
        print("")


    def Start(self):
        # Thread
        RobotTracking_Thread = threading.Thread(target=self.RC.Track_EE, daemon=True)
        RobotTracking_Thread.start()

        self.VS.GUI = False
        VisionTracking_Thread = threading.Thread(target=self.VS.Tracking, daemon=True)
        VisionTracking_Thread.start()


        print("Start TotalSystem!")
        while self.Running:
            self.StartTime = time.time()
            while self.Running:

                time.sleep(self.CT.SamplingTime)
                with self.VS_lock:
                    self.CT.Z_Target = self.VS.Z
                    self.VisionData.AppendData(self.DataName_2,[self.VS.Position[0], self.VS.Position[1], self.VS.Position[2]])

                with self.RC_lock:
                    self.CT.Z_System = (self.RC.EE_Position[2] - (self.RC.System_Offset[2] + self.RC.P_base2world[2])) / 1000
                    self.RobotData.AppendData(self.DataName_3,[self.RC.EE_Position[0], self.RC.EE_Position[1], self.RC.EE_Position[2], self.RC.EE_Rotation[0], self.RC.EE_Rotation[1], self.RC.EE_Rotation[2]])

                PWM = self.CT.Get_PWM()

                if self.PWM_ONOFF:
                    self.AD.Send_PWM(PWM)
                else:
                    self.AD.Send_PWM(0)

                self.ControlData.AppendData(self.DataName_1,[(self.CT.Z_Reference - (self.CT.Z_System - self.CT.Z_Target)), self.CT.Z_System, self.CT.Z_Target, PWM])



    def Print_SystemPose(self):
        with self.RC_lock:
            SystemX = (self.RC.EE_Position[0] - (self.RC.System_Offset[0] + self.RC.P_base2world[0]))
            SystemY = (self.RC.EE_Position[1] - (self.RC.System_Offset[1] + self.RC.P_base2world[1]))
            SystemZ = (self.RC.EE_Position[2] - (self.RC.System_Offset[2] + self.RC.P_base2world[2]))
        print(f"System: {round(SystemX,2)}mm, {round(SystemY,2)}mm, {round(SystemZ,2)}mm")
        print("")


    def Print_TargetPose(self):
        with self.VS_lock:
            TargetX = self.VS.Position[0]
            TargetY = self.VS.Position[1]
            TargetZ = self.VS.Position[2]

        print(f"Target: {round(TargetX*1000,2)}mm, {round(TargetY*1000,2)}mm, {round(TargetZ*1000,2)}mm")
        print("")


    def Print_PWM(self):
        with self.VS_lock, self.RC_lock:
            PWM = self.CT.Get_PWM()
        print(f"PWM: {round(PWM)}")
        print("")


    def Print_Error(self):
        with self.RC_lock, self.VS_lock:
            SystemX = (self.RC.EE_Position[0] - (self.RC.System_Offset[0] + self.RC.P_base2world[0])) / 1000
            SystemY = (self.RC.EE_Position[1] - (self.RC.System_Offset[1] + self.RC.P_base2world[1])) / 1000
            SystemZ = (self.RC.EE_Position[2] - (self.RC.System_Offset[2] + self.RC.P_base2world[2])) / 1000

            TargetX = self.VS.Position[0]
            TargetY = self.VS.Position[1]
            TargetZ = self.VS.Position[2]

        ErrorX = -SystemX + TargetX
        ErrorY = -SystemY + TargetY
        ErrorZ = -SystemZ + TargetZ + self.CT.Z_Reference

        print(f"Error: {round(ErrorX*1000,2)}mm, {round(ErrorY*1000,2)}mm, {round(ErrorZ*1000,2)}mm")
        print("")



    def TotalSystemController(self):
        banner = "\n Waiting Your Order..."
        locals_dict = {'MoveJoint': self.RC.Move_Joint,
                       'MoveRel': self.RC.Move_Rel,
                       'MoveAbs': self.RC.Move_Abs,
                       'GetPose': self.RC.Get_Pose,
                       'GetJoint': self.RC.Get_Joint,
                       'HomePose': self.RC.Move_Home,
                       'InitPose': self.RC.Init_Pose,
                       'Wait': self.RC.Wait,

                       'Target': self.Print_TargetPose,
                       'System': self.Print_SystemPose,
                       'Error': self.Print_Error,
                       'PWM': self.Print_PWM,
                       'TS': self}

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
