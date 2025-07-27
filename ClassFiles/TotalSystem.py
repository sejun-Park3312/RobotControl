import time
import cv2
import threading
import numpy as np
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
        self.ControlData.DefineData(self.DataName_2, ["x_target", "y_target", "z_target"])

        self.RobotData = RealTimeData_Recorder()
        self.DataName_3 = "RobotData"
        self.ControlData.DefineData(self.DataName_3, ["x_robot", "y_robot", "z_robot"])



    def Start(self):
        # Thread
        VisionTracking_Thread = threading.Thread(target=self.VS.Tracking, daemon=True)
        VisionTracking_Thread.start()
        RobotTracking_Thread = threading.Thread(target=self.RC.Track_EE, daemon=True)
        RobotTracking_Thread.start()
        RobotController_Thread = threading.Thread(target=self.RC.StartController, daemon=True)
        RobotController_Thread.start()

        self.StartTime = time.time()
        while self.Running:

            if cv2.waitKey(1) == 27 or self.VS.Running == False:
                self.AD.Running = False
                self.AD.Disconnect()
                self.Running = False
                print("Test Stopped!")
                break

            time.sleep(self.CT.SamplingTime)
            with self.VS_lock:
                self.CT.Z_Target = self.VS.Z
                self.VisionData.AppendData(self.DataName_2,[self.VS.Position[0], self.VS.Position[1], self.VS.Position[2]])

            with self.RC_lock:
                self.CT.Z_System = self.RC.EE_Position[2] - (self.RC.TCP_Offset[2] + self.RC.P_base2world[2])
                self.RobotData.AppendData(self.DataName_3,[self.RC.EE_Position[0], self.RC.EE_Position[1], self.RC.EE_Position[2]])

            PWM = self.CT.Get_PWM()

            self.ControlData.AppendData(self.DataName_1, [(self.CT.Z_Reference - (self.CT.Z_System - self.CT.Z_Target)),
                                                     self.CT.Z_System, self.CT.Z_Target, PWM])

        self.RC.EndController()
        self.AD.Disconnect()
        print("TotalSystem Ended!")
        self.SaveResults()



    def SaveResults(self):
        print("Saving Results...")
        # Align Vision Timestamp to Reference
        self.ControlData.Data[self.DataName_1]["Time"]["StartTime"] = self.StartTime
        self.VisionData.Data[self.DataName_2]["Time"]["StartTime"] = self.StartTime
        self.RobotData.Data[self.DataName_3]["Time"]["StartTime"] = self.StartTime

        # Save Data
        self.ControlData.SaveData(self.DataName_1, self.DataName_1)
        self.VisionData.SaveData(self.DataName_2, self.DataName_2)
        self.RobotData.SaveData(self.DataName_3, self.DataName_3)
