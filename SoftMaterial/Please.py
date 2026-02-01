import time
import cv2
import threading
import code
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.VideoWriter import VideoWriter
from ClassFiles.Control_Water import Control_Water
from ClassFiles.Arduino import Arduino
from ClassFiles.RealTimeData_Recorder import RealTimeData_Recorder
from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK
from pynput import keyboard
import numpy as np

class Please:
    def __init__(self):
        print("Please...")
        print("-------------------------")
        print("")

        # Robot Controller

        self.RC = RC = RobotController_SDK()
        self.RC_lock = self.RC.lock

        # Vision
        self.VS = Vision()
        self.VS.GUI = False
        self.VS_lock = self.VS.lock

        # Recorder
        self.VW = VideoWriter(self.VS)

        # Arduino
        self.AD = Arduino()

        # PID Control
        self.CT = Control_Water()
        self.CT.a = 10.7091
        self.CT.b = -993.3455

        self.RobotOffset = [425,112.5,280]
        self.Running = True
        self.PWM_ONOFF = False
        self.StartTime = None

        self.ControlData = RealTimeData_Recorder()
        self.DataName_1 = "ControlData"
        self.ControlData.DefineData(self.DataName_1, ["Z_Error", "Z_System", "Z_Target", "PWM"])

        self.VisionData = RealTimeData_Recorder()
        self.DataName_2 = "VisionData"
        self.VisionData.DefineData(self.DataName_2, ["x", "y", "z"])

        self.RobotData = RealTimeData_Recorder()
        self.DataName_3 = "RobotData"
        self.RobotData.DefineData(self.DataName_3, ["x", "y", "z", 'rx', 'ry', "rz"])

        print("-------------------------")
        print("I'm Ready!")
        print("")
        print("")


    def Start(self):

        # Vision Thread
        VS_Thread = threading.Thread(target=self.VS.Tracking, daemon=True)
        VS_Thread.start()
        time.sleep(5)

        # VideoWirter Thread
        VW_Thread = threading.Thread(target=self.VW.Capture, daemon=True)
        VW_Thread.start()

        self.StartTime = time.time()
        while self.Running:
            time.sleep(self.CT.SamplingTime)
            with self.VS_lock, self.RC_lock:
                TargetPose = self.VS.Position
                TargetPose = [x*1000 for x in TargetPose]
                RobotPose = self.RC.EE_Position
                RobotRot = self.RC.EE_Rotation

            RobotOffset = self.RobotOffset
            SystemPose = [RobotPose[0] - RobotOffset[0], RobotPose[1] - RobotOffset[1], RobotPose[2] - RobotOffset[2]]

            PWM = self.CT.Get_PWM(SystemPose[2], TargetPose[2])

            if self.PWM_ONOFF:
                self.AD.Send_PWM([PWM, PWM, PWM])
            else:
                self.AD.Send_PWM([0, 0, 0])

            self.ControlData.AppendData(self.DataName_1,
                                        [(self.CT.Z_Reference - (SystemPose[2] - TargetPose[2])), SystemPose[2],
                                         TargetPose[2], PWM])
            self.VisionData.AppendData(self.DataName_2, TargetPose)
            self.RobotData.AppendData(self.DataName_3, [RobotPose[0], RobotPose[1], RobotPose[2], RobotRot[0], RobotRot[1], RobotRot[2]])





    def Pose(self):
        with self.VS_lock, self.RC_lock:
            TargetPose = self.VS.Position
            TargetPose = [x * 1000 for x in TargetPose]
            RobotPose = self.RC.EE_Position
        RobotOffset = self.RobotOffset
        SystemPose = [RobotPose[0] - RobotOffset[0], RobotPose[1] - RobotOffset[1], RobotPose[2] - RobotOffset[2]]
        print(f"Target: {[round(x,2) for x in TargetPose]} [mm]")
        print(f"System: {[round(x,2) for x in SystemPose]} [mm]")
        print(f"Z, Error: {[round((SystemPose[2] - TargetPose[2]),2), round(self.CT.Z_Reference - (SystemPose[2] - TargetPose[2]), 2)]} [mm]")
        print("")



    def PWM_Switch(self, Value = None):
        if Value == None:
            with self.VS_lock, self.RC_lock:
                TargetPose = self.VS.Position
                TargetPose = [x*1000 for x in TargetPose]
                RobotPose = self.RC.EE_Position
            RobotOffset = self.RobotOffset
            SystemPose = [RobotPose[0] - RobotOffset[0], RobotPose[1] - RobotOffset[1], RobotPose[2] - RobotOffset[2]]
            PWM = self.CT.Get_PWM(SystemPose[2], TargetPose[2])
            z = SystemPose[2] - TargetPose[2]
            print(f"Eql PWM: {round(self.CT.a * (z - self.CT.Z_Reference) + self.CT.PWM_Reference + self.CT.beta)}")
            print(f"PID PWM: {PWM}")
            print("")

        elif Value == 0:
            self.PWM_ONOFF = False
            print("PWM OFF")
            print("")

        else:
            self.PWM_ONOFF = True
            print("PWM ON")
            print("")



    def Handle(self):
        banner = "\n Waiting Your Order..."
        locals_dict = {'RC': self.RC,
                       'VS': self.VS,
                       'CT': self.CT,
                       'AD': self.AD,
                       'MoveJoint': self.RC.MoveJoint,
                       'MoveRel': self.RC.MoveRel,
                       'MoveAbs': self.RC.MoveAbs,
                       'GetPose': self.RC.GetPose,
                       'GetJoint': self.RC.GetJoint,
                       'InitPose': self.RC.MoveInit,
                       'HomePose': self.RC.MoveHome,
                       'Wait': self.RC.Wait,

                       'Pose': self.Pose,
                       'PWM': self.PWM_Switch,
                       'MPWM': self.AD.ManualPWM,
                       'PLS': self}

        code.interact(banner=banner, local=locals_dict)

        print("Handle Closed!")


    def SaveResults(self):
        print("Saving Results...")
        # Align Vision Timestamp to Reference
        self.ControlData.Data[self.DataName_1]["Time"]["StartTime"] = self.StartTime
        self.VisionData.Data[self.DataName_2]["Time"]["StartTime"] = self.StartTime
        self.RobotData.Data[self.DataName_3]["Time"]["StartTime"] = self.StartTime

        # Save Data
        self.ControlData.SaveData(self.DataName_1, self.DataName_1, SavePath='Results')
        self.VisionData.SaveData(self.DataName_2, self.DataName_2, SavePath='Results')
        self.RobotData.SaveData(self.DataName_3, self.DataName_3, SavePath='Results')


    def ManualPWMControl(self):

        # Vision Thread
        VS_Thread = threading.Thread(target=self.VS.Tracking, daemon=True)
        VS_Thread.start()
        time.sleep(5)

        def PWM_On(key):
            if key == keyboard.Key.space:
                self.AD.PWM_OnOff = True

        def PWM_Off(key):
            if key == keyboard.Key.space:
                self.AD.PWM_OnOff = False
                return

            # 2️⃣ 일반 키 처리
            try:
                if key.char == 'q':  # 일반 문자 키
                    self.AD.Send_PWM([0, 0, 0])
                    self.AD.Running = False
                    return False  # 리스너 종료
            except AttributeError:
                pass

        listener = keyboard.Listener(on_press=PWM_On, on_release=PWM_Off)
        listener.start()


        print("pls...")
        try:
            while self.AD.Running:
                # ESC 누르면 종료
                if self.AD.PWM_OnOff:
                    TargetPose = self.VS.Position
                    PWM = round(10 * 1.1 * (90-TargetPose[2]*1000 - 98) + 90)
                    PWM = round(float(np.clip(PWM, 0, 255)))
                    self.AD.Send_PWM([PWM, PWM, PWM])
                    print(f"On:{[PWM, round(TargetPose[2]*1000,2)]}")
                else:
                    self.AD.Send_PWM([0,0,0])
                    # print("Off")

                time.sleep(50/1000)
        except KeyboardInterrupt:
            self.AD.Send_PWM([0,0,0])

        self.AD.Running = True