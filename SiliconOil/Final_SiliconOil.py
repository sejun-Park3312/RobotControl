import time
import cv2
import threading
import code
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.VideoWriter import VideoWriter
from ClassFiles.Arduino import Arduino
from ClassFiles.RealTimeData_Recorder import RealTimeData_Recorder
from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK
from Control_SiliconOil import Control_SiliconOil
from pynput import keyboard
import numpy as np

class Final_SiliconOil:
    def __init__(self):
        print("Please...")
        print("-------------------------")
        print("")

        # Robot Controller
        self.RC = RobotController_SDK()
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
        self.CT = Control_SiliconOil()

        # Final
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

            PWM = self.CT.Get_PWM([SystemPose[0]/1000, SystemPose[1]/1000, SystemPose[2]/1000],
                                  [TargetPose[0]/1000, TargetPose[1]/1000, TargetPose[2]/1000])

            if self.PWM_ONOFF:
                self.AD.Send_PWM(PWM)
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
            PWM = self.CT.Get_PWM([SystemPose[0]/1000, SystemPose[1]/1000, SystemPose[2]/1000],
                                  [TargetPose[0]/1000, TargetPose[1]/1000, TargetPose[2]/1000])
            z = SystemPose[2] - TargetPose[2]
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


    def TCP_Control(self, Mode):
        self.RC.SetRobotMode(1)
        time.sleep(1)
        with self.VS_lock:
            TargetPose = self.VS.Position

        if Mode == 1:
            self.RC.AddTCP('Offset', [-1000 * TargetPose[0], 1000 * TargetPose[1], 0,0,0,0])
            self.RC.SetTCP('Offset')
            print('TCP Offset Applied!')
            print('')
        else:
            self.RC.DeleteTCP('Offset')
            print('TCP Deleted!')
            print('')


    def Error(self, DesiredPose):
        with self.VS_lock, self.RC_lock:
            TargetPose = self.VS.Position
            TargetPose = [x * 1000 for x in TargetPose]

        Error = [DesiredPose[0]-TargetPose[0], DesiredPose[1]-TargetPose[1], DesiredPose[2]-TargetPose[2]]
        print(f"Target: {[round(x, 2) for x in TargetPose]} [mm]")
        print(f"Desired: {[round(x, 2) for x in DesiredPose]} [mm]")
        print(f"Error: {[round(x, 2) for x in Error]} [mm]")
        print("")

        return Error


    def MoveDesiredPose(self, DesiredPose):

        def on_press(key):
            try:
                if key.char == 'e':
                    self.Error(DesiredPose)

                elif key.char == 'f':
                    Error = self.Error(DesiredPose)
                    self.RC.MoveRel(Error[0], Error[1], Error[2],0)

                elif key.char == 'q':
                    print("End FeedBack Control...")
                    return False  # 리스너 종료

            except AttributeError:
                pass

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()


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
                       'Error': self.Error,
                       'MoveDesired': self.MoveDesiredPose,
                       'PWM': self.PWM_Switch,
                       'MPWM': self.AD.ManualPWM,
                       'TCP': self.TCP_Control,
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