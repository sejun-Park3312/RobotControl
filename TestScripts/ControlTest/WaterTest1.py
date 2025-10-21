import threading
import time
from pynput import keyboard
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.Arduino import Arduino
from ClassFileSetting.Setting_RobotControl import Setting_RobotControl
from ClassFiles.ManualController import ManualController

# Robot Controller
Set_RC = Setting_RobotControl()
RC = Set_RC.WaterControl_251021()

# Vision
VS = Vision()
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

# Arduino
AD = Arduino()
AD.a = 10.7091
AD.b = -993.3455


def Please(self):

    print("Start!")
    while self.Running:
        self.StartTime = time.time()
        while self.Running:

            time.sleep(self.CT.SamplingTime)
            with self.VS_lock:
                self.CT.TargetPose = [self.VS.Position[0] * self.beta, self.VS.Position[1] * self.beta,
                                      self.VS.Position[2]]
                self.VisionData.AppendData(self.DataName_2,
                                           [self.VS.Position[0], self.VS.Position[1], self.VS.Position[2]])

            with self.RC_lock:
                world2SystemX = self.RC.EE_Position[0] - self.RC.P_base2world[0]
                world2SystemY = self.RC.EE_Position[1] - self.RC.P_base2world[1]
                world2SystemZ = self.RC.EE_Position[2] - self.RC.P_base2world[2] - self.RC.System_Offset[2]

                self.CT.SystemPose = [world2SystemX / 1000 * self.beta, world2SystemY / 1000 * self.beta,
                                      world2SystemZ / 1000]
                self.RobotData.AppendData(self.DataName_3,
                                          [world2SystemX, world2SystemY, world2SystemZ, self.RC.EE_Rotation[0],
                                           self.RC.EE_Rotation[1], self.RC.EE_Rotation[2]])

            PWM = self.CT.Get_PWM()

            if self.PWM_ONOFF:
                self.AD.Send_PWM(PWM)
            else:
                self.AD.Send_PWM([0, 0, 0])

            self.ControlData.AppendData(self.DataName_1,
                                        [(self.CT.Z_Reference - (self.CT.SystemPose[2] - self.CT.TargetPose[2])),
                                         self.CT.SystemPose[2], self.CT.TargetPose[2], PWM])



# Handle
Handle = {'RC': RC,
          'MoveRel': RC.Move_Rel,
          'MoveAbs': RC.Move_Abs,
          'InitPose': RC.Init_Pose,
          'GetPose': RC.Get_Pose,
          'GetJoint': RC.Get_Joint}

MC = ManualController()
MC.AddHandle(Handle)
MC.Start()