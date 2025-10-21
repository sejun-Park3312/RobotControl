import threading
import time
from pynput import keyboard
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.Arduino import Arduino
from ClassFiles.RobotController import RobotController
from ClassFiles.ManualController import ManualController
from ClassFiles.Control_Water import Control_Water

# Robot Controller
RC = RobotController()
RC.launcher_name = "SJ_Custom"
RC.launcher_model = "a0509_custom2"
RC.Velocity = [20, 20]
RC.Acceleration = [20, 20]
RC.InitPose = [450, 25, 370 + 30, 0]
RC.Ready()

# Vision
VS = Vision()
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

# Arduino
AD = Arduino()
pwm = 160
AD.ManualPWM_Value = [pwm, pwm, pwm]

# PID Control
CT = Control_Water()


def GetPosition():
    VisionPose = VS.Position
    RobotPose = RC.Get_Pose()
    RobotOffset = [450, 25, 270]
    RobotPose_ = [RobotPose[0] - RobotOffset[0], RobotPose[1] - RobotOffset[1], RobotPose[2] - RobotOffset[2]]
    z = RobotPose_[2] - VisionPose[2]*1000
    print(f"Vision : {[round(x*1000,2) for x in VisionPose]} [mm]")
    print(f"Robot : {[round(x, 2) for x in RobotPose_]} [mm]")
    print(f"Z : {round(z,2)} [mm]")

def CalPWM(z):
    CT.Z_Reference = z
    PWM = CT.Get_PWM(z,0)
    return PWM

# Handle
Handle = {'RC': RC,
          'VS':VS,
          'AD': AD,
          'CT': CT,
          'Cal': CalPWM,
          'PWM': AD.ManualPWM,
          'Pose':GetPosition,
          'MoveRel': RC.Move_Rel,
          'MoveAbs': RC.Move_Abs,
          'InitPose': RC.Init_Pose,
          'GetPose': RC.Get_Pose,
          'GetJoint': RC.Get_Joint}

MC = ManualController()
MC.AddHandle(Handle)
MC.Start()


# Close Vision
VS.Running = False
# Disconnect Arduino
AD.Disconnect()