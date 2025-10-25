import threading
import time
from pynput import keyboard
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.Arduino import Arduino
from ClassFiles.RobotController import RobotController
from ClassFiles.ManualController import ManualController
from ClassFiles.Control_Water import Control_Water
from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK

# Robot Controller
RC = RobotController_SDK()
RC.Velocity = [20, 20]
RC.Acceleration = [20, 20]
RC.InitPose = [450, 25, 370 + 30, 0]

# Vision
VS = Vision()
VS.GUI = True
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

# Arduino
AD = Arduino()
pwm = 160
AD.ManualPWM_Value = [pwm, pwm, pwm]

# PID Control
CT = Control_Water()


def GetPosition():
    TargetPose = VS.Position
    RobotPose = RC.Get_Pose()
    RobotOffset = [450, 25, 270]
    SystemPose = [RobotPose[0] - RobotOffset[0], RobotPose[1] - RobotOffset[1], RobotPose[2] - RobotOffset[2]]
    z = SystemPose[2] - TargetPose[2]*1000
    print(f"Vision : {[round(x*1000,2) for x in TargetPose]} [mm]")
    print(f"System : {[round(x, 2) for x in SystemPose]} [mm]")
    print(f"Z, Error : {[round(z,2), round(CT.Z_Reference - z,2)]} [mm]")

def CalPWM():
    TargetPose = VS.Position
    RobotPose = RC.Get_Pose()
    RobotOffset = [450, 25, 270]
    SystemPose = [RobotPose[0] - RobotOffset[0], RobotPose[1] - RobotOffset[1], RobotPose[2] - RobotOffset[2]]
    z = SystemPose[2] - TargetPose[2]*1000
    PWM_pid = CT.Get_PWM(SystemPose[2],TargetPose[2]*1000)
    PWM_eql = round(CT.a * (z - CT.Z_Reference) + CT.PWM_Reference + CT.beta)
    print(f"PWM equal, pid: {[PWM_eql, PWM_pid]}")


# Handle
Handle = {'RC': RC,
          'VS':VS,
          'AD': AD,
          'CT': CT,
          'Cal': CalPWM,
          'PWM': AD.ManualPWM,
          'Pose':GetPosition,
          'MoveRel': RC.MoveRel,
          'MoveAbs': RC.MoveAbs,
          'InitPose': RC.MoveInit,
          'GetPose': RC.GetPose,
          'GetJoint': RC.GetJoint}

MC = ManualController()
MC.AddHandle(Handle)
MC.Start()


# Close Vision
VS.Running = False
# Disconnect Arduino
AD.Disconnect()