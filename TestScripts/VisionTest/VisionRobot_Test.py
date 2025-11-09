import threading
import time
from pynput import keyboard
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.ManualController import ManualController
from ClassFiles.VideoWriter import VideoWriter
from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK

# Robot Controller
RC = RobotController_SDK()
RC.Vel_Line = [60, 60]
RC.Acc_Line = [60, 60]
RC.InitPose = [425, 112.5, 280 + 30]

# Vision
VS = Vision()
VS.GUI = True
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

# Recorder
VW = VideoWriter(VS)
VW_Thread = threading.Thread(target=VW.Capture, daemon=True)
VW_Thread.start()

def GetPosition():

    with RC.lock, VS.lock:
        TargetPose = VS.Position
        RobotPose = RC.EE_Position
    RobotOffset = [425, 112.5, 280]
    SystemPose = [RobotPose[0] - RobotOffset[0], RobotPose[1] - RobotOffset[1], RobotPose[2] - RobotOffset[2]]
    z = SystemPose[2] - TargetPose[2]*1000
    print(f"Vision : {[round(x*1000,2) for x in TargetPose]} [mm]")
    print(f"System : {[round(x, 2) for x in SystemPose]} [mm]")



# Handle
Handle = {'RC': RC,
          'VS':VS,
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
VW.SaveVideo('251109')