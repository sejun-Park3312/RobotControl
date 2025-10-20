import threading
import time
from pynput import keyboard
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.Arduino import Arduino
from ClassFiles.RobotController import RobotController
from ClassFiles.ManualController import ManualController

# Robot Controller
RC = RobotController()
RC.launcher_name = "SJ_Custom"
RC.launcher_model = "a0509_custom2"
RC.Ready()
RC.Velocity = [20, 20]
RC.Acceleration = [20, 20]
RC.InitJoint = [3.179781198501587, 12.002516746520996, 82.9296875, -1.4726332437930978e-06, 85.06681060791016, 3.1798245906829834]
RC.InitPose = [450, 25, 370 + 30]
RC.Init_Pose()
RC.Move_Rel(0,0,-30,0)

# Vision
VS = Vision()
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

# Arduino
AD = Arduino()
pwm = 160
AD.ManualPWM_Value = [pwm, pwm, pwm]

def GetPosition():
    VisionPose = VS.Position
    RobotPose = RC.Get_Pose()
    RobotOffset = [450, 25, 270]
    RobotPose_ = [RobotPose[0] - RobotOffset[0], RobotPose[1] - RobotOffset[1], RobotPose[2] - RobotOffset[2]]
    print(f"Vision : {[round(x*1000,2) for x in VisionPose]} [mm]")
    print(f"Robot : {[round(x, 2) for x in RobotPose_]} [mm]")



# Handle
Handle = {'RC': RC,
          'VS':VS,
          'AD': AD,
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