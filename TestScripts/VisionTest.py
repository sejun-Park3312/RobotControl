import threading
import time
from ProjectPath import PROJECT_PATH
from ClassFiles.Vision import Vision
from ClassFiles.ManualController import ManualController
from ClassFiles.RobotController import RobotController


# Robot Controller
RC = RobotController()
RC.launcher_name = "SJ_Custom"
RC.launcher_model = "a0509_Calibration2"
RC.Ready()
RC.Velocity = [20, 20]
RC.Acceleration = [20, 20]
RC.InitJoint = [3.179781198501587, 15.046290397644043, 95.11274719238281, -1.387559109389258e-06, 69.84093475341797, 3.1798245906829834]
RC.InitPose = [450, 25, 270 + 30]
RC.Init_Pose()
RC.Move_Rel(0,0,-30,0)

# 먼저 연결된 웹캠의 번호 확인 후 Vision에 주기
# v4l2-ctl --list-devices
# sudo apt install v4l-utils (위에 설치 필요 시)
VS = Vision()
VS.Calibration_ONOFF = False
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

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