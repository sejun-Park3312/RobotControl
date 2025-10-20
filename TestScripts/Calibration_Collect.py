import numpy as np
import threading
import time
from ProjectPath import PROJECT_PATH
from ClassFiles.RobotController import RobotController
from ClassFiles.RealTimeData_Recorder import RealTimeData_Recorder
from ClassFiles.Vision import Vision

# 각 축의 범위
x_range = (-35, 35)
y_range = (-22, 22)
z_range = (-7, 10)

# 축별 샘플링 개수 (자유롭게 조절 가능)
nx = 10
ny = 7
nz = 8

# linspace로 균일 분포 점 생성
x_vals = np.linspace(x_range[0], x_range[1], nx)
y_vals = np.linspace(y_range[0], y_range[1], ny)
z_vals = np.linspace(z_range[0], z_range[1], nz)

# 3D 그리드 생성
X, Y, Z = np.meshgrid(x_vals, y_vals, z_vals, indexing='ij')

# 포인트 (N, 3)로 병합
SamplingPoints = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T

rel_motion = np.zeros_like(SamplingPoints)
i = 0
prev_point = [0,0,0]
for point in SamplingPoints:
    rel_motion[i, :] = point - prev_point
    prev_point = point
    i += 1

VS = Vision()
VS.Calibration_ONOFF = False
lock = VS.lock
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

RD = RealTimeData_Recorder()
RD.DefineData("Robot_XYZ", ['x', 'y', 'z'])
RD.DefineData("Vision_XYZ", ['x', 'y', 'z'])

RC = RobotController()
RC.launcher_name = "SJ_Custom"
RC.launcher_model = "a0509_Calibration2" # or a0509/a0509_Calibration/a0509_Calibration2/a0509_custom/a0509_custom
RC.Ready()
RC.Velocity = [30, 30]
RC.Acceleration = [30, 30]
RC.InitJoint = [3.179781198501587, 15.046290397644043, 95.11274719238281, -1.387559109389258e-06, 69.84093475341797, 3.1798245906829834]
RC.InitPose = [450, 25, 270 + 30]
RC.Init_Pose()
RC.Move_Rel(0,0,-30,0)

# Base(Robot) 2 World(Reference)
P_Offset = [450/1000, 25/1000, 270/1000]
Robot_XYZ = np.zeros((nx*ny*nz, 3), dtype=np.float64)
Vision_XYZ = np.zeros((nx*ny*nz, 3), dtype=np.float64)

i = 0
for move in rel_motion:
    RC.Move_Rel(move[0], move[1], move[2], 0)
    time.sleep(0.5)

    Pose = RC.Get_Pose()
    Robot_XYZ[i] = [Pose[0]/1000 - P_Offset[0], Pose[1]/1000 - P_Offset[1], Pose[2]/1000 - P_Offset[2]]

    RD.AppendData("Robot_XYZ", Robot_XYZ[i])
    time.sleep(0.5)
    with lock:
        Vision_XYZ[i] = [VS.Position[0], VS.Position[1], VS.Position[2]]
    RD.AppendData("Vision_XYZ", Vision_XYZ[i])

    print(f"Sensing({i+1}/{nx*ny*nz})...")
    print(f"Robot:{np.round(Robot_XYZ[i]*1000,2)} [mm]")
    print(f"Vision:{np.round(Vision_XYZ[i]*1000,2)} [mm]")
    print("")
    print("")

    i += 1

RC.Init_Pose()

RD.SaveData("Robot_XYZ", "Robot_XYZ")
RD.SaveData("Vision_XYZ", "Vision_XYZ")
np.save('../Data/Calibration_Data/Robot_XYZ.npy', Robot_XYZ)
np.save('../Data/Calibration_Data/Vision_XYZ.npy', Vision_XYZ)

VS.Running = False