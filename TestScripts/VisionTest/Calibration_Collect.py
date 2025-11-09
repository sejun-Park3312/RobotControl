import numpy as np
import threading
import time
from ProjectPath import PROJECT_PATH
from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK
from ClassFiles.RealTimeData_Recorder import RealTimeData_Recorder
from ClassFiles.Vision import Vision

# 각 축의 범위
x_range = (-80, 80)
y_range = (-80, 80)
z_range = (-15, 15)

# 축별 샘플링 개수 (자유롭게 조절 가능)
nx = 3
ny = 3
nz = 3

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

# Robot Controller
RC = RobotController_SDK()
RC.InitPose = [425, 112.5, 280 + 50]
RC.Vel_Line = [60, 60]
RC.Acc_Line = [60, 60]
RC.MoveInit()
RC.MoveRel(0,0,-50,0)


# Base(Robot) 2 World(Reference)
P_Offset = [425/1000, 112.5/1000, 280/1000]
Robot_XYZ = np.zeros((nx*ny*nz, 3), dtype=np.float64)
Vision_XYZ = np.zeros((nx*ny*nz, 3), dtype=np.float64)

i = 0
for move in rel_motion:
    RC.MoveRel(move[0], move[1], move[2], 0)
    time.sleep(0.5)

    with RC.lock:
        Pose = RC.EE_Position
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

RC.MoveInit()

RD.SaveData("Robot_XYZ", "Robot_XYZ")
RD.SaveData("Vision_XYZ", "Vision_XYZ")
np.save('../../Data/Calibration_Data/Robot_XYZ.npy', Robot_XYZ)
np.save('../../Data/Calibration_Data/Vision_XYZ.npy', Vision_XYZ)

VS.Running = False