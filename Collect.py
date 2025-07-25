import numpy as np
import threading
import time
from RobotController import RobotController
from RealTimeData_Recorder import RealTimeData_Recorder
from Vision import Vision

# 각 축의 범위
x_range = (-50, 50)
y_range = (-35, 35)
z_range = (-15, 20)

# 축별 샘플링 개수 (자유롭게 조절 가능)
nx = 2
ny = 2
nz = 2

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
lock = VS.lock
VS_Thread = threading.Thread(target=VS.Tracking, daemon=True)
VS_Thread.start()

RD = RealTimeData_Recorder()
RD.DefineData("Robot_XYZ", {'x', 'y', 'z'})
RD.DefineData("Vision_XYZ", {'x', 'y', 'z'})

RC = RobotController()
RC.Ready()
RC.Init_Pose()

# Base(Robot) 2 World(Reference)
P_Offset = [350/1000, 73.75/1000, 275/1000]
Robot_XYZ = np.zeros((nx*ny*nz, 3), dtype=np.float64)
Vision_XYZ = np.zeros((nx*ny*nz, 3), dtype=np.float64)

i = 0
for move in rel_motion:
    RC.Move_Rel(move[0], move[1], move[2], 0)
    Pose = RC.Get_Pose()
    Robot_XYZ[i] = [Pose[0]/1000 - P_Offset[0], Pose[1]/1000 - P_Offset[1], Pose[2]/1000 - P_Offset[2]]

    RD.AppendData("Robot_XYZ", Robot_XYZ[i])
    time.sleep(VS.SamplingTime)
    with lock:
        Vision_XYZ[i] = [VS.Position[0], VS.Position[1], VS.Position[2]]
    RD.AppendData("Vision_XYZ", Vision_XYZ[i])

    print(f"Robot:{Robot_XYZ[i]}")
    print(f"Vision:{Vision_XYZ[i]}")
    print("")
    print("")

    i += 1

RC.Init_Pose()

RD.SaveData("Robot_XYZ", "Robot_XYZ")
RD.SaveData("Vision_XYZ", "Vision_XYZ")
# np.save("Results/Robot_XYZ", Robot_XYZ)
# np.save("Results/Vision_XYZ", Vision_XYZ)
