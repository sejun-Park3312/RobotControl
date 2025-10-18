import time
import numpy as np
from ProjectPath import PROJECT_PATH

from ClassFiles.MagnetFieldSensor import MagnetFieldSensor
# Magnetic Field Sensor
GDX = MagnetFieldSensor()

# <Preprocessing>
# ============================================================================================
from ClassFiles.GazeboSimulator import GazeboSimulator
from ClassFiles.RobotController import RobotController
from ClassFiles.RealTimeData_Recorder import RealTimeData_Recorder
from ClassFiles.Arduino import Arduino

# Gazebo Simulator
Gazebo = GazeboSimulator()
Gazebo.launcher_name = "SJ_Custom" # or single_robot_gazebo/SJ_Custom
Gazebo.launcher_model = "a0509_Sensor" # or a0509/a0509_Calibration/a0509_custom
# Gazebo.VirtualMode()

# RobotController Setting
RC = RobotController()
RC.launcher_name = Gazebo.launcher_name
RC.launcher_model = Gazebo.launcher_model
RC.Ready()
RC.Velocity = [20, 20]
RC.Acceleration = [20, 20]
RC.InitJoint = [3.6749961376190186, 11.92389965057373, 81.28993225097656, -3.941812080346716e-16, 86.78617095947266, 3.6749961376190186]
RC.InitPose = [450, 28.903, 241.1+80+80]
RC.Init_Pose()

# Arduino
AD = Arduino()
PWM = 100

# Real Time Data Recorder
RD = RealTimeData_Recorder()
RD.DefineData("SystemPosition", ['x', 'y', 'z'])
RD.DefineData("MagneticField", ['Bx', 'By', 'Bz'])

# ============================================================================================
# </Preprocessing>



# <Sampling Points>
# ============================================================================================
# 각 축의 범위
x_range = (-10, 10)
y_range = (-10, 10)
z_range = (-10, 10)

# 축별 샘플링 개수 (자유롭게 조절 가능)
nx = 3
ny = 3
nz = 4

# linspace로 균일 분포 점 생성
x_vals = np.linspace(x_range[0], x_range[1], nx)
y_vals = np.linspace(y_range[0], y_range[1], ny)
z_vals = np.linspace(z_range[0], z_range[1], nz)

# 3D 그리드 생성
X, Y, Z = np.meshgrid(x_vals, y_vals, z_vals, indexing='ij')

# 포인트 (N, 3)로 병합
SamplingPoints = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T

# Relative Motion 정의
rel_motion = np.zeros_like(SamplingPoints)
i = 0
prev_point = [0,0,0]
for point in SamplingPoints:
    rel_motion[i, :] = point - prev_point
    prev_point = point
    i += 1
# ============================================================================================
# </Sampling Points>



# <Main Loop>
# ============================================================================================
# Base(Robot) 2 World(Reference)
SystemPosition = np.zeros((nx*ny*nz, 3), dtype=np.float64)
MagneticField = np.zeros((nx*ny*nz, 3), dtype=np.float64)

# Start PWM
AD.Send_PWM([PWM, PWM, PWM])
i = 0
for move in rel_motion:
    RC.Move_Rel(move[0], move[1], move[2], 0)
    time.sleep(2)

    Pose = RC.Get_Pose()
    SystemPosition = [Pose[0], Pose[1], Pose[2]]

    RD.AppendData("SystemPosition", SystemPosition)

    time.sleep(0.5)
    print(f"Sensing({i+1}/{nx*ny*nz})...")
    MeasuredData = GDX.Measure()
    RD.AppendData("MagneticField", MeasuredData)


    print(f"Robot:{[round(x, 2) for x in SystemPosition]} [mm]")
    print(f"MagneticField:{[round(x, 2) for x in MeasuredData]} [mT]")
    print("")
    print("")

    i += 1

AD.Send_PWM([0, 0, 0])
RC.Init_Pose()
# ============================================================================================
# </Main Loop>



# <Save Data>
# ============================================================================================
RD.SaveData("SystemPosition", "SystemPosition")
RD.SaveData("MagneticField", "MagneticField")
# ============================================================================================
# </Save Data>