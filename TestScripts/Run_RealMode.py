import sys
import os
import code
from ProjectPath import PROJECT_PATH
from ClassFiles.GazeboSimulator import GazeboSimulator
from ClassFiles.RobotController import RobotController

## <<you should connect Robot IP first>>
## <<Connect LAN>>
# ip addr show (보통 enp(유선)/enx(어댑터)로 시작한다함)

## <<어댑터 사용했을 경우>>
# sudo ip addr flush dev enx00e04f82fbd0
# sudo ip addr add 192.168.0.100/24 dev enx00e04f82fbd0
# sudo ip link set enx00e04f82fbd0 up

## <<그냥 유선연결>>
# sudo ip addr flush dev enp68s0
# sudo ip addr add 192.168.0.100/24 dev enp68s0
# sudo ip link set enp68s0 up

# ping 192.168.0.181

Gazebo = GazeboSimulator()
Gazebo.launcher_name = "SJ_Custom" # or single_robot_gazebo/SJ_Custom
Gazebo.launcher_model = "a0509_custom" # or a0509/a0509_Calibration/a0509_custom
Gazebo.RealMode()

RC = RobotController()
RC.launcher_name = Gazebo.launcher_name
RC.launcher_model = Gazebo.launcher_model
RC.Ready()

RC.Controller()

RC.EndController()