import sys
import os
import code
from ProjectPath import PROJECT_PATH
from ClassFiles.GazeboSimulator import GazeboSimulator
from ClassFiles.RobotController import RobotController

Gazebo = GazeboSimulator()
Gazebo.launcher_name = "SJ_Custom" # or single_robot_gazebo/SJ_Custom
Gazebo.launcher_model = "a0509_custom" # or a0509/a0509_Calibration/a0509_custom
Gazebo.VirtualMode()

RC = RobotController()
RC.launcher_name = Gazebo.launcher_name
RC.launcher_model = Gazebo.launcher_model
RC.Ready()

RC.Controller()