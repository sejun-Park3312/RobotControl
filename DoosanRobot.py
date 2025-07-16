#!/usr/bin/env python3

import time
import threading
import subprocess
import rospy
from dsr_msgs.srv import MoveLine, MoveHome, MoveWait
from dsr_msgs.srv import GetCurrentPose

class DoosanRobot:
    def __init__(self):
        self.Running = False
        self.lock = threading.Lock()
        self.SamplingTime = 100/1000

        self.Function_MoveWait = None
        self.Function_MoveHome = None
        self.Function_MoveLine = None
        self.Function_GetPose = None

        self.EE_Position = None
        self.EE_Rotation = None

        self.Velocity = [30, 30]
        self.Acceleration = [30, 30]


    def Ready(self):
        # Start Gazebo
        subprocess.run('cd ~/catkin_ws', shell=True)
        subprocess.run('source devel/setup.bash', shell=True)
        subprocess.run('roslaunch dsr_launcher single_robot_gazebo.launch model:=a0509', shell=True)

        # Preprocessing
        rospy.init_node('simple_move_gazebo')
        rospy.wait_for_service('/dsr01a0509/motion/move_home')
        rospy.wait_for_service('/dsr01a0509/motion/move_joint')

        # Functions
        self.Function_MoveHome = rospy.ServiceProxy('/dsr01a0509/motion/move_home', MoveHome)
        self.Function_MoveWait = rospy.ServiceProxy('/dsr01a0509/motion/move_wait', MoveWait)
        self.Function_MoveLine = rospy.ServiceProxy('/dsr01a0509/motion/move_joint', MoveLine)
        self.Function_GetPose = rospy.ServiceProxy('/dsr01a0509/system/get_current_pose', GetCurrentPose)


    def Move_Home(self):
        self.Function_MoveHome()
        self.Function_MoveWait()
        print("Homing Complete!")


    def Move_Abs(self, X, Y, Z, Phi):
        pose = [X, Y, Z, 0, 0, Phi]
        vel = self.Velocity
        acc = self.Acceleration
        time = 0
        radius = 0
        mode = 0
        blendType = 0
        syncType = 0

        self.Function_MoveLine(pose, vel, acc, time, radius, mode, blendType, syncType)
        self.Function_MoveWait()


    def Move_Rel(self, X, Y, Z, Phi):
        pose = [X, Y, Z, 0, 0, Phi]
        vel = self.Velocity
        acc = self.Acceleration
        time = 0
        radius = 0
        mode = 1
        blendType = 0
        syncType = 0

        self.Function_MoveLine(pose, vel, acc, time, radius, mode, blendType, syncType)
        self.Function_MoveWait()


    def Get_Pose(self):
        Pose = self.Function_GetPose(1)

        self.EE_Position = Pose[:3]
        self.EE_Rotation = Pose[3:]
