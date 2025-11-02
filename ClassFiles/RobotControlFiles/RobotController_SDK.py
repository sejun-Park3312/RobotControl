#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import threading, time
import sys
import code
import numpy as np
from ProjectPath import PROJECT_PATH
from scipy.spatial.transform import Rotation as R
sys.dont_write_bytecode = True
import DR_init
from dsr_msgs.msg import RobotState
ROBOT_ID = "dsr01"
ROBOT_MODEL = "a0509_SJ_Custom"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DSR_ROBOT import *


class RobotController_SDK:
    def __init__(self):

        rospy.init_node('Sejun_RobotController', anonymous=True)

        self.Servicer = rospy.wait_for_service('/dsr01' + ROBOT_MODEL + '/motion/move_joint')
        self.Subscriber = rospy.Subscriber('/dsr01' + ROBOT_MODEL + '/state', RobotState, self.CallBack)
        set_robot_mode = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL  + '/system/set_robot_mode', SetRobotMode)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        set_velx(20, 20)  # set global task speed: 30(mm/sec), 20(deg/sec)
        set_accx(20, 20)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

        self.InitPose = [425,112.5,200+83.5+90]
        self.Vel_Joint = 60
        self.Acc_Joint = 30
        self.Vel_Line = [5, 20]
        self.Acc_Line = [5, 20]

        self.lock = threading.Lock()
        self.Running = True
        self.EE_Position = None
        self.EE_Rotation = None
        self.JointAngle = None

    ## <Move Functions>
    # ========================================================================================================================
    def MoveZero(self):
        self.MoveJoint([0,0,0,0,0,0])

    def MoveHome(self):
        self.MoveJoint([0,0,90,0,90,0])

    def MoveInit(self):
        self.MoveAbs(self.InitPose[0], self.InitPose[1], self.InitPose[2], 0)

    def MoveJoint(self, q):
        j = posj(q[0], q[1], q[2], q[3], q[4], q[5])
        print("Moving...")
        bool = movej(j, self.Vel_Joint, self.Acc_Joint)
        if bool == -1:
            print("Failed..")
        else:
            print("Done!")
        print("")

    def MoveRel(self, x,y,z,phi):
        x = posx(x,y,z,0,0,phi)
        print("Moving...")
        bool = movel(x, self.Vel_Line, self.Acc_Line, None, None, None, DR_MV_MOD_REL)
        if bool == -1:
            print("Failed..")
        else:
            print("Done!")
        print("")

    def MoveAbs(self, x,y,z,phi):
        x = posx(x,y,z,0,180,phi)
        print("Moving...")
        bool = movel(x, self.Vel_Line, self.Acc_Line, None, None, None, DR_MV_MOD_ABS)
        if bool == -1:
            print("Failed..")
        else:
            print("Done!")
        print("")

    def MoveTrajectory(self, TrjList, Vel = 5, Acc = 3, Time = 0):
        rospy.sleep(2)
        with self.lock:
            EE_Position = self.EE_Position
            EE_Rotation = self.EE_Rotation

        PoseInit = [EE_Position[0], EE_Position[1], EE_Position[2], EE_Rotation[0], EE_Rotation[1], EE_Rotation[2]]
        posx_list = []
        for i in range(len(TrjList)):
            pos = [(PoseInit[0] + TrjList[i][0]), (PoseInit[1] + TrjList[i][1]), (PoseInit[2] + TrjList[i][2]),
                   (PoseInit[3]), (PoseInit[4]), (PoseInit[5] - TrjList[i][3])]
            posx_list.append(posx([pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]]))

        time.sleep(1)
        bool = movesx(posx_list, Vel, Acc, Time, None, DR_MV_MOD_ABS, DR_MVS_VEL_CONST)
        if bool == -1:
            print("Failed..")
        else:
            print("Done!")
        print("")

    def Wait(self, Time):
        rospy.sleep(Time)

    # ========================================================================================================================
    ## </Move Functions>

    ## <Pose Sensing>
    # ========================================================================================================================
    def CallBack(self, msg):
        with self.lock:
            Position = list(msg.current_posx)
            Joint = list(msg.current_posj)
        self.EE_Position = [Position[0], Position[1], Position[2]]
        self.EE_Rotation = [Position[3], Position[4], Position[5]]
        self.JointAngle = [Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5]]

    def GetPose(self):
        with self.lock:
            EE_Position = self.EE_Position
            EE_Rotation = self.EE_Rotation
        print(f"{[round(EE_Position[0],2), round(EE_Position[1],2), round(EE_Position[2],2)]} [mm] / {round(EE_Rotation[0]- EE_Rotation[2],2)} [degree]")

    def GetJoint(self):
        q = get_current_posj()
        print(f"{[round(x,2) for x in q]} [degree]")
    # ========================================================================================================================
    ## </Pose Sensing>

    ## <TCP Options>
    # ========================================================================================================================
    def AddTCP(self, Name, Pose):
        bool = add_tcp(Name, Pose)
        if bool == -1:
            print("Failed..")
        else:
            print("Done!")
        print("")

    def DeleteTCP(self, Name):
        bool = del_tcp(Name)
        if bool == -1:
            print("Failed..")
        else:
            print("Done!")
        print("")

    def SetTCP(self, Name):
        bool = set_tcp(Name)
        if bool == -1:
            print("Failed..")
        else:
            print("Done!")
        print("")

    def GetTCP(self):
        Name = get_tcp()
        print("Current TCP : " + Name)
        print("")
    # ========================================================================================================================
    ## </TCP Options>

    ## <Robot Mode>
    # ========================================================================================================================
    def SAFE_OFF(self):
        bool = set_robot_control(CONTROL_RESET_SAFET_OFF)
        bool = set_robot_control(CONTROL_RECOVERY_SAFE_OFF)
        if bool == -1:
            print("Failed..")
        else:
            print("Done!")
        print("")

    def SetRobotMode(self, Mode):
        # Mode_1: Manual, Mode_2: Autonomous
        if Mode == 1:
            set_robot_mode(ROBOT_MODE_MANUAL)
        else:
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # ========================================================================================================================
    ## </Robot Mode>




    def GetController(self):
        banner = "\n Waiting Your Order..."
        locals_dict = {"RC": self,
                       'MoveJoint': self.MoveJoint,
                       'MoveRel': self.MoveRel,
                       'MoveAbs': self.MoveAbs,
                       'GetPose': self.GetPose,
                       'GetJoint': self.GetJoint,
                       'HomePose': self.MoveHome,
                       'ZeroPose': self.MoveZero,
                       'InitPose': self.MoveInit,
                       'SafeOff': self.SAFE_OFF,
                       'Wait': self.Wait}

        code.interact(banner=banner, local=locals_dict)

if __name__ == "__main__":

    RC = RobotController_SDK()
    RC.GetController()