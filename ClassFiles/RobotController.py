#!/usr/bin/env python3

import rospy
import threading
import subprocess
import code
from ProjectPath import PROJECT_PATH
from dsr_msgs.srv import MoveLine, MoveJoint, MoveHome, MoveWait, Fkin, Ikin
from dsr_msgs.srv import GetCurrentPose, SetCurrentTcp, ConfigCreateTcp, GetCurrentTcp, ConfigDeleteTcp
from dsr_msgs.srv import SetRobotMode


class RobotController:
    def __init__(self):
        self.Running = False
        self.lock = threading.Lock()
        self.SamplingTime = 100/1000
        self.launcher_model = "a0509_custom"
        self.TCP_Offset = [0, 0, 83.5]
        self.P_base2world = [350, 73.5, 200]

        self.Function_MoveWait = None
        self.Function_MoveHome = None
        self.Function_MoveLine = None
        self.Function_MoveJoint = None
        self.Function_GetPose = None

        self.EE_Position = None
        self.EE_Rotation = None

        self.Velocity = [50, 20]
        self.Acceleration = [30, 20]
        self.InitJoint = [11.859779357910156, -0.6888203024864197, 99.6191177368164, -1.7431619358347097e-15, 81.0697021484375, 11.859779357910159]
        self.InitPose = [350/1000, 73.5/1000, 383.5/1000, 0]


    def Ready(self):
        # Model Name
        modelName = self.launcher_model

        # Preprocessing
        rospy.init_node('Sejun_RobotController', anonymous=True)
        rospy.wait_for_service('/dsr01' + modelName + '/motion/move_home')
        rospy.wait_for_service('/dsr01' + modelName + '/motion/move_joint')

        # Functions
        self.Function_MoveHome = rospy.ServiceProxy('/dsr01' + modelName + '/motion/move_home', MoveHome)
        self.Function_MoveWait = rospy.ServiceProxy('/dsr01' + modelName + '/motion/move_wait', MoveWait)
        self.Function_MoveLine = rospy.ServiceProxy('/dsr01' + modelName + '/motion/move_line', MoveLine)
        self.Function_MoveJoint = rospy.ServiceProxy('/dsr01' + modelName + '/motion/move_joint', MoveJoint)
        self.Function_GetPose = rospy.ServiceProxy('/dsr01' + modelName + '/system/get_current_pose', GetCurrentPose)
        self.Function_CreateTCP = rospy.ServiceProxy('/dsr01' + modelName + '/tcp/config_create_tcp', ConfigCreateTcp)
        self.Function_SetTCP = rospy.ServiceProxy('/dsr01' + modelName + '/tcp/set_current_tcp', SetCurrentTcp)
        self.Function_GetTCP = rospy.ServiceProxy('/dsr01' + modelName + '/tcp/get_current_tcp', GetCurrentTcp)
        self.Function_DeletTCP = rospy.ServiceProxy('/dsr01' + modelName + '/tcp/config_delete_tcp', ConfigDeleteTcp)
        self.Function_SetRobotMode = rospy.ServiceProxy('/dsr01' + modelName + '/system/set_robot_mode', SetRobotMode)

        self.Running = True
        print("Ready!")
        print("")


    def Move_Home(self):

        results = self.Function_MoveHome()
        if results.success == True:
            print("Homing...")
            self.Function_MoveWait()
            print("Done!")
            print("")
        else:
            print("Failed...")
            print("")



    def Move_Abs(self, X, Y, Z, Phi):
        pose = [X, Y, Z, 0, 180, Phi]
        vel = self.Velocity
        acc = self.Acceleration
        time = 0
        radius = 0
        ref = 0
        mode = 0
        blendType = 0
        syncType = 0

        if self.Running:
            print("Moving...")
            results = self.Function_MoveLine(pose, vel, acc, time, radius, ref, mode, blendType, syncType)
            if results.success == True:

                self.Function_MoveWait()
                print("Done!")
                print("")
            else:
                print("Failed...")
                print("")



    def Move_Rel(self, X, Y, Z, Phi):
        pose = [X, Y, Z, 0, 0, Phi]
        vel = self.Velocity
        acc = self.Acceleration
        time = 0
        radius = 0
        ref = 0
        mode = 1
        blendType = 0
        syncType = 0

        if self.Running:
            print("Moving...")
            results = self.Function_MoveLine(pose, vel, acc, time, radius, ref, mode, blendType, syncType)
            if results.success == True:

                self.Function_MoveWait()
                print("Done!")
                print("")
            else:
                print("Failed...")
                print("")



    def Move_Joint(self, q):
        vel = 20
        acc = 20
        time = 0
        radius = 0
        mode = 0
        blendType = 0
        syncType = 0

        if self.Running:
            print("Moving...")
            results = self.Function_MoveJoint(q, vel, acc, time, radius, mode, blendType, syncType)
            if results.success == True:

                self.Function_MoveWait()
                print("Done!")
                print("")
            else:
                print("Failed...")
                print("")



    def Init_Pose(self):
        if self.Running:
            self.Move_Joint(self.InitJoint)



    def Get_Joint(self):
        with self.lock:
            Pose = self.Function_GetPose(0)
        print("")
        return Pose.pos



    def Get_Pose(self):
        with self.lock:
            Pose = self.Function_GetPose(1)
        print("")
        return Pose.pos

    def Wait(self, Time_sec):
        print("Sleeping...")
        rospy.sleep(Time_sec)
        print("Waked up!")
        print("")

    def Track_EE(self):
        print("Tracking...")
        while self.Running:
            Pose = self.Function_GetPose(1)
            with self.lock:
                self.EE_Position = Pose.pos[:3]
                self.EE_Rotation = Pose.pos[3:]

        print("Done!")
        print("")


    # Manual Mode에서 가능
    def SetTCP(self, TCPName = "SJ_TCP", TCP_OFFSET = None):
        if TCP_OFFSET == None:
            TCP_OFFSET = self.TCP_Offset

        Result1 = self.Function_CreateTCP(TCPName, TCP_OFFSET)
        Result2 = self.Function_SetTCP(TCPName)
        Result3 = self.Function_GetTCP()

        if Result1.success == True and Result2.success == True:
            print("TCP Setting Done!")
            print("Current TCP Name: " + Result3.info)
            print(f"Current TCP Pose: {TCP_OFFSET}")
            print("")
        else:
            print("TCP Setting Failed.. Check Robot Mode")
            print("")



    def DeletTCP(self):
        DeletTCP = rospy.ServiceProxy('/dsr01' + self.launcher_model + '/tcp/config_delete_tcp', ConfigDeleteTcp)
        Result = DeletTCP(name="SJ_TCP")
        if Result.success == True:
            print("TCP Deleting Done!")
            print("")
        else:
            print("TCP Setting Failed.. Check Robot Mode")
            print("")


    def SetRobotMode(self, ManualMode = False):
        if ManualMode == True:
            Result = self.Function_SetRobotMode(1)
        else:
            Result = self.Function_SetRobotMode(0)

        if Result.success == True:
            if ManualMode == False:
                print("Auto Mode On!")
                print("")
            else:
                print("Manual Mode On!")
                print("")



    def GetTCP(self):

        Result = self.Function_GetTCP()

        if Result.success == True:
            print("TCP Name: " + Result.info)
            print("")
        else:
            print("TCP Setting Failed.. ")
            print("")




    def DeletTCP(self, TCPName = None):

        if TCPName == None:
            TCPName = self.Function_GetTCP()

        Result = self.Function_DeletTCP(TCPName.info)
        if Result.success == True:
            print("TCP Named " + TCPName.info + " Deleted!")
            print("")
        else:
            print("TCP Setting Failed.. Check Robot Mode")
            print("")




    def Controller(self):
        banner = "\n Waiting Your Order..."
        locals_dict = {"RC": self,
                       'MoveJoint': self.Move_Joint,
                       'MoveRel': self.Move_Rel,
                       'MoveAbs': self.Move_Abs,
                       'GetPose': self.Get_Pose,
                       'GetJoint': self.Get_Joint,
                       'HomePose': self.Move_Home,
                       'InitPose': self.Init_Pose,
                       'SetTcp': self.SetTCP,
                       "GetTcp": self.GetTCP,
                       "DeletTcp": self.DeletTCP,
                       "SetRobotMode": self.SetRobotMode,
                       'Wait': self.Wait}

        code.interact(banner=banner, local=locals_dict)

        self.EndController()


    def EndController(self):
        self.Running = False


if __name__ == "__main__":
    RC = RobotController()
    RC.Ready()

    # EE_Tracker = threading.Thread(target=RC.Track_EE, daemon=True)
    # EE_Tracker.start()

    # RC.Move_Home()

    banner = "\n Waiting Your Order..."
    locals_dict = {"RC":RC,
                   'MoveJoint':RC.Move_Joint,
                   'MoveRel':RC.Move_Rel,
                   'MoveAbs':RC.Move_Abs,
                   'GetPose':RC.Get_Pose,
                   'GetJoint':RC.Get_Joint,
                   'HomePose':RC.Move_Home,
                   'InitPose':RC.Init_Pose,
                   'SetTcp':RC.SetTCP,
                   "GetTcp": RC.GetTCP,
                   "DeletTcp": RC.DeletTCP,
                   "SetRobotMode": RC.SetRobotMode,
                   'Wait': RC.Wait}

    code.interact(banner=banner, local=locals_dict)

    RC.EndController()
