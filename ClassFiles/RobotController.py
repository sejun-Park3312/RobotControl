#!/usr/bin/env python3

## <<Before Starting, Open Gazebo>>
## <<Copy and Paste the Following Commands into Terminal!!>>
# cd ~/catkin_ws
# source devel/setup.bash
# roslaunch dsr_launcher single_robot_gazebo.launch model:=a0509 sim:=true
# roslaunch dsr_launcher SJ_Custom.launch model:=a0509_custom sim:=true
# roslaunch dsr_launcher SJ_Custom.launch model:=a0509_custom mode:=real host:=192.168.0.181 port:=12345
## ---------------------------------------------

## <<Connection LAN>>
# ip addr
# sudo ip addr add 192.168.0.100/24 dev enx00e04f82fbd0
# sudo ip link set enx00e04f82fbd0 up
# ping 192.168.0.181
## ---------------------------------------------
# sudo ip addr add 192.168.0.100/24 dev enxa0cec8ac637d
# sudo ip link set enxa0cec8ac637d up
# ping 192.168.0.181
## ---------------------------------------------

## <<Console Control>>
## <<Copy and Paste the Following Commands into "New" Terminal!!>>
## ---------------------------------------------
# python3 RobotController.py
## ---------------------------------------------

import rospy
import threading
import code
from ProjectPath import PROJECT_PATH
from dsr_msgs.srv import MoveLine, MoveJoint, MoveHome, MoveWait, Fkin, Ikin
from dsr_msgs.srv import GetCurrentPose, SetCurrentTcp, ConfigCreateTcp, GetCurrentTcp, ConfigDeleteTcp


class RobotController:
    def __init__(self):
        print("Robot Initializing...")
        self.Running = False
        self.lock = threading.Lock()
        self.SamplingTime = 100/1000
        self.modelName = "a0509_custom"
        self.TCP_Offset = [0, 0, 83.5]
        self.P_base2world = [350, 73.5, 200]

        self.Function_MoveWait = None
        self.Function_MoveHome = None
        self.Function_MoveLine = None
        self.Function_MoveJoint = None
        self.Function_GetPose = None

        self.EE_Position = None
        self.EE_Rotation = None

        self.Vel_Phi = 30
        self.Vel_X = 20
        self.Vel_Y = 20
        self.Vel_Z = 3
        self.Acceleration = [20, 20]
        self.InitJoint = [11.859809875488281, -0.16671763360500336, 102.27816009521484, -0.00044004168012179434, 77.88888549804688, 11.859648704528809]
        self.InitPose = [210.5/1000, 42/1000, 358.0/1000]

        self.Ready()


    def Ready(self):

        # Model Name
        modelName = self.modelName

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

        self.Running = True
        print("Robot Ready!")
        print("")
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
        if Z == 0:
            vel = [self.Vel_X, self.Vel_Phi]
        else:
            vel = [self.Vel_Z, self.Vel_Phi]
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
        if Z == 0:
            vel = [self.Vel_X, self.Vel_Phi]
        else:
            vel = [self.Vel_Z, self.Vel_Phi]
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

    def Wait(self, Time_sec):
        print("Sleeping...")
        rospy.sleep(Time_sec)
        print("Waked up!")
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



    def Track_EE(self):
        print("Tracking...")
        try:
            while self.Running and not rospy.is_shutdown():
                Pose = self.Function_GetPose(1)
                with self.lock:
                    self.EE_Position = Pose.pos[:3]
                    self.EE_Rotation = Pose.pos[3:]
                rospy.sleep(self.SamplingTime)
        except rospy.ROSInterruptException:
            print("ROS Interruped!")
        finally:
            print("End Robot Tracking!")



    def SetTCP(self):
        CreatTCP = rospy.ServiceProxy('/dsr01' + self.modelName + '/tcp/config_create_tcp', ConfigCreateTcp)
        Result1 = CreatTCP(name="SJ_TCP", pos=self.TCP_Offset)
        SetTCP = rospy.ServiceProxy('/dsr01' + self.modelName + '/tcp/set_current_tcp', SetCurrentTcp)
        Result2 = SetTCP(name="SJ_TCP")
        GetTCP = rospy.ServiceProxy('/dsr01' + self.modelName + '/tcp/get_current_tcp', GetCurrentTcp)
        Result3 = GetTCP()
        if Result1.success == True and Result2.success == True:
            print("TCP Setting Done!")
            print("Current TCP Name:" + Result3.info)
            print("")
        else:
            print("TCP Setting Failed!")
            print("")



    def DeletTCP(self):
        DeletTCP = rospy.ServiceProxy('/dsr01' + self.modelName + '/tcp/config_delete_tcp', ConfigDeleteTcp)
        Result = DeletTCP(name="SJ_TCP")
        if Result.success == True:
            print("TCP Deleting Done!")
            print("")
        else:
            print("TCP Deleting Failed!")
            print("")


    def Fkin(self, q):
        Function_Fkin = rospy.ServiceProxy('/dsr01' + self.modelName + '/motion/fkin', Fkin)
        Result = Function_Fkin(pos=q, ref=2)
        Pose = Result.conv_posx
        return Pose


    def EndController(self):
        self.Running = False



    def StartController(self):
        banner = "\n Waiting Your Order..."
        locals_dict = {"RC": self,
                       'MoveJoint': self.Move_Joint,
                       'MoveRel': self.Move_Rel,
                       'MoveAbs': self.Move_Abs,
                       'GetPose': self.Get_Pose,
                       'GetJoint': self.Get_Joint,
                       'HomePose': self.Move_Home,
                       'InitPose': self.Init_Pose}

        code.interact(banner=banner, local=locals_dict)

        self.EndController()



if __name__ == "__main__":
    RC = RobotController()

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
                   'Wait': RC.Wait
                   }

    code.interact(banner=banner, local=locals_dict)

    RC.EndController()
