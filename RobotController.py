#!/usr/bin/env python3

## <<Before Starting, Open Gazebo>>
## <<Copy and Paste the Following Commands into Terminal!!>>
## ---------------------------------------------
# cd ~/catkin_ws
# source devel/setup.bash
# roslaunch dsr_launcher single_robot_gazebo.launch model:=a0509 sim:=true
# roslaunch dsr_launcher single_robot_gazebo.launch model:=a0509_custom sim:=true
# roslaunch dsr_launcher SJ_Custom.launch model:=a0509_custom sim:=true
## ---------------------------------------------

# roslaunch dsr_launcher single_robot_gazebo.launch mode:=real host:=192.168.0.100 port:=12345


## <<Console Control>>
## <<Copy and Paste the Following Commands into "New" Terminal!!>>
## ---------------------------------------------
# python3 RobotController.py
## ---------------------------------------------

import rospy
import threading
import code
from dsr_msgs.srv import MoveLine, MoveJoint, MoveHome, MoveWait
from dsr_msgs.srv import GetCurrentPose


class RobotController:
    def __init__(self):
        self.Running = False
        self.lock = threading.Lock()
        self.SamplingTime = 100/1000

        self.Function_MoveWait = None
        self.Function_MoveHome = None
        self.Function_MoveLine = None
        self.Function_MoveJoint = None
        self.Function_GetPose = None

        self.EE_Position = None
        self.EE_Rotation = None

        self.Velocity = [20, 20]
        self.Acceleration = [20, 20]
        self.InitJoint = [7.506136417388916, -0.9212368726730347, 102.26158142089844, -4.5320696895583944e-15, 78.65965270996094, 7.506136417388916]



    def Ready(self):

        # Preprocessing
        rospy.init_node('Sejun_RobotController', anonymous=True)
        rospy.wait_for_service('/dsr01a0509/motion/move_home')
        rospy.wait_for_service('/dsr01a0509/motion/move_joint')

        # Functions
        self.Function_MoveHome = rospy.ServiceProxy('/dsr01a0509/motion/move_home', MoveHome)
        self.Function_MoveWait = rospy.ServiceProxy('/dsr01a0509/motion/move_wait', MoveWait)
        self.Function_MoveLine = rospy.ServiceProxy('/dsr01a0509/motion/move_line', MoveLine)
        self.Function_MoveJoint = rospy.ServiceProxy('/dsr01a0509/motion/move_joint', MoveJoint)
        self.Function_GetPose = rospy.ServiceProxy('/dsr01a0509/system/get_current_pose', GetCurrentPose)

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
        pose = [X, Y, Z, 0, 0, Phi]
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


    def Track_EE(self):
        print("Tracking...")
        while self.Running:
            Pose = self.Function_GetPose(1)
            with self.lock:
                self.EE_Position = Pose.pos[:3]
                self.EE_Rotation = Pose.pos[3:]
            print(f"Pose: {self.EE_Position}")
            rospy.sleep(1)
        print("Done!")
        print("")


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
                   }

    code.interact(banner=banner, local=locals_dict)

    RC.EndController()
