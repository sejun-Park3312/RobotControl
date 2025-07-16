#!/usr/bin/env python3

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

        self.Velocity = [30, 30]
        self.Acceleration = [30, 30]


    def Ready(self):
        # Preprocessing
        rospy.init_node('Sejun_Robot_Controller', anonymous=True)
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


    def Move_Home(self):

        results = self.Function_MoveHome()
        if results.success == True:
            print("Homing...")
            self.Function_MoveWait()
            print("Homing Complete!")
        else:
            print("Failed...")


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
                print("Moving Complete!")
            else:
                print("Failed...")


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
                print("Moving Complete!")
            else:
                print("Failed...")


    def Move_Joint(self, q):
        vel = 40
        acc = 40
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
                print("Moving Complete!")
            else:
                print("Failed...")


    def Get_Pose(self):
        with self.lock:
            Pose = self.Function_GetPose(1)
        print(f"Pose: {Pose.pos}")
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
        print("Tracking Done!")


    def EndController(self):
        self.Running = False



if __name__ == "__main__":
    RC = RobotController()
    RC.Ready()

    # EE_Tracker = threading.Thread(target=RC.Track_EE, daemon=True)
    # EE_Tracker.start()

    RC.Move_Home()

    banner = "\n Waiting Your Order..."
    locals_dict = {"RC":RC,
                   'moveJnt':RC.Move_Joint,
                   'moveRel':RC.Move_Rel,
                   'moveAbs':RC.Move_Abs,
                   'getPos':RC.Get_Pose,
                   'homePos':RC.Move_Home
                   }

    code.interact(banner=banner, local=locals_dict)

    RC.EndController()
