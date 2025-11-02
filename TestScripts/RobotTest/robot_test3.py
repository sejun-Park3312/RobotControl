from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK
from ClassFiles.RobotControlFiles.PathCreator import PathCreator
import numpy as np
RC = RobotController_SDK()
#
FlatPlane = [[45,45,0],[45,-45,0],[-45,-45,0],  [-90,90,0], [-45,-45,0],[45,-45,0],[45,45,0]]
# PlaneAngle = 0/180*np.pi
# RelMotionList = []
#
# for pos in FlatPlane:
#     new_pos = [pos[0], pos[1]*np.cos(PlaneAngle), pos[2] * np.abs(pos[1]) * np.sin(PlaneAngle)]
#     RelMotionList.append(new_pos)

PC = PathCreator()
Radius = [63/2, 63/2, 63/2, 63/2, 63/2, 63/2]
Trajectory = PC.MakeTrajectory(FlatPlane, Radius, True, 20)




RC.Vel_Line = [12,20]
RC.Acc_Line = [10,20]
RC.Trajectory = [Trajectory]



RC.InitPose = [425, 112.5, 367.5]
Center = [425,112.5,205 + 83.5 + 90]
# RC.MoveHome()

RC.GetController()

RC.MoveAbs(Center[0],Center[1],Center[2],0)



RC.MoveRel(0,0,0,45)
RC.Wait(5)
RC.MoveTrajectory(RC.Trajectory[0], 8,3, 0)
RC.Wait(1)

RC.MoveRel(0,0,0,-45)

# RC.GetController()
