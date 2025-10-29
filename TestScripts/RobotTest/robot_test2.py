from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK
from ClassFiles.RobotControlFiles.PathCreator import PathCreator

PC = PathCreator()
RelMotionList_1 = [[30,0,0],[0,-30,0],[-30,0,0]]
Radius = [10, 10]
Trajectory_1 = PC.MakeTrajectory(RelMotionList_1, Radius, True)

RelMotionList_2 = [[0,-60,0],[30,0,0],[0,60,0]]
Radius = [7.5, 7.5]
Trajectory_2 = PC.MakeTrajectory(RelMotionList_2, Radius, True)



RC = RobotController_SDK()
RC.Vel_Line = [12,20]
RC.Acc_Line = [10,20]
RC.Trajectory = [Trajectory_1, Trajectory_2]
RC.GetPose()

RC.InitPose = [425, 112.5, 367.5]
# Center = [425,112.5,200+83.5+90]
RC.MoveHome()
RC.MoveInit()
# RC.MoveAbs(Center[0],Center[1],Center[2],0)

RC.MoveRel(-65,-30,0,90)
RC.MoveRel(0,60,0,0)
RC.MoveRel(0,0,0, -90)
RC.MoveTrajectory(RC.Trajectory[0], 0, 0, 20)

RC.MoveRel(50,-30,0,90)
RC.MoveRel(0,60,0,0)
RC.MoveRel(0,0,0,26.57)
RC.MoveRel(30,-60,0,0)
RC.MoveRel(0,0,0,-26.57)
RC.MoveRel(0,60,0,0)
RC.Wait(1)

RC.MoveRel(20,0,0,0)
RC.MoveTrajectory(RC.Trajectory[1], 0,0, 20)


RC.MoveInit()

RC.GetController()

