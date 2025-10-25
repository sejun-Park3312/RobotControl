from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK
from ClassFiles.RobotControlFiles.PathCreator import PathCreator

PC = PathCreator()
RelMotionList_1 = [[25,25,0],[0,17.7,0],[-25,0,0],[0,-12.5,0]]
Radius = [12.4, 12.4,12.4]
Trajectory_1 = PC.MakeTrajectory(RelMotionList_1, Radius, True)

RelMotionList_2 = [[0,12.5,0],[-25,0,0],[0,-17.7,0],[25,-25,0]]
Trajectory_2 = PC.MakeTrajectory(RelMotionList_2, Radius, True)

RC = RobotController_SDK()
RC.MoveHome()
RC.MoveInit()
RC.MoveAbs(450,25,200+83.5+90,0)
RC.MoveRel(0,-21.34,0,0)
RC.MoveRel(0,0,0,45)
RC.MoveTrajectory(Trajectory_1)
RC.MoveRel(0,0,0,-180)
RC.MoveTrajectory(Trajectory_2)

RC.GetController()


