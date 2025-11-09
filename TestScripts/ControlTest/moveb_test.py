from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK
import Trajectory_Infinite as Trj_IF

RC = RobotController_SDK()

b_list = Trj_IF.main()

RC.GetController()

RC.MoveHome()
RC.MoveInit()
RC.MoveRel(0,0,0,45)
RC.MoveBlend(b_list, 5,5)






