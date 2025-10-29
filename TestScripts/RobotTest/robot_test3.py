from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK



RC = RobotController_SDK()
RC.Vel_Line = [40,40]
RC.Acc_Line = [40,40]
RC.InitPose = [425, 112.5,200 + 83.5 + 90 + 30]

RC.GetController()


