from ProjectPath import PROJECT_PATH
from ClassFiles.RobotController import RobotController


class Setting_RobotControl():
    def __init__(self):
        print("Setting RobotControl")


    def WaterControl_251021(self):

        # Robot Controller
        RC = RobotController()
        RC.launcher_name = "SJ_Custom"
        RC.launcher_model = "a0509_custom2"
        RC.InitPose = [450, 25, 400, 0]
        RC.Velocity = [20, 20]
        RC.Acceleration = [20, 20]
        RC.Ready()

        return RC

