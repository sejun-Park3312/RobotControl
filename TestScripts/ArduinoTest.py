from ProjectPath import PROJECT_PATH
from ClassFiles.Arduino import Arduino
from ClassFiles.ManualController import ManualController
from ClassFiles.RobotController import RobotController

MC = ManualController()
AD = Arduino()
alpha = 0
pwm = 30
AD.PWM_Value = 100


RC = RobotController()
RC.launcher_name = "SJ_Custom"
RC.launcher_model = "a0509_custom"
RC.Ready()
RC.Velocity = [20, 20]
RC.Acceleration = [20, 20]
RC.InitJoint = [3.814095973968506, 12.57492446899414, 86.94134521484375, -1.4577581168850884e-06, 80.48158264160156, 3.814035177230835]
RC.InitPose = [450, 30, 370]
RC.Init_Pose()

Handle = {"AD": AD,
          'PWM': AD.ManualPWM,
          'RC': RC,
          'MoveRel': RC.Move_Rel,
          'MoveAbs': RC.Move_Abs,
          'InitPose': RC.Init_Pose,
          'GetPose': RC.Get_Pose,
          'GetJoint': RC.Get_Joint}

MC.AddHandle(Handle)
MC.Start()
