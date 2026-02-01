from ProjectPath import PROJECT_PATH
from ClassFiles.Arduino import Arduino
from ClassFiles.ManualController import ManualController
from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK

MC = ManualController()
AD = Arduino()
alpha = 0
pwm = 30
AD.PWM_Value = 100


RC = RobotController_SDK()
RC.Vel_Line = [10,30]
RC.Acc_Line = [10,30]
RC.InitPose = [425, 112.5, 370.5]

Handle = {"AD": AD,
          'PWM': AD.ManualPWM,
          'RC': RC,
          'MoveRel': RC.MoveRel,
          'MoveAbs': RC.MoveAbs,
          'InitPose': RC.MoveInit,
          'GetPose': RC.GetPose,
          'GetJoint': RC.GetJoint}

MC.AddHandle(Handle)
MC.Start()
