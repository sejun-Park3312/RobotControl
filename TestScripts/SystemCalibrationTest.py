import threading
import code
from ProjectPath import PROJECT_PATH
from simple_pid import PID
from ClassFiles.Vision import Vision
from ClassFiles.Arduino import Arduino
from ClassFiles.Control import Control

VS = Vision()
VS.Tracking()

RC.launcher_model = "a0509_custom"
RC.Ready()
beta = 0
CT.alpha = 1
CT.theta = 0
RC.Velocity = [3, 3]
RC.Acceleration = [3,3]

# PID Gain
TS.CT.Kp = 1e-1/3
TS.CT.Kd = 1e-2
TS.CT.Ki = 0

