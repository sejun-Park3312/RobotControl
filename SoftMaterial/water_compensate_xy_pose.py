import threading
from Please import Please

PLS = Please()

# PID Gain
# system init z_pose = 85mm(VisionFrame)
PLS.CT.Z_Reference = 93
PLS.CT.PWM_Reference = 70
PLS.tilt = -0.15

PLS.CT.Kp = 30
PLS.CT.Kd = 5
PLS.CT.setPID()


# Robot Param
PLS.RC.Vel_Line = [12,30]
PLS.RC.Acc_Line = [6,30]
PLS.RC.InitPose = [425, 112.5, 370.5]


## << Start! >>
## --------------------------------------------------------------------
MainThread = threading.Thread(target=PLS.Start, daemon=True)
MainThread.start()

PLS.Handle()

PLS.Running = False
PLS.VS.Running = False
PLS.RC.Running = False
PLS.AD.Disconnect()
MainThread.join()

PLS.SaveResults()
PLS.AD.Disconnect()
PLS.VW.SaveVideo('260207_Test')

print(".")
print(".")
print(".")
print("")
print("THE END!")