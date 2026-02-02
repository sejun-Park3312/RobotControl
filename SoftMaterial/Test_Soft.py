import threading
from Please import Please

PLS = Please()

# PID Gain
# system init z_pose = 85mm(VisionFrame)
PLS.CT.Z_Reference = 90
PLS.CT.PWM_Reference = 100
# PLS.CT.a = 10.6
# PLS.CT.alpha_p = 1
# PLS.CT.alpha_n = 1.1
# PLS.CT.beta = 1.4
PLS.CT.Kp = 30
PLS.CT.Kd = 5
PLS.CT.setPID()


# Robot Param
PLS.RC.Vel_Line = [5,30]
PLS.RC.Acc_Line = [3,30]
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
PLS.VW.SaveVideo('260201_Test')

print(".")
print(".")
print(".")
print("")
print("THE END!")