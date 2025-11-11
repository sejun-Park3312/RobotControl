import threading
from Final_SiliconOil import Final_SiliconOil

PLS = Final_SiliconOil()

# PID Gain
# system init z_pose = 85mm(VisionFrame)
PLS.CT.Z_Reference = 95
PLS.CT.Kp = 1e-1/3
PLS.CT.Kd = 1e-2
PLS.CT.alpha = 0.52
PLS.CT.beta = 0
PLS.CT.theta = 0
PLS.CT.setPID()

# Robot Param
PLS.RC.Vel_Line = [15,30]
PLS.RC.Acc_Line = [10,30]


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
PLS.VW.SaveVideo('251111_Test')

print(".")
print(".")
print(".")
print("")
print("THE END!")