import threading
from Final_SiliconOil import Final_SiliconOil

PLS = Final_SiliconOil()

# PID Gain
# system init z_pose = 85mm(VisionFrame)
PLS.CT.Z_Reference = 92
PLS.CT.Kp = 1e-1/3
PLS.CT.Kd = 1e-2
PLS.CT.alpha = 0.52
PLS.CT.beta = 0.6
PLS.CT.theta = 0
PLS.CT.setPID()

# Robot Param
PLS.RC.Vel_Line = [15,30]
PLS.RC.Acc_Line = [10,30]
PLS.RC.InitPose = [425, 112.5, 370.5]


## << Start! >>
## --------------------------------------------------------------------
MainThread = threading.Thread(target=PLS.Start, daemon=True)
MainThread.start()


PLS.Handle()

PLS.RC.Vel_Line = [7,30]
PLS.RC.Acc_Line = [5,30]

PLS.RC.Wait(3)
PLS.RC.MoveRel(0,0,20,0)
PLS.RC.Wait(3)
PLS.RC.MoveRel(0,0,-20,0)
PLS.RC.Wait(3)
PLS.RC.MoveRel(0,0,15,0)
PLS.RC.Wait(3)

PLS.RC.Vel_Line = [15,30]
PLS.RC.Acc_Line = [10,30]

PLS.RC.MoveRel(60,0,0,0)
PLS.RC.MoveRel(-120,0,0,0)
PLS.RC.MoveRel(60,0,0,0)
PLS.RC.Wait(1)

PLS.RC.MoveRel(0,0,0,90)
PLS.RC.MoveRel(0,0,0,-180)
PLS.RC.Wait(1)

PLS.RC.MoveRel(60,0,0,0)
PLS.RC.MoveRel(-120,0,0,0)
PLS.RC.Wait(1)

PLS.RC.Vel_Line = [7,30]
PLS.RC.Acc_Line = [5,30]
PLS.RC.MoveAbs(425, 112.5, 360.5, 0)
PLS.RC.Wait(3)


PLS.Running = False
PLS.VS.Running = False
PLS.RC.Running = False
PLS.AD.Disconnect()
MainThread.join()

PLS.SaveResults()
PLS.AD.Disconnect()
PLS.VW.SaveVideo('251111_Basic_Silicon')

print(".")
print(".")
print(".")
print("")
print("THE END!")