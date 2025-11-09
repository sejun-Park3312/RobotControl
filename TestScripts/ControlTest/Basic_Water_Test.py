import threading
from Please import Please

PLS = Please()

# PID Gain
# system init z_pose = 85mm(VisionFrame)
PLS.CT.Z_Reference = 92
PLS.CT.PWM_Reference = 56
PLS.CT.a = 10.6
PLS.CT.alpha_p = 1
PLS.CT.alpha_n = 1.1
PLS.CT.beta = 1.4
PLS.CT.Kp = 15
PLS.CT.Kd = 4
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


PLS.RC.Wait(3)
PLS.RC.MoveRel(0,0,20,0)
PLS.RC.Wait(1)
PLS.RC.MoveRel(0,0,-20,0)
PLS.RC.Wait(1)
PLS.RC.MoveRel(0,0,15,0)
PLS.RC.Wait(2)

PLS.RC.Vel_Line = [10,30]
PLS.RC.Acc_Line = [6,30]

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

PLS.RC.Vel_Line = [5,20]
PLS.RC.Acc_Line = [3,20]
PLS.RC.MoveAbs(425, 112.5, 360.5, 0)
PLS.RC.Wait(3)


PLS.Running = False
PLS.VS.Running = False
PLS.RC.Running = False
PLS.AD.Disconnect()
MainThread.join()

PLS.SaveResults()
PLS.AD.Disconnect()
PLS.VW.SaveVideo('251109_Basic_Water')

print(".")
print(".")
print(".")
print("")
print("THE END!")