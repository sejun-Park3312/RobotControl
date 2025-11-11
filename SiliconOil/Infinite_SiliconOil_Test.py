import threading
from Final_SiliconOil import Final_SiliconOil
import Trajectory_Infinite as Trj_IF

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


## << Start! >>
## --------------------------------------------------------------------
MainThread = threading.Thread(target=PLS.Start, daemon=True)
MainThread.start()


PLS.RC.Vel_Line = [10,30]
PLS.RC.Acc_Line = [7,30]
PLS.RC.InitPose = [425, 112.5, 370.5]

PLS.Handle()


b_list = Trj_IF.main()

PLS.RC.MoveInit()
PLS.RC.Wait(3)
PLS.RC.MoveRel(0,0,0,45)
PLS.RC.Wait(1)
PLS.RC.MoveBlend(b_list, 10, 7)
PLS.RC.Wait(1)
PLS.RC.MoveRel(0,0,0,-45)
PLS.RC.Wait(3)


PLS.Running = False
PLS.VS.Running = False
PLS.RC.Running = False
PLS.AD.Disconnect()
MainThread.join()

PLS.SaveResults()
PLS.AD.Disconnect()
PLS.VW.SaveVideo('251109_Infinite_Silicon')

print(".")
print(".")
print(".")
print("")
print("THE END!")