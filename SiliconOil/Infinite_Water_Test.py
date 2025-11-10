import threading
from Final_SiliconOil import Please
import Trajectory_Infinite as Trj_IF

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
PLS.RC.Vel_Line = [5,15]
PLS.RC.Acc_Line = [3,15]



## << Start! >>
## --------------------------------------------------------------------
MainThread = threading.Thread(target=PLS.Start, daemon=True)
MainThread.start()


PLS.RC.Vel_Line = [7,20]
PLS.RC.Acc_Line = [5,20]
PLS.RC.InitPose = [425, 112.5, 370.5]

PLS.Handle()


b_list = Trj_IF.main()

PLS.RC.MoveInit()
PLS.RC.Wait(3)
PLS.RC.MoveRel(0,0,0,45)
PLS.RC.Wait(1)
PLS.RC.MoveBlend(b_list, 5,5)
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
PLS.VW.SaveVideo('251109_Infinite_Water')

print(".")
print(".")
print(".")
print("")
print("THE END!")