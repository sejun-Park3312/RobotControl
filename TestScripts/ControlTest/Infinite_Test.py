import threading
from Please import Please
from ClassFiles.RobotControlFiles.PathCreator import PathCreator

PLS = Please()

# PID Gain
# system init z_pose = 85mm(VisionFrame)
PLS.CT.Z_Reference = 92
PLS.CT.PWM_Reference = 57
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


PC = PathCreator()
FlatPlane = [[45,45,0],[45,-45,0],[-45,-45,0],  [-90,90,0], [-45,-45,0],[45,-45,0],[45,45,0]]
Radius = [63/2, 63/2, 63/2, 63/2, 63/2, 63/2]
Trajectory = PC.MakeTrajectory(FlatPlane, Radius, True, 20)




## << Start! >>
## --------------------------------------------------------------------
MainThread = threading.Thread(target=PLS.Start, daemon=True)
MainThread.start()


PLS.RC.Vel_Line = [7,20]
PLS.RC.Acc_Line = [5,20]
PLS.RC.InitPose = [425, 112.5, 367.5]
Center = [425,112.5,374]

PLS.Handle()



PLS.RC.MoveAbs(Center[0],Center[1],Center[2],0)
PLS.RC.Vel_Line = [12,20]
PLS.RC.Acc_Line = [10,20]

PLS.RC.Trajectory = [Trajectory]

PLS.RC.MoveRel(0,0,0,45)
PLS.RC.Wait(5)
PLS.RC.MoveTrajectory(PLS.RC.Trajectory[0], 25, 25, 0)
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

print(".")
print(".")
print(".")
print("")
print("THE END!")