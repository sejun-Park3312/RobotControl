import threading
from Please import Please
from ClassFiles.RobotControlFiles.PathCreator import PathCreator

PLS = Please()

# PID Gain
# system init z_pose = 85mm(VisionFrame)
PLS.CT.Z_Reference = 90
PLS.CT.PWM_Reference = 44
PLS.CT.a = 10.6
PLS.CT.alpha_p = 1
PLS.CT.alpha_n = 1.1
PLS.CT.beta = 1.4
PLS.CT.Kp = 15
PLS.CT.Kd = 4
PLS.CT.setPID()


# Robot Param
PLS.RC.Vel_Line = [5,5]
PLS.RC.Acc_Line = [3,3]

PC = PathCreator()
RelMotionList_1 = [[30,0,0],[0,-30,0],[-30,0,0]]
Radius = [10, 10]
Trajectory_1 = PC.MakeTrajectory(RelMotionList_1, Radius, False)

RelMotionList_2 = [[0,-60,0],[30,0,0],[0,60,0]]
Radius = [7.5, 7.5]
Trajectory_2 = PC.MakeTrajectory(RelMotionList_2, Radius, False)




## << Start! >>
## --------------------------------------------------------------------
MainThread = threading.Thread(target=PLS.Start, daemon=True)
MainThread.start()

PLS.Handle()

PLS.RC.Vel_Line = [7,20]
PLS.RC.Acc_Line = [5,20]
PLS.RC.Trajectory = [Trajectory_1, Trajectory_2]

PLS.RC.MoveRel(-65,-30,0,90)
PLS.RC.MoveRel(0,60,0,0)
PLS.RC.MoveRel(0,0,0, -90)
PLS.RC.MoveTrajectory(PLS.RC.Trajectory[0], 0, 0, 30)
PLS.RC.Wait(3)

PLS.RC.MoveRel(50,-30,0,90)
PLS.RC.MoveRel(0,60,0,0)
PLS.RC.MoveRel(0,0,0,26.57)
PLS.RC.MoveRel(30,-60,0,0)
PLS.RC.MoveRel(0,0,0,-26.57)
PLS.RC.MoveRel(0,60,0,0)
PLS.RC.Wait(3)

PLS.RC.MoveRel(20,0,0,0)
PLS.RC.MoveTrajectory(PLS.RC.Trajectory[1], 0,0, 30)
PLS.RC.Wait(3)

PLS.Handle()

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