import threading
from Please import Please
from ClassFiles.RobotControlFiles.PathCreator import PathCreator

PLS = Please()

# PID Gain
# system init z_pose = 85mm(VisionFrame)
PLS.CT.Z_Reference = 90
PLS.CT.PWM_Reference = 46
PLS.CT.a = 10.6
PLS.CT.alpha_p = 1
PLS.CT.alpha_n = 1.1
PLS.CT.beta = 1.4
PLS.CT.Kp = 15
PLS.CT.Kd = 4
PLS.CT.setPID()


# Robot Param
PLS.RC.Velocity = [5,5]
PLS.RC.Acceleration = [3,3]

PC = PathCreator()
RelMotionList_1 = [[25,25,0],[0,17.7,0],[-25,0,0],[0,-12.5,0]]
Radius = [12.4, 12.4,12.4]
Trajectory_1 = PC.MakeTrajectory(RelMotionList_1, Radius, False)

RelMotionList_2 = [[0,12.5,0],[-25,0,0],[0,-17.7,0],[25,-25,0]]
Trajectory_2 = PC.MakeTrajectory(RelMotionList_2, Radius, False)


## << Start! >>
## --------------------------------------------------------------------
MainThread = threading.Thread(target=PLS.Start, daemon=True)
MainThread.start()

PLS.Handle()

PLS.RC.MoveRel(0,-21.34,0,0)
PLS.RC.MoveRel(0,0,0,45)
PLS.RC.MoveTrajectory(Trajectory_1)
PLS.RC.MoveRel(0,0,0,-180)
PLS.RC.MoveTrajectory(Trajectory_2)


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