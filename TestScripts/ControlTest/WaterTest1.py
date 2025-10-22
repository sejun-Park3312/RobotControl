import threading
from Please import Please

PLS = Please()

# PID Gain
PLS.CT.Z_Reference = 90
PLS.CT.PWM_Reference = 48
PLS.CT.a = 10.6
PLS.CT.alpha_p = 1
PLS.CT.alpha_n = 1.1
PLS.CT.beta = 1
PLS.CT.Kp = 20
PLS.CT.Kd = 4
PLS.CT.setPID()


# Robot Param
PLS.RC.Velocity = [5,5]
PLS.RC.Acceleration = [3,3]



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

print(".")
print(".")
print(".")
print("")
print("THE END!")