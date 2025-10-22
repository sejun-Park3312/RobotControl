import threading
from Please import Please

PLS = Please()

# PID Gain
PLS.CT.Z_Reference = 98
PLS.CT.PWM_Reference = 61
PLS.CT.a = 10
PLS.CT.alpha_p = 1
PLS.CT.alpha_n = 1.5
PLS.CT.beta = -1
PLS.CT.Kp = 1
PLS.CT.Kd = 0.5
PLS.CT.setPID()


# Robot Param
PLS.RC.Velocity = [10,10]
PLS.RC.Acceleration = [10,10]



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