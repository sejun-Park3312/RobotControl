import threading
from ProjectPath import PROJECT_PATH
from ClassFiles.TotalSystem import TotalSystem

TS = TotalSystem()

RobotTracking_Thread = threading.Thread(target=TS.RC.Track_EE, daemon=True)
RobotTracking_Thread.start()

VisionControl_Thread = threading.Thread(target=TS.VisionControl, daemon=True)
VisionControl_Thread.start()

Controller_Thread = threading.Thread(target=TS.TotalSystemController, daemon=True)
Controller_Thread.start()

TS.VS.GUI = False
TS.VS.Tracking()

TS.SaveResults()
print("<<Ctl+D for End Controller>>")
Controller_Thread.join()

TS.RC.EndController()
TS.VS.EndVision()
TS.AD.Disconnect()

print(".")
print(".")
print(".")
print("")
print("THE END!")
